#include <algorithm>
#include <cmath>
#include <string>

#include "csg/csg.h"
#include "csg/bsp_output.h"
#include "support/map_builder.h"
#include "support/scratch.h"
#include "support/test.h"
#include "support/test_wad.h"

namespace
{
    csg::map_source pair_map(const math::vec3v &bmin, const math::vec3v &bmax)
    {
        auto map = test_support::map_builder()
            .brush_entity("func_wall", {-128, -48, -32}, {-64, 48, 80}, "STONE")
            .brush_entity("func_wall", bmin, bmax, "STONE").build();
        map.entities.resize(1);
        map.entities[0].first_brush = 0;
        map.entities[0].num_brushes = 2;
        for (size_t i = 0; i < map.brushes.size(); ++i)
        {
            map.brushes[i].entity_num = 0;
            map.brushes[i].original_entity_num = 0;
            map.brushes[i].original_brush_num = (int)i;
        }
        return map;
    }

    csg::map_source seam_map()
    {
        auto map = pair_map({-128, -80, -32}, {-64, -48, 80});
        for (auto &point : map.sides[5].planepts) point.z = (144 - 7 * point.y) / 6;
        return map;
    }

    csg::csg_result build(csg::map_source map, bool merge, csg::clip_type mode = csg::clip_type::legacy)
    {
        const auto scratch = test_support::scratch_directory("csg_clip_merge");
        const auto wad = (scratch / "textures.wad").string();
        test_wad::write(wad, {"STONE"});
        map.entities[0].set_value("wad", wad.c_str());
        csg::csg_options options;
        options.brush.clip = mode;
        options.brush.merge_legacy_clip = merge;
        return csg::run_csg(std::move(map), options);
    }

    size_t count(const csg::csg_result &result, int hull = 1)
    {
        size_t n = 0;
        for (const auto &entity : result.entities)
            for (const auto &brush : entity.brushes)
                n += !brush.hulls[hull].faces.empty();
        return n;
    }

    double top(const csg::csg_result &result, int hull, double y)
    {
        const math::vec3v origin{-96, (vec_t)y, 160}, direction{0, 0, -1};
        double first = 1e30;
        for (const auto &brush : result.entities[0].brushes)
        {
            if (brush.hulls[hull].faces.empty()) continue;
            double lo = 0, hi = 240;
            for (const auto &face : brush.hulls[hull].faces)
            {
                const auto &p = result.planes[(size_t)face.planenum];
                const double offset = math::dot(p.normal, origin) - p.dist;
                const double speed = math::dot(p.normal, direction);
                if (speed < -1e-12) lo = std::max(lo, -offset / speed);
                else if (speed > 1e-12) hi = std::min(hi, -offset / speed);
                else if (offset > 1e-8) hi = -1;
            }
            if (hi > lo) first = std::min(first, lo);
        }
        return 160 - first;
    }
}

suite("unit.csg.clip_merge")
{
    test("merges the platform seam before legacy expansion and preserves render intermediates")
    {
        auto plain = build(seam_map(), false);
        auto joined = build(seam_map(), true);
        require(count(plain) == 2);
        require(count(joined) == 1);
        expect(count(joined, 0) == 2);
        csg::brush_build_options options;
        auto a = csg::build_intermediate_data(plain, options);
        auto b = csg::build_intermediate_data(joined, options);
        expect(a.surfaces[0] == b.surfaces[0]);
        expect(a.brushes[0] == b.brushes[0]);
        expect(std::fabs(top(plain, 1, -32) - 116) < .001);
        expect(std::fabs(top(joined, 1, -32) - 98.934616) < .001);
        for (int h = 1; h < csg::num_hulls; ++h)
        {
            expect(count(joined, h) == 1);
            expect(std::fabs(top(plain, h, 0) - top(joined, h, 0)) < .001);
        }
    }

    test("rejects a nonconvex join, separated brushes, and mere edge contact")
    {
        for (const auto &bounds : {
            std::pair<math::vec3v, math::vec3v>{{-128, -80, -32}, {-96, -48, 80}},
            {{-128, -81, -32}, {-64, -49, 80}},
            {{-64, -80, -32}, {0, -48, 80}}})
            expect(count(build(pair_map(bounds.first, bounds.second), true)) == 2);
    }

    test("leaves simple collision unchanged")
    {
        auto a = build(seam_map(), false, csg::clip_type::simple);
        auto b = build(seam_map(), true, csg::clip_type::simple);
        csg::brush_build_options options;
        auto ia = csg::build_intermediate_data(a, options);
        auto ib = csg::build_intermediate_data(b, options);
        for (int h = 0; h < csg::num_hulls; ++h)
        {
            expect(ia.surfaces[h] == ib.surfaces[h]);
            expect(ia.brushes[h] == ib.brushes[h]);
        }
        expect(ia.planes == ib.planes);
    }

    test("respects noclip, bevels, detail policy and per-brush clip types")
    {
        for (int kind = 0; kind < 7; ++kind)
        {
            auto map = seam_map();
            auto &b = map.brushes[1];
            if (kind == 0) b.noclip = 1;
            if (kind == 1) map.sides[(size_t)b.first_side].bevel = true;
            if (kind == 2) b.detail_level = 1;
            if (kind == 3) b.clipnode_detail_level = 1;
            if (kind == 4) b.local_clip = (int)csg::clip_type::simple;
            if (kind == 5) b.coplanar_priority = 1;
            if (kind == 6) b.original_entity_num = 1;
            auto a = build(map, false), merged = build(map, true);
            csg::brush_build_options options;
            auto ia = csg::build_intermediate_data(a, options);
            auto ib = csg::build_intermediate_data(merged, options);
            expect(ia.planes == ib.planes);
            for (int h = 0; h < csg::num_hulls; ++h)
                expect(ia.surfaces[h] == ib.surfaces[h]);
        }
    }

    test("keeps brush model ownership separate")
    {
        auto map = test_support::map_builder()
            .brush_entity("func_wall", {-128, -48, -32}, {-64, 48, 80}, "STONE")
            .brush_entity("func_wall", {-128, -80, -32}, {-64, -48, 80}, "STONE").build();
        expect(count(build(map, true)) == 2);
    }

    test("preserves custom default hulls")
    {
        auto map = test_support::map_builder()
            .brush_entity("info_hullshape", {-8, -8, -8}, {8, 8, 8}, "STONE",
                          {{"disabled", "0"}, {"defaulthulls", "2"}})
            .brush_entity("func_wall", {-128, -48, -32}, {-64, 48, 80}, "STONE")
            .brush_entity("func_wall", {-128, -80, -32}, {-64, -48, 80}, "STONE").build();
        map.entities.resize(2);
        map.entities[0].first_brush = 1;
        map.entities[0].num_brushes = 2;
        for (int i = 1; i < 3; ++i)
        {
            map.brushes[(size_t)i].entity_num = 0;
            map.brushes[(size_t)i].original_entity_num = 0;
            map.brushes[(size_t)i].original_brush_num = i - 1;
        }
        auto a = build(map, false), b = build(map, true);
        csg::brush_build_options options;
        auto ia = csg::build_intermediate_data(a, options);
        auto ib = csg::build_intermediate_data(b, options);
        expect(ia.planes == ib.planes);
        for (int h = 0; h < csg::num_hulls; ++h)
            expect(ia.surfaces[h] == ib.surfaces[h]);
    }

    test("handles a seam split on an oblique plane in either brush order")
    {
        for (bool reverse : {false, true})
        {
            auto map = pair_map({-128, -80, -32}, {-64, -48, 80});
            for (auto &side : map.sides)
                for (auto &p : side.planepts) p.y += p.z / 4;
            if (reverse) std::swap(map.brushes[0], map.brushes[1]);
            expect(count(build(map, true)) == 1);
        }
    }

    test("revisits merged groups to join a chain of three brushes")
    {
        auto map = pair_map({-128, -80, -32}, {-64, -48, 80});
        auto third = map.brushes[1];
        third.first_side = (int)map.sides.size();
        third.original_brush_num = 2;
        for (int i = 6; i < 12; ++i)
        {
            auto side = map.sides[(size_t)i];
            for (auto &point : side.planepts) point.y -= 32;
            map.sides.push_back(side);
        }
        map.brushes.push_back(third);
        map.entities[0].num_brushes = 3;
        auto merged = build(map, true);
        expect(count(merged) == 1);
        expect(count(merged, 0) == 3);
    }
}
