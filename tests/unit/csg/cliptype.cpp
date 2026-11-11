#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "common/string_util.h"
#include "csg/brush.h"
#include "csg/csg.h"
#include "csg/map_parser.h"
#include "csg/bsp_output.h"
#include "bsp/internal.h"
#include "support/scratch.h"
#include "support/test.h"
#include "support/test_wad.h"

namespace
{
    void add_side(csg::map_source &map, const math::vec3v &p0, const math::vec3v &p1,
                  const math::vec3v &p2)
    {
        csg::brush_side side;
        str::copy(side.texture.name, sizeof(side.texture.name), "STONE");
        side.planepts[0] = p0;
        side.planepts[1] = p1;
        side.planepts[2] = p2;
        map.sides.push_back(side);
    }

    // a 64 unit cube whose brushes live on one func_detail or func_group entity
    csg::map_source make_map(const std::string &wad_path, const char *classname,
                             const char *cliptype)
    {
        csg::map_source map;

        csg::map_entity world;
        world.set_value("classname", "worldspawn");
        world.set_value("wad", wad_path.c_str());
        map.entities.push_back(world);

        csg::map_entity detail;
        detail.set_value("classname", classname);
        detail.first_brush = 0;
        detail.num_brushes = 1;
        if (cliptype)
            detail.set_value("zhlt_cliptype", cliptype);
        map.entities.push_back(detail);

        const math::vec3v mins{0, 0, 0};
        const math::vec3v maxs{64, 64, 64};
        add_side(map, {mins.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z},
                 {mins.x, mins.y, maxs.z});
        add_side(map, {maxs.x, mins.y, mins.z}, {maxs.x, mins.y, maxs.z},
                 {maxs.x, maxs.y, mins.z});
        add_side(map, {mins.x, mins.y, mins.z}, {mins.x, mins.y, maxs.z},
                 {maxs.x, mins.y, mins.z});
        add_side(map, {mins.x, maxs.y, mins.z}, {maxs.x, maxs.y, mins.z},
                 {mins.x, maxs.y, maxs.z});
        add_side(map, {mins.x, mins.y, mins.z}, {maxs.x, mins.y, mins.z},
                 {mins.x, maxs.y, mins.z});
        add_side(map, {mins.x, mins.y, maxs.z}, {mins.x, maxs.y, maxs.z},
                 {maxs.x, mins.y, maxs.z});

        csg::map_brush brush;
        brush.first_side = 0;
        brush.num_sides = 6;
        map.brushes.push_back(brush);
        return map;
    }

    int run_and_read_local_clip(const std::string &wad_path, const char *classname,
                                const char *cliptype)
    {
        csg::csg_options options;
        options.brush.clip = csg::clip_type::legacy;
        options.wad.map_path = wad_path;
        options.wad.wad_textures = true;

        const csg::csg_result result =
            csg::run_csg(make_map(wad_path, classname, cliptype), options);
        require(result.map.entities.size() == 1);
        require(result.map.brushes.size() == 1);
        return result.map.brushes[0].local_clip;
    }

    int surface_contents(const direct_clip::tree &tree, const math::vec3v &point)
    {
        int index = tree.head;
        while (index >= 0)
        {
            const auto &node = tree.nodes.at((size_t)index);
            const auto &plane = tree.planes.at((size_t)node.plane);
            index = math::dot(point, plane.normal) >= plane.dist ? node.front : node.back;
        }
        return index;
    }

    csg::intermediate_data roof_hulls(const std::string &wad_path, int segments, double rotation = 0)
    {
        auto map = make_map(wad_path, "func_detail", "legacytrim");
        const auto cube = map.sides;
        map.sides.clear();
        map.brushes.clear();
        for (int segment = 0; segment < segments; segment++)
        {
            csg::map_brush brush;
            brush.first_side = (int)map.sides.size();
            brush.num_sides = 6;
            for (auto side : cube)
            {
                for (auto &point : side.planepts)
                {
                    if (segments == 1)
                    {
                        // Thin roof terminated by horizontal Z caps, not Y
                        // walls: shifted slope intersections used to run away.
                        const auto p = point;
                        point = {p.x * 2 - 64, p.y / 8 + (64 - p.z) * 8, p.z * 3};
                    }
                    else
                    {
                        const double angle = -(segment + point.x / 64) * 2 * 3.141592653589793 / segments;
                        const double radius = 1024 - point.z * 8 + point.y / 8;
                        point = {(vec_t)(radius * std::cos(angle)), (vec_t)(radius * std::sin(angle)), point.z * 3};
                    }
                    const auto p = point;
                    point.x = p.x * std::cos(rotation) - p.y * std::sin(rotation);
                    point.y = p.x * std::sin(rotation) + p.y * std::cos(rotation);
                }
                map.sides.push_back(side);
            }
            map.brushes.push_back(brush);
        }
        map.entities[1].num_brushes = segments;
        csg::csg_options options;
        options.wad.map_path = wad_path;
        options.wad.wad_textures = true;
        return csg::build_intermediate_data(csg::run_csg(std::move(map), options), options.brush);
    }
}

suite("unit.csg.cliptype")
{
    test("cliptype.parses every reference value and rejects anything else")
    {
        csg::clip_type type = csg::clip_type::inherit;
        expect(csg::parse_clip_type("smallest", type));
        expect(type == csg::clip_type::smallest);
        expect(csg::parse_clip_type("Normalized", type));
        expect(type == csg::clip_type::normalized);
        expect(csg::parse_clip_type("simple", type));
        expect(type == csg::clip_type::simple);
        expect(csg::parse_clip_type("precise", type));
        expect(type == csg::clip_type::precise);
        expect(csg::parse_clip_type("legacy", type));
        expect(type == csg::clip_type::legacy);
        expect(csg::parse_clip_type("legacytrim", type));
        expect(type == csg::clip_type::legacytrim);
        expect(std::string(csg::clip_type_name(type)) == "legacytrim");
        expect(!csg::parse_clip_type("exactunion", type));
        expect(!csg::parse_clip_type("pyexact", type));
        expect(!csg::parse_clip_type("pyboundary", type));
        expect(!csg::parse_clip_type("inherit", type));
        expect(!csg::parse_clip_type("nope", type));
    }

    test("cliptype.a brush override replaces only the clip setting")
    {
        csg::brush_build_options options;
        options.clip = csg::clip_type::simple;
        options.world_extent = 12345;

        csg::map_brush brush;
        const csg::brush_build_options inherited =
            csg::clip_options_for_brush(brush, options);
        expect(inherited.clip == csg::clip_type::simple);
        expect(inherited.world_extent == 12345);

        brush.local_clip = (int)csg::clip_type::precise;
        const csg::brush_build_options overridden =
            csg::clip_options_for_brush(brush, options);
        expect(overridden.clip == csg::clip_type::precise);
        expect(overridden.world_extent == 12345);
    }

    test("cliptype.a func_detail zhlt_cliptype reaches the merged brushes")
    {
        const std::filesystem::path scratch =
            test_support::scratch_directory("csg_cliptype_detail");
        const std::string wad_path = (scratch / "textures.wad").string();
        test_wad::write(wad_path, {"STONE"});

        expect(run_and_read_local_clip(wad_path, "func_detail", "precise")
               == (int)csg::clip_type::precise);
    }

    test("cliptype.legacytrim keeps a detail model separate from worldspawn")
    {
        const std::filesystem::path scratch =
            test_support::scratch_directory("csg_cliptype_surface_detail");
        const std::string wad_path = (scratch / "textures.wad").string();
        test_wad::write(wad_path, {"STONE"});

        csg::csg_options options;
        options.brush.clip = csg::clip_type::simple;
        options.wad.map_path = wad_path;
        options.wad.wad_textures = true;
        csg::csg_result result =
            csg::run_csg(make_map(wad_path, "func_detail", "legacytrim"), options);

        require(result.map.entities.size() == 2);
        expect(std::string(result.map.entities[1].value("classname")) == "func_wall");
        expect(std::string(result.map.entities[1].value("zhlt_collision_only")) == "1");
        require(result.map.entities[1].num_brushes == 1);
        expect(result.map.brushes[(size_t)result.map.entities[1].first_brush].local_clip
               == (int)csg::clip_type::legacytrim);
        require(result.entities.size() == 2);
        require(result.entities[1].brushes.size() == 1);
        expect(!result.entities[1].brushes[0].hulls[0].faces.empty());
        expect(result.entities[1].brushes[0].hulls[1].faces.empty());

        csg::intermediate_data data = csg::build_intermediate_data(result, options.brush);
        require(data.surface_clips.size() == 2);
        expect(!data.surface_clips[0].valid);
        expect(data.surface_clips[1].valid);
        expect(data.surface_clips[1].hulls[1].valid);
        expect(data.surface_clips[1].hulls[3].valid);
        expect(!data.surface_clips[1].hulls[1].nodes.empty());
        expect(data.surfaces[1] == "-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n");
        expect(data.surfaces[2] == "-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n");
        expect(data.surfaces[3] == "-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n");
    }

    test("cliptype.func_group and unknown values inherit the global setting")
    {
        const std::filesystem::path scratch =
            test_support::scratch_directory("csg_cliptype_inherit");
        const std::string wad_path = (scratch / "textures.wad").string();
        test_wad::write(wad_path, {"STONE"});

        expect(run_and_read_local_clip(wad_path, "func_group", "precise") == -1);
        expect(run_and_read_local_clip(wad_path, "func_detail", nullptr) == -1);
        expect(run_and_read_local_clip(wad_path, "func_detail", "bogus") == -1);
    }

    test("cliptype.legacytrim bounds thin roof ends for standing and ducked hulls")
    {
        const auto scratch = test_support::scratch_directory("csg_cliptype_thin_roof");
        const auto wad = (scratch / "textures.wad").string();
        test_wad::write(wad, {"STONE"});
        const auto data = roof_hulls(wad, 1);
        require(data.surface_clips.size() == 2);
        for (int hull : {1, 3})
        {
            const auto &tree = data.surface_clips[1].hulls[hull];
            require(tree.valid);
            expect(surface_contents(tree, {0, 600, -16}) == -1);
            expect(surface_contents(tree, {0, -80, 200}) == -1);
            expect(surface_contents(tree, {0, 260, 96}) == -2);
            expect(surface_contents(tree, {0, 535, 0}) == -2);
            expect(surface_contents(tree, {0, 537, 0}) == -1);
        }
    }

    test("cliptype.legacytrim preserves ring holes and contacts across radial seams")
    {
        const auto scratch = test_support::scratch_directory("csg_cliptype_roof_ring");
        const auto wad = (scratch / "textures.wad").string();
        test_wad::write(wad, {"STONE"});
        const auto data = roof_hulls(wad, 24);
        require(data.surface_clips.size() == 2);
        for (int hull : {1, 3})
        {
            const auto &tree = data.surface_clips[1].hulls[hull];
            require(tree.valid);
            expect(surface_contents(tree, {0, 0, 96}) == -1);
            for (int i = 0; i < 48; i++)
            {
                const double angle = i * 2 * 3.141592653589793 / 48;
                for (double offset : {-1e-5, 0., 1e-5})
                {
                    const double a = angle + offset;
                    const math::vec3v on_roof{(vec_t)(768 * std::cos(a)), (vec_t)(768 * std::sin(a)), 96};
                    expect(surface_contents(tree, on_roof) == -2);
                    expect(surface_contents(tree, {(vec_t)(1100 * std::cos(a)), (vec_t)(1100 * std::sin(a)), -16}) ==
                           -1);
                    expect(surface_contents(tree, {(vec_t)(400 * std::cos(a)), (vec_t)(400 * std::sin(a)), 200}) == -1);
                }
            }
        }
    }

    test("cliptype.legacytrim bounds diagonal roof edges inside the axial bounding box")
    {
        const auto scratch = test_support::scratch_directory("csg_cliptype_diagonal_roof");
        const auto wad = (scratch / "textures.wad").string();
        test_wad::write(wad, {"STONE"});
        const auto data = roof_hulls(wad, 1, 3.141592653589793 / 4);
        require(data.surface_clips.size() == 2);
        const double diagonal = std::sqrt(0.5);
        for (int hull : {1, 3})
        {
            const auto &tree = data.surface_clips[1].hulls[hull];
            require(tree.valid);
            // Both points are inside the expanded axial bounds. Only the
            // edge/box-axis bevel limits the roof end to 520 + 16 * sqrt(2).
            expect(surface_contents(tree, {-541 * diagonal, 541 * diagonal, 0}) == -2);
            expect(surface_contents(tree, {-544 * diagonal, 544 * diagonal, 0}) == -1);
        }
    }
}
