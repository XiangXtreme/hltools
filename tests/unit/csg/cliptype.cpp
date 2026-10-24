#include <string>
#include <cstring>
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
        require(result.map.entities[1].num_brushes == 1);
        expect(result.map.brushes[(size_t)result.map.entities[1].first_brush].local_clip
               == (int)csg::clip_type::legacytrim);
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
}
