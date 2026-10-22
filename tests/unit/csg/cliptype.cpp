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
    test("legacytrim preserves legacy input geometry and transfers exact guards through both input paths")
    {
        const auto scratch = test_support::scratch_directory("csg_legacytrim_bridge");
        const std::string wad_path = (scratch / "textures.wad").string();
        test_wad::write(wad_path, {"STONE"});
        csg::csg_options options;
        options.brush.clip = csg::clip_type::legacy;
        options.wad.map_path = wad_path;
        options.wad.wad_textures = true;
        auto run = [&](const char *mode) {
            auto map = make_map(wad_path,"func_detail",mode);
            // Sloped prism, rather than a box with coincident exact/legacy hulls.
            for (auto &v : map.sides[5].planepts) v.z = 32 + v.x / 2;
            return csg::run_csg(std::move(map),options);
        };
        auto legacy = run("legacy"), trim = run("legacytrim");
        auto old_data = csg::build_intermediate_data(legacy,options.brush);
        auto data = csg::build_intermediate_data(trim,options.brush);
        expect(old_data.planes == data.planes);
        for (int h = 0; h < csg::num_hulls; h++)
        {
            expect(old_data.surfaces[h] == data.surfaces[h]);
            expect(old_data.brushes[h] == data.brushes[h]);
        }
        const auto base = (scratch / "trim").string();
        require(csg::write_intermediate_files(base,data,trim));
        bsp::bsp_state disk, memory;
        disk.base_path = base;
        bsp::load_hull_sizes(disk);
        auto map = csg::build_bsp_data(trim,false);
        memory.map = &map;
        bsp::bsp_input input;
        for (int h = 0; h < csg::num_hulls; h++)
        {
            input.surfaces[h] = std::move(data.surfaces[h]);
            input.brushes[h] = std::move(data.brushes[h]);
        }
        input.hull_sizes = std::move(data.hull_sizes);
        input.planes = std::move(data.planes);
        auto stage_input = input;
        bsp::set_input_data(memory,input);
        require(memory.trim_groups.size() == 3);
        require(disk.trim_groups.size() == 3);
        for (int h = 0; h < 3; h++)
        {
            const auto &a = memory.trim_groups[(size_t)h], &b = disk.trim_groups[(size_t)h];
            expect(a.model == 0); expect(a.hull == h + 1);
            require(a.targets.size() == 1); require(a.guards.size() == 1);
            expect(a.guards[0].bounds.mins == b.guards[0].bounds.mins);
            require(a.guards[0].planes.size() == b.guards[0].planes.size());
            for (size_t i = 0; i < a.guards[0].planes.size(); i++)
            {
                expect(a.guards[0].planes[i].normal == b.guards[0].planes[i].normal);
                expect(a.guards[0].planes[i].dist == b.guards[0].planes[i].dist);
            }
        }
        auto disk_map = map, memory_map = map;
        bsp::bsp_options bsp_options;
        bsp_options.nofill = true;
        bsp_options.noinsidefill = true;
        bsp::run_bsp(disk_map,base,bsp_options);
        bsp::run_bsp(memory_map,(scratch / "memory").string(),bsp_options,std::move(stage_input));
        require(disk_map.clipnodes.size() == memory_map.clipnodes.size());
        if (!disk_map.clipnodes.empty())
            expect(std::memcmp(disk_map.clipnodes.data(),memory_map.clipnodes.data(),
                disk_map.clipnodes.size() * sizeof(format::dclipnode_t)) == 0);
        require(disk_map.planes.size() == memory_map.planes.size());
        if (!disk_map.planes.empty())
            expect(std::memcmp(disk_map.planes.data(),memory_map.planes.data(),
                disk_map.planes.size() * sizeof(format::dplane_t)) == 0);
    }

    test("legacytrim honors clip hull masks and protects unselected and custom hull brushes")
    {
        auto map = make_map("","func_detail","legacytrim");
        auto brush = map.brushes[0];
        brush.contents = csg::content::solid;
        brush.cliphull = 1 << 3;
        csg::brush_build_options options;
        options.clip = csg::clip_type::legacytrim;
        csg::plane_store planes;
        auto built = csg::create_brush(map,brush,planes,{},nullptr,options);
        expect(built.hulls[0].faces.empty());
        expect(built.trim_planes[1].empty());
        expect(built.trim_planes[2].empty());
        expect(!built.trim_planes[3].empty());
        brush.cliphull = 0;
        brush.noclip = true;
        built = csg::create_brush(map,brush,planes,{},nullptr,options);
        expect(built.trim_planes[3].empty());
        brush.noclip = false;
        // An active custom hull uses its own Minkowski expansion, even if the
        // global clip option is legacytrim. No box guard is generated for it.
        csg::hull_shape_library library;
        library.default_hulls[3].disabled = false;
        // Empty custom shape intentionally disables collision in that hull.
        options.hull_shapes = &library;
        built = csg::create_brush(map,brush,planes,{},nullptr,options);
        expect(built.trim_planes[3].empty());
        expect(built.hulls[3].faces.empty());
        map.brushes[0].contents = csg::content::solid;
        csg::add_hull_shape(map,1,library);
        library.default_hulls[3] = library.shapes.back();
        built = csg::create_brush(map,brush,planes,{},nullptr,options);
        expect(built.trim_planes[3].empty());
        expect(!built.hulls[3].faces.empty());
        options.hull_shapes = nullptr;
        options.clip = csg::clip_type::legacy;
        built = csg::create_brush(map,brush,planes,{},nullptr,options);
        expect(built.trim_planes[1].empty());
    }

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
        expect(run_and_read_local_clip(wad_path, "func_detail", "legacytrim")
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
