#include <algorithm>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "common/filesystem.h"
#include "common/string_util.h"
#include "csg/studio_bsp.h"
#include "format/mdl/goldsrc/model.h"
#include "support/scratch.h"
#include "csg/bsp_output.h"
#include "csg/brush.h"
#include "csg/csg.h"
#include "csg/surface_clip.h"
#include "support/test.h"
#include "support/test_wad.h"

namespace
{
    format::studio_model make_quad_model()
    {
        format::studio_model model;
        model.name = "models/props/bake.mdl";

        format::studio_bone bone;
        bone.name = "root";
        model.bones.push_back(bone);

        const float corners[4][2] = {{-8, -8}, {8, -8}, {8, 8}, {-8, 8}};
        const float uv[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        for (int i = 0; i < 4; i++)
        {
            format::studio_vertex vertex;
            vertex.position[0] = corners[i][0];
            vertex.position[1] = corners[i][1];
            vertex.normal[2] = 1;
            vertex.u = uv[i][0];
            vertex.v = uv[i][1];
            model.vertices.push_back(vertex);
        }

        format::studio_mesh mesh;
        mesh.material = 0;
        mesh.indices = {0, 1, 2, 0, 2, 3};
        model.meshes.push_back(mesh);

        format::studio_texture texture;
        texture.name = "bake_skin.bmp";
        texture.image.width = 16;
        texture.image.height = 16;
        texture.image.pixels.assign(256, 0);
        model.textures.push_back(texture);
        return model;
    }

    std::string write_model(const std::filesystem::path &root)
    {
        std::filesystem::create_directories(root / "models" / "props");
        std::filesystem::create_directories(root / "maps");
        std::vector<byte> bytes;
        require(format::write_goldsrc_model(make_quad_model(), bytes));
        const std::string path = (root / "models" / "props" / "bake.mdl").string();
        require(fs::write_all(path, bytes.data(), bytes.size()));
        return path;
    }

    format::studio_model make_sliver_model()
    {
        format::studio_model model;
        model.name = "models/props/sliver.mdl";

        format::studio_bone bone;
        bone.name = "root";
        model.bones.push_back(bone);

        const float corners[3][2] = {{0, 0}, {16, 0}, {8, 0.1f}};
        const float uv[3][2] = {{0, 0}, {1, 0}, {0.5f, 1}};
        for (int i = 0; i < 3; i++)
        {
            format::studio_vertex vertex;
            vertex.position[0] = corners[i][0];
            vertex.position[1] = corners[i][1];
            vertex.normal[2] = 1;
            vertex.u = uv[i][0];
            vertex.v = uv[i][1];
            model.vertices.push_back(vertex);
        }

        format::studio_mesh mesh;
        mesh.material = 0;
        mesh.indices = {0, 1, 2};
        model.meshes.push_back(mesh);

        format::studio_texture texture;
        texture.name = "sliver_skin.bmp";
        texture.image.width = 16;
        texture.image.height = 16;
        texture.image.pixels.assign(256, 0);
        model.textures.push_back(texture);
        return model;
    }

    std::string write_sliver_model(const std::filesystem::path &root)
    {
        std::filesystem::create_directories(root / "models" / "props");
        std::filesystem::create_directories(root / "maps");
        std::vector<byte> bytes;
        require(format::write_goldsrc_model(make_sliver_model(), bytes));
        const std::string path = (root / "models" / "props" / "sliver.mdl").string();
        require(fs::write_all(path, bytes.data(), bytes.size()));
        return path;
    }

        void add_world_cube(csg::map_source &map, const std::string &wad_path)
    {
        csg::map_entity world;
        world.set_value("classname", "worldspawn");
        world.set_value("wad", wad_path.c_str());
        world.first_brush = 0;
        world.num_brushes = 1;

        const math::vec3v mins{-128, -128, -64};
        const math::vec3v maxs{128, 128, 0};
        auto add_s = [&](const math::vec3v &p0, const math::vec3v &p1, const math::vec3v &p2) {
            csg::brush_side side;
            std::memcpy(side.texture.name, "STONE", 6);
            side.planepts[0] = p0;
            side.planepts[1] = p1;
            side.planepts[2] = p2;
            map.sides.push_back(side);
        };
        add_s({mins.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z}, {mins.x, mins.y, maxs.z});
        add_s({maxs.x, mins.y, mins.z}, {maxs.x, mins.y, maxs.z}, {maxs.x, maxs.y, mins.z});
        add_s({mins.x, mins.y, mins.z}, {mins.x, mins.y, maxs.z}, {maxs.x, mins.y, mins.z});
        add_s({mins.x, maxs.y, mins.z}, {maxs.x, maxs.y, mins.z}, {mins.x, maxs.y, maxs.z});
        add_s({mins.x, mins.y, mins.z}, {maxs.x, mins.y, mins.z}, {mins.x, maxs.y, mins.z});
        add_s({mins.x, mins.y, maxs.z}, {mins.x, maxs.y, maxs.z}, {maxs.x, maxs.y, maxs.z});

        csg::map_brush brush;
        brush.first_side = 0;
        brush.num_sides = 6;
        map.brushes.push_back(brush);
        map.entities.push_back(world);
    }

csg::map_entity make_world()
    {
        csg::map_entity world;
        world.set_value("classname", "worldspawn");
        return world;
    }

    csg::map_entity make_studio_entity(const char *extra_key = nullptr)
    {
        csg::map_entity entity;
        entity.set_value("classname", "env_static");
        entity.set_value("model", "models/props/bake.mdl");
        entity.set_value("origin", "0 0 0");
        entity.set_value("zhlt_studiobsp", "1");
        entity.set_value("zhlt_studiobsp_collision", "1");
        if (extra_key)
            entity.set_value(extra_key, "1");
        return entity;
    }
}

suite("unit.csg.studio_bsp")
{
    test("studio_bsp.bakes one five sided prism per triangle")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_bsp");
        write_model(root);

        csg::map_source map;
        map.entities.push_back(make_world());
        map.entities.push_back(make_studio_entity());

        const std::vector<format::mip_texture> textures =
            csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        require(textures.size() == 1);
        expect(textures[0].name.size() <= 15);
        expect(!textures[0].data.empty());

        require(map.entities.size() == 2);
        const csg::map_entity &entity = map.entities[1];
        expect(std::string(entity.value("classname")) == "func_wall");
        expect(std::string(entity.value("model")).empty());
        expect(std::string(entity.value("origin")).empty());
        expect(entity.int_value("zhlt_studiobsp") == 0);
        expect((entity.int_value("zhlt_lightflags") & 2) != 0);

        require(entity.num_brushes == 2);
        require(map.brushes.size() == 2);
        require(map.sides.size() == 10);
        for (int i = 0; i < 2; i++)
        {
            const csg::map_brush &brush = map.brushes[(std::size_t)i];
            expect(brush.num_sides == 5);
            expect(brush.first_side == i * 5);
            expect(brush.detail_level == 1);
            expect(brush.clipnode_detail_level == 1);
            expect(!brush.noclip);
            expect(brush.contents == csg::content::solid);

            const csg::brush_side &front = map.sides[(std::size_t)brush.first_side];
            expect(front.texture.txcommand == 'Q');
            expect(std::string(front.texture.name) == textures[0].name);
            for (int side = 1; side < 5; side++)
            {
                const csg::brush_side &hidden =
                    map.sides[(std::size_t)brush.first_side + side];
                expect(hidden.texture.txcommand == 'Q');
                expect(std::string(hidden.texture.name) == textures[0].name);
            }
        }
    }

    test("studio_bsp.world mode moves the prisms into worldspawn")
    {
        const std::filesystem::path root =
            test_support::scratch_directory("csg_studio_bsp_world");
        write_model(root);

        csg::map_source map;
        map.entities.push_back(make_world());
        map.entities.push_back(make_studio_entity("zhlt_studiobsp_world"));

        csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        expect(map.entities[1].int_value("zhlt_studiobsp") == 0);
        expect(std::string(map.entities[1].value("classname")) == "info_null");
        expect(map.entities[1].num_brushes == 0);
        require(map.entities[0].num_brushes == 2);
        expect(map.brushes[0].entity_num == 0);
        expect(map.brushes[1].entity_num == 0);
    }

    test("studio_bsp.disabled collision leaves render only brushes")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_bsp_noclip");
        write_model(root);

        csg::map_source map;
        map.entities.push_back(make_world());
        csg::map_entity entity = make_studio_entity();
        entity.set_value("zhlt_studiobsp_collision", "0");
        map.entities.push_back(entity);

        csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        require(map.brushes.size() == 2);
        expect(map.brushes[0].noclip);
        expect(!map.brushes[0].cliphull);
    }

    test("studio_bsp.every prism face keeps a usable texture")
    {
        const std::filesystem::path root =
            test_support::scratch_directory("csg_studio_bsp_single");
        write_model(root);

        csg::map_source map;
        map.entities.push_back(make_world());
        csg::map_entity entity = make_studio_entity();
        entity.set_value("zhlt_studiobsp_doublesided", "0");
        map.entities.push_back(entity);

        csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        require(map.brushes.size() == 2);
        const csg::map_brush &brush = map.brushes[0];
        for (int side = 0; side < 5; side++)
        {
            const csg::brush_side &face = map.sides[(std::size_t)brush.first_side + side];
            expect(face.texture.txcommand == 'Q');
            expect(std::string(face.texture.name) != "NULL");
        }
    }

    test("studio_bsp.a missing model leaves the entity untouched")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_bsp_missing");
        std::filesystem::create_directories(root / "maps");

        csg::map_source map;
        map.entities.push_back(make_world());
        map.entities.push_back(make_studio_entity());

        const std::vector<format::mip_texture> textures =
            csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        expect(textures.empty());
        expect(std::string(map.entities[1].value("classname")) == "env_static");
        expect(map.entities[1].num_brushes == 0);
        expect(map.brushes.empty());
        expect(map.sides.empty());
    }

    test("studio_bsp.insertion shifts later brush and side indices")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_bsp_shift");
        write_model(root);

        csg::map_source map;
        csg::map_entity world = make_world();
        world.num_brushes = 1;
        map.entities.push_back(world);

        csg::map_entity studio = make_studio_entity();
        studio.first_brush = 1;
        map.entities.push_back(studio);

        csg::map_entity tail;
        tail.set_value("classname", "func_wall");
        tail.first_brush = 1;
        tail.num_brushes = 1;
        map.entities.push_back(tail);

        csg::map_brush world_brush;
        world_brush.first_side = 0;
        world_brush.num_sides = 1;
        map.brushes.push_back(world_brush);
        csg::map_brush tail_brush;
        tail_brush.first_side = 1;
        tail_brush.num_sides = 1;
        map.brushes.push_back(tail_brush);

        csg::brush_side side;
        std::memcpy(side.texture.name, "BRICK", 6);
        map.sides.push_back(side);

        csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        require(map.entities[1].num_brushes == 2);
        expect(map.entities[1].first_brush == 1);
        expect(map.entities[2].first_brush == 3);
        require(map.brushes.size() == 4);
        expect(map.brushes[1].first_side == 1);
        expect(map.brushes[2].first_side == 6);
        expect(map.brushes[3].first_side == 11);
        expect(map.sides.size() == 11);
        expect(std::string(map.sides[0].texture.name) == "BRICK");
    }

    test("studio_bsp.inflates sliver triangles below the csg epsilon")
    {
        const std::filesystem::path root =
            test_support::scratch_directory("csg_studio_bsp_sliver");
        write_sliver_model(root);

        csg::map_source map;
        map.entities.push_back(make_world());
        csg::map_entity entity;
        entity.set_value("classname", "env_static");
        entity.set_value("model", "models/props/sliver.mdl");
        entity.set_value("origin", "0 0 0");
        entity.set_value("zhlt_studiobsp", "1");
        entity.set_value("zhlt_studiobsp_collision", "1");
        map.entities.push_back(entity);

        csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        // the 0.1 unit tall triangle is too thin for the plane epsilon, so it
        // must be inflated instead of skipped or emitted as a leaky wedge
        require(map.brushes.size() == 1);
        const csg::map_brush &brush = map.brushes[0];
        require(brush.num_sides == 5);
        vec_t minimum = 1e9;
        vec_t maximum = -1e9;
        const csg::brush_side &front = map.sides[(std::size_t)brush.first_side];
        for (int i = 0; i < 3; i++)
        {
            const vec_t y = front.planepts[i].y;
            minimum = std::min(minimum, y);
            maximum = std::max(maximum, y);
        }
        expect(maximum - minimum >= 0.45);
    }

    test("studio_bsp.build_exacthull_hull computes correct distances on slopes and axial faces")
    {
        csg::exacthull_triangle tri;
        tri.points[0] = {0, 0, 0};
        tri.points[1] = {0, 64, 0};
        tri.points[2] = {64, 0, -64};
        math::vec3v n;
        math::cross(tri.points[1] - tri.points[0], tri.points[2] - tri.points[0], n);
        math::normalize(n);
        tri.normal = n;

        const math::vec3v half{16, 16, 36};
        auto tree_simple = csg::build_exacthull_hull({tri}, half, false);
        auto tree_legacy = csg::build_exacthull_hull({tri}, half, true);

        require(tree_simple.valid);
        require(tree_legacy.valid);
        require(!tree_simple.nodes.empty());
        require(!tree_legacy.nodes.empty());

        const math::vec3v slope_n{(vec_t)(1.0 / std::sqrt(2.0)), 0, (vec_t)(1.0 / std::sqrt(2.0))};
        vec_t slope_dist_simple = -1;
        for (const auto &p : tree_simple.planes)
        {
            if (math::length(p.normal - slope_n) < 1e-3)
            {
                slope_dist_simple = p.dist;
                break;
            }
        }
        vec_t slope_dist_legacy = -1;
        for (const auto &p : tree_legacy.planes)
        {
            if (math::length(p.normal - slope_n) < 1e-3)
            {
                slope_dist_legacy = p.dist;
                break;
            }
        }
        require(slope_dist_simple > 0);
        require(slope_dist_legacy > 0);

        const vec_t expected_simple = (vec_t)(52.0 / std::sqrt(2.0));
        const vec_t expected_legacy = (vec_t)26.0;
        expect(std::fabs(slope_dist_simple - expected_simple) < 0.1);
        expect(std::fabs(slope_dist_legacy - expected_legacy) < 0.1);
        expect(slope_dist_simple > slope_dist_legacy + 10.0);

        vec_t x_dist_simple = -1, x_dist_legacy = -1;
        for (const auto &p : tree_simple.planes)
            if (math::length(p.normal - math::vec3v{1, 0, 0}) < 1e-3)
                x_dist_simple = p.dist;
        for (const auto &p : tree_legacy.planes)
            if (math::length(p.normal - math::vec3v{1, 0, 0}) < 1e-3)
                x_dist_legacy = p.dist;
        require(x_dist_simple > 0);
        require(x_dist_legacy > 0);
        expect(std::fabs(x_dist_simple - (64 + 16)) < 0.1);
        expect(std::fabs(x_dist_legacy - (64 + 16)) < 0.1);
        expect(std::fabs(x_dist_simple - x_dist_legacy) < 1e-4);
    }

    test("studio_bsp.collision mode 2 (ExactHull) generates direct clip model and bypasses CSG hull expansion")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_exacthull");
        write_model(root);
        const std::string wad_path = (root / "maps" / "test.wad").string();
        test_wad::write(wad_path, {"STONE"});

        csg::map_source map;
        add_world_cube(map, wad_path);

        csg::map_entity entity = make_studio_entity();
        entity.set_value("zhlt_studiobsp_collision", "2");
        map.entities.push_back(entity);

        csg::csg_options options;
        options.brush.clip = csg::clip_type::simple;
        options.wad.map_path = (root / "maps" / "test.map").string();
        options.wad.wad_textures = true;
        csg::csg_result result = csg::run_csg(map, options);

        csg::intermediate_data data = csg::build_intermediate_data(result, options.brush);
        require(data.surface_clips.size() == 2);
        expect(!data.surface_clips[0].valid);
        expect(data.surface_clips[1].valid);
        expect(data.surface_clips[1].hulls[1].valid);
        expect(data.surface_clips[1].hulls[3].valid);
        expect(!data.surface_clips[1].hulls[1].nodes.empty());
        expect(!data.surface_clips[1].hulls[3].nodes.empty());
        expect(data.surfaces[1].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
        expect(data.surfaces[2].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
        expect(data.surfaces[3].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
        expect(data.brushes[1] == "-1\n-1\n");
        expect(data.brushes[2] == "-1\n-1\n");
        expect(data.brushes[3] == "-1\n-1\n");
    }

    test("studio_bsp.collision mode 3 (ExactHull + Legacytrim) generates direct clip model with slope distance")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_exacthull_legacy");
        write_model(root);
        const std::string wad_path = (root / "maps" / "test.wad").string();
        test_wad::write(wad_path, {"STONE"});

        csg::map_source map;
        add_world_cube(map, wad_path);

        csg::map_entity entity = make_studio_entity();
        entity.set_value("zhlt_studiobsp_collision", "3");
        map.entities.push_back(entity);

        csg::csg_options options;
        options.brush.clip = csg::clip_type::simple;
        options.wad.map_path = (root / "maps" / "test.map").string();
        options.wad.wad_textures = true;
        csg::csg_result result = csg::run_csg(map, options);

        csg::intermediate_data data = csg::build_intermediate_data(result, options.brush);
        require(data.surface_clips.size() == 2);
        expect(!data.surface_clips[0].valid);
        expect(data.surface_clips[1].valid);
        expect(data.surface_clips[1].hulls[1].valid);
        expect(data.surface_clips[1].hulls[3].valid);
        expect(!data.surface_clips[1].hulls[1].nodes.empty());
        expect(!data.surface_clips[1].hulls[3].nodes.empty());
        expect(data.surfaces[1].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
        expect(data.brushes[1] == "-1\n-1\n");
    }

    test("studio_bsp.exacthull world mode is forced to submodel with warning")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_exacthull_world");
        write_model(root);

        csg::map_source map;
        map.entities.push_back(make_world());
        csg::map_entity entity = make_studio_entity("zhlt_studiobsp_world");
        entity.set_value("zhlt_studiobsp_collision", "2");
        map.entities.push_back(entity);

        csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        expect(std::string(map.entities[1].value("classname")) == "func_wall");
        expect(map.entities[1].num_brushes == 2);
        expect(map.entities[0].num_brushes == 0);
    }

    test("studio_bsp.decoupled collision retains original entity and spawns collision submodel")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_decoupled_inject");
        write_model(root);

        csg::map_source map;
        map.entities.push_back(make_world());

        csg::map_entity entity;
        entity.set_value("classname", "cycler_sprite");
        entity.set_value("model", "models/props/bake.mdl");
        entity.set_value("origin", "128 64 32");
        entity.set_value("angles", "0 90 0");
        entity.set_value("zhlt_studiobsp", "0");
        entity.set_value("zhlt_studiobsp_collision", "2");
        entity.set_value("zhlt_studioshadow", "1");
        map.entities.push_back(entity);

        csg::inject_studio_brushes(map, (root / "maps" / "test.map").string());

        require(map.entities.size() == 3);

        const csg::map_entity &orig = map.entities[1];
        expect(std::string(orig.value("classname")) == "cycler_sprite");
        expect(std::string(orig.value("model")) == "models/props/bake.mdl");
        expect(std::string(orig.value("origin")) == "128 64 32");
        expect(std::string(orig.value("angles")) == "0 90 0");
        expect(std::string(orig.value("zhlt_studioshadow")) == "1");
        expect(orig.num_brushes == 0);

        const csg::map_entity &col = map.entities[2];
        expect(std::string(col.value("classname")) == "func_wall");
        expect(std::string(col.value("zhlt_collision_only")) == "1");
        expect(std::string(col.value("zhlt_studiobsp_collision")) == "2");
        expect(col.num_brushes == 2);
    }

    test("studio_bsp.decoupled collision runs through full CSG pipeline with ExactHull")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_decoupled_full");
        write_model(root);
        const std::string wad_path = (root / "maps" / "test.wad").string();
        test_wad::write(wad_path, {"STONE"});

        csg::map_source map;
        add_world_cube(map, wad_path);

        csg::map_entity entity;
        entity.set_value("classname", "cycler_sprite");
        entity.set_value("model", "models/props/bake.mdl");
        entity.set_value("origin", "0 0 0");
        entity.set_value("zhlt_studiobsp_collision", "2");
        map.entities.push_back(entity);

        csg::csg_options options;
        options.brush.clip = csg::clip_type::simple;
        options.wad.map_path = (root / "maps" / "test.map").string();
        options.wad.wad_textures = true;
        csg::csg_result result = csg::run_csg(map, options);

        require(result.map.entities.size() == 3);
        expect(std::string(result.map.entities[1].value("classname")) == "cycler_sprite");
        expect(std::string(result.map.entities[1].value("model")) == "models/props/bake.mdl");
        expect(std::string(result.map.entities[2].value("classname")) == "func_wall");
        expect(std::string(result.map.entities[2].value("zhlt_collision_only")) == "1");

        csg::intermediate_data data = csg::build_intermediate_data(result, options.brush);
        csg::build_bsp_data(result);
        expect(std::string(result.map.entities[2].value("model")) == "*1");
        require(data.surface_clips.size() == 2);
        expect(!data.surface_clips[0].valid);
        expect(data.surface_clips[1].valid);
        expect(data.surface_clips[1].hulls[1].valid);
        expect(data.surface_clips[1].hulls[3].valid);
        expect(!data.surface_clips[1].hulls[1].nodes.empty());
        expect(!data.surface_clips[1].hulls[3].nodes.empty());
        // For hull 0, collision_only emits empty face sequence for submodel
        expect(data.surfaces[0].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
        expect(data.surfaces[1].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
    }

    test("studio_bsp.collision only proxy mode strips mdl and creates invisible func_wall")
    {
        const std::filesystem::path root = test_support::scratch_directory("csg_studio_col_proxy");
        write_model(root);
        const std::string wad_path = (root / "maps" / "test.wad").string();
        test_wad::write(wad_path, {"STONE"});

        csg::map_source map;
        add_world_cube(map, wad_path);

        csg::map_entity entity;
        entity.set_value("classname", "cycler_sprite");
        entity.set_value("model", "models/props/bake.mdl");
        entity.set_value("origin", "128 64 32");
        entity.set_value("angles", "0 90 0");
        entity.set_value("zhlt_studiobsp", "2");
        entity.set_value("zhlt_studiobsp_collision", "2");
        map.entities.push_back(entity);

        csg::csg_options options;
        options.brush.clip = csg::clip_type::simple;
        options.wad.map_path = (root / "maps" / "test.map").string();
        options.wad.wad_textures = true;
        csg::csg_result result = csg::run_csg(map, options);

        // Transformed in-place: map entities count stays 2 (worldspawn + func_wall)
        require(result.map.entities.size() == 2);
        expect(std::string(result.map.entities[1].value("classname")) == "func_wall");
        expect(std::string(result.map.entities[1].value("zhlt_collision_only")) == "1");
        expect(std::string(result.map.entities[1].value("zhlt_studiobsp_collision")) == "2");
        expect(std::string(result.map.entities[1].value("origin")).empty());

        csg::intermediate_data data = csg::build_intermediate_data(result, options.brush);
        csg::build_bsp_data(result);

        expect(std::string(result.map.entities[1].value("model")) == "*1");
        // Verify MDL path is nowhere in entity 1's values
        for (const auto &pair : result.map.entities[1].pairs)
            expect(pair.second.find("bake.mdl") == std::string::npos);

        require(data.surface_clips.size() == 2);
        expect(!data.surface_clips[0].valid);
        expect(data.surface_clips[1].valid);
        expect(data.surface_clips[1].hulls[1].valid);
        expect(data.surface_clips[1].hulls[3].valid);
        expect(data.surfaces[0].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
        expect(data.surfaces[1].rfind("-1 -1 -1 -1 -1\n-1 -1 -1 -1 -1\n") != std::string::npos);
    }
}
