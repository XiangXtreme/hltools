#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "common/filesystem.h"
#include "csg/studio_bsp.h"
#include "format/mdl/goldsrc/model.h"
#include "support/scratch.h"
#include "support/test.h"

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
}
