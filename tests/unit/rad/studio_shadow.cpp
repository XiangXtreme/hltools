#include <cmath>
#include <filesystem>
#include <vector>

#include "common/filesystem.h"
#include "format/mdl/goldsrc/model.h"
#include "rad/internal.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    rad::studio_shadow_triangle make_triangle(float z = 0.0f)
    {
        rad::studio_shadow_triangle triangle;
        triangle.p[0] = math::vec3v{0.0f, 0.0f, z};
        triangle.p[1] = math::vec3v{8.0f, 0.0f, z};
        triangle.p[2] = math::vec3v{0.0f, 8.0f, z};
        triangle.uv[0] = math::vec3v{0.0f, 0.0f, 0.0f};
        triangle.uv[1] = math::vec3v{1.0f, 0.0f, 0.0f};
        triangle.uv[2] = math::vec3v{0.0f, 1.0f, 0.0f};
        triangle.texture = 0;
        triangle.mins = math::vec3v{0.0f, 0.0f, z};
        triangle.maxs = math::vec3v{8.0f, 8.0f, z};
        return triangle;
    }

    rad::studio_shadow_model make_model()
    {
        rad::studio_shadow_model model;
        model.trace_mode = 1;
        model.triangles.push_back(make_triangle());
        model.textures.resize(1);
        model.textures[0].name = "opaque";
        model.textures[0].image.width = 2;
        model.textures[0].image.height = 2;
        model.textures[0].image.pixels.assign(4, 0);
        model.mins = math::vec3v{0.0f, 0.0f, 0.0f};
        model.maxs = math::vec3v{8.0f, 8.0f, 0.0f};
        return model;
    }

    math::vec3v point(float x, float y, float z)
    {
        return math::vec3v{x, y, z};
    }
}

suite("unit.rad.studio_shadow")
{
    test("studio_shadow.a segment through the triangle is blocked")
    {
        const rad::studio_shadow_model model = make_model();
        expect(rad::test_segment_against_studio_model(model, point(1, 1, 10), point(1, 1, -10)));
    }

    test("studio_shadow.a segment which stops before the triangle is not blocked")
    {
        const rad::studio_shadow_model model = make_model();
        expect_false(
            rad::test_segment_against_studio_model(model, point(1, 1, 10), point(1, 1, 5)));
    }

    test("studio_shadow.a segment beside the triangle is not blocked")
    {
        const rad::studio_shadow_model model = make_model();
        expect_false(rad::test_segment_against_studio_model(
            model, point(20, 20, 10), point(20, 20, -10)));
    }

    test("studio_shadow.a surface near the segment start does not block itself")
    {
        const rad::studio_shadow_model model = make_model();
        // the hit lands 1 unit away, inside the 1.5 near plane
        expect_false(
            rad::test_segment_against_studio_model(model, point(1, 1, 1), point(1, 1, -10)));
    }

    test("studio_shadow.an additive skin never blocks")
    {
        rad::studio_shadow_model model = make_model();
        model.textures[0].flags = format::studio_nf_additive;
        expect_false(
            rad::test_segment_against_studio_model(model, point(1, 1, 10), point(1, 1, -10)));
    }

    test("studio_shadow.a masked skin samples the palette index at the hit")
    {
        rad::studio_shadow_model model = make_model();
        model.textures[0].flags = format::studio_nf_masked;

        // the hit at (1,1) is texel (0,0) of the 2x2 skin. the palette's last
        // index is the punched out pixel, so 255 lets light through.
        model.textures[0].image.pixels[0] = 255;
        expect_false(
            rad::test_segment_against_studio_model(model, point(1, 1, 10), point(1, 1, -10)));

        model.textures[0].image.pixels[0] = 7;
        expect(rad::test_segment_against_studio_model(model, point(1, 1, 10), point(1, 1, -10)));
    }

    test("studio_shadow.the fast mode only tests the triangle bounds")
    {
        rad::studio_shadow_model model = make_model();
        model.trace_mode = 0;
        // this segment misses the triangle but crosses its bounding box
        expect(rad::test_segment_against_studio_model(model, point(6, 6, 10), point(6, 6, -10)));
        expect_false(rad::test_segment_against_studio_model(model, point(20, 20, 10),
                                                            point(20, 20, -10)));
    }

    test("studio_shadow.loads an env_static model from the game directory")
    {
        const std::filesystem::path root = test_support::scratch_directory("rad_studio_shadow");
        std::filesystem::create_directories(root / "maps");
        std::filesystem::create_directories(root / "models" / "props");

        format::studio_model source;
        source.name = "models/props/shadow.mdl";
        format::studio_bone bone;
        bone.name = "root";
        source.bones.push_back(bone);

        const float corners[4][2] = {{-8, -8}, {8, -8}, {8, 8}, {-8, 8}};
        for (const auto &corner : corners)
        {
            format::studio_vertex vertex;
            vertex.position[0] = corner[0];
            vertex.position[1] = corner[1];
            vertex.normal[2] = 1;
            vertex.u = (corner[0] + 8.0f) / 16.0f;
            vertex.v = (corner[1] + 8.0f) / 16.0f;
            source.vertices.push_back(vertex);
        }
        format::studio_mesh mesh;
        mesh.material = 0;
        mesh.indices = {0, 1, 2, 0, 2, 3};
        source.meshes.push_back(mesh);

        format::studio_texture texture;
        texture.name = "shadow.bmp";
        texture.image.width = 16;
        texture.image.height = 16;
        texture.image.pixels.assign(256, 0);
        source.textures.push_back(texture);

        std::vector<byte> bytes;
        require(format::write_goldsrc_model(source, bytes));
        require(fs::write_all((root / "models" / "props" / "shadow.mdl").string(),
                              bytes.data(), bytes.size()));

        rad::rad_state state;
        state.base_path = (root / "maps" / "shadow").string();
        state.options.studioshadow = true;
        format::entity entity;
        entity.append("classname", "env_static");
        entity.append("model", "models/props/shadow.mdl");
        entity.append("origin", "0 0 0");
        state.entities.push_back(entity);

        rad::load_studio_models(state);
        require(state.num_studio_models == 1);
        expect(rad::test_segment_against_studio_model(state.studio_models[0], point(0, 0, 10),
                                                      point(0, 0, -10)));

        rad::free_studio_models(state);
        expect(state.num_studio_models == 0);
        expect(state.studio_models.empty());
    }
}
