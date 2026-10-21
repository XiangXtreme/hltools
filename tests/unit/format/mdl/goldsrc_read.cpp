#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "common/binary.h"
#include "format/mdl/goldsrc/model.h"
#include "format/mdl/goldsrc/read.h"
#include "support/test.h"

namespace
{
    format::studio_model make_animated_model()
    {
        format::studio_model model;
        model.name = "props/test.mdl";

        format::studio_bone root;
        root.name = "root";
        root.position[2] = 10;
        model.bones.push_back(root);

        for (int i = 0; i < 3; i++)
        {
            format::studio_vertex vertex;
            vertex.position[0] = (float)i;
            vertex.normal[2] = 1;
            vertex.u = i == 0 ? 0.0f : 1.0f;
            vertex.v = 0.5f;
            vertex.bone = 0;
            model.vertices.push_back(vertex);
        }

        format::studio_mesh mesh;
        mesh.material = 0;
        mesh.indices = {0, 1, 2};
        model.meshes.push_back(mesh);

        format::studio_texture texture;
        texture.name = "test.bmp";
        texture.image.width = 32;
        texture.image.height = 16;
        texture.image.pixels.assign(32 * 16, 7);
        texture.image.palette[7] = {1, 2, 3};
        model.textures.push_back(texture);

        format::studio_sequence sequence;
        sequence.name = "bob";
        sequence.fps = 24;
        for (int f = 0; f < 2; f++)
        {
            format::studio_bone_key key;
            key.position[2] = 10.0f + (float)f * 6.0f;
            sequence.frames.push_back({key});
        }
        model.sequences.push_back(sequence);
        return model;
    }

    format::studio_model make_parented_model()
    {
        format::studio_model model;
        model.name = "props/child.mdl";

        format::studio_bone root;
        root.name = "root";
        root.position[2] = 5;
        model.bones.push_back(root);

        format::studio_bone child;
        child.name = "child";
        child.parent = 0;
        child.position[2] = 2;
        model.bones.push_back(child);

        for (int i = 0; i < 3; i++)
        {
            format::studio_vertex vertex;
            vertex.position[0] = (float)i;
            vertex.normal[2] = 1;
            vertex.bone = 1;
            model.vertices.push_back(vertex);
        }

        format::studio_mesh mesh;
        mesh.material = 0;
        mesh.indices = {0, 1, 2};
        model.meshes.push_back(mesh);

        format::studio_texture texture;
        texture.name = "child.bmp";
        texture.image.width = 8;
        texture.image.height = 8;
        texture.image.pixels.assign(64, 0);
        model.textures.push_back(texture);
        return model;
    }

    std::int32_t i32_at(const std::vector<byte> &data, std::size_t at)
    {
        std::int32_t value = 0;
        binary::reader(data).i32_at(at, value);
        return value;
    }

    std::size_t bone_table(const std::vector<byte> &data)
    {
        return (std::size_t)i32_at(data, 144);
    }
}

suite("unit.format.goldsrc_read")
{
    test("goldsrc_read.poses vertices at the requested frame")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_animated_model(), data));

        format::studio_model first;
        std::string error;
        format::goldsrc_model_pose pose;
        require(format::read_goldsrc_model(data, pose, first, &error));

        expect(first.vertices.size() == 3);
        expect(first.meshes.size() == 1);
        expect(first.meshes[0].indices.size() == 3);
        expect(std::fabs(first.vertices[0].position[2] - 10.0f) < 0.001f);
        expect(std::fabs(first.vertices[1].position[0] - 1.0f) < 0.001f);

        format::studio_model second;
        pose.frame = 1;
        require(format::read_goldsrc_model(data, pose, second, &error));
        expect(std::fabs(second.vertices[0].position[2] - 16.0f) < 0.001f);
    }

    test("goldsrc_read.restores normals and normalized texture coordinates")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_animated_model(), data));

        format::studio_model model;
        require(format::read_goldsrc_model(data, format::goldsrc_model_pose(), model));

        expect(std::fabs(model.vertices[0].normal[2] - 1.0f) < 0.001f);
        expect(std::fabs(model.vertices[0].u - 0.0f) < 0.001f);
        expect(std::fabs(model.vertices[0].v - 0.5f) < 0.001f);
        expect(std::fabs(model.vertices[2].u - 1.0f) < 0.001f);
        expect(std::fabs(model.vertices[2].v - 0.5f) < 0.001f);
    }

    test("goldsrc_read.loads embedded pixels and palette")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_animated_model(), data));

        format::studio_model model;
        require(format::read_goldsrc_model(data, format::goldsrc_model_pose(), model));

        require(model.textures.size() == 1);
        expect(model.textures[0].name == "test.bmp");
        expect(model.textures[0].image.width == 32);
        expect(model.textures[0].image.height == 16);
        expect(model.textures[0].image.pixels.size() == 32u * 16u);
        expect(model.textures[0].image.pixels[0] == 7);
        expect(model.textures[0].image.palette[7][0] == 1);
        expect(model.textures[0].image.palette[7][1] == 2);
        expect(model.textures[0].image.palette[7][2] == 3);
    }

    test("goldsrc_read.composes parent bone transforms")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_parented_model(), data));

        format::studio_model model;
        require(format::read_goldsrc_model(data, format::goldsrc_model_pose(), model));

        expect(model.bones.size() == 2);
        expect(std::fabs(model.vertices[0].position[2] - 7.0f) < 0.001f);
    }

    test("goldsrc_read.recomputes bounds from the posed frame")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_animated_model(), data));

        format::studio_model model;
        require(format::read_goldsrc_model(data, format::goldsrc_model_pose(), model));

        expect(std::fabs(model.bbmin[2] - 10.0f) < 0.001f);
        expect(std::fabs(model.bbmax[2] - 10.0f) < 0.001f);
        expect(std::fabs(model.bbmax[0] - 2.0f) < 0.001f);
    }

    test("goldsrc_read.rejects a foreign or truncated file")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_animated_model(), data));

        format::studio_model model;
        std::string error;

        std::vector<byte> foreign = data;
        foreign[0] = 'X';
        expect_false(format::read_goldsrc_model(foreign, format::goldsrc_model_pose(),
                                                model, &error));
        expect(error.find("IDST") != std::string::npos);

        std::vector<byte> truncated = data;
        truncated.resize(64);
        expect_false(format::read_goldsrc_model(truncated, format::goldsrc_model_pose(),
                                                model, &error));
        expect(error.find("header") != std::string::npos);

        std::vector<byte> version = data;
        binary::writer(version).patch_i32(4, 11);
        expect_false(format::read_goldsrc_model(version, format::goldsrc_model_pose(),
                                                model, &error));
        expect(error.find("not the supported version") != std::string::npos);
    }

    test("goldsrc_read.rejects a forward bone parent")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_parented_model(), data));
        binary::writer(data).patch_i32((std::int32_t)bone_table(data) + 32, 1);

        format::studio_model model;
        std::string error;
        expect_false(format::read_goldsrc_model(data, format::goldsrc_model_pose(),
                                                model, &error));
        expect(error.find("invalid parent") != std::string::npos);
    }

    test("goldsrc_read.rejects an out of range sequence")
    {
        std::vector<byte> data;
        require(format::write_goldsrc_model(make_animated_model(), data));

        format::studio_model model;
        std::string error;
        format::goldsrc_model_pose pose;
        pose.sequence = 3;
        expect_false(format::read_goldsrc_model(data, pose, model, &error));
        expect(error.find("sequence") != std::string::npos);
    }

    test("goldsrc_read.accepts a model whose skins live in a sibling T.mdl")
    {
        format::studio_model source = make_animated_model();
        source.textures.clear();
        source.meshes[0].material = 0;

        std::vector<byte> data;
        require(format::write_goldsrc_model(source, data));

        format::studio_model model;
        require(format::read_goldsrc_model(data, format::goldsrc_model_pose(), model));
        expect(model.textures.empty());
        require(model.meshes.size() == 1);
        expect(model.meshes[0].material == -1);
        expect(model.vertices.size() == 3);
    }
}
