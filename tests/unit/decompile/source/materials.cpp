#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "common/binary.h"
#include "common/filesystem.h"
#include "common/types.h"
#include "decompile/source/materials.h"
#include "format/vbsp/data.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    void pad(binary::writer &sink, std::size_t count)
    {
        for (std::size_t i = 0; i < count; i++)
            sink.u8(0);
    }

    // varied=false writes one flat colour, which is what the automatic
    // single-colour reduction looks for; varied=true defeats it so density
    // behaviour can be tested on its own
    std::vector<byte> make_rgb_vtf(unsigned dimension, bool varied = false)
    {
        std::vector<byte> data;
        binary::writer sink(data);
        sink.raw(reinterpret_cast<const byte *>("VTF"), 3);
        sink.u8(0);
        sink.u32(7);
        sink.u32(2);
        sink.u32(80);
        sink.u16((std::uint16_t)dimension);
        sink.u16((std::uint16_t)dimension);
        sink.u32(0);
        sink.u16(1);
        sink.u16(0);
        pad(sink, 4);
        sink.f32(1);
        sink.f32(1);
        sink.f32(1);
        pad(sink, 4);
        sink.f32(1);
        sink.i32(2);  // RGB888
        sink.u8(1);   // one mip level
        sink.i32(-1); // no low-resolution image
        sink.u8(0);
        sink.u8(0);
        sink.u16(1);
        pad(sink, 15);
        for (unsigned pixel = 0; pixel < dimension * dimension; pixel++)
        {
            sink.u8(varied ? (byte)(pixel * 7) : (byte)96);
            sink.u8(varied ? (byte)(pixel * 13) : (byte)128);
            sink.u8(varied ? (byte)(pixel * 29) : (byte)160);
        }
        return data;
    }

    void write_material(const std::filesystem::path &path, const char *shader)
    {
        std::string text = std::string("\"") + shader + "\"\n{\n"
            "\t\"$basetexture\" \"custom/missing\"\n"
            "\t\"$selfillum\" \"1\"\n"
            "}\n";
        require(fs::write_all(path.string(), text.data(), text.size()));
    }

    void write_additive_material(const std::filesystem::path &path)
    {
        std::string text =
            "\"UnlitGeneric\"\n{\n"
            "\t\"$basetexture\" \"custom/additive\"\n"
            "\t\"$additive\" \"1\"\n"
            "}\n";
        require(fs::write_all(path.string(), text.data(), text.size()));
    }
}

suite("unit.decompile.source_materials")
{
    test("material_catalog distinguishes UnlitGeneric from self illuminated lightmapped materials")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_unlit");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "unlit.vmt", "UnlitGeneric");
        write_material(materials / "masked_glow.vmt", "LightmappedGeneric");

        format::source_map_data map;
        map.material_names = {"custom/unlit", "custom/masked_glow"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()});

        expect(catalog.resolve("CUSTOM/UNLIT").unlit);
        expect_false(catalog.resolve("custom/masked_glow").unlit);
    }

    test("material_catalog reduces texture dimensions and UV vectors together")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_max_texture");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "scaled.vmt", "LightmappedGeneric");
        std::vector<byte> vtf = make_rgb_vtf(32);
        require(fs::write_all((materials / "missing.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/scaled"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()}, 16);

        const decompile::resolved_material &material =
            catalog.resolve("custom/scaled");
        require(material.has_texture);
        expect(material.u_scale == 0.5);
        expect(material.v_scale == 0.5);
        require(catalog.textures().size() == 1);
        expect(catalog.textures()[0].width == 16);
        expect(catalog.textures()[0].height == 16);
    }

    test("material_catalog can preserve selected textures at native size")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_full_texture");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "logo.vmt", "LightmappedGeneric");
        std::vector<byte> vtf = make_rgb_vtf(32);
        require(fs::write_all((materials / "missing.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/logo"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()}, 16, {"{LOGO"});

        const decompile::resolved_material &material =
            catalog.resolve("custom/logo");
        require(material.has_texture);
        expect(material.u_scale == 1.0);
        expect(material.v_scale == 1.0);
        require(catalog.textures().size() == 1);
        expect(catalog.textures()[0].width == 32);
        expect(catalog.textures()[0].height == 32);
    }

    test("material_catalog reduces single-colour materials to the minimum size")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_flat");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "fill.vmt", "LightmappedGeneric");
        std::vector<byte> vtf = make_rgb_vtf(64);
        require(fs::write_all((materials / "missing.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/fill"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()}, 512);

        const decompile::resolved_material &material = catalog.resolve("custom/fill");
        require(material.has_texture);
        expect(material.flat);
        expect(catalog.flattened() == 1);
        // the density falls with the size, which is the whole point: a flat
        // fill costs lightmap atlas for detail it does not have
        expect(material.u_scale == 0.25);
        expect(material.v_scale == 0.25);
        require(catalog.textures().size() == 1);
        expect(catalog.textures()[0].width == 16);
        expect(catalog.textures()[0].height == 16);
    }

    test("material_catalog keeps single-colour materials when auto flat is off")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_flat_off");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "fill.vmt", "LightmappedGeneric");
        std::vector<byte> vtf = make_rgb_vtf(64);
        require(fs::write_all((materials / "missing.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/fill"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()}, 64, {}, 0, false);

        const decompile::resolved_material &material = catalog.resolve("custom/fill");
        require(material.has_texture);
        expect_false(material.flat);
        expect(catalog.flattened() == 0);
        expect(material.u_scale == 1.0);
        require(catalog.textures().size() == 1);
        expect(catalog.textures()[0].width == 64);
    }

    test("material_catalog can derive lightmap density from a smaller size than it stores")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_lmsize");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_material(materials / "detail.vmt", "LightmappedGeneric");
        std::vector<byte> vtf = make_rgb_vtf(64, true);
        require(fs::write_all((materials / "missing.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/detail"};

        decompile::material_catalog catalog;
        catalog.build(map, {game.string()}, 64, {}, 16);

        const decompile::resolved_material &material = catalog.resolve("custom/detail");
        require(material.has_texture);
        expect_false(material.flat);
        // every pixel is kept, but the axes carry the 16-wide density, so the
        // texture tiles over four times the world distance for a quarter of the
        // luxels at the same apparent sharpness
        expect(material.u_scale == 0.25);
        expect(material.v_scale == 0.25);
        require(catalog.textures().size() == 1);
        expect(catalog.textures()[0].width == 64);
        expect(catalog.textures()[0].height == 64);
    }

    test("material_catalog preserves additive blending")
    {
        std::filesystem::path game =
            test_support::scratch_directory("source_materials_additive");
        std::filesystem::path materials = game / "materials" / "custom";
        require(fs::make_directory(materials.string()));
        write_additive_material(materials / "additive.vmt");
        std::vector<byte> vtf = make_rgb_vtf(16);
        require(fs::write_all((materials / "additive.vtf").string(),
                              vtf.data(), vtf.size()));

        format::source_map_data map;
        map.material_names = {"custom/additive"};
        decompile::material_catalog catalog;
        catalog.build(map, {game.string()});

        const decompile::resolved_material &material =
            catalog.resolve("custom/additive");
        expect(material.additive);
        expect(material.render_amount == 255);
        expect_false(material.translucent);
    }
}
