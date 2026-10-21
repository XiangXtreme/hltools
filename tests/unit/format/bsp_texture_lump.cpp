#include <string>
#include <vector>

#include "format/bmp/image.h"
#include "format/bsp/data.h"
#include "format/bsp/texture_lump.h"
#include "format/miptex/texture.h"
#include "support/test.h"

namespace
{
    bool make_texture(const char *name, byte shade, format::mip_texture &texture)
    {
        format::indexed_image image;
        image.width = 16;
        image.height = 16;
        image.pixels.assign(256, shade);
        std::string error;
        return format::build_mip_texture(name, image, texture, &error);
    }
}

suite("unit.format.texture_lump")
{
    test("texture_lump.round trips embedded textures")
    {
        format::mip_texture first;
        format::mip_texture second;
        require(make_texture("FIRST", 1, first));
        require(make_texture("SECOND", 2, second));
        const std::vector<format::mip_texture> source = {first, second};
        const std::vector<byte> lump = format::build_texture_lump(source);

        const std::vector<std::string> names = format::bsp_texture_names(lump);
        require(names.size() == 2);
        expect(names[0] == "FIRST");
        expect(names[1] == "SECOND");

        format::map_data map;
        map.textures = lump;
        std::vector<format::mip_texture> decoded;
        std::string error;
        require(format::collect_bsp_textures(map, decoded, nullptr, &error));
        require(decoded.size() == 2);
        expect(decoded[0].name == "FIRST");
        expect(decoded[0].width == 16);
        expect(decoded[1].name == "SECOND");
        expect(decoded[1].data == source[1].data);
    }

    test("texture_lump.an empty set writes a valid empty directory")
    {
        const std::vector<byte> lump = format::build_texture_lump({});
        require(lump.size() == sizeof(int));
        format::map_data map;
        map.textures = lump;
        std::vector<format::mip_texture> decoded;
        require(format::collect_bsp_textures(map, decoded, nullptr, nullptr));
        expect(decoded.empty());
    }
}
