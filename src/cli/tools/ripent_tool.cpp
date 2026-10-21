#include "ripent_tool.h"

#include <cstring>
#include <string>
#include <vector>

#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "format/bsp/file.h"
#include "format/bsp/texture_lump.h"
#include "format/miptex/texture.h"
#include "format/wad/archive.h"

namespace tools
{
    namespace
    {
        void print_ripent_help()
        {
            logging::console(
                "usage\n"
                "  hltools ripent -export <map[.bsp]>\n"
                "  hltools ripent -import <map[.bsp]>\n"
                "  hltools ripent -textureexport <map[.bsp]> [-out <file.wad>]\n"
                "  hltools ripent -textureimport <map[.bsp]> [-in <file.wad>]\n"
                "\n"
                "  -export writes the entity lump next to the bsp as <map>.ent.\n"
                "  -import reads <map>.ent back into the bsp, leaving every other\n"
                "  lump untouched. -textureexport writes the embedded miptex set as\n"
                "  a wad3 archive (<map>_textures.wad by default); -textureimport\n"
                "  replaces the texture lump from such a wad and remaps texinfo.\n");
        }

        // rebuild the texture lump from decoded textures. bsp offsets are
        // relative to the miptex header, which is exactly the layout
        // collect_bsp_textures decodes again.
        bool replace_texture_lump(format::map_data &map,
                                  const std::vector<format::mip_texture> &textures,
                                  std::string &error)
        {
            const std::vector<std::string> old_names = format::bsp_texture_names(map.textures);
            std::vector<byte> lump = format::build_texture_lump(textures);
            for (format::texinfo_t &info : map.texinfo)
            {
                if (info.miptex < 0 || (std::size_t)info.miptex >= old_names.size())
                    continue;
                const std::string &name = old_names[(std::size_t)info.miptex];
                int replacement = -1;
                for (std::size_t i = 0; i < textures.size(); i++)
                {
                    if (str::iequals(name.c_str(), textures[i].name.c_str()))
                    {
                        replacement = (int)i;
                        break;
                    }
                }
                if (replacement < 0)
                {
                    error = "texture '" + name + "' is missing from the imported wad";
                    return false;
                }
                info.miptex = replacement;
            }
            map.textures = std::move(lump);
            return true;
        }

        std::string ent_path_for(const std::string &bsp_path)
        {
            return fs::strip_extension(bsp_path) + ".ent";
        }

        std::string default_wad_path(const std::string &bsp_path)
        {
            return fs::strip_extension(bsp_path) + "_textures.wad";
        }

        int export_entities(const std::string &bsp_path)
        {
            format::map_data map;
            if (!format::bsp_file::load(bsp_path, map))
            {
                logging::console("ripent: could not load '%s'\n", bsp_path.c_str());
                return 1;
            }
            std::string text = map.entities;
            while (!text.empty() && text.back() == '\0')
                text.pop_back();
            const std::string path = ent_path_for(bsp_path);
            if (!fs::write_all(path, text.data(), text.size()))
            {
                logging::console("ripent: could not write '%s'\n", path.c_str());
                return 1;
            }
            const std::vector<format::entity> entities = format::parse_entities(text);
            logging::console("ripent: exported %d entities to '%s'\n", (int)entities.size(),
                             path.c_str());
            return 0;
        }

        int import_entities(const std::string &bsp_path)
        {
            const std::string path = ent_path_for(bsp_path);
            std::vector<byte> bytes;
            if (!fs::read_all(path, bytes))
            {
                logging::console("ripent: could not read '%s'\n", path.c_str());
                return 1;
            }
            std::string text(bytes.begin(), bytes.end());
            while (!text.empty() && text.back() == '\0')
                text.pop_back();
            text.push_back('\0');

            const std::vector<format::entity> entities = format::parse_entities(text);
            if (entities.empty())
            {
                logging::console("ripent: '%s' contains no entities\n", path.c_str());
                return 1;
            }

            format::map_data map;
            if (!format::bsp_file::load(bsp_path, map))
            {
                logging::console("ripent: could not load '%s'\n", bsp_path.c_str());
                return 1;
            }
            map.entities = std::move(text);
            if (!format::bsp_file::write(bsp_path, map))
            {
                logging::console("ripent: could not write '%s'\n", bsp_path.c_str());
                return 1;
            }
            logging::console("ripent: imported %d entities into '%s'\n", (int)entities.size(),
                             bsp_path.c_str());
            return 0;
        }

        int export_textures(const std::string &bsp_path, const std::string &wad_path)
        {
            format::map_data map;
            if (!format::bsp_file::load(bsp_path, map))
            {
                logging::console("ripent: could not load '%s'\n", bsp_path.c_str());
                return 1;
            }
            std::vector<format::mip_texture> textures;
            std::string error;
            if (!format::collect_bsp_textures(map, textures, nullptr, &error))
            {
                logging::console("ripent: %s\n", error.c_str());
                return 1;
            }
            if (textures.empty())
            {
                logging::console("ripent: '%s' has no embedded textures\n", bsp_path.c_str());
                return 1;
            }
            if (!format::write_wad3(wad_path, textures, &error))
            {
                logging::console("ripent: could not write '%s': %s\n", wad_path.c_str(),
                                 error.c_str());
                return 1;
            }
            logging::console("ripent: exported %d textures to '%s'\n", (int)textures.size(),
                             wad_path.c_str());
            return 0;
        }

        int import_textures(const std::string &bsp_path, const std::string &wad_path)
        {
            format::wad_archive wad;
            std::string error;
            if (!wad.load(wad_path, &error))
            {
                logging::console("ripent: %s\n", error.c_str());
                return 1;
            }

            std::vector<format::mip_texture> textures;
            for (const format::wad_lump &lump : wad.lumps())
            {
                if (lump.type != format::wad_miptex_type)
                    continue;
                format::mip_texture texture;
                if (!format::decode_mip_texture(lump.data, texture, &error))
                {
                    logging::console("ripent: texture '%s': %s\n", lump.name, error.c_str());
                    return 1;
                }
                textures.push_back(std::move(texture));
            }
            if (textures.empty())
            {
                logging::console("ripent: '%s' contains no miptex lumps\n", wad_path.c_str());
                return 1;
            }

            format::map_data map;
            if (!format::bsp_file::load(bsp_path, map))
            {
                logging::console("ripent: could not load '%s'\n", bsp_path.c_str());
                return 1;
            }
            if (!replace_texture_lump(map, textures, error))
            {
                logging::console("ripent: %s\n", error.c_str());
                return 1;
            }
            if (!format::bsp_file::write(bsp_path, map))
            {
                logging::console("ripent: could not write '%s'\n", bsp_path.c_str());
                return 1;
            }
            logging::console("ripent: imported %d textures into '%s'\n", (int)textures.size(),
                             bsp_path.c_str());
            return 0;
        }
    }

    int run_ripent_tool(int argc, char **argv)
    {
        bool export_ent = false;
        bool import_ent = false;
        bool export_tex = false;
        bool import_tex = false;
        bool help = false;
        std::string map_arg;
        std::string wad_arg;
        for (int i = 1; i < argc; i++)
        {
            const std::string token = argv[i];
            if (str::iequals(token.c_str(), "-export"))
                export_ent = true;
            else if (str::iequals(token.c_str(), "-import"))
                import_ent = true;
            else if (str::iequals(token.c_str(), "-textureexport"))
                export_tex = true;
            else if (str::iequals(token.c_str(), "-textureimport"))
                import_tex = true;
            else if (str::iequals(token.c_str(), "-out") || str::iequals(token.c_str(), "-in"))
            {
                if (i + 1 < argc)
                    wad_arg = argv[++i];
            }
            else if (str::iequals(token.c_str(), "-h") || str::iequals(token.c_str(), "-help")
                     || str::iequals(token.c_str(), "--help"))
                help = true;
            else if (!token.empty() && token[0] != '-')
                map_arg = token;
        }

        if (help || map_arg.empty())
        {
            print_ripent_help();
            return help ? 0 : 1;
        }

        const int modes = (int)export_ent + (int)import_ent + (int)export_tex + (int)import_tex;
        if (modes != 1)
        {
            logging::console("ripent: choose exactly one of -export, -import, "
                             "-textureexport or -textureimport\n");
            return 1;
        }

        const std::string bsp_path = fs::with_extension(map_arg, ".bsp");
        if (!fs::exists(bsp_path))
        {
            logging::console("ripent: '%s' does not exist\n", bsp_path.c_str());
            return 1;
        }

        if (export_ent)
            return export_entities(bsp_path);
        if (import_ent)
            return import_entities(bsp_path);

        const std::string wad_path = !wad_arg.empty()
            ? fs::with_extension(wad_arg, ".wad")
            : default_wad_path(bsp_path);
        return export_tex ? export_textures(bsp_path, wad_path)
                          : import_textures(bsp_path, wad_path);
    }
}
