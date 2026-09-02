#include "bsp_port_tool.h"

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "../../common/cmdline.h"
#include "../../common/filesystem.h"
#include "../../common/limits.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "../../decompile/source/map.h"
#include "format/vbsp/file.h"
#include "format/wad/archive.h"

namespace stdfs = std::filesystem;

namespace tools
{
    namespace
    {
        void print_help()
        {
            logging::console(
                "usage\n"
                "  hltools bsp port <source.bsp> <output.map> [options]\n"
                "\n"
                "  ports a Source BSP to GoldSrc MAP, WAD, models, and sky assets.\n"
                "\n"
                "options\n"
                "  -wad <file>         companion WAD path\n"
                "  -game <dir>         content root; repeatable\n"
                "  -toolwad <file>     compiler-texture WAD\n"
                "  -skysize <n>        sky face size, 64..4096; 0 skips (default 256)\n"
                "  -skyexposure <n>    HDR sky brightness, 0.01..16 (default 1)\n"
                "  -maxtexsize <n>     WAD texture cap, 16..512 in steps of 16\n"
                "  -lmtexsize <n>      texture size the lightmap density is\n"
                "                      derived from, 16..maxtexsize; keeps the\n"
                "                      pixels while spending fewer luxels\n"
                "                      (default: follow -maxtexsize)\n"
                "  -noflattex          keep single-colour materials at full size\n"
                "                      instead of reducing them to 16x16\n"
                "  -nolightgamma       keep source's raw light colours instead of\n"
                "                      converting them from sRGB to linear\n"
                "  -fulltex <material> bypass the texture cap; repeatable\n"
                "  -maxskinsize <n>    model skin cap, 16..512 in steps of 16\n"
                "  -chunkskins <n>     tiles per model-skin axis: 1, 2, 4, or 8\n"
                "  -sharedpalette      share palettes across material tiles\n"
                "  -tilearea <n>       surface area receiving tiles, 10..100\n"
                "  -disppower <n>      displacement detail cap, 0..4 (default 4);\n"
                "                      lower it when a displacement-heavy map\n"
                "                      overruns the goldsrc plane/face limits\n"
                "  -dispmodels         draw displacements as studio models, which\n"
                "                      cost no planes, nodes or faces; collision\n"
                "                      stays with the brushes\n"
                "  -dispchunk <n>      terrain model chunk size, 128..8192\n"
                "                      (default 1024)\n"
                "  -dispclip <scope>   displacement collision: all | floors | none\n"
                "                      (default all)\n"
                "  -dispmerge <n>      collision cells flatter than <n> units\n"
                "                      become one brush instead of two, 0..512\n"
                "                      (default 48)\n"
                "  -phyclip            convert .phy collision to brush entities\n"
                "  -force              overwrite output files\n");
        }

        bool is_help(const char *value)
        {
            return str::iequals(value, "-h") || str::iequals(value, "-help")
                || str::iequals(value, "--help") || str::iequals(value, "help");
        }

        bool extension_is(const std::string &path, const char *extension)
        {
            return str::iequals(stdfs::path(path).extension().string().c_str(), extension);
        }

        bool write_skybox(const std::string &map_path,
                          const decompile::skybox_result &sky,
                          bool force, std::string &error)
        {
            if (sky.faces.empty())
                return true;
            std::string dir = fs::directory(map_path);
            std::string env = dir.empty() ? "gfx/env" : dir + "/gfx/env";
            if (!fs::make_directory(env))
            {
                error = "could not create '" + env + "'";
                return false;
            }
            for (const decompile::skybox_face &face : sky.faces)
            {
                std::string path = env + "/" + face.filename;
                if (fs::exists(path) && !force)
                {
                    error = "'" + path + "' already exists (use -force)";
                    return false;
                }
                if (!fs::write_all(path, face.tga.data(), face.tga.size()))
                {
                    error = "could not write '" + path + "'";
                    return false;
                }
            }
            return true;
        }

        bool write_models(const std::string &map_path,
                          const decompile::model_result &models,
                          bool force, std::string &error)
        {
            std::string root = fs::directory(map_path);
            for (const decompile::converted_model &model : models.models)
            {
                std::string path = root.empty() ? model.path : root + "/" + model.path;
                std::string dir = fs::directory(path);
                if (!dir.empty() && !fs::make_directory(dir))
                {
                    error = "could not create '" + dir + "'";
                    return false;
                }
                if (fs::exists(path) && !force)
                {
                    error = "'" + path + "' already exists (use -force)";
                    return false;
                }
                if (!fs::write_all(path, model.data.data(), model.data.size()))
                {
                    error = "could not write '" + path + "'";
                    return false;
                }
            }
            return true;
        }

        // the porter's own knobs live in source_options, parsed straight into it
        struct port_args
        {
            std::string bsp_path;
            std::string map_path;
            std::string wad_path;
            std::string tool_wad;
            bool force = false;
            decompile::source_options options;
        };

        constexpr const char *command = "bsp port";

        int run_port(port_args args)
        {
            format::source_map_data source;
            std::string error;
            if (!format::source_bsp_file::load(args.bsp_path, source, &error))
            {
                logging::console("bsp port: %s\n", error.c_str());
                return 1;
            }
            if (fs::exists(args.map_path) && !args.force)
            {
                logging::console("bsp port: '%s' already exists (use -force)\n",
                                 args.map_path.c_str());
                return 1;
            }
            if (args.wad_path.empty())
                args.wad_path = fs::with_extension(args.map_path, ".wad");

            const std::string &map_path = args.map_path;
            const std::string &wad_path = args.wad_path;
            const std::string &tool_wad = args.tool_wad;
            const bool force = args.force;

            decompile::source_options options = std::move(args.options);
            // terrain models live under models/<map name>/, not the output path
            std::string map_name = fs::filename(args.map_path);
            std::size_t dot = map_name.rfind('.');
            if (dot != std::string::npos)
                map_name.erase(dot);
            options.map_name = map_name;
            options.additional_wad =
                stdfs::absolute(stdfs::path(args.wad_path)).lexically_normal().string();
            if (!args.tool_wad.empty())
                options.tool_wad =
                    stdfs::absolute(stdfs::path(args.tool_wad)).lexically_normal().string();

            decompile::source_result result;
            if (!decompile::port_source_map(source, options, result, &error))
            {
                logging::console("bsp port: %s\n", error.c_str());
                return 1;
            }
            if (!fs::write_all(map_path, result.text.data(), result.text.size()))
            {
                logging::console("bsp port: could not write '%s'\n", map_path.c_str());
                return 1;
            }
            if (!result.wad_textures.empty())
            {
                if (fs::exists(wad_path) && !force)
                {
                    logging::console("bsp port: '%s' already exists (use -force)\n",
                                     wad_path.c_str());
                    return 1;
                }
                if (!format::write_wad3(wad_path, result.wad_textures, &error))
                {
                    logging::console("bsp port: %s\n", error.c_str());
                    return 1;
                }
            }
            if (!write_skybox(map_path, result.skybox, force, error)
                || !write_models(map_path, result.models, force, error))
            {
                logging::console("bsp port: %s\n", error.c_str());
                return 1;
            }

            logging::console("wrote %s (vbsp v%d: %zu entities, %zu world brushes, "
                             "%zu detail brushes, %zu entity brushes, %zu sides)\n",
                             map_path.c_str(), source.version, result.entities,
                             result.world_brushes, result.detail_brushes,
                             result.entity_brushes, result.sides);
            if (!result.wad_textures.empty())
                logging::console("wrote %s (%zu textures, %zu unresolved)\n",
                                 wad_path.c_str(), result.converted_materials,
                                 result.placeholder_materials);
            if (!result.engine_textures.empty() && tool_wad.empty())
                logging::console("warning: %zu compiler textures need -toolwad\n",
                                 result.engine_textures.size());
            if (result.unlit_materials != 0)
                logging::console("preserved %zu unlit materials\n",
                                 result.unlit_materials);
            if (result.flattened_materials != 0)
                logging::console("reduced %zu single-colour materials to 16x16 "
                                 "(no visual change, frees lightmap atlas)\n",
                                 result.flattened_materials);
            if (!result.skybox.faces.empty())
                logging::console("wrote gfx/env/%s{bk,dn,ft,lf,rt,up}.tga (%zu/6)\n",
                                 result.skybox.sky_name.c_str(),
                                 result.skybox.faces.size());
            if (result.skybox.missing != 0)
                logging::console("warning: %zu sky faces unresolved\n",
                                 result.skybox.missing);
            if (result.player_start)
                logging::console("selected one player start; removed %zu extras\n",
                                 result.dropped_spawns);
            if (!result.models.models.empty() || result.models.failed != 0)
                logging::console("converted %zu models (%zu placed, %zu failed)\n",
                                 result.models.converted, result.prop_entities,
                                 result.models.failed);
            if (result.models.missing_skins != 0)
                logging::console("warning: %zu model skins unresolved\n",
                                 result.models.missing_skins);
            if (result.models.chunked_skins != 0)
                logging::console("split %zu model skins into tiles\n",
                                 result.models.chunked_skins);
            if (result.physics_clip_brushes != 0)
                logging::console("rebuilt %zu props as %zu collision brushes\n",
                                 result.physics_clip_props,
                                 result.physics_clip_brushes);
            logging::console("%zu brush models of %d\n", result.brush_models + 1,
                             limits::max_map_models);
            if ((int)result.brush_models + 1 > limits::max_map_models)
                logging::console("warning: map exceeds the model limit\n");
            if (result.skipped_brushes != 0)
                logging::console("warning: skipped %zu degenerate brushes\n",
                                 result.skipped_brushes);
            if (result.skipped_displacement_faces != 0
                || result.skipped_displacement_triangles != 0)
                logging::console("warning: skipped %zu displacement faces and %zu "
                                 "triangles\n", result.skipped_displacement_faces,
                                 result.skipped_displacement_triangles);
            return 0;
        }
    }

    int run_bsp_port_tool(int argc, char **argv)
    {
        if (argc < 2 || is_help(argv[1]))
        {
            print_help();
            return argc < 2 ? 1 : 0;
        }

        port_args args;
        decompile::source_options &options = args.options;
        std::vector<std::string> positional;
        for (int i = 1; i < argc; i++)
        {
            if (str::iequals(argv[i], "-force"))
            {
                args.force = true;
                continue;
            }
            if (str::iequals(argv[i], "-phyclip"))
            {
                options.physics_clips = true;
                continue;
            }
            if (str::iequals(argv[i], "-sharedpalette"))
            {
                options.shared_tile_palette = true;
                continue;
            }
            if (str::iequals(argv[i], "-dispmodels"))
            {
                options.displacement_models = true;
                continue;
            }
            if (str::iequals(argv[i], "-noflattex"))
            {
                options.auto_flat_textures = false;
                continue;
            }
            if (str::iequals(argv[i], "-nolightgamma"))
            {
                options.light_colour_gamma = false;
                continue;
            }
            if (str::iequals(argv[i], "-wad") || str::iequals(argv[i], "-toolwad")
                || str::iequals(argv[i], "-game") || str::iequals(argv[i], "-skysize")
                || str::iequals(argv[i], "-skyexposure")
                || str::iequals(argv[i], "-maxtexsize")
                || str::iequals(argv[i], "-lmtexsize")
                || str::iequals(argv[i], "-maxskinsize")
                || str::iequals(argv[i], "-chunkskins")
                || str::iequals(argv[i], "-tilearea")
                || str::iequals(argv[i], "-disppower")
                || str::iequals(argv[i], "-dispchunk")
                || str::iequals(argv[i], "-dispclip")
                || str::iequals(argv[i], "-dispmerge")
                || str::iequals(argv[i], "-fulltex"))
            {
                const std::string option = argv[i];
                if (++i >= argc || argv[i][0] == '\0')
                {
                    logging::console("bsp port: %s requires a value\n", option.c_str());
                    return 1;
                }
                const char *name = option.c_str();
                const char *text = argv[i];
                if (str::iequals(name, "-wad"))
                    args.wad_path = text;
                else if (str::iequals(name, "-toolwad"))
                    args.tool_wad = text;
                else if (str::iequals(name, "-game"))
                    options.game_dirs.emplace_back(text);
                else if (str::iequals(name, "-fulltex"))
                    options.full_size_textures.emplace_back(text);
                else if (str::iequals(name, "-skysize"))
                {
                    // the exported faces are square mips, and 0 skips them
                    long value = 0;
                    if (!cli::option_pow2(command, name, text, 64, 4096, true, value))
                        return 1;
                    options.sky_size = (unsigned)value;
                }
                else if (str::iequals(name, "-skyexposure"))
                {
                    double value = 0;
                    if (!cli::option_float(command, name, text, 0.01, 16.0, value))
                        return 1;
                    options.sky_exposure = value;
                }
                else if (str::iequals(name, "-maxtexsize"))
                {
                    long value = 0;
                    if (!cli::option_int(command, name, text, 16, 512, 16, value))
                        return 1;
                    options.max_texture_size = (unsigned)value;
                }
                else if (str::iequals(name, "-lmtexsize"))
                {
                    long value = 0;
                    if (!cli::option_int(command, name, text, 16, 512, 16, value))
                        return 1;
                    options.lightmap_texture_size = (unsigned)value;
                }
                else if (str::iequals(name, "-maxskinsize"))
                {
                    long value = 0;
                    if (!cli::option_int(command, name, text, 16, 512, 16, value))
                        return 1;
                    options.max_skin_size = (unsigned)value;
                }
                else if (str::iequals(name, "-chunkskins"))
                {
                    long value = 0;
                    if (!cli::option_choice(command, name, text, {1, 2, 4, 8}, value))
                        return 1;
                    options.skin_chunk_level = (int)value;
                }
                else if (str::iequals(name, "-tilearea"))
                {
                    double value = 0;
                    if (!cli::option_float(command, name, text, 10.0, 100.0, value))
                        return 1;
                    options.tile_area_keep = (float)(value / 100.0);
                }
                else if (str::iequals(name, "-disppower"))
                {
                    long value = 0;
                    if (!cli::option_int(command, name, text, 0, 4, 0, value))
                        return 1;
                    options.displacement_power = (int)value;
                }
                else if (str::iequals(name, "-dispchunk"))
                {
                    double value = 0;
                    if (!cli::option_float(command, name, text, 128.0, 8192.0, value))
                        return 1;
                    options.displacement_chunk = value;
                }
                else if (str::iequals(name, "-dispmerge"))
                {
                    double value = 0;
                    if (!cli::option_float(command, name, text, 0.0, 512.0, value))
                        return 1;
                    options.displacement_merge = value;
                }
                else if (str::iequals(name, "-dispclip"))
                {
                    using scope = decompile::source_options::collision_scope;
                    if (str::iequals(text, "all"))
                        options.displacement_clip = scope::all;
                    else if (str::iequals(text, "floors"))
                        options.displacement_clip = scope::floors;
                    else if (str::iequals(text, "none"))
                        options.displacement_clip = scope::none;
                    else
                    {
                        logging::console("bsp port: -dispclip expects all, floors, "
                                         "or none\n");
                        return 1;
                    }
                }
                else
                {
                    // the list above and these branches must stay in step
                    logging::console("bsp port: %s takes a value but is not "
                                     "handled\n", name);
                    return 1;
                }
                continue;
            }
            if (argv[i][0] == '-')
            {
                logging::console("bsp port: unknown option '%s'\n", argv[i]);
                return 1;
            }
            positional.emplace_back(argv[i]);
        }

        if (positional.size() != 2 || !extension_is(positional[0], ".bsp")
            || !extension_is(positional[1], ".map"))
        {
            logging::console("bsp port: expected a Source .bsp and .map destination\n");
            return 1;
        }
        if (!args.wad_path.empty() && !extension_is(args.wad_path, ".wad"))
        {
            logging::console("bsp port: companion output must end in .wad\n");
            return 1;
        }
        args.bsp_path = positional[0];
        args.map_path = positional[1];
        return run_port(std::move(args));
    }
}
