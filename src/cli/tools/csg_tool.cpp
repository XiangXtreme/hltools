#include "csg_tool.h"

#include <chrono>
#include <cstdlib>
#include <utility>
#include <vector>

#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/threads.h"
#include "format/bsp/data.h"
#include "format/bsp/file.h"
#include "../../csg/bsp_output.h"
#include "../../csg/map_parser.h"
#include "compile_parameters.h"

namespace tools
{
    namespace
    {
        void print_csg_help()
        {
            logging::console(
                "usage\n"
                "  hltools csg [options] <map>\n"
                "\n"
                "  carves brushes into renderable geometry, builds the clipping hull\n"
                "  brushes, and resolves the map's textures from its wads.\n"
                "\n"
                "textures\n"
                "  -wadtextures          keep textures in the wads and reference them at\n"
                "                        runtime (default: embed them into the bsp)\n"
                "  -wadinclude <name>    embed textures from wads matching <name> (repeatable)\n"
                "  -nowadautodetect      disable wad auto-detection\n"
                "  -wadcfgfile <file>    wad configuration file\n"
                "  -wadconfig <name>     named configuration from the wad config file\n"
                "\n"
                "geometry\n"
                "  -cliptype <type>      smallest | normalized | simple | precise | legacy\n"
                "                       legacytrim (continuous exterior surface hulls)\n"
                "                                                       (default: simple)\n"
                "                        func_detail may override with zhlt_cliptype\n"
                "  -noclip               don't create the clipping hulls\n"
                "  -clipmerge            merge convex legacy collision brushes before expansion\n"
                "  -noskyclip            don't clip the world with sky brushes\n"
                "  -scale <n>            scale the whole map\n"
                "  -worldextent <n>      world bounds per axis              (default: 65536)\n"
                "  -brushunion <n>       warn above overlap ratio <n> (0.1 = 10%%)\n"
                "  -hullfile <file>      custom clipping hull sizes\n"
                "  -nullfile <file>      list of textures/entities to nullify\n"
                "  -nullifytrigger       convert AAATRIGGER faces to null\n"
                "\n"
                "entities\n"
                "  -onlyents             update the entity lump of an existing bsp only\n"
                "  -nolightopt           don't strip redundant named lights\n"
                "\n"
                "misc\n"
                "  -threads <n>          worker threads             (default: all cores)\n");
        }
    }

    int run_csg_tool(int argc, char **argv)
    {
        cli::args command_line(argc, argv);
        const bool want_help = command_line.has("-h")
            || command_line.has("-help") || command_line.has("--help");
        if (command_line.empty() || command_line.map_name().empty() || want_help)
        {
            print_csg_help();
            return want_help ? 0 : 1;
        }

        std::string map_path =
            fs::with_extension(command_line.map_name(), ".map");
        std::vector<unsigned char> source_bytes;
        if (!fs::read_all(map_path, source_bytes))
            err::fatal("could not read map source '%s'", map_path.c_str());
        const tools::compile_parameters parameters =
            compile_parameters_from_map(
                std::string(source_bytes.begin(), source_bytes.end()));
        staged_arguments arguments(
            argc, argv, parameters, cli::compiler_stage::csg);
        cli::args args(arguments.argc(), arguments.argv.data());

        threads::set_count(args.int_value("-threads", 0));

        std::string base = fs::strip_extension(map_path);
        logging::open_stage_log(base, "csg");
        logging::banner("csg");

        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        csg::csg_options options = parse_csg_options(args, map_path);
        // a traditional multi process chain must carry the recipe through its
        // intermediate bsp; entity only mode writes a final runtime bsp, so it
        // consumes and removes the metadata immediately
        options.compile_parameters_consumed = options.brush.only_entities;
        bool optimize_lights = !args.has("-nolightopt");
        logging::flush_settings();

        csg::map_source map = csg::load_map_file(map_path);

        auto start = std::chrono::steady_clock::now();
        csg::csg_result result = csg::run_csg(std::move(map), options);
        std::string bsp_path = base + ".bsp";

        if (options.brush.only_entities)
        {
            // rewrite only the entity lump of the existing bsp
            format::map_data existing;
            if (!format::bsp_file::load(bsp_path, existing))
                err::fatal("-onlyents requires an existing bsp '%s'", bsp_path.c_str());
            csg::replace_entities(result, existing, optimize_lights);
            if (!format::bsp_file::write(bsp_path, existing))
                err::fatal("could not write bsp '%s'", bsp_path.c_str());

            logging::console("updated entity lump: %zu entities\n", result.map.entities.size());
            auto onlyents_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> onlyents_elapsed = onlyents_end - start;
            logging::done(onlyents_elapsed.count());
            logging::close();
            return 0;
        }

        if (!csg::write_intermediate_files(base, result, options.brush))
            err::fatal("could not write csg intermediate files");
        // build_bsp_data mutates the entities (model numbers, reordering), so the
        // intermediate files above must be written first
        format::map_data bsp = csg::build_bsp_data(result, optimize_lights);
        if (!format::bsp_file::write(bsp_path, bsp))
            err::fatal("could not write bsp '%s'", bsp_path.c_str());

        logging::console("\nbuilt csg data: %zu entities, %zu texinfos, %zu texture bytes, cliptype %s\n",
                         result.entities.size(), result.texinfos.entries().size(), result.textures.size(),
                         csg::clip_type_name(options.brush.clip));

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        logging::done(elapsed.count());
        logging::close();
        return 0;
    }
}
