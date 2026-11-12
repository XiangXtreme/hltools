#include "rad_tool.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <vector>

#include "../../common/error.h"
#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "../../common/threads.h"
#include "format/bsp/file.h"
#include "format/bsp/usage_chart.h"
#include "compile_parameters.h"

namespace stdfs = std::filesystem;

namespace tools
{
    namespace
    {
        void print_rad_help()
        {
            logging::console(
                "usage\n"
                "  hltools rad [options] <map>\n"
                "  hltools rad -h                     show this help\n"
                "\n"
                "  radiosity lighting: direct light gathering, patch-to-patch bounces,\n"
                "  and the final lightmaps. reads lights.rad / <map>.rad for texlights.\n"
                "\n"
                "quality / speed\n"
                "  -extra                9x oversampled lightmaps (bounces raised to 12)\n"
                "  -fast                 fast low-quality mode\n"
                "  -gpu                  gpu lighting (approximate; cpu fallback)\n"
                "  -bounce <n>           bounce passes                      (default: 8)\n"
                "  -chop <n>             bounce patch size                  (default: 64)\n"
                "  -texchop <n>          texlight patch size                (default: 32)\n"
                "  -vismatrix <type>     normal | sparse | off              (default: sparse)\n"
                "  -compress <0-2>       transfers: float32|float16|float8  (default: 1)\n"
                "  -rgbcompress <0-3>    rgb transfers: 96|48|32|24 bit     (default: 2)\n"
                "  -incremental          keep transfer files between runs\n"
                "\n"
                "brightness / colour\n"
                "  -scale <n>            light intensity scale              (default: 2.0)\n"
                "  -colourscale <r g b>  per-channel intensity scale\n"
                "  -gamma <n>            lightmap gamma                     (default: 0.55)\n"
                "  -colourgamma <r g b>  per-channel gamma\n"
                "  -ambient <r g b>      minimum ambient light (0..1)\n"
                "  -minlight <n>         minimum luxel brightness (0-255)\n"
                "  -limiter <n>          brightness limiter                 (default: 255)\n"
                "  -pre25                pre-anniversary fullbright clamp (-limiter 188)\n"
                "  -sky <n>              sky diffuse lighting scale         (default: 1.0)\n"
                "  -dlight <n>           direct light threshold             (default: 10)\n"
                "  -dscale <n>           direct light scale                 (default: 1.0)\n"
                "  -fade <n>             global light fade                  (default: 1.0)\n"
                "  -coring <n>           light cutoff threshold             (default: 0.01)\n"
                "\n"
                "surfaces\n"
                "  -smooth <n>           phong smoothing angle              (default: 50)\n"
                "  -smooth2 <n>          smoothing angle across texture seams\n"
                "  -blur <n>             lightmap blur                      (default: 1.5)\n"
                "  -blurclamp <0..1>     reduce blur bleeding into shadows  (default: 0)\n"
                "  -pcf <1..8>           shadow filter taps per axis       (default: 1)\n"
                "  -ao                   enable ambient occlusion\n"
                "  -aoscale <1..1024>    AO ray length                     (default: 32)\n"
                "  -aogain <0.125..8>    AO falloff exponent               (default: 1)\n"
                "  -aolevel <1..8>       AO direction density              (default: 3)\n"
                "  -aominweight <0..0.1> skip low-weight AO rays           (default: 0.045)\n"
                "  -aoopacity <0..8>     AO strength                       (default: 1)\n"
                "  -aocolor <r g b>      AO tint                           (default: 0 0 0)\n"
                "  -aomode <0|1>         displayed or additive color       (default: 0)\n"
                "  -aostudio <0..3>      inherit/fast/normal/slow studio AO (default: 0)\n"
                "  -aostats              report AO ray counts\n"
                "  -lightcull            skip faint direct-light rays (approximate)\n"
                "  -nolerp               disable sample triangulation\n"
                "  -softsky <0|1>        soft skylight                      (default: 1)\n"
                "  -noskyfix             disable the sky lighting fix\n"
                "  -texlightgap <n>      texlight-face gap rejection        (default: 0)\n"
                "  -notexscale           don't scale patches with texture scale\n"
                "  -nosubdivide          don't chop patches\n"
                "  -noemitterrange       disable the texlight emitter range\n"
                "  -nobleedfix           disable the light bleed fix\n"
                "  -notextures           ignore texture colours for bounces\n"
                "  -texreflectgamma <n>  texture reflectivity gamma         (default: 1.76)\n"
                "  -texreflectscale <n>  texture reflectivity scale         (default: 0.7)\n"
                "  -depth <n>            translucent light depth            (default: 2.0)\n"
                "\n"
                "shadows\n"
                "  -noopaque             disable opaque entity shadows\n"
                "  -blockopaque <0|1>    opaque entities block light        (default: 1)\n"
                "  -nostudioshadow       disable studio model shadows\n"
                "  -customshadowwithbounce   custom shadows affect bounced light\n"
                "  -rgbtransfers         coloured shadow transfers (more memory)\n"
                "  -nospread             disable sunlight spread\n"
                "\n"
                "files / memory\n"
                "  <map>.rad             preferred texlights file next to <map>.map\n"
                "  lights.rad            supported fallback; may be removed later\n"
                "  -lights <file>        extra lights.rad file\n"
                "  -waddir <folder>      extra wad search folder (repeatable)\n"
                "  -lightdata <kb>       lighting memory budget       (default: 49152)\n"
                "  -texdata <kb>         texture memory budget        (default: 32768)\n"
                "\n"
                "debug\n"
                "  -dump                 write patch debug files\n"
                "  -dumpgather           write the per-face gather dump (<map>.gather)\n"
                "  -drawsample <x y z r> pointfile of samples in a sphere\n"
                "  -drawpatch / -drawedge / -drawlerp / -drawnudge / -drawoverload\n"
                "  -circus               debug colours for unlit luxels\n"
                "  -jitter <r g b> / -colourjitter <r g b>   lighting noise\n"
                "\n"
                "misc\n"
                "  -threads <n>          worker threads             (default: all cores)\n");

            logging::console(
                "\n"
                "map entity reference\n"
                "  Colour values accept: <brightness>, <r g b>, or <r g b brightness>.\n"
                "  In the four-value form, RGB is normalized to 0..255 then scaled by\n"
                "  brightness. Texture names in the info_* tables are case-insensitive.\n"
                "\n"
                "worldspawn\n"
                "  wad <paths>                    external WAD texture paths\n"
                "\n"
                "light (point light)\n"
                "  origin <x y z>                light position\n"
                "  _light <colour>               emitted colour / brightness\n"
                "  _fade <n>                     per-light distance falloff scale\n"
                "  _fast <0|1>                   gather this light through patches\n"
                "  style <0..63>                 emitted light style; negative is accepted\n"
                "  zhlt_stylecoring <n>          cutoff for this light's nonzero style\n"
                "  target <name>                 aim at a target entity (makes a spot)\n"
                "  angles / angle / pitch        direction when no target is used\n"
                "  _cone <degrees>               inner spot cone (default: 10)\n"
                "  _cone2 <degrees>              outer/fade cone (default: _cone)\n"
                "  _sky <n>                      treat this light as skylight\n"
                "\n"
                "light_spot\n"
                "  Uses all light keys. It is always directional; target takes priority\n"
                "  over angles / angle / pitch. _cone and _cone2 control its beam.\n"
                "\n"
                "light_environment\n"
                "  Uses the directional light keys and always lights sky surfaces.\n"
                "  _diffuse_light <colour>       primary diffuse sky light\n"
                "  _diffuse_light2 <colour>      secondary diffuse sky light\n"
                "  _spread <0..180>              angular sun spread / penumbra\n"
                "  pitch overrides angles pitch; target overrides the full direction.\n"
                "\n"
                "info_sunlight\n"
                "  Uses origin, _light, angles, pitch and target. It selects the sun\n"
                "  brightness/direction used for in-game studio-model lighting; use a\n"
                "  light_environment for the map's sky light.\n"
                "\n"
                "light_surface (also any light* entity carrying _tex)\n"
                "  _tex <texture>                texture whose matching faces emit\n"
                "  _light <colour>               emission, overriding lights.rad\n"
                "  origin <x y z>                selector position\n"
                "  _frange <n>                   maximum origin-to-face-centre range\n"
                "  _fdist <n>                    maximum distance from the face plane\n"
                "  _fclass <classname>           only faces on that entity classname\n"
                "  _fname <targetname>           only faces on that named entity\n"
                "  _texcolor <r g b>             override reflectivity colour (0..255)\n"
                "  _scale <n>                    emission multiplier; <=0 disables\n"
                "  _cone / _cone2 <degrees>      inner / outer surface emission cones\n"
                "  _chop <n>                     local emitting-patch size (minimum 1)\n"
                "  _texlightgap <n>              local texlight gap rejection\n"
                "  _fast <0|1|2>                 auto / force fast / force accurate\n"
                "  style <0..63>                 emitted light style\n"
                "  convertto <light classname>   runtime classname (must start 'light')\n"
                "  If several selectors match, the nearest origin wins.\n"
                "\n"
                "light_shadow\n"
                "  target <name>                 opaque brush entity to control\n"
                "  style <0..63>                 dynamic shadow style\n"
                "  convertto <light classname>   runtime classname (default: light)\n"
                "  The target needs zhlt_lightflags bit 2.\n"
                "\n"
                "light_bounce\n"
                "  target <name>                 brush entity whose bounce is styled\n"
                "  style <0..63>                 bounced-light style\n"
                "  convertto <light classname>   runtime classname (default: light)\n"
                "  An unnamed style-0 control leaves ordinary reflection enabled.\n"
                "\n"
                "texture table entities\n"
                "  Each key except classname/origin is a texture name. Its value is:\n"
                "  info_texlights    <colour>     emitted light (same syntax as lights.rad)\n"
                "  info_minlights    <0..1>       minimum light on that texture\n"
                "  info_unlittextures <nonzero>    bake white lightmaps (fullbright texture)\n"
                "  info_chopscale    <positive>   patch-size multiplier\n"
                "  info_smoothvalue  <degrees>    smoothing angle for that texture\n"
                "  info_translucent  <0..1|r g b> front/back light transmission blend\n"
                "  info_angularfade  <power [scale]> angular texlight falloff (>=0)\n"
                "\n"
                "brush / model entity lighting keys\n"
                "  _minlight <0..1>              minimum light on all model faces\n"
                "  style <0..63>                 surface-emission style for its face patches\n"
                "  light_origin <targetname>     light/shadow this model at that origin\n"
                "  model_center <x y z>          CSG-generated centre used by light_origin\n"
                "  zhlt_lightflags <bits>        RAD-only brush shadow flags; combine values\n"
                "    2 = faces cast shadows; 8 = do not treat its volume as solid\n"
                "  zhlt_customshadow <v|r g b>   light transmitted through opaque model\n"
                "  zhlt_striprad <0|1>           strip this model's face lightmaps\n"
                "  zhlt_copylight <targetname>   copy target-origin lighting to this model\n"
                "  zhlt_embedlightmap <0|1>      bake face lightmaps into new textures\n"
                "  zhlt_embedlightmapresolution <n> 1, 2, 4, 8 or 16 (default: 1)\n"
                "  Texture names beginning %%<0..255> also set a per-face minlight.\n"
                "\n"
                "studio model shadows\n"
                "  env_static and zhlt_studioshadow make a studiomodel block light.\n"
                "  zhlt_shadowmode 0 tests the triangle bounds, 1 tests each triangle\n"
                "  with masked skin support (default), 2 currently shares 1's test.\n");
        }
    }

    int run_rad_tool(int argc, char **argv)
    {
        cli::args command_line(argc, argv);
        const bool want_help = command_line.has("-h")
            || command_line.has("-help") || command_line.has("--help");
        if (command_line.empty() || command_line.map_name().empty() || want_help)
        {
            print_rad_help();
            return want_help ? 0 : 1;
        }

        std::string base = fs::strip_extension(
            fs::with_extension(command_line.map_name(), ".bsp"));
        logging::open_stage_log(base, "rad");
        logging::banner("rad");

        std::string bsp_path = base + ".bsp";
        format::map_data map;
        if (!format::bsp_file::load(bsp_path, map))
            err::fatal("could not load bsp '%s'", bsp_path.c_str());

        const tools::compile_parameters parameters =
            compile_parameters_from_bsp(map);
        staged_arguments arguments(
            argc, argv, parameters, cli::compiler_stage::rad);
        cli::args args(arguments.argc(), arguments.argv.data());

        if (args.has("-threads"))
            threads::set_count(args.int_value("-threads", 0));

        logging::setting("threads", std::to_string(threads::count()).c_str(), "varies",
                         args.has("-threads"));
        rad::rad_options options = parse_rad_options(
            args, arguments.argc(), arguments.argv.data(), base);
        logging::flush_settings();

        erase_runtime_compile_parameters(map);

        auto start = std::chrono::steady_clock::now();
        int alloc_block_pages = -1;
        bool lit = rad::run_rad(map, base, options, &alloc_block_pages);

        // rad is the last stage and the only consumer of the csg temp wad, so it is
        // no longer needed once lighting has read its textures
        std::remove((base + ".wa_").c_str());

        if (lit)
        {
            if (!args.has("-nochart"))
                format::print_usage_chart(map, false, alloc_block_pages);

            if (!format::bsp_file::write(bsp_path, map))
                err::fatal("could not write bsp '%s'", bsp_path.c_str());
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        logging::done(elapsed.count());
        logging::close();
        return lit ? 0 : 1;
    }
}
