#include "entities.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

#include "format/bsp/entity_lump.h"

namespace decompile
{
    namespace
    {
        bool iequals(const char *a, const char *b)
        {
            for (; *a && *b; a++, b++)
                if (std::tolower((unsigned char)*a) != std::tolower((unsigned char)*b))
                    return false;
            return *a == *b;
        }

        bool is_source_team_spawn(const char *classname)
        {
            return iequals(classname, "info_player_terrorist")
                || iequals(classname, "info_player_counterterrorist");
        }

        // source's prop family all render a studio model at a point. goldsrc's
        // nearest equivalent is cycler_sprite, which draws a model without being
        // solid: every prop in the maps this targets is non solid anyway, and
        // goldsrc takes collision from brushes rather than from models.
        // source's func_brush is a general purpose brush entity whose solidity
        // is a keyvalue; goldsrc splits that across two classnames instead.
        // solidity 1 is "never solid", 0 ("toggle", solid while enabled) and 2
        // ("always solid") both end up solid.
        bool is_source_brush_entity(const char *classname)
        {
            return iequals(classname, "func_brush")
                || iequals(classname, "func_wall_toggle");
        }

        // source stores feet, goldsrc the hull centre (-VEC_HULL_MIN.z = 36)
        // not for info_teleport_destination, TeleportTouch subtracts mins.z itself
        void raise_spawn_to_hull_centre(format::entity &entity)
        {
            const char *origin = entity.value("origin");
            if (origin[0] == '\0')
                return;
            double x = 0, y = 0, z = 0;
            if (std::sscanf(origin, "%lf %lf %lf", &x, &y, &z) != 3)
                return;
            char raised[96];
            std::snprintf(raised, sizeof(raised), "%.6g %.6g %.6g", x, y, z + 36);
            entity.set("origin", raised);
        }

        bool is_studio_prop(const char *classname)
        {
            return iequals(classname, "prop_dynamic")
                || iequals(classname, "prop_dynamic_override")
                || iequals(classname, "prop_static")
                || iequals(classname, "prop_physics")
                || iequals(classname, "prop_physics_multiplayer")
                || iequals(classname, "prop_physics_override")
                || iequals(classname, "prop_ragdoll");
        }

        // keys the goldsrc engine and this compiler both ignore. they come from
        // hammer bookkeeping, source-only rendering controls, and the source
        // light attenuation model, which goldsrc replaces with a fixed falloff.
        // stripping them keeps the ported entity block readable and small enough
        // to see the keys that do still matter.
        const char *const dead_keys[] = {
            // hammer / vbsp bookkeeping
            "hammerid", "StartDisabled", "InputFilter", "invert_exclusion",
            "solidbsp", "PerformanceMode",
            // source-only shadow and render controls. note hlrad's own "_fade"
            // is a different key from source's prop fade distances below.
            "vrad_brush_cast_shadows", "disableshadows", "disablereceiveshadows",
            "disableflashlight", "shadowcastdist", "screenspacefade",
            "disableshadowdepth", "shadowdepthnocache", "drawinfastreflection",
            "fademindist", "fademaxdist", "fadescale",
            "mindxlevel", "maxdxlevel", "MinDXLevel", "MaxDXLevel",
            "mincpulevel", "maxcpulevel", "mingpulevel", "maxgpulevel",
            // source hdr lighting, inert outside vrad
            "_lightHDR", "_lightscaleHDR", "_ambientHDR", "_AmbientScaleHDR",
            "_diffuse_lightHDR",
            // source's per light attenuation model. hlrad uses a fixed inverse
            // square falloff and reads none of these.
            "_quadratic_attn", "_linear_attn", "_constant_attn", "_distance",
            "_fifty_percent_distance", "_zero_percent_distance",
            // physics properties with no goldsrc analogue
            "physdamagescale", "nodamageforces", "massScale", "inertiaScale",
            // source-only trigger controls; goldsrc's CBaseTrigger reads
            // neither, and a teleport there always uses the destination's
            // own angles
            "CheckDestIfClearForPlayer", "UseLandmarkAngles",
        };

        void strip_dead_keys(format::entity &entity)
        {
            for (const char *key : dead_keys)
                entity.remove(key);
        }

        // moves a key to the name goldsrc knows it by, keeping the value. a key
        // already present under the target name wins, since it was written for
        // goldsrc deliberately.
        bool rename_key(format::entity &entity, const char *from, const char *to)
        {
            const char *value = entity.value(from);
            if (value[0] == '\0')
                return false;
            std::string carried = value;
            entity.remove(from);
            if (entity.value(to)[0] == '\0')
                entity.set(to, carried.c_str());
            return true;
        }

        // source's fog lives on env_fog_controller and is read by the client's
        // material system; goldsrc/cs1.6 has a working env_fog taking the same
        // three quantities under different names. the keys with no equivalent
        // (the second blend colour, the blend direction and lerp, and the source
        // specific far z) are dropped rather than approximated.
        void convert_fog(format::entity &entity)
        {
            entity.set("classname", "env_fog");
            rename_key(entity, "fogcolor", "rendercolor");
            rename_key(entity, "fogstart", "startdist");
            std::string fog_end = entity.value("fogend");
            rename_key(entity, "fogend", "enddist");

            // not a rename: source's "fogmaxdensity" is a 0..1 fraction capping
            // how opaque the fog may become, while goldsrc's "density" is the
            // per-unit coefficient of an exponential falloff. copying the number
            // across gives density 1, which produces a black screen within a
            // unit. exponential fog is ~95% opaque at density*d == 3, so
            // derive the coefficient from the distance at which source said the
            // fog was full instead.
            entity.remove("fogmaxdensity");
            if (entity.value("density")[0] == '\0')
            {
                double end = std::atof(fog_end.c_str());
                double density = end > 1.0 ? 3.0 / end : 0.001;
                // the range every working goldsrc map on record sits in
                if (density < 0.0001)
                    density = 0.0001;
                if (density > 0.01)
                    density = 0.01;
                char text[32];
                std::snprintf(text, sizeof(text), "%.5g", density);
                entity.append("density", text);
            }
            entity.remove("fogcolor2");
            entity.remove("fogdir");
            entity.remove("fogblend");
            entity.remove("foglerptime");
            entity.remove("fogenable");
            entity.remove("farz");
            entity.remove("use_angles");
            entity.remove("angles");
        }

        // vrad lights with the linear colour, hlrad with the raw ratios
        // without this, coloured light desaturates towards white
        double srgb_channel_to_linear(double channel)
        {
            if (channel <= 0.0)
                return 0.0;
            return std::pow(channel / 255.0, 2.2) * 255.0;
        }

        // converts the channels of "R G B [brightness]", not the brightness
        // the 1 value form is greyscale shorthand, so its number is a colour
        // anything not 1, 3 or 4 numbers is left for hlrad to reject
        bool convert_light_colour(format::entity &entity, const char *key)
        {
            const char *value = entity.value(key);
            if (value[0] == '\0')
                return false;

            double parsed[4] = {0, 0, 0, 0};
            int count = std::sscanf(value, "%lf %lf %lf %lf", &parsed[0],
                                    &parsed[1], &parsed[2], &parsed[3]);
            if (count != 1 && count != 3 && count != 4)
                return false;

            int channels = (count == 1) ? 1 : 3;
            char rewritten[128];
            int written = 0;
            for (int i = 0; i < count; i++)
            {
                double out = (i < channels) ? srgb_channel_to_linear(parsed[i])
                                            : parsed[i];
                // colours are whole numbers; brightness keeps its precision
                int room = (int)sizeof(rewritten) - written;
                int step = (i < channels)
                    ? std::snprintf(rewritten + written, (size_t)room, "%s%d",
                                    i ? " " : "", (int)(out + 0.5))
                    : std::snprintf(rewritten + written, (size_t)room, " %g", out);
                if (step <= 0 || step >= room)
                    return false;
                written += step;
            }
            entity.set(key, rewritten);
            return true;
        }

        // hlrad reads _diffuse_light and _spread; source writes the same two
        // quantities as _ambient and SunSpreadAngle. without the rename the sky
        // fill silently falls back to the sun's own colour and brightness, and
        // the sun becomes a mathematical point with razor sharp shadow edges.
        // brightness itself is left alone: it is a per map tuning decision, and
        // rescaling it here would quietly change every already ported map.
        void convert_light_keys(format::entity &entity, bool colour_gamma)
        {
            rename_key(entity, "_ambient", "_diffuse_light");
            rename_key(entity, "SunSpreadAngle", "_spread");
            if (!colour_gamma)
                return;
            // after the rename these two cover _ambient as well
            convert_light_colour(entity, "_light");
            convert_light_colour(entity, "_diffuse_light");
        }
    }

    void remap_source_entities(std::vector<format::entity> &entities,
                               entity_remap_stats &stats, bool light_colour_gamma)
    {
        std::vector<format::entity> kept;
        kept.reserve(entities.size());

        // prefer an existing generic start. otherwise select the first suitable
        // team-specific start and discard the rest: the port only needs one
        // deterministic entry point.
        bool existing_start = false;
        bool any_ct_spawn = false;
        for (const format::entity &entity : entities)
        {
            const char *classname = entity.value("classname");
            if (iequals(classname, "info_player_start"))
                existing_start = true;
            else if (iequals(classname, "info_player_counterterrorist"))
                any_ct_spawn = true;
        }
        // goldsrc's info_player_start is the CT spawn, so prefer a CT donor
        // a t-only map still has to yield one
        bool start_written = false;

        for (format::entity &entity : entities)
        {
            strip_dead_keys(entity);

            const char *classname = entity.value("classname");
            if (iequals(classname, "info_player_start"))
            {
                if (start_written)
                {
                    stats.dropped_spawns++;
                    continue;
                }
                raise_spawn_to_hull_centre(entity);
                start_written = true;
            }
            else if (is_source_team_spawn(classname))
            {
                bool suitable = !any_ct_spawn
                             || iequals(classname, "info_player_counterterrorist");
                if (existing_start || start_written || !suitable)
                {
                    stats.dropped_spawns++;
                    continue;
                }
                entity.set("classname", "info_player_start");
                raise_spawn_to_hull_centre(entity);
                start_written = true;
            }
            else if (is_source_brush_entity(classname))
            {
                bool solid = std::strcmp(entity.value("Solidity"), "1") != 0;
                entity.set("classname", solid ? "func_wall" : "func_illusionary");
                // solidity is now carried by the classname itself
                entity.remove("Solidity");
                stats.brush_entities++;
            }
            else if (is_studio_prop(classname))
            {
                entity.set("classname", "cycler_sprite");
                if (entity.value("framerate")[0] == '\0')
                    entity.append("framerate", "10");
                stats.studio_props++;
            }
            else if (iequals(classname, "env_fog_controller"))
            {
                convert_fog(entity);
                stats.fog_controllers++;
            }
            else if (iequals(classname, "point_viewcontrol"))
            {
                // source renamed goldsrc's trigger_camera; the keys goldsrc reads
                // (targetname, target, wait, speed, moveto) keep their names, the
                // source-only motion controls do not exist.
                entity.set("classname", "trigger_camera");
                entity.remove("acceleration");
                entity.remove("deceleration");
                entity.remove("interpolatepositiontoplayer");
                stats.cameras++;
            }
            else if (std::strncmp(classname, "light", 5) == 0)
            {
                convert_light_keys(entity, light_colour_gamma);
                stats.lights++;
            }
            kept.push_back(std::move(entity));
        }

        entities.swap(kept);
        stats.player_start = start_written;
    }
}
