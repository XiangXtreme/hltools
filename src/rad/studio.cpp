#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "../common/filesystem.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "../format/mdl/goldsrc/read.h"
#include "internal.h"

// opaque studio model (mdl) shadows: models named by env_static entities or
// entities with zhlt_studioshadow block light like opaque brush entities.
//
// the reference builds an axis aligned tree over the posed triangles and traces
// them with three different tests selected by zhlt_shadowmode. the tree only
// changes the iteration order, so this port walks the triangle list directly
// and keeps the tolerance based hit test, which is what decides the lighting.

namespace rad
{
    namespace
    {
        constexpr int shadow_fast = 0;   // per triangle bounds only
        constexpr int shadow_normal = 1; // per triangle, alpha tested
        constexpr int shadow_slow = 2;   // simplified mesh, expanded facets

        constexpr vec_t bary_epsilon = 0.01f;
        constexpr vec_t coplanar_epsilon = 0.25f;
        constexpr vec_t near_shadow_epsilon = 1.5f;
        constexpr vec_t seam_epsilon = 0.0001f;
        constexpr vec_t trace_expand = 1.0f;

        struct placement
        {
            float m[3][4];
        };

        // the reference AngleMatrix for an entity, including the per axis
        // xform scale. angles are degrees and the caller already applied the
        // negative pitch quirk.
        placement placement_for(const vec3v &origin, const vec3v &angles,
                                const vec3v &scale)
        {
            const double to_radians = math::pi * 2.0 / 360.0;
            const double sy = std::sin((double)angles[1] * to_radians);
            const double cy = std::cos((double)angles[1] * to_radians);
            const double sp = std::sin((double)angles[0] * to_radians);
            const double cp = std::cos((double)angles[0] * to_radians);
            const double sr = std::sin((double)angles[2] * to_radians);
            const double cr = std::cos((double)angles[2] * to_radians);

            placement out{};
            out.m[0][0] = (float)(cp * cy * scale[0]);
            out.m[1][0] = (float)(cp * sy * scale[0]);
            out.m[2][0] = (float)(-sp * scale[0]);
            out.m[0][1] = (float)((sr * sp * cy + cr * -sy) * scale[1]);
            out.m[1][1] = (float)((sr * sp * sy + cr * cy) * scale[1]);
            out.m[2][1] = (float)(sr * cp * scale[1]);
            out.m[0][2] = (float)((cr * sp * cy + -sr * -sy) * scale[2]);
            out.m[1][2] = (float)((cr * sp * sy + -sr * cy) * scale[2]);
            out.m[2][2] = (float)(cr * cp * scale[2]);
            out.m[0][3] = origin.x;
            out.m[1][3] = origin.y;
            out.m[2][3] = origin.z;
            return out;
        }

        vec3v transform_point(const placement &p, const float in[3])
        {
            return vec3v{
                p.m[0][0] * in[0] + p.m[0][1] * in[1] + p.m[0][2] * in[2] + p.m[0][3],
                p.m[1][0] * in[0] + p.m[1][1] * in[1] + p.m[1][2] * in[2] + p.m[1][3],
                p.m[2][0] * in[0] + p.m[2][1] * in[1] + p.m[2][2] * in[2] + p.m[2][3]};
        }

        std::string normalize_slashes(std::string path)
        {
            for (char &c : path)
                if (c == '\\')
                    c = '/';
            return path;
        }

        bool has_model_extension(const std::string &path)
        {
            return path.size() >= 4 && str::iequals(path.c_str() + path.size() - 4, ".mdl");
        }

        // the reference resolves models against the game directory, the parent
        // of the map's directory, then falls back to the working directory. the
        // map directory itself and -waddir are accepted as extra convenience.
        std::vector<std::string> model_search_roots(const rad_state &state)
        {
            std::vector<std::string> roots;
            const std::string map_directory = fs::directory(state.base_path);
            if (!map_directory.empty())
            {
                const std::string game_directory = fs::directory(map_directory);
                if (!game_directory.empty() && game_directory != map_directory)
                    roots.push_back(game_directory);
                roots.push_back(map_directory);
            }
            roots.push_back(".");
            for (const std::string &folder : state.options.wad_folders)
                roots.push_back(folder);
            return roots;
        }

        bool load_model_bytes(const rad_state &state, const std::string &model_path,
                              std::vector<byte> &data, std::string &resolved)
        {
            const std::string relative = normalize_slashes(model_path);
            const std::vector<std::string> candidates = [&]() {
                std::vector<std::string> out;
                out.push_back(relative);
                if (!has_model_extension(relative))
                    out.push_back(relative + ".mdl");
                return out;
            }();

            for (const std::string &candidate : candidates)
            {
                if (fs::exists(candidate) && fs::read_all(candidate, data))
                {
                    resolved = candidate;
                    return true;
                }
                for (const std::string &root : model_search_roots(state))
                {
                    const std::string path = root + "/" + candidate;
                    if (fs::exists(path) && fs::read_all(path, data))
                    {
                        resolved = path;
                        return true;
                    }
                }
            }
            return false;
        }

        studio_shadow_model build_shadow_model(const format::studio_model &source,
                                               const placement &place, int trace_mode)
        {
            studio_shadow_model out;
            out.trace_mode = trace_mode;
            out.textures = source.textures;
            out.mins = vec3v{1e30f, 1e30f, 1e30f};
            out.maxs = vec3v{-1e30f, -1e30f, -1e30f};

            for (const format::studio_mesh &mesh : source.meshes)
            {
                for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3)
                {
                    studio_shadow_triangle triangle;
                    triangle.texture = mesh.material;
                    triangle.mins = vec3v{1e30f, 1e30f, 1e30f};
                    triangle.maxs = vec3v{-1e30f, -1e30f, -1e30f};
                    for (int k = 0; k < 3; k++)
                    {
                        const format::studio_vertex &vertex =
                            source.vertices[(std::size_t)mesh.indices[t + (std::size_t)k]];
                        triangle.p[k] = transform_point(place, vertex.position);
                        triangle.uv[k] = vec3v{vertex.u, vertex.v, 0.0f};
                        for (int axis = 0; axis < 3; axis++)
                        {
                            triangle.mins[axis] = std::min(triangle.mins[axis], triangle.p[k][axis]);
                            triangle.maxs[axis] = std::max(triangle.maxs[axis], triangle.p[k][axis]);
                            out.mins[axis] = std::min(out.mins[axis], triangle.p[k][axis]);
                            out.maxs[axis] = std::max(out.maxs[axis], triangle.p[k][axis]);
                        }
                    }
                    out.triangles.push_back(triangle);
                }
            }

            if (out.triangles.empty())
            {
                out.mins = vec3v{};
                out.maxs = vec3v{};
            }
            return out;
        }

        bool bounds_overlap(const vec3v &mins1, const vec3v &maxs1, const vec3v &mins2,
                            const vec3v &maxs2)
        {
            for (int i = 0; i < 3; i++)
                if (mins1[i] > maxs2[i] || maxs1[i] < mins2[i])
                    return false;
            return true;
        }

        void segment_bounds(const vec3v &p1, const vec3v &p2, vec3v &mins, vec3v &maxs)
        {
            for (int i = 0; i < 3; i++)
            {
                mins[i] = std::min(p1[i], p2[i]) - trace_expand;
                maxs[i] = std::max(p1[i], p2[i]) + trace_expand;
            }
        }

        // swept segment against an axis aligned box, used by the fast mode
        bool clip_ray_to_box(const vec3v &start, const vec3v &end, const vec3v &mins,
                             const vec3v &maxs)
        {
            vec_t enter = 0.0f;
            vec_t leave = 1.0f;
            for (int axis = 0; axis < 3; axis++)
            {
                const vec_t delta = end[axis] - start[axis];
                if (std::fabs(delta) < 1e-8f)
                {
                    if (start[axis] < mins[axis] || start[axis] > maxs[axis])
                        return false;
                    continue;
                }
                const vec_t inv = 1.0f / delta;
                vec_t t1 = (mins[axis] - start[axis]) * inv;
                vec_t t2 = (maxs[axis] - start[axis]) * inv;
                if (t1 > t2)
                    std::swap(t1, t2);
                enter = std::max(enter, t1);
                leave = std::min(leave, t2);
                if (enter > leave)
                    return false;
            }
            return leave >= 0.0f;
        }

        // shared masked texture test at a hit point. the palette's last index is
        // the punched out pixel for both goldsrc masked textures and the
        // reference's transparent flag.
        bool triangle_blocks(const studio_shadow_model &model,
                             const studio_shadow_triangle &triangle, vec_t u, vec_t v)
        {
            if (triangle.texture < 0
                || (std::size_t)triangle.texture >= model.textures.size())
                return true;
            const format::studio_texture &texture = model.textures[(std::size_t)triangle.texture];
            const unsigned flags = (unsigned)texture.flags;
            if (flags & (unsigned)format::studio_nf_additive)
                return false;
            if (!(flags & (unsigned)format::studio_nf_masked))
                return true;
            if (texture.image.pixels.empty() || texture.image.width == 0
                || texture.image.height == 0)
                return true;

            // avoid double shadows near triangle seams on alpha tested faces
            if (u < -seam_epsilon || u > 1.0f + seam_epsilon || v < -seam_epsilon
                || (u + v) > 1.0f + seam_epsilon)
                return false;

            const vec_t w = 1.0f - (u + v);
            const vec_t s = w * triangle.uv[0].x + u * triangle.uv[1].x
                + v * triangle.uv[2].x;
            const vec_t t = w * triangle.uv[0].y + u * triangle.uv[1].y
                + v * triangle.uv[2].y;
            const vec_t wrapped_s = (vec_t)(s - std::floor(s));
            const vec_t wrapped_t = (vec_t)(t - std::floor(t));
            int x = (int)(wrapped_s * (vec_t)texture.image.width);
            int y = (int)(wrapped_t * (vec_t)texture.image.height);
            x = std::max(0, std::min(x, (int)texture.image.width - 1));
            y = std::max(0, std::min(y, (int)texture.image.height - 1));
            const std::size_t at = (std::size_t)y * texture.image.width + (std::size_t)x;
            return texture.image.pixels[at] != 0xFF;
        }

        // the reference per triangle test: moeller trumbore with its wide
        // tolerances, a near plane so a surface does not shadow itself, and the
        // masked pixel check
        bool clip_ray_to_triangle(const studio_shadow_model &model,
                                  const studio_shadow_triangle &triangle, const vec3v &start,
                                  const vec3v &direction, vec_t distance)
        {
            const vec3v edge1 = triangle.p[1] - triangle.p[0];
            const vec3v edge2 = triangle.p[2] - triangle.p[0];
            const vec3v pvec = math::cross(direction, edge2);
            const vec_t det = math::dot(edge1, pvec);
            if (std::fabs(det) < coplanar_epsilon)
                return false;

            const vec_t inv_det = 1.0f / det;
            const vec3v tvec = start - triangle.p[0];
            const vec_t u = math::dot(tvec, pvec) * inv_det;
            if (u < -bary_epsilon || u > 1.0f + bary_epsilon)
                return false;

            const vec3v qvec = math::cross(tvec, edge1);
            const vec_t v = math::dot(direction, qvec) * inv_det;
            if (v < -bary_epsilon || (u + v) > 1.0f + bary_epsilon)
                return false;

            const vec_t depth = math::dot(edge2, qvec) * inv_det;
            if (depth <= near_shadow_epsilon || depth >= distance)
                return false;

            return triangle_blocks(model, triangle, u, v);
        }
    }

    void load_studio_models(rad_state &state)
    {
        state.studio_models.clear();
        state.num_studio_models = 0;

        if (!state.options.studioshadow)
            return;

        for (std::size_t i = 0; i < state.entities.size(); i++)
        {
            const format::entity &entity = state.entities[i];
            const char *classname = entity.value("classname");
            const bool env_static = str::iequals(classname, "env_static");

            if (env_static)
            {
                // spawnflag 4 turns the shadow off for this model
                if (int_for_key(entity, "spawnflags") & 4)
                    continue;
            }
            else if (!int_for_key(entity, "zhlt_studioshadow"))
            {
                continue;
            }

            const char *model_key = entity.value("model");
            if (!model_key || !*model_key)
                continue;

            vec3v origin, angles, scale;
            vector_for_key(entity, "origin", origin);
            vector_for_key(entity, "angles", angles);
            angles[0] = -angles[0]; // the engine's pitch convention

            vec3v xform;
            vector_for_key(entity, "xform", xform);
            const vec_t uniform = float_for_key(entity, "scale");
            if (xform == vec3v{})
                xform = vec3v{uniform, uniform, uniform};
            for (int axis = 0; axis < 3; axis++)
            {
                if (xform[axis] < 0.01f)
                    xform[axis] = 1.0f;
                if (xform[axis] > 16.0f)
                    xform[axis] = 16.0f;
            }

            int trace_mode = shadow_normal;
            const char *mode_value = entity.value("zhlt_shadowmode");
            if (mode_value && *mode_value)
                trace_mode = int_for_key(entity, "zhlt_shadowmode");
            if (trace_mode < shadow_fast || trace_mode > shadow_slow)
                trace_mode = shadow_normal;

            std::vector<byte> data;
            std::string resolved;
            if (!load_model_bytes(state, model_key, data, resolved))
            {
                logging::warn("studio shadow model '%s' not found", model_key);
                continue;
            }

            format::goldsrc_model_pose pose;
            pose.body = int_for_key(entity, "body");
            pose.skin = int_for_key(entity, "skin");
            format::studio_model neutral;
            std::string error;
            if (!format::read_goldsrc_model(data, pose, neutral, &error))
            {
                logging::warn("studio shadow model '%s': %s", resolved.c_str(),
                              error.c_str());
                continue;
            }

            const placement place = placement_for(origin, angles, xform);
            studio_shadow_model shadow = build_shadow_model(neutral, place, trace_mode);
            if (shadow.triangles.empty())
                continue;
            state.studio_models.push_back(std::move(shadow));
        }

        state.num_studio_models = (int)state.studio_models.size();
        if (state.num_studio_models)
        {
            logging::info("  %-14s %d opaque studio models\n", "studio",
                          state.num_studio_models);
        }
    }

    void free_studio_models(rad_state &state)
    {
        state.studio_models.clear();
        state.num_studio_models = 0;
    }

    bool test_segment_against_studio_model(const studio_shadow_model &model,
                                           const vec3v &p1, const vec3v &p2)
    {
        if (model.triangles.empty())
            return false;

        vec3v bounds_min, bounds_max;
        segment_bounds(p1, p2, bounds_min, bounds_max);
        if (!bounds_overlap(model.mins, model.maxs, bounds_min, bounds_max))
            return false;

        vec3v direction = p2 - p1;
        const vec_t distance = (vec_t)math::length(direction);
        if (distance <= 0.0f)
            return false;
        direction = direction * (1.0f / distance);

        for (const studio_shadow_triangle &triangle : model.triangles)
        {
            if (!bounds_overlap(triangle.mins, triangle.maxs, bounds_min, bounds_max))
                continue;
            if (model.trace_mode == shadow_fast)
            {
                if (clip_ray_to_box(p1, p2, triangle.mins, triangle.maxs))
                    return true;
            }
            else
            {
                // the reference's slow mode simplifies the mesh first and then
                // expands each facet by the trace width. without the
                // simplification pass the exact same triangles are the closest
                // match, so both modes share the face test for now.
                if (clip_ray_to_triangle(model, triangle, p1, direction, distance))
                    return true;
            }
        }
        return false;
    }

    bool test_segment_against_studio_list(const rad_state &state, const vec3v &p1,
                                          const vec3v &p2)
    {
        if (state.studio_models.empty())
            return false;
        for (const studio_shadow_model &model : state.studio_models)
            if (test_segment_against_studio_model(model, p1, p2))
                return true;
        return false;
    }
}
