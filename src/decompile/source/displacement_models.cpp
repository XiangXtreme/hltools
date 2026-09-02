#include "displacement_models.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>
#include <unordered_map>

#include "displacements.h"
#include "materials.h"
#include "format/mdl/goldsrc/model.h"
#include "format/mdl/studio_model.h"
#include "format/miptex/texture.h"
#include "format/vbsp/data.h"
#include "math/vector.h"

namespace decompile
{
    namespace
    {
        using vec3 = math::vec3<double>;
        using position_key = std::tuple<double, double, double>;

        position_key key_of(const vec3 &p) { return {p.x, p.y, p.z}; }

        // gathered before anything is built, so normals average over the chunk
        struct chunk
        {
            std::vector<const source_displacement_triangle *> triangles;
            vec3 mins{1e30, 1e30, 1e30};
            vec3 maxs{-1e30, -1e30, -1e30};
        };

        void grow(chunk &c, const vec3 &p)
        {
            c.mins.x = std::min(c.mins.x, p.x);
            c.mins.y = std::min(c.mins.y, p.y);
            c.mins.z = std::min(c.mins.z, p.z);
            c.maxs.x = std::max(c.maxs.x, p.x);
            c.maxs.y = std::max(c.maxs.y, p.y);
            c.maxs.z = std::max(c.maxs.z, p.z);
        }

        vec3 triangle_normal(const source_displacement_triangle &t)
        {
            vec3 n = math::cross(t.points[1] - t.points[0],
                                 t.points[2] - t.points[0]);
            if (math::normalize(n) == 0)
                return t.base_normal;
            if (math::dot(n, t.base_normal) < 0)
                n = -n;
            return n;
        }
    }

    displacement_model_result build_displacement_models(
        const format::source_map_data &map, const material_catalog &catalog,
        const source_displacement_mesh &mesh, const std::string &map_name,
        double chunk_size)
    {
        displacement_model_result result;
        if (chunk_size < 1.0)
            chunk_size = 1.0;

        // the wad already holds these images, so skins are looked up, not decoded
        std::unordered_map<std::string, const format::mip_texture *> wad_textures;
        for (const format::mip_texture &texture : catalog.textures())
            wad_textures.emplace(texture.name, &texture);

        std::map<std::tuple<int, int, int>, chunk> chunks;
        for (const source_displacement_triangle &triangle : mesh.triangles)
        {
            vec3 centre = (triangle.points[0] + triangle.points[1]
                           + triangle.points[2]) * (1.0 / 3.0);
            std::tuple<int, int, int> cell{
                (int)std::floor(centre.x / chunk_size),
                (int)std::floor(centre.y / chunk_size),
                (int)std::floor(centre.z / chunk_size)};
            chunk &c = chunks[cell];
            c.triangles.push_back(&triangle);
            for (int k = 0; k < triangle.point_count; k++)
                grow(c, triangle.points[k]);
        }

        std::size_t index = 0;
        for (auto &entry : chunks)
        {
            chunk &c = entry.second;
            if (c.triangles.empty())
                continue;

            // the engine lights and culls from the origin, so author about the centre
            vec3 origin = (c.mins + c.maxs) * 0.5;

            // smooth across the chunk, or coarse rock reads as flat facets
            std::map<position_key, vec3> normals;
            for (const source_displacement_triangle *t : c.triangles)
            {
                vec3 n = triangle_normal(*t);
                for (int k = 0; k < t->point_count; k++)
                {
                    vec3 &sum = normals[key_of(t->points[k])];
                    sum = sum + n;
                }
            }

            format::studio_model model;
            char name[64];
            std::snprintf(name, sizeof(name), "terrain_%04zu", index);
            model.name = name;

            format::studio_bone bone;
            bone.name = "terrain";
            bone.parent = -1;
            model.bones.push_back(bone);

            // the renderer needs a sequence to draw through
            format::studio_sequence sequence;
            sequence.name = "idle";
            sequence.fps = 1;
            sequence.loop = true;
            sequence.frames.push_back({format::studio_bone_key{}});
            model.sequences.push_back(sequence);

            // one mesh per material, built on demand
            std::unordered_map<std::string, int> slot_of;
            std::map<std::tuple<double, double, double, float, float, int>, int>
                vertex_ids;
            bool skipped_skin = false;

            for (const source_displacement_triangle *t : c.triangles)
            {
                if (t->texinfo < 0
                    || (std::size_t)t->texinfo >= map.texinfo.size())
                    continue;
                const format::source_texinfo_t &ti =
                    map.texinfo[(std::size_t)t->texinfo];
                const std::string &material = map.texinfo_material(t->texinfo);
                if (material.empty())
                    continue;
                const resolved_material &rm = catalog.resolve(material);
                if (rm.special || !rm.has_texture)
                    continue;
                auto found = wad_textures.find(rm.name);
                if (found == wad_textures.end())
                {
                    skipped_skin = true;
                    continue;
                }

                auto slot = slot_of.find(rm.name);
                if (slot == slot_of.end())
                {
                    format::indexed_image image;
                    if (!format::decode_mip_image(*found->second, image))
                    {
                        skipped_skin = true;
                        continue;
                    }
                    format::studio_texture skin;
                    skin.name = rm.name + ".bmp";
                    if (rm.unlit)
                        skin.flags |= format::studio_nf_fullbright;
                    if (rm.masked)
                        skin.flags |= format::studio_nf_masked;
                    skin.image = std::move(image);
                    model.textures.push_back(std::move(skin));
                    model.materials.push_back(rm.name);
                    format::studio_mesh fresh;
                    fresh.material = (int)model.meshes.size();
                    model.meshes.push_back(fresh);
                    slot = slot_of.emplace(rm.name,
                                           (int)model.meshes.size() - 1).first;
                }
                const format::mip_texture &texture = *found->second;

                int ids[4] = {0, 0, 0, 0};
                for (int k = 0; k < t->point_count; k++)
                {
                    const vec3 &p = t->points[k];
                    // texture_vecs give texels directly
                    // the catalog's scales fold in the resize to goldsrc dimensions
                    double s = p.x * ti.texture_vecs[0][0]
                        + p.y * ti.texture_vecs[0][1]
                        + p.z * ti.texture_vecs[0][2] + ti.texture_vecs[0][3];
                    double t_coord = p.x * ti.texture_vecs[1][0]
                        + p.y * ti.texture_vecs[1][1]
                        + p.z * ti.texture_vecs[1][2] + ti.texture_vecs[1][3];
                    float u = (float)(s * rm.u_scale / (double)texture.width);
                    float v = (float)(t_coord * rm.v_scale
                                      / (double)texture.height);

                    std::tuple<double, double, double, float, float, int> vk{
                        p.x, p.y, p.z, u, v, slot->second};
                    auto vit = vertex_ids.find(vk);
                    if (vit == vertex_ids.end())
                    {
                        vec3 n = normals[key_of(p)];
                        if (math::normalize(n) == 0)
                            n = triangle_normal(*t);
                        format::studio_vertex vertex;
                        vertex.position[0] = (float)(p.x - origin.x);
                        vertex.position[1] = (float)(p.y - origin.y);
                        vertex.position[2] = (float)(p.z - origin.z);
                        vertex.normal[0] = (float)n.x;
                        vertex.normal[1] = (float)n.y;
                        vertex.normal[2] = (float)n.z;
                        vertex.u = u;
                        vertex.v = v;
                        vertex.bone = 0;
                        vit = vertex_ids
                                  .emplace(vk, (int)model.vertices.size())
                                  .first;
                        model.vertices.push_back(vertex);
                    }
                    ids[k] = vit->second;
                }
                // a fan covers the quad case, which would otherwise drop silently
                std::vector<int> &indices =
                    model.meshes[(std::size_t)slot->second].indices;
                for (int k = 2; k < t->point_count; k++)
                {
                    indices.push_back(ids[0]);
                    indices.push_back(ids[k - 1]);
                    indices.push_back(ids[k]);
                }
            }

            if (skipped_skin)
                result.missing_skins++;
            if (model.meshes.empty() || model.vertices.empty())
                continue;

            for (int k = 0; k < 3; k++)
            {
                model.bbmin[k] = (float)((&c.mins.x)[k] - (&origin.x)[k]);
                model.bbmax[k] = (float)((&c.maxs.x)[k] - (&origin.x)[k]);
            }

            converted_model out;
            out.path = map_name.empty()
                ? "models/" + model.name + ".mdl"
                : "models/" + map_name + "/" + model.name + ".mdl";
            std::string error;
            if (!format::write_goldsrc_model(model, out.data, &error))
            {
                result.failed++;
                continue;
            }

            prop_placement placement;
            placement.model = out.path;
            placement.origin[0] = origin.x;
            placement.origin[1] = origin.y;
            placement.origin[2] = origin.z;
            placement.solid = 0;

            result.triangles += model.triangle_count();
            result.vertices += model.vertices.size();
            result.skins += model.textures.size();
            result.models.push_back(std::move(out));
            result.placements.push_back(placement);
            result.chunks++;
            index++;
        }
        return result;
    }
}
