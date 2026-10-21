#include "inspect_tool.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "../../common/filesystem.h"
#include "../../common/log.h"
#include "../../common/string_util.h"
#include "format/bsp/data.h"
#include "format/bsp/entity_lump.h"
#include "format/bsp/file.h"
#include "format/bsp/hull_geometry.h"
#include "format/bsp/texture_lump.h"
#include "format/mdl/goldsrc/read.h"

namespace tools
{
    namespace
    {
        void print_inspect_help()
        {
            logging::console(
                "usage\n"
                "  hltools inspect <file[.bsp|.mdl]> [-obj <out.obj>]\n"
                "\n"
                "  reports a compiled bsp's lumps, geometry and texture usage, or\n"
                "  a goldsrc studio model's bones, sequences, skins and meshes.\n"
                "  -obj exports hull 0 polygons (bsp) or frame 0 triangles (mdl) as\n"
                "  wavefront obj. read-only.\n");
        }

        bool write_obj(const std::string &path, const std::string &contents)
        {
            return fs::write_all(path, contents.data(), contents.size());
        }

        void append_vec(std::string &out, const char *prefix, float x, float y, float z)
        {
            char line[96];
            std::snprintf(line, sizeof(line), "%s %.6f %.6f %.6f\n", prefix,
                          (double)x, (double)y, (double)z);
            out += line;
        }

        int inspect_bsp(const std::string &path, const std::string &obj_path)
        {
            format::map_data map;
            if (!format::bsp_file::load(path, map))
            {
                logging::console("inspect: could not load '%s'\n", path.c_str());
                return 1;
            }

            const std::vector<format::entity> entities = format::parse_entities(map.entities);
            int brush_models = 0;
            std::string wad_value;
            for (const format::entity &entity : entities)
            {
                if (entity.value("model")[0] == '*')
                    brush_models++;
                if (wad_value.empty() && str::iequals(entity.value("classname"), "worldspawn"))
                    wad_value = entity.value("wad");
            }

            std::vector<format::mip_texture> textures;
            std::vector<std::string> external;
            std::string error;
            if (!format::collect_bsp_textures(map, textures, &external, &error))
            {
                logging::console("inspect: %s\n", error.c_str());
                return 1;
            }

            // face count per texture name, for the usage list
            const std::vector<std::string> texture_names =
                format::bsp_texture_names(map.textures);
            std::map<std::string, int> texture_faces;
            for (const format::dface_t &face : map.faces)
            {
                if (face.texinfo < 0 || (std::size_t)face.texinfo >= map.texinfo.size())
                    continue;
                const int index = map.texinfo[(std::size_t)face.texinfo].miptex;
                if (index < 0 || (std::size_t)index >= texture_names.size())
                    continue;
                texture_faces[texture_names[(std::size_t)index]]++;
            }

            logging::console("bsp  %s\n", path.c_str());
            logging::console("  version       %d (goldsrc)\n", format::bsp_version);
            logging::console("  entities      %d (%d brush models)\n", (int)entities.size(),
                             brush_models);
            logging::console("  models        %d\n", (int)map.models.size());
            logging::console("  planes        %d\n", (int)map.planes.size());
            logging::console("  vertexes      %d\n", (int)map.vertexes.size());
            logging::console("  nodes         %d\n", (int)map.nodes.size());
            logging::console("  leaves        %d\n", (int)map.leafs.size());
            logging::console("  faces         %d\n", (int)map.faces.size());
            logging::console("  clipnodes     %d\n", (int)map.clipnodes.size());
            logging::console("  textures      %d embedded, %d external\n", (int)textures.size(),
                             (int)external.size());
            logging::console("  texdata       %.2f MB\n",
                             (double)map.textures.size() / (1024.0 * 1024.0));
            logging::console("  lighting      %.2f MB\n",
                             (double)map.lighting.size() / (1024.0 * 1024.0));
            if (!wad_value.empty())
                logging::console("  wad           %s\n", wad_value.c_str());

            if (!texture_faces.empty())
            {
                std::vector<std::pair<std::string, int>> ordered(texture_faces.begin(),
                                                                 texture_faces.end());
                std::sort(ordered.begin(), ordered.end(),
                          [](const auto &a, const auto &b) { return a.second > b.second; });
                logging::console("\n  top textures by face count\n");
                for (std::size_t i = 0; i < ordered.size() && i < 10; i++)
                    logging::console("    %-15s %6d\n", ordered[i].first.c_str(),
                                     ordered[i].second);
            }

            if (!obj_path.empty())
            {
                std::string obj = "# hltools inspect hull 0\n";
                int emitted = 0;
                for (int model = 0; model < (int)map.models.size(); model++)
                {
                    std::vector<format::hull0_face> faces;
                    if (!format::collect_hull0_faces(map, model, faces, &error))
                    {
                        logging::console("inspect: model %d: %s\n", model, error.c_str());
                        return 1;
                    }
                    if (faces.empty())
                        continue;
                    char header[64];
                    std::snprintf(header, sizeof(header), "o model_%d\n", model);
                    obj += header;
                    const int base = emitted + 1;
                    for (const format::hull0_face &face : faces)
                        for (const auto &point : face.points)
                            append_vec(obj, "v", point[0], point[1], point[2]);
                    int vertex = base;
                    for (const format::hull0_face &face : faces)
                    {
                        obj += "f";
                        for (std::size_t i = 0; i < face.points.size(); i++)
                        {
                            char index[16];
                            std::snprintf(index, sizeof(index), " %d", vertex + (int)i);
                            obj += index;
                        }
                        obj += "\n";
                        vertex += (int)face.points.size();
                    }
                    emitted = vertex - 1;
                }
                if (emitted == 0)
                {
                    logging::console("inspect: '%s' has no hull 0 faces\n", path.c_str());
                    return 1;
                }
                if (!write_obj(obj_path, obj))
                {
                    logging::console("inspect: could not write '%s'\n", obj_path.c_str());
                    return 1;
                }
                logging::console("\n  wrote %d vertices to '%s'\n", emitted, obj_path.c_str());
            }
            return 0;
        }

        int inspect_mdl(const std::string &path, const std::string &obj_path)
        {
            std::vector<byte> bytes;
            if (!fs::read_all(path, bytes))
            {
                logging::console("inspect: could not read '%s'\n", path.c_str());
                return 1;
            }

            format::goldsrc_model_pose pose;
            format::studio_model model;
            std::string error;
            if (!format::read_goldsrc_model(bytes, pose, model, &error))
            {
                logging::console("inspect: %s: %s\n", path.c_str(), error.c_str());
                return 1;
            }

            int triangles = 0;
            for (const format::studio_mesh &mesh : model.meshes)
                triangles += (int)mesh.indices.size() / 3;

            logging::console("mdl  %s\n", path.c_str());
            logging::console("  name          %s\n", model.name.c_str());
            logging::console("  bones         %d\n", (int)model.bones.size());
            logging::console("  sequences     %d\n", (int)model.sequences.size());
            logging::console("  textures      %d\n", (int)model.textures.size());
            logging::console("  meshes        %d\n", (int)model.meshes.size());
            logging::console("  vertices      %d\n", (int)model.vertices.size());
            logging::console("  triangles     %d\n", triangles);
            logging::console("  bounds        (%.2f %.2f %.2f) - (%.2f %.2f %.2f)\n",
                             (double)model.bbmin[0], (double)model.bbmin[1],
                             (double)model.bbmin[2], (double)model.bbmax[0],
                             (double)model.bbmax[1], (double)model.bbmax[2]);

            if (!model.sequences.empty())
            {
                logging::console("\n  sequences\n");
                for (std::size_t i = 0; i < model.sequences.size() && i < 8; i++)
                    logging::console("    %-20s %d frames\n",
                                     model.sequences[i].name.c_str(),
                                     (int)model.sequences[i].frames.size());
            }
            if (!model.textures.empty())
            {
                logging::console("\n  skins\n");
                for (std::size_t i = 0; i < model.textures.size() && i < 16; i++)
                {
                    const format::studio_texture &texture = model.textures[i];
                    logging::console("    %-20s %ux%u flags=0x%08x\n",
                                     texture.name.c_str(), texture.image.width,
                                     texture.image.height, (unsigned)texture.flags);
                }
            }

            if (!obj_path.empty())
            {
                std::string obj = "# hltools inspect mdl\n";
                for (const format::studio_vertex &vertex : model.vertices)
                    append_vec(obj, "v", vertex.position[0], vertex.position[1],
                               vertex.position[2]);
                for (const format::studio_mesh &mesh : model.meshes)
                {
                    if (mesh.material >= 0 && (std::size_t)mesh.material < model.textures.size())
                    {
                        obj += "usemtl ";
                        obj += model.textures[(std::size_t)mesh.material].name;
                        obj += "\n";
                    }
                    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
                    {
                        char line[96];
                        std::snprintf(line, sizeof(line), "f %d %d %d\n",
                                      (int)mesh.indices[i] + 1,
                                      (int)mesh.indices[i + 1] + 1,
                                      (int)mesh.indices[i + 2] + 1);
                        obj += line;
                    }
                }
                if (!write_obj(obj_path, obj))
                {
                    logging::console("inspect: could not write '%s'\n", obj_path.c_str());
                    return 1;
                }
                logging::console("\n  wrote %d triangles to '%s'\n", triangles,
                                 obj_path.c_str());
            }
            return 0;
        }
    }

    int run_inspect_tool(int argc, char **argv)
    {
        std::string target;
        std::string obj_path;
        bool help = false;
        for (int i = 1; i < argc; i++)
        {
            const std::string token = argv[i];
            if (str::iequals(token.c_str(), "-obj"))
            {
                if (i + 1 < argc)
                    obj_path = argv[++i];
            }
            else if (str::iequals(token.c_str(), "-h") || str::iequals(token.c_str(), "-help")
                     || str::iequals(token.c_str(), "--help"))
                help = true;
            else if (!token.empty() && token[0] != '-')
                target = token;
        }

        if (help || target.empty())
        {
            print_inspect_help();
            return help ? 0 : 1;
        }

        std::string lower = target;
        for (char &c : lower)
            c = (char)std::tolower((unsigned char)c);
        const bool mdl = lower.size() >= 4 && lower.compare(lower.size() - 4, 4, ".mdl") == 0;
        const std::string path = mdl ? target : fs::with_extension(target, ".bsp");
        if (!fs::exists(path))
        {
            logging::console("inspect: '%s' does not exist\n", path.c_str());
            return 1;
        }
        return mdl ? inspect_mdl(path, obj_path) : inspect_bsp(path, obj_path);
    }
}
