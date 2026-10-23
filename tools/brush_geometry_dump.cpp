#include <fstream>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "csg/csg.h"
#include "csg/bsp_output.h"
#include "common/threads.h"
#include "format/bsp/file.h"

namespace
{
    void vector(std::ostream &out, const math::vec3v &v)
    {
        out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
    }
}

int main(int argc, char **argv)
{
    if (argc != 3 && argc != 6)
    {
        std::cerr << "usage: brush_geometry_dump <input.map> <output.json>"
                     " [--background <entity-number> <output-base>]\n";
        return 1;
    }
    int selected_entity = -1;
    if (argc == 6)
    {
        const std::string_view value(argv[4]);
        const auto parsed = std::from_chars(value.data(), value.data() + value.size(), selected_entity);
        if (std::string_view(argv[3]) != "--background" || parsed.ec != std::errc() ||
            parsed.ptr != value.data() + value.size() || selected_entity < 0)
        {
            std::cerr << "invalid background entity selection\n";
            return 1;
        }
    }
    threads::set_count(2);
    csg::csg_options options;
    options.brush.noclip = true;
    options.wad.map_path = argv[1];
    auto result = csg::run_csg(csg::load_map_file(argv[1]), options);
    std::ofstream out(argv[2]);
    if (!out) return 1;
    out << std::setprecision(17) << "{\"brushes\":[";
    bool first_brush = true;
    for (const auto &entity : result.entities)
        for (const auto &brush : entity.brushes)
        {
            if (brush.skipped || brush.hulls[0].faces.empty()) continue;
            if (!first_brush) out << ',';
            first_brush = false;
            out << "{\"entity\":" << brush.original_entity_num
                << ",\"brush\":" << brush.original_brush_num
                << ",\"contents\":" << (int)brush.contents << ",\"faces\":[";
            bool first_face = true;
            for (const auto &face : brush.hulls[0].faces)
            {
                if (!first_face) out << ',';
                first_face = false;
                const auto &plane = result.planes[(size_t)face.planenum];
                out << "{\"normal\":";
                vector(out, plane.normal);
                out << ",\"dist\":" << plane.dist << ",\"vertices\":[";
                bool first_point = true;
                for (const auto &point : face.winding.points())
                {
                    if (!first_point) out << ',';
                    first_point = false;
                    vector(out, point);
                }
                out << "]}";
            }
            out << "]}";
        }
    out << "]}\n";
    out.close();
    if (!out) return 1;
    if (argc == 6)
    {
        auto map = csg::load_map_file(argv[1]);
        if (selected_entity >= (int)map.entities.size() ||
            std::string_view(map.entities[selected_entity].value("classname")) != "func_detail")
        {
            std::cerr << "background selection must name a func_detail entity\n";
            return 1;
        }
        int selected_brushes = 0;
        for (auto &brush : map.brushes)
            if (brush.original_entity_num == selected_entity)
            {
                brush.noclip = 1;
                ++selected_brushes;
            }
        if (!selected_brushes) return 1;
        map.entities[selected_entity].set_value("zhlt_noclip", "1");
        for (auto &entity : map.entities)
        {
            const std::string_view classname(entity.value("classname"));
            if (classname.substr(0, 5) == "light" && !*entity.value("_light") && *entity.value("light"))
            {
                const std::string light(entity.value("light"));
                entity.set_value("_light", light.c_str());
                std::cout << "background lighting: converted light to _light\n";
            }
        }
        options.brush.noclip = false;
        auto background = csg::run_csg(std::move(map), options);
        if (!csg::write_intermediate_files(argv[5], background, options.brush)) return 1;
        auto bsp = csg::build_bsp_data(background);
        if (!format::bsp_file::write(std::string(argv[5]) + ".bsp", bsp)) return 1;
    }
    return 0;
}
