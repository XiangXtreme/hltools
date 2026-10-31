#include "project_input.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <system_error>

#include "common/filesystem.h"
#include "format/tbproject/project.h"

namespace tools
{
    namespace
    {
        std::string lower(std::string text)
        {
            std::transform(text.begin(), text.end(), text.begin(),
                           [](unsigned char c) {
                               return (char)std::tolower(c);
                           });
            return text;
        }

        bool is_flag(const std::string &token)
        {
            return !token.empty() && token[0] == '-';
        }

        // returns the value of a "--name"/"-name" option, or nullptr
        const char *option_value(int argc, char **argv, const char *long_name,
                                 const char *short_name, int &index)
        {
            for (int i = 1; i + 1 < argc; i++)
            {
                const std::string token = argv[i];
                if (token == long_name || token == short_name)
                {
                    index = i;
                    return argv[i + 1];
                }
            }
            return nullptr;
        }

        std::string safe_file_stem(const std::string &name, int fallback)
        {
            std::string stem;
            for (const char c : name)
            {
                const bool allowed = (c >= 'a' && c <= 'z')
                    || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
                    || c == '_' || c == '-';
                stem += allowed ? c : '_';
            }
            if (stem.empty())
                stem = "asset" + std::to_string(fallback);
            return stem;
        }

        bool has_magic(const std::vector<byte> &data, const char *magic)
        {
            return data.size() >= 4
                && data[0] == (byte)magic[0] && data[1] == (byte)magic[1]
                && data[2] == (byte)magic[2] && data[3] == (byte)magic[3];
        }

        std::string unique_directory()
        {
            const auto now = std::chrono::high_resolution_clock::now()
                                 .time_since_epoch()
                                 .count();
            static int counter = 0;
            return (std::filesystem::temp_directory_path()
                    / ("hltools-tbproj-" + std::to_string(now) + "-"
                       + std::to_string(counter++)))
                .string();
        }

    }

    bool project_compile_requested(int argc, char **argv)
    {
        int index = 0;
        return option_value(argc, argv, "--project", "-project", index) != nullptr;
    }

    bool expand_project_compile(int argc, char **argv, project_input &input,
                                std::string *error)
    {
        int project_index = 0;
        int map_index = 0;
        int output_index = 0;
        const char *project_path =
            option_value(argc, argv, "--project", "-project", project_index);
        const char *map_name =
            option_value(argc, argv, "--map", "-map", map_index);
        const char *output =
            option_value(argc, argv, "--output", "-output", output_index);

        if (!project_path)
            return false;
        if (!map_name)
        {
            if (error)
                *error = "--project requires --map <name>";
            return false;
        }
        int wad_config_index = 0;
        if (option_value(argc, argv, "-wadcfgfile", "-wadcfgfile", wad_config_index))
        {
            if (error)
                *error = "--project builds its own wad list; do not pass -wadcfgfile";
            return false;
        }

        auto project = format::tbproject::project::open(project_path, error);
        if (!project)
            return false;

        const format::tbproject::asset *map = project->find_map(map_name);
        if (!map)
        {
            std::string names;
            for (const auto &entry : project->assets())
            {
                if (lower(entry.type) != "map")
                    continue;
                if (!names.empty())
                    names += ", ";
                names += entry.display_name;
            }
            if (error)
                *error = std::string("project has no map named '") + map_name
                    + "' (maps: " + (names.empty() ? "none" : names) + ")";
            return false;
        }

        const std::string map_stem =
            std::filesystem::path(map->virtual_path).stem().string();
        const std::string map_file = map_stem.empty() ? std::string("map") : map_stem;

        input.working_directory = unique_directory();
        if (!fs::make_directory(input.working_directory + "/textures"))
        {
            if (error)
                *error = "could not create temporary directory "
                    + input.working_directory;
            return false;
        }

        const std::string map_path =
            (std::filesystem::path(input.working_directory)
             / (map_file + ".map"))
                .string();
        if (map->compatible.empty()
            || !fs::write_all(map_path, map->compatible.data(),
                              map->compatible.size()))
        {
            if (error)
                *error = "could not write the map source extracted from the project";
            return false;
        }

        std::string wad_list;
        int texture_index = 0;
        for (const auto &entry : project->assets())
        {
            if (lower(entry.type) != "texture" || entry.compatible.empty())
                continue;
            texture_index++;
            if (!has_magic(entry.compatible, "WAD3"))
            {
                if (error)
                    *error = "texture '" + entry.display_name
                        + "' is not a compiled wad; re-import or convert it in "
                          "TrenchBroom";
                return false;
            }
            const std::string wad_path =
                (std::filesystem::path(input.working_directory) / "textures"
                 / (safe_file_stem(entry.display_name, texture_index) + ".wad"))
                    .string();
            if (!fs::write_all(wad_path, entry.compatible.data(),
                               entry.compatible.size()))
            {
                if (error)
                    *error = "could not write texture wad " + wad_path;
                return false;
            }
            wad_list += wad_path + "\n";
        }

        const std::string wad_config =
            (std::filesystem::path(input.working_directory) / "wads.cfg").string();
        if (!fs::write_all(wad_config, wad_list.data(), wad_list.size()))
        {
            if (error)
                *error = "could not write the wad list for the project";
            return false;
        }

        // keep everything the caller passed except the project options, then add
        // the wad list and the extracted map last so the map is the final
        // positional argument
        input.arguments.clear();
        input.arguments.push_back(argv[0]);
        for (int i = 1; i < argc; i++)
        {
            if (i == project_index || i == project_index + 1 || i == map_index
                || i == map_index + 1
                || (output && (i == output_index || i == output_index + 1)))
                continue;
            input.arguments.push_back(argv[i]);
        }
        input.arguments.push_back("-wadcfgfile");
        input.arguments.push_back(wad_config);
        input.arguments.push_back(map_path);

        input.bsp_path = (std::filesystem::path(input.working_directory)
                          / (map_file + ".bsp"))
                             .string();
        if (output && *output)
        {
            input.output_path = output;
        }
        else
        {
            input.output_path =
                (std::filesystem::path(project_path).parent_path()
                 / (map_file + ".bsp"))
                    .string();
        }
        return true;
    }

    void discard_project_input(project_input &input)
    {
        if (input.working_directory.empty())
            return;
        std::error_code ignored;
        std::filesystem::remove_all(input.working_directory, ignored);
        input.working_directory.clear();
    }
}
