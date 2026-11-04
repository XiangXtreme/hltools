#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <sqlite3.h>

#include "cli/hltools/project_input.h"
#include "common/filesystem.h"
#include "format/bmp/image.h"
#include "format/miptex/texture.h"
#include "format/wad/archive.h"
#include "support/test.h"

namespace
{
    void exec(sqlite3 *db, const char *sql)
    {
        char *message = nullptr;
        if (sqlite3_exec(db, sql, nullptr, nullptr, &message) != SQLITE_OK)
        {
            const std::string text = message ? message : "sqlite error";
            sqlite3_free(message);
            throw std::runtime_error(text);
        }
    }

    void bind_blob(sqlite3 *db, const char *sql, const std::vector<byte> &value)
    {
        sqlite3_stmt *statement = nullptr;
        require(sqlite3_prepare_v2(db, sql, -1, &statement, nullptr) == SQLITE_OK);
        require(sqlite3_bind_blob(statement, 1, value.data(), (int)value.size(),
                                  SQLITE_TRANSIENT) == SQLITE_OK);
        require(sqlite3_step(statement) == SQLITE_DONE);
        sqlite3_finalize(statement);
    }

    struct fixture
    {
        std::filesystem::path root;
        std::filesystem::path project;
        const std::string compiled_map = "// compiled terrain\n";

        explicit fixture(const bool conflict)
        {
            root = std::filesystem::path(HLTOOLS_TEST_SCRATCH_DIRECTORY)
                / ("project-input-" + std::to_string(
                    std::chrono::high_resolution_clock::now().time_since_epoch().count()));
            std::filesystem::create_directories(root);
            project = root / "terrain.tbproj";

            format::indexed_image image;
            image.width = image.height = 16;
            image.pixels.resize(256, 1);
            format::mip_texture terrain_texture;
            format::mip_texture base_texture;
            std::string error;
            require(format::build_mip_texture("TBT0_0", image, terrain_texture, &error));
            require(format::build_mip_texture(conflict ? "tbt0_0" : "BASE", image,
                                              base_texture, &error));
            const auto terrain_path = root / "generated.wad";
            const auto base_path = root / "base.wad";
            require(format::write_wad3(terrain_path.string(), {terrain_texture}, &error));
            require(format::write_wad3(base_path.string(), {base_texture}, &error));
            std::vector<byte> terrain_wad;
            std::vector<byte> base_wad;
            require(fs::read_all(terrain_path.string(), terrain_wad));
            require(fs::read_all(base_path.string(), base_wad));

            sqlite3 *db = nullptr;
            require(sqlite3_open(project.string().c_str(), &db) == SQLITE_OK);
            exec(db,
                 "PRAGMA application_id=0x5442504a; PRAGMA user_version=4;"
                 "CREATE TABLE project(name TEXT,game_id TEXT);"
                 "INSERT INTO project VALUES('Terrain','Half-Life');"
                 "CREATE TABLE content(id TEXT PRIMARY KEY,data BLOB);"
                 "INSERT INTO content VALUES"
                 "('map',CAST('\"_tb_terrain_paint\" \"paint\"' AS BLOB)),"
                 "('texture',X'');"
                 "CREATE TABLE revisions(asset_id TEXT,revision INTEGER,type TEXT,"
                 "virtual_path TEXT,publish_path TEXT,display_name TEXT,compatible TEXT);"
                 "INSERT INTO revisions VALUES"
                 "('m',1,'map','maps/terrain.map','maps/terrain.map','Terrain','map'),"
                 "('t',1,'texture','textures/base.wad','textures/base.wad','BASE','texture');"
                 "CREATE TABLE heads(asset_id TEXT,revision INTEGER);"
                 "INSERT INTO heads VALUES('m',1),('t',1);"
                 "CREATE TABLE terrain_compile(asset_id TEXT,source_revision INTEGER,"
                 "generator_version INTEGER,top_faces INTEGER,map BLOB,wad BLOB,error TEXT);"
                 "INSERT INTO terrain_compile VALUES('m',1,1,1,X'',X'','');");
            bind_blob(db, "UPDATE content SET data=? WHERE id='texture'", base_wad);
            bind_blob(db, "UPDATE terrain_compile SET map=? WHERE asset_id='m'",
                      std::vector<byte>(compiled_map.begin(), compiled_map.end()));
            bind_blob(db, "UPDATE terrain_compile SET wad=? WHERE asset_id='m'", terrain_wad);
            sqlite3_close(db);
        }

        ~fixture()
        {
            std::error_code ignored;
            std::filesystem::remove_all(root, ignored);
        }
    };

    struct expanded_input
    {
        tools::project_input value;
        ~expanded_input() { tools::discard_project_input(value); }
    };

    bool expand(const fixture &project, expanded_input &result, std::string &error)
    {
        std::vector<std::string> args = {
            "hltools", "--project", project.project.string(), "--map", "Terrain"};
        std::vector<char *> argv;
        for (auto &arg : args)
            argv.push_back(arg.data());
        return tools::expand_project_compile((int)argv.size(), argv.data(),
                                             result.value, &error);
    }
}

suite("unit.cli.project_input")
{
    test("extracts current terrain and rejects texture conflicts")
    {
        for (const bool conflict : {false, true})
        {
            fixture project(conflict);
            expanded_input input;
            std::string error;
            const bool ok = expand(project, input, error);
            if (conflict)
            {
                expect(!ok);
                expect(error.find("conflicts") != std::string::npos);
                continue;
            }
            require(ok);
            std::vector<byte> actual_map;
            require(fs::read_all(input.value.arguments.back(), actual_map));
            expect(std::string(actual_map.begin(), actual_map.end()) == project.compiled_map);
            std::vector<byte> config;
            require(fs::read_all((std::filesystem::path(input.value.working_directory)
                                  / "wads.cfg").string(), config));
            const std::string list(config.begin(), config.end());
            const auto terrain_pos = list.find("terrain-generated.wad");
            const auto base_pos = list.find("BASE.wad");
            require(terrain_pos != std::string::npos);
            require(base_pos != std::string::npos);
            expect(terrain_pos < base_pos);
        }
    }
}
