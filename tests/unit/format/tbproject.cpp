#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <sqlite3.h>

#include "format/tbproject/project.h"
#include "support/test.h"

namespace
{
    struct database
    {
        sqlite3 *handle = nullptr;

        ~database()
        {
            if (handle)
                sqlite3_close(handle);
        }
    };

    void exec(sqlite3 *handle, const char *sql)
    {
        char *message = nullptr;
        const int rc = sqlite3_exec(handle, sql, nullptr, nullptr, &message);
        if (rc != SQLITE_OK)
        {
            const std::string text = message ? message : "sqlite error";
            sqlite3_free(message);
            throw std::runtime_error(text);
        }
    }

    std::filesystem::path scratch_directory()
    {
        const auto directory =
            std::filesystem::path(HLTOOLS_TEST_SCRATCH_DIRECTORY) / "unit-project";
        std::error_code ignored;
        std::filesystem::remove_all(directory, ignored);
        std::filesystem::create_directories(directory);
        return directory;
    }

    // a project with one map and one compiled texture; schema three adds the
    // publish_path column that older projects do not have
    std::filesystem::path write_project(const std::filesystem::path &path,
                                        const bool schema_three)
    {
        database db;
        require(sqlite3_open(path.string().c_str(), &db.handle) == SQLITE_OK);
        exec(db.handle, schema_three
                            ? "PRAGMA application_id=0x5442504a; PRAGMA user_version=3;"
                            : "PRAGMA application_id=0x5442504a; PRAGMA user_version=2;");
        exec(db.handle,
             "CREATE TABLE project(id TEXT PRIMARY KEY,name TEXT NOT NULL,"
             "game_id TEXT NOT NULL);"
             "CREATE TABLE content(id TEXT PRIMARY KEY,hash TEXT NOT NULL,"
             "data BLOB NOT NULL);"
             "CREATE TABLE revisions(asset_id TEXT NOT NULL,type TEXT NOT NULL,"
             "virtual_path TEXT NOT NULL,display_name TEXT NOT NULL,"
             "source_name TEXT NOT NULL,revision INTEGER NOT NULL,recipe TEXT NOT NULL,"
             "original TEXT NOT NULL,compatible TEXT NOT NULL,"
             "PRIMARY KEY(asset_id,revision));"
             "CREATE TABLE heads(asset_id TEXT PRIMARY KEY,revision INTEGER NOT NULL,"
             "virtual_path TEXT NOT NULL);");
        if (schema_three)
            exec(db.handle, "ALTER TABLE revisions ADD COLUMN publish_path TEXT NOT NULL "
                            "DEFAULT '';");
        exec(db.handle,
             "INSERT INTO project VALUES('p1','Test project','Half-Life');"
             "INSERT INTO content VALUES('cm','hm',X'2f2f206d6170'),"
             "('ct','ht',X'57414433');");
        exec(db.handle,
             "INSERT INTO revisions(asset_id,type,virtual_path,display_name,"
             "source_name,revision,recipe,original,compatible)"
             "VALUES('am','map','maps/alpha.map','alpha','alpha.map',1,'','cm','cm'),"
             "('at','texture','textures/WALL01.wad','WALL01','WALL01.wad',1,'','ct','ct');"
             "INSERT INTO heads VALUES('am',1,'maps/alpha.map'),"
             "('at',1,'textures/WALL01.wad');");
        if (schema_three)
            exec(db.handle, "UPDATE revisions SET publish_path=virtual_path;");
        return path;
    }

    void set_map_display_name(const std::filesystem::path &path,
                              const char *name)
    {
        database db;
        require(sqlite3_open(path.string().c_str(), &db.handle) == SQLITE_OK);
        sqlite3_stmt *statement = nullptr;
        require(sqlite3_prepare_v2(
                    db.handle,
                    "UPDATE revisions SET display_name=? WHERE asset_id='am'",
                    -1, &statement, nullptr) == SQLITE_OK);
        require(sqlite3_bind_text(statement, 1, name, -1, SQLITE_TRANSIENT)
                == SQLITE_OK);
        require(sqlite3_step(statement) == SQLITE_DONE);
        sqlite3_finalize(statement);
    }

    void add_terrain_table(const std::filesystem::path &path, const char *row)
    {
        database db;
        require(sqlite3_open(path.string().c_str(), &db.handle) == SQLITE_OK);
        exec(db.handle,
             "PRAGMA user_version=4;"
             "CREATE TABLE terrain_compile(asset_id TEXT PRIMARY KEY,"
             "source_revision INTEGER NOT NULL,generator_version INTEGER NOT NULL,"
             "top_faces INTEGER NOT NULL,map BLOB NOT NULL,wad BLOB NOT NULL,"
             "error TEXT NOT NULL);");
        if (row)
            exec(db.handle, row);
    }
}

suite("unit.project.tbproject")
{
    test("project.reads_maps_and_textures")
    {
        const auto path = write_project(scratch_directory() / "alpha.tbproj", true);
        std::string error;
        const auto project = format::tbproject::project::open(path.string(), &error);
        require(project != nullptr);
        expect(project->name() == "Test project");
        expect(project->game() == "Half-Life");
        require(project->assets().size() == 2);

        require(project->find_map("alpha") != nullptr);
        expect(project->find_map("alpha")->id == "am");
        expect(project->find_map("alpha")->revision == 1);
        require(project->find_map("alpha.map") != nullptr);
        require(project->find_map("maps/alpha.map") != nullptr);
        expect(project->find_map("beta") == nullptr);
        expect(project->find_map("alpha")->compatible
               == std::vector<byte>{'/', '/', ' ', 'm', 'a', 'p'});

        // GoldSrc texture names are case insensitive
        require(project->find_texture("WALL01") != nullptr);
        require(project->find_texture("wall01") != nullptr);
        expect(project->find_texture("WALL01")->compatible
               == std::vector<byte>{'W', 'A', 'D', '3'});
    }

    test("project.finds_maps_with_utf8_display_names")
    {
        const auto path =
            write_project(scratch_directory() / "unicode.tbproj", true);
        const auto *map_name =
            reinterpret_cast<const char *>(u8"01 光照预览与编译");
        set_map_display_name(path, map_name);

        std::string error;
        const auto project =
            format::tbproject::project::open(path.string(), &error);
        require(project != nullptr);
        const auto *map = project->find_map(map_name);
        require(map != nullptr);
        expect(map->id == "am");
        require(project->find_map("maps/alpha.map") != nullptr);
    }

    test("project.accepts_schema_two")
    {
        const auto path = write_project(scratch_directory() / "legacy.tbproj", false);
        std::string error;
        const auto project = format::tbproject::project::open(path.string(), &error);
        require(project != nullptr);
        // schema two has no publish path, so the virtual path stands in for it
        require(project->find_map("alpha") != nullptr);
        expect(project->find_map("alpha")->publish_path == "maps/alpha.map");
    }

    test("project.rejects_foreign_files")
    {
        const auto path = scratch_directory() / "not-a-project.bin";
        std::ofstream file(path, std::ios::binary);
        file << "definitely not sqlite";
        file.close();

        std::string error;
        expect(format::tbproject::project::open(path.string(), &error) == nullptr);
        expect(!error.empty());
    }

    test("project.terrain_requires_current_successful_cache")
    {
        const auto directory = scratch_directory();
        struct scenario
        {
            const char *name;
            const char *row;
            const char *expected_error;
        };
        const scenario scenarios[] = {
            {"valid", "INSERT INTO terrain_compile VALUES"
                      "('am',1,1,2,X'6d6170',X'57414433','');", ""},
            {"missing", nullptr, "missing"},
            {"stale", "INSERT INTO terrain_compile VALUES"
                      "('am',2,1,2,X'6d6170',X'57414433','');", "outdated"},
            {"failed", "INSERT INTO terrain_compile VALUES"
                       "('am',1,1,0,X'',X'','bad surface');", "bad surface"},
            {"incomplete", "INSERT INTO terrain_compile VALUES"
                           "('am',1,1,0,X'',X'57414433','');", "incomplete"},
        };
        for (const auto &scenario : scenarios)
        {
            const auto path = write_project(directory / (std::string(scenario.name)
                                                          + ".tbproj"), true);
            add_terrain_table(path, scenario.row);
            std::string error;
            const auto project = format::tbproject::project::open(path.string(), &error);
            require(project != nullptr);
            const auto *map = project->find_map("alpha");
            require(map != nullptr);
            format::tbproject::terrain_compile output;
            const bool ok = project->read_terrain_compile(*map, output, &error);
            if (*scenario.expected_error == '\0')
            {
                expect(ok);
                expect(output.map == std::vector<byte>{'m', 'a', 'p'});
                expect(output.wad == std::vector<byte>{'W', 'A', 'D', '3'});
            }
            else
            {
                expect(!ok);
                expect(error.find(scenario.expected_error) != std::string::npos);
            }
        }

        const auto legacy_path = write_project(directory / "legacy-terrain.tbproj", true);
        std::string error;
        const auto legacy = format::tbproject::project::open(legacy_path.string(), &error);
        require(legacy != nullptr);
        const auto *map = legacy->find_map("alpha");
        require(map != nullptr);
        format::tbproject::terrain_compile output;
        expect(!legacy->read_terrain_compile(*map, output, &error));
        expect(error.find("missing") != std::string::npos);
    }
}
