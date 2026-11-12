#include <string>
#include <vector>

#include "cli/tools/rad_tool.h"
#include "common/cmdline.h"
#include "common/filesystem.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    rad::rad_options parse(const std::string &base, const std::vector<std::string> &flags = {})
    {
        std::vector<std::string> storage = {"hltools rad"};
        storage.insert(storage.end(), flags.begin(), flags.end());
        storage.push_back(base);
        std::vector<char *> argv;
        for (std::string &argument : storage)
            argv.push_back(argument.data());
        cli::args args((int)argv.size(), argv.data());
        return tools::parse_rad_options(args, (int)argv.size(), argv.data(), base);
    }

    void write_definition(const std::filesystem::path &path)
    {
        const char definition[] = "LIGHT 255\n";
        require(fs::write_all(path.string(), definition, sizeof(definition) - 1));
    }
}
suite("unit.rad.options")
{
    test("new SDHLT lighting options preserve defaults and accept explicit values")
    {
        const auto base = (test_support::scratch_directory("rad_quality") / "sample").string();
        const auto defaults = parse(base);
        expect_false(defaults.ao);
        expect(defaults.pcf == 1);
        expect(defaults.blurclamp == 0.0f);
        expect_false(defaults.light_cull);
        const auto configured = parse(base, {"-ao", "-aoscale", "64", "-aogain", "2",
            "-aolevel", "4", "-aominweight", "0.02", "-aoopacity", "0.5",
            "-aocolor", "20", "30", "40", "-aomode", "additive", "-aostudio", "fast",
            "-aostats", "-pcf", "2", "-blurclamp", "0.75", "-lightcull"});
        expect(configured.ao);
        expect(configured.ao_scale == 64.0f);
        expect(configured.ao_gain == 2.0f);
        expect(configured.ao_level == 4);
        expect(configured.ao_minweight == 0.02f);
        expect(configured.ao_opacity == 0.5f);
        expect(configured.ao_color[0] == 20.0f);
        expect(configured.ao_color[1] == 30.0f);
        expect(configured.ao_color[2] == 40.0f);
        expect(configured.ao_mode == 1);
        expect(configured.ao_studio_mode == 0);
        expect(configured.ao_stats);
        expect(configured.pcf == 2);
        expect(configured.blurclamp == 0.75f);
        expect(configured.light_cull);
    }
    test("options.tracks a generic lights.rad fallback")
    {
        std::filesystem::path scratch = test_support::scratch_directory("rad_options");
        std::string base = (scratch / "sample").string();
        std::string generic = (scratch / "lights.rad").string();
        write_definition(generic);

        rad::rad_options options = parse(base);
        require(options.rad_files.size() == 1);
        expect(options.rad_files[0] == generic);
        expect(options.generic_rad_file == generic);
    }

    test("options.loads map-specific definitions after the generic fallback")
    {
        std::filesystem::path scratch = test_support::scratch_directory("rad_options");
        std::string base = (scratch / "sample").string();
        std::string generic = (scratch / "lights.rad").string();
        std::string specific = base + ".rad";
        write_definition(generic);
        write_definition(specific);

        rad::rad_options options = parse(base);
        require(options.rad_files.size() == 2);
        expect(options.rad_files[0] == generic);
        expect(options.rad_files[1] == specific);
        expect(options.generic_rad_file == generic);
    }
}
