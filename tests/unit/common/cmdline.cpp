#include "common/cmdline.h"
#include "support/test.h"

#include <limits>
#include <string>
#include <vector>

suite("unit.common.cmdline")
{
    test("integer values preserve signed bounds and absent defaults")
    {
        std::vector<std::string> input = {
            "hltools", "-threads", std::to_string(std::numeric_limits<int>::max()),
            "-bounce", std::to_string(std::numeric_limits<int>::min()), "map"
        };
        std::vector<char *> argv;
        for (std::string &value : input)
            argv.push_back(value.data());
        const cli::args args((int)argv.size(), argv.data());
        expect(args.int_value("-threads", 0) == std::numeric_limits<int>::max());
        expect(args.int_value("-bounce", 0) == std::numeric_limits<int>::min());
        expect(args.int_value("-lightdata", 42) == 42);
    }

    test("bounded integer options reject conversion overflow even at long limits")
    {
        const long lo = std::numeric_limits<long>::min();
        const long hi = std::numeric_limits<long>::max();
        long value = 42;
        const std::string above = std::to_string(hi) + "0";
        const std::string below = std::to_string(lo) + "0";
        expect_false(cli::option_int("test", "-value", above.c_str(), lo, hi, 0, value));
        expect_false(cli::option_int("test", "-value", below.c_str(), lo, hi, 0, value));
        expect_false(cli::option_choice("test", "-value", above.c_str(), {hi}, value));
        expect_false(cli::option_int("test", "-value", "12garbage", lo, hi, 0, value));
        expect(value == 42);
        expect(cli::option_int("test", "-value", std::to_string(hi).c_str(), lo, hi, 0, value));
        expect(value == hi);
    }
}
