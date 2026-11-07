#include <cmath>
#include <cstring>

#include "rad/internal.h"
#include "support/test.h"

suite("unit.rad.smoothing")
{
    test("terrain memberships enable only local cross-texture smoothing")
    {
        rad::rad_state state;
        state.smoothing_threshold = (float)std::cos(50.0 * math::pi / 180.0);
        state.smoothing_threshold_2 = 1.0f;
        state.textures.resize(5);
        const char *names[] = {"tbt0_0", "tbt0_1", "tbt1_0", "WALL", "tbt9_0"};
        for (size_t i = 0; i < state.textures.size(); ++i)
            std::strcpy(state.textures[i].name, names[i]);
        state.entities.emplace_back();
        state.entities[0].append("classname", "worldspawn");
        state.entities[0].append("zhlt_smooth_tbt0_0", "0 2");
        state.entities[0].append("zhlt_smooth_tbt0_1", "0");
        state.entities[0].append("zhlt_smooth_tbt1_0", "1");
        rad::read_custom_smooth_value(state);

        expect(rad::texture_smoothing_threshold(state, 0, 1) == state.smoothing_threshold);
        expect(rad::texture_smoothing_threshold(state, 1, 0) == state.smoothing_threshold);
        expect(rad::texture_smoothing_threshold(state, 0, 2) == 1.0f);
        expect(rad::texture_smoothing_threshold(state, 0, 3) == 1.0f);
        expect(rad::texture_smoothing_threshold(state, 0, 4) == 1.0f);
        expect(rad::texture_smoothing_threshold(state, 3, 3) == state.smoothing_threshold);

        // Per-texture hard-edge overrides still apply inside terrain groups.
        state.entities.emplace_back();
        state.entities.back().append("classname", "info_smoothvalue");
        state.entities.back().append("tbt0_1", "0");
        rad::read_custom_smooth_value(state);
        expect(rad::texture_smoothing_threshold(state, 0, 1) == 1.0f);
    }

    test("unmarked materials retain the global cross-texture setting")
    {
        rad::rad_state state;
        state.smoothing_threshold = 0.5f;
        state.smoothing_threshold_2 = 0.8f;
        state.textures.resize(2);
        std::strcpy(state.textures[0].name, "WALL");
        std::strcpy(state.textures[1].name, "ROOF");
        rad::read_custom_smooth_value(state);
        expect(rad::texture_smoothing_threshold(state, 0, 1) == 0.8f);
    }
}
