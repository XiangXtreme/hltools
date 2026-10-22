#pragma once

#include <vector>
#include "../math/bounding_box.h"
#include "../math/plane.h"

namespace bsp
{
    struct trim_volume
    {
        math::bounding_box bounds;
        std::vector<math::plane> planes; // outward, inside <= dist
    };

    struct trim_group
    {
        int model = 0, hull = 1;
        std::vector<trim_volume> targets, guards;
    };

    struct bsp_state;
    class token_file;
    void parse_clip_trim(bsp_state &state, token_file &file);
    // Run after brink fixes and disk plane rounding. Only SOLID can become EMPTY.
    void trim_legacy_collision(bsp_state &state);
}
