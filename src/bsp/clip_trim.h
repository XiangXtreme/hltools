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
        bool complete = false; // every solid source expansion is an addition
        std::vector<trim_volume> targets, guards, additions;
        std::vector<trim_volume> exterior; // captured by BSP before leaf collapse
    };

    struct bsp_state;
    class token_file;
    struct node;
    void parse_clip_trim(bsp_state &state, token_file &file);
    void capture_collision_fill(bsp_state &state, node *root);
    // Run after brink fixes and disk plane rounding. Additions rebuild selected
    // exact expansions; targets outside all guards can only remove SOLID.
    void trim_legacy_collision(bsp_state &state);
}
