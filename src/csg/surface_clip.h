#pragma once

#include <vector>

#include "../common/direct_clip.h"

namespace csg
{
    struct brush_build_options;
    struct csg_result;

    // Build the direct Python-style surface collision payload in the same
    // model order as csg_result::entities.  Models with mixed clip modes are
    // left invalid so the normal BSP path remains responsible for them.
    std::vector<direct_clip::model> build_surface_clip_models(
        const csg_result &result, const brush_build_options &options);
}
