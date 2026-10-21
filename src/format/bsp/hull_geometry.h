#pragma once

#include <array>
#include <string>
#include <vector>

#include "data.h"

namespace format
{
    struct hull0_face
    {
        int model = 0;
        int texinfo = -1;
        std::vector<std::array<float, 3>> points;
    };

    // hull 0 polygons of one model as world space points, in face order. a
    // malformed surfedge or vertex reference fails the whole call instead of
    // returning partial geometry.
    bool collect_hull0_faces(const map_data &map, int model,
                             std::vector<hull0_face> &faces, std::string *error = nullptr);
}
