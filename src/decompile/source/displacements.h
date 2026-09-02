#pragma once

#include <cstddef>
#include <vector>

#include "math/vector.h"

namespace format
{
    struct source_map_data;
}

namespace decompile
{
    // one tessellated displacement face
    // a quad when a collision cell's 4 corners were near enough to coplanar
    struct source_displacement_triangle
    {
        math::vec3<double> points[4];
        int point_count = 3;
        math::vec3<double> base_normal;
        double minimum_projection = 0;
        int texinfo = -1;
        // this drawn triangle carries the collision, so no clip triangle exists
        bool solid = false;
    };

    // how far a collision cell's corners may sit off their best-fit plane and
    // still merge into 1 quad brush
    // merging halves the cell's brushes and drops its diagonal side plane
    constexpr double default_displacement_merge = 48.0;

    struct source_displacement_mesh
    {
        std::vector<source_displacement_triangle> triangles;
        // collision is capped at a power-2 grid
        // the full power-3/4 tessellation would overflow the 16-bit clipnode lump
        // empty when the drawn grid is no finer, since those triangles are solid
        std::vector<source_displacement_triangle> collision_triangles;
        std::size_t faces = 0;
        std::size_t skipped_faces = 0;
        std::size_t skipped_triangles = 0;
    };

    // reconstructs source's regular displacement grids
    // triangles wind outward and keep the parent face's texinfo, so the porter
    // can apply the original world-space uv axes
    //
    // draw_power_cap samples power p on a 2^min(p, cap) grid
    // drawn_carries_collision is false for a model-backed terrain, which
    // collides with nothing
    source_displacement_mesh build_source_displacement_mesh(
        const format::source_map_data &map, int draw_power_cap = 4,
        bool drawn_carries_collision = true,
        double merge_tolerance = default_displacement_merge);
}
