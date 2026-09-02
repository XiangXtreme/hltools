#include "displacements.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "format/vbsp/data.h"

namespace decompile
{
    namespace
    {
        using vec3 = math::vec3<double>;

        bool face_points(const format::source_map_data &map,
                         const format::source_dface_t &face,
                         std::vector<vec3> &points)
        {
            if (face.numedges != 4 || face.firstedge < 0)
                return false;
            std::size_t first = (std::size_t)face.firstedge;
            if (first > map.surfedges.size()
                || (std::size_t)face.numedges > map.surfedges.size() - first)
                return false;

            points.clear();
            points.reserve(4);
            for (int i = 0; i < face.numedges; i++)
            {
                int surfedge = map.surfedges[first + (std::size_t)i];
                if (surfedge == std::numeric_limits<int>::min())
                    return false;
                std::size_t edge_index =
                    (std::size_t)(surfedge < 0 ? -surfedge : surfedge);
                if (edge_index >= map.edges.size())
                    return false;
                const format::source_dedge_t &edge = map.edges[edge_index];
                unsigned short vertex = edge.v[surfedge < 0 ? 1 : 0];
                if ((std::size_t)vertex >= map.vertexes.size())
                    return false;
                const float *p = map.vertexes[(std::size_t)vertex].point;
                points.push_back({p[0], p[1], p[2]});
            }
            return true;
        }

        int closest_corner(const std::vector<vec3> &points, const float start[3])
        {
            vec3 wanted{start[0], start[1], start[2]};
            int best = 0;
            double best_distance = std::numeric_limits<double>::max();
            for (int i = 0; i < 4; i++)
            {
                vec3 delta = points[(std::size_t)i] - wanted;
                double distance = math::dot(delta, delta);
                if (distance < best_distance)
                {
                    best = i;
                    best_distance = distance;
                }
            }
            return best;
        }

        bool face_plane(const format::source_map_data &map,
                        const format::source_dface_t &face,
                        vec3 &normal, double &dist)
        {
            if ((std::size_t)face.planenum >= map.planes.size())
                return false;
            const format::source_dplane_t &plane =
                map.planes[(std::size_t)face.planenum];
            double sign = face.side ? -1.0 : 1.0;
            normal = {plane.normal[0] * sign, plane.normal[1] * sign,
                      plane.normal[2] * sign};
            dist = plane.dist * sign;
            return true;
        }

        int displacement_face(const format::source_map_data &map,
                              std::size_t displacement_index,
                              const format::source_ddispinfo_t &info)
        {
            if ((std::size_t)info.map_face < map.faces.size()
                && map.faces[(std::size_t)info.map_face].dispinfo
                    == (short)displacement_index)
                return (int)info.map_face;
            for (std::size_t i = 0; i < map.faces.size(); i++)
                if (map.faces[i].dispinfo == (short)displacement_index)
                    return (int)i;
            return -1;
        }

        void add_triangle(source_displacement_mesh &mesh,
                          std::vector<source_displacement_triangle> &output,
                          vec3 a, vec3 b, vec3 c,
                          const vec3 &base_normal, double minimum_projection,
                          int texinfo, bool solid)
        {
            vec3 normal = math::cross(b - a, c - a);
            if (math::normalize(normal) == 0)
            {
                mesh.skipped_triangles++;
                return;
            }
            if (math::dot(normal, base_normal) < 0)
                std::swap(b, c);

            source_displacement_triangle triangle;
            triangle.points[0] = a;
            triangle.points[1] = b;
            triangle.points[2] = c;
            triangle.base_normal = base_normal;
            triangle.minimum_projection = minimum_projection;
            triangle.texinfo = texinfo;
            triangle.solid = solid;
            output.push_back(triangle);
        }

        // a cell's 4 corners as 1 brush, when flat enough to share a top plane
        // false leaves the cell triangulated
        bool add_quad(std::vector<source_displacement_triangle> &output,
                      const vec3 &a, const vec3 &b, const vec3 &c, const vec3 &d,
                      const vec3 &base_normal, double minimum_projection,
                      int texinfo, double tolerance, bool solid)
        {
            // newell's normal averages the outline rather than trusting 3 corners,
            // so a warped cell splits down the middle rather than hinging
            const vec3 outline[4] = {a, b, c, d};
            vec3 normal{0, 0, 0};
            vec3 centre{0, 0, 0};
            for (int i = 0; i < 4; i++)
            {
                const vec3 &p = outline[i];
                const vec3 &q = outline[(i + 1) % 4];
                normal.x += (p.y - q.y) * (p.z + q.z);
                normal.y += (p.z - q.z) * (p.x + q.x);
                normal.z += (p.x - q.x) * (p.y + q.y);
                centre = centre + p;
            }
            if (math::normalize(normal) == 0)
                return false;
            centre = centre * 0.25;

            // the merged face is 1 plane, and tolerance bounds the corner movement
            for (const vec3 &p : outline)
                if (std::fabs(math::dot(normal, p - centre)) > tolerance)
                    return false;

            // a reflex corner would enclose more than the cell
            vec3 corners[4];
            for (int i = 0; i < 4; i++)
                corners[i] = outline[i]
                    - normal * math::dot(normal, outline[i] - centre);
            double sign = 0;
            for (int i = 0; i < 4; i++)
            {
                vec3 e0 = corners[(i + 1) % 4] - corners[i];
                vec3 e1 = corners[(i + 2) % 4] - corners[(i + 1) % 4];
                double turn = math::dot(math::cross(e0, e1), normal);
                if (sign == 0)
                    sign = turn;
                else if (turn * sign < 0)
                    return false;
                if (std::fabs(turn) < 1e-9)
                    return false;
            }

            source_displacement_triangle quad;
            quad.point_count = 4;
            for (int i = 0; i < 4; i++)
                quad.points[i] = corners[i];
            if (math::dot(normal, base_normal) < 0)
                std::swap(quad.points[1], quad.points[3]);
            quad.base_normal = base_normal;
            quad.minimum_projection = minimum_projection;
            quad.texinfo = texinfo;
            quad.solid = solid;
            output.push_back(quad);
            return true;
        }
    }

    source_displacement_mesh build_source_displacement_mesh(
        const format::source_map_data &map, int draw_power_cap,
        bool drawn_carries_collision, double merge_tolerance)
    {
        draw_power_cap = std::max(0, std::min(draw_power_cap, 4));
        source_displacement_mesh mesh;
        std::vector<vec3> corners;

        for (std::size_t displacement_index = 0;
             displacement_index < map.dispinfo.size(); displacement_index++)
        {
            const format::source_ddispinfo_t &info =
                map.dispinfo[displacement_index];
            int face_index = displacement_face(map, displacement_index, info);
            if (face_index < 0 || info.power < 1 || info.power > 4
                || info.disp_vert_start < 0)
            {
                mesh.skipped_faces++;
                continue;
            }

            const format::source_dface_t &face =
                map.faces[(std::size_t)face_index];
            if (!face_points(map, face, corners))
            {
                mesh.skipped_faces++;
                continue;
            }

            int start = closest_corner(corners, info.start_position);
            std::rotate(corners.begin(), corners.begin() + start, corners.end());

            int side = (1 << info.power) + 1;
            std::size_t vertex_count = (std::size_t)side * (std::size_t)side;
            std::size_t first = (std::size_t)info.disp_vert_start;
            if (first > map.disp_verts.size()
                || vertex_count > map.disp_verts.size() - first)
            {
                mesh.skipped_faces++;
                continue;
            }

            vec3 normal;
            double base_dist = 0;
            if (!face_plane(map, face, normal, base_dist)
                || math::normalize(normal) == 0)
            {
                mesh.skipped_faces++;
                continue;
            }

            std::vector<vec3> grid(vertex_count);
            double minimum_projection = std::numeric_limits<double>::max();
            double interval = 1.0 / (double)(side - 1);
            vec3 edge0 = (corners[1] - corners[0]) * interval;
            vec3 edge1 = (corners[2] - corners[3]) * interval;
            for (int row = 0; row < side; row++)
            {
                vec3 end0 = corners[0] + edge0 * (double)row;
                vec3 end1 = corners[3] + edge1 * (double)row;
                vec3 across = (end1 - end0) * interval;
                for (int column = 0; column < side; column++)
                {
                    std::size_t index =
                        (std::size_t)row * (std::size_t)side
                        + (std::size_t)column;
                    const format::source_ddispvert_t &disp =
                        map.disp_verts[first + index];
                    vec3 direction{
                        disp.vector[0], disp.vector[1], disp.vector[2]};
                    grid[index] = end0 + across * (double)column
                        + direction * (double)disp.dist;
                    minimum_projection = std::min(
                        minimum_projection, math::dot(normal, grid[index]));
                }
            }

            auto append_grid =
                [&](int step, bool solid, double tolerance,
                    std::vector<source_displacement_triangle> &output)
            {
                for (int row = 0; row < side - 1; row += step)
                {
                    for (int column = 0; column < side - 1; column += step)
                    {
                        std::size_t a = (std::size_t)row * side + column;
                        std::size_t b = a + (std::size_t)step;
                        std::size_t d =
                            a + (std::size_t)step * (std::size_t)side;
                        std::size_t c = d + (std::size_t)step;
                        if (tolerance > 0
                            && add_quad(output, grid[a], grid[b], grid[c],
                                        grid[d], normal, minimum_projection,
                                        face.texinfo, tolerance, solid))
                            continue;
                        if ((((row / step) + (column / step)) & 1) == 0)
                        {
                            add_triangle(mesh, output, grid[a], grid[b], grid[c],
                                         normal, minimum_projection,
                                         face.texinfo, solid);
                            add_triangle(mesh, output, grid[a], grid[c], grid[d],
                                         normal, minimum_projection,
                                         face.texinfo, solid);
                        }
                        else
                        {
                            add_triangle(mesh, output, grid[a], grid[b], grid[d],
                                         normal, minimum_projection,
                                         face.texinfo, solid);
                            add_triangle(mesh, output, grid[b], grid[c], grid[d],
                                         normal, minimum_projection,
                                         face.texinfo, solid);
                        }
                    }
                }
            };

            int draw_power = std::min(info.power, draw_power_cap);
            int draw_step = 1 << (info.power - draw_power);
            // collision is never finer than the drawn grid or the power-2 cap
            int collision_power = std::min(draw_power, 2);
            int collision_step = 1 << (info.power - collision_power);
            // identical grids would duplicate every triangle, so draw 1 solid brush
            bool solid = drawn_carries_collision && collision_step == draw_step;

            // the drawn mesh is never merged, since a quad through non-coplanar
            // corners creases against its neighbours
            append_grid(draw_step, solid, 0.0, mesh.triangles);
            if (!solid)
                append_grid(collision_step, false, merge_tolerance,
                            mesh.collision_triangles);
            mesh.faces++;
        }
        return mesh;
    }
}
