#include "surface_clip.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <tuple>
#include <utility>
#include <iterator>

#include "../common/error.h"
#include "../common/log.h"
#include "csg.h"

namespace csg
{
    namespace
    {
        constexpr vec_t polygon_epsilon = (vec_t)1e-6; // plane classification EPS
        constexpr vec_t polygon_point_epsilon = (vec_t)1e-9;
        constexpr vec_t polygon_area_epsilon = (vec_t)1e-8;
        constexpr vec_t fragment_epsilon = (vec_t)1e-3;
        // These are the canonicalization tolerances used by exacthull.py.
        // They prevent thin, nearly parallel triangle facets from creating
        // compiler-style ambiguous leaves and an explosive fragment tree.
        constexpr vec_t plane_normal_tolerance = (vec_t)1e-4;
        constexpr vec_t plane_dist_tolerance = (vec_t)0.05;
        constexpr vec_t boundary_plane_epsilon = (vec_t)1e-7;
        constexpr vec_t boundary_clip_epsilon = (vec_t)1e-8;
        constexpr vec_t boundary_volume_epsilon = (vec_t)1e-3;
        constexpr vec_t boundary_roof_lift = (vec_t)0.00025;
        constexpr vec_t boundary_envelope = (vec_t)0.001;

        using point_list = std::vector<math::vec3v>;

        struct polygon
        {
            point_list points;
        };

        struct fragment
        {
            point_list vertices;
            std::vector<std::set<int>> incidence;
            std::set<int> sides;
            std::set<int> used;
        };

        // Convex polyhedron used by the boundary-normalized composition.
        // Incidence records are deliberately retained: the same two-plane
        // edge rule is used by brushbsp.py when a splitter cuts a fragment.
        struct boundary_poly
        {
            point_list vertices;
            std::vector<std::set<int>> incidence;
            math::bounding_box bounds;
        };

        struct boundary_volume
        {
            std::vector<direct_clip::plane> planes;
            math::bounding_box bounds;
        };

        struct boundary_term
        {
            std::vector<int> members;
        };

        struct triangle
        {
            math::vec3v points[3];
            math::vec3v normal;
        };

        struct plane_table
        {
            std::vector<direct_clip::plane> values;

            int add(math::vec3v normal, vec_t dist)
            {
                math::normalize(normal);
                for (int i = 0; i < (int)values.size(); i++)
                {
                    const auto &p = values[(size_t)i];
                    if (math::length(normal - p.normal) <= plane_normal_tolerance
                        && std::fabs(dist - p.dist) <= plane_dist_tolerance)
                        return i;
                }
                values.push_back({normal, dist});
                return (int)values.size() - 1;
            }
        };

        vec_t polygon_area(const point_list &p)
        {
            if (p.size() < 3) return 0;
            math::vec3v sum{};
            for (size_t i = 1; i + 1 < p.size(); i++)
            {
                math::vec3v a = p[i] - p[0];
                math::vec3v b = p[i + 1] - p[0];
                math::vec3v cross;
                math::cross(a, b, cross);
                sum += cross;
            }
            return (vec_t)(math::length(sum) * 0.5);
        }

        void remove_repeated_points(point_list &p)
        {
            if (p.empty()) return;
            point_list out;
            out.reserve(p.size());
            for (const auto &v : p)
            {
                if (out.empty() || math::length(v - out.back()) > polygon_point_epsilon)
                    out.push_back(v);
            }
            if (out.size() > 1 && math::length(out.front() - out.back()) <= polygon_point_epsilon)
                out.pop_back();
            p = std::move(out);
        }

        void split_polygon(const point_list &polygon_in, const math::vec3v &normal,
                           vec_t dist, point_list &front, point_list &back)
        {
            front.clear();
            back.clear();
            if (polygon_in.empty()) return;
            for (size_t i = 0; i < polygon_in.size(); i++)
            {
                size_t j = (i + 1) % polygon_in.size();
                vec_t a = math::dot(polygon_in[i], normal) - dist;
                vec_t b = math::dot(polygon_in[j], normal) - dist;
                if (std::fabs(a) <= polygon_epsilon) a = 0;
                if (std::fabs(b) <= polygon_epsilon) b = 0;
                if (a >= 0) front.push_back(polygon_in[i]);
                if (a <= 0) back.push_back(polygon_in[i]);
                if ((a < 0 && b > 0) || (a > 0 && b < 0))
                {
                    vec_t t = a / (a - b);
                    math::vec3v cut = polygon_in[i] + (polygon_in[j] - polygon_in[i]) * t;
                    front.push_back(cut);
                    back.push_back(cut);
                }
            }
            remove_repeated_points(front);
            remove_repeated_points(back);
            if (polygon_area(front) <= polygon_area_epsilon) front.clear();
            if (polygon_area(back) <= polygon_area_epsilon) back.clear();
        }

        bool bounds_disjoint(const math::bounding_box &a, const math::bounding_box &b)
        {
            return a.disjoint(b);
        }

        std::vector<polygon> subtract_polygon(const polygon &input,
                                              const std::vector<direct_clip::plane> &planes,
                                              const math::vec3v &face_normal,
                                              int owner, int other)
        {
            if (input.points.empty()) return {};
            std::vector<vec_t> offsets;
            offsets.reserve(planes.size());
            bool all_outside = false;
            for (const auto &p : planes)
            {
                vec_t minv = std::numeric_limits<vec_t>::max();
                for (const auto &v : input.points)
                    minv = std::min(minv, math::dot(v, p.normal) - p.dist);
                offsets.push_back(minv);
                if (minv > polygon_epsilon) all_outside = true;
            }
            if (all_outside) return {input};

            std::vector<bool> coplanar(planes.size(), false);
            for (size_t i = 0; i < planes.size(); i++)
            {
                const auto &p = planes[i];
                vec_t minv = std::numeric_limits<vec_t>::max();
                vec_t maxv = -std::numeric_limits<vec_t>::max();
                for (const auto &v : input.points)
                {
                    vec_t x = math::dot(v, p.normal) - p.dist;
                    minv = std::min(minv, x);
                    maxv = std::max(maxv, x);
                }
                coplanar[i] = std::fabs(minv) <= polygon_epsilon
                    && std::fabs(maxv) <= polygon_epsilon
                    && std::fabs(math::dot(p.normal, face_normal)) > (vec_t)1 - (vec_t)1e-8;
                if (minv >= -polygon_epsilon && !coplanar[i])
                    return {input};
            }
            // A same-facing duplicate boundary belongs to the earlier brush,
            // matching the Python owner rule. Opposite-facing contact is
            // deliberately ignored and is removed by the other planes.
            if (other > owner)
                for (size_t i = 0; i < planes.size(); i++)
                    if (coplanar[i] && math::dot(planes[i].normal, face_normal) > (vec_t)1 - (vec_t)1e-8)
                        return {input};

            point_list inside = input.points;
            std::vector<polygon> outside;
            for (size_t i = 0; i < planes.size(); i++)
            {
                if (coplanar[i]) continue;
                point_list front, back;
                split_polygon(inside, planes[i].normal, planes[i].dist, front, back);
                if (!front.empty()) outside.push_back({std::move(front)});
                if (back.empty()) return {input};
                inside = std::move(back);
            }
            return outside;
        }

        std::vector<direct_clip::plane> brush_planes(const csg_result &result,
                                                     const built_brush &brush)
        {
            std::vector<direct_clip::plane> out;
            for (const auto &face : brush.hulls[0].faces)
            {
                const auto &p = result.planes[(size_t)face.planenum];
                out.push_back({p.normal, p.dist});
            }
            return out;
        }

        std::vector<triangle> extract_triangles(const csg_result &result,
                                                 const built_entity &entity,
                                                 bool boundary_only = false)
        {
            size_t input_faces = 0, exterior_fragments = 0;
            std::vector<const built_brush *> brushes;
            for (const auto &brush : entity.brushes)
                if (brush.contents == content::solid && !brush.hulls[0].faces.empty()
                    && (!boundary_only || brush.pyboundary))
                    brushes.push_back(&brush);
            if (brushes.empty()) return {};

            struct volume
            {
                std::vector<direct_clip::plane> planes;
                math::bounding_box bounds;
            };
            std::vector<volume> volumes;
            volumes.reserve(brushes.size());
            for (const built_brush *brush : brushes)
                volumes.push_back({brush_planes(result, *brush), brush->hulls[0].bounds});

            std::vector<triangle> triangles;
            for (size_t owner = 0; owner < brushes.size(); owner++)
            {
                const built_brush &brush = *brushes[owner];
                for (const auto &face : brush.hulls[0].faces)
                {
                    input_faces++;
                    const auto &plane = result.planes[(size_t)face.planenum];
                    std::vector<polygon> pieces = {{face.winding.points()}};
                    math::bounding_box face_bounds;
                    for (const auto &p : face.winding.points()) face_bounds.add(p);
                    for (size_t other = 0; other < volumes.size(); other++)
                    {
                        if (other == owner || bounds_disjoint(face_bounds, volumes[other].bounds))
                            continue;
                        std::vector<polygon> next;
                        for (const polygon &piece : pieces)
                        {
                            auto cut = subtract_polygon(piece, volumes[other].planes,
                                                        plane.normal, (int)owner, (int)other);
                            next.insert(next.end(), std::make_move_iterator(cut.begin()),
                                        std::make_move_iterator(cut.end()));
                        }
                        pieces = std::move(next);
                        if (pieces.empty()) break;
                    }
                    for (const polygon &piece : pieces)
                    {
                        exterior_fragments++;
                        for (size_t i = 1; i + 1 < piece.points.size(); i++)
                        {
                            triangle tri{{piece.points[0], piece.points[i], piece.points[i + 1]}, plane.normal};
                            math::vec3v cross;
                            math::cross(tri.points[1] - tri.points[0], tri.points[2] - tri.points[0], cross);
                            if (math::length(cross) <= polygon_area_epsilon * 2) continue;
                            if (math::dot(cross, tri.normal) < 0)
                                std::swap(tri.points[1], tri.points[2]);
                            triangles.push_back(tri);
                        }
                    }
                }
            }
            logging::info("pyexact: exterior input %zu faces, %zu fragments, %zu triangles\n",
                          input_faces, exterior_fragments, triangles.size());
            return triangles;
        }

        std::vector<direct_clip::plane> expanded_planes(const triangle &tri,
                                                        const math::vec3v &half)
        {
            std::vector<math::vec3v> points;
            points.reserve(24);
            for (const auto &corner : {math::vec3v{-1, -1, -1}, math::vec3v{-1, -1, 1},
                                       math::vec3v{-1, 1, -1}, math::vec3v{-1, 1, 1},
                                       math::vec3v{1, -1, -1}, math::vec3v{1, -1, 1},
                                       math::vec3v{1, 1, -1}, math::vec3v{1, 1, 1}})
                for (const auto &v : tri.points)
                    points.push_back(v + math::vec3v{corner.x * half.x, corner.y * half.y, corner.z * half.z});

            std::vector<direct_clip::plane> out;
            for (size_t i = 0; i < points.size(); i++)
                for (size_t j = i + 1; j < points.size(); j++)
                    for (size_t k = j + 1; k < points.size(); k++)
                    {
                        math::vec3v n;
                        math::cross(points[j] - points[i], points[k] - points[i], n);
                        if (math::normalize(n) == 0) continue;
                        vec_t d = math::dot(n, points[i]);
                        vec_t minv = std::numeric_limits<vec_t>::max();
                        vec_t maxv = -std::numeric_limits<vec_t>::max();
                        for (const auto &p : points)
                        {
                            vec_t x = math::dot(n, p) - d;
                            minv = std::min(minv, x);
                            maxv = std::max(maxv, x);
                        }
                        if (maxv <= fragment_epsilon)
                        {
                            // n points out of the convex hull.
                        }
                        else if (minv >= -fragment_epsilon)
                        {
                            n = -n;
                            d = -d;
                        }
                        else continue;

                        int on = 0;
                        for (const auto &p : points)
                            if (std::fabs(math::dot(n, p) - d) <= fragment_epsilon * 10) on++;
                        if (on < 3) continue;
                        direct_clip::plane candidate{n, d};
                        bool duplicate = false;
                        for (const auto &p : out)
                            duplicate = duplicate
                                || (math::length(p.normal - candidate.normal) <= plane_normal_tolerance
                                    && std::fabs(p.dist - candidate.dist) <= plane_dist_tolerance);
                        if (!duplicate) out.push_back(candidate);
                    }
            return out;
        }

        fragment make_fragment(plane_table &planes, const triangle &tri,
                               const math::vec3v &half)
        {
            std::vector<math::vec3v> points;
            points.reserve(24);
            for (const auto &corner : {math::vec3v{-1, -1, -1}, math::vec3v{-1, -1, 1},
                                       math::vec3v{-1, 1, -1}, math::vec3v{-1, 1, 1},
                                       math::vec3v{1, -1, -1}, math::vec3v{1, -1, 1},
                                       math::vec3v{1, 1, -1}, math::vec3v{1, 1, 1}})
                for (const auto &v : tri.points)
                    points.push_back(v + math::vec3v{corner.x * half.x, corner.y * half.y, corner.z * half.z});
            auto facets = expanded_planes(tri, half);
            fragment out;
            out.vertices = std::move(points);
            out.incidence.resize(out.vertices.size());
            std::vector<int> local_planes;
            for (const auto &p : facets)
            {
                int id = planes.add(p.normal, p.dist);
                bool near_local = false;
                for (int previous : local_planes)
                {
                    const auto &q = planes.values[(size_t)previous];
                    if (math::length(q.normal - planes.values[(size_t)id].normal) <= (vec_t)3e-4
                        && std::fabs(q.dist - planes.values[(size_t)id].dist) <= (vec_t)0.5)
                    {
                        near_local = true;
                        break;
                    }
                }
                if (near_local) continue;
                local_planes.push_back(id);
                for (size_t i = 0; i < out.vertices.size(); i++)
                    if (std::fabs(math::dot(p.normal, out.vertices[i]) - p.dist) <= fragment_epsilon * 10)
                        out.incidence[i].insert(id);
            }
            for (int id = 0; id < (int)planes.values.size(); id++)
            {
                int count = 0;
                for (const auto &inc : out.incidence)
                    count += inc.count(id) != 0;
                if (count >= 3) out.sides.insert(id);
            }
            return out;
        }

        void merge_vertices(point_list &vertices, std::vector<std::set<int>> &incidence)
        {
            point_list new_vertices;
            std::vector<std::set<int>> new_incidence;
            std::map<std::tuple<long long, long long, long long>, size_t> lookup;
            const vec_t step = fragment_epsilon * 4;
            for (size_t i = 0; i < vertices.size(); i++)
            {
                auto key = std::make_tuple((long long)std::llround(vertices[i].x / step),
                                           (long long)std::llround(vertices[i].y / step),
                                           (long long)std::llround(vertices[i].z / step));
                auto it = lookup.find(key);
                if (it == lookup.end())
                {
                    lookup.emplace(key, new_vertices.size());
                    new_vertices.push_back(vertices[i]);
                    new_incidence.push_back(incidence[i]);
                }
                else
                {
                    auto &dst = new_incidence[it->second];
                    dst.insert(incidence[i].begin(), incidence[i].end());
                }
            }
            vertices = std::move(new_vertices);
            incidence = std::move(new_incidence);
        }

        std::pair<std::unique_ptr<fragment>, std::unique_ptr<fragment>>
        split_fragment(const fragment &input, int pid, const direct_clip::plane &plane)
        {
            std::vector<vec_t> side(input.vertices.size());
            vec_t minv = std::numeric_limits<vec_t>::max();
            vec_t maxv = -std::numeric_limits<vec_t>::max();
            for (size_t i = 0; i < side.size(); i++)
            {
                side[i] = math::dot(input.vertices[i], plane.normal) - plane.dist;
                minv = std::min(minv, side[i]);
                maxv = std::max(maxv, side[i]);
            }
            if (maxv <= fragment_epsilon || minv >= -fragment_epsilon)
            {
                auto copy = std::make_unique<fragment>(input);
                if (input.sides.count(pid)) copy->used.insert(pid);
                if (maxv <= fragment_epsilon)
                    return {nullptr, std::move(copy)};
                return {std::move(copy), nullptr};
            }

            point_list cuts;
            std::vector<std::set<int>> cut_incidence;
            for (size_t i = 0; i < input.vertices.size(); i++)
                if (side[i] < -fragment_epsilon)
                    for (size_t j = 0; j < input.vertices.size(); j++)
                        if (side[j] > fragment_epsilon)
                        {
                            int common = 0;
                            for (int p : input.incidence[i]) common += input.incidence[j].count(p) != 0;
                            if (common < 2) continue;
                            vec_t t = side[i] / (side[i] - side[j]);
                            cuts.push_back(input.vertices[i] + (input.vertices[j] - input.vertices[i]) * t);
                            std::set<int> inc;
                            std::set_intersection(input.incidence[i].begin(), input.incidence[i].end(),
                                                  input.incidence[j].begin(), input.incidence[j].end(),
                                                  std::inserter(inc, inc.begin()));
                            inc.insert(pid);
                            cut_incidence.push_back(std::move(inc));
                        }

            std::unique_ptr<fragment> result[2] = {std::make_unique<fragment>(), std::make_unique<fragment>()};
            for (int which = 0; which < 2; which++)
            {
                auto &out = *result[which];
                for (size_t i = 0; i < input.vertices.size(); i++)
                {
                    bool keep = which == 0 ? side[i] >= -fragment_epsilon : side[i] <= fragment_epsilon;
                    if (!keep) continue;
                    out.vertices.push_back(input.vertices[i]);
                    out.incidence.push_back(input.incidence[i]);
                    if (std::fabs(side[i]) <= fragment_epsilon)
                        out.incidence.back().insert(pid);
                }
                out.vertices.insert(out.vertices.end(), cuts.begin(), cuts.end());
                out.incidence.insert(out.incidence.end(), cut_incidence.begin(), cut_incidence.end());
                merge_vertices(out.vertices, out.incidence);
                for (const auto &old : input.sides)
                {
                    int count = 0;
                    for (const auto &inc : out.incidence) count += inc.count(old) != 0;
                    if (count >= 3) out.sides.insert(old);
                }
                out.sides.insert(pid);
                out.used = input.used;
                out.used.insert(pid);
            }
            return {std::move(result[0]), std::move(result[1])};
        }

        int build_tree(std::vector<fragment> fragments, plane_table &planes,
                       std::vector<direct_clip::node> &nodes, int depth = 0)
        {
            if (fragments.empty()) return -1;
            if (depth > 4096)
                err::fatal("pyexact: fragment BSP depth exceeded");
            for (const auto &f : fragments)
            {
                bool complete = true;
                for (int side : f.sides)
                    if (!f.used.count(side)) { complete = false; break; }
                if (complete) return -2;
            }

            std::set<int> candidate_set;
            for (const auto &f : fragments)
                for (int side : f.sides)
                    if (!f.used.count(side)) candidate_set.insert(side);
            std::vector<int> candidates(candidate_set.begin(), candidate_set.end());
            if (candidates.empty()) return -2;
            // Match brushbsp.py's bounded splitter survey.  Without this cap
            // a large terrain shell spends most of its time scoring every
            // historical facet plane at every recursive cell.
            if (candidates.size() > 24)
            {
                std::mt19937 rng(0x5eedu + (unsigned)depth);
                std::shuffle(candidates.begin(), candidates.end(), rng);
                candidates.resize(24);
            }

            int best = -1;
            int best_score = std::numeric_limits<int>::max();
            for (int pid : candidates)
            {
                int front = 0, back = 0, cut = 0;
                const auto &p = planes.values[(size_t)pid];
                for (const auto &f : fragments)
                {
                    vec_t minv = std::numeric_limits<vec_t>::max();
                    vec_t maxv = -std::numeric_limits<vec_t>::max();
                    for (const auto &v : f.vertices)
                    {
                        vec_t x = math::dot(v, p.normal) - p.dist;
                        minv = std::min(minv, x); maxv = std::max(maxv, x);
                    }
                    if (maxv <= fragment_epsilon) back++;
                    else if (minv >= -fragment_epsilon) front++;
                    else cut++;
                }
                int score = 5 * cut + std::abs(front - back);
                if (score < best_score) { best_score = score; best = pid; }
            }

            std::vector<fragment> front, back;
            const auto &p = planes.values[(size_t)best];
            for (const auto &f : fragments)
            {
                auto split = split_fragment(f, best, p);
                if (split.first) front.push_back(std::move(*split.first));
                if (split.second) back.push_back(std::move(*split.second));
            }
            int index = (int)nodes.size();
            if (nodes.size() >= 500000)
                err::fatal("pyexact: fragment BSP exceeded the internal node budget (fragments=%zu, depth=%d)", fragments.size(), depth);
            nodes.push_back({best, -1, -1});
            nodes[(size_t)index].front = build_tree(std::move(front), planes, nodes, depth + 1);
            nodes[(size_t)index].back = build_tree(std::move(back), planes, nodes, depth + 1);
            return index;
        }

        constexpr int boundary_box_plane_base = 1000000;
        constexpr int boundary_source_plane_base = 2000000;
        constexpr int boundary_envelope_plane_base = 3000000;

        void rebuild_boundary_bounds(boundary_poly &poly)
        {
            poly.bounds = {};
            for (const auto &v : poly.vertices)
                poly.bounds.add(v);
        }

        void merge_boundary_vertices(point_list &vertices,
                                      std::vector<std::set<int>> &incidence)
        {
            point_list merged;
            std::vector<std::set<int>> merged_incidence;
            // Match the Python polyhedron deduplication scale.  A millimeter
            // snap merges neighboring terrain vertices and destroys edge
            // incidence, which is fatal to the adaptive arrangement.
            const vec_t snap = (vec_t)1e-9;
            for (size_t i = 0; i < vertices.size(); i++)
            {
                size_t found = merged.size();
                for (size_t j = 0; j < merged.size(); j++)
                    if (std::fabs(vertices[i].x - merged[j].x) <= snap
                        && std::fabs(vertices[i].y - merged[j].y) <= snap
                        && std::fabs(vertices[i].z - merged[j].z) <= snap)
                    {
                        found = j;
                        break;
                    }
                if (found == merged.size())
                {
                    merged.push_back(vertices[i]);
                    merged_incidence.push_back(incidence[i]);
                }
                else
                    merged_incidence[found].insert(incidence[i].begin(), incidence[i].end());
            }
            vertices = std::move(merged);
            incidence = std::move(merged_incidence);
        }

        boundary_poly make_boundary_box(const math::bounding_box &bounds)
        {
            boundary_poly out;
            const auto &lo = bounds.mins;
            const auto &hi = bounds.maxs;
            out.vertices = {
                {lo.x, lo.y, lo.z}, {lo.x, lo.y, hi.z},
                {lo.x, hi.y, lo.z}, {lo.x, hi.y, hi.z},
                {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z},
                {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z},
            };
            out.incidence.resize(out.vertices.size());
            for (size_t i = 0; i < out.vertices.size(); i++)
            {
                const auto &v = out.vertices[i];
                if (std::fabs(v.x - hi.x) <= boundary_clip_epsilon) out.incidence[i].insert(boundary_box_plane_base + 0);
                if (std::fabs(v.x - lo.x) <= boundary_clip_epsilon) out.incidence[i].insert(boundary_box_plane_base + 1);
                if (std::fabs(v.y - hi.y) <= boundary_clip_epsilon) out.incidence[i].insert(boundary_box_plane_base + 2);
                if (std::fabs(v.y - lo.y) <= boundary_clip_epsilon) out.incidence[i].insert(boundary_box_plane_base + 3);
                if (std::fabs(v.z - hi.z) <= boundary_clip_epsilon) out.incidence[i].insert(boundary_box_plane_base + 4);
                if (std::fabs(v.z - lo.z) <= boundary_clip_epsilon) out.incidence[i].insert(boundary_box_plane_base + 5);
            }
            rebuild_boundary_bounds(out);
            return out;
        }

        bool boundary_edge(const std::set<int> &a, const std::set<int> &b)
        {
            int common = 0;
            auto i = a.begin();
            auto j = b.begin();
            while (i != a.end() && j != b.end())
            {
                if (*i == *j) { common++; ++i; ++j; }
                else if (*i < *j) ++i;
                else ++j;
            }
            return common >= 2;
        }

        bool boundary_geometric_edge(const point_list &vertices, size_t a, size_t b)
        {
            const auto edge = vertices[b] - vertices[a];
            if (math::length(edge) <= (vec_t)1e-9) return false;
            for (size_t k = 0; k < vertices.size(); k++)
            {
                if (k == a || k == b) continue;
                math::vec3v normal;
                math::cross(edge, vertices[k] - vertices[a], normal);
                const vec_t nlen = math::length(normal);
                if (nlen <= (vec_t)1e-9) continue;
                normal = normal * ((vec_t)1 / nlen);
                vec_t minv = std::numeric_limits<vec_t>::max();
                vec_t maxv = -std::numeric_limits<vec_t>::max();
                for (const auto &v : vertices)
                {
                    const vec_t side = math::dot(v - vertices[a], normal);
                    minv = std::min(minv, side);
                    maxv = std::max(maxv, side);
                }
                if (!(minv >= -(vec_t)1e-6 || maxv <= (vec_t)1e-6)) continue;
                // On that supporting face, the pair is an edge only when all
                // coplanar points lie on one side of the pair line.  This
                // rejects diagonals of large coplanar faces, which otherwise
                // create quadratic numbers of recovery vertices.
                math::vec3v line_side;
                math::cross(normal, edge, line_side);
                vec_t line_min = std::numeric_limits<vec_t>::max();
                vec_t line_max = -std::numeric_limits<vec_t>::max();
                for (const auto &v : vertices)
                {
                    const vec_t side = math::dot(v - vertices[a], line_side);
                    line_min = std::min(line_min, side);
                    line_max = std::max(line_max, side);
                }
                if (line_min >= -(vec_t)1e-5 || line_max <= (vec_t)1e-5)
                    return true;
            }
            return false;
        }

        std::unique_ptr<boundary_poly> clip_boundary_poly(const boundary_poly &input,
                                                           const direct_clip::plane &plane,
                                                           int plane_id)
        {
            if (input.vertices.empty()) return nullptr;
            std::vector<vec_t> side(input.vertices.size());
            vec_t minv = std::numeric_limits<vec_t>::max();
            vec_t maxv = -std::numeric_limits<vec_t>::max();
            for (size_t i = 0; i < input.vertices.size(); i++)
            {
                side[i] = math::dot(input.vertices[i], plane.normal) - plane.dist;
                minv = std::min(minv, side[i]);
                maxv = std::max(maxv, side[i]);
            }
            if (maxv <= boundary_clip_epsilon)
            {
                auto out = std::make_unique<boundary_poly>(input);
                for (size_t i = 0; i < out->vertices.size(); i++)
                    if (std::fabs(side[i]) <= boundary_clip_epsilon)
                        out->incidence[i].insert(plane_id);
                return out;
            }
            if (minv >= -boundary_clip_epsilon)
                return nullptr;

            point_list vertices;
            std::vector<std::set<int>> incidence;
            for (size_t i = 0; i < input.vertices.size(); i++)
            {
                if (side[i] <= boundary_clip_epsilon)
                {
                    vertices.push_back(input.vertices[i]);
                    auto inc = input.incidence[i];
                    if (std::fabs(side[i]) <= boundary_clip_epsilon) inc.insert(plane_id);
                    incidence.push_back(std::move(inc));
                }
            }
            size_t cut_count = 0;
            for (size_t i = 0; i < input.vertices.size(); i++)
                for (size_t j = i + 1; j < input.vertices.size(); j++)
                {
                    if (!((side[i] < -boundary_clip_epsilon && side[j] > boundary_clip_epsilon)
                          || (side[j] < -boundary_clip_epsilon && side[i] > boundary_clip_epsilon)))
                        continue;
                    if (!boundary_edge(input.incidence[i], input.incidence[j])) continue;
                    vec_t t = side[i] / (side[i] - side[j]);
                    vertices.push_back(input.vertices[i] + (input.vertices[j] - input.vertices[i]) * t);
                    std::set<int> inc;
                    std::set_intersection(input.incidence[i].begin(), input.incidence[i].end(),
                                          input.incidence[j].begin(), input.incidence[j].end(),
                                          std::inserter(inc, inc.begin()));
                    inc.insert(plane_id);
                    incidence.push_back(std::move(inc));
                    cut_count++;
                }
            // Recover edges when numerical conditioning dropped one of the
            // incidence planes.  The geometric test accepts only pairs lying
            // on a supporting plane, so it cannot produce the all-pairs
            // combinatorial explosion of the old fallback.
            if (!cut_count)
                for (size_t i = 0; i < input.vertices.size(); i++)
                    for (size_t j = i + 1; j < input.vertices.size(); j++)
                    {
                        if (!((side[i] < -boundary_clip_epsilon && side[j] > boundary_clip_epsilon)
                              || (side[j] < -boundary_clip_epsilon && side[i] > boundary_clip_epsilon)))
                            continue;
                        if (!boundary_geometric_edge(input.vertices, i, j)) continue;
                        vec_t t = side[i] / (side[i] - side[j]);
                        vertices.push_back(input.vertices[i] + (input.vertices[j] - input.vertices[i]) * t);
                        auto inc = input.incidence[i];
                        inc.insert(input.incidence[j].begin(), input.incidence[j].end());
                        inc.insert(plane_id);
                        incidence.push_back(std::move(inc));
                        cut_count++;
                    }
            // Very small, numerically degenerate polyhedra may have lost all
            // incidence edges.  Their complete crossing set is bounded and is
            // safe to recover; never use this path for a large fragment.
            if (!cut_count && input.vertices.size() <= 32)
                for (size_t i = 0; i < input.vertices.size(); i++)
                    for (size_t j = i + 1; j < input.vertices.size(); j++)
                    {
                        if (!((side[i] < -boundary_clip_epsilon && side[j] > boundary_clip_epsilon)
                              || (side[j] < -boundary_clip_epsilon && side[i] > boundary_clip_epsilon)))
                            continue;
                        vec_t t = side[i] / (side[i] - side[j]);
                        vertices.push_back(input.vertices[i] + (input.vertices[j] - input.vertices[i]) * t);
                        auto inc = input.incidence[i];
                        inc.insert(input.incidence[j].begin(), input.incidence[j].end());
                        inc.insert(plane_id);
                        incidence.push_back(std::move(inc));
                        cut_count++;
                    }
            if (!cut_count) return nullptr;
            merge_boundary_vertices(vertices, incidence);
            if (vertices.size() < 4) return nullptr;
            auto out = std::make_unique<boundary_poly>();
            out->vertices = std::move(vertices);
            out->incidence = std::move(incidence);
            rebuild_boundary_bounds(*out);
            return out;
        }

        vec_t boundary_poly_volume(const boundary_poly &poly)
        {
            if (poly.vertices.size() < 4) return 0;
            math::vec3v center{};
            for (const auto &v : poly.vertices) center += v;
            center = center * ((vec_t)1 / (vec_t)poly.vertices.size());
            struct face_group
            {
                math::vec3v normal;
                vec_t dist;
                point_list points;
            };
            std::vector<face_group> groups;
            std::set<int> plane_ids;
            for (const auto &inc : poly.incidence) plane_ids.insert(inc.begin(), inc.end());
            for (int pid : plane_ids)
            {
                std::vector<math::vec3v> points;
                for (size_t i = 0; i < poly.vertices.size(); i++)
                    if (poly.incidence[i].count(pid)) points.push_back(poly.vertices[i]);
                if (points.size() < 3) continue;
                math::vec3v normal;
                bool have_normal = false;
                for (size_t i = 1; i + 1 < points.size() && !have_normal; i++)
                    for (size_t j = i + 1; j < points.size(); j++)
                    {
                        math::cross(points[i] - points[0], points[j] - points[0], normal);
                        if (math::length(normal) > (vec_t)1e-10)
                        {
                            math::normalize(normal);
                            have_normal = true;
                            break;
                        }
                    }
                if (!have_normal) continue;
                vec_t dist = math::dot(normal, points[0]);
                if (math::dot(normal, center) - dist > 0)
                {
                    normal = -normal;
                    dist = -dist;
                }
                face_group *group = nullptr;
                for (auto &candidate : groups)
                    if (math::length(candidate.normal - normal) <= (vec_t)1e-5
                        && std::fabs(candidate.dist - dist) <= (vec_t)1e-4)
                    {
                        group = &candidate;
                        break;
                    }
                if (!group)
                {
                    groups.push_back({normal, dist, {}});
                    group = &groups.back();
                }
                for (const auto &p : points)
                {
                    bool duplicate = false;
                    for (const auto &old : group->points)
                        if (math::length(old - p) <= (vec_t)1e-8) { duplicate = true; break; }
                    if (!duplicate) group->points.push_back(p);
                }
            }

            vec_t volume = 0;
            for (const auto &group : groups)
            {
                const auto &normal = group.normal;
                const auto &points = group.points;
                if (points.size() < 3) continue;
                math::vec3v ref = std::fabs(normal.z) < (vec_t)0.9
                    ? math::vec3v{0, 0, 1} : math::vec3v{1, 0, 0};
                math::vec3v u;
                math::cross(ref, normal, u);
                if (math::normalize(u) == 0) continue;
                math::vec3v v;
                math::cross(normal, u, v);
                struct angled { math::vec3v p; vec_t angle; };
                std::vector<angled> sorted;
                math::vec3v face_center{};
                for (const auto &p : points) face_center += p;
                face_center = face_center * ((vec_t)1 / (vec_t)points.size());
                for (const auto &p : points)
                {
                    const auto delta = p - face_center;
                    sorted.push_back({p, (vec_t)std::atan2(math::dot(delta, v), math::dot(delta, u))});
                }
                std::sort(sorted.begin(), sorted.end(), [](const angled &a, const angled &b) {
                    return a.angle < b.angle;
                });
                for (size_t i = 1; i + 1 < sorted.size(); i++)
                {
                    const auto a = sorted[0].p - center;
                    const auto b = sorted[i].p - center;
                    const auto c = sorted[i + 1].p - center;
                    volume += math::dot(a, math::cross(b, c)) / (vec_t)6;
                }
            }
            return std::fabs(volume);
        }

        int boundary_add_plane(std::vector<direct_clip::plane> &planes,
                               const direct_clip::plane &candidate)
        {
            for (int i = 0; i < (int)planes.size(); i++)
                if (math::length(planes[(size_t)i].normal - candidate.normal) <= boundary_plane_epsilon
                    && std::fabs(planes[(size_t)i].dist - candidate.dist) <= boundary_plane_epsilon)
                    return i;
            planes.push_back(candidate);
            return (int)planes.size() - 1;
        }

        std::vector<direct_clip::plane> normalized_boundary_planes(
            const std::vector<triangle> &triangles)
        {
            std::vector<direct_clip::plane> planes;
            for (const auto &tri : triangles)
                boundary_add_plane(planes, {tri.normal, math::dot(tri.normal, tri.points[0])});

            std::vector<bool> remove(planes.size(), false);
            for (size_t index = 0; index < planes.size(); index++)
            {
                const auto &p = planes[index];
                if (std::fabs(p.normal.z) > boundary_plane_epsilon) continue;
                vec_t max_height = 0;
                bool found = false;
                for (const auto &tri : triangles)
                {
                    bool same_normal = math::length(tri.normal - p.normal) < boundary_plane_epsilon;
                    bool same_dist = std::fabs(math::dot(tri.points[0], p.normal) - p.dist) < (vec_t)1e-6;
                    if (!same_normal || !same_dist) continue;
                    found = true;
                    vec_t span = 0;
                    for (int e = 0; e < 3; e++)
                    {
                        const auto delta = tri.points[(e + 1) % 3] - tri.points[e];
                        span = std::max(span, (vec_t)std::sqrt(delta.x * delta.x + delta.y * delta.y));
                    }
                    math::vec3v cross;
                    math::cross(tri.points[1] - tri.points[0], tri.points[2] - tri.points[0], cross);
                    vec_t twice_area = (vec_t)math::length(cross);
                    if (span > (vec_t)1e-10)
                        max_height = std::max(max_height, twice_area / span);
                }
                if (found && max_height <= boundary_roof_lift)
                    remove[index] = true;
            }

            std::vector<direct_clip::plane> result;
            for (size_t i = 0; i < planes.size(); i++)
                if (!remove[i])
                {
                    auto p = planes[i];
                    p.dist += boundary_roof_lift * std::max(p.normal.z, (vec_t)0);
                    result.push_back(p);
                }
            logging::info("pyboundary: %zu boundary planes, removed %zu micro seams\n",
                          result.size(), planes.size() - result.size());
            return result;
        }

        boundary_poly source_poly(const built_brush &brush,
                                  const csg_result &result, int source_index)
        {
            boundary_poly out;
            for (const auto &face : brush.hulls[0].faces)
                for (const auto &v : face.winding.points())
                {
                    bool duplicate = false;
                    for (const auto &old : out.vertices)
                        if (math::length(v - old) <= boundary_clip_epsilon)
                        {
                            duplicate = true;
                            break;
                        }
                    if (!duplicate) out.vertices.push_back(v);
                }
            out.incidence.resize(out.vertices.size());
            for (size_t i = 0; i < out.vertices.size(); i++)
                for (size_t f = 0; f < brush.hulls[0].faces.size(); f++)
                {
                    const auto &face = brush.hulls[0].faces[f];
                    const auto &p = result.planes[(size_t)face.planenum];
                    if (std::fabs(math::dot(out.vertices[i], p.normal) - p.dist) <= (vec_t)1e-5)
                        out.incidence[i].insert(boundary_source_plane_base + source_index * 1024 + (int)f);
                }
            rebuild_boundary_bounds(out);
            return out;
        }

        std::vector<direct_clip::plane> envelope_planes(const built_brush &brush,
                                                         const csg_result &result,
                                                         int source_index)
        {
            std::vector<direct_clip::plane> result_planes;
            for (const auto &face : brush.hulls[0].faces)
            {
                const auto &p = result.planes[(size_t)face.planenum];
                result_planes.push_back({p.normal,
                    p.dist + boundary_envelope * std::max(p.normal.z, (vec_t)0)});
            }
            std::vector<math::vec3v> xy;
            for (const auto &face : brush.hulls[0].faces)
                for (const auto &v : face.winding.points())
                {
                    bool duplicate = false;
                    for (const auto &old : xy)
                        if (std::fabs(v.x - old.x) <= boundary_clip_epsilon
                            && std::fabs(v.y - old.y) <= boundary_clip_epsilon)
                        { duplicate = true; break; }
                    if (!duplicate) xy.push_back({v.x, v.y, 0});
                }
            std::sort(xy.begin(), xy.end(), [](const math::vec3v &a, const math::vec3v &b) {
                return a.x < b.x || (a.x == b.x && a.y < b.y);
            });
            auto cross2 = [](const math::vec3v &o, const math::vec3v &a, const math::vec3v &b) {
                return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
            };
            std::vector<math::vec3v> hull;
            for (const auto &p : xy)
            {
                while (hull.size() >= 2 && cross2(hull[hull.size() - 2], hull.back(), p) <= 0) hull.pop_back();
                hull.push_back(p);
            }
            size_t lower = hull.size();
            for (size_t i = xy.size(); i-- > 0;)
            {
                const auto &p = xy[i];
                while (hull.size() > lower && cross2(hull[hull.size() - 2], hull.back(), p) <= 0) hull.pop_back();
                hull.push_back(p);
            }
            if (hull.size() > 1) hull.pop_back();
            for (size_t i = 0; i < hull.size(); i++)
            {
                const auto &a = hull[i];
                const auto &b = hull[(i + 1) % hull.size()];
                math::vec3v n{b.y - a.y, -(b.x - a.x), 0};
                if (math::normalize(n) == 0) continue;
                result_planes.push_back({n, math::dot(n, a)});
            }
            (void)source_index;
            return result_planes;
        }

        std::vector<std::unique_ptr<boundary_poly>> boundary_outside_parts(
            const boundary_poly &volume, const std::vector<direct_clip::plane> &planes,
            int plane_base)
        {
            std::vector<std::unique_ptr<boundary_poly>> outside;
            std::unique_ptr<boundary_poly> remaining = std::make_unique<boundary_poly>(volume);
            for (size_t i = 0; i < planes.size() && remaining; i++)
            {
                vec_t minv = std::numeric_limits<vec_t>::max();
                vec_t maxv = -std::numeric_limits<vec_t>::max();
                for (const auto &v : remaining->vertices)
                {
                    vec_t x = math::dot(v, planes[i].normal) - planes[i].dist;
                    minv = std::min(minv, x);
                    maxv = std::max(maxv, x);
                }
                if (maxv <= boundary_clip_epsilon) continue;
                if (minv >= -boundary_clip_epsilon)
                {
                    outside.push_back(std::move(remaining));
                    break;
                }
                auto outside_plane = planes[i];
                outside_plane.normal = -outside_plane.normal;
                outside_plane.dist = -outside_plane.dist;
                auto front = clip_boundary_poly(*remaining, outside_plane,
                                                 plane_base + (int)i);
                if (front) outside.push_back(std::move(front));
                remaining = clip_boundary_poly(*remaining, planes[i], plane_base + (int)i);
            }
            return outside;
        }

        std::vector<std::unique_ptr<boundary_poly>> boundary_outside_fragments(
            const boundary_poly &volume, const std::vector<boundary_volume> &source)
        {
            std::vector<std::unique_ptr<boundary_poly>> remaining;
            remaining.push_back(std::make_unique<boundary_poly>(volume));
            for (size_t brush = 0; brush < source.size() && !remaining.empty(); brush++)
            {
                std::vector<std::unique_ptr<boundary_poly>> next;
                for (const auto &piece : remaining)
                {
                    if (piece->bounds.disjoint(source[brush].bounds))
                    {
                        next.push_back(std::make_unique<boundary_poly>(*piece));
                        continue;
                    }
                    auto parts = boundary_outside_parts(*piece, source[brush].planes,
                                                        boundary_envelope_plane_base + (int)brush * 1024);
                    for (auto &part : parts) next.push_back(std::move(part));
                }
                remaining = std::move(next);
                if (remaining.size() > 10000)
                    err::fatal("pyboundary: source subtraction exceeded the piece budget");
            }
            return remaining;
        }

        std::unique_ptr<boundary_poly> boundary_closure(
            const math::bounding_box &bounds,
            const std::vector<direct_clip::plane> &planes,
            const std::vector<int> &members)
        {
            auto result = std::make_unique<boundary_poly>(make_boundary_box(bounds));
            for (int member : members)
            {
                auto clipped = clip_boundary_poly(*result, planes[(size_t)member], member);
                if (!clipped) return nullptr;
                result = std::move(clipped);
            }
            return result;
        }

        std::vector<int> boundary_members(const boundary_poly &piece,
                                          const std::vector<direct_clip::plane> &planes)
        {
            std::vector<int> members;
            for (size_t i = 0; i < planes.size(); i++)
            {
                vec_t maxv = -std::numeric_limits<vec_t>::max();
                for (const auto &v : piece.vertices)
                    maxv = std::max(maxv, math::dot(v, planes[i].normal) - planes[i].dist);
                if (maxv <= (vec_t)1e-6) members.push_back((int)i);
            }
            return members;
        }

        std::vector<boundary_term> boundary_source_terms(
            const std::vector<direct_clip::plane> &planes,
            const std::vector<boundary_poly> &source_pieces)
        {
            // Large entities use the same exterior-plane union as the Python
            // pipeline, but seed one convex term per source piece.  This is a
            // bounded fallback for pathological adaptive arrangements: it
            // retains only real exposed supporting planes and never allocates
            // the unbounded intermediate fragment set.
            std::set<std::vector<int>> unique;
            for (const auto &piece : source_pieces)
            {
                std::vector<int> members = boundary_members(piece, planes);
                if (!members.empty()) unique.insert(std::move(members));
            }
            std::vector<boundary_term> result;
            for (const auto &members : unique) result.push_back({members});
            // These seed closures are not independently containment-proven;
            // keep every source term instead of applying Boolean absorption.
            return result;
        }

        bool boundary_subset(const std::vector<int> &small, const std::vector<int> &large)
        {
            size_t i = 0, j = 0;
            while (i < small.size() && j < large.size())
            {
                if (small[i] == large[j]) { i++; j++; }
                else if (small[i] > large[j]) j++;
                else return false;
            }
            return i == small.size();
        }

        std::vector<boundary_term> compose_boundary_terms(
            const std::vector<direct_clip::plane> &planes,
            const math::bounding_box &bounds,
            const std::vector<boundary_poly> &source_pieces,
            const std::vector<boundary_volume> &allowed)
        {
            if (source_pieces.size() > 128)
            {
                auto bounded = boundary_source_terms(planes, source_pieces);
                logging::info("pyboundary: bounded source composition, %zu terms\n", bounded.size());
                return bounded;
            }
            std::vector<boundary_poly> pending = source_pieces;
            std::map<std::vector<int>, std::pair<vec_t, math::vec3v>> cache;
            std::map<std::vector<int>, vec_t> accepted;
            int splits = 0;
            int visited = 0;
            while (!pending.empty())
            {
                boundary_poly piece = std::move(pending.back());
                pending.pop_back();
                visited++;
                if (visited > 20000)
                    err::fatal("pyboundary: adaptive composition exceeded the piece budget");
                std::vector<int> members = boundary_members(piece, planes);
                if (accepted.count(members)) continue;
                bool absorbed = false;
                for (const auto &entry : accepted)
                    if (boundary_subset(entry.first, members)) { absorbed = true; break; }
                if (absorbed) continue;

                auto cached = cache.find(members);
                if (cached == cache.end())
                {
                    auto closure = boundary_closure(bounds, planes, members);
                    if (!closure) err::fatal("pyboundary: boundary closure lost a source piece");
                    auto outside = boundary_outside_fragments(*closure, allowed);
                    vec_t excess = 0;
                    math::vec3v witness{};
                    vec_t largest = -1;
                    for (const auto &fragment : outside)
                    {
                        vec_t volume = boundary_poly_volume(*fragment);
                        excess += volume;
                        if (volume > largest)
                        {
                            largest = volume;
                            witness = {};
                            for (const auto &v : fragment->vertices) witness += v;
                            if (!fragment->vertices.empty()) witness = witness * ((vec_t)1 / fragment->vertices.size());
                        }
                    }
                    cached = cache.emplace(members, std::make_pair(excess, witness)).first;
                }
                const vec_t excess = cached->second.first;
                if (excess <= boundary_volume_epsilon)
                {
                    accepted.emplace(members, excess);
                    continue;
                }

                const auto &witness = cached->second.second;
                std::vector<std::pair<vec_t, int>> candidates;
                for (int index = 0; index < (int)planes.size(); index++)
                {
                    vec_t minv = std::numeric_limits<vec_t>::max();
                    vec_t maxv = -std::numeric_limits<vec_t>::max();
                    for (const auto &v : piece.vertices)
                    {
                        vec_t x = math::dot(v, planes[(size_t)index].normal) - planes[(size_t)index].dist;
                        minv = std::min(minv, x); maxv = std::max(maxv, x);
                    }
                    vec_t outside = math::dot(witness, planes[(size_t)index].normal) - planes[(size_t)index].dist;
                    if (minv < -(vec_t)1e-7 && maxv > (vec_t)1e-7 && outside > (vec_t)1e-7)
                        candidates.push_back({outside / std::max(maxv - minv, (vec_t)1e-8), index});
                }
                std::sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
                    return a.first > b.first;
                });
                bool split = false;
                std::unique_ptr<boundary_poly> fallback;
                const vec_t piece_volume = boundary_poly_volume(piece);
                int last_candidate = -1;
                vec_t last_min = 0, last_max = 0;
                for (const auto &candidate : candidates)
                {
                    const int index = candidate.second;
                    last_candidate = index;
                    last_min = std::numeric_limits<vec_t>::max();
                    last_max = -std::numeric_limits<vec_t>::max();
                    for (const auto &v : piece.vertices)
                    {
                        const vec_t x = math::dot(v, planes[(size_t)index].normal) - planes[(size_t)index].dist;
                        last_min = std::min(last_min, x); last_max = std::max(last_max, x);
                    }
                    auto front = clip_boundary_poly(piece, planes[(size_t)index], index);
                    auto back_plane = planes[(size_t)index];
                    back_plane.normal = -back_plane.normal;
                    back_plane.dist = -back_plane.dist;
                    auto back = clip_boundary_poly(piece, back_plane, index);
                    if (front && back)
                    {
                        pending.push_back(std::move(*front));
                        pending.push_back(std::move(*back));
                        splits++;
                        split = true;
                        break;
                    }
                    auto &only = front ? front : back;
                    if (only && std::fabs(boundary_poly_volume(*only) - piece_volume)
                        <= boundary_volume_epsilon && !fallback)
                        fallback = std::move(only);
                }
                if (!split && fallback)
                {
                    pending.push_back(std::move(*fallback));
                    splits++;
                    split = true;
                }
                if (!split)
                    err::fatal("pyboundary: adaptive split could not separate empty space (visited=%d members=%zu excess=%g piecevol=%g candidates=%zu vertices=%zu pid=%d range=%g..%g)",
                               visited, members.size(), (double)excess, (double)piece_volume,
                               candidates.size(), piece.vertices.size(), last_candidate,
                               (double)last_min, (double)last_max);
                if (visited % 100 == 0)
                    logging::info("pyboundary: composed %d pieces, %zu pending, %zu terms\n",
                                  visited, pending.size(), accepted.size());
            }

            std::vector<std::pair<std::vector<int>, vec_t>> sorted;
            for (const auto &entry : accepted) sorted.push_back(entry);
            std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) {
                if (a.first.size() != b.first.size()) return a.first.size() < b.first.size();
                return a.first < b.first;
            });
            std::vector<boundary_term> terms;
            for (const auto &entry : sorted)
            {
                bool covered = false;
                for (const auto &term : terms)
                    if (boundary_subset(term.members, entry.first)) { covered = true; break; }
                if (!covered) terms.push_back({entry.first});
            }
            logging::info("pyboundary: %zu whole-entity terms after absorption (%d splits)\n",
                          terms.size(), splits);
            return terms;
        }

        bool boundary_supporting_plane(const boundary_poly &poly,
                                       const direct_clip::plane &plane)
        {
            std::vector<math::vec3v> face;
            for (const auto &v : poly.vertices)
                if (std::fabs(math::dot(v, plane.normal) - plane.dist) <= (vec_t)1e-5)
                    face.push_back(v);
            if (face.size() < 3) return false;
            for (size_t i = 1; i + 1 < face.size(); i++)
            {
                math::vec3v cross;
                math::cross(face[i] - face[0], face[i + 1] - face[0], cross);
                if (math::length(cross) > (vec_t)1e-8) return true;
            }
            return false;
        }

        direct_clip::tree build_boundary_hull(
            const std::vector<triangle> &triangles,
            const std::vector<const built_brush *> &brushes,
            const csg_result &result,
            const math::vec3v &half)
        {
            direct_clip::tree tree;
            if (triangles.empty() || brushes.empty()) return tree;
            std::vector<direct_clip::plane> boundaries = normalized_boundary_planes(triangles);
            if (boundaries.empty()) return tree;

            math::bounding_box bounds;
            for (const auto &tri : triangles)
                for (const auto &v : tri.points) bounds.add(v);
            bounds.mins -= math::vec3v{100, 100, 100};
            bounds.maxs += math::vec3v{100, 100, 200};

            std::vector<boundary_poly> source_pieces;
            std::vector<boundary_volume> allowed;
            source_pieces.reserve(brushes.size());
            allowed.reserve(brushes.size());
            for (size_t i = 0; i < brushes.size(); i++)
            {
                source_pieces.push_back(source_poly(*brushes[i], result, (int)i));
                // Match boundary_normalize.py's vertical envelope: shifted
                // upward faces are accompanied by the expanded Z bound.
                auto envelope_bounds = brushes[i]->hulls[0].bounds;
                envelope_bounds.maxs.z += boundary_envelope;
                allowed.push_back({envelope_planes(*brushes[i], result, (int)i), envelope_bounds});
            }
            std::vector<boundary_term> terms = compose_boundary_terms(
                boundaries, bounds, source_pieces, allowed);
            if (terms.empty()) return tree;

            plane_table output_planes;
            std::vector<fragment> fragments;
            for (const auto &term : terms)
            {
                std::vector<direct_clip::plane> shifted = boundaries;
                for (auto &p : shifted)
                    p.dist += p.normal.x * p.normal.x * half.x
                        + p.normal.y * p.normal.y * half.y
                        + p.normal.z * p.normal.z * half.z;
                auto poly = boundary_closure(bounds, shifted, term.members);
                if (!poly || poly->vertices.size() < 4) continue;

                fragment f;
                f.vertices = poly->vertices;
                f.incidence.resize(f.vertices.size());
                for (int member : term.members)
                {
                    const auto &p = shifted[(size_t)member];
                    if (!boundary_supporting_plane(*poly, p)) continue;
                    const int pid = output_planes.add(p.normal, p.dist);
                    int count = 0;
                    for (size_t i = 0; i < f.vertices.size(); i++)
                        if (std::fabs(math::dot(f.vertices[i], p.normal) - p.dist) <= fragment_epsilon * 10)
                        {
                            f.incidence[i].insert(pid);
                            count++;
                        }
                    if (count >= 3) f.sides.insert(pid);
                }
                if (f.vertices.size() >= 4 && !f.sides.empty())
                    fragments.push_back(std::move(f));
            }
            if (fragments.empty()) return tree;
            logging::info("pyboundary: %zu terms, %zu active planes, hull half %g %g %g\n",
                          fragments.size(), output_planes.values.size(),
                          (double)half.x, (double)half.y, (double)half.z);
            tree.head = build_tree(std::move(fragments), output_planes, tree.nodes);
            tree.planes = std::move(output_planes.values);
            tree.valid = tree.head >= 0;
            bool have_bounds = false;
            for (const auto &term : terms)
            {
                auto poly = boundary_closure(bounds, boundaries, term.members);
                if (!poly) continue;
                for (const auto &v : poly->vertices)
                    for (const auto &corner : {math::vec3v{-1, -1, -1}, math::vec3v{-1, -1, 1},
                                               math::vec3v{-1, 1, -1}, math::vec3v{-1, 1, 1},
                                               math::vec3v{1, -1, -1}, math::vec3v{1, -1, 1},
                                               math::vec3v{1, 1, -1}, math::vec3v{1, 1, 1}})
                    {
                        math::vec3v p = v + math::vec3v{corner.x * half.x, corner.y * half.y, corner.z * half.z};
                        if (!have_bounds) { tree.mins = tree.maxs = p; have_bounds = true; }
                        else
                        {
                            for (int axis = 0; axis < 3; axis++)
                            {
                                tree.mins[axis] = std::min(tree.mins[axis], p[axis]);
                                tree.maxs[axis] = std::max(tree.maxs[axis], p[axis]);
                            }
                        }
                    }
            }
            return tree;
        }

        direct_clip::tree build_hull(const std::vector<triangle> &triangles,
                                     const math::vec3v &half)
        {
            direct_clip::tree tree;
            plane_table planes;
            std::vector<fragment> fragments;
            for (const triangle &tri : triangles)
                fragments.push_back(make_fragment(planes, tri, half));
            if (fragments.empty()) return tree;
            logging::info("pyexact: fragment input %zu triangles, %zu planes\n",
                          triangles.size(), planes.values.size());
            tree.head = build_tree(std::move(fragments), planes, tree.nodes);
            tree.planes = std::move(planes.values);
            tree.valid = tree.head >= 0;
            bool have_bounds = false;
            for (const triangle &tri : triangles)
                for (const auto &v : tri.points)
                    for (const auto &corner : {math::vec3v{-1, -1, -1}, math::vec3v{-1, -1, 1},
                                               math::vec3v{-1, 1, -1}, math::vec3v{-1, 1, 1},
                                               math::vec3v{1, -1, -1}, math::vec3v{1, -1, 1},
                                               math::vec3v{1, 1, -1}, math::vec3v{1, 1, 1}})
                    {
                        math::vec3v p = v + math::vec3v{corner.x * half.x, corner.y * half.y, corner.z * half.z};
                        if (!have_bounds) { tree.mins = tree.maxs = p; have_bounds = true; }
                        else { tree.mins.x = std::min(tree.mins.x, p.x); tree.mins.y = std::min(tree.mins.y, p.y); tree.mins.z = std::min(tree.mins.z, p.z);
                               tree.maxs.x = std::max(tree.maxs.x, p.x); tree.maxs.y = std::max(tree.maxs.y, p.y); tree.maxs.z = std::max(tree.maxs.z, p.z); }
                    }
            return tree;
        }

    }

    std::vector<direct_clip::model> build_surface_clip_models(
        const csg_result &result, const brush_build_options &options)
    {
        std::vector<direct_clip::model> out;
        out.reserve(result.entities.size());
        for (size_t entity_index = 0; entity_index < result.entities.size(); entity_index++)
        {
            const auto &entity = result.entities[entity_index];
            if (entity.entity_num < 0 || entity.entity_num >= (int)result.map.entities.size()
                || result.map.entities[(size_t)entity.entity_num].num_brushes <= 0)
                continue;
            out.emplace_back();
            direct_clip::model &model = out.back();
            bool any = false, all = true;
            bool boundary_mode = options.clip == clip_type::pyboundary;
            bool exact_mode = options.clip == clip_type::pyexact;
            for (const auto &brush : entity.brushes)
            {
                if (brush.contents == content::origin || brush.contents == content::bounding_box
                    || brush.skipped)
                    continue;
                if (boundary_mode && brush.pyboundary)
                {
                    if (brush.contents != content::solid || brush.hulls[0].faces.empty())
                        continue;
                    any = true;
                    all = all && brush.pyboundary;
                    continue;
                }
                if (boundary_mode)
                    continue;
                if (brush.contents != content::solid || brush.hulls[0].faces.empty()
                    || brush.hulls[1].faces.empty())
                {
                    all = false;
                    continue;
                }
                any = true;
                boundary_mode = boundary_mode || brush.pyboundary;
                exact_mode = exact_mode || brush.pyexact;
                all = all && (brush.pyboundary || brush.pyexact);
            }
            if (!any || !all) continue;
            auto triangles = extract_triangles(result, entity, boundary_mode && !exact_mode);
            if (triangles.empty()) continue;
            if (boundary_mode && !exact_mode)
            {
                std::vector<const built_brush *> brushes;
                for (const auto &brush : entity.brushes)
                    if (brush.pyboundary && brush.contents == content::solid && !brush.skipped
                        && !brush.hulls[0].faces.empty())
                        brushes.push_back(&brush);
                // The composition routine switches to a bounded source-term
                // seed for terrain-sized entities, while retaining the
                // adaptive proof path for small assemblies.
                model.hulls[1] = build_boundary_hull(triangles, brushes, result,
                                                      options.hull_size[1][1]);
                model.hulls[2] = model.hulls[1];
                model.hulls[3] = build_boundary_hull(triangles, brushes, result,
                                                      options.hull_size[3][1]);
            }
            else
            {
                model.hulls[1] = build_hull(triangles, options.hull_size[1][1]);
                model.hulls[2] = model.hulls[1];
                model.hulls[3] = build_hull(triangles, options.hull_size[3][1]);
            }
            model.valid = model.hulls[1].valid && model.hulls[3].valid;
            if (model.valid)
                logging::info("%s: model %zu, %zu exterior triangles, hull1 %zu nodes, hull3 %zu nodes\n",
                              boundary_mode && !exact_mode ? "pyboundary" : "pyexact",
                              out.size() - 1, triangles.size(), model.hulls[1].nodes.size(), model.hulls[3].nodes.size());
        }
        return out;
    }
}

