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
                                                 const built_entity &entity)
        {
            size_t input_faces = 0, exterior_fragments = 0;
            std::vector<const built_brush *> brushes;
            for (const auto &brush : entity.brushes)
                if (brush.contents == content::solid && !brush.hulls[0].faces.empty())
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
            for (const auto &brush : entity.brushes)
            {
                if (brush.contents == content::origin || brush.contents == content::bounding_box
                    || brush.skipped)
                    continue;
                if (brush.contents != content::solid || brush.hulls[0].faces.empty()
                    || brush.hulls[1].faces.empty())
                {
                    all = false;
                    continue;
                }
                any = true;
                all = all && brush.pyexact;
            }
            if (!any || !all) continue;
            auto triangles = extract_triangles(result, entity);
            if (triangles.empty()) continue;
            model.hulls[1] = build_hull(triangles, options.hull_size[1][1]);
            model.hulls[2] = model.hulls[1];
            model.hulls[3] = build_hull(triangles, options.hull_size[3][1]);
            model.valid = model.hulls[1].valid && model.hulls[3].valid;
            if (model.valid)
                logging::info("pyexact: model %zu, %zu exterior triangles, hull1 %zu nodes, hull3 %zu nodes\n",
                              out.size() - 1, triangles.size(), model.hulls[1].nodes.size(), model.hulls[3].nodes.size());
        }
        return out;
    }
}
