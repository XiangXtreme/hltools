#include "surface_clip.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <future>
#include <functional>
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
            using plane_mask = std::vector<uint64_t>;
            std::vector<plane_mask> incidence;
            plane_mask sides;
            plane_mask used;
        };

        struct fragment_split_scratch
        {
            std::vector<vec_t> side;
            point_list cuts;
            std::vector<fragment::plane_mask> cut_incidence;
        };

        struct build_tree_scratch
        {
            std::vector<int> plane_marks;
            int generation = 0;
        };

        using fragment_plane_mask = fragment::plane_mask;

        void fragment_mask_set(fragment_plane_mask &mask, int plane)
        {
            if (plane < 0) return;
            const size_t word = (size_t)plane / 64;
            if (mask.size() <= word) mask.resize(word + 1, 0);
            mask[word] |= uint64_t(1) << (plane % 64);
        }

        bool fragment_mask_test(const fragment_plane_mask &mask, int plane)
        {
            const size_t word = (size_t)plane / 64;
            return word < mask.size() && (mask[word] & (uint64_t(1) << (plane % 64))) != 0;
        }

        int fragment_mask_popcount(uint64_t value)
        {
            int result = 0;
            while (value)
            {
                value &= value - 1;
                result++;
            }
            return result;
        }

        int fragment_mask_first_bit(uint64_t value)
        {
            int result = 0;
            while ((value & uint64_t(1)) == 0) { value >>= 1; result++; }
            return result;
        }

        bool fragment_mask_contains(const fragment_plane_mask &large,
                                    const fragment_plane_mask &small)
        {
            for (size_t i = 0; i < small.size(); i++)
                if (((i >= large.size() ? 0 : large[i]) & small[i]) != small[i])
                    return false;
            return true;
        }

        void fragment_mask_or(fragment_plane_mask &destination,
                              const fragment_plane_mask &source)
        {
            if (destination.size() < source.size()) destination.resize(source.size(), 0);
            for (size_t i = 0; i < source.size(); i++) destination[i] |= source[i];
        }

        fragment_plane_mask fragment_mask_intersection(const fragment_plane_mask &a,
                                                       const fragment_plane_mask &b)
        {
            fragment_plane_mask result(std::min(a.size(), b.size()), 0);
            for (size_t i = 0; i < result.size(); i++) result[i] = a[i] & b[i];
            return result;
        }

        int fragment_mask_common_count(const fragment_plane_mask &a,
                                       const fragment_plane_mask &b)
        {
            const size_t words = std::min(a.size(), b.size());
            int count = 0;
            for (size_t i = 0; i < words; i++)
                count += fragment_mask_popcount(a[i] & b[i]);
            return count;
        }

        void fragment_mask_intersection_into(fragment_plane_mask &result,
                                              const fragment_plane_mask &a,
                                              const fragment_plane_mask &b)
        {
            const size_t words = std::min(a.size(), b.size());
            result.resize(words);
            for (size_t i = 0; i < words; i++) result[i] = a[i] & b[i];
        }

        void fragment_normalize_masks(std::vector<fragment> &fragments, size_t plane_count)
        {
            const size_t words = (plane_count + 63) / 64;
            for (auto &fragment : fragments)
            {
                fragment.sides.resize(words, 0);
                fragment.used.resize(words, 0);
                for (auto &incidence : fragment.incidence) incidence.resize(words, 0);
            }
        }

        struct boundary_plane_ranges
        {
            std::vector<std::pair<vec_t, vec_t>> values;
            std::vector<size_t> min_indices;
            std::vector<size_t> max_indices;
            std::vector<int> members;
        };

        using boundary_edge_key = std::pair<size_t, size_t>;

        // Convex polyhedron used by the boundary-normalized composition.
        // Incidence records are deliberately retained: the same two-plane
        // edge rule is used by brushbsp.py when a splitter cuts a fragment.
        struct boundary_poly
        {
            using incidence_list = std::vector<int>;
            point_list vertices;
            std::vector<incidence_list> incidence;
            std::shared_ptr<std::map<int, direct_clip::plane>> constraints;
            std::shared_ptr<boundary_plane_ranges> boundary_ranges;
            mutable std::shared_ptr<std::set<boundary_edge_key>> hull_edges;
            math::bounding_box bounds;
        };

        struct boundary_volume
        {
            std::vector<direct_clip::plane> planes;
            math::bounding_box bounds;
        };

        void boundary_incidence_add(boundary_poly::incidence_list &ids, int plane)
        {
            const auto found = std::lower_bound(ids.begin(), ids.end(), plane);
            if (found == ids.end() || *found != plane) ids.insert(found, plane);
        }

        bool boundary_incidence_contains(const boundary_poly::incidence_list &ids, int plane)
        {
            return std::binary_search(ids.begin(), ids.end(), plane);
        }

        void boundary_incidence_union(boundary_poly::incidence_list &destination,
                                      const boundary_poly::incidence_list &source)
        {
            boundary_poly::incidence_list merged;
            merged.reserve(destination.size() + source.size());
            std::set_union(destination.begin(), destination.end(),
                           source.begin(), source.end(), std::back_inserter(merged));
            destination = std::move(merged);
        }

        void boundary_incidence_intersection(boundary_poly::incidence_list &destination,
                                             const boundary_poly::incidence_list &a,
                                             const boundary_poly::incidence_list &b)
        {
            destination.clear();
            destination.reserve(std::min(a.size(), b.size()));
            std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                                  std::back_inserter(destination));
        }

        // Static broad-phase index for the closed brush assembly.  The
        // subtraction order remains the original source order, so this only
        // removes provably disjoint brushes and cannot change the generated
        // fragments or their floating-point path.
        class boundary_volume_index
        {
            struct node
            {
                math::bounding_box bounds;
                int left = -1;
                int right = -1;
                std::vector<int> items;
            };

            std::vector<math::bounding_box> item_bounds;
            std::vector<node> nodes;

            int build(std::vector<int> items)
            {
                node current;
                for (int index : items) current.bounds.add(item_bounds[(size_t)index]);
                const int result = (int)nodes.size();
                nodes.push_back(std::move(current));
                if (items.size() <= 8)
                {
                    nodes[(size_t)result].items = std::move(items);
                    return result;
                }

                math::bounding_box centers;
                for (int index : items)
                {
                    const auto &bounds = item_bounds[(size_t)index];
                    centers.add((bounds.mins + bounds.maxs) * (vec_t)0.5);
                }
                int axis = 0;
                if (centers.maxs.y - centers.mins.y > centers.maxs.x - centers.mins.x) axis = 1;
                if (centers.maxs.z - centers.mins.z > centers.maxs[axis] - centers.mins[axis]) axis = 2;
                std::sort(items.begin(), items.end(), [&](int a, int b) {
                    const auto &aa = item_bounds[(size_t)a];
                    const auto &bb = item_bounds[(size_t)b];
                    return (aa.mins[axis] + aa.maxs[axis]) < (bb.mins[axis] + bb.maxs[axis]);
                });
                const size_t middle = items.size() / 2;
                std::vector<int> left(items.begin(), items.begin() + middle);
                std::vector<int> right(items.begin() + middle, items.end());
                nodes[(size_t)result].left = build(std::move(left));
                nodes[(size_t)result].right = build(std::move(right));
                return result;
            }

            void query_node(int node_index, const math::bounding_box &bounds,
                            std::vector<int> &result) const
            {
                const auto &current = nodes[(size_t)node_index];
                if (current.bounds.disjoint(bounds)) return;
                if (current.left < 0)
                {
                    for (int item : current.items)
                        if (!item_bounds[(size_t)item].disjoint(bounds)) result.push_back(item);
                    return;
                }
                query_node(current.left, bounds, result);
                query_node(current.right, bounds, result);
            }

        public:
            explicit boundary_volume_index(const std::vector<math::bounding_box> &bounds)
                : item_bounds(bounds)
            {
                if (!item_bounds.empty())
                {
                    std::vector<int> items(item_bounds.size());
                    for (size_t i = 0; i < items.size(); i++) items[i] = (int)i;
                    build(std::move(items));
                }
            }

            std::vector<int> query(const math::bounding_box &bounds) const
            {
                std::vector<int> result;
                if (!nodes.empty()) query_node(0, bounds, result);
                std::sort(result.begin(), result.end());
                return result;
            }
        };

        struct boundary_term
        {
            std::vector<int> members;
        };

        // The adaptive split topology depends on the normalized exterior
        // surface, but not on the hull expansion.  Keep it once per entity so
        // hull 1 and hull 3 can reuse the same composition work.
        struct boundary_composition
        {
            std::vector<direct_clip::plane> boundaries;
            math::bounding_box bounds;
            std::vector<boundary_term> terms;
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
            std::vector<math::bounding_box> volume_bounds;
            volume_bounds.reserve(volumes.size());
            for (const auto &volume : volumes) volume_bounds.push_back(volume.bounds);
            const boundary_volume_index volume_index(volume_bounds);

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
                    const auto candidates = volume_index.query(face_bounds);
                    for (int other_index : candidates)
                    {
                        const size_t other = (size_t)other_index;
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
                        fragment_mask_set(out.incidence[i], id);
            }
            for (int id = 0; id < (int)planes.values.size(); id++)
            {
                int count = 0;
                for (const auto &inc : out.incidence)
                    count += fragment_mask_test(inc, id) ? 1 : 0;
                if (count >= 3) fragment_mask_set(out.sides, id);
            }
            return out;
        }

        void merge_vertices(point_list &vertices, std::vector<fragment_plane_mask> &incidence)
        {
            point_list new_vertices;
            std::vector<fragment_plane_mask> new_incidence;
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
                    fragment_mask_or(dst, incidence[i]);
                }
            }
            vertices = std::move(new_vertices);
            incidence = std::move(new_incidence);
        }

        std::pair<std::unique_ptr<fragment>, std::unique_ptr<fragment>>
        split_fragment(const fragment &input, int pid, const direct_clip::plane &plane,
                       fragment_split_scratch &scratch)
        {
            auto &side = scratch.side;
            side.resize(input.vertices.size());
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
                if (fragment_mask_test(input.sides, pid)) fragment_mask_set(copy->used, pid);
                if (maxv <= fragment_epsilon)
                    return {nullptr, std::move(copy)};
                return {std::move(copy), nullptr};
            }

            auto &cuts = scratch.cuts;
            auto &cut_incidence = scratch.cut_incidence;
            cuts.clear();
            cut_incidence.clear();
            for (size_t i = 0; i < input.vertices.size(); i++)
                if (side[i] < -fragment_epsilon)
                    for (size_t j = 0; j < input.vertices.size(); j++)
                        if (side[j] > fragment_epsilon)
                        {
                            if (fragment_mask_common_count(input.incidence[i], input.incidence[j]) < 2)
                                continue;
                            vec_t t = side[i] / (side[i] - side[j]);
                            cuts.push_back(input.vertices[i] + (input.vertices[j] - input.vertices[i]) * t);
                            fragment_plane_mask inc;
                            fragment_mask_intersection_into(inc, input.incidence[i], input.incidence[j]);
                            fragment_mask_set(inc, pid);
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
                        fragment_mask_set(out.incidence.back(), pid);
                }
                out.vertices.insert(out.vertices.end(), cuts.begin(), cuts.end());
                out.incidence.insert(out.incidence.end(), cut_incidence.begin(), cut_incidence.end());
                merge_vertices(out.vertices, out.incidence);
                for (size_t word_index = 0; word_index < input.sides.size(); word_index++)
                {
                    uint64_t bits = input.sides[word_index];
                    while (bits)
                    {
                        const int bit = fragment_mask_first_bit(bits);
                        const int old = (int)(word_index * 64 + bit);
                        int count = 0;
                        for (const auto &inc : out.incidence)
                            count += fragment_mask_test(inc, old) ? 1 : 0;
                        if (count >= 3) fragment_mask_set(out.sides, old);
                        bits &= bits - 1;
                    }
                }
                fragment_mask_set(out.sides, pid);
                out.used = input.used;
                fragment_mask_set(out.used, pid);
            }
            return {std::move(result[0]), std::move(result[1])};
        }

        int build_tree(std::vector<fragment> fragments, plane_table &planes,
                       std::vector<direct_clip::node> &nodes,
                       build_tree_scratch &scratch, int depth = 0)
        {
            if (fragments.empty()) return -1;
            if (depth > 4096)
                err::fatal("pyexact: fragment BSP depth exceeded");
            for (const auto &f : fragments)
                if (fragment_mask_contains(f.used, f.sides)) return -2;

            std::vector<int> candidates;
            if (scratch.plane_marks.size() != planes.values.size())
                scratch.plane_marks.assign(planes.values.size(), 0);
            if (++scratch.generation == std::numeric_limits<int>::max())
            {
                std::fill(scratch.plane_marks.begin(), scratch.plane_marks.end(), 0);
                scratch.generation = 1;
            }
            for (const auto &f : fragments)
                for (size_t word_index = 0; word_index < f.sides.size(); word_index++)
                {
                    const uint64_t used = word_index < f.used.size() ? f.used[word_index] : 0;
                    uint64_t bits = f.sides[word_index] & ~used;
                    while (bits)
                    {
                        const int plane = (int)(word_index * 64 + fragment_mask_first_bit(bits));
                        if (plane < (int)scratch.plane_marks.size()
                            && scratch.plane_marks[(size_t)plane] != scratch.generation)
                        {
                            scratch.plane_marks[(size_t)plane] = scratch.generation;
                            candidates.push_back(plane);
                        }
                        bits &= bits - 1;
                    }
                }
            if (candidates.empty()) return -2;
            // Match brushbsp.py's bounded splitter survey.  Without this cap
            // a large terrain shell spends most of its time scoring every
            // historical facet plane at every recursive cell.
            if (candidates.size() > 12)
            {
                std::mt19937 rng(0x5eedu + (unsigned)depth);
                std::shuffle(candidates.begin(), candidates.end(), rng);
                candidates.resize(12);
            }

            int best = -1;
            int best_score = std::numeric_limits<int>::max();
            int best_front_count = 0, best_back_count = 0, best_cut_count = 0;
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
                if (score < best_score)
                {
                    best_score = score;
                    best = pid;
                    best_front_count = front;
                    best_back_count = back;
                    best_cut_count = cut;
                }
            }

            std::vector<fragment> front, back;
            front.reserve((size_t)best_front_count + (size_t)best_cut_count);
            back.reserve((size_t)best_back_count + (size_t)best_cut_count);
            fragment_split_scratch split_scratch;
            const auto &p = planes.values[(size_t)best];
            for (const auto &f : fragments)
            {
                auto split = split_fragment(f, best, p, split_scratch);
                if (split.first) front.push_back(std::move(*split.first));
                if (split.second) back.push_back(std::move(*split.second));
            }
            int index = (int)nodes.size();
            if (nodes.size() >= 500000)
                err::fatal("pyexact: fragment BSP exceeded the internal node budget (fragments=%zu, depth=%d)", fragments.size(), depth);
            nodes.push_back({best, -1, -1});
            nodes[(size_t)index].front = build_tree(std::move(front), planes, nodes, scratch, depth + 1);
            nodes[(size_t)index].back = build_tree(std::move(back), planes, nodes, scratch, depth + 1);
            return index;
        }

        // brushbsp.py compacts its fragment tree before it is spliced into
        // the final BSP.  The uncompressed recursive builder can reach the
        // same logical leaf through many paths, especially after adjacent
        // triangle terms have been split by shared planes.  Fold identical
        // children and structurally identical subtrees while preserving the
        // exact plane and front/back semantics.
        void compact_direct_tree(direct_clip::tree &tree)
        {
            if (!tree.valid || tree.head < 0 || tree.nodes.empty()) return;

            std::vector<direct_clip::node> ordered;
            ordered.reserve(tree.nodes.size());
            std::map<int, int> memo;
            std::map<std::tuple<int, int, int>, int> cache;
            std::set<int> visiting;

            std::function<int(int)> visit = [&](int index) -> int {
                if (index < 0) return index;
                if ((size_t)index >= tree.nodes.size())
                    err::fatal("pyboundary: invalid fragment tree child %d", index);
                auto known = memo.find(index);
                if (known != memo.end()) return known->second;
                if (!visiting.insert(index).second)
                    err::fatal("pyboundary: cyclic fragment tree at node %d", index);
                const auto &node = tree.nodes[(size_t)index];
                const int front = visit(node.front);
                const int back = visit(node.back);
                int value;
                if (front == back)
                    value = front;
                else
                {
                    const auto key = std::make_tuple(node.plane, front, back);
                    auto found = cache.find(key);
                    if (found != cache.end()) value = found->second;
                    else
                    {
                        value = (int)ordered.size();
                        cache.emplace(key, value);
                        ordered.push_back({node.plane, front, back});
                    }
                }
                visiting.erase(index);
                memo.emplace(index, value);
                return value;
            };

            const int root = visit(tree.head);
            auto remap = [&](int index) {
                return index >= 0 ? (int)ordered.size() - 1 - index : index;
            };
            std::vector<direct_clip::node> compacted;
            compacted.reserve(ordered.size());
            for (auto it = ordered.rbegin(); it != ordered.rend(); ++it)
                compacted.push_back({it->plane, remap(it->front), remap(it->back)});
            const size_t before = tree.nodes.size();
            tree.nodes = std::move(compacted);
            tree.head = remap(root);
            logging::info("pyboundary: compacted direct tree %zu -> %zu nodes\n",
                          before, tree.nodes.size());
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

        void compact_boundary_vertices(boundary_poly &poly)
        {
            if (poly.vertices.size() < 5 || !poly.constraints || poly.constraints->empty()) return;
            point_list vertices;
            std::vector<boundary_poly::incidence_list> incidence;
            vertices.reserve(poly.vertices.size());
            incidence.reserve(poly.incidence.size());
            for (size_t i = 0; i < poly.vertices.size(); i++)
            {
                const auto &ids = poly.incidence[i];
                bool corner = false;
                std::vector<const direct_clip::plane *> planes;
                for (int id : ids)
                {
                    auto it = poly.constraints->find(id);
                    if (it != poly.constraints->end()) planes.push_back(&it->second);
                }
                for (size_t a = 0; a < planes.size() && !corner; a++)
                    for (size_t b = a + 1; b < planes.size() && !corner; b++)
                    {
                        math::vec3v cross;
                        math::cross(planes[a]->normal, planes[b]->normal, cross);
                        if (math::length(cross) <= (vec_t)1e-9) continue;
                        for (size_t c = b + 1; c < planes.size(); c++)
                        {
                            if (std::fabs(math::dot(cross, planes[c]->normal)) > (vec_t)1e-9)
                            {
                                corner = true;
                                break;
                            }
                        }
                    }
                // A convex 3D vertex must be incident to three independent
                // half-spaces.  Points generated by an all-pair recovery
                // that lie in the interior of a face therefore disappear,
                // matching Qhull's vertex reduction.
                if (planes.size() < 3 || !corner)
                    continue;
                vertices.push_back(poly.vertices[i]);
                incidence.push_back(poly.incidence[i]);
            }
            if (vertices.size() >= 4)
            {
                poly.vertices = std::move(vertices);
                poly.incidence = std::move(incidence);
                rebuild_boundary_bounds(poly);
            }
        }

        void merge_boundary_vertices(point_list &vertices,
                                      std::vector<boundary_poly::incidence_list> &incidence)
        {
            point_list merged;
            std::vector<boundary_poly::incidence_list> merged_incidence;
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
                    boundary_incidence_union(merged_incidence[found], incidence[i]);
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
            out.constraints = std::make_shared<std::map<int, direct_clip::plane>>(std::initializer_list<
                std::pair<const int, direct_clip::plane>>{
                {boundary_box_plane_base + 0, {{1, 0, 0}, hi.x}},
                {boundary_box_plane_base + 1, {{-1, 0, 0}, -lo.x}},
                {boundary_box_plane_base + 2, {{0, 1, 0}, hi.y}},
                {boundary_box_plane_base + 3, {{0, -1, 0}, -lo.y}},
                {boundary_box_plane_base + 4, {{0, 0, 1}, hi.z}},
                {boundary_box_plane_base + 5, {{0, 0, -1}, -lo.z}},
            });
            for (size_t i = 0; i < out.vertices.size(); i++)
            {
                const auto &v = out.vertices[i];
                if (std::fabs(v.x - hi.x) <= boundary_clip_epsilon) boundary_incidence_add(out.incidence[i], boundary_box_plane_base + 0);
                if (std::fabs(v.x - lo.x) <= boundary_clip_epsilon) boundary_incidence_add(out.incidence[i], boundary_box_plane_base + 1);
                if (std::fabs(v.y - hi.y) <= boundary_clip_epsilon) boundary_incidence_add(out.incidence[i], boundary_box_plane_base + 2);
                if (std::fabs(v.y - lo.y) <= boundary_clip_epsilon) boundary_incidence_add(out.incidence[i], boundary_box_plane_base + 3);
                if (std::fabs(v.z - hi.z) <= boundary_clip_epsilon) boundary_incidence_add(out.incidence[i], boundary_box_plane_base + 4);
                if (std::fabs(v.z - lo.z) <= boundary_clip_epsilon) boundary_incidence_add(out.incidence[i], boundary_box_plane_base + 5);
            }
            rebuild_boundary_bounds(out);
            return out;
        }

        bool boundary_edge(const boundary_poly::incidence_list &a,
                           const boundary_poly::incidence_list &b)
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

        void boundary_add_edge(std::set<boundary_edge_key> &edges, size_t a, size_t b)
        {
            if (a == b) return;
            if (a > b) std::swap(a, b);
            edges.insert({a, b});
        }

        const std::set<boundary_edge_key> &boundary_hull_edges(const boundary_poly &poly)
        {
            if (poly.hull_edges) return *poly.hull_edges;
            std::set<boundary_edge_key> edges;
            std::map<int, std::vector<size_t>> faces;
            for (size_t i = 0; i < poly.incidence.size(); i++)
                for (int plane : poly.incidence[i])
                    faces[plane].push_back(i);

            // In the normal case every true edge is already identified by
            // two supporting planes.  Build that inexpensive edge set first;
            // the 2D face reconstruction below is only needed for a face
            // whose incidence lost one of those supporting IDs.
            for (size_t i = 0; i < poly.vertices.size(); i++)
                for (size_t j = i + 1; j < poly.vertices.size(); j++)
                    if (boundary_edge(poly.incidence[i], poly.incidence[j]))
                        boundary_add_edge(edges, i, j);

            // Rebuild the actual convex boundary of each supporting face in
            // 2D.  This is the equivalent of the edges exposed by
            // scipy.spatial.ConvexHull.simplices in the Python clipper; it
            // discards coplanar interior points and face diagonals.
            for (const auto &entry : faces)
            {
                const auto &indices = entry.second;
                if (indices.size() < 3) continue;
                bool complete = true;
                for (size_t index : indices)
                {
                    int degree = 0;
                    for (size_t other : indices)
                        if (other != index && edges.count(index < other
                                                         ? boundary_edge_key{index, other}
                                                         : boundary_edge_key{other, index}))
                            degree++;
                    if (degree < 2)
                    {
                        complete = false;
                        break;
                    }
                }
                if (complete) continue;
                math::vec3v normal{};
                bool have_normal = false;
                for (size_t i = 1; i + 1 < indices.size() && !have_normal; i++)
                    for (size_t j = i + 1; j < indices.size(); j++)
                    {
                        math::cross(poly.vertices[indices[i]] - poly.vertices[indices[0]],
                                    poly.vertices[indices[j]] - poly.vertices[indices[0]], normal);
                        if (math::length(normal) > (vec_t)1e-10)
                        {
                            math::normalize(normal);
                            have_normal = true;
                            break;
                        }
                    }
                if (!have_normal) continue;

                const math::vec3v ref = std::fabs(normal.z) < (vec_t)0.9
                    ? math::vec3v{0, 0, 1} : math::vec3v{1, 0, 0};
                math::vec3v u;
                math::cross(ref, normal, u);
                if (math::normalize(u) == 0) continue;
                math::vec3v v;
                math::cross(normal, u, v);

                struct projected
                {
                    size_t index;
                    vec_t x, y;
                };
                std::vector<projected> points;
                points.reserve(indices.size());
                const auto origin = poly.vertices[indices[0]];
                for (size_t index : indices)
                {
                    const auto delta = poly.vertices[index] - origin;
                    points.push_back({index, math::dot(delta, u), math::dot(delta, v)});
                }
                std::sort(points.begin(), points.end(), [](const projected &a, const projected &b) {
                    return a.x < b.x || (a.x == b.x && a.y < b.y);
                });
                auto cross2 = [](const projected &o, const projected &a, const projected &b) {
                    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
                };
                std::vector<projected> hull;
                for (const auto &point : points)
                {
                    while (hull.size() >= 2
                           && cross2(hull[hull.size() - 2], hull.back(), point) <= (vec_t)1e-9)
                        hull.pop_back();
                    hull.push_back(point);
                }
                const size_t lower = hull.size();
                for (size_t i = points.size(); i-- > 0;)
                {
                    const auto &point = points[i];
                    while (hull.size() > lower
                           && cross2(hull[hull.size() - 2], hull.back(), point) <= (vec_t)1e-9)
                        hull.pop_back();
                    hull.push_back(point);
                }
                if (hull.size() > 1) hull.pop_back();
                if (hull.size() < 2) continue;
                for (size_t i = 0; i < hull.size(); i++)
                    boundary_add_edge(edges, hull[i].index, hull[(i + 1) % hull.size()].index);
            }

            poly.hull_edges = std::make_shared<std::set<boundary_edge_key>>(std::move(edges));
            return *poly.hull_edges;
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
                if (!out->constraints) out->constraints = std::make_shared<std::map<int, direct_clip::plane>>();
                if (!out->constraints->count(plane_id))
                    out->constraints = std::make_shared<std::map<int, direct_clip::plane>>(*out->constraints);
                (*out->constraints)[plane_id] = plane;
                for (size_t i = 0; i < out->vertices.size(); i++)
                    if (std::fabs(side[i]) <= boundary_clip_epsilon)
                        boundary_incidence_add(out->incidence[i], plane_id);
                return out;
            }
            if (minv >= -boundary_clip_epsilon)
                return nullptr;

            point_list vertices;
            std::vector<boundary_poly::incidence_list> incidence;
            auto constraints = input.constraints;
            if (!constraints) constraints = std::make_shared<std::map<int, direct_clip::plane>>();
            if (!constraints->count(plane_id))
                constraints = std::make_shared<std::map<int, direct_clip::plane>>(*constraints);
            (*constraints)[plane_id] = plane;
            const auto &hull_edges = boundary_hull_edges(input);
            for (size_t i = 0; i < input.vertices.size(); i++)
            {
                if (side[i] <= boundary_clip_epsilon)
                {
                    vertices.push_back(input.vertices[i]);
                    auto inc = input.incidence[i];
                    if (std::fabs(side[i]) <= boundary_clip_epsilon)
                        boundary_incidence_add(inc, plane_id);
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
                    if (!hull_edges.count({i, j})) continue;
                    vec_t t = side[i] / (side[i] - side[j]);
                    vertices.push_back(input.vertices[i] + (input.vertices[j] - input.vertices[i]) * t);
                    boundary_poly::incidence_list inc;
                    boundary_incidence_intersection(inc, input.incidence[i], input.incidence[j]);
                    boundary_incidence_add(inc, plane_id);
                    incidence.push_back(std::move(inc));
                    cut_count++;
                }
            // Recover an edge when a face was represented by too few
            // incidence IDs.  This is intentionally limited to the actual
            // supporting-face hull edges, never the complete crossing pair
            // set that caused the previous arrangement explosion.
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
                        boundary_poly::incidence_list inc;
                        boundary_incidence_intersection(inc, input.incidence[i], input.incidence[j]);
                        boundary_incidence_add(inc, plane_id);
                        incidence.push_back(std::move(inc));
                        cut_count++;
                    }
            // A closure can contain a numerically coplanar face whose
            // incidence basis has disappeared entirely.  Python's Qhull
            // still exposes an edge in that case.  Recover the bounded
            // crossing set as a last resort for one such small polyhedron;
            // this path is unreachable for the large fragment stream and is
            // capped to avoid reintroducing its quadratic explosion.
            if (!cut_count && input.vertices.size() <= 128)
                for (size_t i = 0; i < input.vertices.size(); i++)
                    for (size_t j = i + 1; j < input.vertices.size(); j++)
                    {
                        if (!((side[i] < -boundary_clip_epsilon && side[j] > boundary_clip_epsilon)
                              || (side[j] < -boundary_clip_epsilon && side[i] > boundary_clip_epsilon)))
                            continue;
                        vec_t t = side[i] / (side[i] - side[j]);
                        vertices.push_back(input.vertices[i] + (input.vertices[j] - input.vertices[i]) * t);
                        boundary_poly::incidence_list inc;
                        boundary_incidence_intersection(inc, input.incidence[i], input.incidence[j]);
                        boundary_incidence_add(inc, plane_id);
                        incidence.push_back(std::move(inc));
                        cut_count++;
                    }
            if (!cut_count) return nullptr;
            merge_boundary_vertices(vertices, incidence);
            if (vertices.size() < 4) return nullptr;
            auto out = std::make_unique<boundary_poly>();
            out->vertices = std::move(vertices);
            out->incidence = std::move(incidence);
            out->constraints = std::move(constraints);
            compact_boundary_vertices(*out);
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
                    if (boundary_incidence_contains(poly.incidence[i], pid))
                        points.push_back(poly.vertices[i]);
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
                    if (!out.constraints) out.constraints = std::make_shared<std::map<int, direct_clip::plane>>();
                    (*out.constraints)[boundary_source_plane_base + source_index * 1024 + (int)f] =
                        direct_clip::plane{p.normal, p.dist};
                    if (std::fabs(math::dot(out.vertices[i], p.normal) - p.dist) <= (vec_t)1e-5)
                        boundary_incidence_add(out.incidence[i],
                                               boundary_source_plane_base + source_index * 1024 + (int)f);
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
            boundary_poly volume, const std::vector<direct_clip::plane> &planes,
            int plane_base)
        {
            std::vector<std::unique_ptr<boundary_poly>> outside;
            outside.reserve(planes.size());
            std::unique_ptr<boundary_poly> remaining = std::make_unique<boundary_poly>(std::move(volume));
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
            const boundary_poly &volume, const std::vector<boundary_volume> &source,
            const boundary_volume_index &source_index)
        {
            std::vector<std::unique_ptr<boundary_poly>> remaining;
            remaining.push_back(std::make_unique<boundary_poly>(volume));
            for (int brush_index : source_index.query(volume.bounds))
            {
                if (remaining.empty()) break;
                const size_t brush = (size_t)brush_index;
                std::vector<std::unique_ptr<boundary_poly>> next;
                next.reserve(remaining.size());
                for (auto &piece : remaining)
                {
                    if (piece->bounds.disjoint(source[brush].bounds))
                    {
                        next.push_back(std::move(piece));
                        continue;
                    }
                    auto parts = boundary_outside_parts(std::move(*piece), source[brush].planes,
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
            for (size_t step = 0; step < members.size(); step++)
            {
                const int member = members[step];
                auto clipped = clip_boundary_poly(*result, planes[(size_t)member], member);
                if (!clipped)
                {
                    logging::info("pyboundary: closure clip empty member=%d step=%zu/%zu vertices=%zu\n",
                                  member, step, members.size(), result->vertices.size());
                    return nullptr;
                }
                result = std::move(clipped);
            }
            return result;
        }

        boundary_plane_ranges boundary_plane_ranges_for(
            const boundary_poly &piece,
            const std::vector<direct_clip::plane> &planes)
        {
            boundary_plane_ranges ranges;
            ranges.values.resize(planes.size(), {
                std::numeric_limits<vec_t>::max(), -std::numeric_limits<vec_t>::max()});
            ranges.min_indices.resize(planes.size(), 0);
            ranges.max_indices.resize(planes.size(), 0);
            ranges.members.reserve(planes.size());
            for (size_t i = 0; i < planes.size(); i++)
            {
                for (size_t vertex = 0; vertex < piece.vertices.size(); vertex++)
                {
                    const auto &v = piece.vertices[vertex];
                    const vec_t value = math::dot(v, planes[i].normal) - planes[i].dist;
                    if (value < ranges.values[i].first)
                    {
                        ranges.values[i].first = value;
                        ranges.min_indices[i] = vertex;
                    }
                    if (value > ranges.values[i].second)
                    {
                        ranges.values[i].second = value;
                        ranges.max_indices[i] = vertex;
                    }
                }
                if (ranges.values[i].second <= (vec_t)1e-6) ranges.members.push_back((int)i);
            }
            return ranges;
        }

        std::shared_ptr<boundary_plane_ranges> boundary_ranges_after_clip(
            const boundary_poly &input,
            const boundary_poly &output,
            const std::vector<direct_clip::plane> &planes)
        {
            if (!input.boundary_ranges)
                return std::make_shared<boundary_plane_ranges>(
                    boundary_plane_ranges_for(output, planes));

            const auto &old = *input.boundary_ranges;
            auto result = std::make_shared<boundary_plane_ranges>();
            result->values = old.values;
            result->min_indices.resize(old.min_indices.size(), 0);
            result->max_indices.resize(old.max_indices.size(), 0);
            result->members.reserve(old.members.size());
            std::vector<int> output_for_input(input.vertices.size(), -1);
            for (size_t old_index = 0; old_index < input.vertices.size(); old_index++)
                for (size_t new_index = 0; new_index < output.vertices.size(); new_index++)
                    if (math::length(input.vertices[old_index] - output.vertices[new_index]) <= (vec_t)1e-7)
                    {
                        output_for_input[old_index] = (int)new_index;
                        break;
                    }
            for (size_t i = 0; i < planes.size(); i++)
            {
                const int min_index = old.min_indices[i] < output_for_input.size()
                    ? output_for_input[old.min_indices[i]] : -1;
                const int max_index = old.max_indices[i] < output_for_input.size()
                    ? output_for_input[old.max_indices[i]] : -1;
                if (min_index >= 0)
                    result->min_indices[i] = (size_t)min_index;
                if (max_index >= 0)
                    result->max_indices[i] = (size_t)max_index;
                if (min_index < 0 || max_index < 0)
                {
                    vec_t minv = std::numeric_limits<vec_t>::max();
                    vec_t maxv = -std::numeric_limits<vec_t>::max();
                    size_t min_vertex = 0;
                    size_t max_vertex = 0;
                    for (size_t vertex = 0; vertex < output.vertices.size(); vertex++)
                    {
                        const auto &v = output.vertices[vertex];
                        const vec_t value = math::dot(v, planes[i].normal) - planes[i].dist;
                        if (value < minv) { minv = value; min_vertex = vertex; }
                        if (value > maxv) { maxv = value; max_vertex = vertex; }
                    }
                    if (min_index < 0) { result->values[i].first = minv; result->min_indices[i] = min_vertex; }
                    if (max_index < 0) { result->values[i].second = maxv; result->max_indices[i] = max_vertex; }
                }
                if (result->values[i].second <= (vec_t)1e-6)
                    result->members.push_back((int)i);
            }
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
            std::vector<boundary_poly> pending = source_pieces;
            std::map<std::vector<int>, std::pair<vec_t, math::vec3v>> cache;
            std::map<std::vector<int>, vec_t> accepted;
            std::vector<math::bounding_box> allowed_bounds;
            allowed_bounds.reserve(allowed.size());
            for (const auto &volume : allowed) allowed_bounds.push_back(volume.bounds);
            const boundary_volume_index allowed_index(allowed_bounds);
            int splits = 0;
            int visited = 0;
            while (!pending.empty())
            {
                boundary_poly piece = std::move(pending.back());
                pending.pop_back();
                visited++;
                if (visited > 20000)
                    err::fatal("pyboundary: adaptive composition exceeded the piece budget");
                if (!piece.boundary_ranges)
                    piece.boundary_ranges = std::make_shared<boundary_plane_ranges>(
                        boundary_plane_ranges_for(piece, planes));
                const auto &ranges = *piece.boundary_ranges;
                const auto &members = ranges.members;
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
                    auto outside = boundary_outside_fragments(*closure, allowed, allowed_index);
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
                    vec_t outside = math::dot(witness, planes[(size_t)index].normal) - planes[(size_t)index].dist;
                    if (ranges.values[(size_t)index].first < -(vec_t)1e-7
                        && ranges.values[(size_t)index].second > (vec_t)1e-7
                        && outside > (vec_t)1e-7)
                        candidates.push_back({outside / std::max(
                            ranges.values[(size_t)index].second - ranges.values[(size_t)index].first,
                            (vec_t)1e-8), index});
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
                    last_min = ranges.values[(size_t)index].first;
                    last_max = ranges.values[(size_t)index].second;
                    auto front = clip_boundary_poly(piece, planes[(size_t)index], index);
                    auto back_plane = planes[(size_t)index];
                    back_plane.normal = -back_plane.normal;
                    back_plane.dist = -back_plane.dist;
                    auto back = clip_boundary_poly(piece, back_plane, index);
                    if (front)
                        front->boundary_ranges = boundary_ranges_after_clip(piece, *front, planes);
                    if (back)
                        back->boundary_ranges = boundary_ranges_after_clip(piece, *back, planes);
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
            math::vec3v first{};
            bool have_first = false;
            for (const auto &v : poly.vertices)
                if (std::fabs(math::dot(v, plane.normal) - plane.dist) <= (vec_t)1e-5)
                if (!have_first)
                {
                    first = v;
                    have_first = true;
                }
                else
                {
                    math::vec3v edge = v - first;
                    if (math::length(edge) <= (vec_t)1e-8) continue;
                    for (const auto &other : poly.vertices)
                    {
                        if (std::fabs(math::dot(other, plane.normal) - plane.dist) > (vec_t)1e-5)
                            continue;
                        math::vec3v cross;
                        math::cross(edge, other - first, cross);
                        if (math::length(cross) > (vec_t)1e-8) return true;
                    }
                    break;
                }
            return false;
        }

        boundary_composition prepare_boundary_composition(
            const std::vector<triangle> &triangles,
            const std::vector<const built_brush *> &brushes,
            const csg_result &result)
        {
            boundary_composition composition;
            if (triangles.empty() || brushes.empty()) return composition;
            composition.boundaries = normalized_boundary_planes(triangles);
            if (composition.boundaries.empty()) return composition;

            math::bounding_box &bounds = composition.bounds;
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
            composition.terms = compose_boundary_terms(
                composition.boundaries, bounds, source_pieces, allowed);
            return composition;
        }

        direct_clip::tree build_boundary_hull(
            const boundary_composition &composition,
            const math::vec3v &half)
        {
            direct_clip::tree tree;
            if (composition.boundaries.empty() || composition.terms.empty()) return tree;
            const auto &boundaries = composition.boundaries;
            const auto &bounds = composition.bounds;
            const auto &terms = composition.terms;

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
                            fragment_mask_set(f.incidence[i], pid);
                            count++;
                        }
                    if (count >= 3) fragment_mask_set(f.sides, pid);
                }
                if (f.vertices.size() >= 4 && !f.sides.empty())
                    fragments.push_back(std::move(f));
            }
            if (fragments.empty()) return tree;
            fragment_normalize_masks(fragments, output_planes.values.size());
            logging::info("pyboundary: %zu terms, %zu active planes, hull half %g %g %g\n",
                          fragments.size(), output_planes.values.size(),
                          (double)half.x, (double)half.y, (double)half.z);
            build_tree_scratch build_scratch;
            tree.head = build_tree(std::move(fragments), output_planes, tree.nodes, build_scratch);
            tree.planes = std::move(output_planes.values);
            tree.valid = tree.head >= 0;
            compact_direct_tree(tree);
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
            fragment_normalize_masks(fragments, planes.values.size());
            logging::info("pyexact: fragment input %zu triangles, %zu planes\n",
                          triangles.size(), planes.values.size());
            build_tree_scratch build_scratch;
            tree.head = build_tree(std::move(fragments), planes, tree.nodes, build_scratch);
            tree.planes = std::move(planes.values);
            tree.valid = tree.head >= 0;
            compact_direct_tree(tree);
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
                // Every entity uses the whole-entity adaptive composition
                // path, matching boundary_normalize.py.
                const auto composition = prepare_boundary_composition(triangles, brushes, result);
                auto hull1 = std::async(std::launch::async, [&composition, &options]() {
                    return build_boundary_hull(composition, options.hull_size[1][1]);
                });
                auto hull3 = std::async(std::launch::async, [&composition, &options]() {
                    return build_boundary_hull(composition, options.hull_size[3][1]);
                });
                model.hulls[1] = hull1.get();
                model.hulls[2] = model.hulls[1];
                model.hulls[3] = hull3.get();
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

