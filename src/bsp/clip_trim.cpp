#include "internal.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <tuple>
#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"

namespace bsp
{
    void capture_collision_fill(bsp_state &state, node *root)
    {
        trim_group *group = nullptr;
        for (auto &g : state.trim_groups)
            if (g.model == state.nummodels - 1 && g.hull == state.hullnum
                && !g.additions.empty()) group = &g;
        if (!group) return;
        math::bounding_box bounds;
        for (const auto &v : group->targets) bounds.add(v.bounds);
        for (const auto &v : group->additions) bounds.add(v.bounds);
        for (int a = 0; a < 3; a++) { bounds.mins[a] -= 1; bounds.maxs[a] += 1; }
        std::vector<math::plane> path;
        size_t total = 0;
        std::function<void(node *)> visit = [&](node *n) {
            if (n->exterior_filled)
            {
                total += path.size();
                if (total > 4000000) err::fatal("exactunion: exterior fill budget exceeded");
                group->exterior.push_back({bounds, path});
                return;
            }
            if (n->planenum == -1) return;
            const auto &p = state.planes[(size_t)n->planenum];
            path.push_back({-p.normal, -p.dist});
            visit(n->children[0]);
            path.back() = {p.normal, p.dist};
            visit(n->children[1]);
            path.pop_back();
        };
        visit(root);
    }

    void parse_clip_trim(bsp_state &state, token_file &file)
    {
        state.trim_groups.clear();
        int version, count;
        if (!file.has_more()) return; // ordinary legacy .hsz
        auto check = [](bool ok) {
            if (!ok) err::fatal("invalid collision hull-size extension");
        };
        check(file.next_int(version) && (version == 1 || version == 2) && file.next_int(count)
            && count > 0 && count <= 3 * limits::max_map_models);
        size_t total_planes = 0;
        auto read_volume = [&]() {
            trim_volume v;
            int n;
            check(file.next_int(n) && n >= 4 && n <= 8192);
            total_planes += (size_t)n;
            check(total_planes <= 4000000);
            auto number = [&](vec_t &out) {
                double d;
                check(file.next_double(d) && std::isfinite(d) && std::fabs(d) < 1e9);
                out = d;
            };
            for (int a = 0; a < 3; a++) number(v.bounds.mins[a]);
            for (int a = 0; a < 3; a++)
            {
                number(v.bounds.maxs[a]);
                check(v.bounds.maxs[a] > v.bounds.mins[a]);
            }
            for (int i = 0; i < n; i++)
            {
                math::plane p;
                for (int a = 0; a < 3; a++) number(p.normal[a]);
                number(p.dist);
                check(std::fabs(math::length(p.normal) - 1) < 1e-5);
                v.planes.push_back(p);
            }
            return v;
        };
        for (int i = 0; i < count; i++)
        {
            trim_group g;
            int targets, guards, additions = 0;
            check(file.next_int(g.model) && g.model >= 0 && g.model < limits::max_map_models);
            check(file.next_int(g.hull) && g.hull > 0 && g.hull < num_hulls);
            check(file.next_int(targets) && targets > 0 && targets <= limits::max_map_brushes);
            check(file.next_int(guards) && guards >= targets && guards <= limits::max_map_brushes);
            if (version == 2)
                check(file.next_int(additions) && additions >= 0 && additions <= targets);
            for (const auto &old : state.trim_groups)
                check(old.model != g.model || old.hull != g.hull);
            for (int j = 0; j < targets; j++) g.targets.push_back(read_volume());
            for (int j = 0; j < guards; j++) g.guards.push_back(read_volume());
            for (int j = 0; j < additions; j++) g.additions.push_back(read_volume());
            state.trim_groups.push_back(std::move(g));
        }
        check(!file.has_more());
    }

    namespace
    {
        constexpr double epsilon = 1e-7;
        // Disk planes use float coefficients. Treat their roundoff-scale
        // slivers consistently during union pruning and fragment scoring.
        constexpr double union_epsilon = math::normal_epsilon;
        using point = math::vec3d;
        using polygon = std::vector<point>;
        using cell = std::vector<polygon>;
        using candidates = std::vector<std::vector<int>>;

        cell box_cell(const math::bounding_box &b)
        {
            cell out;
            for (int axis = 0; axis < 3; axis++)
                for (int side = 0; side < 2; side++)
                {
                    polygon face;
                    for (int i : {0, 1, 3, 2})
                    {
                        point p;
                        p[axis] = side ? b.maxs[axis] : b.mins[axis];
                        p[(axis + 1) % 3] = (i & 1) ? b.maxs[(axis + 1) % 3] : b.mins[(axis + 1) % 3];
                        p[(axis + 2) % 3] = (i & 2) ? b.maxs[(axis + 2) % 3] : b.mins[(axis + 2) % 3];
                        face.push_back(p);
                    }
                    out.push_back(std::move(face));
                }
            return out;
        }

        std::pair<double, double> range(const cell &c, const math::plane &p)
        {
            double lo = std::numeric_limits<double>::infinity(), hi = -lo;
            for (const auto &face : c)
                for (const auto &v : face)
                {
                    double d = p.distance_to(v);
                    lo = std::min(lo, d);
                    hi = std::max(hi, d);
                }
            return {lo, hi};
        }

        // Clip every face and close the cut with a sorted cap. Every decision
        // uses the entire convex cell; no corner/centre sampling of a leaf.
        cell half_cell(const cell &c, const math::plane &p, bool back)
        {
            cell out;
            polygon cap;
            auto add_cap = [&](const point &v) {
                for (const auto &old : cap)
                    if (math::length(v - old) < 1e-8) return;
                cap.push_back(v);
            };
            for (const auto &face : c)
            {
                polygon clipped;
                for (size_t i = 0; i < face.size(); i++)
                {
                    const auto &a = face[i], &b = face[(i + 1) % face.size()];
                    double da = p.distance_to(a), db = p.distance_to(b);
                    if (back ? da <= 0 : da >= 0) clipped.push_back(a);
                    if (std::fabs(da) < 1e-9) add_cap(a);
                    if ((da < 0 && db > 0) || (da > 0 && db < 0))
                    {
                        point v = a + (b - a) * (da / (da - db));
                        clipped.push_back(v);
                        add_cap(v);
                    }
                }
                if (clipped.size() >= 3) out.push_back(std::move(clipped));
            }
            if (cap.size() >= 3)
            {
                point centre;
                for (const auto &v : cap) centre += v;
                centre = centre * (1.0 / cap.size());
                point axis = std::fabs(p.normal.x) < 0.8 ? point{1, 0, 0} : point{0, 1, 0};
                point u = math::cross(p.normal, axis), v = math::cross(p.normal, u);
                std::sort(cap.begin(), cap.end(), [&](const point &a, const point &b) {
                    return std::atan2(math::dot(a - centre, v), math::dot(a - centre, u))
                         < std::atan2(math::dot(b - centre, v), math::dot(b - centre, u));
                });
                out.push_back(std::move(cap));
            }
            return out;
        }

        struct wide_node { int plane, front, back; };

        class trimmer
        {
            format::map_data &map;
            std::vector<format::dplane_t> planes;
            std::vector<math::plane> geometric_planes;
            std::map<std::tuple<float, float, float, float, int>, int> plane_ids;
            std::vector<wide_node> nodes;
            std::map<std::tuple<int, int, int>, int> node_ids;
            size_t visits = 0, removed = 0;

            int add_plane(const format::dplane_t &p)
            {
                auto key = std::make_tuple(p.normal[0], p.normal[1], p.normal[2], p.dist, p.type);
                auto found = plane_ids.find(key);
                if (found != plane_ids.end()) return found->second;
                if (planes.size() >= limits::max_internal_map_planes)
                    err::fatal("legacytrim: internal plane budget exceeded");
                int id = (int)planes.size();
                plane_ids.emplace(key, id);
                planes.push_back(p);
                // Match the engine's axial fast path exactly.
                math::plane gp{{p.normal[0], p.normal[1], p.normal[2]}, p.dist};
                if (p.type < 3) { gp.normal = {}; gp.normal[p.type] = 1; }
                geometric_planes.push_back(gp);
                return id;
            }

            int rounded_plane(const math::plane &p, const math::bounding_box &bounds, bool guard)
            {
                format::dplane_t out{};
                double error = 1e-6;
                int dominant = 0;
                for (int a = 0; a < 3; a++)
                {
                    out.normal[a] = (float)p.normal[a];
                    error += std::fabs(out.normal[a] - p.normal[a])
                           * std::max(std::fabs(bounds.mins[a]), std::fabs(bounds.maxs[a]));
                    if (std::fabs(out.normal[a]) > std::fabs(out.normal[dominant])) dominant = a;
                }
                // Guards round outward, preserving real box collision. Targets
                // use the same float rounding as legacy itself: shrinking them
                // would leave a thin solid skin on each old protrusion.
                out.dist = guard ? std::nextafter((float)(p.dist + error),
                    std::numeric_limits<float>::infinity()) : (float)p.dist;
                out.type = 3 + dominant;
                if (out.normal[dominant] == 1 && out.normal[(dominant + 1) % 3] == 0
                    && out.normal[(dominant + 2) % 3] == 0) out.type = dominant;
                return add_plane(out);
            }

            int node(int p, int front, int back)
            {
                if (front == back) return front;
                auto key = std::make_tuple(p, front, back);
                auto found = node_ids.find(key);
                if (found != node_ids.end()) return found->second;
                if (nodes.size() >= 2000000) err::fatal("legacytrim: internal clipnode budget exceeded");
                int id = (int)nodes.size();
                nodes.push_back({p, front, back});
                node_ids.emplace(key, id);
                return id;
            }

            // Discard disjoint convex volumes and already satisfied planes.
            // An empty plane list means this cell is covered by that volume.
            bool prune(candidates &list, const cell &c, double tolerance = epsilon)
            {
                candidates next;
                for (auto &volume : list)
                {
                    std::vector<int> remaining;
                    bool disjoint = false;
                    for (int p : volume)
                    {
                        auto r = range(c, geometric_planes[(size_t)p]);
                        // Tangency within the selected geometry tolerance
                        // does not add interior volume to this cell.
                        if (r.first >= -tolerance) { disjoint = true; break; }
                        if (r.second > tolerance) remaining.push_back(p);
                    }
                    if (disjoint) continue;
                    if (remaining.empty()) { list = {{}}; return true; }
                    next.push_back(std::move(remaining));
                }
                list = std::move(next);
                return false;
            }

            int walk(int original, const cell &c, candidates targets, candidates guards, int depth)
            {
                if (++visits > 2000000 || depth > 768)
                    err::fatal("legacytrim: subdivision budget exceeded (model collision too complex)");
                if (original < 0 && original != contents_solid) return original;
                bool covered = prune(guards, c);
                if (covered) return original;
                bool selected = prune(targets, c);
                if (targets.empty()) return original;
                if (original == contents_solid && selected && guards.empty())
                { removed++; return contents_empty; }
                int splitter;
                wide_node old{};
                if (original >= 0)
                {
                    old = nodes[(size_t)original];
                    splitter = old.plane;
                }
                else splitter = !guards.empty() ? guards[0][0] : targets[0][0];
                const math::plane p = geometric_planes[(size_t)splitter];
                auto r = range(c, p);
                if (original >= 0)
                {
                    if (r.first >= 0) return walk(old.front, c, std::move(targets), std::move(guards), depth + 1);
                    if (r.second <= 0) return walk(old.back, c, std::move(targets), std::move(guards), depth + 1);
                }
                // Numerical slivers are kept conservatively. No constraints
                // are dropped to make an over-budget hull appear successful.
                if (r.first >= -epsilon || r.second <= epsilon) return original;
                cell front = half_cell(c, p, false), back = half_cell(c, p, true);
                if (front.size() < 4 || back.size() < 4) return original;
                int f = walk(original >= 0 ? old.front : original, front, targets, guards, depth + 1);
                int b = walk(original >= 0 ? old.back : original, back, std::move(targets), std::move(guards), depth + 1);
                return node(splitter, f, b);
            }

            struct fragment
            {
                cell geometry;
                std::vector<int> sides;
            };

            // Only planes with a non-collinear face remain splitter candidates.
            // Supporting edge/vertex planes and faces removed by earlier cuts
            // are already implied by the region and the surviving faces.
            bool bounds_fragment(const cell &geometry, int id)
            {
                const auto &p = geometric_planes[(size_t)id];
                point first{}, edge{};
                bool have_first = false, have_edge = false;
                for (const auto &face : geometry)
                    for (const auto &v : face)
                    {
                        if (std::fabs(p.distance_to(v)) > union_epsilon) continue;
                        if (!have_first) { first = v; have_first = true; }
                        else if (!have_edge)
                        {
                            edge = v - first;
                            if (math::length(edge) > union_epsilon) have_edge = true;
                        }
                        else if (math::length(math::cross(edge, v - first))
                                 > union_epsilon * math::length(edge)) return true;
                    }
                return false;
            }

            // Construct the union directly from convex fragments. Score real
            // fragment cuts, so adjoining brushes need not inherit the old BSP
            // split order. Region tests remove redundant supporting planes.
            int build_union(const cell &region, std::vector<fragment> fragments, int depth)
            {
                if (++visits > 2000000 || depth > 768)
                    err::fatal("exactunion: subdivision budget exceeded");
                std::vector<fragment> active;
                std::vector<int> splitters;
                for (auto &frag : fragments)
                {
                    candidates remaining{std::move(frag.sides)};
                    if (prune(remaining, region, union_epsilon)) return contents_solid;
                    if (remaining.empty()) continue;
                    frag.sides = std::move(remaining[0]);
                    for (int id : frag.sides)
                        if (bounds_fragment(frag.geometry, id)) splitters.push_back(id);
                    active.push_back(std::move(frag));
                }
                if (active.empty()) return contents_empty;
                // Keep every halfspace for coverage tests, even when it only
                // touches a fragment edge. A numerical degeneracy must not
                // turn an empty candidate list into full-region coverage.
                if (splitters.empty())
                    for (const auto &frag : active)
                        splitters.insert(splitters.end(), frag.sides.begin(), frag.sides.end());
                std::sort(splitters.begin(), splitters.end());
                splitters.erase(std::unique(splitters.begin(), splitters.end()), splitters.end());
                int splitter = -1;
                size_t best = std::numeric_limits<size_t>::max();
                const size_t samples = std::min<size_t>(24, splitters.size());
                for (size_t i = 0; i < samples; i++)
                {
                    int id = splitters[i * splitters.size() / samples];
                    size_t front = 0, back = 0, cuts = 0;
                    for (const auto &frag : active)
                    {
                        auto r = range(frag.geometry, geometric_planes[(size_t)id]);
                        if (r.second <= union_epsilon) back++;
                        else if (r.first >= -union_epsilon) front++;
                        else cuts++;
                    }
                    size_t score = 5 * cuts + (front > back ? front - back : back - front);
                    if (score < best) { best = score; splitter = id; }
                }
                if (splitter < 0) err::fatal("exactunion: no valid splitter");
                const auto p = geometric_planes[(size_t)splitter];
                std::vector<fragment> front, back;
                for (auto &frag : active)
                {
                    auto r = range(frag.geometry, p);
                    if (r.first >= -union_epsilon) front.push_back(std::move(frag));
                    else if (r.second <= union_epsilon) back.push_back(std::move(frag));
                    else
                    {
                        front.push_back({half_cell(frag.geometry, p, false), frag.sides});
                        back.push_back({half_cell(frag.geometry, p, true), std::move(frag.sides)});
                    }
                }
                int f = build_union(half_cell(region, p, false), std::move(front), depth + 1);
                int b = build_union(half_cell(region, p, true), std::move(back), depth + 1);
                return node(splitter, f, b);
            }

            // Solid union takes precedence over water/empty; everywhere else
            // retain the background tree, including its filled exterior.
            int restrict_tree(int original, const cell &region, int depth)
            {
                if (++visits > 2000000 || depth > 768)
                    err::fatal("exactunion: background subdivision budget exceeded");
                if (original < 0) return original;
                const auto n = nodes[(size_t)original];
                const auto p = geometric_planes[(size_t)n.plane];
                auto r = range(region, p);
                if (r.first >= 0) return restrict_tree(n.front, region, depth + 1);
                if (r.second <= 0) return restrict_tree(n.back, region, depth + 1);
                int f = restrict_tree(n.front, half_cell(region, p, false), depth + 1);
                int b = restrict_tree(n.back, half_cell(region, p, true), depth + 1);
                return node(n.plane, f, b);
            }

            int without_solid(int id, std::map<int, int> &cache)
            {
                if (id < 0) return id == contents_solid ? contents_empty : id;
                auto found = cache.find(id);
                if (found != cache.end()) return found->second;
                const auto n = nodes[(size_t)id];
                int f = without_solid(n.front, cache), b = without_solid(n.back, cache);
                return cache[id] = node(n.plane, f, b);
            }

            // An outside branch is unbounded. Propagate exact axial intervals
            // instead of retaining the complete old tree under every box side.
            int restrict_outside(int original, math::bounding_box bounds, int depth)
            {
                if (++visits > 2000000 || depth > 768)
                    err::fatal("exactunion: outside subdivision budget exceeded");
                if (original < 0) return original;
                const auto n = nodes[(size_t)original];
                const auto p = geometric_planes[(size_t)n.plane];
                double lo = -p.dist, hi = -p.dist;
                int axis = -1, nonzero = 0;
                for (int a = 0; a < 3; a++)
                    if (p.normal[a] != 0)
                    {
                        axis = a; nonzero++;
                        lo += p.normal[a] * (p.normal[a] > 0 ? bounds.mins[a] : bounds.maxs[a]);
                        hi += p.normal[a] * (p.normal[a] > 0 ? bounds.maxs[a] : bounds.mins[a]);
                    }
                if (lo >= 0) return restrict_outside(n.front, bounds, depth + 1);
                if (hi <= 0) return restrict_outside(n.back, bounds, depth + 1);
                auto front = bounds, back = bounds;
                if (nonzero == 1)
                {
                    double cut = p.dist / p.normal[axis];
                    if (p.normal[axis] > 0)
                    { front.mins[axis] = cut; back.maxs[axis] = cut; }
                    else
                    { front.maxs[axis] = cut; back.mins[axis] = cut; }
                }
                int f = restrict_outside(n.front, front, depth + 1);
                int b = restrict_outside(n.back, back, depth + 1);
                return node(n.plane, f, b);
            }

            int overlay(int original, int addition, const cell &region, int depth)
            {
                if (++visits > 2000000 || depth > 768)
                    err::fatal("exactunion: overlay budget exceeded");
                if (addition == contents_solid || original == contents_solid) return contents_solid;
                if (addition == contents_empty) return restrict_tree(original, region, depth + 1);
                while (original >= 0)
                {
                    const auto old = nodes[(size_t)original];
                    auto r = range(region, geometric_planes[(size_t)old.plane]);
                    if (r.first >= 0) original = old.front;
                    else if (r.second <= 0) original = old.back;
                    else break;
                    if (original == contents_solid) return contents_solid;
                }
                const auto n = nodes[(size_t)addition];
                const auto p = geometric_planes[(size_t)n.plane];
                auto r = range(region, p);
                if (r.first >= 0) return overlay(original, n.front, region, depth + 1);
                if (r.second <= 0) return overlay(original, n.back, region, depth + 1);
                int f = overlay(original, n.front, half_cell(region, p, false), depth + 1);
                int b = overlay(original, n.back, half_cell(region, p, true), depth + 1);
                return node(n.plane, f, b);
            }

            int retain_covered_solid(int original, int coverage, const cell &region,
                                     std::map<int, int> &empty_cache, int depth)
            {
                if (++visits > 2000000 || depth > 768)
                    err::fatal("exactunion: coverage budget exceeded");
                if (coverage == contents_solid) return restrict_tree(original, region, depth + 1);
                if (coverage == contents_empty)
                    return restrict_tree(without_solid(original, empty_cache), region, depth + 1);
                if (original < 0 && original != contents_solid) return original;
                const auto n = nodes[(size_t)coverage];
                const auto p = geometric_planes[(size_t)n.plane];
                auto r = range(region, p);
                if (r.first >= 0) return retain_covered_solid(original, n.front, region, empty_cache, depth + 1);
                if (r.second <= 0) return retain_covered_solid(original, n.back, region, empty_cache, depth + 1);
                int f = retain_covered_solid(original, n.front, half_cell(region, p, false), empty_cache, depth + 1);
                int b = retain_covered_solid(original, n.back, half_cell(region, p, true), empty_cache, depth + 1);
                return node(n.plane, f, b);
            }

            int volume_union(const candidates &volumes, const math::bounding_box &bounds)
            {
                std::vector<fragment> fragments;
                for (const auto &ids : volumes)
                {
                    cell geometry = box_cell(bounds);
                    for (int id : ids)
                        geometry = half_cell(geometry, geometric_planes[(size_t)id], true);
                    if (geometry.size() >= 4) fragments.push_back({std::move(geometry), ids});
                }
                return build_union(box_cell(bounds), std::move(fragments), 0);
            }

        public:
            explicit trimmer(format::map_data &m) : map(m)
            {
                // Keep original numbering, including duplicate planes/nodes.
                for (const auto &p : m.planes)
                {
                    planes.push_back(p);
                    math::plane gp{{p.normal[0], p.normal[1], p.normal[2]}, p.dist};
                    if (p.type < 3) { gp.normal = {}; gp.normal[p.type] = 1; }
                    geometric_planes.push_back(gp);
                    plane_ids.emplace(std::make_tuple(p.normal[0], p.normal[1], p.normal[2], p.dist, p.type), (int)planes.size() - 1);
                }
                for (const auto &n : m.clipnodes)
                {
                    nodes.push_back({n.planenum, n.children[0], n.children[1]});
                    node_ids.emplace(std::make_tuple(n.planenum, (int)n.children[0], (int)n.children[1]), (int)nodes.size() - 1);
                }
            }

            void group(const trim_group &g, std::vector<format::dmodel_t> &models)
            {
                if (g.model >= (int)models.size()) err::fatal("legacytrim: missing model %d", g.model);
                candidates targets, guards;
                math::bounding_box bounds;
                auto prepare = [&](const std::vector<trim_volume> &volumes, candidates &out, bool guard) {
                    for (const auto &v : volumes)
                    {
                        std::vector<int> ids;
                        for (const auto &p : v.planes) ids.push_back(rounded_plane(p, v.bounds, guard));
                        out.push_back(std::move(ids));
                    }
                };
                prepare(g.targets, targets, false);
                prepare(g.guards, guards, true);
                for (const auto &v : g.targets) bounds.add(v.bounds);
                for (const auto &v : g.additions) bounds.add(v.bounds);
                for (int a = 0; a < 3; a++) { bounds.mins[a] -= 1; bounds.maxs[a] += 1; }
                // Bounds themselves use exactly representable disk planes.
                for (int a = 0; a < 3; a++)
                { bounds.mins[a] = (float)bounds.mins[a]; bounds.maxs[a] = (float)bounds.maxs[a]; }
                int original = models[(size_t)g.model].headnode[g.hull];
                size_t before = removed;
                int result;
                if (!g.additions.empty())
                {
                    // Reclassify solid from the complete source-volume union.
                    // The face BSP may contain solid outside even its source
                    // legacy volumes, so subtracting those volumes is insufficient.
                    // Preserve non-solid contents and the actual exterior flood.
                    candidates additions;
                    prepare(g.additions, additions, true);
                    // S union (L intersection (S union G)) equals
                    // S union (L intersection G). Keeping S in both trees
                    // only cross-subdivides the old tree by the same terrain.
                    candidates selected = additions;
                    std::sort(selected.begin(), selected.end());
                    guards.erase(std::remove_if(guards.begin(), guards.end(),
                        [&](const std::vector<int> &v) {
                            return std::binary_search(selected.begin(), selected.end(), v);
                        }), guards.end());
                    prepare(g.exterior, guards, false);
                    int coverage = volume_union(guards, bounds);
                    std::map<int, int> cache;
                    result = retain_covered_solid(original, coverage, box_cell(bounds), cache, 0);
                    int addition = volume_union(additions, bounds);
                    result = overlay(result, addition, box_cell(bounds), 0);
                    logging::info("exactunion: model %d hull %d, %zu exact expansions\n",
                        g.model, g.hull, g.additions.size());
                }
                else
                {
                    result = walk(original, box_cell(bounds), std::move(targets), std::move(guards), 0);
                    if (removed == before) return;
                }
                // Outside the local box use the original tree, including world
                // fill. A finite source-brush union must not open the void.
                for (int a = 0; a < 3; a++)
                    for (int side = 0; side < 2; side++)
                    {
                        format::dplane_t p{};
                        p.normal[a] = 1; p.type = a;
                        p.dist = (float)(side ? bounds.maxs[a] : bounds.mins[a]);
                        int id = add_plane(p);
                        int outside = original;
                        if (!g.additions.empty())
                        {
                            math::bounding_box region;
                            for (int k = 0; k < 3; k++)
                            {
                                region.mins[k] = -std::numeric_limits<double>::infinity();
                                region.maxs[k] = std::numeric_limits<double>::infinity();
                            }
                            if (side) region.mins[a] = p.dist;
                            else region.maxs[a] = p.dist;
                            outside = restrict_outside(original, region, 0);
                        }
                        result = side ? node(id, outside, result) : node(id, result, outside);
                    }
                models[(size_t)g.model].headnode[g.hull] = result;
                if (g.additions.empty())
                    logging::info("legacytrim: model %d hull %d, removed %zu solid cells\n",
                        g.model, g.hull, removed - before);
            }

            void finish(std::vector<format::dmodel_t> models)
            {
                std::vector<format::dclipnode_t> output;
                std::vector<int> remap(nodes.size(), std::numeric_limits<int>::min()), plane_remap(planes.size(), -1);
                // Original plane indices are also referenced by draw nodes/faces.
                std::vector<format::dplane_t> output_planes = map.planes;
                for (size_t i = 0; i < output_planes.size(); i++) plane_remap[i] = (int)i;
                std::function<int(int, int)> emit = [&](int id, int depth) -> int {
                    if (id < 0) return id;
                    if (depth > 1024) err::fatal("legacytrim: output clip tree depth exceeded");
                    int &mapped = remap[(size_t)id];
                    if (mapped != std::numeric_limits<int>::min()) return mapped;
                    const auto &n = nodes[(size_t)id];
                    int f = emit(n.front, depth + 1), b = emit(n.back, depth + 1);
                    if (f == b) return mapped = f;
                    if (output.size() >= limits::max_map_clipnodes)
                        err::fatal("legacytrim: collision exceeds %d clipnodes", limits::max_map_clipnodes);
                    int &p = plane_remap[(size_t)n.plane];
                    if (p < 0)
                    {
                        if (output_planes.size() >= limits::max_map_planes)
                            err::fatal("legacytrim: collision exceeds %d planes", limits::max_map_planes);
                        p = (int)output_planes.size(); output_planes.push_back(planes[(size_t)n.plane]);
                    }
                    mapped = (int)output.size();
                    format::dclipnode_t out{};
                    out.planenum = p; out.children[0] = (short)f; out.children[1] = (short)b;
                    output.push_back(out);
                    return mapped;
                };
                for (auto &m : models)
                    for (int h = 1; h < num_hulls; h++) m.headnode[h] = emit(m.headnode[h], 0);
                logging::info("collision union: %zu -> %zu clipnodes, %zu -> %zu planes\n",
                    map.clipnodes.size(), output.size(), map.planes.size(), output_planes.size());
                map.models = std::move(models);
                map.clipnodes = std::move(output);
                map.planes = std::move(output_planes);
            }
        };
    }

    void trim_legacy_collision(bsp_state &state)
    {
        if (state.options.noclip || state.trim_groups.empty()) return;
        trimmer trim(*state.map);
        auto models = state.map->models;
        for (const auto &group : state.trim_groups) trim.group(group, models);
        trim.finish(std::move(models));
    }
}
