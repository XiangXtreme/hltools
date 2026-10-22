#include "internal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
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
        constexpr size_t max_surface_volumes = 1024;
        constexpr size_t max_surface_work = 16000000;
        constexpr size_t max_split_trial_work = 64000000;
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

            struct surface { int plane; polygon vertices; };

            enum class split_order { sampled_area, area, face_count };
            struct split_search
            {
                split_order order;
                size_t work = 0, visits = 0;
                bool stopped = false;
            };

            bool stop_trial(split_search &search, int depth)
            {
                if (search.order == split_order::sampled_area) return false;
                // Trials have their own work limit and do not consume the
                // subdivision budget needed to construct the remaining hulls.
                if (++search.visits > 100000 || depth >= 768 || nodes.size() >= 1900000)
                    search.stopped = true;
                return search.stopped;
            }

            double twice_area(const polygon &vertices)
            {
                point cross{};
                for (size_t i = 1; i + 1 < vertices.size(); i++)
                    cross += math::cross(vertices[i] - vertices[0], vertices[i+1] - vertices[0]);
                return math::length(cross);
            }

            polygon half_polygon(const polygon &vertices, const math::plane &p, bool back)
            {
                polygon out;
                for (size_t i = 0; i < vertices.size(); i++)
                {
                    const auto &a = vertices[i], &b = vertices[(i+1) % vertices.size()];
                    double da = p.distance_to(a), db = p.distance_to(b);
                    if (back ? da <= 0 : da >= 0) out.push_back(a);
                    if ((da < 0 && db > 0) || (da > 0 && db < 0))
                        out.push_back(a + (b-a) * (da / (da-db)));
                }
                return out;
            }

            std::pair<double, double> polygon_range(const polygon &vertices, const math::plane &p)
            {
                double lo = std::numeric_limits<double>::infinity(), hi = -lo;
                for (const auto &v : vertices)
                {
                    double d = p.distance_to(v);
                    lo = std::min(lo, d); hi = std::max(hi, d);
                }
                return {lo, hi};
            }

            bool subtract_volume(const surface &face, const std::vector<int> &volume,
                                 bool keep_coplanar, std::vector<surface> &outside, size_t &work)
            {
                polygon inside = face.vertices;
                // Equal outward faces have one owner; opposite faces are an
                // internal seam and both are removed from the union surface.
                for (int id : volume)
                {
                    if (++work > max_surface_work) return false;
                    const auto &p = geometric_planes[(size_t)id];
                    auto r = polygon_range(inside, p);
                    if (r.first >= -union_epsilon)
                    {
                        bool coplanar = r.second <= union_epsilon
                            && std::fabs(math::dot(p.normal,
                                geometric_planes[(size_t)face.plane].normal)) > 1-1e-8;
                        if (!coplanar || (keep_coplanar && math::dot(p.normal,
                            geometric_planes[(size_t)face.plane].normal) > 0))
                        { outside.push_back(face); return true; }
                    }
                }
                for (int id : volume)
                {
                    if (++work > max_surface_work) return false;
                    const auto &p = geometric_planes[(size_t)id];
                    auto r = polygon_range(inside, p);
                    if (r.second <= union_epsilon) continue;
                    polygon part = half_polygon(inside, p, false);
                    if (part.size() >= 3 && twice_area(part) > union_epsilon*union_epsilon)
                        outside.push_back({face.plane, std::move(part)});
                    inside = half_polygon(inside, p, true);
                    if (inside.size() < 3) return true;
                }
                return true;
            }

            // Exposed faces guide the split order. Whole-cell halfspace tests
            // certify the contents, including when boundary clipping removes
            // a very thin face or several brushes jointly cover the cell.
            int build_surfaces(const cell &region, std::vector<surface> faces,
                               std::vector<fragment> fragments, int depth, split_search &search)
            {
                // A trial is optional: its partial result is never published.
                if (stop_trial(search, depth)) return contents_empty;
                if (search.order == split_order::sampled_area && (++visits > 2000000 || depth > 768))
                    err::fatal("exactunion: surface subdivision budget exceeded");
                std::vector<fragment> active;
                for (auto &frag : fragments)
                {
                    candidates remaining{std::move(frag.sides)};
                    if (prune(remaining, region, union_epsilon)) return contents_solid;
                    if (remaining.empty()) continue;
                    frag.sides = std::move(remaining[0]);
                    active.push_back(std::move(frag));
                }
                if (active.empty()) return contents_empty;
                std::vector<int> splitters;
                for (const auto &face : faces)
                {
                    auto r = range(region, geometric_planes[(size_t)face.plane]);
                    if (r.first < -union_epsilon && r.second > union_epsilon)
                        splitters.push_back(face.plane);
                }
                if (splitters.empty()) return build_union(region, std::move(active), depth+1, &search);
                std::sort(splitters.begin(), splitters.end());
                splitters.erase(std::unique(splitters.begin(), splitters.end()), splitters.end());
                int splitter = splitters[0];
                double best = std::numeric_limits<double>::infinity();
                std::vector<double> weights;
                // Area weights favor broad surfaces; face counts favor
                // eliminating many boundary fragments in a single split.
                for (const auto &face : faces)
                    weights.push_back(search.order == split_order::face_count ? 1 : twice_area(face.vertices));
                size_t samples = search.order == split_order::sampled_area
                    ? std::min<size_t>(24, splitters.size()) : splitters.size();
                for (size_t i = 0; i < samples; i++)
                {
                    int id = splitters[i * splitters.size() / samples];
                    double front = 0, back = 0, cuts = 0;
                    for (size_t j = 0; j < faces.size(); j++)
                    {
                        const auto &face = faces[j];
                        if (search.order != split_order::sampled_area
                            && (search.work += face.vertices.size()) > max_split_trial_work)
                        { search.stopped = true; return contents_empty; }
                        auto r = polygon_range(face.vertices, geometric_planes[(size_t)id]);
                        if (r.first >= -union_epsilon && r.second <= union_epsilon) continue;
                        if (r.second <= union_epsilon) back += weights[j];
                        else if (r.first >= -union_epsilon) front += weights[j];
                        else cuts += weights[j];
                    }
                    double score = 20*cuts + std::fabs(front-back);
                    if (score < best) { best = score; splitter = id; }
                }
                const auto p = geometric_planes[(size_t)splitter];
                std::vector<surface> front, back;
                for (auto &face : faces)
                {
                    auto r = polygon_range(face.vertices, p);
                    if (r.first >= -union_epsilon && r.second <= union_epsilon) continue;
                    if (r.second <= union_epsilon) back.push_back(std::move(face));
                    else if (r.first >= -union_epsilon) front.push_back(std::move(face));
                    else
                    {
                        front.push_back({face.plane, half_polygon(face.vertices, p, false)});
                        back.push_back({face.plane, half_polygon(face.vertices, p, true)});
                    }
                }
                std::vector<fragment> front_fragments, back_fragments;
                for (auto &frag : active)
                {
                    auto r = range(frag.geometry, p);
                    if (r.first >= -union_epsilon) front_fragments.push_back(std::move(frag));
                    else if (r.second <= union_epsilon) back_fragments.push_back(std::move(frag));
                    else
                    {
                        front_fragments.push_back({half_cell(frag.geometry, p, false), frag.sides});
                        back_fragments.push_back({half_cell(frag.geometry, p, true), std::move(frag.sides)});
                    }
                }
                int f = build_surfaces(half_cell(region, p, false), std::move(front), std::move(front_fragments), depth+1, search);
                if (search.stopped) return contents_empty;
                int b = build_surfaces(half_cell(region, p, true), std::move(back), std::move(back_fragments), depth+1, search);
                if (search.stopped) return contents_empty;
                return node(splitter, f, b);
            }

            size_t reachable_nodes(int root) const
            {
                std::vector<bool> seen(nodes.size());
                std::vector<int> pending{root};
                size_t count = 0;
                while (!pending.empty())
                {
                    int id = pending.back(); pending.pop_back();
                    if (id < 0 || seen[(size_t)id]) continue;
                    seen[(size_t)id] = true; count++;
                    const auto &n = nodes[(size_t)id];
                    pending.push_back(n.front); pending.push_back(n.back);
                }
                return count;
            }

            void discard_nodes(size_t checkpoint)
            {
                for (size_t i = checkpoint; i < nodes.size(); i++)
                {
                    const auto &n = nodes[i];
                    node_ids.erase(std::make_tuple(n.plane, n.front, n.back));
                }
                nodes.resize(checkpoint);
            }

            int forced_side(int known, bool front, int tested) const
            {
                const auto &p = planes[(size_t)known], &q = planes[(size_t)tested];
                // Identical engine dot products have monotone thresholds.
                // Keep axial fast paths separate and require exact normals;
                // nearly parallel or reversed planes are not interchangeable.
                if (p.type < 3 || q.type < 3)
                {
                    if (p.type != q.type) return -1;
                }
                else
                    for (int a = 0; a < 3; ++a)
                        if (p.normal[a] != q.normal[a]) return -1;
                if (front && q.dist <= p.dist) return 0;
                if (!front && q.dist >= p.dist) return 1;
                return -1;
            }

            int synthesize_window(int root, const std::vector<int> &references,
                                  size_t &work, bool &stopped, int width, size_t alternatives)
            {
                constexpr size_t work_limit = 64000000;
                auto stop = [&]() {
                    if (++work >= work_limit || nodes.size() >= 1900000) stopped = true;
                    return stopped;
                };
                // Descendants still referenced outside this window already
                // occupy the output graph. Reusing them has no added cost.
                std::set<int> released;
                std::map<int, int> removed;
                std::vector<int> dropping{root};
                while (!dropping.empty() && !stopped)
                {
                    if (stop()) break;
                    int id = dropping.back(); dropping.pop_back();
                    if (!released.insert(id).second) continue;
                    const auto n = nodes[(size_t)id];
                    for (int child : {n.front, n.back})
                        if (child >= 0 && ++removed[child] == references[(size_t)child])
                            dropping.push_back(child);
                }
                // Treat unknown descendants as distinct opaque terminals.
                // Every truth-table assignment is retained, so rebuilding a
                // window preserves even geometrically unrealizable predicates.
                std::vector<int> variables, pending{root};
                std::set<int> seen;
                for (size_t i = 0; i < pending.size() && !stopped; ++i)
                {
                    if (stop()) break;
                    int id = pending[i];
                    if (id < 0 || !seen.insert(id).second) continue;
                    const auto n = nodes[(size_t)id];
                    if (std::find(variables.begin(), variables.end(), n.plane) == variables.end())
                    {
                        if (variables.size() >= (size_t)width) continue;
                        variables.push_back(n.plane);
                    }
                    pending.push_back(n.front); pending.push_back(n.back);
                }
                if (stopped) return root;
                std::sort(variables.begin(), variables.end());
                std::vector<int> table;
                for (size_t assignment = 0; assignment < (size_t(1) << variables.size()); ++assignment)
                {
                    int id = root;
                    while (id >= 0 && !stopped)
                    {
                        if (stop()) break;
                        const auto n = nodes[(size_t)id];
                        auto found = std::lower_bound(variables.begin(), variables.end(), n.plane);
                        if (found == variables.end() || *found != n.plane) break;
                        id = assignment & (size_t(1) << (found - variables.begin())) ? n.front : n.back;
                    }
                    table.push_back(id);
                }
                if (stopped) return root;
                std::map<int, size_t> costs;
                std::map<int, std::vector<int>> footprints;
                size_t footprint_entries = 0;
                constexpr size_t footprint_limit = 1000000;
                const std::vector<int> empty_footprint;
                auto free_node = [&](int id) {
                    return id < 0 || ((size_t)id < references.size()
                        && references[(size_t)id] > 0 && released.find(id) == released.end());
                };
                // Cache exact descendant sets so sibling candidates reuse their
                // common work. Keep this optional cache bounded; large windows
                // fall back to direct traversal with the same scoring function.
                auto cache_footprint = [&](int id) {
                    std::vector<int> pending{id};
                    while (!pending.empty() && !stopped)
                    {
                        if (stop()) break;
                        int next = pending.back();
                        if (free_node(next) || footprints.find(next) != footprints.end())
                        { pending.pop_back(); continue; }
                        const auto n = nodes[(size_t)next];
                        bool ready = true;
                        for (int child : {n.front, n.back})
                            if (!free_node(child) && footprints.find(child) == footprints.end())
                            { pending.push_back(child); ready = false; }
                        if (!ready) continue;
                        const auto &f = free_node(n.front) ? empty_footprint : footprints.at(n.front);
                        const auto &b = free_node(n.back) ? empty_footprint : footprints.at(n.back);
                        size_t bound = f.size() + b.size() + 1;
                        if (bound > footprint_limit - footprint_entries) return false;
                        work += bound;
                        if (work >= work_limit) { stopped = true; break; }
                        std::vector<int> combined;
                        combined.reserve(bound);
                        size_t fi = 0, bi = 0;
                        while (fi < f.size() || bi < b.size())
                        {
                            if (bi == b.size() || (fi < f.size() && f[fi] < b[bi]))
                                combined.push_back(f[fi++]);
                            else if (fi == f.size() || b[bi] < f[fi])
                                combined.push_back(b[bi++]);
                            else { combined.push_back(f[fi++]); ++bi; }
                        }
                        combined.insert(std::lower_bound(combined.begin(), combined.end(), next), next);
                        footprint_entries += combined.size();
                        costs.emplace(next, combined.size());
                        footprints.emplace(next, std::move(combined));
                        pending.pop_back();
                    }
                    return !stopped;
                };
                auto cost = [&](int id) {
                    auto found = costs.find(id);
                    if (found != costs.end()) return found->second;
                    if (free_node(id)) { costs.emplace(id, 0); return size_t(0); }
                    if (cache_footprint(id)) return costs.at(id);
                    if (stopped) return size_t(0);
                    std::set<int> used;
                    std::vector<int> todo{id};
                    while (!todo.empty() && !stopped)
                    {
                        if (stop()) break;
                        int next = todo.back(); todo.pop_back();
                        if (free_node(next)) continue;
                        if (!used.insert(next).second) continue;
                        const auto n = nodes[(size_t)next];
                        todo.push_back(n.front); todo.push_back(n.back);
                    }
                    costs.emplace(id, used.size());
                    return used.size();
                };
                using key = std::pair<std::vector<int>, std::vector<int>>;
                std::map<key, std::vector<int>> cache;
                std::function<std::vector<int>(const std::vector<int> &, const std::vector<int> &)> build
                    = [&](const std::vector<int> &vars, const std::vector<int> &values) -> std::vector<int> {
                    if (stop()) return {root};
                    if (std::all_of(values.begin(), values.end(), [&](int i) { return i == values[0]; }))
                        return {values[0]};
                    key k{vars, values};
                    auto found = cache.find(k);
                    if (found != cache.end()) return found->second;
                    std::vector<int> candidates;
                    std::set<int> unique;
                    for (size_t v = 0; v < vars.size() && !stopped; ++v)
                    {
                        std::vector<int> front, back, remaining = vars;
                        remaining.erase(remaining.begin() + v);
                        for (size_t i = 0; i < values.size(); ++i)
                            (i & (size_t(1) << v) ? front : back).push_back(values[i]);
                        if (front == back) return build(remaining, front);
                        auto fronts = build(remaining, front), backs = build(remaining, back);
                        if (stopped) break;
                        // Retain several realizations: a slightly larger child
                        // can share enough nodes with its sibling to win overall.
                        for (int f : fronts) for (int b : backs)
                        {
                            if (stop()) return {root};
                            int candidate = node(vars[v], f, b);
                            cost(candidate);
                            if (unique.insert(candidate).second) candidates.push_back(candidate);
                        }
                    }
                    if (stopped) return {root};
                    std::stable_sort(candidates.begin(), candidates.end(), [&](int a, int b) {
                        return costs.at(a) < costs.at(b);
                    });
                    if (candidates.size() > alternatives) candidates.resize(alternatives);
                    cache.emplace(std::move(k), candidates);
                    return candidates;
                };
                return build(variables, table).front();
            }

            bool optimize_roots(std::vector<format::dmodel_t> &models, int levels,
                                std::uint32_t seed = 0, int window_width = 0, size_t alternatives = 1)
            {
                const bool windows = window_width != 0;
                std::vector<int> roots;
                for (const auto &m : models)
                    for (int h = 1; h < num_hulls; h++) roots.push_back(m.headnode[h]);
                size_t work = 0, trials = 0, steps = 0;
                constexpr size_t work_limit = 64000000;
                const size_t trial_limit = levels || windows ? 65536 : 256;
                const bool explore = seed != 0;
                const size_t step_limit = explore ? 256 : std::numeric_limits<size_t>::max();
                auto shuffle = [&](std::vector<int> &values) {
                    if (!explore) return;
                    for (size_t i = values.size(); i > 1; --i)
                    {
                        seed = seed * 1664525u + 1013904223u;
                        std::swap(values[i-1], values[seed % i]);
                    }
                };
                auto reachable = [&](const std::vector<int> &heads) {
                    std::vector<int> result, pending = heads;
                    std::vector<bool> seen(nodes.size());
                    while (!pending.empty() && work < work_limit)
                    {
                        int id = pending.back(); pending.pop_back(); ++work;
                        if (id < 0 || seen[(size_t)id]) continue;
                        seen[(size_t)id] = true; result.push_back(id);
                        pending.push_back(nodes[(size_t)id].front);
                        pending.push_back(nodes[(size_t)id].back);
                    }
                    std::sort(result.begin(), result.end());
                    return result;
                };
                auto live = reachable(roots);
                const size_t initial_size = live.size();
                auto best_roots = roots;
                size_t best_size = live.size();
                std::set<std::vector<int>> explored;
                if (explore) explored.insert(roots);
                while (work < work_limit && trials < trial_limit && steps < step_limit
                       && nodes.size() < 1900000)
                {
                    std::vector<std::vector<int>> parents(nodes.size());
                    std::vector<int> references(nodes.size());
                    for (int root : roots) if (root >= 0) ++references[(size_t)root];
                    for (int id : live)
                        for (int child : {nodes[(size_t)id].front, nodes[(size_t)id].back})
                            if (child >= 0)
                            { ++references[(size_t)child]; parents[(size_t)child].push_back(id); }
                    bool improved = false;
                    auto order = live;
                    shuffle(order);
                    for (int id : order)
                    {
                        if (work >= work_limit || trials >= trial_limit || nodes.size() >= 1900000) break;
                        const auto n = nodes[(size_t)id];
                        std::vector<int> splitters;
                        if (windows) splitters.push_back(n.plane);
                        else if (levels)
                        {
                            std::vector<std::pair<int, int>> pending;
                            for (int child : {n.front, n.back})
                                if (child >= 0) pending.emplace_back(child, 1);
                            while (!pending.empty() && work < work_limit)
                            {
                                ++work;
                                auto [child, level] = pending.back(); pending.pop_back();
                                const auto &c = nodes[(size_t)child];
                                if (c.plane != n.plane) splitters.push_back(c.plane);
                                if (level < levels)
                                    for (int next : {c.front, c.back})
                                        if (next >= 0) pending.emplace_back(next, level + 1);
                            }
                            std::sort(splitters.begin(), splitters.end());
                            splitters.erase(std::unique(splitters.begin(), splitters.end()), splitters.end());
                        }
                        else
                        {
                            if (n.front < 0 || n.back < 0) continue;
                            const auto f = nodes[(size_t)n.front], b = nodes[(size_t)n.back];
                            if (f.plane != b.plane || (f.front != b.front && f.back != b.back)) continue;
                            splitters.push_back(f.plane);
                        }
                        shuffle(splitters);
                        for (int splitter : splitters)
                        {
                            if (work >= work_limit || trials >= trial_limit || nodes.size() >= 1900000) break;
                            ++trials;
                            size_t checkpoint = nodes.size();
                            bool stopped = false;
                            std::map<std::pair<int, bool>, int> cofactors;
                            std::function<int(int, bool, int)> restrict = [&](int old, bool side, int depth) -> int {
                                if (old < 0 || stopped) return old;
                                if (++work >= work_limit || depth > 1024 || nodes.size() >= 1900000)
                                { stopped = true; return old; }
                                auto key = std::make_pair(old, side);
                                auto found = cofactors.find(key);
                                if (found != cofactors.end()) return found->second;
                                const auto child = nodes[(size_t)old];
                                int result;
                                int forced = forced_side(splitter, side, child.plane);
                                if (forced >= 0)
                                    result = restrict(forced ? child.back : child.front, side, depth + 1);
                                else
                                {
                                    int cf = restrict(child.front, side, depth + 1);
                                    int cb = restrict(child.back, side, depth + 1);
                                    if (stopped) return old;
                                    result = node(child.plane, cf, cb);
                                }
                                cofactors.emplace(key, result);
                                return result;
                            };
                            int replacement;
                            if (windows) replacement = synthesize_window(id, references, work, stopped,
                                                                         window_width, alternatives);
                            else
                            {
                                int front = restrict(id, true, 0), back = restrict(id, false, 0);
                                replacement = stopped ? id : node(splitter, front, back);
                            }
                            if (stopped) { discard_nodes(checkpoint); break; }
                            if (replacement == id) { discard_nodes(checkpoint); continue; }
                            std::vector<bool> affected(checkpoint);
                            std::vector<int> pending{id};
                            while (!pending.empty())
                            {
                                if (++work >= work_limit) { stopped = true; break; }
                                int old = pending.back(); pending.pop_back();
                                if (affected[(size_t)old]) continue;
                                affected[(size_t)old] = true;
                                for (int parent : parents[(size_t)old]) pending.push_back(parent);
                            }
                            if (stopped) { discard_nodes(checkpoint); break; }
                            std::vector<int> remap(nodes.size(), std::numeric_limits<int>::min());
                            remap[(size_t)id] = replacement;
                            std::function<int(int, int)> rewrite = [&](int old, int depth) -> int {
                                if (old < 0 || stopped || !affected[(size_t)old]) return old;
                                if (++work >= work_limit || depth > 1024 || nodes.size() >= 1900000)
                                { stopped = true; return old; }
                                // Only original roots and their original children
                                // are traversed; freshly interned nodes are results.
                                int &mapped = remap[(size_t)old];
                                if (mapped != std::numeric_limits<int>::min()) return mapped;
                                const auto child = nodes[(size_t)old];
                                int cf = rewrite(child.front, depth + 1);
                                int cb = rewrite(child.back, depth + 1);
                                if (stopped) return old;
                                return mapped = node(child.plane, cf, cb);
                            };
                            std::vector<int> candidate;
                            for (int root : roots) candidate.push_back(rewrite(root, 0));
                            size_t count = live.size();
                            std::unordered_map<int, int> changed_references;
                            std::function<void(int, bool, int)> reference = [&](int old, bool add, int depth) {
                                if (old < 0 || stopped) return;
                                if (++work >= work_limit || depth > 1024)
                                { stopped = true; return; }
                                int base = (size_t)old < references.size() ? references[(size_t)old] : 0;
                                int &refs = changed_references.emplace(old, base).first->second;
                                bool traverse = add ? refs++ == 0 : --refs == 0;
                                if (!traverse) return;
                                if (add) ++count; else --count;
                                const auto child = nodes[(size_t)old];
                                reference(child.front, add, depth + 1);
                                reference(child.back, add, depth + 1);
                            };
                            // Add before removing: unchanged shared subtrees stay
                            // live, so trial cost is proportional to changed paths.
                            for (int root : candidate) reference(root, true, 0);
                            for (int root : roots) reference(root, false, 0);
                            // Cofactors preserve the engine's ordered thresholds
                            // for each identical dot product, including distance
                            // == 0. Count every model/hull root: a shared old
                            // subtree may remain live elsewhere.
                            if (!stopped && work < work_limit
                                && (count < live.size() || (explore && count == live.size()
                                    && explored.find(candidate) == explored.end())))
                            {
                                // Promotion can lengthen a path which never
                                // tested the promoted plane. Cached/shared
                                // subtrees must still obey the output depth
                                // limit when reached through a new ancestor.
                                std::vector<int> heights(nodes.size(), -1);
                                std::function<int(int, int)> height = [&](int current, int depth) -> int {
                                    if (current < 0) return 0;
                                    if (++work >= work_limit) { stopped = true; return 1026; }
                                    if (depth > 1024) return 1026;
                                    int &cached = heights[(size_t)current];
                                    if (cached >= 0) return cached;
                                    const auto child = nodes[(size_t)current];
                                    int f = height(child.front, depth + 1);
                                    if (stopped || f > 1025) return 1026;
                                    int b = height(child.back, depth + 1);
                                    return cached = std::min(1026, 1 + std::max(f, b));
                                };
                                bool safe_depth = true;
                                for (int root : candidate)
                                    if (height(root, 0) > 1025) { safe_depth = false; break; }
                                if (stopped) { discard_nodes(checkpoint); break; }
                                if (!safe_depth) { discard_nodes(checkpoint); continue; }
                                auto next = reachable(candidate);
                                if (work >= work_limit) { discard_nodes(checkpoint); break; }
                                roots = std::move(candidate); live = std::move(next);
                                if (explore) explored.insert(roots);
                                if (live.size() < best_size)
                                { best_size = live.size(); best_roots = roots; }
                                ++steps;
                                improved = true; break;
                            }
                            discard_nodes(checkpoint);
                        }
                        if (improved) break;
                    }
                    if (!improved) break;
                }
                size_t i = 0;
                for (auto &m : models)
                    for (int h = 1; h < num_hulls; h++) m.headnode[h] = best_roots[i++];
                if (windows)
                    logging::info("collision graph: %zu -> %zu nodes, %zu width-%d window trials (%s)\n",
                        initial_size, best_size, trials, window_width,
                        work >= work_limit || trials >= trial_limit || nodes.size() >= 1900000
                            ? "budget reached" : "complete");
                else if (levels)
                    logging::info("collision graph: %zu -> %zu nodes, %zu level-%d %s trials (%s)\n",
                        initial_size, best_size, trials, levels,
                        explore ? "plateau" : "plane-order",
                        work >= work_limit || trials >= trial_limit || steps >= step_limit || nodes.size() >= 1900000
                            ? "budget reached" : "complete");
                return best_size < initial_size;
            }

            int surface_union(const candidates &volumes, const math::bounding_box &bounds)
            {
                // Boundary subtraction is quadratic in the input volumes.
                // Keep the full fragment construction available for large sets.
                if (volumes.size() > max_surface_volumes) return volume_union(volumes, bounds);
                std::vector<surface> faces;
                std::vector<fragment> fragments;
                size_t work = 0;
                for (size_t i = 0; i < volumes.size(); i++)
                {
                    cell geometry = box_cell(bounds);
                    for (int id : volumes[i]) geometry = half_cell(geometry, geometric_planes[(size_t)id], true);
                    if (geometry.size() >= 4) fragments.push_back({geometry, volumes[i]});
                    for (int id : volumes[i])
                    {
                        polygon face;
                        double largest = union_epsilon * union_epsilon;
                        for (const auto &vertices : geometry)
                        {
                            auto r = polygon_range(vertices, geometric_planes[(size_t)id]);
                            if (r.first < -epsilon || r.second > epsilon) continue;
                            double size = twice_area(vertices);
                            if (size > largest) { largest = size; face = vertices; }
                        }
                        if (face.empty()) continue;
                        std::vector<surface> visible{{id, std::move(face)}};
                        for (size_t j = 0; j < volumes.size() && !visible.empty(); j++)
                        {
                            if (i == j) continue;
                            std::vector<surface> next;
                            for (const auto &part : visible)
                                if (!subtract_volume(part, volumes[j], i < j, next, work))
                                    return volume_union(volumes, bounds);
                            visible = std::move(next);
                        }
                        for (auto &part : visible) faces.push_back(std::move(part));
                    }
                }
                split_search initial{split_order::sampled_area};
                int best = build_surfaces(box_cell(bounds), faces, fragments, 0, initial);
                size_t best_size = reachable_nodes(best);
                // Compare complete trees of the same volumes. Changing the
                // split order never relaxes the contents-classification tests.
                for (auto order : {split_order::area, split_order::face_count})
                {
                    size_t checkpoint = nodes.size();
                    split_search trial{order};
                    int candidate = build_surfaces(box_cell(bounds), faces, fragments, 0, trial);
                    size_t size = trial.stopped ? best_size : reachable_nodes(candidate);
                    if (size < best_size) { best = candidate; best_size = size; }
                    else discard_nodes(checkpoint);
                }
                return best;
            }

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
            int build_union(const cell &region, std::vector<fragment> fragments, int depth,
                            split_search *search = nullptr)
            {
                if (search && stop_trial(*search, depth)) return contents_empty;
                if ((!search || search->order == split_order::sampled_area)
                    && (++visits > 2000000 || depth > 768))
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
                        if (search && search->order != split_order::sampled_area
                            && (search->work += frag.geometry.size()) > max_split_trial_work)
                        { search->stopped = true; return contents_empty; }
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
                int f = build_union(half_cell(region, p, false), std::move(front), depth + 1, search);
                if (search && search->stopped) return contents_empty;
                int b = build_union(half_cell(region, p, true), std::move(back), depth + 1, search);
                if (search && search->stopped) return contents_empty;
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

            // Express only retained solid leaves as convex volumes. Building
            // them together with the additions removes their shared boundaries
            // before the union tree is partitioned.
            bool solid_volumes(int root, const cell &region, std::vector<int> sides,
                               candidates &volumes, int depth)
            {
                if (++visits > 2000000 || depth > 768) err::fatal("exactunion: solid leaf budget exceeded");
                if (volumes.size() >= max_surface_volumes) return false;
                if (root < 0)
                {
                    if (root == contents_solid) volumes.push_back(std::move(sides));
                    return true;
                }
                auto n = nodes[(size_t)root];
                auto p = geometric_planes[(size_t)n.plane];
                auto r = range(region, p);
                if (r.first >= 0) return solid_volumes(n.front, region, std::move(sides), volumes, depth+1);
                if (r.second <= 0) return solid_volumes(n.back, region, std::move(sides), volumes, depth+1);
                auto reverse = planes[(size_t)n.plane];
                for (float &v : reverse.normal) v = -v;
                reverse.dist = -reverse.dist;
                if (reverse.type < 3) reverse.type += 3;
                sides.push_back(add_plane(reverse));
                if (!solid_volumes(n.front, half_cell(region, p, false), sides, volumes, depth+1)) return false;
                sides.back() = n.plane;
                return solid_volumes(n.back, half_cell(region, p, true), std::move(sides), volumes, depth+1);
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
                    candidates combined = additions;
                    std::vector<int> sides;
                    for (int a = 0; a < 3; a++)
                        for (int side = 0; side < 2; side++)
                        {
                            format::dplane_t p{};
                            p.normal[a] = side ? 1 : -1;
                            p.dist = side ? bounds.maxs[a] : -bounds.mins[a];
                            p.type = side ? a : a+3;
                            sides.push_back(add_plane(p));
                        }
                    if (solid_volumes(result, box_cell(bounds), std::move(sides), combined, 0))
                    {
                        int solid = surface_union(combined, bounds);
                        result = overlay(without_solid(original, cache), solid, box_cell(bounds), 0);
                    }
                    else result = overlay(result, surface_union(additions, bounds), box_cell(bounds), 0);
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
                optimize_roots(models, 0);
                optimize_roots(models, 1);
                optimize_roots(models, 2);
                optimize_roots(models, 3);
                optimize_roots(models, 4);
                // Neutral reorders expose new strict reductions; alternate
                // both searches while retaining their completed smaller graphs.
                // Bound the outer schedule as well as each individual pass.
                for (std::uint32_t round = 0; round < 8; ++round)
                {
                    bool improved = false;
                    for (std::uint32_t seed : {17u,193u,1021u})
                        improved |= optimize_roots(models, 1, seed + round * 4099u);
                    for (int level = 1; level <= 4; ++level)
                        improved |= optimize_roots(models, level);
                    if (!improved) break;
                }
                if (optimize_roots(models, 0, 0, 6))
                    for (int level = 1; level <= 4; ++level) optimize_roots(models, level);
                if (optimize_roots(models, 0, 0, 8, 4))
                    for (int level = 1; level <= 4; ++level) optimize_roots(models, level);
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
