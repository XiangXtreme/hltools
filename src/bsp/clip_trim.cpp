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
    void parse_clip_trim(bsp_state &state, token_file &file)
    {
        state.trim_groups.clear();
        int version, count;
        if (!file.has_more()) return; // ordinary legacy .hsz
        auto check = [](bool ok) {
            if (!ok) err::fatal("invalid legacytrim hull-size extension");
        };
        check(file.next_int(version) && version == 1 && file.next_int(count)
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
            int targets, guards;
            check(file.next_int(g.model) && g.model >= 0 && g.model < limits::max_map_models);
            check(file.next_int(g.hull) && g.hull > 0 && g.hull < num_hulls);
            check(file.next_int(targets) && targets > 0 && targets <= limits::max_map_brushes);
            check(file.next_int(guards) && guards >= targets && guards <= limits::max_map_brushes);
            for (const auto &old : state.trim_groups)
                check(old.model != g.model || old.hull != g.hull);
            for (int j = 0; j < targets; j++) g.targets.push_back(read_volume());
            for (int j = 0; j < guards; j++) g.guards.push_back(read_volume());
            state.trim_groups.push_back(std::move(g));
        }
        check(!file.has_more());
    }

    namespace
    {
        constexpr double epsilon = 1e-7;
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
            bool prune(candidates &list, const cell &c)
            {
                candidates next;
                for (auto &volume : list)
                {
                    std::vector<int> remaining;
                    bool disjoint = false;
                    for (int p : volume)
                    {
                        auto r = range(c, geometric_planes[(size_t)p]);
                        // Guard rounding is wider than epsilon, so a tangent
                        // cell here cannot contain real brush-box overlap.
                        if (r.first >= -epsilon) { disjoint = true; break; }
                        if (r.second > epsilon) remaining.push_back(p);
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
                for (int a = 0; a < 3; a++) { bounds.mins[a] -= 1; bounds.maxs[a] += 1; }
                // Bounds themselves use exactly representable disk planes.
                for (int a = 0; a < 3; a++)
                { bounds.mins[a] = (float)bounds.mins[a]; bounds.maxs[a] = (float)bounds.maxs[a]; }
                int original = models[(size_t)g.model].headnode[g.hull];
                size_t before = removed;
                int result = walk(original, box_cell(bounds), std::move(targets), std::move(guards), 0);
                if (removed == before) return;
                // Outside the local box use the original tree, including world
                // fill. A finite source-brush union must not open the void.
                for (int a = 0; a < 3; a++)
                    for (int side = 0; side < 2; side++)
                    {
                        format::dplane_t p{};
                        p.normal[a] = 1; p.type = a;
                        p.dist = (float)(side ? bounds.maxs[a] : bounds.mins[a]);
                        int id = add_plane(p);
                        result = side ? node(id, original, result) : node(id, result, original);
                    }
                models[(size_t)g.model].headnode[g.hull] = result;
                logging::info("legacytrim: model %d hull %d, removed %zu solid cells\n", g.model, g.hull, removed - before);
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
                logging::info("legacytrim: %zu -> %zu clipnodes, %zu -> %zu planes\n",
                    map.clipnodes.size(), output.size(), map.planes.size(), output_planes.size());
                map.models = std::move(models);
                map.clipnodes = std::move(output);
                map.planes = std::move(output_planes);
            }
        };
    }

    void trim_legacy_collision(bsp_state &state)
    {
        if (state.trim_groups.empty()) return;
        trimmer trim(*state.map);
        auto models = state.map->models;
        for (const auto &group : state.trim_groups) trim.group(group, models);
        trim.finish(std::move(models));
    }
}
