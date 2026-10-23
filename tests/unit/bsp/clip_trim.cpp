#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include "bsp/internal.h"
#include "support/test.h"

namespace
{
    bsp::trim_volume box(math::vec3v mins, math::vec3v maxs)
    {
        bsp::trim_volume v;
        v.bounds.mins = mins; v.bounds.maxs = maxs;
        for (int a = 0; a < 3; a++)
        {
            math::vec3v n{}; n[a] = 1;
            v.planes.push_back({n, maxs[a]});
            v.planes.push_back({-n, -mins[a]});
        }
        return v;
    }

    int contents(const format::map_data &map, int model, int hull, math::vec3v p)
    {
        int node = map.models[(size_t)model].headnode[hull];
        for (int depth = 0; node >= 0 && depth < 1024; depth++)
        {
            const auto &n = map.clipnodes[(size_t)node];
            const auto &plane = map.planes[(size_t)n.planenum];
            double d = plane.type < 3 ? p[plane.type] - plane.dist
                : math::dot(p, plane.normal) - plane.dist;
            node = n.children[d < 0 ? 1 : 0];
        }
        require(node < 0);
        return node;
    }

    format::map_data solid_world()
    {
        format::map_data map;
        format::dmodel_t m{};
        m.headnode[0] = 123; // draw hull must remain untouched
        for (int h = 1; h < 4; h++) m.headnode[h] = bsp::contents_solid;
        map.models = {m, m};
        return map;
    }

    void check_engine_hull_ranges(const format::map_data &map)
    {
        for (const auto &model : map.models)
            for (int hull = 1; hull < bsp::num_hulls; ++hull)
            {
                const int first = model.headnode[hull];
                std::vector<bool> seen(map.clipnodes.size());
                std::vector<int> pending{first};
                while (!pending.empty())
                {
                    int id = pending.back(); pending.pop_back();
                    if (id < 0) continue;
                    // GoldSrc sets firstclipnode to this hull's headnode.
                    require(id >= first);
                    require((size_t)id < map.clipnodes.size());
                    if (seen[(size_t)id]) continue;
                    seen[(size_t)id] = true;
                    const auto &node = map.clipnodes[(size_t)id];
                    pending.push_back(node.children[0]);
                    pending.push_back(node.children[1]);
                }
            }
    }
}

test("legacytrim uses cross-brush coverage and protects exterior fill and other hulls/models")
{
    auto map = solid_world();
    bsp::bsp_state state; state.map = &map;
    bsp::trim_group g;
    g.targets = {box({-8,-10,-10},{4,10,10}), box({0,-10,-10},{10,10,10})};
    g.guards = {box({-8,-10,-10},{-2,10,10}), box({-4,-10,-10},{6,10,10})};
    state.trim_groups.push_back(g);
    bsp::trim_legacy_collision(state);
    // x=2 is in target A but only guard B: a per-brush intersection loses it.
    expect(contents(map,0,1,{2,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{8,0,0}) == bsp::contents_empty);
    expect(contents(map,0,1,{-9,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{100,0,0}) == bsp::contents_solid);
    expect(contents(map,0,2,{8,0,0}) == bsp::contents_solid);
    expect(contents(map,0,3,{8,0,0}) == bsp::contents_solid);
    expect(contents(map,1,1,{8,0,0}) == bsp::contents_solid);
    expect(map.models[0].headnode[0] == 123);
}

test("legacytrim only removes solid and preserves original empty and liquid leaves")
{
    auto map = solid_world();
    format::dplane_t x{}, z{};
    x.normal[0] = 1; x.type = 0;
    z.normal[2] = 1; z.type = 2;
    map.planes = {x,z};
    format::dclipnode_t first{}, second{};
    first.planenum = 0; first.children[0] = bsp::contents_empty; first.children[1] = 1;
    second.planenum = 1; second.children[0] = bsp::contents_water; second.children[1] = bsp::contents_solid;
    map.clipnodes = {first,second}; map.models[0].headnode[1] = 0;
    bsp::bsp_state state; state.map = &map;
    bsp::trim_group g;
    g.targets = {box({-10,-10,-10},{10,10,10})};
    g.guards = {box({-10,-10,-10},{10,10,-2})};
    state.trim_groups.push_back(g);
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{1,0,-5}) == bsp::contents_empty);
    expect(contents(map,0,1,{-1,0,1}) == bsp::contents_water);
    expect(contents(map,0,1,{-1,0,-1}) == bsp::contents_empty);
    expect(contents(map,0,1,{-1,0,-5}) == bsp::contents_solid);
}

test("collision factoring preserves predicate ties and counts shared hull roots")
{
    for (bool shared : {false, true})
    for (bool common_front : {false, true})
    {
        auto map = solid_world();
        format::dplane_t x{}, y{};
        x.normal[0] = 1; x.type = 0;
        y.normal[1] = 1; y.type = 1;
        map.planes = {x,y};
        format::dclipnode_t root{}, front{}, back{};
        root.planenum = 0; root.children[0] = 1; root.children[1] = 2;
        front.planenum = back.planenum = 1;
        int common = common_front ? 0 : 1, different = 1 - common;
        front.children[different] = bsp::contents_water;
        back.children[different] = bsp::contents_empty;
        front.children[common] = back.children[common] = bsp::contents_solid;
        map.clipnodes = {root,front,back}; map.models[0].headnode[1] = 0;
        if (shared) map.models[0].headnode[2] = 1;
        // Exercise final graph compaction while the selected model's trim
        // preserves its solid contents. The measured tree is in model zero.
        bsp::trim_group g; g.model = 1;
        g.targets = g.guards = {box({-2,-2,-2},{2,2,2})};
        bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
        bsp::trim_legacy_collision(state);
        expect(map.clipnodes.size() == (shared ? 3u : 2u));
        for (double px : {-1.,-1e-9,0.,1e-9,1.})
            for (double py : {-1.,-1e-9,0.,1e-9,1.})
            {
                bool solid = common_front ? py >= 0 : py < 0;
                int expected = solid ? bsp::contents_solid
                    : (px < 0 ? bsp::contents_empty : bsp::contents_water);
                expect(contents(map,0,1,{px,py,0}) == expected);
                if (shared) expect(contents(map,0,2,{px,py,0})
                    == (solid ? bsp::contents_solid : bsp::contents_water));
            }
        expect(map.models[0].headnode[0] == 123);
    }
}

test("collision plane promotion reuses another hull without a common immediate branch")
{
    auto map = solid_world();
    format::dplane_t x{}, y{};
    x.normal[0] = 1; x.type = 0;
    y.normal[1] = 1; y.type = 1;
    map.planes = {x,y};
    format::dclipnode_t root{}, front{}, shared{};
    root.planenum = 0; root.children[0] = 1; root.children[1] = bsp::contents_empty;
    front.planenum = 1; front.children[0] = bsp::contents_water; front.children[1] = bsp::contents_empty;
    shared.planenum = 0; shared.children[0] = bsp::contents_water; shared.children[1] = bsp::contents_empty;
    map.clipnodes = {root,front,shared};
    map.models[0].headnode[1] = 0; map.models[0].headnode[2] = 2;
    bsp::trim_group g; g.model = 1;
    g.targets = g.guards = {box({-2,-2,-2},{2,2,2})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(map.clipnodes.size() == 2u);
    for (double px : {-1.,-1e-9,0.,1e-9,1.})
        for (double py : {-1.,-1e-9,0.,1e-9,1.})
        {
            expect(contents(map,0,1,{px,py,0})
                == (px >= 0 && py >= 0 ? bsp::contents_water : bsp::contents_empty));
            expect(contents(map,0,2,{px,py,0})
                == (px >= 0 ? bsp::contents_water : bsp::contents_empty));
        }
}

test("collision promotion preserves the output depth limit through shared subtrees")
{
    auto map = solid_world();
    int tail = bsp::contents_empty;
    for (int i = 0; i < 1024; i++)
    {
        format::dplane_t p{}; p.normal[2] = 1;
        p.normal[0] = float(0.01 + 0.000001*i); p.type = 5; p.dist = float(i);
        map.planes.push_back(p);
        format::dclipnode_t n{}; n.planenum = i;
        n.children[0] = (short)tail; n.children[1] = bsp::contents_solid;
        tail = (int)map.clipnodes.size(); map.clipnodes.push_back(n);
    }
    format::dplane_t x{}, y{};
    x.normal[0] = 1; x.type = 0;
    y.normal[1] = 1; y.type = 1;
    map.planes.push_back(x); map.planes.push_back(y);
    auto branch = [&](int plane, int front, int back) {
        format::dclipnode_t n{}; n.planenum = plane;
        n.children[0] = (short)front; n.children[1] = (short)back;
        int id = (int)map.clipnodes.size(); map.clipnodes.push_back(n); return id;
    };
    int q = branch(1025, bsp::contents_water, bsp::contents_empty);
    map.models[0].headnode[1] = branch(1024, q, tail);
    map.models[0].headnode[2] = branch(1024, bsp::contents_water, tail);
    map.models[0].headnode[3] = branch(1024, bsp::contents_empty, tail);
    bsp::trim_group g; g.model = 1;
    g.targets = g.guards = {box({-2,-2,-2},{2,2,2})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    std::vector<int> heights(map.clipnodes.size());
    std::function<int(int)> height = [&](int id) {
        if (id < 0) return 0;
        int &cached = heights[(size_t)id];
        if (cached) return cached;
        const auto &n = map.clipnodes[(size_t)id];
        return cached = 1 + std::max(height(n.children[0]), height(n.children[1]));
    };
    for (int h = 1; h < 4; h++) expect(height(map.models[0].headnode[h]) <= 1025);
    expect(contents(map,0,1,{1,1,2000}) == bsp::contents_water);
    expect(contents(map,0,1,{1,-1,2000}) == bsp::contents_empty);
}

test("collision graph search crosses equal-size rewrites to reduce repeated predicates")
{
    auto map = solid_world();
    for (int axis = 0; axis < 3; axis++)
    {
        format::dplane_t p{}; p.normal[axis] = 1; p.type = axis;
        map.planes.push_back(p);
    }
    format::dplane_t oblique{};
    oblique.normal[0] = oblique.normal[1] = oblique.normal[2] = float(1/std::sqrt(3.0));
    oblique.type = 3; map.planes.push_back(oblique);
    auto branch = [&](int plane, int front, int back) {
        format::dclipnode_t n{}; n.planenum = plane;
        n.children[0] = (short)front; n.children[1] = (short)back;
        map.clipnodes.push_back(n);
    };
    branch(2, bsp::contents_empty, bsp::contents_solid);
    branch(0, 0, bsp::contents_solid);
    branch(3, bsp::contents_water, bsp::contents_solid);
    branch(1, 2, bsp::contents_solid);
    branch(1, 3, 2);
    branch(1, bsp::contents_solid, 4);
    map.models[0].headnode[1] = 1;
    map.models[0].headnode[2] = 4;
    map.models[0].headnode[3] = 5;
    const auto original = map;
    bsp::trim_group g; g.model = 1;
    g.targets = g.guards = {box({-2,-2,-2},{2,2,2})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(map.clipnodes.size() <= 4u);
    for (int h = 1; h < 4; h++)
        for (double x : {-1.,0.,1.})
            for (double y : {-1.,0.,1.})
                for (double z : {-1.,0.,1.})
                    expect(contents(map,0,h,{x,y,z}) == contents(original,0,h,{x,y,z}));
}

test("collision threshold implications retain strict back sides and exact normal groups")
{
    for (bool axial : {false, true})
    for (bool front_side : {false, true})
    for (bool same_normal : {false, true})
    {
        auto map = solid_world();
        format::dplane_t p{}, q{};
        p.normal[0] = 1; p.type = axial ? 0 : 3;
        if (!axial) p.normal[1] = float(0.125);
        q = p; q.dist = front_side ? -1.f : 1.f;
        if (!same_normal)
        {
            q.type = 3;
            q.normal[1] += float(0.00001);
        }
        map.planes = {p,q};
        format::dclipnode_t root{}, child{};
        root.planenum = 0; child.planenum = 1;
        int side = front_side ? 0 : 1;
        root.children[side] = 1; root.children[1-side] = bsp::contents_empty;
        child.children[side] = bsp::contents_water;
        child.children[1-side] = bsp::contents_empty;
        map.clipnodes = {root,child}; map.models[0].headnode[1] = 0;
        const auto original = map;
        bsp::trim_group g; g.model = 1;
        g.targets = g.guards = {box({-2,-2,-2},{2,2,2})};
        bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
        bsp::trim_legacy_collision(state);
        expect(map.clipnodes.size() == (same_normal ? 1u : 2u));
        for (double x : {-2.,-1.,-1e-9,0.,1e-9,1.,2.})
            for (double y : {-1e7,-1.,0.,1.,1e7})
            {
                expect(contents(map,0,1,{x,y,0}) == contents(original,0,1,{x,y,0}));
                // Also place points exactly on each disk plane.
                for (const auto &plane : original.planes)
                {
                    double px = plane.dist - (plane.type < 3 ? 0 : double(plane.normal[1])*y);
                    expect(contents(map,0,1,{px,y,0}) == contents(original,0,1,{px,y,0}));
                }
            }
    }
}

test("collision graph search preserves shared repeated predicates across all roots")
{
    for (std::uint32_t seed : {7u,31u,149u,65537u})
    {
        auto map = solid_world();
        for (int axis = 0; axis < 3; axis++)
        {
            format::dplane_t p{}; p.normal[axis] = 1; p.type = axis;
            map.planes.push_back(p);
        }
        auto random = [&]() { seed = seed * 1664525u + 1013904223u; return seed; };
        std::vector<int> choices{bsp::contents_empty,bsp::contents_solid,bsp::contents_water};
        for (int i = 0; i < 64; i++)
        {
            format::dclipnode_t n{}; n.planenum = (int)(random() % 3);
            n.children[0] = (short)choices[random() % choices.size()];
            n.children[1] = (short)choices[random() % choices.size()];
            map.clipnodes.push_back(n); choices.push_back(i);
        }
        for (int h = 1; h < 4; h++) map.models[0].headnode[h] = 64 - h;
        map.models[1].headnode[2] = 60; map.models[1].headnode[3] = 59;
        const auto original = map;
        bsp::trim_group g; g.model = 1;
        g.targets = g.guards = {box({-2,-2,-2},{2,2,2})};
        bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
        bsp::trim_legacy_collision(state);
        expect(map.clipnodes.size() <= original.clipnodes.size());
        for (int m = 0; m < 2; m++)
            for (int h = 1; h < 4; h++)
                for (double x : {-1.,0.,1.})
                    for (double y : {-1.,0.,1.})
                        for (double z : {-1.,0.,1.})
                            expect(contents(map,m,h,{x,y,z}) == contents(original,m,h,{x,y,z}));
    }
}

test("collision windows preserve every predicate assignment across shared model roots")
{
    size_t threshold_size = 0;
    for (bool deep : {false, true})
    {
        auto map = solid_world();
        for (int i = 0; i < 8; ++i)
        {
            format::dplane_t p{}; p.type = 5;
            p.normal[0] = float(0.03125 * (i+1));
            p.normal[1] = float(0.015625 * i*i); p.normal[2] = 1;
            map.planes.push_back(p);
        }
        std::uint32_t seed = 20261023u;
        auto random = [&]() { seed = seed * 1664525u + 1013904223u; return seed; };
        std::vector<int> choices{bsp::contents_empty,bsp::contents_solid,bsp::contents_water};
        for (int i = 0; i < 160; ++i)
        {
            format::dclipnode_t n{}; n.planenum = int(random() % 8);
            n.children[0] = (short)choices[random() % choices.size()];
            n.children[1] = (short)choices[random() % choices.size()];
            map.clipnodes.push_back(n); choices.push_back(i);
        }
        for (int m = 0; m < 2; ++m)
            for (int h = 1; h < 4; ++h) map.models[m].headnode[h] = 159-m*3-h;
        const auto original = map;
        bsp::trim_group g; g.model = 1;
        g.targets = g.guards = {box({-2,-2,-2},{2,2,2})};
        bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
        expect(!state.options.deepclip);
        state.options.deepclip = deep;
        bsp::trim_legacy_collision(state);
        check_engine_hull_ranges(map);
        auto classify = [](const format::map_data &data, int m, int h, unsigned assignment) {
            int id = data.models[m].headnode[h];
            for (int depth = 0; id >= 0 && depth < 1024; ++depth)
            {
                const auto &n = data.clipnodes[(size_t)id];
                require(n.planenum >= 0); require(n.planenum < 8);
                id = n.children[(assignment & (1u << n.planenum)) ? 0 : 1];
            }
            require(id < 0);
            return id;
        };
        if (deep)
        {
            expect(map.clipnodes.size() <= 60u);
            expect(map.clipnodes.size() <= threshold_size);
        }
        else threshold_size = map.clipnodes.size();
        for (int m = 0; m < 2; ++m)
            for (int h = 1; h < 4; ++h)
                for (unsigned assignment = 0; assignment < 256; ++assignment)
                    expect(classify(map,m,h,assignment) == classify(original,m,h,assignment));
    }
}

test("legacytrim classifies oblique boundaries throughout a convex cell")
{
    auto map = solid_world();
    bsp::bsp_state state; state.map = &map;
    bsp::trim_group g;
    g.targets = {box({-10,-10,-10},{10,10,10})};
    auto guard = g.targets[0];
    double n = 1 / std::sqrt(3.0);
    guard.planes.push_back({{n,n,n},3});
    g.guards = {guard}; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    for (int x = -8; x <= 8; x++)
        for (int y = -8; y <= 8; y++)
            for (int z = -8; z <= 8; z++)
                expect(contents(map,0,1,{(double)x,(double)y,(double)z})
                    == (n*(x+y+z) < 3 ? bsp::contents_solid : bsp::contents_empty));
}

test("legacytrim parser accepts ordinary hull sizes without enabling a trim")
{
    bsp::bsp_state state;
    bsp::token_file file;
    file.set_text("0 0 0 0 0 0 -16 -16 -36 16 16 36 -32 -32 -32 32 32 32 -16 -16 -18 16 16 18");
    bsp::parse_hull_sizes(state,file);
    expect(state.trim_groups.empty());
}

test("collision sidecar versions preserve local scope unless complete coverage is explicit")
{
    const std::string volume = "6 -4 -4 -4 4 4 4 "
        "1 0 0 4 -1 0 0 4 0 1 0 4 0 -1 0 4 0 0 1 4 0 0 -1 4 ";
    for (int version : {1,2,3})
    {
        std::string data = std::to_string(version) + " 1 0 1 1 1 ";
        if (version >= 2) data += "1 ";
        if (version >= 3) data += "1 ";
        data += volume + volume;
        if (version >= 2) data += volume;
        bsp::bsp_state state;
        bsp::token_file file; file.set_text(data);
        bsp::parse_clip_trim(state,file);
        require(state.trim_groups.size() == 1);
        expect(state.trim_groups[0].complete == (version == 3));
        expect(state.trim_groups[0].additions.size() == (version >= 2 ? 1u : 0u));
    }
}

test("legacytrim does not leave a collision skin on the old legacy surface")
{
    auto map = solid_world();
    format::dplane_t p{}; p.normal[0] = 1; p.dist = 10; p.type = 0;
    map.planes = {p};
    format::dclipnode_t n{};
    n.planenum = 0; n.children[0] = bsp::contents_empty; n.children[1] = bsp::contents_solid;
    map.clipnodes = {n}; map.models[0].headnode[1] = 0;
    bsp::trim_group g;
    g.targets = {box({-10,-10,-10},{10,10,10})};
    g.guards = {box({-10,-10,-10},{5,10,10})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{9.999999,0,0}) == bsp::contents_empty);
    expect(contents(map,0,1,{10.000001,0,0}) == bsp::contents_empty);
    expect(contents(map,0,1,{4.999999,0,0}) == bsp::contents_solid);
}

test("sidecar numbers reject malformed and overflowing trim data")
{
    bsp::token_file file;
    int i; double d;
    file.set_text("0oops 9999999999999999999999");
    expect_false(file.next_int(i)); expect_false(file.next_int(i));
    file.set_text("garbage 1.0oops 1e9999");
    expect(file.has_more());
    expect_false(file.next_double(d)); expect_false(file.next_double(d)); expect_false(file.next_double(d));
    expect_false(file.has_more());
}

test("exactunion removes excess and fills gaps while preserving background contents")
{
    auto map = solid_world();
    format::dplane_t p{}; p.normal[0] = 1; p.type = 0;
    format::dclipnode_t n{};
    n.planenum = 0; n.children[0] = bsp::contents_water; n.children[1] = bsp::contents_solid;
    map.planes = {p}; map.clipnodes = {n}; map.models[0].headnode[1] = 0;
    bsp::trim_group g;
    g.targets = {box({-10,-10,-10},{0,10,10})};
    auto a = box({-6,-10,-10},{4,10,10});
    auto b = box({2,-10,-10},{8,10,10});
    // Two oblique surfaces meet inside the overlap. The reference is their
    // analytic union, independently evaluated throughout the test lattice.
    const double k = 1 / std::sqrt(2.0);
    a.planes.push_back({{k,0,k},2*k});
    b.planes.push_back({{-k,0,k},-2*k});
    g.guards = g.additions = {a,b};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    for (double x = -9.75; x < 10; x += 0.5)
        for (double z = -9.9; z < 10; z += 0.5)
        {
            bool exact = (x >= -6 && x <= 4 && x+z <= 2)
                || (x >= 2 && x <= 8 && z-x <= -2);
            int expected = exact ? bsp::contents_solid
                : (x < 0 ? bsp::contents_empty : bsp::contents_water);
            expect(contents(map,0,1,{x,0,z}) == expected);
        }
    expect(contents(map,0,1,{-100,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{100,0,0}) == bsp::contents_water);
    expect(contents(map,1,1,{3,0,0}) == bsp::contents_solid);
    expect(contents(map,0,2,{3,0,0}) == bsp::contents_solid);
    expect(map.models[0].headnode[0] == 123);
}

test("exactunion preserves unselected guards and honors noclip")
{
    auto map = solid_world();
    bsp::trim_group g;
    g.targets = {box({-10,-10,-10},{10,10,10})};
    g.additions = {box({-5,-10,-10},{0,10,10})};
    g.guards = g.additions;
    g.guards.push_back(box({6,-10,-10},{10,10,10}));
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{8,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{3,0,0}) == bsp::contents_empty);
    auto empty = solid_world();
    empty.models[0].headnode[1] = bsp::contents_empty;
    state.map = &empty; state.options.noclip = true;
    bsp::trim_legacy_collision(state);
    expect(empty.models[0].headnode[1] == bsp::contents_empty);
    expect(empty.clipnodes.empty());
}

test("exactunion reclassifies false solid beyond source faces and retains captured fill")
{
    auto map = solid_world();
    bsp::trim_group g;
    // The original face BSP labels the entire region solid, including space
    // outside the raw source brush. Removal must extend beyond that brush.
    g.targets = {box({-4,-4,-4},{4,4,4})};
    g.guards = g.additions = {box({-2,-2,-2},{2,2,2})};
    g.exterior = {box({3,-5,-5},{5,5,5})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{-4.5,0,0}) == bsp::contents_empty);
    expect(contents(map,0,1,{0,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{4.5,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{100,0,0}) == bsp::contents_solid);
    // A protected brush is coverage, not authorization to fill an old gap.
    map = solid_world(); map.models[0].headnode[1] = bsp::contents_empty;
    g.guards.push_back(box({3,-4,-4},{4,4,4})); g.exterior.clear();
    state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{3.5,0,0}) == bsp::contents_empty);
    expect(contents(map,0,1,{0,0,0}) == bsp::contents_solid);
}

test("exactunion captures exterior paths for the current model and hull before collapse")
{
    auto map = solid_world();
    bsp::bsp_state state; state.map = &map; state.nummodels = 2; state.hullnum = 3;
    bsp::trim_group g; g.model = 1; g.hull = 3;
    g.targets = {box({-4,-4,-4},{4,4,4})};
    g.guards = g.additions = {box({-2,-2,-2},{2,2,2})};
    state.trim_groups = {g};
    bsp::plane p{}; p.normal[0] = 1; p.type = 0; state.planes = {p};
    bsp::node root, front, back;
    root.planenum = 0; root.children[0] = &front; root.children[1] = &back;
    front.exterior_filled = true;
    bsp::capture_collision_fill(state, &root);
    require(state.trim_groups[0].exterior.size() == 1);
    bsp::trim_legacy_collision(state);
    expect(contents(map,1,3,{4.5,0,0}) == bsp::contents_solid);
    expect(contents(map,1,3,{-4.5,0,0}) == bsp::contents_empty);
    expect(contents(map,0,3,{-4.5,0,0}) == bsp::contents_solid);
    expect(contents(map,1,1,{-4.5,0,0}) == bsp::contents_solid);
}

test("exactunion keeps tessellated planar collision compact")
{
    auto map = solid_world();
    bsp::trim_group g;
    const double diagonal = 1 / std::sqrt(2.0);
    const double slope = 1 / std::sqrt(1.13);
    for (int x = 0; x < 16; x += 4)
        for (int y = 0; y < 16; y += 4)
            for (int side = 0; side < 2; side++)
            {
                auto v = box({(double)x,(double)y,-4},{x+4.,y+4.,12});
                v.planes.push_back({{-0.2*slope,-0.3*slope,slope},2*slope});
                v.planes.push_back(side
                    ? math::plane{{-diagonal,-diagonal,0},-(x+y+4)*diagonal}
                    : math::plane{{diagonal,diagonal,0},(x+y+4)*diagonal});
                // A supporting plane touching only an edge cannot partition
                // the union's interior, including after fragment clipping.
                v.planes.push_back({{diagonal,0,-diagonal},(x+8)*diagonal});
                g.targets.push_back(v);
            }
    g.guards = g.additions = g.targets;
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(map.clipnodes.size() < 200);
    for (double x = -0.75; x < 17; x += 0.5)
        for (double y = -0.75; y < 17; y += 0.5)
            for (double z : {-4.25,-3.25,1.125,3.125,7.125,11.125})
            {
                bool exact = x >= 0 && x <= 16 && y >= 0 && y <= 16
                    && z >= -4 && z <= 0.2*x + 0.3*y + 2;
                expect(contents(map,0,1,{x,y,z})
                    == (exact ? bsp::contents_solid : bsp::contents_empty));
            }
}

test("exactunion prunes outside branches without changing distant contents")
{
    auto map = solid_world();
    format::dplane_t x{}, y{}, oblique{};
    x.normal[0] = 1; x.type = 0; x.dist = 100;
    y.normal[1] = -1; y.type = 4; y.dist = -200;
    oblique.normal[0] = oblique.normal[2] = float(1/std::sqrt(2.0));
    oblique.type = 3;
    format::dclipnode_t a{}, b{}, c{};
    a.planenum = 0; a.children[0] = 1; a.children[1] = bsp::contents_solid;
    b.planenum = 1; b.children[0] = 2; b.children[1] = bsp::contents_empty;
    c.planenum = 2; c.children[0] = bsp::contents_water; c.children[1] = bsp::contents_solid;
    map.planes = {x,y,oblique}; map.clipnodes = {a,b,c}; map.models[0].headnode[1] = 0;
    const auto original = map;
    bsp::trim_group g;
    g.targets = {box({-8,-8,-8},{8,8,8})};
    g.guards = g.additions = {box({-4,-4,-4},{4,4,4})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    for (double px : {-1e12,-1000.,-10.,10.,150.,1e12})
        for (double py : {-1e12,150.,250.,1e12})
            for (double pz : {-1e12,-1000.,1000.,1e12})
                expect(contents(map,0,1,{px,py,pz}) == contents(original,0,1,{px,py,pz}));
}

test("complete exactunion removes unbounded false solid while retaining liquid and other hulls")
{
    auto map = solid_world();
    format::dplane_t p{}; p.normal[0] = 1; p.dist = 1000; p.type = 0;
    format::dclipnode_t n{};
    n.planenum = 0; n.children[0] = bsp::contents_water; n.children[1] = bsp::contents_solid;
    map.planes = {p}; map.clipnodes = {n}; map.models[0].headnode[1] = 0;
    bsp::trim_group g; g.complete = true;
    g.targets = {box({-8,-8,-8},{8,8,8})};
    g.guards = g.additions = {box({-4,-4,-4},{4,4,4})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{0,0,0}) == bsp::contents_solid);
    for (double x : {-10000.,0.,999.})
        for (double y : {-10000.,10000.})
            for (double z : {-10000.,10000.})
                expect(contents(map,0,1,{x,y,z}) == bsp::contents_empty);
    expect(contents(map,0,1,{1001,0,0}) == bsp::contents_water);
    expect(contents(map,0,1,{1001,-10000,10000}) == bsp::contents_water);
    expect(contents(map,0,2,{0,10000,0}) == bsp::contents_solid);
    expect(contents(map,1,1,{0,10000,0}) == bsp::contents_solid);
    expect(map.models[0].headnode[0] == 123);
    expect(map.clipnodes.size() < 24);
    check_engine_hull_ranges(map);
}

test("complete exactunion preserves actual exterior fill beyond the selected bounds")
{
    auto map = solid_world();
    bsp::trim_group g; g.complete = true;
    g.targets = {box({-8,-8,-8},{8,8,8})};
    g.guards = g.additions = {box({-4,-4,-4},{4,4,4})};
    g.exterior = {box({6,-10,-10},{10,10,10})};
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{0,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{5,0,0}) == bsp::contents_empty);
    expect(contents(map,0,1,{7,0,0}) == bsp::contents_solid);
    expect(contents(map,0,1,{10000,0,0}) == bsp::contents_solid);
    check_engine_hull_ranges(map);
}

test("exactunion keeps cavities and disconnected solids through shared boundary clipping")
{
    auto map = solid_world();
    bsp::trim_group g;
    g.targets = {box({-24,-24,-24},{24,24,24})};
    for (int axis = 0; axis < 3; axis++)
        for (int side = 0; side < 2; side++)
        {
            math::vec3v lo{-8,-8,-8}, hi{8,8,8};
            if (side) lo[axis] = 6;
            else hi[axis] = -6;
            g.additions.push_back(box(lo, hi));
        }
    g.additions.push_back(g.additions[0]); // coincident outward faces
    g.additions.push_back(box({8,-8,-8},{12,8,8})); // opposite faces at the join
    g.additions.push_back(box({16,16,16},{20,20,20}));
    g.guards = g.additions;
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    for (double x = -21.75; x < 22; x += 1)
        for (double y = -21.75; y < 22; y += 1)
            for (double z = -21.75; z < 22; z += 1)
            {
                bool shell = std::fabs(x) < 8 && std::fabs(y) < 8 && std::fabs(z) < 8
                    && (std::fabs(x) > 6 || std::fabs(y) > 6 || std::fabs(z) > 6);
                bool joined = x > 8 && x < 12 && std::fabs(y) < 8 && std::fabs(z) < 8;
                bool separate = x > 16 && x < 20 && y > 16 && y < 20 && z > 16 && z < 20;
                expect(contents(map,0,1,{x,y,z})
                    == (shell || joined || separate ? bsp::contents_solid : bsp::contents_empty));
            }
}

test("exactunion preserves a tessellated nonplanar height field")
{
    auto map = solid_world();
    bsp::trim_group g;
    auto height = [](int x, int y) { return 4. + ((x*x + 3*y*y + x*y) % 7); };
    const double diagonal = 1 / std::sqrt(2.0);
    for (int x = 0; x < 6; x++)
        for (int y = 0; y < 6; y++)
            for (int side = 0; side < 2; side++)
            {
                auto v = box({4.*x,4.*y,-4},{4.*(x+1),4.*(y+1),16});
                double a = (side ? height(x+1,y+1)-height(x,y+1) : height(x+1,y)-height(x,y))/4;
                double b = (side ? height(x+1,y+1)-height(x+1,y) : height(x,y+1)-height(x,y))/4;
                double c = side ? height(x+1,y+1)-4*a*(x+1)-4*b*(y+1) : height(x,y)-4*a*x-4*b*y;
                double scale = 1/std::sqrt(a*a+b*b+1);
                v.planes.push_back({{-a*scale,-b*scale,scale},c*scale});
                v.planes.push_back(side
                    ? math::plane{{-diagonal,-diagonal,0},-4*(x+y+1)*diagonal}
                    : math::plane{{diagonal,diagonal,0},4*(x+y+1)*diagonal});
                g.additions.push_back(v);
            }
    g.guards = g.targets = g.additions;
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    for (double x = -0.75; x < 25; x += 0.5)
        for (double y = -0.75; y < 25; y += 0.5)
            for (double z : {-4.25,-3.25,3.125,5.125,7.125,9.125,11.125})
            {
                double top = -100;
                if (x > 0 && x < 24 && y > 0 && y < 24)
                {
                    int ix = int(x/4), iy = int(y/4);
                    double u = x/4-ix, v = y/4-iy;
                    top = u+v <= 1
                        ? height(ix,iy)*(1-u-v)+height(ix+1,iy)*u+height(ix,iy+1)*v
                        : height(ix+1,iy+1)*(u+v-1)+height(ix,iy+1)*(1-u)+height(ix+1,iy)*(1-v);
                }
                expect(contents(map,0,1,{x,y,z})
                    == (z >= -4 && z <= top ? bsp::contents_solid : bsp::contents_empty));
            }
}

test("exactunion handles large repeated brush sets with disconnected coverage")
{
    auto map = solid_world();
    bsp::trim_group g;
    g.targets = {box({-12,-12,-12},{12,12,12})};
    g.additions.assign(1100, box({-8,-8,-8},{-4,-4,-4}));
    g.additions.push_back(box({4,4,4},{8,8,8}));
    g.guards = g.additions;
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    expect(contents(map,0,1,{-6,-6,-6}) == bsp::contents_solid);
    expect(contents(map,0,1,{6,6,6}) == bsp::contents_solid);
    expect(contents(map,0,1,{0,0,0}) == bsp::contents_empty);
    expect(contents(map,0,1,{-6,6,-6}) == bsp::contents_empty);
    expect(map.clipnodes.size() < 30);
}

test("exactunion preserves thin walls and a narrow connected passage across split orders")
{
    auto map = solid_world();
    bsp::trim_group g;
    g.targets = {box({-12,-12,-12},{12,12,12})};
    // Alternate slopes force surface subdivision around both sides of a
    // through passage. Its width and the thin wall exceed disk roundoff.
    for (int i = 0; i < 8; i++)
        for (int side = 0; side < 2; side++)
        {
            double x = -8 + 2.*i;
            auto v = box({x,side ? 0.001 : -8.,-8},
                         {x+2,side ? 8. : -0.001,8});
            double slope = i % 2 ? -0.25 : 0.25;
            double scale = 1/std::sqrt(1+slope*slope);
            v.planes.push_back({{-slope*scale,0,scale},(4-slope*x)*scale});
            g.additions.push_back(v);
        }
    g.additions.push_back(box({-8,9,-8},{8,9.002,8}));
    g.guards = g.additions;
    bsp::bsp_state state; state.map = &map; state.trim_groups = {g};
    bsp::trim_legacy_collision(state);
    for (double x = -7.875; x < 8; x += 0.25)
    {
        int segment = int((x+8)/2);
        double top = 4 + (segment % 2 ? -0.25 : 0.25)*(x-(-8+2.*segment));
        for (double y : {-8.01,-7.,-0.0011,-0.0009,0.,0.0009,0.0011,7.,8.01,
                         8.9999,9.0001,9.001,9.0019,9.0021})
            for (double z : {-8.01,-7.99,0.,top-0.001,top+0.001,7.99,8.01})
            {
                bool bank = std::fabs(y) > 0.001 && std::fabs(y) < 8 && z > -8 && z < top;
                bool wall = y > 9 && y < 9.002 && z > -8 && z < 8;
                expect(contents(map,0,1,{x,y,z})
                    == (bank || wall ? bsp::contents_solid : bsp::contents_empty));
            }
    }
}
