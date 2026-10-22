#include <cmath>
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
