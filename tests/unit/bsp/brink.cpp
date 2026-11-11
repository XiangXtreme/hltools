#include "bsp/internal.h"
#include "support/test.h"

test("brink preserves empty hulls and shared subtrees")
{
    for (const bool shared : {false, true})
    {
        capture(shared);
        format::map_data map;
        map.models.resize(2);
        for (auto &model : map.models)
            for (int hull = 1; hull < 4; hull++)
                model.headnode[hull] = bsp::contents_empty;
        map.models[1].headnode[3] = bsp::contents_water;

        bsp::bsp_state state;
        state.map = &map;
        bsp::plane x{}, y{};
        x.normal = {1, 0, 0};
        x.type = 0;
        y.normal = {0, 1, 0};
        y.type = 1;
        y.dist = 32;
        state.planes = {x, y};
        if (shared)
        {
            format::dclipnode_t root{}, child{};
            root.planenum = 0;
            root.children[0] = root.children[1] = 1;
            child.planenum = 1;
            child.children[0] = bsp::contents_empty;
            child.children[1] = bsp::contents_solid;
            map.clipnodes = {root, child};
            map.models[0].headnode[1] = map.models[0].headnode[3] = 0;
        }
        bsp::fix_all_brinks(state);
        expect(map.models[0].headnode[2] == bsp::contents_empty);
        expect(map.models[1].headnode[3] == bsp::contents_water);
        for (int hull : {1, 3})
            for (const vec_t px : {-8., 8.})
                for (const vec_t py : {31., 33.})
                {
                    const math::vec3v point{px, py, 0};
                    int node = map.models[0].headnode[hull];
                    while (node >= 0)
                    {
                        const auto &clip = map.clipnodes.at((size_t)node);
                        const auto &plane = state.planes.at((size_t)clip.planenum);
                        node = clip.children[math::dot(point, plane.normal) >= plane.dist ? 0 : 1];
                    }
                    expect(node == (shared && py < 32 ? bsp::contents_solid : bsp::contents_empty));
                }
    }
}
