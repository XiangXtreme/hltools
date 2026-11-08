#include "bsp/internal.h"
#include "support/scratch.h"
#include "support/test.h"

namespace
{
    direct_clip::tree box_tree(int node_count)
    {
        direct_clip::tree tree;
        tree.planes = {{{1, 0, 0}, 40},  {{-1, 0, 0}, 10}, {{0, 1, 0}, 50},
                       {{0, -1, 0}, 20}, {{0, 0, 1}, 60},  {{0, 0, -1}, 30}};
        // Additional nonaxial constraints leave the box unchanged while
        // exercising the large-tree writer path used by terrain.
        while ((int)tree.planes.size() < node_count)
            tree.planes.push_back({{0.6, 0.8, 0}, 200});
        for (int i = 0; i < node_count; i++)
            tree.nodes.push_back({i, bsp::contents_empty, i == 0 ? bsp::contents_solid : i - 1});
        tree.head = node_count - 1;
        tree.mins = {-10, -20, -30};
        tree.maxs = {40, 50, 60};
        tree.valid = true;
        return tree;
    }

    int engine_contents(const format::map_data &map, int head, const math::vec3v &point)
    {
        // SV_HullPointContents and player movement use the coordinate alone
        // for axial planes; a negative axial normal is not consulted.
        int node = head;
        for (size_t steps = 0; node >= 0; steps++)
        {
            require(steps < map.clipnodes.size());
            require((size_t)node < map.clipnodes.size());
            const auto &clip = map.clipnodes[(size_t)node];
            require(clip.planenum >= 0);
            require((size_t)clip.planenum < map.planes.size());
            const auto &plane = map.planes[(size_t)clip.planenum];
            const vec_t distance = plane.type < 3 ? point[plane.type] - plane.dist
                                                  : math::dot(point, plane.normal) - plane.dist;
            node = clip.children[distance < 0 ? 1 : 0];
        }
        return node;
    }
} // namespace

test("write_bsp preserves direct surface collision across the large tree boundary")
{
    const auto root = test_support::scratch_directory("bsp_direct_surface_planes");
    for (const int count : {6, 100, 101})
    {
        capture(count);
        format::map_data map;
        map.models.resize(2);
        // Keep an existing world hull to exercise appended node offsets.
        format::dclipnode_t world = {};
        world.children[0] = world.children[1] = bsp::contents_empty;
        map.clipnodes.push_back(world);

        bsp::bsp_state state;
        state.map = &map;
        state.base_path = (root / "output").string();
        state.options.noopt = true;
        state.options.nobrink = true;
        bsp::plane plane;
        plane.normal = {1, 0, 0};
        plane.dist = 1000;
        plane.type = 0;
        state.planes.push_back(plane);
        state.surface_clips.resize(2);
        auto &direct = state.surface_clips[1];
        direct.hulls[1] = direct.hulls[3] = box_tree(count);
        direct.valid = true;

        bsp::finish_bsp_file(state);
        expect(map.clipnodes.size() == (size_t)(1 + 2 * count));
        expect(map.models[1].headnode[1] == count);
        expect(map.models[1].headnode[2] == count);
        expect(map.models[1].headnode[3] == 2 * count);
        expect(map.clipnodes[0].children[0] == bsp::contents_empty);
        expect(map.clipnodes[0].children[1] == bsp::contents_empty);

        for (int hull = 1; hull <= 3; hull++)
        {
            capture(hull);
            for (const auto point : {math::vec3v{0, 0, 0},
                                     {-11, 0, 0},
                                     {41, 0, 0},
                                     {0, -21, 0},
                                     {0, 51, 0},
                                     {0, 0, -31},
                                     {0, 0, 61}})
            {
                capture(point.x, point.y, point.z);
                const int expected =
                    point == math::vec3v{0, 0, 0} ? bsp::contents_solid : bsp::contents_empty;
                expect(engine_contents(map, map.models[1].headnode[hull], point) == expected);
            }
        }
        for (size_t i = 1; i < map.clipnodes.size(); i++)
        {
            const auto &p = map.planes[(size_t)map.clipnodes[i].planenum];
            if (p.type < 3)
                expect(p.normal[p.type] == 1);
        }
    }
}
