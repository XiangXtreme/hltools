#include "common/threads.h"
#include "format/bsp/entity_lump.h"
#include "support/scratch.h"
#include "support/test.h"
#include "vis/vis.h"

#include <fstream>
#include <string>
#include <vector>

namespace
{
    format::entity entity(const char *classname, int leaf, const char *target = nullptr,
                          int neighbor = 0)
    {
        format::entity result;
        result.append("classname", classname);
        result.append("origin", std::to_string(leaf * 10) + " 0 0");
        if (target)
            result.append(std::string(classname) == "info_leaf" ? "targetname" : "target", target);
        if (neighbor)
            result.append("neighbor", std::to_string(neighbor));
        return result;
    }

    format::map_data map_for_entities(const std::vector<format::entity> &entities)
    {
        format::map_data map;
        map.entities = format::write_entities(entities);
        map.models.resize(1);
        map.models[0].visleafs = 7;
        map.leafs.resize(8); // the first BSP leaf is solid
        map.nodes.resize(6);
        map.planes.resize(6);
        for (int i = 0; i < 6; i++)
        {
            map.planes[(size_t)i].normal[0] = 1;
            map.planes[(size_t)i].dist = (float)(i * 10 + 5);
            map.nodes[(size_t)i].planenum = i;
            map.nodes[(size_t)i].children[0] = (short)(i == 5 ? -8 : i + 1);
            map.nodes[(size_t)i].children[1] = (short)(-i - 2);
        }
        return map;
    }

    format::map_data calculate(const std::vector<format::entity> &entities, bool fast,
                               const std::string &portal_path, bool full = true)
    {
        format::map_data map = map_for_entities(entities);
        vis::vis_options options;
        options.portal_path = portal_path;
        options.fast = fast;
        options.full = full;
        options.nofixprt = true;
        vis::run_vis(map, options);
        return map;
    }

    bool sees(const format::map_data &map, int source, int target)
    {
        size_t cursor = (size_t)map.leafs[(size_t)source + 1].visofs;
        int output_byte = 0;
        while (output_byte <= target / 8)
        {
            const unsigned char value = map.visibility.at(cursor++);
            if (value == 0)
            {
                const int skipped = map.visibility.at(cursor++);
                if (target / 8 < output_byte + skipped)
                    return false;
                output_byte += skipped;
            }
            else
            {
                if (output_byte == target / 8)
                    return (value & (1 << (target % 8))) != 0;
                output_byte++;
            }
        }
        return false;
    }

    std::string write_portals(const char *name)
    {
        const auto root = test_support::scratch_directory(name);
        const auto path = root / "fixture.prt";
        std::ofstream file(path);
        // Leaves 4, 5 and 6 are isolated. Leaf 3 is two hops from leaf 0
        // via 2, although 2 can also be reached first through 1.
        file << "7\n4\n1\n1\n1\n1\n1\n1\n1\n"
             << "3 0 1 (0 0 0) (0 1 0) (0 0 1)\n"
             << "3 1 2 (10 0 0) (10 1 0) (10 0 1)\n"
             << "3 0 2 (20 0 0) (20 1 0) (20 0 1)\n"
             << "3 2 3 (30 0 0) (30 1 0) (30 0 1)\n";
        file.close();
        require(file.good());
        return path.string();
    }
}

suite("unit.vis.portals")
{
    test("missing portal targets do not introduce visibility")
    {
        const std::string prt = write_portals("vis_missing_target");
        threads::set_count(2);
        const std::vector<format::entity> baseline = {entity("worldspawn", 0)};
        auto invalid = baseline;
        invalid.push_back(entity("info_portal", 6, "missing", 1));
        for (bool fast : {false, true})
        {
            const auto without = calculate(baseline, fast, prt);
            const auto with = calculate(invalid, fast, prt);
            for (int from = 0; from < 7; from++)
                for (int to = 0; to < 7; to++)
                    expect(sees(without, from, to) == sees(with, from, to));
        }
    }

    test("each portal keeps its own neighbor depth and follows shortest paths")
    {
        const std::string prt = write_portals("vis_neighbor_depth");
        threads::set_count(2);
        const auto deep = entity("info_portal", 0, "deep", 2);
        const auto local = entity("info_portal", 0, "local");
        const auto deep_target = entity("info_leaf", 4, "deep");
        const auto local_target = entity("info_leaf", 5, "local");
        for (bool fast : {false, true})
        {
            const auto a = calculate({deep, local, deep_target, local_target}, fast, prt);
            const auto b = calculate({local, deep, deep_target, local_target}, fast, prt);
            for (int from = 0; from < 7; from++)
                for (int to = 0; to < 7; to++)
                    expect(sees(a, from, to) == sees(b, from, to));

            expect(sees(a, 0, 4));
            expect(sees(a, 1, 4));
            expect(sees(a, 2, 4));
            expect(sees(a, 3, 4));
            expect(sees(a, 0, 5));
            expect_false(sees(a, 1, 5));
            expect_false(sees(a, 2, 5));
            expect_false(sees(a, 3, 5));
            expect_false(sees(a, 4, 0)); // the legacy link is one-way
        }
    }

    test("new portal targets support multiple entities and both neighbor clusters")
    {
        const std::string prt = write_portals("vis_multi_target");
        threads::set_count(2);
        auto portal = entity("info_portal", 4, "a; b");
        portal.append("targetneighbor", "1");
        auto a = entity("info_target", 0);
        a.append("targetname", "a");
        auto b = entity("light", 5);
        b.append("targetname", "b");
        for (bool fast : {false, true})
        {
            const auto result = calculate({portal, a, b}, fast, prt);
            for (int target : {0, 1, 2, 5})
            {
                expect(sees(result, 4, target));
                expect(sees(result, target, 4));
            }
            expect_false(sees(result, 4, 3));
        }
    }

    test("portal oneway and reverse apply after normal visibility")
    {
        const std::string prt = write_portals("vis_reverse");
        threads::set_count(2);
        auto target = entity("info_target", 1);
        target.append("targetname", "a");
        auto portal = entity("info_portal", 0, "a");
        portal.append("reverse", "1");
        portal.append("oneway", "1");
        for (bool fast : {false, true})
        {
            const auto baseline = calculate({target}, fast, prt);
            const auto reversed = calculate({portal, target}, fast, prt);
            expect(sees(baseline, 0, 1));
            expect(sees(baseline, 1, 0));
            expect_false(sees(reversed, 0, 1));
            expect(sees(reversed, 1, 0));
            expect(sees(reversed, 0, 0));
        }
    }

    test("explicit oneway zero selects two-way behavior for an info_leaf")
    {
        const std::string prt = write_portals("vis_legacy_explicit");
        threads::set_count(2);
        auto portal = entity("info_portal", 4, "a");
        portal.append("oneway", "0");
        const auto result = calculate({portal, entity("info_leaf", 5, "a")}, true, prt);
        expect(sees(result, 4, 5));
        expect(sees(result, 5, 4));
    }

    test("legacy duplicate info_leaf names keep the last target")
    {
        const std::string prt = write_portals("vis_legacy_duplicate");
        threads::set_count(2);
        const auto portal = entity("info_portal", 6, "same");
        const auto first = entity("info_leaf", 4, "same");
        const auto last = entity("info_leaf", 5, "same");
        const auto result = calculate({portal, first, last}, true, prt);
        expect_false(sees(result, 6, 4));
        expect(sees(result, 6, 5));
        expect_false(sees(result, 5, 6));
    }

    test("a brush target contributes the leaves touched by its faces")
    {
        const std::string prt = write_portals("vis_brush_target");
        threads::set_count(2);
        auto target = entity("func_wall", 0);
        target.append("targetname", "brush");
        target.append("model", "*1");
        auto map = map_for_entities({entity("info_portal", 6, "brush"), target});
        map.models.resize(2);
        map.models[1].firstface = 0;
        map.models[1].numfaces = 1;
        map.planes.resize(7);
        map.planes[6].normal[1] = 1;
        map.faces.resize(1);
        map.faces[0].planenum = 6;
        map.faces[0].numedges = 3;
        map.vertexes = {{{40, 0, 0}}, {{50, 0, 0}}, {{40, 0, 5}}};
        map.edges = {{{0, 1}}, {{1, 2}}, {{2, 0}}};
        map.surfedges = {0, 1, 2};
        vis::vis_options options;
        options.portal_path = prt;
        options.fast = true;
        options.nofixprt = true;
        vis::run_vis(map, options);
        for (int leaf : {4, 5})
        {
            expect(sees(map, 6, leaf));
            expect(sees(map, leaf, 6));
        }
    }

    test("entity links and overview points map through grouped BSP leaves")
    {
        const auto root = test_support::scratch_directory("vis_grouped_leaves");
        const auto path = root / "fixture.prt";
        std::ofstream file(path);
        file << "4\n0\n2\n1\n2\n2\n";
        file.close();
        require(file.good());
        threads::set_count(2);

        const auto linked = calculate({entity("info_portal", 1, "target"),
                                       entity("info_leaf", 6, "target")},
                                      true, path.string());
        for (int source : {0, 1})
        {
            expect(sees(linked, source, 5));
            expect(sees(linked, source, 6));
            expect_false(sees(linked, source, 4));
        }
        expect_false(sees(linked, 2, 5));

        auto reverse = entity("info_overview_point", 3);
        reverse.append("reverse", "1");
        const auto overview = calculate({entity("info_overview_point", 2), reverse},
                                        true, path.string());
        for (int target = 0; target < 7; target++)
            expect(sees(overview, 2, target));
        for (int source = 0; source < 7; source++)
        {
            expect(sees(overview, source, 3));
            expect(sees(overview, source, 4));
        }
        expect_false(sees(overview, 0, 5));
    }

    test("full and normal visibility agree across worker counts")
    {
        const std::string prt = write_portals("vis_worker_counts");
        const std::vector<format::entity> entities = {entity("worldspawn", 0)};
        for (bool full : {false, true})
        {
            threads::set_count(1);
            const auto serial = calculate(entities, false, prt, full);
            threads::set_count(4);
            const auto parallel = calculate(entities, false, prt, full);
            for (int from = 0; from < 7; from++)
                for (int to = 0; to < 7; to++)
                    expect(sees(serial, from, to) == sees(parallel, from, to));
        }
    }
}
