#include <string>
#include <vector>

#include "format/bsp/data.h"
#include "format/bsp/hull_geometry.h"
#include "support/test.h"

namespace
{
    format::map_data make_quad_map()
    {
        format::map_data map;

        format::dmodel_t model{};
        model.firstface = 0;
        model.numfaces = 1;
        map.models.push_back(model);

        const float corners[4][3] = {{0, 0, 0}, {64, 0, 0}, {64, 64, 0}, {0, 64, 0}};
        for (const auto &corner : corners)
        {
            format::dvertex_t vertex{};
            vertex.point[0] = corner[0];
            vertex.point[1] = corner[1];
            vertex.point[2] = corner[2];
            map.vertexes.push_back(vertex);
        }

        for (int i = 0; i < 4; i++)
        {
            format::dedge_t edge{};
            edge.v[0] = (unsigned short)i;
            edge.v[1] = (unsigned short)((i + 1) % 4);
            map.edges.push_back(edge);
            map.surfedges.push_back(i);
        }

        format::dface_t face{};
        face.firstedge = 0;
        face.numedges = 4;
        face.texinfo = 2;
        map.faces.push_back(face);
        return map;
    }
}

suite("unit.format.hull_geometry")
{
    test("hull_geometry.collects every model polygon in order")
    {
        const format::map_data map = make_quad_map();
        std::vector<format::hull0_face> faces;
        std::string error;
        require(format::collect_hull0_faces(map, 0, faces, &error));

        require(faces.size() == 1);
        expect(faces[0].model == 0);
        expect(faces[0].texinfo == 2);
        require(faces[0].points.size() == 4);
        expect(faces[0].points[0][0] == 0.0f);
        expect(faces[0].points[1][0] == 64.0f);
        expect(faces[0].points[2][1] == 64.0f);
        expect(faces[0].points[3][1] == 64.0f);
    }

    test("hull_geometry.honours reversed surfedges")
    {
        format::map_data map = make_quad_map();
        // -1 selects edge 1's second vertex; point that edge back at vertex 1
        map.edges[1].v[0] = 0;
        map.edges[1].v[1] = 1;
        map.surfedges[0] = -1;
        std::vector<format::hull0_face> faces;
        require(format::collect_hull0_faces(map, 0, faces, nullptr));

        require(faces.size() == 1);
        require(faces[0].points.size() == 4);
        // negative surfedge selects edge.v[1], so the first point is vertex 1
        expect(faces[0].points[0][0] == 64.0f);
        expect(faces[0].points[0][1] == 0.0f);
    }

    test("hull_geometry.rejects a bad model and a bad surfedge")
    {
        const format::map_data map = make_quad_map();
        std::vector<format::hull0_face> faces;
        std::string error;
        expect(!format::collect_hull0_faces(map, 1, faces, &error));
        expect(!error.empty());

        format::map_data broken = make_quad_map();
        broken.surfedges[2] = 99;
        expect(!format::collect_hull0_faces(broken, 0, faces, &error));
        expect(!error.empty());
    }
}
