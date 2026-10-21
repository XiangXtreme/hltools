#include "hull_geometry.h"

#include <algorithm>
#include <cstdlib>

namespace format
{
    namespace
    {
        void set_error(std::string *error, const std::string &message)
        {
            if (error)
                *error = message;
        }
    }

    bool collect_hull0_faces(const map_data &map, int model,
                             std::vector<hull0_face> &faces, std::string *error)
    {
        faces.clear();
        if (model < 0 || (std::size_t)model >= map.models.size())
        {
            set_error(error, "model index is outside the model lump");
            return false;
        }
        const dmodel_t &info = map.models[(std::size_t)model];
        if (info.firstface < 0 || info.numfaces < 0
            || (std::size_t)info.firstface + (std::size_t)info.numfaces > map.faces.size())
        {
            set_error(error, "model face range is outside the face lump");
            return false;
        }

        for (int f = 0; f < info.numfaces; f++)
        {
            const dface_t &face = map.faces[(std::size_t)info.firstface + (std::size_t)f];
            if (face.firstedge < 0 || face.numedges < 0
                || (std::size_t)face.firstedge + (std::size_t)face.numedges
                    > map.surfedges.size())
            {
                set_error(error, "face edge range is outside the surfedge lump");
                return false;
            }

            hull0_face out;
            out.model = model;
            out.texinfo = face.texinfo;
            for (int e = 0; e < face.numedges; e++)
            {
                const int index = map.surfedges[(std::size_t)face.firstedge + (std::size_t)e];
                const std::size_t edge_index = (std::size_t)std::abs(index);
                if (edge_index >= map.edges.size())
                {
                    set_error(error, "surfedge points outside the edge lump");
                    return false;
                }
                const int vertex = index >= 0 ? map.edges[edge_index].v[0]
                                              : map.edges[edge_index].v[1];
                if (vertex < 0 || (std::size_t)vertex >= map.vertexes.size())
                {
                    set_error(error, "edge points outside the vertex lump");
                    return false;
                }
                const dvertex_t &point = map.vertexes[(std::size_t)vertex];
                out.points.push_back({point.point[0], point.point[1], point.point[2]});
            }

            if (out.points.size() >= 3)
                faces.push_back(std::move(out));
        }
        return true;
    }
}
