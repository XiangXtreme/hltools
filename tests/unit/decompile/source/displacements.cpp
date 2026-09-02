#include <cmath>

#include "decompile/source/displacements.h"
#include "format/vbsp/data.h"
#include "support/test.h"

namespace
{
    format::source_dvertex_t vertex(float x, float y, float z)
    {
        format::source_dvertex_t out{};
        out.point[0] = x;
        out.point[1] = y;
        out.point[2] = z;
        return out;
    }

    bool near(double a, double b)
    {
        return std::fabs(a - b) < 0.00001;
    }

    // a single flat quad carrying one displacement of the given power, with
    // every vertex pushed up by one unit and the start corner pushed by five
    format::source_map_data one_displacement(int power)
    {
        format::source_map_data map;
        format::source_dplane_t plane{};
        plane.normal[2] = 1;
        map.planes.push_back(plane);

        map.vertexes = {
            vertex(0, 0, 0),
            vertex(0, 4, 0),
            vertex(4, 4, 0),
            vertex(4, 0, 0),
        };
        map.edges = {{{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}}};
        map.surfedges = {0, 1, 2, 3};

        format::source_dface_t face{};
        face.planenum = 0;
        face.firstedge = 0;
        face.numedges = 4;
        face.texinfo = 7;
        face.dispinfo = 0;
        map.faces.push_back(face);

        format::source_ddispinfo_t info{};
        info.start_position[0] = 0;
        info.start_position[1] = 4;
        info.start_position[2] = 0;
        info.disp_vert_start = 0;
        info.power = power;
        info.map_face = 0;
        map.dispinfo.push_back(info);

        int side = (1 << power) + 1;
        map.disp_verts.resize((std::size_t)(side * side));
        for (format::source_ddispvert_t &disp : map.disp_verts)
        {
            disp.vector[2] = 1;
            disp.dist = 1;
        }
        map.disp_verts[0].dist = 5;
        return map;
    }
}

suite("source displacement reconstruction")
{
    test("rotates the base quad to startPosition and emits the full power grid")
    {
        format::source_map_data map = one_displacement(2);

        decompile::source_displacement_mesh mesh =
            decompile::build_source_displacement_mesh(map);
        require(mesh.faces == 1);
        expect(mesh.skipped_faces == 0);
        expect(mesh.skipped_triangles == 0);
        require(mesh.triangles.size() == 32);
        // a power-2 grid is already at the power-2 collision cap, so the
        // triangles carry their own collision instead of duplicating it
        require(mesh.collision_triangles.empty());
        for (const decompile::source_displacement_triangle &triangle
             : mesh.triangles)
            expect(triangle.solid);

        bool found_start = false;
        for (const decompile::source_displacement_triangle &triangle
             : mesh.triangles)
        {
            expect(triangle.texinfo == 7);
            expect(near(triangle.minimum_projection, 1));
            math::vec3<double> edge0 =
                triangle.points[1] - triangle.points[0];
            math::vec3<double> edge1 =
                triangle.points[2] - triangle.points[0];
            expect(math::dot(math::cross(edge0, edge1),
                             triangle.base_normal) > 0);
            for (const math::vec3<double> &point : triangle.points)
            {
                expect(point.x >= 0);
                expect(point.x <= 4);
                expect(point.y >= 0);
                expect(point.y <= 4);
                if (near(point.x, 0) && near(point.y, 4) && near(point.z, 5))
                    found_start = true;
            }
        }
        expect(found_start);
    }

    test("caps collision below the drawn grid once drawing is finer")
    {
        format::source_map_data map = one_displacement(3);

        // merging off, so the collision grid is comparable triangle for triangle
        decompile::source_displacement_mesh mesh =
            decompile::build_source_displacement_mesh(map, 4, true, 0.0);
        require(mesh.faces == 1);
        // drawn at power 3, collided at the power-2 cap, so the two differ and
        // the drawn shell hands its collision to a separate clip mesh
        require(mesh.triangles.size() == 128);
        require(mesh.collision_triangles.size() == 32);
        for (const decompile::source_displacement_triangle &triangle
             : mesh.triangles)
            expect(!triangle.solid);
    }

    test("merges flat collision cells into single quads")
    {
        format::source_map_data map = one_displacement(3);

        decompile::source_displacement_mesh merged =
            decompile::build_source_displacement_mesh(map, 4, true, 48.0);
        decompile::source_displacement_mesh plain =
            decompile::build_source_displacement_mesh(map, 4, true, 0.0);

        // 16 collision cells, each two triangles, become 16 quads. Even the
        // cell holding the pushed start corner merges: the best-fit plane tilts
        // to meet it, leaving every corner about a third of a unit off.
        require(plain.collision_triangles.size() == 32);
        require(merged.collision_triangles.size() == 16);
        for (const decompile::source_displacement_triangle &face
             : merged.collision_triangles)
            expect(face.point_count == 4);

        // tightened below that residual, the pushed cell stays split
        decompile::source_displacement_mesh tight =
            decompile::build_source_displacement_mesh(map, 4, true, 0.1);
        std::size_t quads = 0;
        std::size_t triangles = 0;
        for (const decompile::source_displacement_triangle &face
             : tight.collision_triangles)
            (face.point_count == 4 ? quads : triangles)++;
        expect(quads == 15);
        expect(triangles == 2);

        // the drawn mesh is never merged, whatever the tolerance
        for (const decompile::source_displacement_triangle &face
             : merged.triangles)
            expect(face.point_count == 3);
    }

    test("draw_power_cap decimates the grid and keeps it solid")
    {
        format::source_map_data map = one_displacement(2);

        decompile::source_displacement_mesh mesh =
            decompile::build_source_displacement_mesh(map, 1);
        require(mesh.faces == 1);
        expect(mesh.skipped_triangles == 0);
        // a power-2 displacement sampled on a power-1 grid: 4 quads, not 16
        require(mesh.triangles.size() == 8);
        require(mesh.collision_triangles.empty());

        // the cap samples real displaced vertices, so the pushed start corner
        // still survives at its full height
        bool found_start = false;
        for (const decompile::source_displacement_triangle &triangle
             : mesh.triangles)
        {
            expect(triangle.solid);
            for (const math::vec3<double> &point : triangle.points)
                if (near(point.x, 0) && near(point.y, 4) && near(point.z, 5))
                    found_start = true;
        }
        expect(found_start);
    }

    test("a cap above the displacement's own power changes nothing")
    {
        format::source_map_data map = one_displacement(2);

        decompile::source_displacement_mesh capped =
            decompile::build_source_displacement_mesh(map, 4);
        decompile::source_displacement_mesh plain =
            decompile::build_source_displacement_mesh(map);
        require(capped.triangles.size() == plain.triangles.size());
        require(capped.triangles.size() == 32);
    }
}
