#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "models.h"

namespace format
{
    struct source_map_data;
}

namespace decompile
{
    class material_catalog;
    struct source_displacement_mesh;

    struct displacement_model_result
    {
        std::vector<converted_model> models;
        std::vector<prop_placement> placements;
        std::size_t chunks = 0;
        std::size_t triangles = 0;
        std::size_t vertices = 0;
        std::size_t skins = 0;
        std::size_t missing_skins = 0;
        std::size_t failed = 0;
    };

    // rebuilds displacement terrain as goldsrc studio models
    // as brushes it overruns max_map_planes/nodes/verts; models cost no lumps
    // collision stays with the brushes
    //
    // chunk_size trades lighting resolution against duplicating skins per chunk
    displacement_model_result build_displacement_models(
        const format::source_map_data &map, const material_catalog &catalog,
        const source_displacement_mesh &mesh, const std::string &map_name,
        double chunk_size);
}
