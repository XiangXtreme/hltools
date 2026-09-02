#pragma once

#include <cstddef>
#include <set>
#include <string>
#include <vector>

#include "format/miptex/texture.h"
#include "models.h"
#include "skybox.h"

namespace format
{
    struct source_map_data;
}

namespace decompile
{
    struct source_options
    {
        // source content root for materials not embedded in the pakfile.
        // reserved for the material pipeline; unused by geometry porting.
        std::vector<std::string> game_dirs;
        // companion wad appended to worldspawn's wad list, as for the goldsrc
        // decompiler. reserved for the material pipeline.
        std::string additional_wad;
        // wad supplying the engine textures (NULL/SKY/AAATRIGGER/...), normally
        // sdhlt.wad. appended to worldspawn's wad list after the companion wad.
        std::string tool_wad;
        // edge length of the exported gfx/env sky faces. every stock goldsrc sky
        // is 256; values through 4096 support the skybox-model workflow; 0
        // disables skybox export.
        unsigned sky_size = 256;
        // linear RGB multiplier applied only to exported gfx/env sky faces.
        double sky_exposure = 1.0;
        // maximum WAD texture dimension. Lower values preserve UV coverage while
        // trading image resolution for fewer BSP subdivisions and lightmap luxels.
        unsigned max_texture_size = 512;
        // texture size lightmap density is derived from, decoupled from the
        // stored size, so a texture keeps its pixels and tiles over more world
        // 0 follows max_texture_size
        unsigned lightmap_texture_size = 0;
        // reduce single-colour textures to the minimum legal dimension
        // they carry no detail, so this is visually identical
        bool auto_flat_textures = true;
        // Source material names (or generated GoldSrc basenames) that bypass the
        // global cap and retain their native dimensions, up to GoldSrc's 512 limit.
        std::vector<std::string> full_size_textures;
        // rebuild the map's studio models as goldsrc models and place them as
        // entities. off leaves the map brush only.
        bool convert_models = true;
        // maximum edge length of converted model skins.
        unsigned max_skin_size = 512;
        // maximum tiles per axis; 1 disables tiling.
        int skin_chunk_level = 1;
        // use one palette per material instead of per tile.
        bool shared_tile_palette = false;
        // fraction of surface area receiving dedicated tiles.
        float tile_area_keep = 1.0f;
        // rebuild .phy convexes as invisible solid brush entities.
        bool physics_clips = false;
        // cap on the drawn tessellation, 0..4, sampled on a 2^min(power, cap) grid
        // the lever that brings a displacement-heavy map under the plane and
        // face limits; 4 keeps source's full detail
        int displacement_power = 4;
        // draw the displacement surface from studio models, which cost no planes,
        // nodes, faces or verts
        // a model does not collide, so the brushes below keep the collision
        bool displacement_models = false;
        // edge length of a terrain model chunk
        // the engine lights a model from 1 sample at its origin, so smaller
        // chunks light better and cost another copy of every skin they touch
        double displacement_chunk = 1024.0;
        // which displacements keep brush collision
        // floors drops walls and ceilings, which is what brings a large terrain
        // under max_map_clipnodes once the surface is drawn from models
        enum class collision_scope
        {
            all,
            floors,
            none
        };
        collision_scope displacement_clip = collision_scope::all;
        // how far a collision cell's corners may move onto a single plane,
        // so the cell is 1 brush instead of 2
        // higher trades surface accuracy for cheaper brushes
        double displacement_merge = 48.0;
        // gives generated terrain models a home under models/<map_name>/
        // empty writes them to models/
        std::string map_name;
    };

    struct source_result
    {
        std::string text;
        std::size_t entities = 0;
        std::size_t world_brushes = 0;
        std::size_t entity_brushes = 0;
        std::size_t detail_brushes = 0; // world detail routed to func_detail
        std::size_t window_brushes = 0; // translucent brushes routed to func_wall
        std::size_t masked_brushes = 0;  // world brushes carrying a '{' texture
        std::size_t masked_entities = 0; // brush entities retagged rendermode 4
        // brush entities that inherited a translucent material's opacity
        std::size_t translucent_entities = 0;
        std::size_t water_brushes = 0;  // liquid brushes routed to func_water
        std::size_t clip_brushes = 0;   // pure playerclip volumes given CLIP
        std::size_t physics_clip_brushes = 0; // .phy clip brushes placed
        std::size_t physics_clip_props = 0;   // props that got .phy collision
        std::size_t trigger_noclip_entities = 0; // triggers spared clip hulls
        // source 3d skybox brushes excluded until their scene is converted
        std::size_t skybox_brushes = 0;
        std::size_t sides = 0;
        std::size_t skipped_brushes = 0;
        std::size_t skipped_sides = 0;
        std::size_t realigned_sides = 0; // edge-on texture axes replaced
        std::size_t displacement_faces = 0;
        std::size_t displacement_brushes = 0;
        std::size_t displacement_collision_brushes = 0;
        std::size_t skipped_displacement_faces = 0;
        std::size_t skipped_displacement_triangles = 0;
        std::size_t converted_materials = 0;
        std::size_t placeholder_materials = 0;
        std::size_t masked_materials = 0; // converted with a '{' transparency mask
        // single-colour materials reduced to 16x16, dropping their density too
        std::size_t flattened_materials = 0;
        // UnlitGeneric materials exported through info_unlittextures so RAD
        // gives their faces constant-white fullbright lightmaps.
        std::size_t unlit_materials = 0;
        // engine textures the map names (NULL/CLIP/SKY/...) that no source
        // material provides. they carry meaning to csg rather than pixels, but
        // still need a real wad entry, so a tool wad like sdhlt.wad must be on
        // the map's wad list for these to resolve.
        std::set<std::string> engine_textures;
        std::size_t dropped_spawns = 0; // extra Source spawn entities removed
        bool player_start = false; // the ported map has one info_player_start
        std::size_t fog_controllers = 0; // env_fog_controller -> env_fog
        std::size_t cameras = 0;         // point_viewcontrol -> trigger_camera
        std::size_t converted_lights = 0; // light* with source keys renamed
        // world brushes routed to func_wall rendermode 5 for an $additive material
        std::size_t additive_brushes = 0;
        std::size_t additive_entities = 0; // brush entities retagged rendermode 5
        // far clip distance written to worldspawn, sized to the map's own extent
        long max_range = 0;

        // companion wad textures built from the map's materials, to be written
        // by the caller alongside the map.
        std::vector<format::mip_texture> wad_textures;

        // gfx/env sky faces converted from the source 2d skybox, to be written
        // by the caller alongside the map. skybox.sky_name is already stored in
        // worldspawn's "skyname".
        skybox_result skybox;

        // studio models rebuilt for goldsrc, to be written by the caller. their
        // placements are already emitted into the map as entities.
        model_result models;
        std::size_t prop_entities = 0; // static props turned back into entities
        // displacement terrain drawn from studio models rather than brushes
        std::size_t terrain_chunks = 0;
        std::size_t terrain_entities = 0;
        std::size_t terrain_triangles = 0;
        std::size_t terrain_skins = 0;
        std::size_t failed_terrain_chunks = 0;
        // inert brush entities folded together to stay under the model limit
        std::size_t merged_brush_entities = 0;
        std::size_t brush_entity_groups = 0;
        std::size_t brush_models = 0; // brush entities left in the ported map
    };

    // ports a source engine map to an editable valve 220 map. unlike the goldsrc
    // decompiler this reads the original brushes and brushsides directly, so no
    // tree reconstruction is needed. displacement surfaces are tessellated into
    // thin func_detail triangle brushes. texture names are mapped to goldsrc;
    // the companion wad is produced separately by the material pipeline.
    bool port_source_map(const format::source_map_data &map, const source_options &options,
                         source_result &result, std::string *error = nullptr);
}
