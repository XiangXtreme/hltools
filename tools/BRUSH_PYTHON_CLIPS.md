# Brush Surfaces Through the Reference Python Scripts

`brush_python_clips.py` extracts the exposed boundary of a solid brush entity
and supplies its triangles to the original Python collision builder. Its only
geometry adapter replaces the Source BSP displacement reader. The seven supplied
scripts are copied unchanged, with SHA256s recorded in `config.json`.

The geometry export and source MAP must have matching provenance. This
experiment reuses the audited hull-0 geometry export from
`untitledx_legacy_union_planes_20261023`; it does not extract legacy expanded
hulls. `extract_exterior` removes internal contact faces and overlaps, checks
both sides of the boundary, and preserves the exposed polygon coordinates.

`terrain` selects upward exposed faces. `shell` includes sides and bottoms as
well. Both use the original `build_boreas_clips.triangles` grid (1/16 unit) and
degenerate-triangle threshold (cross-product length > 1e-3). No simplification
or displacement-grid resampling is applied.

## Reproduce

Run from the repository root using a fresh output directory:

```powershell
python tools/brush_python_clips.py prepare `
  --source D:/steam/steamapps/common/Half-Life/cstrike/maps/untitledx.map `
  --geometry build/tmp/untitledx_legacy_union_planes_20261023/geometry.json `
  --manifest build/tmp/untitledx_legacy_union_planes_20261023/manifest.json `
  --scripts C:/Users/Xiang/Downloads `
  --compiler build/tmp/untitledx_legacy_union_planes_20261023/compiler/hltools.exe `
  --wad D:/steam/steamapps/common/JACK/mapper/wad/sdhlt.wad `
  --entity 3 --surface terrain `
  --output build/tmp/untitledx_python_terrain_20261023

python tools/source_clip_pipeline.py build `
  --output build/tmp/untitledx_python_terrain_20261023 --workers 1
python tools/source_clip_validate.py repair `
  --output build/tmp/untitledx_python_terrain_20261023 --workers 1
python tools/source_clip_validate.py validate `
  --output build/tmp/untitledx_python_terrain_20261023 `
  --variants raw repaired --samples 50000 --rays 4096

python tools/brush_python_clips.py package `
  --output build/tmp/untitledx_python_terrain_20261023 `
  --background build/tmp/untitledx_closed_hull_final_20261023/background_lit.bsp `
  --compare build/tmp/untitledx_exterior_final_20261023/untitledx_surface_terrain.bsp `
  --simple build/tmp/untitledx_exterior_final_20261023/ordinary_simple.bsp `
  --name untitledx_python_terrain.bsp
```

For the shell experiment use `--surface shell`, a different output directory,
the previous `untitledx_surface_shell.bsp` comparison, and a distinct BSP name.

## Actual Calls

1. `source_clip_pipeline.run_build` calls the original `split`,
   `exacthull.base_template`, and `build_boreas_clips.compile_chunk`.
2. The original `exacthull.solid_tree` grows triangles by the standing/ducking
   box, canonicalizes planes, writes pre-expanded NULL brushes, and compiles
   hull 0 with `-noclip`. `exacthull.splice` converts these trees into player
   clip hulls. This does not use the compiler's legacy/simple brush expansion.
3. `source_clip_validate.repair` calls the original `fix_ambiguous.init` and
   `fix_file`, including the original SAT tests, region samples, and
   `brushbsp` repair. No custom brush-surface repair implementation is called.
4. The raw and repaired external collision BSPs remain in `clips_raw` and
   `clips_out`. Like the original scripts, hull 0 is empty and hull 2 aliases
   hull 1. These files are collision models, not independently playable maps.
5. `package` unions one repaired collision file with the previously compiled
   room collision. It preserves all render geometry, entities, textures,
   visibility and lighting lumps. Reachable nodes are compacted without
   changing collision occupancy. Hull 2 still uses the standing terrain tree,
   so it is not an independently calculated GoldSrc large hull.

Packaging validates child/plane indices, cycles, hull root lower bounds,
the 32767 clipnode budget, 150000 union occupancy probes per hull, and player
start clearance. It additionally compares a 257 x 257 downward-ray grid
against independent swept box/triangle SAT and the supplied comparison BSPs.
Numerical discrepancies are recorded, not silently repaired or called perfect.
Only one external collision file is supported by the packaging adapter.

The normal Source reader workflow remains available through
`source_clip_pipeline.py prepare`. None of these commands installs game files
or modifies the production compiler collision algorithm.

## untitledx Results (2026-10-23)

Source MAP SHA256:
`b9741eaeb656c312d13a7b46254276c11520c9d9927e54ff916f4453a838fe99`

All seven snapshot hashes match the supplied Downloads scripts. The compiler
matches the previous experiment. The prepared terrain array is identical in
values and order to the previous 376-triangle terrain input, so the
comparison changes the build/repair path rather than the triangle surface.

| Input | Triangles | Raw external nodes | Repaired external nodes | Packaged h1 / h2 / h3 | Packaged total | Build / repair wall seconds |
| --- | ---: | ---: | ---: | --- | ---: | --- |
| Upward terrain | 376 | 4380 | 4460 | 1970 / 1970 / 2040 | 5980 | 8.39 / 3.28 |
| Complete shell | 830 | 4652 | 4719 | 2137 / 2137 / 2110 | 6384 | 52.81 / 3.30 |

Build timing is the existing runner's timed chunk stage; preparation, base
template, packaging and validation are separate. Raw external totals store h1
and h3, sharing h1 with h2. Packaged totals store all three room-unioned hulls.
The packaged maps retain 46713 lighting bytes and 142 visibility bytes.

Both repaired variants pass graph checks and have zero occupancy disagreements
in 50000 surface-near probes per tested hull. On 4096 normal-directed rays per
hull, the shell's maximum errors are 0.000415 (h1) and 0.000202 (h3). The terrain
variant has 14 h3 errors above 0.1, reaching 0.333058. These checks alone do not
establish exactness: the denser downward h1 grid finds residual errors.

| Packaged h1 vs triangle SAT + room, 66049 downward rays | Errors > 0.01 | Maximum error (units) |
| --- | ---: | ---: |
| Original Python, upward terrain | 24 | 4.174434 |
| Previous terrain implementation, same input | 0 | 0.006892 |
| Original Python, complete shell | 54 | 6.836094 |
| Previous shell implementation, same input | 7 | 0.019627 |
| Ordinary simple, compared to upward terrain | 1290 | 58.206072 |

At XY `(-289.611729, 570.368600)`, ordinary simple contacts at Z `60.752191`,
the original Python terrain output at `2.546083`, and triangle SAT at `2.546119`.
Thus the actual ordinary simple BSP and this reference build do differ
substantially at some locations. Most terrain follows the same surface, so
similar appearance on other slopes does not identify the build algorithm.

The original repair checks sampled points in SOLID regions, not all points or
every erroneous EMPTY region. Its sparse checks can miss local defects. The
measurements above retain those defects and do not claim that rerunning the
original scripts magically improves the previous implementation.

Outputs are in `build/tmp/untitledx_python_terrain_20261023` and
`build/tmp/untitledx_python_shell_20261023`. The playable wrappers are
`untitledx_python_terrain.bsp` and `untitledx_python_shell.bsp`; original external
collision outputs are under each directory's `clips_out/bclip000.bsp`.
`validation.json`, `package.json`, the saved probe arrays and compiler logs
provide the numerical checks. No in-game runtime test was performed.
