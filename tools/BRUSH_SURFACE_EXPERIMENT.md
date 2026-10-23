# Closed brush / top surface collision experiment

The question is whether closed terrain brushes can produce the same exterior
player contact as their top triangles. For a solid set S and player box B, the
blocked player origins are S + (-B). Minkowski addition distributes over union.
For matching terrain tops, the upper boundary can therefore agree even though
the complete solid sets differ below or along the sides.

This diagnostic uses the real CSG MAP parser and hull-0 windings, instead of
reconstructing planes with a second MAP parser. Each selected solid brush must
have one upward face. Its top polygon is triangulated without snapping points.
Both the closed vertices and top triangles pass through the supplied
exacthull.py canonicalization, then CSG/BSP with hull expansion disabled.
Drawnodes become clipnodes. Misclassified bounded leaf regions are rebuilt
using brushbsp.py fragments, including false EMPTY as well as false SOLID.

The geometry oracle uses the original, unsnapped convex sums. Repair probes and
holdout probes use different seeds. Sampling does not prove whole-space
equivalence. In particular, canonical plane merging can move a boundary by
hundredths of a unit, and a grazing ray magnifies that into a larger distance
along its direction. Counts and residual errors are retained in results.json.

Build the opt-in helper:

```powershell
cmake --build build/vs --config Release --target brush_geometry_dump -j 2
```

Run in a fresh output directory, selecting the original func_detail entity
number. The runner needs NumPy, SciPy and the seven original reference scripts.

```powershell
python tools/brush_surface_pipeline.py `
  --source D:/maps/terrain.map `
  --entity 3 `
  --compiler bin/hltools.exe `
  --dumper bin/tools/brush_geometry_dump.exe `
  --scripts C:/Users/Xiang/Downloads `
  --wad D:/assets/sdhlt.wad `
  --output build/tmp/brush_surface_experiment
python tools/brush_surface_report.py build/tmp/brush_surface_experiment
```

Use `--mode closed` to build only the closed-brush hulls, without the additional
top-triangle comparison. This mode accepts convex brushes with multiple upward
faces and expands their complete vertex sets. The collision splice checks the
actual player starts from the BSP entity lump for standing and crouched hulls.

The helper's optional background mode preserves all visible brushes and makes
only the selected func_detail noncolliding. In this experimental copy, light
entities with a legacy `light` key and no `_light` key receive the equivalent
`_light` value so RAD has an actual light source. The source MAP is unchanged.
The runner compiles BSP, VIS and RAD, then overlays terrain SOLID onto the
background's EMPTY regions. Background SOLID and exterior fill remain present.
Hull 1, 2 and 3 use their own player sizes and independent trees.

Outputs include input hashes and snapshots, compiler logs, raw expanded MAPs,
repaired terrain graphs, independent checks, and two experimental BSPs sharing
the same visible geometry and lighting. They are not installed into the game.
The runner checks the node budget, indices, cycles, root lower bounds, retained
noncollision lumps, union point contents and the original player start.

This experiment currently supports a single merged world model, solid terrain
and an EMPTY/SOLID background. It is not a new production cliptype. The purpose
is to establish the geometric target before changing compiler behavior.

## Measured example (2026-10-23)

The 288-brush clip_null_legacy.map snapshot has SHA256
`0c3a99c3a8dbfe0021243c3bc05bc7b86f2d37e421f9e616a6edbd7dbfa112c5`.
Each hull was checked with 66,049 vertical and 8,192 oblique rays from above.
Closed-volume and top-triangle geometry agreed within 1e-9 units. Their compiled
BSPs differed by at most 0.034005 units vertically and 0.144806 units along these
oblique rays. These are measurements, not a whole-space equivalence proof.

| Test BSP | Hull 1 nodes | Hull 2 nodes | Hull 3 nodes | Total |
| --- | ---: | ---: | ---: | ---: |
| Closed brushes | 2049 | 1663 | 2001 | 5713 |
| Top triangles | 2021 | 1693 | 2041 | 5755 |

These counts include room collision. Both BSPs retain 1,323 visible faces,
53,754 lighting bytes and 87 visibility bytes. No node reduction over simple
was established. Independent full-volume holdout (250,000 points and 8,192 rays
per variant) retained one extra-solid point in closed hull 3 at the 0.01-unit
threshold, and maximum ray-distance error 0.627370 units in surface hull 2.
Residual errors are recorded rather than treated as a perfect-accuracy pass.
Generated logs, snapshots, plots and full results are under
`build/tmp/brush_surface_pipeline_final_20261023` for this workspace.

Focused geometry and splice checks:

```powershell
python tests/tools/test_brush_surface_pipeline.py
```
