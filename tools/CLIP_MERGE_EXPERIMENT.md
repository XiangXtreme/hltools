# Convex Legacy Collision Merging

`-clipmerge` enables an opt-in CSG pass for ordinary legacy brushes. It keeps
each original render brush and replaces only hulls 1 through 3. Use it with
`-cliptype legacy`, or with a `func_detail` whose `zhlt_cliptype` is `legacy`:

```powershell
bin/hltools.exe compile test.map -cliptype legacy -clipmerge -threads 2
```

The default remains off. `info_compile_parameters` can persist the option as
`"csg_clipmerge" "1"`. Local `zhlt_cliptype` overrides still take precedence
over the global cliptype. The pass does not merge simple, precise, normalized,
or the continuous-surface `legacytrim` collision.

## Geometry Contract

Two eligible convex solids must have an opposing plane pair. After removing
that cut from both, every remaining plane of each solid must contain all
vertices of the other within 0.0000001 units. Their combined intersection
then equals their union: which side of the removed plane contains a point
determines which original solid contains it. This rejects nonconvex joins,
gaps, and edge-only contacts without taking a convex hull across empty space.

Plane adjacency indexes restrict candidate pairs. Successful groups are
revisited, allowing chains of more than two brushes. The greedy partition
does not guarantee a minimum number of groups or the same partition for
every brush ordering.

Merging requires matching original entity ownership, detail/chop settings,
clipnode detail level and coplanar priority. Noclip brushes, clip-only
brushes, bevels, generated studio brushes, and custom collision shapes are
excluded. Groups never span BSP models. All original render brushes and
source map records remain intact. Expansion uses the same legacy plane
offset and configured player hull sizes as before.

## Controlled Seam Result

Build and validate using the baseline from `BRUSH_SEAM_DIAGNOSIS.md`:

```powershell
cmake --build build/vs --config Release --target hltools unit_csg unit_cli
python tools/clip_merge_diagnose.py `
  --baseline build/tmp/cliptype_sl_seam_final_20261023 `
  --compiler bin/hltools.exe `
  --output build/tmp/cliptype_sl_clipmerge_verified_20261023
```

Use a fresh output directory. The script creates five BSPs with VIS and RAD,
keeps the original game files untouched, and compares against the independently
authored merged brush from the user's MAP. It checks three merged builds in
all three hulls: 900,000 random point queries and 109,737 rays in total.
No point differences outside the boundary tolerance, ray hit/miss differences,
or ray errors above 0.001 occurred. Player starts are clear. BSP graph checks
cover indices, valid contents, cycles, hull lower bounds and clipnode limits.

| Input | Original legacy clipnodes | With clipmerge |
| --- | ---: | ---: |
| Original left/right comparison layout | 42 | 37 |
| Isolated two-brush terrain | 39 | 32 |
| Isolated merged terrain | 32 | 32 |

Standing hull 1 at X=-96, Y=-32 changes from Z=116 to Z=98.934616, removing
the 17.065384-unit ledge. At Y=0, both original legacy and clipmerge remain
Z=61.601281; simple is Z=78.666668. Thus removing the ledge preserves the thin
legacy slope. Hulls 2 and 3 also match the independently merged legacy target.

All render/visibility/lighting lumps remain byte-identical to the matching
baseline. Render plane references and equations also match; BSP may reorder
or omit unused collision planes. Entity data differs only in compiler identity
and timestamp. Model data changes only at the three collision roots.

Eight focused CSG cases cover the seam, nonconvex/disjoint/edge rejection,
simple preservation, policy compatibility, model ownership, custom hulls,
oblique cuts in both orders, and chained merging. CLI tests cover persistence
and stage routing. All 29 CSG and 6 CLI tests passed.

The combined `compile` command was also checked against the staged output:
300,000 point queries across the three hulls agree, with the same 37 stored
clipnodes. Its separate result is in `combined_validation.json`.

For a legacy-only input copy, `brush_geometry_dump <input.map> <output.json>
--legacy-hulls --clipmerge` exports the merged expanded hulls. Absorbed brushes
retain their render faces but have empty collision plane arrays; skip those
empty arrays when constructing a collision-volume oracle.

## Larger Map Limit

`build/tmp/untitledx_clipmerge_regression_20261023` contains a separate lit
comparison of the user's `untitledx.map`. Its source has a local simple
override; only the experimental copies replace that override with legacy.
The source snapshot, compiler, command logs and `results.json` are preserved.

360 terrain brushes become 231 convex collision groups (129 joins).
Total clipnodes change from 11,934 to 8,565; hull 1 reachable nodes change
from 4,217 to 2,938. CSG+BSP+VIS+RAD took approximately 9.98 and 7.16 seconds
in this run; these single-run timings are not a benchmark. Render, VIS and
lighting lumps match byte for byte, and both collision graphs validate.

However, final BSP classification still differs from the actual expanded
CSG volumes. Each row below uses the same 50,000 points and 4,096 rays, a
0.001 point boundary band and 0.001 ray error threshold:

| Hull | Legacy extra-solid points | Clipmerge extra-solid points | Legacy ray errors | Clipmerge ray errors |
| --- | ---: | ---: | ---: | ---: |
| 1 | 45 | 11 | 292 | 126 |
| 2 | 2 | 9 | 47 | 33 |
| 3 | 96 | 5 | 208 | 32 |

Neither version had missing-solid samples or ray hit/miss differences in
this set. Ray distance discrepancies can be large, so this is not merely
floating-point noise. The geometry-merging contract does not resolve these
BSP classification discrepancies, and improvement is not monotonic per hull.

This feature is therefore an experiment for convex joins, with a validated
small reproduction. It does not establish smooth collision across arbitrary
nonconvex terrain or complete collision correctness on the larger map.
No in-game movement test was performed.
