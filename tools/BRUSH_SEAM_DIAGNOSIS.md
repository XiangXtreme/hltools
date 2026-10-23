# Brush Decomposition and Legacy Collision

The updated `cliptype_sl.map` contains six room brushes, a ramp (brush 6),
an adjoining platform (brush 7), and a merged copy (brush 8) on the other side
of the room. Brush 8 translated by (-192, 0, 0) is exactly the same solid as
the union of brushes 6 and 7. The shared plane is Y=-48. Both have volume
573,440 cubic units. Their convex hull and containment checks establish the
equivalence independently of collision expansion.

This is a different input from the earlier seven-brush single-slope test in
`SLOPE_CONTACT_DIAGNOSIS.md`. That test measures slope thickness; it does not
test the effect of joining brushes.

## Reproduction

Run from the repository root, using Python with NumPy, SciPy and Matplotlib:

```powershell
python tools/brush_seam_diagnose.py `
  --source 'D:\steam\steamapps\common\Half-Life\cstrike\maps\cliptype_sl.map' `
  --compiler bin/hltools.exe `
  --dumper bin/tools/brush_geometry_dump.exe `
  --output build/tmp/cliptype_sl_seam_final_20261023
```

Use a fresh output directory. The script snapshots the source and binaries,
exports actual CSG legacy hulls, and compiles the original arrangement and
aligned split/merged arrangements in legacy and simple modes. All six BSPs
include VIS and RAD. The original MAP and game files are unchanged.

## Measurements

At X=-96, downward traces from Z=160 measure the top boundary of each hull.
Standing hull 1 at Y=-32 gives:

| Mode | Two brushes, top Z | One merged brush, top Z | Difference |
| --- | ---: | ---: | ---: |
| Legacy | 116.000000 | 98.934616 | 17.065384 |
| Simple | 116.000000 | 116.000000 | 0.000000 |

The platform ends at Y=-48, Z=80. Its expanded standing collision reaches
Y=-32, Z=116. In the merged legacy brush, the sloped plane already limits
the top to Z=98.934616 there. Expanding the separate platform therefore adds
a ledge above the merged brush's collision. Beyond that ledge, both legacy
variants return to the same thin slope boundary.

Maximum excess height is 17.065384 for hull 1, 20.162428 for hull 2, and
10.779627 for hull 3. Split/merged simple profiles agree in all three hulls.
These are boundary measurements without the engine's trace epsilon.

The script checks each of six BSPs in all three hulls against the expanded
brush union: 50,000 random points, 4,001 vertical profile rays, and 4,096
oblique rays per combination. Actual legacy CSG planes supply its oracle;
simple uses independent convex brush-plus-player-box geometry. No point
classification differences outside a 0.0001 boundary band, ray hit/miss
differences, or ray errors above 0.001 occurred. Original-layout right-hand
terrain also matches the aligned merged profiles. Hull-0 BSP classification
agrees at the same 50,000 points.

Total stored clipnodes were 39/32 for split/merged legacy and 32/32 for
split/merged simple. Original-layout legacy/simple had 42/37. Shared nodes
mean total storage is not the sum of per-hull reachable counts.

## Implication

For this case, the protrusion is present before BSP construction. Precisely
rebuilding the union of the existing legacy hulls would preserve it.

Simple corresponds to expansion by a fixed player box, for which Minkowski
addition distributes over union: `(A union B) + H = (A + H) union (B + H)`.
The legacy plane shift and bounding planes used here do not have that
decomposition invariance. A flatter result cannot be obtained merely by
changing clipnode partitioning while preserving the same expanded solid.

This does not establish that thin slopes and smooth joins are incompatible.
In this example, merging the two source solids before legacy expansion gives
the desired continuous, thin boundary with fewer clipnodes. General terrain
requires rules for joining its exposed surfaces or compatible convex parts;
replacing an arbitrary nonconvex assembly with its convex hull would change
the map's geometry. This investigation does not change production cliptypes
or claim an in-game movement test.
