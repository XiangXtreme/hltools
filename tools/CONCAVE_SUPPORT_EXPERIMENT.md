# Nonconvex Legacy Support Experiment

`concave_support_diagnose.py` tests the 2026-10-23 nonconvex revision of
`cliptype_sl.map`. Select worldspawn brushes explicitly; room walls must not
participate in the terrain's support-plane search.

```powershell
python tools/concave_support_diagnose.py `
  --source 'D:\CS\Mapper\02_Sandbox_Tests\Unity_Test\Cliptype\cliptype_sl.map' `
  --compiler bin/hltools.exe --dumper bin/tools/brush_geometry_dump.exe `
  --exacthull 'C:\Users\Xiang\Downloads\exacthull.py' `
  --brushes 6 7 8 9 `
  --probe-xy 63.99 -55 --probe-xy 63.9999 -55 --probe-xy 64.0001 -55 `
  --probe-xy 80 0 --probe-xy 80 -64 `
  --output build/tmp/concave_support_verified_20261023
```

Use a fresh output folder. The source, compiler, geometry dumper, reference
script and runner are snapshotted and hashed. Source and installed game files
are not modified. The result includes VIS and lightmaps copied from the
matching full compile; all noncollision lumps remain identical.

## Geometry Rule

Let `L_i` be the original legacy-expanded brush volumes. From the selected
source faces retain only planes `n.x <= d` that contain every source vertex.
For a standard symmetric hull with half extents `h`, legacy moves a unit plane
by `sum(n[k]^2 * h[k])`, while simple uses `sum(abs(n[k]) * h[k])`.

The new target is:

```text
(union_i L_i) intersect (intersection_j shifted_support_halfspace_j)
```

The implementation intersects each convex brush with those shifted planes
and compiles the resulting pieces as already-expanded hull-0 solids. The
reference `exacthull.py` is used only for plane-to-MAP formatting, with
12-decimal point coordinates. It never expands these vertices by a player box.
The resulting draw tree is spliced into the player's clipnode hulls.

The source remains nonconvex. The method does not replace the solid by a
convex hull. The cuts cannot add collision volume, and the script separately
proves each original source brush is contained by a clipped convex group.

For this input the six supporting planes include the flat top and both roof
slopes. Convex merging alone leaves a separate platform expansion extending
beyond the neighboring roof plane. Applying that roof plane to both pieces
removes the resulting ledge, even though the footprint has a real concavity.

## Measured Result

The same source revision produces 62 clipnodes with legacy, 39 with
`legacy + clipmerge`, 48 with simple, and 48 with the support experiment.
The independent full compiles take roughly 0.25 seconds each; Python geometry
analysis, additional hull compiles and validation take several seconds.
These timings describe a small reproduction, not a general benchmark.

Standing hull 1:

| XY | Legacy | Legacy + clipmerge | Simple | Shared support |
| --- | ---: | ---: | ---: | ---: |
| 63.99, -55 (ledge) | 116.000000 | 116.000000 | 116.000000 | 96.766205 |
| 80, 0 (slope) | 80.267948 | 74.352202 | 93.599996 | 74.352205 |
| 80, -64 (concave gap, floor contact) | 4.000000 | 4.000000 | 59.999993 | 4.000000 |

The excess ledge is reduced by about 19.25 units. The tested roof contact
retains the thin legacy offset rather than the simple distance. A sampled
point within the source's convex hull remains empty in the final standing
hull, demonstrating that the real concavity is still present.

Per hull, validation checks the terrain against its half-space target with
250,000 points and 8,192 rays. Final checks use 150,000 interior points,
8,192 independent oblique rays, over 176,000 vertical rays including near-seam
coordinates, and roughly 100,000 points outside the terrain's expansion box.
The final BSP has no sampled classification mismatches or ray errors above
0.001 units. Structural checks and player starts pass. Sampling does not prove
correctness of every engine trace or in-game movement.

## Limits

This is an experiment for explicitly selected ordinary world solids with
standard GoldSrc hulls, including a nonconvex footprint under common roof
supports. It is not an automatic production cliptype. The source must have
one solid world model and no persistent clip/hull overrides; special brush
policies and arbitrary material semantics have not been validated.

A concave roof valley has faces whose extended planes cut into the other
side. Such planes fail global support containment and are not propagated.
General terrain will need an independently validated rule for local surface
neighborhoods and their influence limits. Global support clipping therefore
does not establish a general solution to every nonconvex seam, nor guarantee
invariance to every possible brush decomposition.

Focused geometry tests:

```powershell
python -m unittest discover -s tests/tools -p test_concave_support_diagnose.py
```

They reject non-supporting cuts, distinguish legacy from box support offsets,
and exercise an L-shaped source whose ledge can be trimmed while its notch
and all original source brush volumes remain intact.
