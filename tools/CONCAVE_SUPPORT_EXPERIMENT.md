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

## Larger Entity Build

`legacy_support_pipeline.py` applies the same global support rule to a selected
solid `func_detail`, with map-relative probes and the existing legacy plane
union repair backend for BSP classification errors. It overrides local
cliptype settings in a compilation copy and proves each source brush remains
inside its clipped collision group. It does not add local support propagation.

```powershell
python tools/legacy_support_pipeline.py `
  --source D:/steam/steamapps/common/Half-Life/cstrike/maps/untitledx.map `
  --entity 3 --compiler bin/hltools.exe `
  --dumper bin/tools/brush_geometry_dump.exe `
  --scripts C:/Users/Xiang/Downloads `
  --output build/tmp/untitledx_surface_support_20261023_run1
```

For source SHA256
`b9741eaeb656c312d13a7b46254276c11520c9d9927e54ff916f4453a838fe99`,
360 terrain brushes have 13 shared supporting source planes. Only 29 brushes
are clipped in each player hull. The original native legacy build stores
11,934 clipnodes; the validated support build stores 13,680:

| Hull | Reachable final nodes | Depth | Max sampled top trim |
| --- | ---: | ---: | ---: |
| Standing | 4,796 | 79 | 6.148775 |
| Large | 4,115 | 73 | 18.625685 |
| Crouched | 4,769 | 79 | 7.466508 |

Raw collision compilation had leaf-classification errors. Direct half-space
repair corrected the sampled errors. Per hull, independent terrain validation
uses 250,000 points, 8,192 rays, exterior probes and a 66,049-ray top grid, with
no remaining misses or errors above 0.01 units. Final collision is unioned
with the room, with further sampled checks of that union and player starts.
The final BSP retains the native legacy build's noncollision lumps byte for
byte, including 46,713 light bytes and 142 VIS bytes.

The effect is confined to the global envelope. Interior terrain seams whose
roof planes are not shared supports are not repaired by this rule. This is a
test artifact, not evidence that all 360-brush seams are smooth in-game.

Run entity clipping checks with:

```powershell
python -m unittest discover -s tests/tools -p test_legacy_support_pipeline.py
```

## Complex Terrain Failure: untitledx

The `untitledx_surface_support` build does **not** solve the complex terrain
seams. Inspection of the final BSP confirmed actual top-height steps, beyond
the lines introduced by BSP face subdivision. At the following XY locations,
probe both sides along X by 0.0001 units:

| X | Y | Source top difference | Hull 1 BSP top difference |
| ---: | ---: | ---: | ---: |
| 321.880950 | 250 | 0.000061 | 38.398917 |
| -370.416862 | 750 | 0.000248 | 16.287502 |
| -329.938729 | -250 | 0.000133 | 19.246266 |

These BSP contacts agree with both the clipped target and the original legacy
brush union to within 0.00003 units. Consequently the target itself retains
the steps; correcting leaf labels or reducing node counts cannot remove them.
The earlier target-conformity and structure checks remain useful but are not
surface-continuity checks. This build should not be treated as a successful
general terrain collision mode.

The first two locations involve brushes 346/53 and 353/277 respectively,
whose source roof faces share edges. Some source brushes have multiple upward
faces and slightly tilted side faces. Treating every positive-Z normal as a
terrain roof, or propagating one neighbor's plane over an entire brush, does
not establish a valid local surface rule. An exploratory neighbor-plane test
removed some steps but also introduced new candidate steps elsewhere. It was
not compiled or installed as a replacement.

Reproduce the failure against the saved experiment (the command deliberately
returns exit code 1 because `--fail-on-step` is enabled):

```powershell
python tools/surface_seam_audit.py `
  --geometry build/tmp/untitledx_surface_support_20261023_run1/geometry.json `
  --bsp build/tmp/untitledx_surface_support_20261023_run1/untitledx_surface_support.bsp `
  --target build/tmp/untitledx_surface_support_20261023_run1/target_h1.json `
  --entity 3 --hull 1 `
  --probe 321.88095 250 --probe -370.416862 750 --probe -329.938729 -250 `
  --scan-y -850 -688 -500 -250 0 250 500 750 --fail-on-step `
  --output build/tmp/untitledx_surface_seam_audit_20261023
```

Use a fresh output directory for another run. The audit records source,
original expansion, optional target and final BSP top contacts at X offsets
of 0.01, 0.001 and 0.0001 units. It rejects ray misses and steep continuous
profiles whose differences shrink with the probe spacing. JSON witnesses,
raw sample pairs and a contact-profile image are saved without changing BSPs.
The three explicit probes reproduce persistent steps at all three spacings.

Scanning eight rows of reachable vertical BSP planes produced 4,756 candidate
locations and flagged 293. These are candidate locations, **not** a count of
distinct artificial seams: multiple planes can describe nearly the same
boundary, nearby real cliffs need review, and nonvertical boundaries or
different rows may be missed. The diagnostic does not guarantee that the
vertical ray origin is clear of unrelated scene geometry; inspect the saved
Z range when auditing another map.

Future geometry rules must be judged against original source containment,
local contact continuity, intended legacy slope offsets, BSP target conformity
and engine limits independently. A correct BSP representation of a fragmented
target is insufficient. Start with the shared-edge witnesses above before
attempting another whole-map candidate.

```powershell
python -m unittest discover -s tests/tools -p test_surface_seam_audit.py
```
