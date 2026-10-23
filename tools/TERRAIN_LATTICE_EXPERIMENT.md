# Cropped Terrain Roof Lattice Experiment

`terrain_lattice_diagnose.py` tests a continuous roof-plane offset on bounded
patches of the actual exposed terrain. It is a Python geometry experiment,
not a production cliptype or an installed whole-map collision replacement.

## Construction

Extract the union boundary before selecting its upward faces. A rectangular
crop must have complete, nonoverlapping roof projections. For an upward plane
`n.x <= d`, its roof height is the affine function
`l(x,y) = (d - nx*x - ny*y) / nz`.

Build a lattice expression `f = max_k min_(j in J_k) l_j` from these functions.
Split source patches where the ordering of their affine functions changes.
The resulting cells supply the candidate terms; remove terms dominated by
other terms. Verify coverage on every cell and use linear programming on
every term/source-patch pair to bound excess over the original roof.
Discontinuous roofs cannot pass this verification.

Then shift each roof plane by the legacy distance `sum(h_i*n_i^2)` and retain
the lattice expression. Each minimum is an intersection of shifted roof
half-spaces; the maximum is their union. Artificial crop sides and a bottom
make those terms bounded solids for CSG/BSP compilation with `-noclip`.

Finite minima and maxima of affine functions are continuous. Since every
roof height increases when its plane shifts outward, monotonicity also
preserves the supplied source roof and solid beneath it within the crop.
The checks use numerical tolerances; sampled comparisons to original closed
brushes independently verify that the extracted graph describes this crop.

This retains the legacy offset on each plane, but changes their composition.
It is **not** the original per-brush legacy rule. Nor is it player-box/simple
expansion. Convex ridges use minima and concave valleys use maxima; mixed
regions need both. Individual planes can cease to be active after shifting.

## Reproduction

The input is the saved entity-support build for source MAP SHA256
`b9741eaeb656c312d13a7b46254276c11520c9d9927e54ff916f4453a838fe99`.
Its exported source geometry is reused; no further source-map editing or
native cliptype changes are needed.

```powershell
python tools/terrain_lattice_diagnose.py `
  --build build/tmp/untitledx_surface_support_20261023_run1 `
  --entity 3 --radius 64 `
  --probe 321.88095 250 --probe -370.416862 750 --probe -329.938729 -250 `
  --output build/tmp/untitledx_roof_lattice_20261023_run3
```

Use a fresh output directory. The third patch is deliberately rejected and
the command returns a nonzero status. The completed patches, rejection
reason, original source-cliff samples and profile image remain in the output.
Geometry, reference scripts and compiler are snapshotted and hashed.

## Measurements

All crops are 128 by 128 map units. Standing-hull contact differences at
X offsets of +/-0.0001 around the original witnesses:

| Witness XY | Old support BSP difference | Roof-lattice BSP difference | Roof planes | Terms | Local draw nodes |
| --- | ---: | ---: | ---: | ---: | ---: |
| 321.88095, 250 | 38.398917 | 0.000061 | 4 | 3 | 9 |
| -370.416862, 750 | 16.287502 | 0.000248 | 9 | 5 | 15 |

The new differences shrink with the X sampling interval, as expected on a
continuous slope. Hulls 1, 2 and 3 all pass the two accepted crops. These node
counts include the crop sides and bottom but exclude the rest of the map;
they cannot be compared with whole-map clipnode counts.

For the accepted crops, cell coverage and linear-program bounds reproduce
the unshifted source roof within 0.000002 units. Independent 129x129 top grids
match original closed-source contacts within 0.00000000002 units. After
shifting, the maximum BSP top-grid error is below 0.000027 units. Each compiled
tree also passes the existing independent 250,000-point, 8,192-ray and exterior
tests at their 0.01-unit threshold; maximum oblique-ray error is below 0.00065.

Standing-hull plane shifts at the active witness roofs:

| Source brush | Legacy vertical offset | Simple vertical offset |
| ---: | ---: | ---: |
| 53 | 35.781130 | 46.417445 |
| 353 | 38.260380 | 60.494845 |

The lattice uses the legacy column. Preserving those distances while joining
the shifted planes removes the two measured brush-partition steps.

## Rejected Source Cliff

The third crop, around (-329.938729,-250), is not a continuous roof graph.
On its actual source boundary at approximately (-345.956376,-250), vertical
source rays at X +/-0.0001 contact Z 67.795171 and 61.465025: a real source step
of 6.330146 units. Other source-edge midpoint probes find similar steps.
The roof-only lattice overestimates portions of this crop, and is rejected
before collision compilation.

This also qualifies the previous seam audit: continuity immediately below a
collision step does not establish that the step is artificial. Expansion can
move a nearby real source cliff sideways. The previous three witnesses prove
actual collision steps and target agreement, but do not by themselves prove
that all three should be removed. The first two crops now have the additional
continuous-source verification; the third requires its vertical source faces.

## Limits and Next Work

The generated `patch*_h*.bsp` files are diagnostic hull-0 compilations, with
no complete room, lighting, VIS or playable player hulls. Validated trees are
also saved as NPZ, alongside convex target volumes. They are not installed
into the game and are not playable map releases.

The crop's walls and bottom are artificial. Adjacent crops may produce
different extensions after plane shifting; concatenating them would not
establish a continuous whole-map collision. The experiment does not prove
arbitrary brush-decomposition invariance or resolve a nonconvex outer
footprint, caves, overhangs, source cliffs or special brush semantics.

The next generalization needs a composition of the **full exposed 3D boundary**,
including real vertical walls and bottoms, and a consistent rule across
neighboring regions. Do not apply this roof-only construction to the entire
map or replace rejected cliffs by interpolated smooth surfaces.

```powershell
python -m unittest discover -s tests/tools -p test_terrain_lattice_diagnose.py
```

Focused tests cover convex ridges, concave valleys, mixed flat/valley surfaces,
coplanar subdivision, source-cliff rejection, overlapping roof projections
and independent measurement of real source steps.
