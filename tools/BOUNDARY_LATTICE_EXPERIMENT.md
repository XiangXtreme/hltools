# Exposed 3D Boundary Composition Experiment

`boundary_lattice_diagnose.py` extends the roof-only experiment to include
actual source walls and bottom faces. It remains an experimental Python
pipeline. Existing source maps and game BSPs are not modified.

## Rule and Checks

Extract the source union boundary first. Every term may use only an outward
half-space belonging to a real exposed source face. The represented solid is
a union of intersections of these half-spaces. Internal source-brush cutting
planes are not added as candidate boundaries.

The bounded arrangement method partitions a crop by its boundary planes,
classifies the resulting cells against the closed source brushes, and checks
the classifications with convex intersection/subtraction volumes. It removes
constraints only when doing so does not include an empty arrangement cell.
Every solid cell must remain covered. The unshifted terms must also pass
independent source-contact checks.

After verification, each unit plane moves outward by `sum(h_i*n_i^2)`, using
the standard legacy hull half extents. The same union/intersection formula
is retained. This monotone change preserves the represented original solid
within the experiment domain. It is a different composition from per-brush
legacy, with the same individual plane distances, not a player-box expansion.

Both source conformity and shifted BSP conformity are required. Neither alone
establishes that every collision seam is desirable. Real source cliffs must
be distinguished from internal-brush artifacts.

## Three Witness Regions

Source MAP SHA256:
`b9741eaeb656c312d13a7b46254276c11520c9d9927e54ff916f4453a838fe99`.
The 360-brush terrain has 454 distinct exposed planes: 361 upward, 92 vertical
and one downward plane. The roof-only extraction contains 398 upward triangles;
the full boundary includes vertical source cliffs omitted by that experiment.

```powershell
python tools/boundary_lattice_diagnose.py `
  --build build/tmp/untitledx_surface_support_20261023_run1 --entity 3 `
  --probe 321.88095 250 --probe -370.416862 750 --probe -329.938729 -250 `
  --output build/tmp/untitledx_boundary_lattice_20261023_verified64
```

Each crop extends 64 units in X and Y from its witness. Use a fresh output
directory. Three standing-hull results, sampled at X +/-0.0001:

| Witness XY | Previous collision difference | 3D boundary result | Boundary planes | Terms | Local nodes |
| --- | ---: | ---: | ---: | ---: | ---: |
| 321.88095, 250 | 38.398917 | 0.000061 | 5 | 3 | 9 |
| -370.416862, 750 | 16.287502 | 0.000248 | 10 | 4 | 15 |
| -329.938729, -250 | 19.246266 | 19.246266 | 8 | 4 | 10 |

The first two profiles are continuous, retaining legacy roof-plane offsets.
All three hulls compile and pass their geometry checks in all three crops.
The local node counts do not include the rest of the map and are not a
whole-map optimization result. The diagnostic BSPs are hull-0 compilations;
validated collision trees are also saved as NPZ. They are not playable maps.

The third step is intentionally retained. Its real outward vertical plane is
approximately `(0.9988982695, 0.0469281066, 0), d=-357.307251803`.
At Y=-250 its source position is X=-345.956375839. The hull-1 legacy offset is
16 units perpendicular to that plane, which shifts its X intersection by
16.017647130 to X=-329.938728709, matching the collision witness. The original
wall's source top difference is about 6.33 units. Intersecting its expanded
position with the shifted neighboring roofs gives the larger 19.25-unit step.
This establishes a real wall's provenance rather than an internal seam.

## Overlap Check

The same three centers were rebuilt with radius 128, saved in
`build/tmp/untitledx_boundary_lattice_20261023_r128`.

```powershell
python tools/boundary_lattice_compare.py `
  --small build/tmp/untitledx_boundary_lattice_20261023_run2 `
  --large build/tmp/untitledx_boundary_lattice_20261023_r128 `
  --output build/tmp/untitledx_boundary_lattice_overlap_20261023
```

The central 64x64 portions agree for all three hulls in all three crops:
129x129 contact grids and 50,000 independent points per hull have no sampled
differences. This is not proof that arbitrary neighboring regions agree.
Across the entire smaller crop, the third region differs near its edges:
hull 2 reaches about 1.643331 units of top difference and hull 3 about
0.528960 units. Therefore direct concatenation of cropped outputs is not a
validated whole-map construction. The comparison saves wall provenance,
overlap measurements and `profiles.png`.

## Adaptive Whole-Entity Construction

Enumerating the full arrangement of 454 planes is unsuitable here. The
adaptive method instead starts with the closed source brushes. For each
source piece it intersects all exposed outward half-spaces containing that
piece. If the resulting closure extends beyond the original source union,
an outside witness selects another boundary plane to split the source piece.
Accepted closures must remain within the source. Final subtraction verifies
that their union covers every original brush; both total excess and total
missing-volume limits are 0.001 cubic units.

```powershell
python tools/boundary_lattice_diagnose.py `
  --build build/tmp/untitledx_surface_support_20261023_run1 --entity 3 `
  --method adaptive --whole-entity --geometry-only --probe -329.938729 -250 `
  --output build/tmp/untitledx_boundary_whole_fresh
```

This is an opt-in construction experiment. It may reject a source that cannot
be separated with the available outward half-spaces, exceed its piece budget,
or fail numerical checks. A whole-entity domain is padded around the source;
it must not be interpreted as a valid finite collision extent if shifted
terms reach its artificial bounds. No whole-map release follows merely from
successful source composition.

Progress checkpoints preserve pending source pieces and accepted terms, with
an input signature. A fresh output folder can resume a matching checkpoint
using `--resume-composition path/to/composition_checkpoint.json`.

Narrow source slivers need special care. Qhull is used with affine
preconditioning, retaining the original coordinates. Small failed hulls use
exact rational orientation predicates to recover their supporting faces;
points are not joggled and wide-facet results are not silently accepted.
Volume checks and independent half-space holdouts remain required. A real
clustered-vertex failure and a thin rotated solid are regression fixtures.

Supporting-plane reduction is checked separately. Near-face vertices can
give a valid face numerical rank three; requiring rank exactly two deleted
essential constraints in the first whole-entity run. Supporting faces now
allow that extra rank, and the reconstructed reduced convex body must satisfy
every original half-space within 1e-5 units. The regression tests cover both
the near-face case and rejection of a missing constraint.

## Whole-Entity Diagnostic Result

The completed geometry is saved in
`build/tmp/untitledx_boundary_lattice_20261023_whole8`. This is a source-equivalent
construction, but it **does not pass whole-terrain smoothness inspection**.

There are 363 terms after absorption, from 3,872 processed source pieces and
1,800 adaptive splits. Missing source volume is zero; the summed numerical
excess is 0.000246573 cubic units, within the unchanged 0.001 limit. All 250,000
unshifted point classifications agree. The 129x129 source-contact grid has no
misses and a maximum distance error of 1.75e-11 units.

```powershell
python tools/boundary_lattice_build.py `
  --geometry build/tmp/untitledx_boundary_lattice_20261023_whole8 `
  --reference build/tmp/untitledx_surface_support_20261023_run1 `
  --continuous-probe 321.88095 250 --continuous-probe -370.416862 750 `
  --probe -329.938729 -250 `
  --output build/tmp/untitledx_boundary_lattice_compiled_fresh
```

The diagnostic BSP produced in
`build/tmp/untitledx_boundary_lattice_compiled_20261023/untitledx_boundary_lattice.bsp`
has the following complete-map trees, including background collision:

| Hull | Reachable clipnodes | Maximum depth |
| --- | ---: | ---: |
| 1 | 2,015 | 218 |
| 2 | 1,574 | 134 |
| 3 | 1,941 | 221 |
| Total | 5,530 | |

The original legacy BSP has 11,934 stored clipnodes; the previous
`untitledx_surface_support` has 13,680. Node reduction alone is not acceptance:
the diagnostic trees are also deeper than the original legacy trees (46-51).
Engine movement and runtime stack behavior have not been tested.

Compiling, repairing and validating all three prepared targets took 190.3
seconds, excluding adaptive source construction. CSG/BSP alone took about
3.16 seconds combined, and repair about 68.5 seconds. Adaptive geometry
construction took several minutes and involved resumed numerical debugging;
there is no measured fresh end-to-end production time.

Per hull, 250,000 points, 8,192 oblique rays, about 96,000 exterior points,
and a 257x257 top grid pass the compiled-target checks. Node indices, roots,
cycles, the 32,767-node budget, player starts and background union are checked.
Rendering, entities, textures, 46,713 light bytes and 142 VIS bytes match the
lit reference. The source MAP is unchanged. No game installation was performed.

## Wider Seam Audit Rejects Smoothness

```powershell
python tools/boundary_lattice_audit.py `
  --geometry build/tmp/untitledx_boundary_lattice_20261023_whole8 `
  --tree build/tmp/untitledx_boundary_lattice_compiled_20261023/collision_h1.npz `
  --hull 1 --rows -1000 -900 -800 -700 -600 -500 -400 -300 -200 -100 0 `
                 100 200 300 400 500 600 700 800 900 1000 `
  --output build/tmp/untitledx_boundary_lattice_audit_fresh
```

Both axis directions are sampled at boundary-plane crossings with shrinking
spacing (0.01, 0.001, 0.0001). The 1,933 locations contain 85 persistent steps
in the mathematical hull-1 target. At 67 of these locations the corresponding
exposed source wall is less than 0.001 units high. The compiled-tree scan
detects 83 steps, with 7 substantial source-wall/outline correspondences,
65 thin-wall correspondences, and further unconfirmed wall extensions.
These categories are inspection evidence, not an exhaustive artifact count.

For example, at XY `(873.446598470, 500)` the target and actual BSP retain a
17.357-unit step. The corresponding source wall lies at X=856.664556962.
It is a single exposed sliver triangle with maximum endpoint height difference
about 0.000097559; at Y=500 its vertical extent is about 0.000015283 units.
The legacy shifts of the neighboring roof planes differ substantially, so
preserving this tiny wall can yield a large collision discontinuity.

The same wall plane produces an 8.946-unit step at XY `(841.801028849, 400)`,
although its exposed triangle spans Y=450.5 to 766.5. Thus extending a finite
boundary face to an infinite affine plane can affect remote locations after
offsetting even when the unshifted Boolean formula represents the source.

The audit also finds two near-boundary samples exceeding 0.01 units of vertical
target/BSP error. At X=900, the relevant float32 wall boundary moves from
Y=583.908748836 to 583.908637875 (0.000110961 units), larger than the smallest
probe spacing. A discontinuous top height amplifies this small horizontal
plane quantization into a large ray-height difference. Broad random/grid
checks do not establish exact boundary agreement.

The two original artificial witnesses remain continuous: hull-1 differences
are approximately 0.000061 and 0.000248 units. The real-cliff witness remains
a step, but the whole-entity formula changes its height to about 5.964 units
from the local crop's 19.246. This is further evidence that source-equivalent
formulas can differ after offsetting. Four actual BSP profiles are saved in
`build/tmp/untitledx_boundary_lattice_profiles_20261023/seam_profiles.png`.

The next construction must address shared surface topology and very small
boundary slivers, while bounding the influence of genuine walls. Simply
deleting every vertical plane would destroy real cliffs. Increasing the
source-volume tolerance or accepting target conformity would not resolve
these failures.

## Remaining Limits

A source-equivalent boundary formula is not necessarily unique after plane
offsetting. The experiment does not prove arbitrary decomposition invariance,
global smoothness, engine movement behavior or all special brush policies.
The measured whole-entity result reduces nodes but retains substantial steps;
it remains a diagnostic artifact and is not recommended as a smooth release.

```powershell
python -m unittest discover -s tests/tools -p test_boundary_lattice_diagnose.py
```
