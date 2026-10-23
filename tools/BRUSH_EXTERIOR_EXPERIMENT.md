# Brush Union Boundary Experiment

`brush_exterior_pipeline.py` tests the triangle route using the exterior of the
selected brush union. It reuses geometry snapshots and the lit background from
the legacy-union and closed-brush experiments, checking their source hashes.
This runner currently selects the solid terrain entity 3 in untitledx.map.

Each convex face is subtracted against other brushes. Subtraction retains
convex polygon pieces, removes opposite-facing internal contacts, and assigns
same-facing duplicate surfaces to one brush. Nonintersecting subtraction does
not subdivide a face. Triangle winding follows the original outward normal.
Four interior points per triangle check union occupancy on both sides, with
adaptive offsets for narrow slivers and a 0.00001-unit probe tolerance. This is
a numerical extraction, not a manifold or whole-space proof.

Triangles are snapped to the original build_boreas_clips.py grid of 1/16 units;
degenerate and duplicate triangles are discarded. The full shell and its
upward-facing subset are expanded with the actual player boxes. Both use the
supplied exacthull.py canonicalization, hull-0 CSG/BSP compilation, drawnode
conversion and brushbsp.py local fragment repair. Hull 2 is independent. Legacy
expansion planes are not used as the geometric repair target.

The complete shell preserves side and bottom surfaces but can leave deep
interiors empty. The upward subset models terrain and omits side and bottom
collisions. Neither is a general closed-solid replacement without defining the
intended collision semantics.

```powershell
python tools/brush_exterior_pipeline.py `
  --source D:/steam/steamapps/common/Half-Life/cstrike/maps/untitledx.map `
  --baseline build/tmp/untitledx_legacy_union_planes_20261023 `
  --closed build/tmp/untitledx_closed_hull_final_20261023 `
  --output build/tmp/untitledx_exterior_experiment
python tools/brush_exterior_report.py build/tmp/untitledx_exterior_experiment
python tests/tools/test_brush_exterior_pipeline.py
```

Use a fresh output directory. Inputs contain geometry exports, the original
seven scripts and compiler snapshot. NumPy, SciPy and Matplotlib are required.
The experiment uses the existing local sdhlt.wad. `--reuse` can import a previous
fragment checkpoint after matching source, geometry, compiler, script, snap-grid
and background hashes. Its path and hash are recorded and collision is checked
again. A discarded repair attempt is recorded separately from the output.

Independent tests use 250,000 points, 8,192 arbitrary rays and roughly 96,000
outside-bounds points per hull/variant. Triangle SAT checks the convex expansion
oracle on 2,048 more rays. A 257x257 vertical grid compares top contact against
legacy and closed-box targets. Seeds differ from repair. Error counts retain
the 0.01-unit threshold. For this comparison, the runner permits residual ray
distance errors up to 0.1 units, while rejecting sampled extra/missing solid,
hit/miss mismatches and exterior solid leakage. This limit does not certify
engine movement. Larger failures trigger a recorded local half-space repair;
unresolved failures stop BSP delivery.

The report also compiles ordinary simple from a derived MAP and compares all
BSPs against triangle contact on rays where terrain precedes the background.
Unsnapped geometry is compared separately so snap displacement is not mistaken
for a new collision rule. Timings distinguish generation, compiler and repair;
historical runs and reused checkpoints are identified rather than presented as
a controlled performance benchmark.

Final trees are bounded, compacted and spliced into the same lit background.
Plane and child indices, cycles, root bounds, node budgets, actual player starts
and sampled background union are checked. Noncollision BSP lumps are preserved
byte for byte. The original MAP and game BSP are not overwritten. This is an
isolated experiment, not a production cliptype.

## Measured untitledx Result (2026-10-23)

Source SHA256:
`b9741eaeb656c312d13a7b46254276c11520c9d9927e54ff916f4453a838fe99`.
360 brushes contain 1,828 faces. Removing internal/duplicate area (85.627%)
leaves 858 convex boundary fragments and 942 triangles, including 398 upward
triangles. After 1/16 snapping, 830 triangles remain, including 376 upward.
Maximum vertex displacement is 0.039987 units. Extraction takes 3.624 seconds.

| Variant | Hull 1 | Hull 2 | Hull 3 | Total clipnodes |
| --- | ---: | ---: | ---: | ---: |
| Ordinary legacy | 4217 | 3704 | 4013 | 11934 |
| Repaired legacy union | 6461 | 5628 | 6226 | 18315 |
| Exterior triangles | 2199 | 2019 | 2135 | 6353 |
| Upward exterior triangles | 1988 | 1959 | 2080 | 6027 |
| Closed-brush convex box expansion | 982 | 917 | 998 | 2897 |
| Ordinary simple | 927 | 837 | 927 | 2691 |

Generation, compilation and local repair total 242.959 seconds for the shell
and 59.688 seconds for upward terrain. These sums include reused build phases;
they exclude extraction, background compilation and independent validation.
Ordinary simple takes 1.652 seconds for CSG+BSP in this run. Previous closed-box
and legacy-union build phases total 56.978 and 66.285 seconds respectively.
These are measured runs, not a controlled benchmark.

All six variant/hull tests have zero extra/missing solid samples at tolerance
0.01, and zero outside-bounds solid samples. Upward-terrain random ray errors
are at most 0.008896 units and top-grid errors at most 0.008754 units. The shell
retains one random ray error of 0.041416 units; its largest top-grid error is
0.025858 units. A trial half-space repair did not improve shell hull 1, and its
larger tree is discarded. Final hulls use the original fragment builder.

On 8,192 arbitrary exterior rays per hull, unsnapped boundary expansion and
closed-brush box expansion agree within 0.000000441 units. This supports a
shared external contact rule for this geometry. It does not make the actual
simple BSP identical: on 61,404 terrain-first vertical rays for hull 1, its
maximum difference from the snapped shell target is 55.691202 units, versus
0.014779 for the new shell. The origin of that production-simple discrepancy
(expansion or subsequent BSP processing) is not isolated by this experiment.

Both test BSPs contain 46,713 lighting bytes and 142 VIS bytes, share their
noncollision lumps with the closed-box background, and pass graph/root/player
start/union checks. Full shells can have empty deep interiors; upward terrain
omits side/bottom contact. Engine movement remains untested.

Outputs and detailed evidence are under
`build/tmp/untitledx_exterior_final_20261023`, including REPORT.md,
comparison.json, surface_comparison.png, untitledx_surface_shell.bsp,
untitledx_surface_terrain.bsp and a legacy-union comparison BSP using the same
background. Original MAP and game BSP files are unchanged.
