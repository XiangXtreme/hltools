# Legacy Collision Diagnosis

This experiment separates collision already present in individual legacy brush
expansions from excess SOLID introduced while building their BSP. It does not
change a production cliptype or the compiler executable.

## Reproduce

Dependencies: Python, NumPy, matplotlib, hltools, the MAP's WAD files, and the
user-supplied `brushbsp.py` for the one-leaf experiment. Run from the repo root:

```powershell
python tools/test_legacy_clip_diagnose.py
python tools/legacy_clip_diagnose.py `
  --source D:/CS/Mapper/02_Sandbox_Tests/Unity_Test/Cliptype/clip_null_legacy.map `
  --compiler bin/hltools.exe `
  --output build/tmp/legacy_union_diagnosis_final_20261023
python tools/legacy_clip_local_probe.py `
  --experiment build/tmp/legacy_union_diagnosis_final_20261023 `
  --brushbsp C:/Users/Xiang/Downloads/brushbsp.py --radius 96
```

Use a new output directory when repeating the diagnosis. The probe refuses to
overwrite its BSP. The recorded results below use the MAP available on
2026-10-23: 288 terrain brushes plus six room brushes. Its local cliptype was
`exactunion`; diagnostic copies explicitly replace that override for each case.
The earlier 72-terrain-brush findings describe a different revision.

## Geometric Baseline

The `legacytrim` CSG sidecar stores the actual legacy expanded brushes as its
targets, before BSP construction. It also stores brush-plus-player-box volumes
as guards. These guard volumes are useful for explaining the geometric
difference; they are not used as the local repair target.

The experiment runs plain legacy CSG and an audit CSG separately. All `.p0` to
`.p3`, `.b0` to `.b3`, and `.pln` files match byte for byte. The audit sidecar is
read without running its BSP stage. Thus the exported legacy volumes describe
the ordinary legacy tree's input without enabling legacytrim or exactunion.

Both actual BSP builds use two threads, `-nofill -noinsidefill`, and default
brink handling. Removing fills isolates the finite source-brush union; these
are diagnostic BSPs with no VIS, RAD, game deployment, or movement testing.
The window is XY [-288, 288], player-origin Z [-20, 180].

Each hull uses the same 100000 random points, 4096 oblique descending rays and
82944 vertical grid rays. Signed brush-plane margins of 0.05 exclude immediate
boundary disagreements. Ray differences use a 0.1-unit threshold. Grid X/Y
offsets differ to avoid repeated exact diagonal-seam ties. These are finite
geometric probes, not a full-space proof or GoldSrc movement simulation.

## Findings

| Hull | Legacy extra SOLID points vs legacy volumes | Missing points | Rays differing by >0.1 | Maximum finite ray error |
| --- | ---: | ---: | ---: | ---: |
| 1 | 666 | 0 | 161 | 154.92433 |
| 2 | 128 | 0 | 109 | 47.32366 |
| 3 | 27 | 0 | 41 | 20.61712 |

Simple matches its box-expansion baseline on all 100000 points per hull outside
the 0.05 margin. It has no ray errors over 0.1; its maximum finite ray error is
approximately 0.0413. This does not make its contact boundary a legacy boundary.

Two distinct witnesses occur in hull 1:

- BSP classification: (260.10970, -11.36268, 141.84131) is SOLID in the legacy
  BSP, but lies outside all exported legacy volumes, with union plane margin
  61.93831. This is a finite-volume discrepancy, not a surface tie.
- Expansion geometry: at XY (24.173205, 190.271828), the legacy volume union's
  top is 72.285267, while the box-expansion union's top is 62.812687. The
  difference is 9.472581 units. The actual legacy BSP top is 72.285270.
  An interior point between the two heights is covered by legacy volume 196,
  has legacy union margin -0.128965, and box union margin +4.492228.

Consequently, correcting BSP classification alone cannot remove the second
kind of protrusion while preserving all legacy expansion geometry.
`comparison.png`, `volumes.json`, and `summary.json` record both cases.

## One-Leaf Experiment

The local probe walks only the hull-1 path containing the first witness. It
intersects that path's convex cell with a bounded diagnostic box, clips the
original legacy volumes to that region, and calls the supplied brushbsp local
builder. It uses 36 bounding-box candidates, two clipped fragments, and a
two-node local subtree. Outside that box the original leaf remains SOLID.
Every other original path retains its contents. This is one witnessed leaf
occurrence, not automatic discovery or repair of all bad leaves.

| Output | Stored clipnodes | Reachable hull 1 | Hull 2 | Hull 3 |
| --- | ---: | ---: | ---: | ---: |
| Plain legacy | 5597 | 1925 | 1243 | 2429 |
| One-leaf probe | 5605 | 1933 | 1243 | 2429 |
| Plain simple | 3243 | 947 | 815 | 1481 |

Plain legacy CSG+BSP takes approximately 2.36 seconds; simple takes 1.42
seconds. The one-leaf modification takes approximately 0.062 seconds, including
fragment construction and BSP encoding, excluding validation and plotting.
The separate metadata audit CSG takes 0.61 seconds. These are individual
subprocess wall times, not a whole-map repair benchmark.

Validation of the probe:

- The classification witness changes from SOLID to EMPTY. The expansion
  witness remains SOLID.
- 100000 global plus 100000 box-local probes find 7339 SOLID-to-EMPTY changes;
  none removes coverage with legacy plane margin below -0.05. Local samples
  are deliberately oversampled and must not be treated as global error rates.
- In the unchanged global probe set, extra-solid points decrease from 666 to
  329, with zero missing-solid points after repair. In the local set they
  decrease from 7253 to 251, also with zero missing-solid points.
- On the fixed global ray set, errors over 0.1 decrease from 161 to 116.
  The maximum remaining error is 127.29614, so this is not a complete repair.
- Hulls 2 and 3 have identical recursive graph hashes using disk plane bytes,
  after allowing index renumbering. All noncollision lumps match byte for byte.
- All hull roots, descendants, plane indices, contents and depths pass
  structural validation. Hull 1 depth stays 44; hulls 2 and 3 stay 52 and 48.
  Nodes are emitted parent before child to preserve GoldSrc headnode limits.
- Input MAP, compiler, baseline BSP, and supplied Python module hashes remain
  unchanged. The output is `legacy_local_probe.bsp`; it is a diagnostic file.

`local_probe.png` shows the corrected section; `local_probe.json` contains
counts for the global and box-local probe sets separately. The retained
expansion spikes and remaining bad BSP leaves are intentional experiment
limits, not evidence that the complete map has been repaired.

## Validator Regression

During this experiment, integer-valued trace lengths exposed integer interval
storage in `source_clip_validate.tree_first_hit`. Fractional exit distances
were truncated, potentially discarding thin solid intervals. The shared helper
now explicitly allocates floating-point bounds. A regression checks a thin
slab from both directions and from an interior start, with integer and float
lengths. The previous Source terrain validation passed `192.0`, so that run
did not use the defective integer storage.

Preliminary scratch runs with the integer-bound checker and coincident grid
offsets are superseded by `legacy_union_diagnosis_final_20261023`.
