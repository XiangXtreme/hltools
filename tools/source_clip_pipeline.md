# Source Terrain Collision Reproduction

This experiment runs the seven supplied Python scripts against a Source BSP
and compares their collision output with the distributed surf_boreas clips.
The compiler's production collision implementation is not changed.

## Reproduce

Requirements: Python 3.12, NumPy, SciPy, the supplied script directory,
hltools.exe, and a WAD containing the special NULL and CLIP textures.
Run these commands from the repository root with a new output directory:

```powershell
python tools/source_clip_pipeline.py prepare `
  --source D:/CS/tools/CS2/bspsrc-windows/surf_boreas.bsp `
  --vmf D:/CS/tools/CS2/bspsrc-windows/surf_boreas_d.vmf `
  --scripts C:/Users/Xiang/Downloads `
  --compiler D:/Code_Development/Source_code/CPP/hltools/bin/hltools.exe `
  --wad D:/steam/steamapps/common/JACK/mapper/wad/sdhlt.wad `
  --output D:/Code_Development/Source_code/CPP/hltools/build/tmp/source_boreas_pipeline_20261023

python tools/source_clip_pipeline.py build --output <output> --workers 2
python tools/source_clip_validate.py repair --output <output> --workers 2
python tools/source_clip_validate.py compare --output <output> --archive C:/Users/Xiang/Downloads/surf_boreas.zip
python tools/source_clip_validate.py validate --output <output> --archive C:/Users/Xiang/Downloads/surf_boreas.zip
```

`<output>` means the absolute directory supplied to `prepare`.
Optional `repair-chunks` can process the current `build_progress.json` while
compilation continues; the final repair command reuses these results by hash.
It must finish before running the final repair command.

## Adapters

- Snapshot the original Python files and compiler, recording their SHA256s.
- Expand compressed top-level lumps using Source's LZMA header and declared
  output length. Append decoded data to preserve existing absolute offsets.
  This is a reader input, not a Source engine round-trip validation.
- Redirect all hard-coded input/output paths into the experiment directory.
- Supply the missing `bsp_hull` reader import with a compatibility reader.
  The original `clipcheck.box_hits` and repair geometry code are retained.
- Limit compiler threads and preserve every command, return code, and log.
  Successful CSG is required before BSP conversion.

The runner uses the original triangulation, 1/16 snapping, plane
canonicalization, convex expansion, split rule, `solid_tree`, `splice`, and
`fix_file` functions. It does not install models or plugins into the game.
The original compiler flags are retained; the added `-threads` controls
scheduling. Collision files do not need VIS or RAD.

The compression layout is documented in Valve's
[lzmaDecoder.h](https://github.com/ValveSoftware/source-sdk-2013/blob/master/src/public/tier1/lzmaDecoder.h)
and [bspfile.h](https://github.com/ValveSoftware/source-sdk-2013/blob/master/src/public/bspfile.h).

## Input Audit

- Source BSP: VBSP v20, 1125 displacement records.
- VMF: 1125 records; start/power keys and their multiplicities match the BSP.
- There are 93 repeated start/power keys, with no conflicting no-hull flags.
- Script selection: 706 solid displacements; 369 excluded by material and
  another 50 by collision flags. The VMF has 59 flagged records in total.
- Selected geometry: 83648 nondegenerate triangles, initially 35 chunks.
- Maximum selected vertex displacement from 1/16 snapping: 0.0540734 units.

Input BSP SHA256:
`b8d3fac43ab01db112d047f1dbe6eee268c54a406eac428bd60e12fa37d69dc7`

VMF SHA256:
`59ba1858b6820e688a53c537122ef65af7244821314711a01f17a9b58524c37b`

## First File Evidence

For `bclip000.bsp` versus the archive's `bclipv2_000.bsp`:

| Stage | Stored clipnodes |
| --- | ---: |
| Original script build, before repair | 24463 |
| Original script repair | 24641 |
| Distributed reference | 24642 |

The repair checks 11258 solid regions and replaces 82 of them, appending
178 nodes. Of the changed primary-tree child slots, 61 become EMPTY and
21 point to appended subtrees. All changed slots were previously SOLID.

The reference has one reserved empty clipnode at index zero, with plane zero
and two EMPTY children. After accounting for that node and its index offset:

- All 10872 plane records are byte-identical, including the repair planes.
- All 24641 collision nodes have identical plane and child references.
- Hull roots and model bounds agree.

The complete BSP files are not byte-identical: metadata and packaging may
differ. This demonstrates reproduction of the collision data. It provides
strong evidence for this pipeline, rather than a record of the historical
build invocation or exact script revision.

Plane lump SHA256:
`6f004bb70acfbff05de67fe605c77a036edddb35c149a258a93395dbb4271b27`

Normalized collision-node SHA256 (little-endian int64 triples):
`dece76cdb27828e94542c16d3927b0efbce0d1efd8f08a063a9d3961f8a931c4`

## Full Build Results

Four initial chunks exceed the 31000-node limit and split again, producing
39 files. All compiler stages return zero, with no dropped-side retries.

| Output | Files | Total stored clipnodes |
| --- | ---: | ---: |
| Rebuilt primary trees | 39 | 984139 |
| Rebuilt, original local repair | 39 | 993637 |
| Distributed reference | 39 | 998129 |

The repair checks 437022 SOLID regions and replaces 2383. It appends 9498
nodes, an increase of 0.9651%. No file falls back because of memory or node
limits. Peak reported worker memory reaches approximately 1039 MB.

The build takes 1086.8 seconds with two Python workers and two compiler
threads per invocation. This includes Python geometry generation and the
oversized chunks that are subsequently split. Across all 43 attempted
chunks, 86 CSG invocations total 456.9 seconds and 86 BSP invocations total
372.4 seconds. These accumulated invocation times overlap across workers.
Repair overlaps compilation; accumulated per-file repair elapsed time is
2260.0 seconds, not a separate wall-time benchmark.

After accounting for the reference's reserved node, 35 of 39 files have
byte-identical plane lumps and identical complete collision-node arrays,
including the appended repairs. Their hull roots also agree after the
offset. Files 005, 006, 009, and 017 differ already in the primary trees.
Their provenance differences have not been resolved by this experiment.
Four otherwise matching files, 010, 011, 012, and 038, additionally differ
in model bounds. Thus complete-file or all-metadata equality is not claimed.

## Validation Scope

The independent ray checker intersects a moving box with triangles using
intervals on all 13 separating axes. It is checked against independently
constructed convex expansions on 128 synthetic rays. The BSP ray walker
also passes three known box hit/miss/start-solid cases.

The full checks use the same selected, snapped triangles as the scripts.
They compare raw, repaired, and distributed collision on identical probes.
Point checks shrink/expand box half-sizes by 0.01 to identify errors away
from the immediate boundary. Ray checks traverse actual BSP intervals,
instead of assuming a single crossing for bisection. These are sampled
geometry checks, not an in-game movement test or a full correctness proof.

Both player hulls use 50000 point probes and 4096 normal-directed rays,
with seed 20261023. Half of the point samples lie near triangle corners.
The raw/archive and repaired passes use exactly identical probe arrays.

| Hull | Output | Extra solid points | Missing solid points | Ray errors >0.1 | Ray hit/miss differences | Maximum finite ray error |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | Raw | 112 | 0 | 75 | 0 | 70.1183 |
| 1 | Repaired | 3 | 0 | 10 | 0 | 2.1126 |
| 1 | Archive | 3 | 11 | 10 | 1 | 2.1126 |
| 3 | Raw | 104 | 3 | 80 | 0 | 68.8983 |
| 3 | Repaired | 1 | 3 | 12 | 0 | 3.4536 |
| 3 | Archive | 1 | 15 | 13 | 1 | 3.4536 |

Point counts exclude disagreements within the box-size tolerance described
above. Maximum distances exclude hit/miss differences; the separate column
reports those. This validates the selected, snapped triangle target, rather
than every aspect of Source or GoldSrc engine movement.

The archive's hit/miss witness is ray 1894, near triangle 4122 at
(853.3333333, 11583.1875, 9142.1875), in clip file 017. At that triangle's
centre, the rebuilt raw and repaired hulls report SOLID, while the archive
reports EMPTY. This is a concrete difference relative to this input.

All three variants pass plane/child range, cycle, hull-root lower-bound,
and per-file clipnode budget checks. Their maximum observed depth is 749.

The original `clipcheck.main()` was also attempted with 4096 samples. It
fails at line 109 with `index 83648 is out of bounds ... size 83648`.
Its candidate fallback at line 108 can retain the invalid index when the
first candidate is also missing. The independent checker above avoids the
fixed 96-candidate limit and uses complete swept bounds. The original
`box_hits` function used during repair is unaffected by this CLI defect.

Fresh preparation reproduces exactly the same triangle array. Hashes confirm
that the original BSP, VMF, and all seven supplied scripts remain unchanged.

## GoldSrc Implications

The useful construction pattern is to precompute the blocked player-origin
volume, compile its union as ordinary hull-zero geometry, convert that
primary tree, and rebuild only failing leaf regions. The reference does not
require a global collision-tree rebuild during repair.

Several aspects require care for a general GoldSrc brush workflow:

- The input here is a triangle surface. A filled brush volume, including its
  interior, is a different target from a shell of expanded surface triangles.
- The script's repair tests SOLID leaves at selected points and skips thin
  regions. EMPTY errors and unsampled gaps can survive.
- Plane canonicalization and removing rejected sides introduce geometric
  approximations. The name "exact" is not an unrestricted precision claim.
- Hull 2 aliases hull 1 in these clips. A general compiler may need a distinct
  hull 2 with its configured dimensions.
- Each external BSP has its own node budget. Whole-map node totals must be
  considered alongside per-file counts and entity/model overhead.
- Production world geometry also has liquids, exterior fill, other models,
  and unselected brushes that are outside this isolated collision workflow.

## Artifacts

All large outputs live under the supplied output directory:

- `config.json`: input, script, compiler hashes and decompression records.
- `input.json`, `input_audit.json`: extracted geometry and input correspondence.
- `exact_chunks/`: expanded MAPs, per-hull BSPs, all compiler logs.
- `clips_raw/`, `clips_out/`: collision files before and after repair.
- `build_results.json`, `repair_results.json`: counts and elapsed times.
- `archive_comparison.json`: exact per-file comparison and fingerprints.
- `validation.json`, `validation_probes.npz`: independent probe results.

The output BSPs are external collision models, not standalone playable maps.

## Native Map Installation

For the existing `surf_brs_native` map, install the repaired reference build
into its already-preloaded external model paths:

```powershell
python tools/install_native_reference_clips.py `
  --build build/tmp/source_boreas_pipeline_20261023 `
  --game D:/steam/steamapps/common/Half-Life/cstrike `
  --output build/tmp/surf_brs_native_reference_install_20261023 --install
```

Use a new output directory. Without `--install`, this stages and backs up the
files for inspection. The installer verifies all 39 source repair hashes and
script snapshots, map precache/wall references, collision structures and backup
hashes before overwriting files. Installation failures attempt to restore the
complete backup. The main map and texture carrier hashes must stay unchanged.

The adapter preserves the native draw-tree container, its reserved empty node
and the existing broad-phase bounds needed by the native loading setup. It
extends those bounds when needed, offsets rebuilt collision references by one,
and verifies that the resulting planes and collision nodes match the rebuilt
source. Template draw-plane compatibility is checked before replacement.

On 2026-10-23, all 39 installed files passed independent `sdHLInspect` checks
and readback SHA256 verification. Total terrain clipnodes changed from 998129
to 993676, including 39 reserved nodes; the largest file has 31092 nodes.
Files 005, 006, 009 and 017 changed. The other 35 installed files were already
byte-identical to the adapted original-Python rebuild. The main map remains
SHA256 `0e5388e232bf76057e4f0fd8a8094198e34dbb41b83468b394bfa11c287d05fc`.

The output directory contains `staged/`, `backup/` and `installation.json`
with per-file source, before/after hashes, counts and structure checks.
Disconnect and reload `map surf_brs_native` to test the installed files;
an already-loaded map continues using its in-memory collision models.
