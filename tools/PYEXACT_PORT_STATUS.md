# pyexact C++ port

## Goal

Reproduce `untitledx.before_boundary_normalized.bsp` collision behavior by
porting the Python path directly:

1. Extract the exposed exterior surface of each closed brush assembly.
2. Remove faces covered by other brushes in the same entity.
3. Triangulate the remaining polygons.
4. Expand each triangle by the player hull box (the convex hull of the three
   triangle vertices plus the eight box-corner offsets).
5. Build a GoldSrc clipnode tree with the `brushbsp.py` fragment splitter.

The result must preserve concave assemblies, remove internal brush seams, and
avoid both legacy seam bumps and simple's over-thick collision.

## Scope

- New independent clip type: `pyexact`.
- Existing `simple`, `legacy`, `legacytrim`, and `exactunion` remain unchanged.
- Do not route this feature through `solid_bsp`, `legacytrim`, or `exactunion`.
- First validation target: `cliptype_sl.map`, then `untitledx.map`.

## Python reference

- `C:/Users/Xiang/Downloads/brushbsp.py`: fragment representation, split,
  splitter score `5 * cut + abs(front - back)`, leaves `EMPTY=-1`, `SOLID=-2`.
- `tools/brush_exterior_pipeline.py`: per-face subtraction by other convex
  brushes, exposed polygon triangulation, orientation/probe checks.
- `C:/Users/Xiang/Downloads/exacthull.py`: triangle + eight hull-box corners;
  standing hull uses +/-16,+/-16,+/-36, ducking uses +/-16,+/-16,+/-18.

## C++ integration

Current compile flow is in-memory:

`run_csg -> build_intermediate_data -> build_bsp_data -> bsp::run_bsp`.

Existing BSP input only carries `.p0-.p3`, `.b0-.b3`, `.hsz`, `.pln` text/data.
The direct tree needs a new `surface_clip` payload in `intermediate_data` and
`bsp_input`. In `bsp::process_model`, hull 0 remains the normal draw BSP;
`pyexact` hulls install direct planes/nodes and skip `solid_bsp` for those
hulls. Hull 2 may reuse hull 1. Direct nodes need plane and node index offsets
when appended to `map.planes` / `map.clipnodes`.

## Current state

- `pyexact` geometry/tree path now exists in `src/csg/surface_clip.cpp`.
- Old `boundary_normalized` registration is removed.
- `tools/BOUNDARY_NORMALIZED_STATUS.md` records the old failed approach.
- Working tree has that status document untracked; preserve unrelated changes.

## Validation so far

- `cliptype_sl.map` with `-cliptype pyexact` compiled successfully; its direct
  hulls contained 326 and 342 reachable nodes.
- A temporary `untitledx` copy with its local `zhlt_cliptype` changed to
  `pyexact` compiled with VIS and RAD. The direct path produced 896 exterior
  triangles, hull1 14946 nodes, hull3 15261 nodes, and 30207 total clipnodes.
- The original `untitledx.map` has a local `zhlt_cliptype simple`, so a global
  `-cliptype pyexact` correctly leaves that entity on simple under the normal
  local-override rule. The temporary copy was used to exercise pyexact.
- The first full scale attempt exposed the need to skip ordinary clip hull
  emission for direct models; that is now fixed in `src/bsp/bsp.cpp`.
- The standalone `csg` → `bsp` chain is supported through a versioned `.scl`
  sidecar containing the direct trees; the unified `compile` path passes the
  same payload in memory. The sidecar is removed after BSP completion like the
  existing `.pN/.bN/.hsz/.pln` files.
- With the final Python-aligned thresholds, `untitledx` compiles with 942
  exterior triangles, 15215 hull1 nodes, 15160 hull3 nodes, and 30375 total
  clipnodes. A 60000-point random occupancy check had zero mismatches against
  the expanded-triangle convex union.
- The complete existing test suite passes: 193/193 tests.

## 2026-10-24 implementation notes

- The direct tree will be emitted after the normal BSP writer's plane
  optimization and brink pass. This keeps the Python tree out of the legacy
  `solid_bsp` and out of brink rewrites, while still using the final BSP plane
  and clipnode lumps.
- Direct planes are stored as explicit `(normal, dist)` records and appended
  as even/odd GoldSrc plane pairs. Direct clip nodes refer to the even plane;
  their front child is the `n dot x - d >= 0` branch, matching `brushbsp.py`.
- The CSG payload follows the original `result.entities` model order. This is
  the same order used by the existing `.pN/.bN` intermediate surface files,
  so `reuse_models` entity reordering does not invalidate the direct payload.
- A model uses the direct path only when it contains at least one solid brush
  and every solid brush in that model is marked `pyexact`; mixed clip modes
  fall back to the existing BSP path until an explicit mixed-mode policy is
  added.

## Risks/checks

- `dclipnode_t.children` are signed 16-bit; total clipnodes must stay below
  32767.
- Direct planes must be emitted as `bsp::plane`/`format::dplane_t` with correct
  indices; do not depend on ordinary CSG plane-store indices.
- Model order can change in `reuse_models`; direct payload must follow the
  final model order or be keyed by entity/model consistently.
- Do not triangulate all brush faces without subtraction: internal seams would
  remain and reproduce the legacy problem.

## Next implementation slice

Implement `src/csg/surface_clip.h/.cpp` with the data structures and geometry
builder first, add the payload fields and `pyexact` parser second, then wire
the BSP direct-node installer. Build after each slice.
