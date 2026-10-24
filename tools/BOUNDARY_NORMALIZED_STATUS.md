# Boundary-normalized collision status

## User goal

The desired result is the collision behavior of the Python
`untitledx_boundary_normalized.bsp` experiment: a closed collection of convex
brushes should behave as one continuous exterior surface. Shared internal
brush boundaries must not create legacy-style outward bumps, while concave
assemblies must retain their real exterior shape. Clipnode counts must remain
close to the Python result instead of expanding to the whole-map legacy limit.

## Evidence from the Python path

The reference scripts are under `C:\Users\Xiang\Downloads`:

- `brushbsp.py` builds a clipnode tree from convex fragments.
- `exacthull.py` converts each source surface triangle into a player-box
  expanded convex term by taking the convex hull of the triangle vertices plus
  the eight hull-box corner offsets.
- `srcbsp.py` and `build_boreas_clips.py` write the independent collision BSP
  and splice it into a normal GoldSrc BSP while preserving render, VIS and
  lighting data.

The successful `untitledx` Python output was recorded in
`build/tmp/untitledx_boundary_normalized_compiled_fragments_20261024`.
The recorded result was about 3,067 stored clipnodes across hulls 1-3, with
roughly 155 seconds of Python processing and holdout checks over approximately
250,000 points and 8,192 rays.

## C++ implementation history

An earlier experimental C++ option named `boundary_normalized` was added to
the ordinary brush CSG pipeline. It merged adjacent convex brushes and then
marked their expanded brush terms as `exact_union`, causing the BSP stage to
reuse the existing `exactunion`/`clip_trim` path. This was not equivalent to
the Python algorithm: it still processed the complete map as ordinary brush
volumes and did not extract an exterior surface or build an independent
triangle-fragment tree. On `untitledx.map` it reached about 37,000 clipnodes and
failed with `legacytrim: collision exceeds 32767 clipnodes`.

That option has now been removed. The standalone `exactunion` cliptype remains
available for maps that explicitly request it, but it is not the implementation
of the new feature.

## Required next architecture

Implement a separate collision-surface input path rather than extending
`exactunion`:

1. During CSG, identify the selected closed brush assembly and extract only its
   exposed hull-0 boundary faces. Internal brush faces must be removed.
2. Triangulate each exposed polygon and retain the outward source normal.
3. For each requested hull, form one convex term per triangle from its three
   vertices plus the eight player-box corner offsets. Compute the term's plane
   set and bounds.
4. Port the fragment representation and splitter selection from `brushbsp.py`
   (`make_frag`, `split_frag`, `build`, `clip_tree`) into a C++ collision-tree
   builder. Its leaves must use GoldSrc clip contents and its nodes must be
   emitted directly as `format::dclipnode_t`.
5. Attach the generated tree to the relevant model's `headnode[hull]` without
   sending those terms through `solid_bsp`, `legacytrim`, or `exactunion`.
6. Preserve hull 0, draw geometry, VIS, lightmaps, ordinary brush entities and
   all other cliptypes. Hull 2 may reuse hull 1 only if this matches the selected
   player hull semantics.

The first implementation target should be one explicitly selected world or
entity assembly and hull 1. Validate it against the Python BSP before adding
automatic whole-map selection and hulls 2/3.

## Current source state

The working tree contains unrelated-to-this-document experimental changes in
the CSG files. The old `boundary_normalized` registration and its unit tests
were removed because they described the wrong algorithm. Do not reintroduce the
name until the direct surface/tree path exists.

## Validation checklist

After the direct path is implemented:

- `cmake --build build\\vs --config Release --parallel 4`
- `ctest --test-dir build\\vs -C Release --output-on-failure`
- compile `D:\CS\Mapper\02_Sandbox_Tests\Unity_Test\Cliptype\cliptype_sl.map`
  and `untitledx.map` with the new cliptype;
- verify hull 1 player movement at brush seams and concave joins;
- inspect stored and reachable clipnode counts per hull;
- verify render, VIS, light and entity lumps are unchanged except for the
  intended collision headnodes.

## Render-gap diagnosis and fix (25 October 2026)

The three reported coordinates

```text
(-846.702, 125.916, -40.404)
(-843.000, -5.500, -33.000)
(-868.375, 93.514, -35.573)
```

are an actual legacytrim model-1 face (face 620) in the old output.  The
simple BSP contains the same area in world faces 589/590.  The face was present
in the file, so this was not a missing triangle in the direct collision tree;
the problem was that the complete render hull was being built as a separate
`func_wall` model while its collision hull was also replaced by the direct
surface tree.  That split produced a render seam/hole at this brush assembly.

The current fix keeps two paths for a local `func_detail` with
`zhlt_cliptype=legacytrim`:

1. A render-only copy of its brushes is inserted into worldspawn.  Those
   copies have `noclip=1`, so they contribute hull 0 faces but no ordinary
   hull 1-3 clipnodes.
2. The original entity remains a `func_wall`, is tagged internally with
   `zhlt_collision_only=1`, and emits no hull-0 draw faces.  Its hulls 1-3 are
   replaced by the Python-style direct fragment trees.

The resulting reproduction BSP has 712 world faces, 2 models, zero visible
faces in the collision-only model, and 4,925 total clipnodes.  The target
coordinates now resolve to world geometry while the direct collision model
still owns hull 1/3.  Release build and all 165 tests pass.
