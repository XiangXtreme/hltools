# Legacy Union Experiment

This experiment preserves the geometry produced by the actual legacy brush
expansion. It does not expand brush vertices by the player box or use simple
collision as the repair target. Some protrusions can already belong to the
legacy brush union; these remain part of the target and are not removed here.

The helper exports per-brush legacy hull planes, winding vertices and original
entity identities. The runner selects the requested solid func_detail entity
and matches every selected hull against the legacytrim diagnostic sidecar.
Ordinary legacy and audit CSG intermediates must be byte-identical. The
legacytrim BSP pass is never invoked. Box-expansion volumes in the sidecar
serve only as a comparison, never as replacement geometry.

Already-expanded legacy volumes are written as hull-0 brushes and compiled with
`-noclip`. Misclassified leaf regions are located with region probes, random
points and rays, and a 193x193 vertical training grid. Each selected leaf gets
direct convex half-space union tests of the original legacy planes. This avoids
relying on new fragment-edge incidence to determine its contents. All decisions
use floating-point tolerances; sampling is not a whole-space proof.

The repaired terrain trees are explicitly bounded and compacted, then unioned
with the background room collision. The helper retains the visible terrain in
that background while disabling its player collision. BSP, VIS and RAD run
before collision splicing. Player hulls 1, 2 and 3 are independent. Other brush
models and non-EMPTY/SOLID backgrounds remain outside the experiment's scope.

```powershell
cmake --build build/vs --config Release --target brush_geometry_dump -j 2
python tools/legacy_union_pipeline.py `
  --source D:/maps/terrain.map `
  --entity 3 `
  --compiler bin/hltools.exe `
  --dumper bin/tools/brush_geometry_dump.exe `
  --scripts C:/Users/Xiang/Downloads `
  --wad D:/assets/sdhlt.wad `
  --output build/tmp/legacy_union_experiment
```

Use a new output directory. NumPy, SciPy and Matplotlib are required along with
the seven reference scripts. Inputs, hashes, generated geometry, compiler logs,
holdout results, a comparison plot, report and test BSP are retained there.
The original MAP and installed BSP are not overwritten.

Independent validation uses 250,000 points, 8,192 random rays, about 96,000
exterior points and a separate 257x257 top grid for each hull. Raw and repaired
errors, node counts and timings remain in results.json, including residual
errors. The final graph also checks player starts, the node budget, indices,
cycles and GoldSrc root lower bounds, and preserves all noncollision BSP lumps.

This is an isolated experiment, not a new production cliptype. It tests whether
CSG/BSP classification artifacts can be corrected while keeping legacy slope
positions. Changing joins intrinsic to the legacy-expanded union needs a
separately defined geometric rule and is not established by this experiment.

## Measured untitledx Example

For source SHA256
`b9741eaeb656c312d13a7b46254276c11520c9d9927e54ff916f4453a838fe99`,
the 360 selected brushes produce the following test result:

| Hull | Ordinary legacy nodes | Repaired nodes | Original extra-solid samples | Repaired extra-solid samples | Top-grid max error |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 4217 | 6461 | 328 | 0 | 0.021562 |
| 2 | 3704 | 5628 | 14 | 0 | 0.018093 |
| 3 | 4013 | 6226 | 895 | 0 | 0.016153 |

Both counts include background collision and independent hull 2. Total nodes
increase from 11,934 to 18,315. Extra-solid probes use the independent 250,000
point sets restricted to background EMPTY, at a 0.01-unit threshold. Missing
solid counts are zero for both versions. Complete top holdout checks retain
1/1/2 rays above the 0.01-unit error threshold, so this is not a perfect result.

At hull 1 XY (294.424571, 197.990226), the preserved legacy target is Z 59.055327,
the repaired BSP is Z 59.055360, and the player-box target is Z 90.732101. This
witness explicitly distinguishes the preserved target from simple-like box
expansion. It does not establish equivalence with the boreas reference BSP.

The test took about 171 seconds including compilation, repair and validation.
It retains 46,713 lighting bytes and 142 VIS bytes. Workspace outputs are under
`build/tmp/untitledx_legacy_union_planes_20261023`, including REPORT.md and
untitledx_legacy_union.bsp. This remains a correctness experiment; node count
and local chain depth have not been optimized.

```powershell
python tests/tools/test_legacy_union_pipeline.py
python tests/tools/test_brush_surface_pipeline.py
```
