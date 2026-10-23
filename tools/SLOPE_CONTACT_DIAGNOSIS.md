# Single-Slope Contact Diagnosis

The user's `cliptype_sl.map` isolates the distance a standing or crouching
player can approach one slope from its lower end. The six room brushes and
one triangular-prism slope are compiled in isolation; the original MAP and
game installation remain unchanged.

These measurements use the earlier seven-brush snapshot. The updated source
adds a platform and a merged comparison brush; its decomposition-dependent
legacy ledge is investigated separately in `BRUSH_SEAM_DIAGNOSIS.md` with
`brush_seam_diagnose.py`. Single-slope thickness does not establish seam
behavior.

Run `tools/slope_contact_diagnose.py` with `--source`, `--compiler`, `--dumper`,
`--scripts`, `--wad`, and a fresh `--output` directory. It compiles lit legacy
and simple maps, extracts the unchanged slope top into two triangles, invokes
the original reference build/repair functions, and packages a lit Python test
map. The existing background exporter requires a detail entity, so the
diagnostic moves world brush 6 unchanged into detail entity 3 in a separate
temporary MAP. This adaptation is recorded in the manifest.

The test slope is `7*y + 6*z = 144`, spanning X=-96..48 and Y=-48..48. The
contact trace starts at X=-48, Y=140 and travels toward negative Y at fixed Z.
Larger contact Y means the player is held farther from the visible slope.
The measurements traverse actual BSP trees, without engine trace epsilon.

## Measured Results

Artifacts: `build/tmp/cliptype_sl_contact_20261023`.

| Posture | Player centre Z | Legacy contact Y | Simple contact Y | Original Python contact Y | Simple minus legacy |
| --- | ---: | ---: | ---: | ---: | ---: |
| Standing near floor | 4.0625 | 49.318953 | 63.946428 | 63.946407 | 14.627475 |
| Standing | 20 | 35.658239 | 50.285714 | 50.285691 | 14.627475 |
| Standing | 40 | 18.515383 | 33.142857 | 33.142832 | 14.627475 |
| Crouching near floor | -13.9375 | 54.706745 | 63.946427 | 63.946442 | 9.239682 |

All eight tested contacts match the independent swept triangle/box SAT to
within 0.001 for simple and Python. Python's raw external collision model
and world-packaged version produce identical contacts. The original repair
reports both SOLID regions exact and makes no changes. Input snapping moves
vertices by at most 8.53e-12 units. This isolates the thickness difference
from triangulation errors, leaf repair and collision packaging.

In this implementation, for unit normal `n` and symmetric player half-size `h`,
simple shifts a non-axial plane by `sum(abs(n[i])*h[i])`. The legacy branch
uses `sum(n[i]*n[i]*h[i])`. The original Python triangle/box convex expansion
has the first support distance. The two measured offsets therefore differ
on this slope even with only two coplanar triangles and no ambiguous repairs.

This demonstrates the user's reported approach-distance difference on an
actual MAP. Reproducing Boreas collision data and recognizing its in-game
feel as legacy does not establish that its algorithm uses the same plane
offset as the compiler's `-cliptype legacy` on every other map. The Boreas
comparison used different terrain, not this controlled slope.

Playtest BSPs, all with VIS and RAD, are `cliptype_sl_legacy.bsp`,
`cliptype_sl_simple.bsp` and `cliptype_sl_python.bsp`. The latter retains
the original script's hull-2-to-hull-1 alias. `contacts.json`, `REPORT.md`,
script snapshots and compiler command logs preserve the measurements.
No game runtime test is claimed.
