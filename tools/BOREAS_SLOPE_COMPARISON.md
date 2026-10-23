# Boreas Triangle Versus Simple

`boreas_slope_compare.py` selects actual saved Source terrain triangle 62996,
belonging to rebuilt `bclip000.bsp` and archive `bclipv2_000.bsp`. It retains
the original map coordinates and calls the original Python build and repair
functions on that triangle. The same triangle becomes the outward face of a
256-unit inward prism, compiled with ordinary simple and legacy expansion.

```powershell
python tools/boreas_slope_compare.py `
  --build D:/Code_Development/Source_code/CPP/hltools/build/tmp/source_boreas_pipeline_20261023 `
  --archive C:/Users/Xiang/Downloads/surf_boreas.zip `
  --output D:/Code_Development/Source_code/CPP/hltools/build/tmp/boreas_simple_sample_20261023
```

Use a new output directory. Original script and compiler snapshot hashes are
verified. All five actual collision trees are structurally checked. Tests
trace 1024 rays per hull toward the central part of the surface, using the
same origins and directions for every BSP and independent swept-box SAT.
This also checks whether other triangles in the parent BSP change the
selected surface's measured contact.

The input triangle vertices are:

```text
(-8098.6875, -15183.1250, 1289.0625)
(-8039.1250, -15103.5625, 1146.8750)
(-8191.3750, -15139.8125, 1123.5625)
```

Its normal is approximately `(-0.269004, 0.884129, 0.382037)`.
The following are outward normal distances from its centroid to the
player-centre collision boundary. These are actual BSP interval-trace
measurements, before engine trace epsilon:

| Hull | Archive parent | Rebuilt parent | Python isolated triangle | Simple | Legacy |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 32.203285 | 32.203285 | 32.203285 | 32.202872 | 18.918693 |
| 3 | 25.327546 | 25.327546 | 25.327546 | 25.325919 | 16.291740 |

Across 1024 rays per hull, simple differs from the isolated Python result
by at most 0.000415 units (h1) and 0.001629 units (h3). The archive and rebuilt
parent contact distances are identical on every tested ray. The isolated
Python and full-parent results also have identical measured errors relative
to SAT. No hit/miss differences occur. All simple/Python results agree with
SAT within 0.001 units individually; remaining differences are consistent
with finite plane/coordinate precision at the retained large coordinates.
Legacy differs from SAT by approximately 13.284 units (h1) and 9.035 units
(h3), confirming a different slope offset.

This establishes simple-equivalent front-face thickness on this actual
Boreas fragment, including the author's distributed collision BSP. It does
not imply byte-identical BSPs, equal full volumes, identical edge behavior
on every terrain patch, or the compiler invocation used historically.
The prism has filled material behind its front face; the original Python
target is an expanded triangle shell. Only exterior first contacts in the
specified central region are compared.

Artifacts include `sample_simple.map/.bsp`, `sample_legacy.map/.bsp`,
`clips_out/sample.bsp`, `comparison.json`, `REPORT.md`, `probes.npz`, and logs.
They are isolated collision models for inspection, not standalone playtest
maps. No game files are installed or modified.
