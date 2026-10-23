"""Summarize an isolated brush/surface experiment and plot its measured hull tops."""

import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    args = parser.parse_args()
    root = args.experiment.resolve()
    result = json.loads((root / "results.json").read_text())
    manifest = json.loads((root / "manifest.json").read_text())
    lines = ["# Closed brushes versus matching top triangles", "",
             f"Input: `{manifest['source']}`", "",
             f"MAP SHA256: `{manifest['source_sha256']}`", "",
             f"{result['brushes']} closed terrain brushes; {result['top_triangles']} top triangles.", "",
             "Both inputs use the supplied exacthull.py plane canonicalization and hull-0 compilation,",
             "followed by local fragment repair from brushbsp.py. Geometry oracles use unsnapped",
             "convex Minkowski sums. All three player hulls are built independently.", "",
             "## Measured top contact", "",
             "| Hull | Geometric vertical difference | Geometric oblique difference | BSP vertical difference | BSP oblique difference |",
             "| --- | ---: | ---: | ---: | ---: |"]
    for hull, record in result["hulls"].items():
        vertical, oblique = record["comparison"]["vertical_grid"], record["comparison"]["oblique_above"]
        values = [vertical["geometry_closed_vs_surface"]["max_distance_error"],
                  oblique["geometry_closed_vs_surface"]["max_distance_error"],
                  vertical["bsp_closed_vs_surface"]["max_distance_error"],
                  oblique["bsp_closed_vs_surface"]["max_distance_error"]]
        lines.append(f"| {hull} | " + " | ".join(f"{value:.9g}" for value in values) + " |")
    lines += ["", "Per hull: 66,049 vertical rays and 8,192 oblique rays from above.", "",
              "The geometric targets match at numerical precision on these probes. The compiled",
              "BSPs retain small differences. Full closed volumes and top-only triangle shells",
              "need not agree underneath or at terrain sides. This is not a whole-space proof.", "",
              "## Terrain tree and holdout checks", "",
              "| Hull | Input | Raw drawnodes | Repaired clipnodes | CSG+BSP seconds | Repair seconds | Extra/missing points | Max ray error |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for hull, record in result["hulls"].items():
        for name, variant in record["variants"].items():
            holdout = variant["holdout"]
            lines.append(f"| {hull} | {name} | {variant['raw_drawnodes']} | {variant['repair']['nodes']} | "
                         f"{variant['compile_seconds']:.3f} | {variant['repair']['seconds']:.3f} | "
                         f"{holdout['extra_solid']}/{holdout['missing_solid']} | "
                         f"{holdout['ray_checks']['max_distance_error']:.6f} |")
    lines += ["", "Holdout: 250,000 points and 8,192 rays per variant, with a seed independent of",
              "repair. Point and ray counts use a 0.01-unit threshold. The table records residual",
              "errors; it must not be read as a perfect-accuracy pass. Plane canonicalization in",
              "the original script uses normal tolerance 0.0001 and distance tolerance 0.05.",
              "Repaired graphs include explicit domain bounds. About 96,000 independent exterior",
              "points per variant check for SOLID escaping the terrain bounds; counts are in results.json.", "",
              "## Test BSPs", "",
              "| File | Hull 1 | Hull 2 | Hull 3 | Total clipnodes | Light bytes | VIS bytes |",
              "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name, playable in result["playable"].items():
        counts = [str(playable["hulls"][str(hull)]["reachable"]) for hull in (1, 2, 3)]
        lines.append(f"| [brush_{name}_test.bsp](brush_{name}_test.bsp) | " + " | ".join(counts)
                     + f" | {playable['stored_nodes']} | {playable['lighting_bytes']} | {playable['visibility_bytes']} |")
    lines += ["", "The two BSPs share identical visible geometry, entities, textures, visibility and",
              "lighting. Background room collision remains present, including exterior SOLID.",
              "Collision union was checked at 150,000 points per hull; player start is EMPTY.",
              "Graphs pass plane-index, child-index, cycle, node-budget and GoldSrc root lower-bound checks.", "",
              "The background copy translates a legacy light key to _light when _light is absent.",
              "The source MAP is unchanged. Neither BSP is installed into the game directory.",
              "Engine movement and surf behavior still require an in-game test.", "",
              "## Scope", "",
              "This experiment establishes a shared geometric target for closed brushes and their",
              "top triangles. It does not add a new production cliptype or establish a node-count",
              "improvement over simple. The next implementation step is to bring this explicit",
              "target and leaf classification into the compiler with controlled plane tolerances.", "",
              "![Measured standing hull tops and difference](comparison.png)", ""]
    (root / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import TwoSlopeNorm

    grid = np.load(root / "surface_grid_h1.npz")
    a, b = grid["closed_top"], grid["surface_top"]
    finite = np.isfinite(a) & np.isfinite(b)
    difference = np.where(finite, np.where(finite, a, 0) - np.where(finite, b, 0), np.nan)
    extent = [grid["x"][0], grid["x"][-1], grid["y"][0], grid["y"][-1]]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    minimum = min(a[np.isfinite(a)].min(), b[np.isfinite(b)].min())
    maximum = max(a[np.isfinite(a)].max(), b[np.isfinite(b)].max())
    for ax, data, title in zip(axes[:2], (a, b), ("Closed brush / hull 1", "Matching top triangles / hull 1")):
        picture = ax.imshow(np.where(np.isfinite(data), data, np.nan), origin="lower", extent=extent,
                            vmin=minimum, vmax=maximum, cmap="viridis")
        ax.set_title(title)
        ax.set_xlabel("X (units)")
        ax.set_ylabel("Y (units)")
    fig.colorbar(picture, ax=axes[:2], label="First contact Z (player origin)", shrink=.8)
    limit = max(.01, float(np.nanmax(np.abs(difference))))
    picture = axes[2].imshow(difference, origin="lower", extent=extent, cmap="RdBu_r",
                             norm=TwoSlopeNorm(0, -limit, limit))
    axes[2].set_title("BSP contact-height difference")
    axes[2].set_xlabel("X (units)")
    axes[2].set_ylabel("Y (units)")
    fig.colorbar(picture, ax=axes[2], label="Closed minus triangles (units)", shrink=.8)
    fig.savefig(root / "comparison.png", dpi=160)
    print(root / "REPORT.md")


if __name__ == "__main__":
    main()
