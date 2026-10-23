"""Compare overlapping full-boundary crops and plot their source provenance."""

import argparse
import json
from pathlib import Path

import numpy as np

from brush_surface_pipeline import HALF, Tree, ray_stats
from legacy_clip_diagnose import convex_hit, margin, save_json, sha256
from surface_seam_audit import EPSILONS, pair_origins


def load_target(path):
    return [{k: np.array(v) for k, v in item.items()} for item in json.loads(path.read_text())]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("small", "large", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    args = parser.parse_args()
    roots = [args.small.resolve(), args.large.resolve()]
    reports = [json.loads((r / "results.json").read_text()) for r in roots]
    if reports[0]["inputs"] != reports[1]["inputs"] or reports[0]["entity"] != reports[1]["entity"]:
        raise ValueError("Overlap comparison requires matching source inputs")
    if [p["xy"] for p in reports[0]["patches"]] != [p["xy"] for p in reports[1]["patches"]]:
        raise ValueError("Match crop centers and ordering")
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    geometry = json.loads((roots[0] / "geometry.json").read_text())
    brushes = [b for b in geometry["brushes"] if b["entity"] == reports[0]["entity"]]
    source = [{"planes": np.array([[*f["normal"], f["dist"]] for f in b["faces"]])} for b in brushes]
    legacy = [{"planes": np.array(b["legacy_hulls"][0]["planes"])} for b in brushes]
    report = {"small": str(roots[0]), "large": str(roots[1]), "runner_sha256": sha256(__file__),
              "scope": "Sampled overlap comparison; not a proof of all region connections.", "patches": []}
    profiles = []
    rng = np.random.default_rng(20261023)
    for index, patch in enumerate(reports[0]["patches"]):
        bounds = np.array(patch["bounds"])
        zlo, zhi = bounds[0, 2] + 1, bounds[1, 2] - 1
        x, y = patch["xy"]
        record = {"xy": patch["xy"], "hulls": {}}
        for hull in (1, 2, 3):
            targets = [load_target(r / f"patch{index}_h{hull}.json") for r in roots]
            trees = [Tree.load(r / f"patch{index}_h{hull}.npz") for r in roots]
            comparisons = {}
            for name, factor in (("core", .5), ("full_crop", 1.)):
                lo = np.array([x, y]) + (bounds[0, :2] - [x, y]) * factor + .001
                hi = np.array([x, y]) + (bounds[1, :2] - [x, y]) * factor - .001
                xy = np.stack(np.meshgrid(np.linspace(lo[0], hi[0], 129), np.linspace(lo[1], hi[1], 129)), -1).reshape(-1, 2)
                o = np.c_[xy, np.full(len(xy), zhi)]
                d = np.tile([0., 0., -1.], (len(o), 1))
                wanted = [convex_hit(o, d, zhi - zlo, target) for target in targets]
                got = [tree.hit(o, d, zhi - zlo) for tree in trees]
                points = rng.uniform([*lo, zlo], [*hi, zhi], (50000, 3))
                distances = [margin(points, target) for target in targets]
                away = (abs(distances[0]) > 1e-4) & (abs(distances[1]) > 1e-4)
                comparisons[name] = {"geometry_tops": ray_stats(*wanted), "bsp_tops": ray_stats(*got),
                    "interior_point_mismatches": int(np.sum(away & ((distances[0] < 0) != (distances[1] < 0))))}
            record["hulls"][str(hull)] = comparisons
            if hull == 1:
                px = np.linspace(x - 40, x + 40, 1601)
                o = np.c_[px, np.full(len(px), y), np.full(len(px), zhi)]
                d = np.tile([0., 0., -1.], (len(o), 1))
                profiles.append({"xy": [x, y], "x": px,
                    "source": zhi - convex_hit(o, d, zhi - zlo, source),
                    "legacy": zhi - convex_hit(o, d, zhi - zlo, legacy),
                    "boundary": zhi - trees[1].hit(o, d, zhi - zlo)})
                planes = np.array(patch["planes"])
                walls = planes[(abs(planes[:, 2]) < 1e-8) & (abs(planes[:, 0]) > 1e-6)]
                if len(walls):
                    source_x = (walls[:, 3] - walls[:, 1] * y) / walls[:, 0]
                    shifted_x = source_x + walls[:, :3] ** 2 @ HALF[1] / walls[:, 0]
                    nearest = int(np.argmin(abs(shifted_x - x)))
                    if abs(shifted_x[nearest] - x) < .001:
                        pairs = pair_origins([[source_x[nearest], y]], zhi).reshape(-1, 3)
                        dirs = np.tile([0., 0., -1.], (len(pairs), 1))
                        tops = (zhi - convex_hit(pairs, dirs, zhi - zlo, source)).reshape(3, 2)
                        record["shifted_wall"] = {"plane": walls[nearest].tolist(),
                            "source_x": float(source_x[nearest]), "hull1_x": float(shifted_x[nearest]),
                            "delta_x": float(shifted_x[nearest] - source_x[nearest]),
                            "source_top_pairs": tops.tolist(), "epsilons": EPSILONS.tolist()}
                        profiles[-1]["wall_source_x"] = source_x[nearest]
        report["patches"].append(record)
    save_json(root / "results.json", report)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(profiles), 1, figsize=(10, 3.5 * len(profiles)), squeeze=False, constrained_layout=True)
    for ax, data in zip(axes[:, 0], profiles):
        for key, label, color in (("source", "Source surface", "#666666"),
                                  ("legacy", "Per-brush legacy", "#b95137"),
                                  ("boundary", "Full-boundary offset", "#187c91")):
            ax.plot(data["x"], data[key], label=label, color=color, linewidth=2)
        ax.axvline(data["xy"][0], color="#333333", linestyle=":")
        if "wall_source_x" in data:
            ax.axvline(data["wall_source_x"], color="#777777", linestyle="--", label="Original source wall")
        ax.set(xlabel="X (map units)", ylabel="Top Z", title=f"Hull 1, Y={data['xy'][1]:g}")
        ax.grid(alpha=.2)
        ax.legend(loc="best", fontsize=8)
    fig.savefig(root / "profiles.png", dpi=150)
    plt.close(fig)
    print(f"Overlap comparison: {root}")


if __name__ == "__main__":
    main()
