"""Measure persistent top-height steps in an existing collision BSP.

Matching a per-brush expansion target does not establish surface continuity.
This diagnostic samples both sides of selected XY locations at shrinking
distances. Automatic candidates come from vertical BSP planes along X rows;
they are warnings to inspect, not proof that every detected step is unwanted.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from brush_seam_diagnose import vertices
from legacy_clip_diagnose import convex_hit, save_json, sha256
from source_clip_pipeline import HullFile
from source_clip_validate import structure, tree_first_hit


EPSILONS = np.array([.01, .001, .0001])


def pair_origins(xy, zhi, epsilons=EPSILONS):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    origins = np.empty((len(epsilons), len(xy), 2, 3))
    origins[..., :2] = xy[None, :, None, :]
    origins[..., 2] = zhi
    origins[..., 0] += np.asarray(epsilons)[:, None, None] * [-1., 1.]
    return origins


def persistent_steps(source, collision, threshold=.25, source_tolerance=.05):
    """Reject misses and continuous steep slopes that shrink with probe spacing."""
    finite = np.isfinite(source).all(axis=(0, 2)) & np.isfinite(collision).all(axis=(0, 2))
    safe_source = np.where(np.isfinite(source), source, 0.)
    safe_collision = np.where(np.isfinite(collision), collision, 0.)
    source_delta = abs(safe_source[..., 1] - safe_source[..., 0])
    collision_delta = abs(safe_collision[..., 1] - safe_collision[..., 0])
    return (finite & (source_delta[-1] < source_tolerance)
            & (collision_delta.min(axis=0) > threshold)
            & (collision_delta[-1] >= .8 * collision_delta[0]))


def scan_candidates(tree, hull, rows, xbounds):
    root = int(tree.models[0]["head"][hull])
    seen, pending = set(), [root]
    while pending:
        node = pending.pop()
        if node < 0 or node in seen:
            continue
        seen.add(node)
        pending.extend(map(int, tree.nodes[node, 1:]))
    plane_ids = sorted({int(tree.nodes[node, 0]) for node in seen})
    planes = tree.planes[plane_ids, :4]
    planes = planes[(abs(planes[:, 2]) < 1e-8) & (abs(planes[:, 0]) > 1e-8)]
    xy = []
    for y in rows:
        xs = (planes[:, 3] - planes[:, 1] * y) / planes[:, 0]
        # Do not round candidate coordinates: they locate actual BSP boundaries.
        xs = np.unique(xs[(xs > xbounds[0]) & (xs < xbounds[1])])
        xy.extend((float(x), float(y)) for x in xs)
    return xy


def finite_list(values):
    return np.where(np.isfinite(values), values, None).tolist()


def plot_profiles(root, xy, source, legacy, target, tree, hull, zhi, length):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not len(xy):
        return
    fig, axes = plt.subplots(len(xy), 1, figsize=(9, 3 * len(xy)),
                             squeeze=False, constrained_layout=True)
    for ax, (x, y) in zip(axes[:, 0], xy):
        offsets = np.linspace(-.5, .5, 501)
        origins = np.c_[x + offsets, np.full(len(offsets), y), np.full(len(offsets), zhi)]
        directions = np.tile([0., 0., -1.], (len(offsets), 1))
        for volumes, label, color in ((source, "Source surface", "#666666"),
                                       (legacy, "Legacy union target", "#ce6938")):
            tops = zhi - convex_hit(origins, directions, length, volumes)
            ax.plot(offsets, np.where(np.isfinite(tops), tops, np.nan), label=label, color=color)
        tops = zhi - tree_first_hit(tree, origins, directions, length, hull)
        ax.plot(offsets, np.where(np.isfinite(tops), tops, np.nan),
                label="Actual support BSP", color="#187c91", linewidth=2)
        if target is not None:
            tops = zhi - convex_hit(origins, directions, length, target)
            ax.plot(offsets, np.where(np.isfinite(tops), tops, np.nan),
                    label="Support target", color="#943d8e", linestyle="--")
        ax.axvline(0., color="#888888", linestyle=":")
        ax.set(xlabel="X offset (map units)", ylabel="Top Z",
               title=f"Hull {hull}: X={x:.9f}, Y={y:g}")
        ax.grid(alpha=.2)
        ax.legend(loc="best", fontsize=8)
    fig.savefig(root / "seam_profiles.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("geometry", "bsp", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--target", type=Path, help="Optional expanded convex-volume JSON")
    parser.add_argument("--entity", type=int, required=True)
    parser.add_argument("--hull", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--probe", type=float, nargs=2, action="append", default=[])
    parser.add_argument("--scan-y", type=float, nargs="*", default=[])
    parser.add_argument("--fail-on-step", action="store_true", help="Nonzero exit on a flagged candidate")
    args = parser.parse_args()
    if not args.probe and not args.scan_y:
        parser.error("Provide --probe XY or --scan-y rows")
    geometry = json.loads(args.geometry.read_text())
    brushes = [b for b in geometry["brushes"] if b["entity"] == args.entity]
    if not brushes or any(b["contents"] != -2 for b in brushes):
        raise ValueError("Select an entity of ordinary solid brushes")
    source = [{"planes": np.array([[*f["normal"], f["dist"]] for f in b["faces"]])} for b in brushes]
    legacy = [{"planes": np.array(b["legacy_hulls"][args.hull - 1]["planes"])} for b in brushes]
    points = np.concatenate([vertices(b) for b in brushes])
    expanded = np.concatenate([b["legacy_hulls"][args.hull - 1]["vertices"] for b in brushes])
    zhi, zlo = float(expanded[:, 2].max() + 32), float(expanded[:, 2].min() - 32)
    tree = HullFile(args.bsp)
    checked = structure(tree)
    xy = args.probe + scan_candidates(tree, args.hull, args.scan_y,
                                      [points[:, 0].min(), points[:, 0].max()])
    xy = np.unique(np.asarray(xy).reshape(-1, 2), axis=0)
    shaped = pair_origins(xy, zhi)
    origins = shaped.reshape(-1, 3)
    directions = np.tile([0., 0., -1.], (len(origins), 1))
    shape = shaped.shape[:-1]
    samples = {"source": zhi - convex_hit(origins, directions, zhi - zlo, source),
               "legacy_union": zhi - convex_hit(origins, directions, zhi - zlo, legacy),
               "bsp": zhi - tree_first_hit(tree, origins, directions, zhi - zlo, args.hull)}
    target = None
    if args.target:
        target = [{k: np.array(v) for k, v in t.items()} for t in json.loads(args.target.read_text())]
        samples["target"] = zhi - convex_hit(origins, directions, zhi - zlo, target)
    samples = {key: value.reshape(shape) for key, value in samples.items()}
    flagged = persistent_steps(samples["source"], samples["bsp"])
    witnesses = []
    for i in np.flatnonzero(flagged):
        tops = {key: finite_list(value[:, i]) for key, value in samples.items()}
        witnesses.append({"xy": xy[i].tolist(), "top_pairs_by_epsilon": tops,
                          "smallest_spacing_bsp_step": float(abs(samples["bsp"][-1, i, 1] - samples["bsp"][-1, i, 0]))})
    witnesses.sort(key=lambda w: w["smallest_spacing_bsp_step"], reverse=True)
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    report = {"geometry": str(args.geometry.resolve()), "geometry_sha256": sha256(args.geometry),
              "bsp": str(args.bsp.resolve()), "bsp_sha256": sha256(args.bsp),
              "runner_sha256": sha256(__file__), "entity": args.entity, "hull": args.hull,
              "target_sha256": sha256(args.target) if args.target else None,
              "structure": checked, "epsilons": EPSILONS.tolist(), "ray_z_range": [zlo, zhi],
              "candidate_locations": len(xy), "flagged_locations": int(flagged.sum()),
              "scope": "Finite samples only; nearby real cliffs and other geometry need review. "
                       "No guarantee of exhaustive seam detection or engine movement behavior.",
              "witnesses": witnesses}
    save_json(root / "results.json", report)
    np.savez_compressed(root / "pairs.npz", xy=xy, epsilons=EPSILONS, flagged=flagged, **samples)
    chosen = np.asarray(args.probe if args.probe else [w["xy"] for w in witnesses[:3]]).reshape(-1, 2)
    plot_profiles(root, chosen, source, legacy, target, tree, args.hull, zhi, zhi - zlo)
    print(json.dumps({"candidate_locations": len(xy), "flagged_locations": int(flagged.sum()),
                      "output": str(root)}, indent=2))
    if args.fail_on_step and flagged.any():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
