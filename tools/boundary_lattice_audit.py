"""Audit persistent steps and their exposed source-wall provenance.

Source equivalence and compiled-target equivalence do not prove smoothness.
The finite axis scans distinguish substantial source jumps, very thin exposed
walls, and wall-plane extensions needing further inspection.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from brush_exterior_pipeline import extract_exterior
from brush_surface_pipeline import HALF, Tree, ray_stats
from concave_support_diagnose import shift_legacy
from legacy_clip_diagnose import convex_hit, save_json, sha256
from surface_seam_audit import EPSILONS


def persistent_jumps(values, allow_outline=False):
    finite = np.isfinite(values)
    delta = abs(np.where(finite, values, 0.)[..., 1] - np.where(finite, values, 0.)[..., 0])
    jumps = (finite.all(axis=(0, 2)) & (delta.min(0) > .25)
             & (delta[-1] >= .8 * delta[0]))
    if allow_outline:
        jumps |= ((finite[..., 0] & ~finite[..., 1]).all(0)
                  | (~finite[..., 0] & finite[..., 1]).all(0))
    return jumps


def wall_height(triangles, axis, row):
    heights = []
    for tri in triangles:
        cuts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            low, high = tri[a], tri[b]
            if min(low[axis], high[axis]) - 1e-7 <= row <= max(low[axis], high[axis]) + 1e-7:
                if abs(high[axis] - low[axis]) < 1e-8:
                    cuts.extend([low[2], high[2]])
                else:
                    cuts.append(low[2] + (high[2] - low[2]) * (row - low[axis]) / (high[axis] - low[axis]))
        if cuts:
            heights.append(max(cuts) - min(cuts))
    return float(max(heights)) if heights else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hull", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--rows", type=float, nargs="+", required=True)
    parser.add_argument("--tree", type=Path, help="Optional compiled terrain NPZ, audited instead of target")
    parser.add_argument("--candidate-geometry", type=Path,
                        help="Replay boundary-plane crossings from a previous geometry, including removed planes")
    args = parser.parse_args()
    base, root = args.geometry.resolve(), args.output.resolve()
    summary = json.loads((base / "results.json").read_text())
    record = json.loads((base / "construction.json").read_text())
    if not summary.get("whole_entity") or not record.get("source_coverage_checked"):
        raise ValueError("Select a completed whole-entity boundary composition")
    if sha256(base / "geometry.json") != summary["inputs"]["geometry.json"]:
        raise ValueError("Source geometry changed since composition")
    geometry = json.loads((base / "geometry.json").read_text())
    triangles, normals, source, _ = extract_exterior(geometry, summary["entity"])
    target_path = base / f"patch0_h{args.hull}.json"
    target = [{k: np.array(v) for k, v in t.items()} for t in json.loads(target_path.read_text())]
    candidates = args.candidate_geometry.resolve() if args.candidate_geometry else base
    previous = json.loads((candidates / "results.json").read_text())
    if (previous["entity"] != summary["entity"] or previous["inputs"] != summary["inputs"]
            or not previous.get("whole_entity")):
        raise ValueError("Candidate geometry must use the same whole-entity source inputs")
    candidate_record = json.loads((candidates / "construction.json").read_text())
    planes = np.array(candidate_record["planes"])
    shifted = shift_legacy(planes, HALF[args.hull])
    tree = Tree.load(args.tree) if args.tree else None
    low = np.min([v["bounds"][0] for v in target], axis=0)
    high = np.max([v["bounds"][1] for v in target], axis=0)
    scan_low, scan_high = low, high
    if candidates != base:
        old_target = json.loads((candidates / f"patch0_h{args.hull}.json").read_text())
        scan_low = np.min([v["bounds"][0] for v in old_target], axis=0)
        scan_high = np.max([v["bounds"][1] for v in old_target], axis=0)
    xy, source_xy, axes, plane_indices = [], [], [], []
    for axis in (0, 1):
        other = 1 - axis
        for row in args.rows:
            for index, p in enumerate(shifted):
                if abs(p[2]) > 1e-8 or abs(p[axis]) < 1e-8:
                    continue
                coordinate = (p[3] - p[other] * row) / p[axis]
                if not scan_low[axis] < coordinate < scan_high[axis]:
                    continue
                point = np.zeros(2)
                point[axis], point[other] = coordinate, row
                old = point.copy()
                old[axis] = (planes[index, 3] - planes[index, other] * row) / planes[index, axis]
                xy.append(point)
                source_xy.append(old)
                axes.append(axis)
                plane_indices.append(index)
    if not xy:
        raise ValueError("No vertical boundary-plane crossings in the selected rows")
    xy, source_xy = np.array(xy), np.array(source_xy)
    origins = np.zeros((len(EPSILONS), len(xy), 2, 3))
    origins[..., :2] = xy[None, :, None, :]
    origins[..., 2] = high[2] + 16
    old = origins.copy()
    old[..., :2] = source_xy[None, :, None, :]
    for i, axis in enumerate(axes):
        origins[:, i, :, axis] += EPSILONS[:, None] * [-1., 1.]
        old[:, i, :, axis] += EPSILONS[:, None] * [-1., 1.]
    length = float(high[2] - low[2] + 32)

    def tops(points, volumes):
        o = points.reshape(-1, 3)
        d = np.tile([0., 0., -1.], (len(o), 1))
        return (high[2] + 16 - convex_hit(o, d, length, volumes)).reshape(points.shape[:-1])

    target_tops = tops(origins, target)
    o = origins.reshape(-1, 3)
    d = np.tile([0., 0., -1.], (len(o), 1))
    collision = ((high[2] + 16 - tree.hit(o, d, length)).reshape(origins.shape[:-1])
                 if tree else target_tops)
    original_wall = tops(old, source)
    detected = persistent_jumps(collision)
    confirmed = persistent_jumps(original_wall, allow_outline=True)
    witnesses = []
    for i in np.flatnonzero(detected):
        p = planes[plane_indices[i]]
        matching = ((np.max(abs(normals - p[:3]), axis=1) < 1e-7)
                    & (abs(np.einsum("ij,ij->i", triangles[:, 0], normals) - p[3]) < 1e-6))
        wall_triangles = triangles[matching]
        row_axis = 1 - axes[i]
        witnesses.append({"xy": xy[i].tolist(), "axis": axes[i], "source_xy": source_xy[i].tolist(),
            "plane": p.tolist(), "delta": float(abs(collision[-1, i, 1] - collision[-1, i, 0])),
            "source_wall_delta": (float(abs(original_wall[-1, i, 1] - original_wall[-1, i, 0]))
                                  if np.isfinite(original_wall[:, i]).all() else None),
            "source_wall_confirmed": bool(confirmed[i]), "matching_wall_triangles": len(wall_triangles),
            "source_wall_local_height": wall_height(wall_triangles, row_axis, source_xy[i, row_axis])})
    report = {"scope": "Finite axis scans, not an exhaustive smoothness proof. Unconfirmed steps require inspection.",
        "geometry": str(base), "candidate_geometry": str(candidates),
        "candidate_construction_sha256": sha256(candidates / "construction.json"),
        "target_sha256": sha256(target_path), "hull": args.hull, "rows": args.rows,
        "tree": str(args.tree.resolve()) if tree else None, "tree_sha256": sha256(args.tree) if tree else None,
        "runner_sha256": sha256(__file__), "candidate_locations": len(xy),
        "detected_steps": int(detected.sum()), "source_wall_confirmed_steps": int((detected & confirmed).sum()),
        "unconfirmed_steps": int((detected & ~confirmed).sum()),
        "steps_at_sub_0_001_walls": sum(w["source_wall_local_height"] is not None
                                        and w["source_wall_local_height"] < .001 for w in witnesses),
        "sampled_tree_vs_target": ray_stats(collision.ravel(), target_tops.ravel()),
        "witnesses": sorted(witnesses, key=lambda w: -w["delta"])}
    root.mkdir(parents=True, exist_ok=False)
    save_json(root / "results.json", report)
    np.savez_compressed(root / "pairs.npz", xy=xy, source_xy=source_xy, axes=axes,
                        collision=collision, target=target_tops, original_wall=original_wall,
                        detected=detected, confirmed=confirmed)
    print(json.dumps({k: v for k, v in report.items() if k != "witnesses"}, indent=2))


if __name__ == "__main__":
    main()
