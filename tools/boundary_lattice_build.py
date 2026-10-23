"""Compile a diagnostic BSP from a verified whole-entity boundary composition.

Keep the existing lit visual BSP and background collision. Checks cover source,
explicit continuity probes, target conformity and engine limits. Run the wider
boundary audit separately; target conformity does not establish smoothness.
The output is not automatically installed into the game.
"""

import argparse
import json
from pathlib import Path
import shutil
import time

import numpy as np

from brush_surface_pipeline import load_module, ray_stats, repair, splice, validate
from concave_support_diagnose import collision_carrier
from legacy_clip_diagnose import convex_hit, save_json, sha256
from legacy_union_pipeline import compile_volumes
from source_clip_pipeline import HullFile
from surface_seam_audit import pair_origins, persistent_steps


def acceptable(record):
    rays = record["ray_checks"]
    return not (record["extra_solid"] or record["missing_solid"] or record["exterior_extra_solid"]
                or rays["hit_miss_differences"] or rays["errors_over_0_01"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("geometry", "reference", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--continuous-probe", type=float, nargs=2, action="append", required=True)
    parser.add_argument("--probe", type=float, nargs=2, action="append", default=[])
    args = parser.parse_args()
    geometry_root, reference, root = [p.resolve() for p in (args.geometry, args.reference, args.output)]
    construction = json.loads((geometry_root / "results.json").read_text())
    baseline = json.loads((reference / "results.json").read_text())
    if (construction["method"] != "adaptive" or not construction.get("whole_entity")
            or len(construction["patches"]) != 1):
        raise ValueError("Select a completed adaptive whole-entity geometry build")
    patch = construction["patches"][0]
    if (not patch["source_coverage_checked"] or patch["term_excess_volume_sum"] > .001
            or patch["missing_source_volume_sum"] > .001 or patch["unshifted_point_mismatches"]):
        raise ValueError("Source composition did not pass its checks")
    for name, digest in construction["inputs"].items():
        if sha256(geometry_root / name) != digest or sha256(reference / name) != digest:
            raise ValueError(f"Geometry/reference input differs: {name}")
    source_path = Path(baseline["manifest"]["source"])
    source_hash = baseline["manifest"]["source_sha256"]
    if sha256(source_path) != source_hash:
        raise ValueError("Source MAP changed since the lit reference build")
    root.mkdir(parents=True, exist_ok=False)
    for name in ("hltools.exe", "exacthull.py", "brushbsp.py"):
        shutil.copy2(reference / name, root / name)
    shutil.copy2(__file__, root / "runner_snapshot.py")
    started = time.perf_counter()
    report = {"geometry_root": str(geometry_root), "reference": str(reference),
              "source_sha256": source_hash, "runner_sha256": sha256(__file__),
              "scope": "Diagnostic BSP; explicit seam probes pass, wider surface continuity needs a separate audit.",
              "source_checks": {key: patch[key] for key in (
                  "source_coverage_checked", "term_excess_volume_sum",
                  "missing_source_volume_sum", "unshifted_point_mismatches")}, "hulls": {}}
    source_geometry = json.loads((reference / "geometry.json").read_text())
    brushes = [b for b in source_geometry["brushes"] if b["entity"] == construction["entity"]]
    domain = np.array(patch["bounds"])
    source_points = np.array([p for b in brushes for f in b["faces"] for p in f["vertices"]])
    if (not brushes or np.any(source_points.min(0) <= domain[0] + .01)
            or np.any(source_points.max(0) >= domain[1] - .01)):
        raise ValueError("Composition domain does not contain the entire source entity")
    source = [{"planes": np.array([[*f["normal"], f["dist"]] for f in b["faces"]])}
              for b in brushes]
    exact = load_module("boundary_finish_exact", root / "exacthull.py")
    exact.WAD = ""
    exact.fmt = lambda p: "( " + " ".join(f"{v:.12f}" for v in p) + " )"
    bb = load_module("boundary_finish_brushbsp", root / "brushbsp.py")
    bb.EPS = 1e-5
    trees = {}
    for hull in (1, 2, 3):
        target_path = geometry_root / f"patch0_h{hull}.json"
        target = [{k: np.array(v) for k, v in t.items()} for t in json.loads(target_path.read_text())]
        points = np.concatenate([v["vertices"] for v in target])
        low, high = points.min(0), points.max(0)
        if np.any(low <= domain[0] + .01) or np.any(high >= domain[1] - .01):
            raise ValueError("A whole-entity target reaches artificial domain bounds")
        zhi, length = float(high[2] + 16), float(high[2] - low[2] + 32)
        xs = np.linspace(low[0] - 1., high[0] + 1., 257)
        ys = np.linspace(low[1] - 1., high[1] + 1., 257)
        xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
        origins = np.c_[xy, np.full(len(xy), zhi)]
        probes = args.continuous_probe + args.probe
        pairs = pair_origins(probes, zhi)
        o = pairs.reshape(-1, 3)
        d = np.tile([0., 0., -1.], (len(o), 1))
        source_tops = (zhi - convex_hit(o, d, length, source)).reshape(pairs.shape[:-1])
        target_tops = (zhi - convex_hit(o, d, length, target)).reshape(pairs.shape[:-1])
        record = {"target_sha256": sha256(target_path), "probe_xy": probes,
                  "source_top_pairs": source_tops.tolist(), "target_top_pairs": target_tops.tolist()}
        report["hulls"][str(hull)] = record
        save_json(root / "results.json", report)
        if (not np.isfinite(source_tops).all() or not np.isfinite(target_tops).all()
                or np.any(target_tops < source_tops - 1e-5)):
            raise ValueError("Whole-entity probe loses source collision")
        if persistent_steps(source_tops, target_tops)[:len(args.continuous_probe)].any():
            raise ValueError("Whole-entity composition regressed a continuous-source seam")
        origins = np.r_[origins, o]
        directions = np.tile([0., 0., -1.], (len(origins), 1))
        print(f"Hull {hull}: seam checks passed, compiling {len(target)} convex terms", flush=True)
        tree, seconds = compile_volumes(root / f"collision_h{hull}.map", target, exact, root / "hltools.exe")
        record.update({"raw_nodes": len(tree.nodes), "compile_seconds": seconds,
                       "before": validate(tree, target)})
        if not acceptable(record["before"]):
            record["repair"] = repair(tree, target, bb, tolerance=1e-4,
                extra_rays=(origins, directions, length), local_builder="planes")
        tree.compact()
        record["nodes"] = len(tree.nodes)
        record["holdout"] = validate(tree, target)
        actual_hits = tree.hit(origins, directions, length)
        record["bsp_top_pairs"] = (zhi - actual_hits[-len(o):]).reshape(pairs.shape[:-1]).tolist()
        record["top_grid"] = ray_stats(actual_hits,
                                        convex_hit(origins, directions, length, target))
        report["hulls"][str(hull)] = record
        save_json(root / "results.json", report)
        if (not acceptable(record["holdout"]) or record["top_grid"]["hit_miss_differences"]
                or record["top_grid"]["errors_over_0_01"]):
            raise ValueError("Compiled whole-entity hull did not match its target")
        tree.save(root / f"collision_h{hull}.npz")
        trees[hull] = tree
        print(f"Hull {hull}: {len(tree.nodes)} validated terrain nodes", flush=True)
    carrier = root / "collision_carrier.bsp"
    collision_carrier(reference / "legacy.bsp", reference / "background.bsp", carrier)
    data, checked = splice(carrier, trees)
    final, visual = HullFile(data=data), HullFile(reference / "legacy.bsp")
    if not final.lumps[4] or not any(final.lumps[8]):
        raise ValueError("Missing VIS or lighting")
    for index in range(15):
        if index not in (1, 9, 14) and final.lumps[index] != visual.lumps[index]:
            raise ValueError("Noncollision BSP data changed")
    if sha256(source_path) != source_hash:
        raise ValueError("Source MAP changed during compilation")
    path = root / (source_path.stem + "_boundary_lattice.bsp")
    path.write_bytes(data)
    report["output"] = {"path": str(path), "sha256": sha256(path), "structure": checked,
                         "light_bytes": len(final.lumps[8]), "vis_bytes": len(final.lumps[4])}
    report["wall_seconds"] = time.perf_counter() - started
    save_json(root / "results.json", report)
    print(json.dumps(report["output"], indent=2), flush=True)


if __name__ == "__main__":
    main()
