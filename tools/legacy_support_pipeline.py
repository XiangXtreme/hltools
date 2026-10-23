"""Apply the shared legacy support experiment to one solid func_detail entity."""

import argparse
import json
from pathlib import Path
import shutil
import time

import numpy as np

from brush_seam_diagnose import vertices
from brush_surface_pipeline import (HALF, load_module, repair, require_command,
                                    splice, validate, ray_stats)
from concave_support_diagnose import (collision_carrier, intersection,
                                      shift_legacy, support_planes)
from legacy_clip_diagnose import convex_hit, save_json, sha256
from legacy_union_pipeline import compile_volumes, override_cliptype
from remove_map_group_volume import parse_map
from source_clip_pipeline import HullFile
from source_clip_validate import structure


def clipped_volumes(brushes, supports, hull):
    caps = shift_legacy(supports, HALF[hull])
    target, changed = [], []
    for brush in brushes:
        exported = brush["legacy_hulls"][hull - 1]
        planes = np.array(exported["planes"])
        points = np.unique(np.array(exported["vertices"]), axis=0)
        if not len(planes) or len(points) < 4:
            raise ValueError("Expected an ordinary expanded solid brush")
        violated = np.max(points @ caps[:, :3].T - caps[:, 3], axis=0) > 1e-7
        if violated.any():
            volume = intersection(np.r_[planes, caps[violated]])
            changed.append(brush["brush"])
        else:
            volume = {"planes": planes, "vertices": points,
                      "bounds": np.array([points.min(0), points.max(0)])}
        source = vertices(brush)
        if np.max(source @ volume["planes"][:, :3].T - volume["planes"][:, 3]) > 1e-6:
            raise ValueError("A support cut removed original source geometry")
        target.append(volume)
    return target, changed


def acceptable(record):
    return not (record["extra_solid"] or record["missing_solid"]
                or record["exterior_extra_solid"]
                or record["ray_checks"]["hit_miss_differences"]
                or record["ray_checks"]["errors_over_0_01"])


def contact_grid(brushes):
    points = np.concatenate([vertices(b) for b in brushes])
    low, high = points.min(0), points.max(0)
    xs = np.linspace(low[0] - 33 + .173205, high[0] + 33 - .271828, 257)
    ys = np.linspace(low[1] - 33 + .271828, high[1] + 33 - .173205, 257)
    xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
    zhi, zlo = float(high[2] + 100), float(low[2] - 100)
    origins = np.c_[xy, np.full(len(xy), zhi)]
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    return xs, ys, origins, directions, zhi, zhi - zlo


def report_file(root, report):
    lines = ["# Shared Legacy Support Entity Build", "",
             f"Selected entity: {report['manifest']['entity']}. "
             f"Source brushes: {report['source_brushes']}. Shared support planes: {len(report['support_planes'])}.",
             "The compile copy overrides local cliptype settings to legacy. Source MAP is unchanged.", "",
             "| Hull | Changed brushes | Final reachable nodes | Max sampled top trim | Max top BSP error |",
             "| --- | ---: | ---: | ---: | ---: |"]
    for hull, value in report["hulls"].items():
        trim = value["top_contact"]["max_trim"]
        error = value["top_contact"]["bsp_vs_target"]["max_distance_error"]
        count = report["output"]["structure"]["hulls"][hull]["reachable"]
        lines.append(f"| {hull} | {len(value['changed_brushes'])} | {count} | {trim:.6f} | {error:.6f} |")
    lines += ["", f"Stored clipnodes: {report['output']['structure']['stored_nodes']}.",
              f"Wall seconds including analysis, compilation and validation: {report['wall_seconds']:.3f}.", "",
              "Already-expanded legacy brushes are intersected with legacy-shifted global source supports.",
              "The source volume is retained and the real concavities are not filled. No player-box",
              "expansion is applied to the selected vertices. Each source brush is contained by its",
              "own clipped convex group, as proved by vertex-to-plane containment.", "",
              "Raw CSG/BSP classification is checked against the target half-spaces. If needed,",
              "misclassified leaf regions are rebuilt with direct half-space unions. Holdout limits",
              "are 0.01 map units, with 250,000 points, 8,192 random rays and an independent",
              "257x257 vertical grid per hull. This is sampled geometry validation, not an engine test.", "",
              "All noncollision data matches the native lit legacy build byte for byte. Player starts,",
              "graph indices, cycles, hull root lower bounds and the clipnode budget are checked.", "",
              "Global supports do not propagate roof planes across concave valleys. Most interior",
              "seams can remain unchanged. This build uses the same global rule as the small",
              "cliptype_sl experiment, without introducing a local neighborhood heuristic.", "",
              f"Output: `{Path(report['output']['path']).name}`. Source and existing game BSP are untouched."]
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "compiler", "dumper", "scripts", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--entity", type=int, required=True)
    args = parser.parse_args()
    for key in ("source", "compiler", "dumper", "scripts", "output"):
        setattr(args, key, getattr(args, key).resolve())
    text = args.source.read_text(encoding="utf-8")
    document = parse_map(text)
    if not 0 <= args.entity < len(document.entities) or document.entities[args.entity].classname != "func_detail":
        raise ValueError("Select one solid func_detail terrain entity")
    if any(e.classname in ("info_compile_parameters", "info_hullshape") for e in document.entities):
        raise ValueError("Persistent compiler/hull configuration needs explicit handling")
    started = time.perf_counter()
    root = args.output
    root.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.source, root / "source_snapshot.map")
    for key, name in (("compiler", "hltools.exe"), ("dumper", "brush_geometry_dump.exe")):
        shutil.copy2(getattr(args, key), root / name)
    for name in ("exacthull.py", "brushbsp.py"):
        shutil.copy2(args.scripts / name, root / name)
    shutil.copy2(__file__, root / "runner_snapshot.py")
    report = {"manifest": {"source": str(args.source), "source_sha256": sha256(args.source),
                           "entity": args.entity, "runner_sha256": sha256(__file__),
                           "dependencies": {name: sha256(root / name) for name in
                               ("hltools.exe", "brush_geometry_dump.exe", "exacthull.py", "brushbsp.py")}},
              "commands": {}, "hulls": {}}
    compiler, dumper = root / "hltools.exe", root / "brush_geometry_dump.exe"
    source_copy = root / "legacy.map"
    source_copy.write_text(override_cliptype(text, "legacy"), encoding="utf-8")
    report["commands"]["geometry"] = require_command(
        [dumper, source_copy, root / "geometry.json", "--legacy-hulls"], root / "geometry.txt")
    geometry = json.loads((root / "geometry.json").read_text())
    brushes = [b for b in geometry["brushes"] if b["entity"] == args.entity]
    if len(brushes) != len(document.entities[args.entity].brushes) or any(b["contents"] != -2 for b in brushes):
        raise ValueError("Selected entity must consist entirely of ordinary solid brushes")
    supports = support_planes(brushes)
    report["source_brushes"], report["support_planes"] = len(brushes), supports.tolist()
    print(f"Selected {len(brushes)} terrain brushes; {len(supports)} global support planes", flush=True)
    save_json(root / "results.json", report)
    report["commands"]["legacy_compile"] = require_command(
        [compiler, "compile", source_copy, "-cliptype", "legacy", "-threads", "2"], root / "legacy.compile.txt")
    visual = HullFile(source_copy.with_suffix(".bsp"))
    report["legacy_structure"] = structure(visual)
    report["commands"]["background_csg"] = require_command(
        [dumper, source_copy, root / "background_geometry.json", "--background", args.entity, root / "background"],
        root / "background.csg.txt")
    for stage in ("bsp", "vis", "rad"):
        report["commands"]["background_" + stage] = require_command(
            [compiler, stage, root / "background.bsp", "-threads", "2"], root / f"background.{stage}.txt")
    exact = load_module("entity_support_exacthull", root / "exacthull.py")
    exact.WAD = document.entities[0].keyvalues["wad"].replace("\\", "/")
    exact.fmt = lambda p: "( " + " ".join(f"{value:.12f}" for value in p) + " )"
    bb = load_module("entity_support_brushbsp", root / "brushbsp.py")
    bb.EPS = 1e-5
    trees = {}
    xs, ys, origins, directions, zhi, length = contact_grid(brushes)
    for h in (1, 2, 3):
        target, changed = clipped_volumes(brushes, supports, h)
        print(f"Hull {h}: {len(changed)} changed brushes; compiling collision", flush=True)
        tree, seconds = compile_volumes(root / f"support_h{h}.map", target, exact, compiler)
        record = {"changed_brushes": changed, "compile_seconds": seconds,
                  "raw_nodes": len(tree.nodes), "before": validate(tree, target),
                  "source_containment": True}
        print(f"Hull {h} raw checks: {json.dumps(record['before'])}", flush=True)
        if not acceptable(record["before"]):
            record["repair"] = repair(tree, target, bb, tolerance=1e-4,
                                       extra_rays=(origins, directions, length), local_builder="planes")
        tree.compact()
        record["holdout"] = validate(tree, target)
        got, wanted = tree.hit(origins, directions, length), convex_hit(origins, directions, length, target)
        top_stats = ray_stats(got, wanted)
        if not acceptable(record["holdout"]) or top_stats["hit_miss_differences"] or top_stats["errors_over_0_01"]:
            record["top_errors"] = top_stats
            save_json(root / f"h{h}.failed.json", record)
            raise ValueError(f"Hull {h} did not pass independent collision validation")
        original = [{"planes": np.array(b["legacy_hulls"][h - 1]["planes"])} for b in brushes]
        old = convex_hit(origins, directions, length, original)
        both = np.isfinite(old) & np.isfinite(wanted)
        delta = np.where(both, np.where(both, wanted, 0) - np.where(both, old, 0), -np.inf)
        index = int(np.argmax(delta))
        record["top_contact"] = {"rays": len(origins), "bsp_vs_target": top_stats,
            "changed_samples": int(np.sum(delta > .01)), "max_trim": float(delta[index]),
            "witness_xy": origins[index, :2].tolist(), "original_z": float(zhi - old[index]),
            "support_z": float(zhi - wanted[index])}
        np.savez_compressed(root / f"tops_h{h}.npz", x=xs, y=ys,
                            original=(zhi - old).reshape(len(ys), len(xs)),
                            support=(zhi - wanted).reshape(len(ys), len(xs)),
                            bsp=(zhi - got).reshape(len(ys), len(xs)))
        tree.save(root / f"support_h{h}.npz")
        save_json(root / f"target_h{h}.json", [{k: v.tolist() for k, v in t.items()} for t in target])
        trees[h] = tree
        report["hulls"][str(h)] = record
        save_json(root / "results.json", report)
        print(f"Hull {h} validated: {len(tree.nodes)} terrain nodes, max trim {delta[index]:.6f}", flush=True)
    carrier = root / "collision_carrier.bsp"
    collision_carrier(source_copy.with_suffix(".bsp"), root / "background.bsp", carrier)
    data, checked = splice(carrier, trees)
    final = HullFile(data=data)
    if not final.lumps[4] or not any(final.lumps[8]):
        raise ValueError("Missing VIS or lighting")
    for index in range(15):
        if index not in (1, 9, 14) and final.lumps[index] != visual.lumps[index]:
            raise ValueError("Noncollision data changed")
    path = root / (args.source.stem + "_surface_support.bsp")
    path.write_bytes(data)
    report["output"] = {"path": str(path), "sha256": sha256(path), "structure": checked,
                        "light_bytes": len(final.lumps[8]), "vis_bytes": len(final.lumps[4]),
                        "visual_lumps_match": True}
    report["wall_seconds"] = time.perf_counter() - started
    if sha256(args.source) != report["manifest"]["source_sha256"]:
        raise ValueError("Source MAP changed during compilation")
    save_json(root / "results.json", report)
    report_file(root, report)
    print(json.dumps(report["output"], indent=2), flush=True)


if __name__ == "__main__":
    main()
