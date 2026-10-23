"""Validate opt-in legacy clip merging against the controlled seam baseline."""

import argparse
import json
from pathlib import Path
import shutil
import struct

import numpy as np

from brush_seam_diagnose import ray_errors, require_match, volumes
from brush_surface_pipeline import require_command
from legacy_clip_diagnose import convex_hit, error_counts, margin, save_json, sha256
from source_clip_pipeline import HullFile
from source_clip_validate import structure, tree_first_hit
from remove_map_group_volume import parse_map


def entities(tree):
    document = parse_map(tree.lumps[0].rstrip(b"\0").decode("latin-1"))
    result = [dict(entity.keyvalues) for entity in document.entities]
    for key in ("compiler", "compiled_at"):
        result[0].pop(key, None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("baseline", "compiler", "output"):
        parser.add_argument("--" + key, required=True, type=Path)
    args = parser.parse_args()
    baseline, compiler, root = [getattr(args, key).resolve() for key in ("baseline", "compiler", "output")]
    json.loads((baseline / "results.json").read_text())
    root.mkdir(parents=True, exist_ok=False)
    shutil.copy2(compiler, root / "hltools.exe")
    shutil.copy2(__file__, root / "runner_snapshot.py")
    compiler = root / "hltools.exe"
    manifest = {"baseline": str(baseline), "baseline_results_sha256": sha256(baseline / "results.json"),
                "source_sha256": sha256(baseline / "source_snapshot.map"),
                "compiler_sha256": sha256(compiler), "runner_sha256": sha256(__file__)}
    save_json(root / "manifest.json", manifest)
    report = {"manifest": manifest, "builds": {}, "hulls": {}}
    files = {}
    for name, input_name, mode, merge in (
        ("cliptype_sl_legacy", "original", "legacy", False),
        ("cliptype_sl_clipmerge", "original", "legacy", True),
        ("cliptype_sl_simple", "original", "simple", False),
        ("split_clipmerge", "split", "legacy", True),
        ("merged_clipmerge", "merged", "legacy", True),
    ):
        path = root / f"{name}.map"
        shutil.copy2(baseline / f"{input_name}.map", path)
        seconds = {"csg": require_command([compiler, "csg", path, "-cliptype", mode,
                                           *(["-clipmerge"] if merge else []), "-threads", "2"],
                                          root / f"{name}.csg.txt")}
        for stage in ("bsp", "vis", "rad"):
            seconds[stage] = require_command([compiler, stage, path.with_suffix(".bsp"), "-threads", "2"],
                                             root / f"{name}.{stage}.txt")
        tree = files[name] = HullFile(path.with_suffix(".bsp"))
        report["builds"][name] = {"seconds": seconds, "structure": structure(tree),
                                  "sha256": sha256(path.with_suffix(".bsp"))}
        if not tree.lumps[4] or not any(tree.lumps[8]):
            raise ValueError("Missing VIS or lighting")
        original = HullFile(baseline / f"{input_name}_{mode}.bsp")
        # BSP may reorder or remove collision planes; render plane references
        # and their equations must still remain identical.
        for lump in range(15):
            if lump not in (0, 1, 9, 14) and tree.lumps[lump] != original.lumps[lump]:
                raise ValueError(f"Visible data changed: {name}, lump {lump}")
        if entities(tree) != entities(original):
            raise ValueError("Map entity data changed beyond compiler identity/timestamp")
        render_planes = {struct.unpack_from("<i", tree.lumps[5], i)[0]
                         for i in range(0, len(tree.lumps[5]), 24)}
        render_planes.update(struct.unpack_from("<H", tree.lumps[7], i)[0]
                             for i in range(0, len(tree.lumps[7]), 20))
        for index in render_planes:
            if tree.lumps[1][20*index:20*(index+1)] != original.lumps[1][20*index:20*(index+1)]:
                raise ValueError("Render plane changed")
        a, b = bytearray(tree.lumps[14]), bytearray(original.lumps[14])
        a[40:52] = b[40:52]
        if a != b:
            raise ValueError("Model data other than collision roots changed")
        print(f"built {name}: {len(tree.nodes)} clipnodes", flush=True)
    merged_geometry = json.loads((baseline / "merged_geometry.json").read_text())
    original_geometry = json.loads((baseline / "original_geometry.json").read_text())
    # The left terrain is replaced by the independently authored merged brush.
    # The original right-hand terrain and six room brushes remain in the oracle.
    original_target = {"brushes": (original_geometry["brushes"][:6]
                                    + merged_geometry["brushes"][6:]
                                    + original_geometry["brushes"][8:])}
    rng = np.random.default_rng(20261024)
    points = rng.uniform([-159.73, -159.61, 4.37], [159.81, 159.89, 203.53], (100000, 3))
    ys = np.linspace(-100, 100, 4001)
    origins = np.c_[np.full(len(ys), -96.), ys, np.full(len(ys), 160.)]
    directions = np.tile([0., 0., -1.], (len(ys), 1))
    ro = rng.uniform([-143.73, -111.61, 130.37], [-48.81, 95.89, 159.53], (8192, 3))
    rt = rng.uniform([-127.73, -79.61, -16.37], [-64.81, 47.89, 79.53], (8192, 3))
    rd = rt - ro
    rd /= np.linalg.norm(rd, axis=1)[:, None]
    profile = {"y": ys}
    for h in (1, 2, 3):
        record = {}
        for name in ("cliptype_sl_clipmerge", "split_clipmerge", "merged_clipmerge"):
            target = volumes(original_target if name == "cliptype_sl_clipmerge" else merged_geometry, h, "legacy")
            actual = files[name]
            counts = error_counts(actual.contents(points, h) == -2, margin(points, target), .0001)
            if any(counts.values()):
                raise ValueError(f"Point mismatch in {name} hull {h}: {counts}")
            ray_record = {}
            for label, o, d, length in (("vertical", origins, directions, 180.), ("oblique", ro, rd, 240.)):
                measured = tree_first_hit(actual, o, d, length, h)
                errors = ray_errors(measured, convex_hit(o, d, length, target))
                require_match(errors)
                ray_record[label] = errors
                if label == "vertical":
                    profile[f"{name}_h{h}"] = 160 - measured
            if actual.contents([[-48., 112., 20.]], h if h != 2 else 1)[0] != -1:
                raise ValueError("Player start is blocked")
            record[name] = {"points": len(points), **counts, "rays": ray_record}
        a = profile[f"split_clipmerge_h{h}"]
        b = profile[f"merged_clipmerge_h{h}"]
        require_match(ray_errors(a, b))
        plain = files["cliptype_sl_legacy"]
        simple = files["cliptype_sl_simple"]
        profile[f"legacy_h{h}"] = 160 - tree_first_hit(plain, origins, directions, 180., h)
        profile[f"simple_h{h}"] = 160 - tree_first_hit(simple, origins, directions, 180., h)
        at_seam, on_slope = np.argmin(abs(ys + 32)), np.argmin(abs(ys))
        record["contacts"] = {"seam_y": float(ys[at_seam]), "seam_z": float(a[at_seam]),
                              "slope_y": float(ys[on_slope]), "slope_z": float(a[on_slope]),
                              "legacy_slope_z": float(profile[f"legacy_h{h}"][on_slope]),
                              "simple_slope_z": float(profile[f"simple_h{h}"][on_slope]),
                              "max_split_merged_difference": float(np.max(abs(a - b)))}
        report["hulls"][str(h)] = record
    save_json(root / "results.json", report)
    np.savez_compressed(root / "profile.npz", **profile)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    for name, label, color, style in (("legacy", "Original legacy", "#c44132", "-"),
                                       ("simple", "Simple", "#777777", ":"),
                                       ("split_clipmerge", "Legacy + clipmerge", "#16748b", "--")):
        ax.plot(ys, profile[f"{name}_h1"], label=label, color=color, linestyle=style, linewidth=2)
    ax.set(xlim=(-72, 8), ylim=(68, 122), xlabel="Y (map units)", ylabel="Hull 1 top Z (player centre)",
           title="Brush seam: continuous legacy collision after convex merging")
    ax.legend()
    ax.grid(alpha=.2)
    fig.savefig(root / "clipmerge_hull1.png", dpi=160)
    plt.close(fig)
    lines = ["# Legacy Clip Merge Validation", "", "All five BSPs include VIS and RAD.", "",
             "| BSP | Stored clipnodes | Hull 1 reachable | CSG+BSP+VIS+RAD seconds |",
             "| --- | ---: | ---: | ---: |"]
    for name, item in report["builds"].items():
        s = item["structure"]
        lines.append(f"| {name}.bsp | {s['stored_nodes']} | {s['hulls']['1']['reachable']} | {sum(item['seconds'].values()):.3f} |")
    lines += ["", "Merged split terrain matches the independently authored merged legacy target in all three hulls.",
        "The thin legacy slope is retained. Every non-collision geometry/lighting lump and referenced render plane",
              "matches its baseline byte for byte. Entity data matches except compiler identity/timestamp.",
              "Only collision roots change in model data.",
              "Full measurements, command logs, input snapshots and profile data are adjacent to this report.",
              "The original MAP and game installation are unchanged. No in-game movement test was performed.", "",
              "![Hull 1 contact profiles](clipmerge_hull1.png)"]
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({h: item["contacts"] for h, item in report["hulls"].items()}, indent=2), flush=True)


if __name__ == "__main__":
    main()
