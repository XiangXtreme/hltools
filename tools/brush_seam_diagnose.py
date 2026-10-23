"""Measure brush-decomposition dependence in the user's cliptype_sl seam case."""

import argparse
import itertools
import json
from pathlib import Path
import re
import shutil

import numpy as np
from scipy.spatial import ConvexHull

from brush_surface_pipeline import HALF, Tree, expanded, require_command
from legacy_clip_diagnose import convex_hit, error_counts, margin, save_json, sha256
from remove_map_group_volume import POINT_RE, parse_map
from source_clip_pipeline import HullFile
from source_clip_validate import structure, tree_first_hit


def vertices(brush):
    points = np.concatenate([face["vertices"] for face in brush["faces"]])
    return np.unique(np.round(points, 8), axis=0)


def volumes(geometry, hull, mode):
    brushes = geometry["brushes"]
    if mode == "simple":
        return expanded([vertices(brush) for brush in brushes], HALF[hull],
                        np.array(list(itertools.product((-1., 1.), repeat=3))))
    return [{"planes": np.array(brush["legacy_hulls"][hull - 1]["planes"])}
            for brush in brushes]


def translate_block(block, offset):
    def point(match):
        return "( " + " ".join(f"{float(match[i + 1]) + offset[i]:.12g}"
                                for i in range(3)) + " )"
    return POINT_RE.sub(point, block)


def replace_terrain(document, block):
    terrain = document.entities[0].brushes[6:]
    return "".join(document.lines[:terrain[0].comment_start]) + block + "\n" + "".join(
        document.lines[terrain[-1].end + 1:])


def ray_errors(actual, expected):
    finite = np.isfinite(actual) & np.isfinite(expected)
    delta = np.abs(actual[finite] - expected[finite])
    return {"rays": len(actual),
            "hit_miss_differences": int(np.sum(np.isfinite(actual) != np.isfinite(expected))),
            "errors_over_0_001": int(np.sum(delta > .001)),
            "max_distance_error": float(delta.max()) if len(delta) else 0.}


def require_match(record):
    if record["hit_miss_differences"] or record["errors_over_0_001"]:
        raise ValueError(f"BSP does not match expanded brush union: {record}")


def plot_profile(root, profile):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    y = profile["y"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True, constrained_layout=True)
    for ax, mode in zip(axes, ("legacy", "simple")):
        split, merged = (profile[f"{name}_{mode}_h1"] for name in ("split", "merged"))
        ax.plot(y, split, color="#c44132", linewidth=2.5, label="Two brushes")
        ax.plot(y, merged, color="#16748b", linewidth=2, linestyle="--", label="One merged brush")
        ax.fill_between(y, merged, split, where=split > merged + .001, color="#c44132", alpha=.2)
        ax.axvline(-48, color="#666666", linewidth=1, linestyle=":")
        ax.axvline(-32, color="#888888", linewidth=1, linestyle=":")
        ax.set(xlim=(-72, 8), ylim=(68, 122), xlabel="Y (map units)", title=mode.capitalize())
        ax.grid(alpha=.2)
        ax.legend(loc="lower left")
    axes[0].set_ylabel("Hull 1 top Z (player centre)")
    axes[0].annotate("17.07 units", xy=(-32, 107.5), xytext=(-23, 116),
                     arrowprops={"arrowstyle": "->", "color": "#333333"})
    fig.suptitle("Same solid geometry, different brush decomposition (X = -96)")
    fig.savefig(root / "seam_hull1.png", dpi=160)
    plt.close(fig)


def write_report(root, report):
    lines = ["# Brush Seam Diagnosis", "",
             "The current source has six room brushes, split terrain brushes 6+7,",
             "and merged terrain brush 8. Translating brush 8 by (-192,0,0)",
             "makes its hull-0 solid equal to the union of brushes 6+7.", "",
             "Both aligned maps preserve the six room brushes and point entities.",
             "All six BSPs (including two original-layout builds) include VIS and RAD.", "",
             "Vertical rays at X=-96 start at Z=160, below the ceiling collision.",
             "Z values are collision boundaries for the player centre, without engine trace epsilon.", "",
             "| Hull | Y | Legacy split Z | Legacy merged Z | Excess height | Simple split Z | Simple merged Z |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for row in report["contacts"]:
        c = row["top_z"]
        lines.append(f"| {row['hull']} | {row['y']:.3f} | {c['split_legacy']:.6f} | "
                     f"{c['merged_legacy']:.6f} | {c['split_legacy'] - c['merged_legacy']:.6f} | "
                     f"{c['split_simple']:.6f} | {c['merged_simple']:.6f} |")
    lines += ["", "The dense profiles and oblique rays compare the actual BSPs with the",
              "exported CSG legacy brush volumes. Simple uses an independent convex",
              "brush-plus-player-box oracle. Full-room random point checks ignore only",
              "points within 0.0001 units of a volume boundary.", "",
              "The split legacy protrusion is already present in the expanded brush union.",
              "BSP classification reproduces that union in the tested region; it does not",
              "create this protrusion. Simple split and merged boundaries agree.", "",
              "The platform is Y=-80..-48 with top Z=80. Its standing legacy expansion",
              "reaches Y=-32 with top Z=116. At Y=-32 the merged legacy slope reaches",
              "only Z=98.934616. The split platform therefore adds a 17.065384-unit ledge.", "",
              "![Standing collision cross-section](seam_hull1.png)", "",
              "These are controlled geometry measurements and sampled BSP checks, not",
              "an in-game movement test or a proof about every possible brush assembly.", "",
              "## BSP Size and Compile Time", "",
              "| BSP | Stored clipnodes | Hull 1 reachable | CSG+BSP seconds | VIS+RAD seconds |",
              "| --- | ---: | ---: | ---: | ---: |"]
    for name, item in report["builds"].items():
        s, t = item["structure"], item["seconds"]
        lines.append(f"| {name}.bsp | {s['stored_nodes']} | {s['hulls']['1']['reachable']} | "
                     f"{t['csg'] + t['bsp']:.3f} | {t['vis'] + t['rad']:.3f} |")
    lines += ["", "Inputs, hashes, command logs, full results, and profile.npz are preserved",
              "beside this report. The original MAP and game installation are unchanged."]
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "compiler", "dumper", "output"):
        parser.add_argument("--" + key, required=True, type=Path)
    args = parser.parse_args()
    source, compiler, dumper, root = [getattr(args, key).resolve() for key in
                                    ("source", "compiler", "dumper", "output")]
    text = source.read_text(encoding="utf-8")
    document = parse_map(text)
    if len(document.entities) != 3 or len(document.entities[0].brushes) != 9:
        raise ValueError("Expected updated cliptype_sl: six room brushes and terrain brushes 6,7,8")
    if re.search(r'"(?:zhlt_cliptype|info_compile_parameters)"', text):
        raise ValueError("Persistent cliptype overrides require explicit handling")
    root.mkdir(parents=True, exist_ok=False)
    (root / "compiler").mkdir()
    shutil.copy2(source, root / "source_snapshot.map")
    shutil.copy2(compiler, root / "compiler/hltools.exe")
    shutil.copy2(dumper, root / "compiler/brush_geometry_dump.exe")
    shutil.copy2(__file__, root / "runner_snapshot.py")
    compiler, dumper = root / "compiler/hltools.exe", root / "compiler/brush_geometry_dump.exe"
    manifest = {"source": str(source), "source_sha256": sha256(source),
                "compiler_sha256": sha256(compiler), "dumper_sha256": sha256(dumper),
                "runner_sha256": sha256(__file__), "seed": 20261023,
                "merged_translation": [-192, 0, 0]}
    save_json(root / "manifest.json", manifest)

    def block(index):
        brush = document.entities[0].brushes[index]
        return "".join(document.lines[brush.start:brush.end + 1])

    maps = {"original": text, "split": replace_terrain(document, block(6) + block(7)),
            "merged": replace_terrain(document, translate_block(block(8), (-192, 0, 0)))}
    geometry = {}
    for name, content in maps.items():
        path = root / f"{name}.map"
        path.write_text(content, encoding="utf-8")
        require_command([dumper, path, root / f"{name}_geometry.json", "--legacy-hulls"],
                        root / f"{name}_geometry.txt")
        geometry[name] = json.loads((root / f"{name}_geometry.json").read_text())

    a, b = [vertices(brush) for brush in geometry["split"]["brushes"][6:]]
    merged = vertices(geometry["merged"]["brushes"][6])
    merged_hull = ConvexHull(merged)
    combined_hull = ConvexHull(np.concatenate((a, b)))
    split_volume = ConvexHull(a).volume + ConvexHull(b).volume
    # The two inputs lie on opposite sides of their common face, so their
    # interiors cannot overlap. Equal convex volumes then establish the union.
    if not (np.all(a[:, 1] >= -48 - 1e-7) and np.all(b[:, 1] <= -48 + 1e-7)):
        raise ValueError("Split brushes do not meet on Y=-48")
    if max(abs(split_volume - merged_hull.volume),
           abs(combined_hull.volume - merged_hull.volume)) > 1e-5:
        raise ValueError("Split and merged hull-0 volumes differ")
    equations = merged_hull.equations
    if np.max(np.concatenate((a, b)) @ equations[:, :3].T + equations[:, 3]) > 1e-7:
        raise ValueError("Split vertices lie outside merged solid")
    report = {"manifest": manifest, "hull0_equivalence": {
        "split_volume": split_volume, "merged_volume": merged_hull.volume,
        "combined_convex_volume": combined_hull.volume, "shared_face_y": -48},
        "builds": {}, "validation": {}, "contacts": [], "profiles": {}}
    files = {}
    for name, content in maps.items():
        for mode in ("legacy", "simple"):
            key = f"{name}_{mode}"
            path = root / f"{key}.map"
            path.write_text(content, encoding="utf-8")
            seconds = {"csg": require_command(
                [compiler, "csg", path, "-cliptype", mode, "-threads", "2"],
                root / f"{key}.csg.txt")}
            for stage in ("bsp", "vis", "rad"):
                seconds[stage] = require_command([compiler, stage, path.with_suffix(".bsp"),
                                                  "-threads", "2"], root / f"{key}.{stage}.txt")
            files[key] = HullFile(path.with_suffix(".bsp"))
            tree = files[key]
            if not tree.lumps[4] or not any(tree.lumps[8]):
                raise ValueError("VIS or lighting is missing")
            report["builds"][key] = {"seconds": seconds, "structure": structure(tree),
                                      "sha256": sha256(path.with_suffix(".bsp"))}
            start = np.array([document.entities[1].keyvalues["origin"].split()], dtype=float)
            if any(tree.contents(start, hull)[0] != -1 for hull in (1, 3)):
                raise ValueError(f"Player start is blocked: {key}")
            print(f"built {key}: {len(tree.nodes)} clipnodes", flush=True)

    rng = np.random.default_rng(20261023)
    points = rng.uniform([-159.73, -159.61, 4.37], [159.81, 159.89, 203.53], (50000, 3))
    ys = np.unique(np.r_[np.linspace(-100, 100, 4001), -48, -32, -16, 0, 16, 32, 48])
    origins = np.c_[np.full(len(ys), -96.), ys, np.full(len(ys), 160.)]
    directions = np.tile([0., 0., -1.], (len(ys), 1))
    oblique_origins = rng.uniform([-143.73, -111.61, 130.37], [-48.81, 95.89, 159.53], (4096, 3))
    targets = rng.uniform([-127.73, -79.61, -16.37], [-64.81, 47.89, 79.53], (4096, 3))
    oblique_directions = targets - oblique_origins
    oblique_directions /= np.linalg.norm(oblique_directions, axis=1)[:, None]
    profile = {"y": ys}
    for hull in (1, 2, 3):
        contacts = {}
        for name in maps:
            for mode in ("legacy", "simple"):
                key = f"{name}_{mode}"
                target = volumes(geometry[name], hull, mode)
                actual = files[key]
                expected_margin = margin(points, target)
                counts = error_counts(actual.contents(points, hull) == -2, expected_margin, .0001)
                if any(counts.values()):
                    raise ValueError(f"BSP point mismatch: {key}, hull {hull}, {counts}")
                measured = tree_first_hit(actual, origins, directions, 180., hull)
                expected = convex_hit(origins, directions, 180., target)
                vertical_errors = ray_errors(measured, expected)
                require_match(vertical_errors)
                oblique_errors = ray_errors(
                    tree_first_hit(actual, oblique_origins, oblique_directions, 240., hull),
                    convex_hit(oblique_origins, oblique_directions, 240., target))
                require_match(oblique_errors)
                report["validation"][f"{key}_h{hull}"] = {
                    "points": len(points), **counts, "vertical_rays": vertical_errors,
                    "oblique_rays": oblique_errors}
                contacts[key] = 160 - measured
                profile[f"{key}_h{hull}"] = contacts[key]
        delta = contacts["split_legacy"] - contacts["merged_legacy"]
        simple_delta = contacts["split_simple"] - contacts["merged_simple"]
        if np.max(np.abs(simple_delta)) > .001 or delta.max() < 1:
            raise ValueError("Expected legacy-only decomposition difference was not reproduced")
        report["profiles"][str(hull)] = {
            "legacy_max_split_excess_z": float(delta.max()),
            "legacy_excess_sample_y": ys[delta > .001][[0, -1]].tolist(),
            "simple_max_split_merged_difference": float(np.max(np.abs(simple_delta)))}
        for y in (-64., -48., -40., -32., -24., 0.):
            index = np.argmin(abs(ys - y))
            report["contacts"].append({"hull": hull, "x": -96., "y": float(ys[index]),
                                       "top_z": {key: float(value[index]) for key, value in contacts.items()}})
        for mode in ("legacy", "simple"):
            original = files[f"original_{mode}"]
            right_origins = origins + [192., 0., 0.]
            right = tree_first_hit(original, right_origins, directions, 180., hull)
            require_match(ray_errors(right, 160 - contacts[f"merged_{mode}"]))
    draw_split = Tree.draw(root / "split_legacy.bsp")
    draw_merged = Tree.draw(root / "merged_legacy.bsp")
    if np.any(draw_split.contents(points) != draw_merged.contents(points)):
        raise ValueError("Hull-0 classification differs")
    report["hull0_equivalence"]["bsp_point_checks"] = len(points)
    if sha256(source) != manifest["source_sha256"]:
        raise ValueError("Original MAP changed")
    np.savez_compressed(root / "profile.npz", **profile)
    save_json(root / "results.json", report)
    plot_profile(root, profile)
    write_report(root, report)
    print(json.dumps(report["profiles"], indent=2), flush=True)


if __name__ == "__main__":
    main()
