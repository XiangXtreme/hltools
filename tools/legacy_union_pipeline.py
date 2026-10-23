"""Compile the union of actual legacy-expanded brushes, preserving legacy slopes.

The legacytrim sidecar is read only as diagnostic metadata. Its trimming pass
is never run. No player-box expansion or simple target is used for repair.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import time

os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

import numpy as np

from brush_surface_pipeline import (Tree, load_module, repair, require_command,
                                    splice, validate, ray_stats)
from legacy_clip_diagnose import convex_hit, margin, read_volumes, save_json, sha256
from remove_map_group_volume import parse_map
from source_clip_pipeline import HullFile, SCRIPTS
from source_clip_validate import structure


def override_cliptype(source, mode):
    document = parse_map(source)
    if any(entity.classname == "info_compile_parameters" for entity in document.entities):
        raise ValueError("Persistent compile parameters require explicit handling")
    for entity in document.entities:
        for i in range(entity.start + 1, entity.end):
            line = document.lines[i].strip()
            if line.startswith('"zhlt_cliptype"'):
                document.lines[i] = f'"zhlt_cliptype" "{mode}"\n'
    return "".join(document.lines)


def compile_volumes(path, volumes, exact, compiler):
    # These planes have already been expanded by CSG in legacy mode. Expanding
    # vertices by a box here would change the target back into a simple hull.
    body = ["{\n" + "\n".join(exact.plane_face(p[:3], p[3]) for p in v["planes"]) + "\n}"
            for v in volumes]
    path.write_text('{\n"classname" "worldspawn"\n"mapversion" "220"\n'
                    + f'"wad" "{exact.WAD}"\n' + "\n".join(body) + "\n}\n", encoding="ascii")
    seconds = {}
    for stage, flags in (("csg", ["-noclip"]),
                         ("bsp", ["-noclip", "-nofill", "-noinsidefill", "-nonulltex"])):
        seconds[stage] = require_command([compiler, stage, path, *flags, "-threads", "2"],
                                          path.with_suffix(f".{stage}.txt"))
    return Tree.draw(path.with_suffix(".bsp")), seconds


def selected_volumes(geometry, groups, entity):
    brushes = [b for b in geometry["brushes"] if b["entity"] == entity and b["contents"] == -2]
    if not brushes:
        raise ValueError("No selected solid terrain brushes")
    selected = {}

    def key(planes):
        return tuple(sorted(tuple(np.round(p, 7)) for p in planes))

    for hull in (1, 2, 3):
        lookup = {key(v["planes"]): i for i, v in enumerate(groups[hull][0])}
        pairs = []
        for brush in brushes:
            exported = brush["legacy_hulls"][hull - 1]
            planes = np.array(exported["planes"])
            if key(planes) not in lookup:
                raise ValueError("Selected hull does not match the verified legacy CSG metadata")
            index = lookup[key(planes)]
            volume = dict(groups[hull][0][index])
            values = np.array(exported["vertices"])
            _, ids = np.unique(np.round(values, 9), axis=0, return_index=True)
            volume["vertices"] = values[np.sort(ids)]
            if len(volume["vertices"]) < 4:
                raise ValueError("Degenerate selected legacy hull")
            pairs.append((volume, groups[hull][1][index]))
        selected[hull] = ([v for v, _ in pairs], [v for _, v in pairs])
    return selected


def top_checks(tree, legacy, box, hull, root):
    low = np.min([v["bounds"][0] for v in legacy], axis=0)
    high = np.max([v["bounds"][1] for v in legacy], axis=0)
    xs = np.linspace(low[0] + .173205, high[0] - .271828, 257)
    ys = np.linspace(low[1] + .271828, high[1] - .173205, 257)
    xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
    origins = np.c_[xy, np.full(len(xy), high[2] + 20)]
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    length = float(high[2] - low[2] + 40)
    a = convex_hit(origins, directions, length, legacy)
    b = convex_hit(origins, directions, length, box)
    actual = tree.hit(origins, directions, length)
    both = np.isfinite(a) & np.isfinite(b)
    ids = np.flatnonzero(both)
    if not len(ids):
        raise ValueError("No comparable top contact")
    delta = b[ids] - a[ids]
    index = ids[np.argmax(np.abs(delta))]
    point = origins[index] + directions[index] * ((a[index] + b[index]) / 2)
    np.savez_compressed(root / f"tops_h{hull}.npz", x=xs, y=ys,
                        legacy_target=(origins[:, 2] - a).reshape(257, 257),
                        box_target=(origins[:, 2] - b).reshape(257, 257),
                        repaired=(origins[:, 2] - actual).reshape(257, 257))
    return {"rays": len(origins), "repaired_vs_legacy": ray_stats(actual, a),
            "legacy_vs_box": ray_stats(a, b),
            "witness": {"xy": xy[index].tolist(), "point": point.tolist(),
                        "legacy_height": float(origins[index, 2] - a[index]),
                        "box_height": float(origins[index, 2] - b[index]),
                        "repaired_height": float(origins[index, 2] - actual[index]),
                        "legacy_margin": float(margin(point[None], legacy)[0]),
                        "box_margin": float(margin(point[None], box)[0]),
                        "repaired_contents": int(tree.contents(point[None])[0])}}


def write_report(root, report):
    lines = ["# Legacy union collision experiment", "",
             "Repair target: union of actual CSG legacy-expanded terrain brushes.",
             "The legacytrim metadata exporter is used for audit only; its BSP trim pass is not run.",
             "Selected planes match the metadata and the primary CSG data matches ordinary legacy byte for byte.", "",
             "Full vertices are not expanded again. No box-expansion volume is used for repair.",
             "Original legacy plane positions are therefore the geometric target, including any",
             "undesirable join that already belongs to that union. This does not solve such intrinsic joins.", "",
             "| Hull | Terrain clipnodes | Final clipnodes | Repair seconds | Holdout extra/missing | Holdout max ray error | Top grid max error |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    final = report["output"]["structure"]
    for hull, value in report["hulls"].items():
        holdout = value["holdout"]
        lines.append(f"| {hull} | {value['repair']['nodes']} | {final['hulls'][hull]['reachable']} | "
                     f"{value['repair']['seconds']:.3f} | {holdout['extra_solid']}/{holdout['missing_solid']} | "
                     f"{holdout['ray_checks']['max_distance_error']:.6f} | "
                     f"{value['top_contact']['repaired_vs_legacy']['max_distance_error']:.6f} |")
    lines += ["", "Each holdout uses 250,000 points, 8,192 rays and about 96,000 exterior points.",
              "Top holdout uses 66,049 vertical rays. Repair uses a different 193x193 top grid,",
             "a separate random seed, region EPS 1e-5 and classification tolerance 1e-4.",
             "Local replacement tests the original convex half-spaces directly; it does not rebuild fragment incidence.",
              "Holdout error counts use 0.01 units. Sampling is not a whole-space proof.", "",
              "## Difference From Box Expansion", ""]
    for hull, value in report["hulls"].items():
        witness = value["top_contact"]["witness"]
        lines.append(f"Hull {hull}, XY {witness['xy']}: legacy target Z {witness['legacy_height']:.6f}, "
                     f"box target Z {witness['box_height']:.6f}, repaired Z {witness['repaired_height']:.6f}.")
        lines.append("")
    lines += ["## Output", "", f"[{Path(report['output']['file']).name}]({Path(report['output']['file']).name})", "",
              f"Total clipnodes: {final['stored_nodes']}. Light bytes: {report['output']['light_bytes']}; "
              f"VIS bytes: {report['output']['vis_bytes']}. Wall time: {report['wall_seconds']:.3f} seconds.", "",
              "Visible geometry, embedded textures, entities, VIS and lightmaps match the background BSP.",
              "Background and exterior collision remain present. Union checks use 150,000 points per hull.",
              "Player start, graph indices, cycles, node budget and root lower bounds are checked.",
              "Source MAP and installed BSP are unchanged. In-game movement has not been tested.", "",
              "![Measured hull 1 legacy target, box target and repaired result](legacy_comparison.png)", ""]
    (root / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    grid = np.load(root / "tops_h1.npz")
    a, b, repaired = grid["legacy_target"], grid["box_target"], grid["repaired"]
    extent = [grid["x"][0], grid["x"][-1], grid["y"][0], grid["y"][-1]]
    finite = np.isfinite(a) & np.isfinite(repaired)
    delta = np.where(finite, np.where(finite, repaired, 0) - np.where(finite, a, 0), np.nan)
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    minimum, maximum = min(a[np.isfinite(a)].min(), b[np.isfinite(b)].min()), max(a[np.isfinite(a)].max(), b[np.isfinite(b)].max())
    for ax, data, title in zip(axes[0], (a, b), ("Actual legacy union / hull 1", "Player-box union / hull 1")):
        picture = ax.imshow(np.where(np.isfinite(data), data, np.nan), origin="lower", extent=extent,
                            vmin=minimum, vmax=maximum, cmap="viridis")
        ax.set_title(title)
        fig.colorbar(picture, ax=ax, label="First contact Z")
    limit = max(.01, float(np.nanmax(np.abs(delta))))
    picture = axes[1, 0].imshow(delta, origin="lower", extent=extent, cmap="RdBu_r", vmin=-limit, vmax=limit)
    axes[1, 0].set_title("Repaired minus legacy target")
    fig.colorbar(picture, ax=axes[1, 0], label="Z error (units)")
    y = report["hulls"]["1"]["top_contact"]["witness"]["xy"][1]
    row = int(np.argmin(abs(grid["y"] - y)))
    for data, label, color, style in ((a, "Legacy target", "#16875b", "-"),
                                      (b, "Box expansion target", "#286bb2", "-"),
                                      (repaired, "Repaired legacy union", "#c43c39", "--")):
        axes[1, 1].plot(grid["x"], np.where(np.isfinite(data[row]), data[row], np.nan), label=label, color=color, linestyle=style)
    axes[1, 1].set_title(f"Cross section at Y = {y:.3f}")
    axes[1, 1].set_ylabel("Player origin Z")
    axes[1, 1].legend()
    for ax in axes.flat:
        ax.set_xlabel("X (units)")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.set_ylabel("Y (units)")
    fig.savefig(root / "legacy_comparison.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "compiler", "dumper", "scripts", "wad", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--entity", required=True, type=int)
    args = parser.parse_args()
    for name in ("source", "compiler", "dumper", "scripts", "wad", "output"):
        setattr(args, name, getattr(args, name).resolve())
    root = args.output
    root.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    (root / "compiler").mkdir()
    (root / "scripts").mkdir()
    manifest = {"source": str(args.source), "source_sha256": sha256(args.source),
                "compiler_sha256": sha256(args.compiler), "dumper_sha256": sha256(args.dumper),
                "runner_sha256": sha256(__file__), "entity": args.entity,
                "scripts": {name: sha256(args.scripts / name) for name in SCRIPTS}}
    save_json(root / "manifest.json", manifest)
    shutil.copy2(args.source, root / "source_snapshot.map")
    shutil.copy2(args.compiler, root / "compiler/hltools.exe")
    shutil.copy2(args.dumper, root / "compiler/brush_geometry_dump.exe")
    shutil.copy2(__file__, root / "runner_snapshot.py")
    for name in SCRIPTS:
        shutil.copy2(args.scripts / name, root / "scripts" / name)
    source = args.source.read_text(encoding="utf-8")
    compiler = root / "compiler/hltools.exe"
    exact = load_module("legacy_reference_exacthull", root / "scripts/exacthull.py")
    exact.WAD = args.wad.as_posix()
    bb = load_module("legacy_reference_brushbsp", root / "scripts/brushbsp.py")
    bb.EPS = 1e-5
    report = {"target": "Union of actual legacy-expanded solid brushes; box volumes used only for comparison",
              "commands": {}, "hulls": {}}
    for name, mode in (("legacy", "legacy"), ("audit", "legacytrim")):
        path = root / f"{name}.map"
        path.write_text(override_cliptype(source, mode), encoding="utf-8")
        report["commands"][f"{name}_csg"] = require_command(
            [compiler, "csg", path, "-cliptype", mode, "-threads", "2"], root / f"{name}.csg.txt")
    equal = {suffix: sha256(root / f"legacy.{suffix}") == sha256(root / f"audit.{suffix}")
             for suffix in ("p0", "p1", "p2", "p3", "b0", "b1", "b2", "b3", "pln")}
    if not all(equal.values()):
        raise ValueError("Diagnostic export changed primary legacy CSG geometry")
    report["audit_primary_identical"] = equal
    _, groups = read_volumes(root / "audit.hsz")
    report["commands"]["legacy_geometry_export"] = require_command(
        [root / "compiler/brush_geometry_dump.exe", root / "legacy.map", root / "legacy_geometry.json",
         "--legacy-hulls"], root / "legacy_geometry.txt")
    groups = selected_volumes(json.loads((root / "legacy_geometry.json").read_text()), groups, args.entity)
    save_json(root / "volumes.json", {str(h): {"legacy": [{k: v.tolist() for k, v in volume.items()}
                                                          for volume in pair[0]]}
                                           for h, pair in groups.items()})
    report["commands"]["legacy_bsp"] = require_command(
        [compiler, "bsp", root / "legacy.map", "-threads", "2"], root / "legacy.bsp.txt")
    legacy_bsp = HullFile(root / "legacy.bsp")
    report["original_structure"] = structure(legacy_bsp)
    report["commands"]["background_csg"] = require_command(
        [root / "compiler/brush_geometry_dump.exe", root / "legacy.map", root / "geometry.json",
         "--background", args.entity, root / "background"], root / "background.csg.txt")
    for stage in ("bsp", "vis", "rad"):
        report["commands"][f"background_{stage}"] = require_command(
            [compiler, stage, root / "background.bsp", "-threads", "2"], root / f"background.{stage}.txt")
    background = HullFile(root / "background.bsp")
    if len(background.lumps[8]) <= 1 or not any(background.lumps[8]):
        raise ValueError("Lighting did not produce nonzero samples")
    trees = {}
    for hull in (1, 2, 3):
        legacy, box = groups[hull]
        print(f"legacy hull {hull}: {len(legacy)} expanded brushes", flush=True)
        tree, seconds = compile_volumes(root / f"legacy_union_h{hull}.map", legacy, exact, compiler)
        record = {"source_volumes": len(legacy), "compile_seconds": seconds,
                  "raw_nodes": len(tree.nodes), "before": validate(tree, legacy)}
        low = np.min([v["bounds"][0] for v in legacy], axis=0)
        high = np.max([v["bounds"][1] for v in legacy], axis=0)
        xs = np.linspace(low[0] + .314159, high[0] - .141421, 193)
        ys = np.linspace(low[1] + .161803, high[1] - .271828, 193)
        xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
        origins = np.c_[xy, np.full(len(xy), high[2] + 20)]
        rays = (origins, np.tile([0., 0., -1.], (len(xy), 1)), float(high[2] - low[2] + 40))
        record["repair"] = repair(tree, legacy, bb, tolerance=1e-4, extra_rays=rays, local_builder="planes")
        record["holdout"] = validate(tree, legacy)
        record["top_contact"] = top_checks(tree, legacy, box, hull, root)
        trees[hull] = tree
        tree.save(root / f"legacy_union_h{hull}.npz")
        report["hulls"][str(hull)] = record
        save_json(root / "results.json", report)
        print(json.dumps(record), flush=True)
    data, checked = splice(root / "background.bsp", trees)
    out = root / (args.source.stem + "_legacy_union.bsp")
    out.write_bytes(data)
    report["output"] = {"file": str(out), "sha256": sha256(out), "structure": checked,
                        "light_bytes": len(background.lumps[8]), "vis_bytes": len(background.lumps[4])}
    report["wall_seconds"] = time.perf_counter() - started
    if sha256(args.source) != manifest["source_sha256"]:
        raise ValueError("Source changed during compilation")
    save_json(root / "results.json", report)
    write_report(root, report)
    print(json.dumps(report["output"], indent=2), flush=True)


if __name__ == "__main__":
    main()
