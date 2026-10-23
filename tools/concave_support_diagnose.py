"""Test shared exterior legacy support planes on an explicitly selected assembly.

This is a bounded experiment, not an automatic terrain cliptype. It intersects
the existing convex collision groups with planes supporting the entire source
assembly. The union remains nonconvex; no convex hull replaces its volume.
"""

import argparse
import itertools
import json
from pathlib import Path
import shutil
import struct
import time

import numpy as np
from scipy.spatial import ConvexHull

from brush_seam_diagnose import vertices, volumes, ray_errors, require_match
from brush_surface_pipeline import HALF, Tree, load_module, require_command, splice, validate
from legacy_clip_diagnose import convex_hit, margin, error_counts, save_json, sha256
from legacy_union_pipeline import compile_volumes
from remove_map_group_volume import parse_map
from source_clip_pipeline import HullFile
from source_clip_validate import structure, tree_first_hit


def support_planes(brushes):
    points = np.concatenate([vertices(brush) for brush in brushes])
    found = []
    for brush in brushes:
        for face in brush["faces"]:
            p = np.array([*face["normal"], face["dist"]], dtype=float)
            if np.max(points @ p[:3] - p[3]) > 1e-7:
                continue
            if not any(np.max(abs(p - other)) < 1e-8 for other in found):
                found.append(p)
    return np.array(found).reshape(-1, 4)


def shift_legacy(planes, half):
    result = planes.copy()
    result[:, 3] += planes[:, :3] ** 2 @ half
    return result


def intersection(planes):
    """Enumerate bounded half-space vertices and discard inactive planes."""
    points = []
    for ids in itertools.combinations(range(len(planes)), 3):
        triple = planes[list(ids)]
        if abs(np.linalg.det(triple[:, :3])) < 1e-10:
            continue
        point = np.linalg.solve(triple[:, :3], triple[:, 3])
        if np.max(planes[:, :3] @ point - planes[:, 3]) <= 1e-7:
            if not any(np.linalg.norm(point - other) < 1e-6 for other in points):
                points.append(point)
    points = np.array(points)
    if len(points) < 4 or ConvexHull(points).volume < 1e-6:
        raise ValueError("Empty or degenerate clipped collision group")
    active = []
    for p in planes:
        face = points[abs(points @ p[:3] - p[3]) < 1e-6]
        if len(face) < 3 or np.linalg.matrix_rank(face[1:] - face[0], tol=1e-6) < 2:
            continue
        if not any(np.max(abs(p - other)) < 1e-8 for other in active):
            active.append(p)
    return {"planes": np.array(active), "vertices": points,
            "bounds": np.array([points.min(0), points.max(0)])}


def collision_carrier(visual, collision, path):
    """Keep visual BSP data and append the separate background collision tree."""
    base, other = HullFile(visual), HullFile(collision)
    lumps = list(base.lumps)
    pshift = len(lumps[1]) // 20
    lumps[1] += other.lumps[1]
    lumps[9] = b"".join(struct.pack("<ihh", int(p) + pshift, int(f), int(b))
                         for p, f, b in other.nodes)
    model = bytearray(lumps[14])
    struct.pack_into("<3i", model, 40, *map(int, other.models[0]["head"][1:]))
    lumps[14] = bytes(model)
    data = bytearray(124)
    struct.pack_into("<i", data, 0, 30)
    for index, lump in enumerate(lumps):
        data.extend(b"\0" * (-len(data) % 4))
        struct.pack_into("<ii", data, 4 + 8 * index, len(data), len(lump))
        data.extend(lump)
    structure(HullFile(data=data))
    path.write_bytes(data)


def background_map(text, selected):
    document = parse_map(text)
    world = document.entities[0]
    blocks = []
    for index in sorted(selected):
        brush = world.brushes[index]
        blocks.append("".join(document.lines[brush.start:brush.end + 1]))
        for line in range(brush.comment_start, brush.end + 1):
            document.lines[line] = ""
    # Noclip detail retains render surfaces while the room supplies background
    # and exterior collision. Final rendering comes from the original BSP.
    return ("".join(document.lines) + '\n{\n"classname" "func_detail"\n'
            '"zhlt_noclip" "1"\n' + "\n".join(blocks) + "\n}\n")


def plot(root, grids, contacts):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    grid = grids[1]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    a, b = grid["clipmerge"], grid["support"]
    both = np.isfinite(a) & np.isfinite(b)
    picture = axes[0].pcolormesh(grid["x"], grid["y"],
                               np.where(both, np.where(both, a, 0) - np.where(both, b, 0), np.nan),
                               shading="auto", cmap="viridis")
    fig.colorbar(picture, ax=axes[0], label="Removed excess height")
    axes[0].set(title="Hull 1: shared support clipping", xlabel="X", ylabel="Y")
    y = contacts["1"]["max_trim_xy"][1]
    if contacts["1"]["probes"]:
        probe = max(contacts["1"]["probes"], key=lambda p: p["top_z"]["clipmerge"] - p["top_z"]["support"])
        y = probe["xy"][1]
    row = int(np.argmin(abs(grid["y"] - y)))
    for key, label, color, style in (("legacy", "Legacy", "#c44132", ":"),
                                    ("clipmerge", "Legacy + clipmerge", "#777777", "-"),
                                    ("simple", "Simple", "#356bc1", "-"),
                                    ("support", "Shared support / legacy offsets", "#16745b", "--")):
        axes[1].plot(grid["x"], grid[key][row], label=label, color=color, linestyle=style)
    axes[1].set(title=f"Hull 1 contact at Y={y:g}", xlabel="X", ylabel="Contact Z")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=.2)
    fig.savefig(root / "support_hull1.png", dpi=160)
    plt.close(fig)


def write_report(root, report):
    lines = ["# Shared Legacy Support Experiment", "",
             "An explicitly selected nonconvex world-brush assembly is intersected with",
             "legacy-shifted source planes that contain every source vertex. Each original",
             "legacy collision brush remains a convex piece; their union preserves concavities.", "",
             "| Mode | Stored clipnodes | Compile seconds |",
             "| --- | ---: | ---: |"]
    for name in ("legacy", "clipmerge", "simple"):
        record = report["builds"][name]
        lines.append(f"| {name} | {record['structure']['stored_nodes']} | {record['seconds']:.3f} |")
    lines += [f"| Shared support | {report['output']['structure']['stored_nodes']} | See results.json |", "",
              "All playable versions include VIS and lighting. The experiment reuses the",
              "clipmerge render BSP byte for byte, appending only planes and replacing collision nodes/roots.", "",
              "## Standing Hull Contacts", "",
              "Collision boundaries are player-origin Z, before the engine trace epsilon.", "",
              "| X | Y | Legacy | Clipmerge | Simple | Shared support |",
              "| ---: | ---: | ---: | ---: | ---: | ---: |"]
    for probe in report["contacts"]["1"]["probes"]:
        z = probe["top_z"]
        lines.append(f"| {probe['xy'][0]:.4f} | {probe['xy'][1]:.4f} | {z['legacy']:.6f} | "
                     f"{z['clipmerge']:.6f} | {z['simple']:.6f} | {z['support']:.6f} |")
    lines += ["", "## Validation", "",
              "Each original convex source brush is contained by one clipped convex group:",
              "all its vertices satisfy that group's planes, proving whole-brush containment.",
              "Intersection can only remove collision from the original legacy union.", "",
              "Per hull, the terrain BSP passes 250,000 points and 8,192 rays against the",
              "unsnapped clipped half-space volumes. The final BSP passes 150,000 interior",
              "points, an independent set of 8,192 oblique rays, the dense vertical grid, and",
              "roughly 100,000 exterior points. Final ray tolerance is 0.001 map units.", "",
              "The retained concave-gap witness in results.json lies inside the convex hull",
              "of the source vertices, outside its brush union, and in final hull-1 EMPTY.",
              "Player starts, plane/node indices, cycles, hull lower bounds and budget pass.", "",
              "## Scope", "",
              "This establishes a nonconvex-footprint case with shared supporting roof planes.",
              "A valley face whose extended plane crosses another part of the terrain is",
              "excluded. General concave roof profiles, automatic neighborhood selection,",
              "custom hulls, special brush policies and arbitrary brush decomposition are unproven.",
              "This experiment does not change production cliptypes or game installation files.",
              "In-game movement has not been tested.", "",
              "![Hull 1 measured comparison](support_hull1.png)", "",
              f"Output: `{Path(report['output']['path']).name}`."]
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "compiler", "dumper", "exacthull", "output"):
        parser.add_argument("--" + key, required=True, type=Path)
    parser.add_argument("--brushes", nargs="+", type=int, required=True,
                        help="Explicit worldspawn brush indices, numbered from zero")
    parser.add_argument("--probe-xy", nargs=2, type=float, action="append", default=[])
    args = parser.parse_args()
    for key in ("source", "compiler", "dumper", "exacthull", "output"):
        setattr(args, key, getattr(args, key).resolve())
    started = time.perf_counter()
    text = args.source.read_text(encoding="utf-8")
    document = parse_map(text)
    if document.entities[0].classname != "worldspawn":
        raise ValueError("Expected worldspawn as entity zero")
    if any(key in text.lower() for key in ("zhlt_cliptype", "info_compile_parameters", "info_hullshape")):
        raise ValueError("Persistent or custom hull options need explicit handling")
    selected = set(args.brushes)
    if not selected or len(selected) != len(args.brushes) or not all(
            0 <= i < len(document.entities[0].brushes) for i in selected):
        raise ValueError("Invalid or duplicate source brush indices")
    root = args.output
    root.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.source, root / "source_snapshot.map")
    for key, name in (("compiler", "hltools.exe"), ("dumper", "brush_geometry_dump.exe"),
                      ("exacthull", "exacthull.py")):
        shutil.copy2(getattr(args, key), root / name)
    shutil.copy2(__file__, root / "runner_snapshot.py")
    compiler, dumper = root / "hltools.exe", root / "brush_geometry_dump.exe"
    report = {"manifest": {"source": str(args.source), "source_sha256": sha256(args.source),
                           "compiler_sha256": sha256(compiler), "dumper_sha256": sha256(dumper),
                           "exacthull_sha256": sha256(root / "exacthull.py"),
                           "runner_sha256": sha256(__file__), "selected_brushes": sorted(selected)},
              "builds": {}, "hulls": {}, "contacts": {}}
    geometries = {}
    for name, flags in (("geometry", []), ("merged_geometry", ["--clipmerge"])):
        require_command([dumper, root / "source_snapshot.map", root / (name + ".json"),
                         "--legacy-hulls", *flags], root / (name + ".txt"))
        geometries[name] = json.loads((root / (name + ".json")).read_text())
    original = geometries["geometry"]["brushes"]
    if any(b["entity"] != 0 or b["contents"] != -2 for b in original):
        raise ValueError("This small experiment requires a single solid world model")
    terrain = [b for b in original if b["brush"] in selected]
    if len(terrain) != len(selected):
        raise ValueError("Selected geometry is missing")
    support = support_planes(terrain)
    report["support_planes"] = support.tolist()
    print(f"Selected {len(terrain)} brushes; {len(support)} shared source support planes", flush=True)
    files = {}
    for name, mode, flags in (("legacy", "legacy", []), ("clipmerge", "legacy", ["-clipmerge"]),
                              ("simple", "simple", []), ("background", "legacy", [])):
        path = root / (name + ".map")
        path.write_text(background_map(text, selected) if name == "background" else text, encoding="utf-8")
        seconds = require_command([compiler, "compile", path, "-cliptype", mode, *flags,
                                   "-threads", "2"], root / (name + ".compile.txt"))
        tree = files[name] = HullFile(path.with_suffix(".bsp"))
        if not tree.lumps[4] or not any(tree.lumps[8]):
            raise ValueError("Missing VIS or lighting")
        report["builds"][name] = {"seconds": seconds, "structure": structure(tree)}
    exact = load_module("support_reference_formatter", root / "exacthull.py")
    exact.WAD = document.entities[0].keyvalues["wad"].replace("\\", "/")
    # The reference four-decimal MAP coordinates amplify errors on grazing
    # traces. Preserve the measured plane when serializing expanded solids.
    exact.fmt = lambda point: "( " + " ".join(f"{value:.12f}" for value in point) + " )"
    trees, grids = {}, {}
    source_points = np.concatenate([vertices(b) for b in terrain])
    axes = []
    for axis in (0, 1):
        transitions = np.unique(source_points[:, axis])
        transitions = np.r_[transitions, transitions - 16, transitions + 16,
                            transitions - 32, transitions + 32]
        axes.append(np.unique(np.r_[
            np.linspace(source_points[:, axis].min() - 34, source_points[:, axis].max() + 34, 393),
            transitions - .0001, transitions + .0001,
            [p[axis] for p in args.probe_xy]]))
    xs, ys = axes
    xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
    origins = np.c_[xy, np.full(len(xy), 160.)]
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    rng = np.random.default_rng(20261024)
    points = rng.uniform([-159.7, -159.6, -15.3], [159.8, 159.9, 190.5], (150000, 3))
    ro = rng.uniform([-140., -140., 125.], [140., 140., 160.], (8192, 3))
    rt = rng.uniform([-10., -60., -20.], [125., 60., 80.], (8192, 3))
    rd = rt - ro
    rd /= np.linalg.norm(rd, axis=1)[:, None]
    target_by_hull = {}
    for h in (1, 2, 3):
        caps = shift_legacy(support, HALF[h])
        # Use original brushes so a convex merge cannot absorb an unselected
        # brush into a selected group. Shared cuts are applied before BSP CSG.
        active = terrain
        target = [intersection(np.r_[np.array(b["legacy_hulls"][h - 1]["planes"]), caps]) for b in active]
        # Each original convex source brush must be contained by ONE clipped
        # convex group. Vertex containment then proves whole-brush containment.
        for b in terrain:
            v = vertices(b)
            if not any(np.max(v @ t["planes"][:, :3].T - t["planes"][:, 3]) < 1e-6 for t in target):
                raise ValueError("Clipping removed source brush geometry or crossed selection ownership")
        tree, seconds = compile_volumes(root / f"support_h{h}.map", target, exact, compiler)
        tree.compact()
        checked = validate(tree, target)
        if checked["extra_solid"] or checked["missing_solid"] or checked["exterior_extra_solid"]:
            raise ValueError(f"Compiled support hull differs from its plane target: {checked}")
        if checked["ray_checks"]["hit_miss_differences"] or checked["ray_checks"]["errors_over_0_01"]:
            raise ValueError(f"Compiled support hull trace mismatch: {checked}")
        trees[h], target_by_hull[h] = tree, target
        room = volumes({"brushes": [b for b in original if b["brush"] not in selected]}, h, "legacy")
        oracle = room + target
        grid = {"x": xs, "y": ys}
        for name in ("legacy", "clipmerge", "simple"):
            grid[name] = (160 - tree_first_hit(files[name], origins, directions, 220., h)).reshape(len(ys), len(xs))
        grid["support"] = (160 - convex_hit(origins, directions, 220., oracle)).reshape(len(ys), len(xs))
        valid = np.isfinite(grid["clipmerge"]) & np.isfinite(grid["support"])
        delta = np.where(valid, grid["clipmerge"] - grid["support"], -np.inf)
        iy, ix = np.unravel_index(np.argmax(delta), delta.shape)
        report["contacts"][str(h)] = {"max_trim": float(delta[iy, ix]), "max_trim_xy": [float(xs[ix]), float(ys[iy])],
                                      "clipmerge_z": float(grid["clipmerge"][iy, ix]),
                                      "support_z": float(grid["support"][iy, ix]),
                                      "changed_grid_points": int(np.sum(delta > .001))}
        probes = []
        for px, py in args.probe_xy:
            probe_origin = np.array([[px, py, 160.]])
            probe_direction = np.array([[0., 0., -1.]])
            heights = {name: float(160 - tree_first_hit(files[name], probe_origin, probe_direction, 220., h)[0])
                       for name in ("legacy", "clipmerge", "simple")}
            heights["support"] = float(160 - convex_hit(probe_origin, probe_direction, 220., oracle)[0])
            probes.append({"xy": [px, py], "top_z": heights})
        report["contacts"][str(h)]["probes"] = probes
        report["hulls"][str(h)] = {"target_groups": len(target), "compile_seconds": seconds,
                                   "terrain_validation": checked, "source_brush_containment": True}
        grids[h] = grid
        np.savez_compressed(root / f"grid_h{h}.npz", **grid)
        save_json(root / f"target_h{h}.json", [{k: v.tolist() for k, v in t.items()} for t in target])
    carrier = root / "collision_carrier.bsp"
    collision_carrier(root / "clipmerge.bsp", root / "background.bsp", carrier)
    data, checked = splice(carrier, trees)
    output = root / "cliptype_sl_surface_support.bsp"
    output.write_bytes(data)
    result = HullFile(data=data)
    for i in range(15):
        if i not in (1, 9, 14) and result.lumps[i] != files["clipmerge"].lumps[i]:
            raise ValueError(f"Visual or entity data changed: lump {i}")
    exterior = rng.uniform([-1500.] * 3, [1500.] * 3, (100000, 3))
    outside_terrain = np.any(exterior < source_points.min(0) - 40, axis=1) | np.any(
        exterior > source_points.max(0) + 40, axis=1)
    exterior = exterior[outside_terrain]
    for h in (1, 2, 3):
        room = volumes({"brushes": [b for b in original if b["brush"] not in selected]}, h, "legacy")
        target = room + target_by_hull[h]
        counts = error_counts(result.contents(points, h) == -2, margin(points, target), .001)
        if any(counts.values()):
            raise ValueError(f"Final interior classification mismatch: hull {h}, {counts}")
        rays = ray_errors(tree_first_hit(result, ro, rd, 300., h), convex_hit(ro, rd, 300., target))
        require_match(rays)
        grid_hit = 160 - tree_first_hit(result, origins, directions, 220., h)
        require_match(ray_errors(grid_hit, grids[h]["support"].ravel()))
        report["hulls"][str(h)]["final_validation"] = {"interior_points": len(points), **counts,
            "oblique_rays": rays, "vertical_rays": len(origins)}
        if np.any(result.contents(exterior, h) != files["clipmerge"].contents(exterior, h)):
            raise ValueError("Collision outside the selected terrain changed")
        report["hulls"][str(h)]["final_validation"]["unchanged_exterior_points"] = len(exterior)
    source_hull = ConvexHull(source_points)
    source_volumes = [{"planes": np.array([[*f["normal"], f["dist"]] for f in b["faces"]])} for b in terrain]
    candidates = rng.uniform(source_points.min(0), source_points.max(0), (100000, 3))
    candidates = candidates[np.max(candidates @ source_hull.equations[:, :3].T
                                   + source_hull.equations[:, 3], axis=1) < -1e-5]
    gap = candidates[margin(candidates, source_volumes) > 1]
    gap = gap[margin(gap, target_by_hull[1]) > 1]
    gap = gap[result.contents(gap, 1) == -1]
    if not len(gap):
        raise ValueError("No retained nonconvex gap witness; use a nonconvex test assembly")
    gap_point = gap[0]
    if np.max(source_hull.equations[:, :3] @ gap_point + source_hull.equations[:, 3]) > 1e-6:
        raise ValueError("Gap witness is outside the source convex hull")
    report["nonconvex_gap"] = {"point": gap_point.tolist(), "source_empty": True,
                               "inside_source_convex_hull": True, "support_hull1_terrain_empty": True,
                               "final_hull1_empty": True}
    report["output"] = {"path": str(output), "structure": checked, "sha256": sha256(output),
                        "visual_lumps_match": True, "light_bytes": len(result.lumps[8]),
                        "vis_bytes": len(result.lumps[4])}
    report["wall_seconds"] = time.perf_counter() - started
    if sha256(args.source) != report["manifest"]["source_sha256"]:
        raise ValueError("Source MAP changed during the experiment")
    plot(root, grids, report["contacts"])
    save_json(root / "results.json", report)
    write_report(root, report)
    print(json.dumps({"contacts": report["contacts"], "output": report["output"]}, indent=2), flush=True)


if __name__ == "__main__":
    main()
