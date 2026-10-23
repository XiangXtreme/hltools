"""Compare boundary-triangle hulls against the recorded legacy and closed hulls."""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from brush_surface_pipeline import Tree, expanded, HALF, ray_stats, probes, require_command, splice
from legacy_clip_diagnose import convex_hit, save_json
from legacy_union_pipeline import override_cliptype
from source_clip_pipeline import HullFile
from source_clip_validate import structure, tree_first_hit


def compile_simple(root):
    manifest = json.loads((root / "manifest.json").read_text())
    source = (root / "source_snapshot.map").read_text(encoding="utf-8")
    path = root / "ordinary_simple.map"
    if not path.with_suffix(".bsp").exists():
        path.write_text(override_cliptype(source, "simple"), encoding="utf-8")
        compiler = Path(manifest["baseline"]) / "compiler/hltools.exe"
        require_command([compiler, "csg", path, "-cliptype", "simple", "-threads", "2"],
                        root / "ordinary_simple.csg.txt")
        require_command([compiler, "bsp", path, "-threads", "2"], root / "ordinary_simple.bsp.txt")
    return path.with_suffix(".bsp")


def build_report(root):
    manifest = json.loads((root / "manifest.json").read_text())
    result = json.loads((root / "results.json").read_text())
    legacy_root, closed_root = Path(manifest["baseline"]), Path(manifest["closed"])
    previous = json.loads((legacy_root / "results.json").read_text())
    old_closed = json.loads((closed_root / "results.json").read_text())
    legacy_comparison = root / "untitledx_legacy_union_comparison.bsp"
    data, _ = splice(closed_root / "background_lit.bsp",
                     {h: Tree.load(legacy_root / f"legacy_union_h{h}.npz") for h in (1, 2, 3)})
    legacy_comparison.write_bytes(data)
    files = {"Ordinary legacy": legacy_root / "legacy.bsp",
             "Ordinary simple": compile_simple(root),
             "Legacy union": legacy_comparison,
             "Closed box": closed_root / "untitledx_closed.bsp",
             "Exterior shell": Path(result["outputs"]["shell"]["path"]),
             "Upward terrain": Path(result["outputs"]["terrain"]["path"])}
    trees = {name: HullFile(path) for name, path in files.items()}
    comparison = {"structures": {name: structure(tree) for name, tree in trees.items()}, "hulls": {}}
    background = HullFile(closed_root / "background_lit.bsp")
    for name, tree in trees.items():
        if name in ("Ordinary legacy", "Ordinary simple"):
            continue
        if any(tree.lumps[i] != background.lumps[i] for i in range(15) if i not in (1, 9, 14)):
            raise ValueError(f"Background mismatch: {name}")
    corners = np.array(list(itertools.product((-1., 1.), repeat=3)))
    raw = np.load(root / "exterior.npz")
    geometry = json.loads((legacy_root / "geometry.json").read_text())
    brushes = [b for b in geometry["brushes"] if b["entity"] == 3]
    for hull in (1, 2, 3):
        grid = np.load(root / f"tops_h{hull}.npz")
        x, y = grid["x"], grid["y"]
        zhi = max(float(np.max(grid[key][np.isfinite(grid[key])]))
                  for key in ("legacy", "box", "shell_target", "terrain_target")) + 100
        xy = np.stack(np.meshgrid(x, y), -1).reshape(-1, 2)
        origins = np.c_[xy, np.full(len(xy), zhi)]
        directions = np.tile([0., 0., -1.], (len(origins), 1))
        length = 1500.
        target = zhi - grid["shell_target"].ravel()
        background_hit = tree_first_hit(background, origins, directions, length, hull)
        mask = np.isfinite(target) & (target < background_hit - .01)
        record = {"terrain_first_rays": int(mask.sum()), "bsp_vs_triangle_target": {}}
        compiled_tops = {}
        for name, tree in trees.items():
            actual = tree_first_hit(tree, origins, directions, length, hull)
            stats = ray_stats(actual[mask], target[mask])
            ids = np.flatnonzero(mask & np.isfinite(actual))
            if len(ids):
                index = ids[np.argmax(np.abs(actual[ids] - target[ids]))]
                stats["max_error_witness"] = {"xy": xy[index].tolist(),
                                              "bsp_height": float(zhi - actual[index]),
                                              "triangle_height": float(zhi - target[index])}
            record["bsp_vs_triangle_target"][name] = stats
            compiled_tops[name] = (zhi - actual).reshape(257, 257)
        np.savez_compressed(root / f"compiled_tops_h{hull}.npz", x=x, y=y, **compiled_tops)
        shapes = []
        for brush in brushes:
            vertices = np.concatenate([face["vertices"] for face in brush["faces"]])
            _, ids = np.unique(np.round(vertices, 9), axis=0, return_index=True)
            shapes.append(vertices[np.sort(ids)])
        box = expanded(shapes, HALF[hull], corners)
        unsnapped = expanded(raw["triangles"], HALF[hull], corners)
        _, o, d, limit = probes(box, 582910, 1, 8192)
        record["unsnapped_shell_vs_closed_geometry"] = ray_stats(convex_hit(o, d, limit, unsnapped),
                                                                 convex_hit(o, d, limit, box))
        # Compare actual terrain BSPs against the closed-box oracle on the same
        # exterior rays, independently of the room walls.
        record["terrain_bsp_vs_closed_box"] = {}
        wanted = convex_hit(o, d, limit, box)
        for name in ("shell", "terrain"):
            tree = Tree.load(root / f"{name}_h{hull}.npz")
            record["terrain_bsp_vs_closed_box"][name] = ray_stats(tree.hit(o, d, limit), wanted)
        comparison["hulls"][str(hull)] = record
    save_json(root / "comparison.json", comparison)
    lines = ["# Brush exterior triangle experiment", "",
             f"Input: `{manifest['source']}`", "",
             f"MAP SHA256: `{manifest['source_sha256']}`", "",
             "The union boundary is obtained by subtracting other convex brushes from each face.",
             "Opposite-facing contact faces disappear; duplicate coplanar faces have one owner.",
             "Convex remainder polygons are triangulated, then snapped to 1/16 units like build_boreas_clips.py.",
             "Collapsed triangles are removed. The terrain variant keeps upward-facing boundary triangles.", ""]
    e = result["extraction"]
    lines += [f"{e['brushes']} brushes, {e['input_faces']} original faces, {e['exterior_fragments']} boundary fragments,",
              f"{e['triangles']} unsnapped triangles ({e['upward_triangles']} upward),",
              f"{e['compiled_triangles']} snapped triangles ({e['compiled_upward_triangles']} upward).",
              f"Internal/duplicate face area removed: {100 * e['removed_area'] / e['input_face_area']:.3f}%.",
              f"Extraction: {e['seconds']:.3f}s; maximum snapped vertex displacement: {e['snap_max_vertex_displacement']:.6f}.", "",
              "| Variant | Hull 1 | Hull 2 | Hull 3 | Total clipnodes |",
              "| --- | ---: | ---: | ---: | ---: |"]
    for name, checked in comparison["structures"].items():
        values = [checked["hulls"][str(h)]["reachable"] for h in (1, 2, 3)]
        lines.append(f"| {name} | " + " | ".join(map(str, values)) + f" | {checked['stored_nodes']} |")
    lines += ["", "## Build Costs", "",
              "Timings exclude geometry extraction, background BSP/VIS/RAD and independent validation.",
              "Historical baseline timings are from the previous run on this machine, not a controlled benchmark.", "",
              "| Variant | MAP generation + CSG/BSP | Local repair | Total |", "| --- | ---: | ---: | ---: |"]
    simple_seconds = sum(json.loads((root / f"ordinary_simple.{stage}.command.json").read_text())["seconds"]
                         for stage in ("csg", "bsp"))
    legacy_seconds = sum(previous["commands"][stage] for stage in ("legacy_csg", "legacy_bsp"))
    lines.append(f"| Ordinary simple | {simple_seconds:.3f}s | 0s | {simple_seconds:.3f}s |")
    lines.append(f"| Ordinary legacy (previous) | {legacy_seconds:.3f}s | 0s | {legacy_seconds:.3f}s |")
    for name, key in (("Exterior shell", "shell"), ("Upward terrain", "terrain")):
        generate = sum(v[key]["generation_and_compile_seconds"] for v in result["hulls"].values())
        repair = sum(v[key]["repair"]["seconds"] + v[key].get("plane_repair", {}).get("seconds", 0)
                     for v in result["hulls"].values())
        lines.append(f"| {name} | {generate:.3f}s | {repair:.3f}s | {generate + repair:.3f}s |")
    generate = sum(v["variants"]["closed"]["generation_and_compile_seconds"] for v in old_closed["hulls"].values())
    repair = sum(v["variants"]["closed"]["repair"]["seconds"] for v in old_closed["hulls"].values())
    lines.append(f"| Closed box (previous) | {generate:.3f}s | {repair:.3f}s | {generate + repair:.3f}s |")
    generate = sum(sum(v["compile_seconds"].values()) for v in previous["hulls"].values())
    repair = sum(v["repair"]["seconds"] for v in previous["hulls"].values())
    lines.append(f"| Legacy union (previous, planes already generated) | {generate:.3f}s | {repair:.3f}s | {generate + repair:.3f}s |")
    lines += ["", f"Current runner wall time including validation: {result['wall_seconds']:.3f}s.", "",
              "Build phase sums include imported checkpoints, when present. Runner wall time excludes their",
              "previous build and discarded repair attempts; see per-hull imported_checkpoint metadata.", "",
              "## Independent Validation", "",
              "Every variant/hull uses 250,000 points, 8,192 arbitrary exterior rays and about 96,000",
              "outside-bounds points. Threshold: 0.01 units. The reference triangle SAT implementation",
              "independently cross-checks expanded convex volumes on 2,048 rays per variant/hull.", "",
              "| Hull | Variant | Extra/missing points | Ray errors >0.01 | Max ray error | Max top-grid error | Repair fallback |",
              "| --- | --- | ---: | ---: | ---: | ---: | --- |"]
    for hull, value in result["hulls"].items():
        for name in ("shell", "terrain"):
            v = value[name]
            hold = v["holdout"]
            rays = hold["ray_checks"]
            top = value["top_contact"][name]["bsp_vs_target"]
            lines.append(f"| {hull} | {name} | {hold['extra_solid']}/{hold['missing_solid']} | "
                         f"{rays['errors_over_0_01']} | {rays['max_distance_error']:.6f} | "
                         f"{top['max_distance_error']:.6f} | {'half-space union' if 'plane_repair' in v else 'none'} |")
    lines += ["", "Raw compiler and fragment repair results remain in results.json. Residual errors above 0.01",
              "are explicitly retained. This diagnostic permits ray distance error up to 0.1 while rejecting",
              "sampled extra/missing solid and hit/miss mismatches. Larger failures trigger a recorded local",
              "half-space union fallback, which differs from the original fragment builder. A discarded",
              "fallback is not part of the output. Compiler plane canonicalization is retained.", "",
              "## Boundary Comparison", "",
              "The following geometry comparison uses unsnapped triangles to separate representation",
              "from snapping/compiler errors. Rays start outside the full grown-brush bounds.", "",
              "| Hull | Unsnapped shell vs closed box, max ray difference | Hit/miss differences |",
              "| --- | ---: | ---: |"]
    for hull, record in comparison["hulls"].items():
        v = record["unsnapped_shell_vs_closed_geometry"]
        lines.append(f"| {hull} | {v['max_distance_error']:.12g} | {v['hit_miss_differences']} |")
    lines += ["", "For a solid set S and the same player box B, a box coming from outside first touches",
              "the boundary of S when it first touches S. Thus boundary-triangle expansion can share",
              "the external contact of closed-brush box expansion. Removing interior brush seams does",
              "not introduce a different player shape or a different slope-contact rule. The shell's deep",
              "interior can be empty, unlike a closed solid. Upward-only terrain also omits side/bottom collision.", "",
              "Top checks (comparison.json) exclude rays hitting the background before terrain; ordinary legacy",
              "and repaired legacy can legitimately disagree with the triangle target because their",
              "expanded geometric planes differ. Such differences are not all compiler errors.", "",
              "The actual simple BSP is measured separately from the mathematical closed-box target.",
              "Its differences must not be dismissed just because the intended expansion rules are similar.", "",
              "| Hull | Actual BSP | Terrain-first rays | Errors >0.01 vs shell target | Max difference |",
              "| --- | --- | ---: | ---: | ---: |"]
    for hull, record in comparison["hulls"].items():
        for name, value in record["bsp_vs_triangle_target"].items():
            lines.append(f"| {hull} | {name} | {record['terrain_first_rays']} | "
                         f"{value['errors_over_0_01']} | {value['max_distance_error']:.6f} |")
    lines += ["",
              "## Outputs", ""]
    for name, info in result["outputs"].items():
        filename = Path(info["path"]).name
        lines.append(f"- [{filename}]({filename})")
    lines.append(f"- [{legacy_comparison.name}]({legacy_comparison.name}) (previous terrain tree in the common background)")
    lines += ["", f"Both outputs retain {result['light_bytes']} lighting bytes and {result['vis_bytes']} VIS bytes.",
              "Visible geometry, textures, entities, VIS and lightmaps are byte-identical across the compared",
              "closed, repaired legacy, shell and upward outputs. Player starts, child indices, cycles,",
              "root lower bounds, node budget and 150,000-point background union per hull pass.",
              "Source MAP and game BSP are untouched. These are sampled checks; engine movement is untested.", "",
              "![Standing hull contact comparison](surface_comparison.png)", ""]
    (root / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    plot(root)
    print(json.dumps(comparison, indent=2))


def plot(root):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    grid = np.load(root / "tops_h1.npz")
    extent = [grid["x"][0], grid["x"][-1], grid["y"][0], grid["y"][-1]]
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    for ax, key, title in ((axes[0, 0], "legacy", "Actual legacy union / hull 1"),
                            (axes[0, 1], "shell", "Exterior triangles / hull 1")):
        data = grid[key]
        artist = ax.imshow(np.where(np.isfinite(data), data, np.nan), origin="lower", extent=extent,
                           vmin=-250, vmax=400, cmap="terrain", aspect="equal")
        fig.colorbar(artist, ax=ax, label="First contact Z")
        ax.set_title(title)
    a, b = grid["shell"], grid["box"]
    delta = np.full_like(a, np.nan)
    valid = np.isfinite(a) & np.isfinite(b)
    delta[valid] = a[valid] - b[valid]
    picture = axes[1, 0].imshow(delta, origin="lower", extent=extent, cmap="RdBu_r", vmin=-.05, vmax=.05)
    fig.colorbar(picture, ax=axes[1, 0], label="Z difference (units)")
    axes[1, 0].set_title("Exterior triangles minus closed box")
    compiled = np.load(root / "compiled_tops_h1.npz")
    finite = np.isfinite(grid["shell_target"]) & np.isfinite(grid["box"])
    finite &= np.abs(compiled["Closed box"] - np.where(finite, grid["box"], 0)) < .1
    diff = np.zeros_like(a)
    diff[finite] = np.abs(compiled["Ordinary simple"][finite] - grid["shell_target"][finite])
    row = int(np.unravel_index(np.argmax(diff), diff.shape)[0])
    for key, label, color, style in (("legacy", "Legacy union", "#b54734", "-"),
                                     ("shell", "Exterior triangles", "#19795c", "-"),
                                     ("terrain", "Upward terrain", "#925bac", ":"),
                                     ("box", "Closed box", "#303030", "--")):
        data = grid[key][row]
        axes[1, 1].plot(grid["x"], np.where(finite[row] & np.isfinite(data), data, np.nan), label=label, color=color, linestyle=style)
    data = compiled["Ordinary simple"][row]
    axes[1, 1].plot(grid["x"], np.where(finite[row] & np.isfinite(data), data, np.nan), color="#3577c1", linestyle="-.", label="Actual simple BSP")
    axes[1, 1].set_title(f"Cross section at Y = {grid['y'][row]:.3f}")
    axes[1, 1].set_ylabel("First contact Z")
    axes[1, 1].legend()
    for ax in axes.flat:
        ax.set_xlabel("X")
    for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
        ax.set_ylabel("Y")
    fig.savefig(root / "surface_comparison.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    build_report(parser.parse_args().experiment.resolve())
