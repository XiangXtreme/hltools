"""Compare compiled collision with the actual per-brush legacy hulls.

The legacytrim CSG sidecar exposes the unmodified legacy volumes as targets.
Its primary intermediates must match a plain legacy CSG invocation byte for
byte. Only the plain legacy intermediates are passed to BSP. This diagnostic
does not enable trimming or modify the compiler or the input MAP.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

import numpy as np

from source_clip_pipeline import HullFile
from source_clip_validate import structure, tree_first_hit


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def read_volumes(path):
    tokens = iter(path.read_text().split())
    sizes = np.array([float(next(tokens)) for _ in range(24)]).reshape(4, 2, 3)
    if (int(next(tokens)), int(next(tokens))) != (1, 3):
        raise ValueError("Expected three legacytrim v1 groups")
    groups = {}

    def volume():
        count = int(next(tokens))
        bounds = np.array([float(next(tokens)) for _ in range(6)]).reshape(2, 3)
        planes = np.array([float(next(tokens)) for _ in range(4 * count)]).reshape(count, 4)
        return {"bounds": bounds, "planes": planes}

    for _ in range(3):
        model, hull, targets, guards = [int(next(tokens)) for _ in range(4)]
        if model != 0 or targets != guards or hull in groups:
            raise ValueError("Diagnostic requires one completely selected solid world model")
        groups[hull] = ([volume() for _ in range(targets)], [volume() for _ in range(guards)])
    if next(tokens, None) is not None or set(groups) != {1, 2, 3}:
        raise ValueError("Unexpected hull metadata")
    return sizes, groups


def margin(points, volumes):
    result = np.full(len(points), np.inf)
    for volume in volumes:
        p = volume["planes"]
        result = np.minimum(result, (points @ p[:, :3].T - p[:, 3]).max(axis=1))
    return result


def convex_hit(origins, directions, length, volumes):
    hit = np.full(len(origins), np.inf)
    for volume in volumes:
        lo, hi = np.zeros(len(origins)), np.full(len(origins), length, dtype=float)
        valid = np.ones(len(origins), bool)
        for a, b, c, distance in volume["planes"]:
            normal = np.array([a, b, c])
            offset, speed = origins @ normal - distance, directions @ normal
            cut = np.divide(-offset, speed, out=np.zeros(len(origins)), where=np.abs(speed) > 1e-12)
            entering, leaving = speed < -1e-12, speed > 1e-12
            lo[entering] = np.maximum(lo[entering], cut[entering])
            hi[leaving] = np.minimum(hi[leaving], cut[leaving])
            valid &= (np.abs(speed) > 1e-12) | (offset <= 0)
        valid &= hi > lo + 1e-9
        hit[valid] = np.minimum(hit[valid], lo[valid])
    return hit


def tops(tree, hull, xy, zlo, zhi):
    origins = np.column_stack((xy, np.full(len(xy), zhi)))
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    return zhi - tree_first_hit(tree, origins, directions, zhi - zlo, hull)


def volume_tops(volumes, xy, zlo, zhi):
    origins = np.column_stack((xy, np.full(len(xy), zhi)))
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    return zhi - convex_hit(origins, directions, zhi - zlo, volumes)


def error_counts(actual, target, tolerance):
    return {"extra_solid": int(np.sum(actual & (target > tolerance))),
            "missing_solid": int(np.sum(~actual & (target < -tolerance)))}


def ray_errors(actual, expected):
    finite = np.isfinite(actual) & np.isfinite(expected)
    delta = np.abs(actual[finite] - expected[finite])
    return {"hit_miss_differences": int(np.sum(np.isfinite(actual) != np.isfinite(expected))),
            "errors_over_0_1": int(np.sum(delta > .1)),
            "max_finite_distance_error": float(delta.max()) if len(delta) else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--compiler", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--xy", nargs=2, type=float, default=[-288, 288])
    parser.add_argument("--z", nargs=2, type=float, default=[-20, 180])
    parser.add_argument("--step", type=float, default=2)
    parser.add_argument("--grid-offset", nargs=2, type=float, default=[.173205, .271828])
    parser.add_argument("--points", type=int, default=100000)
    parser.add_argument("--rays", type=int, default=4096)
    args = parser.parse_args()
    args.source, args.compiler, args.output = [p.resolve() for p in (args.source, args.compiler, args.output)]
    if args.output.exists():
        raise ValueError("Use a new output directory")
    if args.step <= 0 or args.xy[0] >= args.xy[1] or args.z[0] >= args.z[1]:
        raise ValueError("Invalid diagnostic range")
    args.output.mkdir(parents=True)
    source_hash = sha256(args.source)
    source = args.source.read_text(encoding="utf-8")
    if '"info_compile_parameters"' in source:
        raise ValueError("Persistent compile parameters need explicit review")
    pattern = re.compile(r'(?m)^(\s*"zhlt_cliptype"\s+)"[^"\r\n]*"')
    report = {"source": str(args.source), "source_sha256": source_hash,
              "compiler": str(args.compiler), "compiler_sha256": sha256(args.compiler),
              "local_override_count": len(pattern.findall(source)), "commands": [], "hulls": {},
              "scope": "No fill or inside fill; one solid world model; no game movement validation",
              "xy": args.xy, "z": args.z, "step": args.step,
              "grid_offset": args.grid_offset, "seed": 20261023}
    args.output.joinpath("input.map").write_bytes(args.source.read_bytes())

    def compile_stage(name, stage, flags):
        command = [str(args.compiler), stage, str(args.output / (name + ".map")), "-threads", "2", *flags]
        start = time.perf_counter()
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
        elapsed = time.perf_counter() - start
        args.output.joinpath(f"{name}.{stage}.txt").write_bytes(result.stdout)
        report["commands"].append({"name": name, "stage": stage, "args": command,
                                   "seconds": elapsed, "returncode": result.returncode})
        print(f"{name} {stage}: {elapsed:.3f}s, exit {result.returncode}", flush=True)
        if result.returncode:
            raise RuntimeError(f"{name} {stage} failed; inspect log")

    for name, mode in (("legacy", "legacy"), ("audit", "legacytrim"), ("simple", "simple")):
        args.output.joinpath(name + ".map").write_text(
            pattern.sub(lambda match: match[1] + '"' + mode + '"', source), encoding="utf-8")
        compile_stage(name, "csg", ["-cliptype", mode])
    sizes, groups = read_volumes(args.output / "audit.hsz")
    report["hull_sizes"] = sizes.tolist()
    matches = {}
    for suffix in ("p0", "p1", "p2", "p3", "b0", "b1", "b2", "b3", "pln"):
        matches[suffix] = sha256(args.output / ("legacy." + suffix)) == sha256(args.output / ("audit." + suffix))
    if not all(matches.values()):
        raise ValueError(f"Metadata export changed primary CSG data: {matches}")
    report["legacy_vs_audit_primary_intermediates_identical"] = matches
    save_json(args.output / "volumes.json", {str(h): {"legacy": [
        {k: v.tolist() for k, v in item.items()} for item in pair[0]], "box_expansion": [
        {k: v.tolist() for k, v in item.items()} for item in pair[1]]} for h, pair in groups.items()})
    trees = {}
    for name in ("legacy", "simple"):
        compile_stage(name, "bsp", ["-nofill", "-noinsidefill"])
        trees[name] = HullFile(args.output / (name + ".bsp"))
        if len(trees[name].models) != 1:
            raise ValueError("Expected one world model")
    report["structure"] = {name: structure(tree) for name, tree in trees.items()}
    # Avoid integer brush seams, where opposite half-space tie conventions
    # can select different surfaces without describing a finite-volume error.
    grid = np.arange(args.xy[0] + args.grid_offset[0], args.xy[1], args.step)
    ygrid = np.arange(args.xy[0] + args.grid_offset[1], args.xy[1], args.step)
    xx, yy = np.meshgrid(grid, ygrid)
    xy = np.column_stack((xx.ravel(), yy.ravel()))
    rng = np.random.default_rng(report["seed"])
    points = rng.uniform([args.xy[0], args.xy[0], args.z[0]],
                         [args.xy[1], args.xy[1], args.z[1]], (args.points, 3))
    origins = rng.uniform([args.xy[0], args.xy[0], args.z[1] - 20],
                          [args.xy[1], args.xy[1], args.z[1]], (args.rays, 3))
    directions = np.column_stack((rng.uniform(-.5, .5, (args.rays, 2)), -np.ones(args.rays)))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    ray_length = 2 * (args.z[1] - args.z[0])
    surfaces = {"grid": grid, "grid_y": ygrid}
    for hull in (1, 2, 3):
        legacy, exact = groups[hull]
        legacy_margin, exact_margin = margin(points, legacy), margin(points, exact)
        surface = {"legacy_union": volume_tops(legacy, xy, *args.z),
                   "box_union": volume_tops(exact, xy, *args.z)}
        item = {"brush_count": len(legacy), "point_count": len(points), "grid_count": len(xy),
                "rays": len(origins), "plane_margin_tolerance": .05, "bsp": {}}
        for name, tree in trees.items():
            actual = tree.contents(points, hull) == -2
            surface[name] = tops(tree, hull, xy, *args.z)
            expected = legacy if name == "legacy" else exact
            item["bsp"][name] = {
                "vs_legacy_union": error_counts(actual, legacy_margin, .05),
                "vs_box_union": error_counts(actual, exact_margin, .05),
                "rays_vs_own_target": ray_errors(
                    tree_first_hit(tree, origins, directions, ray_length, hull),
                    convex_hit(origins, directions, ray_length, expected))}
        for name, values in surface.items():
            surfaces[f"h{hull}_{name}"] = values.reshape(xx.shape)
        finite = np.logical_and.reduce([np.isfinite(v) for v in surface.values()])
        item["grid_all_four_surfaces_finite"] = int(finite.sum())
        if not finite.any():
            raise ValueError("No comparable top surfaces in diagnostic window")
        residual = surface["legacy"] - surface["legacy_union"]
        shape_delta = surface["legacy_union"] - surface["box_union"]
        finite_ids = np.flatnonzero(finite)
        witness_id = int(finite_ids[np.argmax(shape_delta[finite])])
        witness = np.r_[xy[witness_id], (surface["legacy_union"][witness_id] + surface["box_union"][witness_id]) / 2]
        covering = [i for i, v in enumerate(legacy) if margin(witness[None], [v])[0] < -.01]
        item["legacy_surface_residual"] = {
            "max_absolute": float(np.max(np.abs(residual[finite]))),
            "extra_over_0_1": int(np.sum(residual[finite] > .1)),
            "missing_over_0_1": int(np.sum(residual[finite] < -.1))}
        extra = np.flatnonzero((trees["legacy"].contents(points, hull) == -2) & (legacy_margin > .05))
        if len(extra):
            index = int(extra[np.argmax(legacy_margin[extra])])
            item["bsp_error_witness"] = {"point": points[index].tolist(),
                                         "legacy_union_margin": float(legacy_margin[index])}
        item["expansion_witness"] = {
            "xy": xy[witness_id].tolist(),
            "heights": {name: float(v[witness_id]) for name, v in surface.items()},
            "legacy_union_minus_box_union": float(shape_delta[witness_id]),
            "point": witness.tolist(), "covering_legacy_volume_indices": covering,
            "point_legacy_bsp_contents": int(trees["legacy"].contents(witness[None], hull)[0]),
            "point_legacy_union_margin": float(margin(witness[None], legacy)[0]),
            "point_box_union_margin": float(margin(witness[None], exact)[0])}
        section_xy = np.column_stack((grid, np.full(len(grid), xy[witness_id, 1])))
        section = {"legacy_union": volume_tops(legacy, section_xy, *args.z),
                   "box_union": volume_tops(exact, section_xy, *args.z),
                   **{name: tops(tree, hull, section_xy, *args.z) for name, tree in trees.items()}}
        for name, v in section.items():
            surfaces[f"h{hull}_section_{name}"] = v
        report["hulls"][str(hull)] = item
        save_json(args.output / "summary.json", report)
        print(f"hull {hull}: " + json.dumps(item), flush=True)
    np.savez_compressed(args.output / "surfaces.npz", **surfaces)
    np.savez_compressed(args.output / "probes.npz", points=points, origins=origins, directions=directions)
    plot(args.output, report, surfaces)
    if sha256(args.source) != source_hash or sha256(args.compiler) != report["compiler_sha256"]:
        raise ValueError("Input or compiler changed during experiment")
    report["source_and_compiler_unchanged"] = True
    save_json(args.output / "summary.json", report)


def plot(output, report, surfaces):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), dpi=160)
    grid = surfaces["grid"]
    for row, hull in enumerate((1, 3)):
        witness = report["hulls"][str(hull)]["expansion_witness"]
        ax = axes[row, 0]
        for name, label, color, style in (
                ("legacy_union", "Legacy brush union", "#16875b", "-"),
                ("legacy", "Compiled legacy BSP", "#c43c39", "--"),
                ("box_union", "Brush + player-box union", "#286bb2", "-"),
                ("simple", "Compiled simple BSP", "#977a18", ":")):
            ax.plot(grid, surfaces[f"h{hull}_section_{name}"], label=label, color=color, linestyle=style)
        ax.axvline(witness["xy"][0], color="#555555", linewidth=.7)
        ax.set(title=f"Hull {hull}, Y = {witness['xy'][1]:g}", xlabel="X", ylabel="Player origin Z")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
        ax = axes[row, 1]
        delta = surfaces[f"h{hull}_legacy"] - surfaces[f"h{hull}_legacy_union"]
        ygrid = surfaces["grid_y"]
        im = ax.imshow(delta, origin="lower", extent=[grid[0], grid[-1], ygrid[0], ygrid[-1]],
                       cmap="RdBu_r", vmin=-1, vmax=1)
        ax.plot(*witness["xy"], marker="+", color="#151515")
        ax.set(title=f"Hull {hull}: BSP minus legacy union", xlabel="X", ylabel="Y")
        fig.colorbar(im, ax=ax, label="Height difference (map units)")
    fig.tight_layout()
    fig.savefig(output / "comparison.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
