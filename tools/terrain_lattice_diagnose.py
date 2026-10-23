"""Build bounded continuous roof offsets from an exposed terrain height graph.

This is a cropped experiment, not a replacement for the whole-map collision.
The crop's side walls and bottom are artificial. Legacy distances are applied
to roof planes in a verified max-of-mins representation, not to source brushes.
"""

import argparse
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

from brush_exterior_pipeline import extract_exterior
from brush_surface_pipeline import HALF, load_module, ray_stats, repair, validate
from concave_support_diagnose import intersection, shift_legacy
from legacy_clip_diagnose import convex_hit, save_json, sha256
from legacy_union_pipeline import compile_volumes
from surface_seam_audit import EPSILONS, pair_origins, persistent_steps


def area(polygon):
    a, b = polygon[1:-1] - polygon[0], polygon[2:] - polygon[0]
    return abs(float((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum())) / 2


def split(polygon, line):
    """Split a convex XY polygon by a*x+b*y+c = 0."""
    length = np.linalg.norm(line[:2])
    if length < 1e-12:
        return (polygon, None) if line[2] >= 0 else (None, polygon)
    offsets = (polygon @ line[:2] + line[2]) / length
    if offsets.min() >= -1e-9:
        return polygon, None
    if offsets.max() <= 1e-9:
        return None, polygon
    sides = [[], []]
    for i, point in enumerate(polygon):
        j = (i + 1) % len(polygon)
        a, b = offsets[i], offsets[j]
        if a >= 0:
            sides[0].append(point)
        if a <= 0:
            sides[1].append(point)
        if a * b < 0:
            cut = point + (polygon[j] - point) * a / (a - b)
            for side in sides:
                side.append(cut)
    result = []
    for points in sides:
        points = np.array(points)
        result.append(points if len(points) >= 3 and area(points) > 1e-10 else None)
    return tuple(result)


def crop_lines(bounds):
    xmin, ymin, xmax, ymax = bounds
    return np.array([[-1., 0., xmin], [1., 0., -xmax],
                     [0., -1., ymin], [0., 1., -ymax]])


def crop_polygon(polygon, lines):
    for line in lines:
        _, polygon = split(polygon, line)
        if polygon is None:
            break
    return polygon


def roof_patches(triangles, normals, bounds):
    patches, planes = [], []
    for tri, normal in zip(triangles, normals):
        if normal[2] <= 1e-6:
            continue
        polygon = crop_polygon(tri[:, :2], crop_lines(bounds))
        if polygon is None or area(polygon) < 1e-8:
            continue
        plane = np.r_[normal, tri[0] @ normal]
        index = next((i for i, p in enumerate(planes) if max(abs(p - plane)) < 1e-7), len(planes))
        if index == len(planes):
            planes.append(plane)
        patches.append((polygon, index))
    expected = (bounds[2] - bounds[0]) * (bounds[3] - bounds[1])
    total = sum(area(p) for p, _ in patches)
    if abs(total - expected) > 1e-3:
        raise ValueError(f"Roof does not cover a single rectangular height graph: {total} vs {expected}")
    for i, (a, _) in enumerate(patches):
        for b, _ in patches[i + 1:]:
            if np.any(a.max(0) <= b.min(0) + 1e-8) or np.any(b.max(0) <= a.min(0) + 1e-8):
                continue
            h = ConvexHull(b)
            overlap = crop_polygon(a, h.equations)
            if overlap is not None and area(overlap) > 1e-3:
                raise ValueError("Overlapping roof projections require a different representation")
    return patches, np.array(planes)


def coefficients(planes):
    if np.any(planes[:, 2] <= 1e-6):
        raise ValueError("Only upward roof planes define this height graph")
    return np.c_[-planes[:, :2], planes[:, 3]] / planes[:, 2, None]


def lattice_value(xy, linear, terms):
    xy = np.asarray(xy).reshape(-1, 2)
    values = np.c_[xy, np.ones(len(xy))] @ linear.T
    return np.max([values[:, term].min(1) for term in terms], axis=0)


def build_lattice(patches, planes):
    """Split patches where affine order changes, then verify the lattice on them."""
    linear = coefficients(planes)
    cells, terms = [], set()
    for polygon, owner in patches:
        parts = [polygon]
        for line in linear - linear[owner]:
            parts = [side for p in parts for side in split(p, line) if side is not None]
            if len(parts) > 10000:
                raise ValueError("Affine arrangement too large for a bounded prototype")
        for cell in parts:
            center = np.r_[cell.mean(0), 1.]
            members = frozenset(np.flatnonzero((linear - linear[owner]) @ center >= -1e-7).tolist() + [owner])
            terms.add(members)
            cells.append((cell, owner))
    # min over a superset cannot exceed min over its subset.
    kept = []
    for term in sorted(terms, key=lambda t: (len(t), sorted(t))):
        if not any(other.issubset(term) for other in kept):
            kept.append(term)
    kept = [sorted(term) for term in kept]
    for cell, owner in cells:
        values = np.c_[cell, np.ones(len(cell))] @ (linear - linear[owner]).T
        if not any(values[:, term].min() >= -2e-6 for term in kept):
            raise ValueError("Lattice does not cover an original affine-order cell")
    maximum_excess = 0.
    for polygon, owner in patches:
        domain = ConvexHull(polygon).equations
        for term in kept:
            delta = linear[term] - linear[owner]
            # Maximize min_j(l_j-l_owner) on this source patch.
            result = linprog([0., 0., -1.],
                             A_ub=np.r_[np.c_[domain[:, :2], np.zeros(len(domain))],
                                        np.c_[-delta[:, :2], np.ones(len(delta))]],
                             b_ub=np.r_[-domain[:, 2], delta[:, 2]],
                             bounds=[(None, None)] * 3, method="highs")
            if not result.success:
                raise ValueError(f"Lattice verification failed: {result.message}")
            maximum_excess = max(maximum_excess, -float(result.fun))
    if maximum_excess > 2e-6:
        raise ValueError(f"Lattice exceeds source roof by {maximum_excess}")
    return kept, {"order_cells": len(cells), "terms": len(kept),
                  "source_max_excess_bound": maximum_excess, "cell_coverage_checked": True}


def lattice_volumes(planes, terms, bounds, bottom, half):
    shifted = shift_legacy(planes, half)
    sides = np.array([[a, b, 0., -c] for a, b, c in crop_lines(bounds)] + [[0., 0., -1., -bottom]])
    return [intersection(np.r_[shifted[term], sides]) for term in terms], shifted


def source_cliffs(patches, source, bounds, zhi, length):
    probes, normals = [], []
    for polygon, _ in patches:
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            middle = (a + b) / 2
            if min(middle - bounds[:2]) < .001 or min(np.array(bounds[2:]) - middle) < .001:
                continue
            edge = b - a
            if np.linalg.norm(edge) < 1e-6:
                continue
            probes.append(middle)
            normals.append(np.array([-edge[1], edge[0]]) / np.linalg.norm(edge))
    if not probes:
        return []
    shaped = np.empty((3, len(probes), 2, 3))
    shaped[..., :2] = np.array(probes)[None, :, None, :]
    shaped[..., :2] += EPSILONS[:, None, None, None] * np.array(normals)[None, :, None, :] * np.array([-1., 1.])[None, None, :, None]
    shaped[..., 2] = zhi
    origins = shaped.reshape(-1, 3)
    directions = np.tile([0., 0., -1.], (len(origins), 1))
    tops = (zhi - convex_hit(origins, directions, length, source)).reshape(shaped.shape[:-1])
    finite = np.isfinite(tops).all(axis=(0, 2))
    delta = abs(np.where(np.isfinite(tops), tops, 0.)[..., 0] - np.where(np.isfinite(tops), tops, 0.)[..., 1])
    ids = np.flatnonzero(finite & (delta.min(0) > .05) & (delta[-1] >= .8 * delta[0]))
    result = [{"xy": probes[i].tolist(), "normal_xy": normals[i].tolist(),
               "top_pairs": tops[:, i].tolist(), "smallest_spacing_step": float(delta[-1, i])} for i in ids]
    return sorted(result, key=lambda r: r["smallest_spacing_step"], reverse=True)


def plot(root, profiles):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not profiles:
        return
    fig, axes = plt.subplots(len(profiles), 1, figsize=(10, 3.4 * len(profiles)),
                             squeeze=False, constrained_layout=True)
    for ax, record in zip(axes[:, 0], profiles):
        for key, label, color in (("source", "Source roof", "#666666"),
                                  ("legacy", "Per-brush legacy", "#b95137"),
                                  ("lattice", "Continuous roof offset", "#187c91")):
            ax.plot(record["x"], record[key], label=label, color=color, linewidth=2)
        ax.axvline(record["xy"][0], color="#777777", linestyle=":")
        ax.set(xlabel="X (map units)", ylabel="Top Z (player centre for collision)",
               title=f"Y={record['xy'][1]:g}; cropped diagnostic")
        ax.grid(alpha=.2)
        ax.legend(loc="best")
    fig.savefig(root / "profiles.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True, help="Existing entity-support build")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--entity", type=int, required=True)
    parser.add_argument("--probe", type=float, nargs=2, action="append", required=True)
    parser.add_argument("--radius", type=float, default=64.)
    args = parser.parse_args()
    if args.radius <= 1.:
        parser.error("Choose a crop radius greater than 1 unit")
    base, root = args.build.resolve(), args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    for name in ("geometry.json", "hltools.exe", "exacthull.py", "brushbsp.py"):
        shutil.copy2(base / name, root / name)
    shutil.copy2(__file__, root / "runner_snapshot.py")
    g = json.loads((root / "geometry.json").read_text())
    triangles, normals, source, extraction = extract_exterior(g, args.entity)
    selected = [b for b in g["brushes"] if b["entity"] == args.entity]
    bottom = float(triangles[:, :, 2].min() - 100.)
    zhi = float(triangles[:, :, 2].max() + 500.)
    length = zhi - bottom + 50.
    exact = load_module("lattice_exacthull", root / "exacthull.py")
    exact.WAD = ""
    exact.fmt = lambda p: "( " + " ".join(f"{v:.12f}" for v in p) + " )"
    bb = load_module("lattice_brushbsp", root / "brushbsp.py")
    bb.EPS = 1e-5
    report = {"reference_build": str(base), "runner_sha256": sha256(__file__),
              "entity": args.entity, "crop_radius": args.radius,
              "inputs": {name: sha256(root / name) for name in
                         ("geometry.json", "hltools.exe", "exacthull.py", "brushbsp.py")},
              "scope": "Cropped roof experiments. Artificial crop walls/bottom; no whole-map replacement.",
              "extraction": extraction, "patches": []}
    profiles = []
    for index, xy in enumerate(args.probe):
        x, y = xy
        bounds = [x - args.radius, y - args.radius, x + args.radius, y + args.radius]
        patches, planes = roof_patches(triangles, normals, bounds)
        try:
            terms, proof = build_lattice(patches, planes)
        except ValueError as error:
            record = {"xy": xy, "bounds": bounds, "roof_planes": len(planes),
                      "status": "roof_representation_rejected", "reason": str(error),
                      "source_height_steps": source_cliffs(patches, source, bounds, zhi, length)}
            report["patches"].append(record)
            save_json(root / "results.json", report)
            print(f"Patch {index} rejected: {error}", flush=True)
            continue
        record = {"xy": xy, "bounds": bounds, "status": "validated_crop",
                  "roof_planes": len(planes), "source_roof_planes": planes.tolist(),
                  "terms": terms, "bottom": bottom, "lattice": proof, "hulls": {}}
        xs = np.linspace(bounds[0] + .001, bounds[2] - .001, 129)
        ys = np.linspace(bounds[1] + .001, bounds[3] - .001, 129)
        grid = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
        origins = np.c_[grid, np.full(len(grid), zhi)]
        directions = np.tile([0., 0., -1.], (len(grid), 1))
        original_top = zhi - convex_hit(origins, directions, length, source)
        source_error = abs(original_top - lattice_value(grid, coefficients(planes), terms))
        if not np.isfinite(source_error).all() or source_error.max() > 1e-4:
            raise ValueError("Cropped source graph disagrees with the original closed solid")
        record["source_grid_max_error"] = float(source_error.max())
        for hull in (1, 2, 3):
            volumes, shifted = lattice_volumes(planes, terms, bounds, bottom, HALF[hull])
            tree, seconds = compile_volumes(root / f"patch{index}_h{hull}.map", volumes, exact, root / "hltools.exe")
            wanted = convex_hit(origins, directions, length, volumes)
            got = tree.hit(origins, directions, length)
            if ray_stats(got, wanted)["max_distance_error"] > .001:
                repair(tree, volumes, bb, tolerance=1e-4,
                       extra_rays=(origins, directions, length), local_builder="planes")
                tree.compact()
                got = tree.hit(origins, directions, length)
            checks = validate(tree, volumes)
            stats = ray_stats(got, wanted)
            expected = lattice_value(grid, coefficients(shifted), terms)
            target_error = abs(zhi - wanted - expected)
            if (checks["extra_solid"] or checks["missing_solid"] or checks["exterior_extra_solid"]
                    or checks["ray_checks"]["hit_miss_differences"]
                    or checks["ray_checks"]["errors_over_0_01"]
                    or stats["hit_miss_differences"] or stats["errors_over_0_01"]
                    or target_error.max() > 1e-5 or np.any(expected < original_top - 1e-5)):
                raise ValueError("Cropped offset failed geometry or source-retention checks")
            pairs = pair_origins([xy], zhi)
            o = pairs.reshape(-1, 3)
            d = np.tile([0., 0., -1.], (len(o), 1))
            s = (zhi - convex_hit(o, d, length, source)).reshape(3, 1, 2)
            b = (zhi - tree.hit(o, d, length)).reshape(3, 1, 2)
            if persistent_steps(s, b).any():
                raise ValueError("A witness still has a persistent height step")
            record["hulls"][str(hull)] = {"nodes": len(tree.nodes), "compile_seconds": seconds,
                "geometry_checks": checks, "top_grid": stats, "target_vs_field_max_error": float(target_error.max()),
                "witness_top_pairs": b[:, 0].tolist(), "epsilons": EPSILONS.tolist(),
                "source_top_pairs": s[:, 0].tolist(), "minimum_source_roof_clearance": float((expected - original_top).min())}
            tree.save(root / f"patch{index}_h{hull}.npz")
            save_json(root / f"patch{index}_h{hull}.json", [{k: v.tolist() for k, v in t.items()} for t in volumes])
            print(f"Patch {index}, hull {hull}: {len(planes)} planes, {len(terms)} terms, "
                  f"{len(tree.nodes)} nodes, witness delta {abs(b[-1, 0, 1] - b[-1, 0, 0]):.6f}", flush=True)
            if hull == 1:
                px = np.linspace(x - 24, x + 24, 961)
                pxy = np.c_[px, np.full(len(px), y)]
                po = np.c_[pxy, np.full(len(px), zhi)]
                pd = np.tile([0., 0., -1.], (len(px), 1))
                legacy = [{"planes": np.array(b["legacy_hulls"][0]["planes"])} for b in selected]
                profiles.append({"xy": xy, "x": px, "source": zhi - convex_hit(po, pd, length, source),
                                 "legacy": zhi - convex_hit(po, pd, length, legacy),
                                 "lattice": zhi - tree.hit(po, pd, length)})
        report["patches"].append(record)
        save_json(root / "results.json", report)
    plot(root, profiles)
    print(f"Results: {root}", flush=True)
    if any(r.get("status") == "roof_representation_rejected" for r in report["patches"]):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
