"""Extract a brush union boundary and run the reference triangle hull pipeline."""

import argparse
import itertools
import json
import os
from pathlib import Path
import shutil
import time
from types import SimpleNamespace

os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

import numpy as np

import brush_surface_pipeline as pipeline
from legacy_clip_diagnose import convex_hit, margin, save_json, sha256
from source_clip_pipeline import HullFile, SCRIPTS
import source_clip_validate as validator

EPS = 1e-6


def area(polygon):
    if len(polygon) < 3:
        return 0.
    return float(np.linalg.norm(np.cross(polygon[1:-1] - polygon[0],
                                         polygon[2:] - polygon[0]).sum(0)) / 2)


def split_polygon(polygon, normal, distance):
    offsets = polygon @ normal - distance
    offsets[np.abs(offsets) <= EPS] = 0
    if offsets.min() >= 0:
        return polygon, None
    if offsets.max() <= 0:
        return None, polygon
    front, back = [], []
    for i, point in enumerate(polygon):
        following = (i + 1) % len(polygon)
        a, b = offsets[i], offsets[following]
        if a >= 0:
            front.append(point)
        if a <= 0:
            back.append(point)
        if a * b < 0:
            cut = point + (polygon[following] - point) * (a / (a - b))
            front.append(cut)
            back.append(cut)
    result = []
    for values in (front, back):
        values = np.array(values)
        keep = np.linalg.norm(values - np.roll(values, 1, axis=0), axis=1) > 1e-9
        values = values[keep]
        result.append(values if area(values) > 1e-8 else None)
    return tuple(result)


def subtract_brush(polygon, planes, normal, owner, other):
    offsets = polygon @ planes[:, :3].T - planes[:, 3]
    if np.any(offsets.min(0) > EPS):
        return [polygon]
    coplanar = (np.max(np.abs(offsets), axis=0) <= EPS) & (np.abs(planes[:, :3] @ normal) > 1 - 1e-10)
    if np.any((offsets.min(0) >= -EPS) & ~coplanar):
        return [polygon]
    # Same-facing duplicate boundary belongs to the first brush. Opposite-facing
    # contact is internal and is removed from both brushes.
    if other > owner and np.any(coplanar & (planes[:, :3] @ normal > 1 - 1e-8)):
        return [polygon]
    outside, inside = [], polygon
    for plane, on_plane in zip(planes, coplanar):
        if on_plane:
            continue
        front, inside = split_polygon(inside, plane[:3], plane[3])
        if front is not None:
            outside.append(front)
        if inside is None:
            return [polygon]
    return outside


def extract_exterior(geometry, entity):
    brushes = [b for b in geometry["brushes"] if b["entity"] == entity]
    if not brushes or any(b["contents"] != -2 for b in brushes):
        raise ValueError("Select an entity containing only solid brushes")
    volumes = []
    for brush in brushes:
        vertices = np.concatenate([f["vertices"] for f in brush["faces"]])
        _, unique = np.unique(np.round(vertices, 9), axis=0, return_index=True)
        vertices = vertices[np.sort(unique)]
        planes = np.array([[*f["normal"], f["dist"]] for f in brush["faces"]])
        volumes.append({"vertices": vertices, "planes": planes,
                        "bounds": np.array([vertices.min(0), vertices.max(0)])})
    triangles, normals, owners = [], [], []
    input_area, exterior_area, input_faces, fragments = 0., 0., 0, 0
    for i, brush in enumerate(brushes):
        for face in brush["faces"]:
            polygon, normal = np.array(face["vertices"]), np.array(face["normal"])
            input_faces += 1
            input_area += area(polygon)
            low, high = polygon.min(0), polygon.max(0)
            pieces = [polygon]
            for j, volume in enumerate(volumes):
                if j == i or np.any(volume["bounds"][0] > high + EPS) or np.any(volume["bounds"][1] < low - EPS):
                    continue
                pieces = [p for piece in pieces for p in
                          subtract_brush(piece, volume["planes"], normal, i, j)]
                if not pieces:
                    break
            for piece in pieces:
                fragments += 1
                exterior_area += area(piece)
                for k in range(1, len(piece) - 1):
                    tri = piece[[0, k, k + 1]]
                    if area(tri) <= 1e-8:
                        continue
                    if np.cross(tri[1] - tri[0], tri[2] - tri[0]) @ normal < 0:
                        tri = tri[[0, 2, 1]]
                    triangles.append(tri)
                    normals.append(normal)
                    owners.append(i)
    triangles, normals = np.array(triangles), np.array(normals)
    # Probe several points inside every triangle, away from its edges. Both sides
    # of an exposed boundary must have different union occupancy.
    weights = np.array([[1/3, 1/3, 1/3], [.8, .1, .1], [.1, .8, .1], [.1, .1, .8]])
    points = np.einsum("ki,nij->nkj", weights, triangles).reshape(-1, 3)
    directions = np.repeat(normals, len(weights), axis=0)
    # Fixed offsets can cross a different face in very thin exposed slivers.
    # Limit each probe displacement to before its next plane crossing.
    steps = np.full(len(points), .0001)
    for volume in volumes:
        offsets = points @ volume["planes"][:, :3].T - volume["planes"][:, 3]
        velocity = directions @ volume["planes"][:, :3].T
        moving = (np.abs(offsets) > 1e-9) & (np.abs(velocity) > 1e-12)
        crossings = np.divide(np.abs(offsets), np.abs(velocity),
                              out=np.full_like(offsets, np.inf), where=moving)
        steps = np.minimum(steps, crossings.min(1) * .1)
    inside = margin(points - directions * steps[:, None], volumes)
    outside = margin(points + directions * steps[:, None], volumes)
    bad = (inside > 1e-5) | (outside < -1e-5)
    if bad.any():
        raise ValueError(f"Extracted boundary failed orientation/occupancy checks: {bad.sum()} probes; "
                         f"max inside {inside.max()}, min outside {outside.min()}; "
                         f"point {points[np.flatnonzero(bad)[0]].tolist()}, "
                         f"owner {owners[np.flatnonzero(bad)[0] // len(weights)]}")
    return triangles, normals, volumes, {
        "brushes": len(brushes), "input_faces": input_faces, "exterior_fragments": fragments,
        "triangles": len(triangles), "upward_triangles": int(np.sum(normals[:, 2] > EPS)),
        "input_face_area": input_area, "exterior_area": exterior_area,
        "removed_area": input_area - exterior_area, "boundary_probes": len(points),
        "boundary_probe_failures": int(bad.sum()), "plane_epsilon": EPS,
        "boundary_probe_tolerance": 1e-5, "max_boundary_probe_error": float(max(0., inside.max(), -outside.min())),
    }


def sat_check(triangles, volumes, hull):
    tlo, thi = triangles.min(1), triangles.max(1)
    half = pipeline.HALF[hull]
    def near(a, b, unused):
        return np.flatnonzero(np.all(thi >= a - half, axis=1) & np.all(tlo <= b + half, axis=1))
    validator.FIX = SimpleNamespace(_tris=triangles, X=SimpleNamespace(HALF=pipeline.HALF), near=near)
    _, origins, directions, length = pipeline.probes(volumes, 635721, 1, 2048)
    sat = validator.triangle_first_hit(origins, directions, length, hull)
    convex = convex_hit(origins, directions, length, volumes)
    result = pipeline.ray_stats(convex, sat)
    if result["hit_miss_differences"] or result["max_distance_error"] > 1e-6:
        raise ValueError(f"Triangle SAT disagrees with expanded convex volumes: {result}")
    return {"rays": len(origins), **result}


def contacts(root, trees, volumes, legacy, box, hull):
    low = np.min([v["bounds"][0] for v in box], axis=0)
    high = np.max([v["bounds"][1] for v in box], axis=0)
    xs = np.linspace(low[0] + .173205, high[0] - .271828, 257)
    ys = np.linspace(low[1] + .271828, high[1] - .173205, 257)
    xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
    origins = np.c_[xy, np.full(len(xy), high[2] + 40)]
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    length = float(high[2] - low[2] + 80)
    hits = {"legacy": convex_hit(origins, directions, length, legacy),
            "box": convex_hit(origins, directions, length, box)}
    result = {}
    for name in trees:
        hits[name + "_target"] = convex_hit(origins, directions, length, volumes[name])
        hits[name] = trees[name].hit(origins, directions, length)
        result[name] = {"bsp_vs_target": pipeline.ray_stats(hits[name], hits[name + "_target"]),
                        "target_vs_box": pipeline.ray_stats(hits[name + "_target"], hits["box"]),
                        "target_vs_legacy": pipeline.ray_stats(hits[name + "_target"], hits["legacy"])}
    np.savez_compressed(root / f"tops_h{hull}.npz", x=xs, y=ys,
                        **{name: (origins[:, 2] - values).reshape(257, 257) for name, values in hits.items()})
    return result


def acceptable_for_comparison(value):
    rays = value["ray_checks"]
    return not (value["extra_solid"] or value["missing_solid"] or value["exterior_extra_solid"]
                or rays["hit_miss_differences"] or rays["max_distance_error"] > .1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "baseline", "closed", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--reuse", type=Path, help="Import validated checkpoints from matching experiment inputs")
    args = parser.parse_args()
    args.source, args.baseline, args.closed, root = [p.resolve() for p in
                                                   (args.source, args.baseline, args.closed, args.output)]
    started = time.perf_counter()
    root.mkdir(parents=True, exist_ok=True)
    baseline_manifest = json.loads((args.baseline / "manifest.json").read_text())
    closed_manifest = json.loads((args.closed / "manifest.json").read_text())
    source_hash = sha256(args.source)
    if source_hash != baseline_manifest["source_sha256"] or source_hash != closed_manifest["source_sha256"]:
        raise ValueError("Comparison source snapshots differ")
    manifest = {"source": str(args.source), "source_sha256": source_hash, "entity": 3,
                "baseline": str(args.baseline), "closed": str(args.closed),
                "runner_sha256": sha256(__file__), "shared_pipeline_sha256": sha256(pipeline.__file__),
                "compiler_sha256": sha256(args.baseline / "compiler/hltools.exe"),
                "scripts": {s: sha256(args.baseline / "scripts" / s) for s in SCRIPTS},
                "geometry_sha256": sha256(args.baseline / "geometry.json"),
                "snap_grid": 1 / 16, "background_sha256": sha256(args.closed / "background_lit.bsp"),
                "ray_distance_comparison_limit": .1, "reuse": str(args.reuse.resolve()) if args.reuse else None}
    if args.reuse:
        previous_manifest = json.loads((args.reuse / "manifest.json").read_text())
        for key in ("source_sha256", "geometry_sha256", "compiler_sha256", "scripts", "snap_grid", "background_sha256"):
            if previous_manifest[key] != manifest[key]:
                raise ValueError(f"Checkpoint inputs differ: {key}")
    if (root / "manifest.json").exists() and json.loads((root / "manifest.json").read_text()) != manifest:
        raise ValueError("Inputs changed; use a new experiment directory")
    save_json(root / "manifest.json", manifest)
    shutil.copy2(args.source, root / "source_snapshot.map")
    shutil.copy2(__file__, root / "runner_snapshot.py")
    geometry = json.loads((args.baseline / "geometry.json").read_text())
    extract_started = time.perf_counter()
    triangles, normals, original, extraction = extract_exterior(geometry, 3)
    extraction["seconds"] = time.perf_counter() - extract_started
    np.savez_compressed(root / "exterior.npz", triangles=triangles, normals=normals)
    snapped = np.round(triangles * 16) / 16
    keep = np.linalg.norm(np.cross(snapped[:, 1] - snapped[:, 0], snapped[:, 2] - snapped[:, 0]), axis=1) > 1e-8
    extraction["snap_max_vertex_displacement"] = float(np.linalg.norm(snapped - triangles, axis=2).max())
    extraction["snap_degenerate_triangles"] = int(np.sum(~keep))
    triangles, normals = snapped[keep], normals[keep]
    keys = [tuple(sorted(map(tuple, t))) for t in triangles]
    seen, keep = set(), []
    for i, key in enumerate(keys):
        if key not in seen:
            seen.add(key)
            keep.append(i)
    triangles, normals = triangles[keep], normals[keep]
    extraction["compiled_triangles"] = len(triangles)
    extraction["compiled_upward_triangles"] = int(np.sum(normals[:, 2] > EPS))
    if args.reuse:
        previous_triangles = np.load(args.reuse / "compiled_exterior.npz")
        if not np.array_equal(triangles, previous_triangles["triangles"]) or not np.array_equal(normals, previous_triangles["normals"]):
            raise ValueError("Checkpoint triangle geometry differs")
    np.savez_compressed(root / "compiled_exterior.npz", triangles=triangles, normals=normals)
    save_json(root / "extraction.json", extraction)
    print("extraction", json.dumps(extraction), flush=True)
    exact = pipeline.load_module("exterior_exacthull", args.baseline / "scripts/exacthull.py")
    bb = pipeline.load_module("exterior_brushbsp", args.baseline / "scripts/brushbsp.py")
    exact.HALF = pipeline.HALF
    exact.WAD = "D:/steam/steamapps/common/JACK/mapper/wad/sdhlt.wad"
    compiler = args.baseline / "compiler/hltools.exe"
    legacy_data = json.loads((args.baseline / "volumes.json").read_text())
    report = {"extraction": extraction, "hulls": {}, "outputs": {}}
    all_trees = {name: {} for name in ("shell", "terrain")}
    for hull in (1, 2, 3):
        trees, volumes = {}, {}
        record = {}
        for name, shapes in (("shell", triangles), ("terrain", triangles[normals[:, 2] > EPS])):
            prefix = root / f"{name}_h{hull}"
            print(f"{name} hull {hull}: {len(shapes)} triangles", flush=True)
            volumes[name] = pipeline.expanded(shapes, pipeline.HALF[hull], exact.CORNERS)
            imported = args.reuse / prefix.name if args.reuse else None
            if imported and imported.with_suffix(".npz").exists():
                checkpoint = imported.with_suffix(".npz")
                tree = pipeline.Tree.load(checkpoint)
                value = json.loads(imported.with_suffix(".json").read_text())
                value["holdout"] = pipeline.validate(tree, volumes[name])
                value["imported_checkpoint"] = {"path": str(checkpoint.resolve()), "sha256": sha256(checkpoint)}
                if not acceptable_for_comparison(value["holdout"]):
                    raise ValueError("Imported checkpoint failed comparison limits")
                value["triangle_sat_crosscheck"] = sat_check(shapes, volumes[name], hull)
                tree.save(prefix.with_suffix(".npz"))
                save_json(prefix.with_suffix(".json"), value)
            elif imported and imported.with_name(imported.name + "_fragments.npz").exists():
                checkpoint = imported.with_name(imported.name + "_fragments.npz")
                tree = pipeline.Tree.load(checkpoint)
                value = json.loads(imported.with_suffix(".failed.json").read_text())
                value["discarded_plane_repair"] = value.pop("plane_repair")
                value["holdout"] = pipeline.validate(tree, volumes[name])
                value["imported_checkpoint"] = {"path": str(checkpoint.resolve()), "sha256": sha256(checkpoint)}
                if not acceptable_for_comparison(value["holdout"]):
                    raise ValueError("Imported checkpoint failed comparison limits")
                value["triangle_sat_crosscheck"] = sat_check(shapes, volumes[name], hull)
                tree.save(prefix.with_suffix(".npz"))
                save_json(prefix.with_suffix(".json"), value)
            elif prefix.with_suffix(".npz").exists():
                tree = pipeline.Tree.load(prefix.with_suffix(".npz"))
                value = json.loads(prefix.with_suffix(".json").read_text())
            else:
                tree, value = pipeline.compile_expanded(prefix.with_suffix(".map"), shapes, hull, exact, compiler)
                value["raw_nodes"] = len(tree.nodes)
                value["raw_validation"] = pipeline.validate(tree, volumes[name])
                value["repair"] = pipeline.repair(tree, volumes[name], bb)
                value["holdout"] = pipeline.validate(tree, volumes[name])
                # Retain the reference fragment result. If it fails the independent
                # checks, record that failure and retry with local half-space union.
                if not acceptable_for_comparison(value["holdout"]):
                    tree.save(root / f"{name}_h{hull}_fragments.npz")
                    value["fragment_holdout"] = value["holdout"]
                    bb.EPS = 1e-5
                    value["plane_repair"] = pipeline.repair(tree, volumes[name], bb, tolerance=1e-4,
                                                            local_builder="planes")
                    bb.EPS = 1e-3
                    value["holdout"] = pipeline.validate(tree, volumes[name])
                if not acceptable_for_comparison(value["holdout"]):
                    save_json(prefix.with_suffix(".failed.json"), value)
                    raise ValueError(f"Unresolved collision errors: {prefix}")
                value["triangle_sat_crosscheck"] = sat_check(shapes, volumes[name], hull)
                tree.save(prefix.with_suffix(".npz"))
                save_json(prefix.with_suffix(".json"), value)
            record[name] = value
            trees[name] = all_trees[name][hull] = tree
            print(json.dumps(value), flush=True)
        legacy = [{k: np.array(v) for k, v in volume.items()} for volume in legacy_data[str(hull)]["legacy"]]
        box = pipeline.expanded([v["vertices"] for v in original], pipeline.HALF[hull], exact.CORNERS)
        record["top_contact"] = contacts(root, trees, volumes, legacy, box, hull)
        # Arbitrary-direction first contacts test the complete exterior shell.
        _, origins, directions, length = pipeline.probes(box, 582910, 1, 8192)
        record["shell_external_first_contact_vs_closed_box"] = pipeline.ray_stats(
            trees["shell"].hit(origins, directions, length), convex_hit(origins, directions, length, box))
        report["hulls"][str(hull)] = record
        save_json(root / "results.json", report)
    for name, trees in all_trees.items():
        data, checked = pipeline.splice(args.closed / "background_lit.bsp", trees)
        output = root / f"untitledx_surface_{name}.bsp"
        output.write_bytes(data)
        report["outputs"][name] = {"path": str(output), "sha256": sha256(output), "structure": checked}
    if sha256(args.source) != source_hash:
        raise ValueError("Source changed during experiment")
    report["wall_seconds"] = time.perf_counter() - started
    report["light_bytes"] = len(HullFile(args.closed / "background_lit.bsp").lumps[8])
    report["vis_bytes"] = len(HullFile(args.closed / "background_lit.bsp").lumps[4])
    save_json(root / "results.json", report)
    print(json.dumps(report["outputs"], indent=2), flush=True)


if __name__ == "__main__":
    main()
