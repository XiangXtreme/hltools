"""Repair and compare an isolated Source collision build against triangle geometry."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import time
import zipfile
from concurrent.futures import ProcessPoolExecutor, as_completed

import source_clip_pipeline as pipeline
import numpy as np

FIX = None


def init_geometry(config):
    global FIX
    pipeline.load_scripts(config)
    import fix_ambiguous
    FIX = fix_ambiguous
    FIX.init()


def repair_worker(filename, source_folder="clips_raw", destination_folder="clips_out", records_folder="repair_records"):
    source = Path(source_folder) / filename
    destination = Path(destination_folder) / filename
    started = time.perf_counter()
    note = FIX.fix_file(str(source), str(destination))
    record = {"file": filename, "seconds": time.perf_counter() - started, "note": note,
              "raw_sha256": pipeline.sha256(source), "repaired_sha256": pipeline.sha256(destination),
              "raw_nodes": len(pipeline.HullFile(source).nodes),
              "repaired_nodes": len(pipeline.HullFile(destination).nodes),
              "fallback": "left as it was" in note}
    pipeline.save_json(Path(records_folder) / (filename + ".json"), record)
    return record


def repair(args, config):
    pipeline.load_scripts(config)
    files = sorted(Path("clips_raw").glob("*.bsp"))
    if not files:
        raise RuntimeError("Build the raw collision files first")
    Path("repair_records").mkdir(exist_ok=True)
    chunks = {record["file"]: record["name"] for record in json.loads(Path("build_results.json").read_text())["chunks"]} if Path("build_results.json").exists() else {}
    done, todo = [], []
    for path in files:
        record_path = Path("repair_records") / (path.name + ".json")
        chunk = chunks.get(path.name)
        cached_record = Path("chunk_repair_records") / (str(chunk) + ".bsp.json")
        cached_file = Path("chunk_repairs") / (str(chunk) + ".bsp")
        if not record_path.exists() and cached_record.exists() and cached_file.exists():
            record = json.loads(cached_record.read_text())
            if record["raw_sha256"] == pipeline.sha256(path) and record["repaired_sha256"] == pipeline.sha256(cached_file):
                import shutil
                shutil.copy2(cached_file, Path("clips_out") / path.name)
                record.update(file=path.name, source_chunk=chunk)
                pipeline.save_json(record_path, record)
        if record_path.exists():
            record = json.loads(record_path.read_text())
            if record["raw_sha256"] == pipeline.sha256(path) and record["repaired_sha256"] == pipeline.sha256(Path("clips_out") / path.name):
                done.append(record)
                continue
        todo.append(path.name)
    started = time.perf_counter()
    with ProcessPoolExecutor(args.workers, initializer=init_geometry, initargs=(config,)) as pool:
        futures = [pool.submit(repair_worker, filename) for filename in todo]
        for future in as_completed(futures):
            record = future.result()
            done.append(record)
            print(f"{len(done)}/{len(files)}: {record['note']}", flush=True)
    pipeline.save_json("repair_results.json", {"wall_seconds_this_run": time.perf_counter() - started,
                                              "files": sorted(done, key=lambda record: record["file"])})


def repair_chunks(args, config):
    pipeline.load_scripts(config)
    chunks = json.loads(Path("build_progress.json").read_text())
    Path("chunk_repairs").mkdir(exist_ok=True)
    Path("chunk_repair_records").mkdir(exist_ok=True)
    with ProcessPoolExecutor(args.workers, initializer=init_geometry, initargs=(config,)) as pool:
        jobs = []
        for chunk in chunks:
            name = chunk["name"]
            if not (Path("chunk_repair_records") / (name + ".bsp.json")).exists():
                jobs.append(pool.submit(repair_worker, name + ".bsp", "exact_chunks/" + name,
                                        "chunk_repairs", "chunk_repair_records"))
        for index, future in enumerate(as_completed(jobs), 1):
            print(f"{index}/{len(jobs)}: {future.result()['note']}", flush=True)


def structure(tree):
    if len(tree.nodes) > 32767:
        raise ValueError("Clipnode budget exceeded")
    if not np.isfinite(tree.planes).all():
        raise ValueError("Nonfinite plane")
    result = {"stored_nodes": len(tree.nodes), "hulls": {}}
    for hull in (1, 2, 3):
        root = int(tree.models[0]["head"][hull])
        seen, depths, stack = set(), {}, [(root, False)]
        visiting = set()
        while stack:
            node, leaving = stack.pop()
            if node < 0:
                if node not in (-1, -2):
                    raise ValueError("Unexpected leaf contents")
                continue
            if leaving:
                children = tree.nodes[node, 1:]
                depths[node] = 1 + max((depths[int(child)] if child >= 0 else 0) for child in children)
                visiting.remove(node)
                seen.add(node)
                continue
            if node in visiting:
                raise ValueError("Clipnode cycle")
            if node in seen:
                continue
            if not root <= node < len(tree.nodes):
                raise ValueError("GoldSrc hull lower-bound violation")
            plane, front, back = map(int, tree.nodes[node])
            if not 0 <= plane < len(tree.planes):
                raise ValueError("Invalid plane index")
            visiting.add(node)
            stack.extend([(node, True), (back, False), (front, False)])
        result["hulls"][str(hull)] = {"root": root, "reachable": len(seen), "depth": depths.get(root, 0)}
    return result


def solid_union(trees, points, hull):
    hit = np.zeros(len(points), bool)
    half = FIX.X.HALF[hull]
    for tree in trees:
        model = tree.models[0]
        active = ~hit & np.all(points >= model["mins"] - half - 1, axis=1)
        active &= np.all(points <= model["maxs"] + half + 1, axis=1)
        if active.any():
            indices = np.flatnonzero(active)
            hit[indices] = tree.contents(points[indices], hull=hull) == -2
    return hit


def tree_first_hit(tree, origins, directions, length, hull):
    hit = np.full(len(origins), np.inf)
    stack = [(int(tree.models[0]["head"][hull]), np.arange(len(origins)),
              np.zeros(len(origins)), np.full(len(origins), length, dtype=float))]
    while stack:
        node, indices, lo, hi = stack.pop()
        if not len(indices):
            continue
        if node < 0:
            if node == -2:
                hit[indices] = np.minimum(hit[indices], lo)
            continue
        plane, front, back = tree.nodes[node]
        normal, distance = tree.planes[plane, :3], tree.planes[plane, 3]
        offset = origins[indices] @ normal - distance
        velocity = directions[indices] @ normal
        moving = np.abs(velocity) > 1e-12
        cut = np.divide(-offset, velocity, out=np.zeros(len(indices)), where=moving)
        for child, sign in ((front, 1), (back, -1)):
            lower, upper = lo.copy(), hi.copy()
            enters, exits = velocity * sign > 1e-12, velocity * sign < -1e-12
            lower[enters] = np.maximum(lower[enters], cut[enters])
            upper[exits] = np.minimum(upper[exits], cut[exits])
            keep = (upper > lower + 1e-9) & (moving | (offset * sign >= 0))
            stack.append((int(child), indices[keep], lower[keep], upper[keep]))
    return hit


def union_first_hit(trees, origins, directions, length, hull):
    hit = np.full(len(origins), np.inf)
    half = FIX.X.HALF[hull]
    end = origins + directions * length
    for tree in trees:
        model = tree.models[0]
        active = np.all(np.maximum(origins, end) >= model["mins"] - half - 1, axis=1)
        active &= np.all(np.minimum(origins, end) <= model["maxs"] + half + 1, axis=1)
        indices = np.flatnonzero(active)
        if len(indices):
            hit[indices] = np.minimum(hit[indices], tree_first_hit(tree, origins[indices], directions[indices], length, hull))
    return hit


def triangle_first_hit(origins, directions, length, hull):
    """Intersect a moving box with triangles using intervals on all 13 SAT axes."""
    hit = np.full(len(origins), np.inf)
    half = FIX.X.HALF[hull]
    for index, (origin, direction) in enumerate(zip(origins, directions)):
        end = origin + direction * length
        candidates = FIX.near(np.minimum(origin, end), np.maximum(origin, end), hull)
        if not len(candidates):
            continue
        triangles = FIX._tris[candidates]
        edges = np.roll(triangles, -1, axis=1) - triangles
        normals = np.cross(edges[:, 0], edges[:, 1])
        axes = [np.broadcast_to(axis, normals.shape) for axis in np.eye(3)] + [normals]
        axes += [np.cross(edge, axis) for edge in edges.transpose(1, 0, 2) for axis in np.eye(3)]
        lower, upper = np.zeros(len(candidates)), np.full(len(candidates), length)
        possible = np.ones(len(candidates), bool)
        relative = triangles - origin
        for axis in axes:
            projections = np.einsum("nij,nj->ni", relative, axis)
            radius = np.abs(axis) @ half
            minimum = projections.min(1) - radius
            maximum = projections.max(1) + radius
            velocity = axis @ direction
            moving = np.abs(velocity) > 1e-12
            possible &= moving | ((minimum <= 1e-10) & (maximum >= -1e-10))
            enter = np.divide(minimum, velocity, out=np.full(len(axis), -np.inf), where=moving)
            leave = np.divide(maximum, velocity, out=np.full(len(axis), np.inf), where=moving)
            negative = moving & (velocity < 0)
            enter[negative], leave[negative] = leave[negative].copy(), enter[negative].copy()
            lower = np.maximum(lower, enter)
            upper = np.minimum(upper, leave)
        possible &= upper >= lower
        if possible.any():
            hit[index] = lower[possible].min()
    return hit


def validate(args, config):
    started = time.perf_counter()
    init_geometry(config)
    root = Path(config["output"])
    variants = {name: [pipeline.HullFile(path) for path in sorted((root / folder).glob("*.bsp"))]
                for name, folder in (("raw", "clips_raw"), ("repaired", "clips_out")) if name in args.variants}
    if "archive" in args.variants and args.archive:
        with zipfile.ZipFile(args.archive) as archive:
            variants["archive"] = [pipeline.HullFile(data=archive.read(name)) for name in sorted(archive.namelist())
                                   if re.search(r"/bclipv2_\d+\.bsp$", name)]
    if any(not trees for trees in variants.values()):
        raise ValueError("A collision variant contains no files")
    result = {"seed": args.seed, "samples_per_hull": args.samples, "rays_per_hull": args.rays,
              "boundary_box_tolerance": 0.01, "reference": "Script-selected, 1/16-snapped triangles",
              "variants": {}, "hulls": {}}
    for name, trees in variants.items():
        records = [structure(tree) for tree in trees]
        result["variants"][name] = {"files": len(trees), "total_nodes": sum(len(tree.nodes) for tree in trees),
                                     "structure": records}
        print(name, len(trees), result["variants"][name]["total_nodes"], "structure OK", flush=True)
    tris = FIX._tris
    rng = np.random.default_rng(args.seed)
    sample_indices = rng.integers(0, len(tris), args.samples)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    weights = rng.dirichlet(np.ones(3), size=args.samples)
    weights[::2] = np.roll(np.array([0.98, 0.01, 0.01]), 0)
    surface = np.einsum("ni,nij->nj", weights, tris[sample_indices])
    n = normals[sample_indices]
    ray_indices = rng.integers(0, len(tris), args.rays)
    ray_surface = tris[ray_indices].mean(1)
    ray_normals = normals[ray_indices]
    probes = {}
    for hull in (1, 3):
        half = FIX.X.HALF[hull].copy()
        offsets = rng.uniform(-8, 8, args.samples)
        points = surface + n * (np.abs(n) @ half + offsets)[:, None]
        expected = FIX.touches(points, hull)
        FIX.X.HALF[hull] = half + 0.01
        expanded = FIX.touches(points, hull)
        FIX.X.HALF[hull] = half - 0.01
        shrunk = FIX.touches(points, hull)
        FIX.X.HALF[hull] = half
        origins = ray_surface + ray_normals * 96
        directions = -ray_normals
        expected_rays = triangle_first_hit(origins, directions, 192.0, hull)
        record = {}
        for name, trees in variants.items():
            observed = solid_union(trees, points, hull)
            rays = union_first_hit(trees, origins, directions, 192.0, hull)
            finite = np.isfinite(rays) & np.isfinite(expected_rays)
            delta = rays[finite] - expected_rays[finite]
            record[name] = {"point_disagreements": int(np.count_nonzero(observed != expected)),
                            "extra_solid_beyond_tolerance": int(np.count_nonzero(observed & ~expanded)),
                            "missing_solid_beyond_tolerance": int(np.count_nonzero(~observed & shrunk)),
                            "ray_hit_mismatches": int(np.count_nonzero(np.isfinite(rays) != np.isfinite(expected_rays))),
                            "ray_errors_above_0_01": int(np.count_nonzero(np.abs(delta) > 0.01)),
                            "ray_errors_above_0_1": int(np.count_nonzero(np.abs(delta) > 0.1)),
                            "ray_errors_above_1": int(np.count_nonzero(np.abs(delta) > 1)),
                            "max_ray_error": float(np.abs(delta).max(initial=0)),
                            "early_ray_errors_above_0_1": int(np.count_nonzero(delta < -0.1)),
                            "late_ray_errors_above_0_1": int(np.count_nonzero(delta > 0.1))}
            probes[f"h{hull}_{name}_contents"] = observed
            probes[f"h{hull}_{name}_rays"] = rays
            print("hull", hull, name, json.dumps(record[name]), flush=True)
        probes[f"h{hull}_points"] = points
        probes[f"h{hull}_expected_contents"] = expected
        probes[f"h{hull}_ray_origins"] = origins
        probes[f"h{hull}_ray_directions"] = directions
        probes[f"h{hull}_expected_rays"] = expected_rays
        result["hulls"][str(hull)] = record
        result["wall_seconds"] = time.perf_counter() - started
        pipeline.save_json(args.report_stem + ".json", result)
        np.savez_compressed(args.report_stem + "_probes.npz", **probes)
    if pipeline.sha256(config["source"]) != config["source_sha256"]:
        raise ValueError("Original collision source changed")
    if config.get("input_kind") == "brush_exterior_triangles":
        if pipeline.sha256(root / "terrain_tris.npy") != config["triangle_sha256"]:
            raise ValueError("Prepared triangles changed")
        print("Original MAP and prepared triangle hashes unchanged", flush=True)
    else:
        if pipeline.sha256(config["vmf"]) != config["vmf_sha256"]:
            raise ValueError("Original VMF changed")
        print("Original BSP and VMF hashes unchanged", flush=True)


def compare_archive(args, config):
    root = Path(config["output"])
    records = []
    with zipfile.ZipFile(args.archive) as archive:
        entries = {Path(name).name: name for name in archive.namelist() if re.search(r"/bclipv2_\d+\.bsp$", name)}
        for path in sorted((root / "clips_out").glob("bclip*.bsp")):
            index = int(path.stem.removeprefix("bclip"))
            reference_name = f"bclipv2_{index:03d}.bsp"
            if reference_name not in entries:
                records.append({"file": path.name, "reference_missing": True})
                continue
            rebuilt = pipeline.HullFile(path)
            raw = pipeline.HullFile(root / "clips_raw" / path.name)
            reference = pipeline.HullFile(data=archive.read(entries[reference_name]))
            shifted = rebuilt.nodes.copy()
            shifted[:, 1:] = np.where(shifted[:, 1:] >= 0, shifted[:, 1:] + 1, shifted[:, 1:])
            normalized = reference.nodes[1:].copy()
            normalized[:, 1:] = np.where(normalized[:, 1:] >= 0, normalized[:, 1:] - 1, normalized[:, 1:])
            record = {"file": path.name, "reference": entries[reference_name],
                      "raw_nodes": len(raw.nodes), "rebuilt_nodes": len(rebuilt.nodes),
                      "reference_nodes": len(reference.nodes),
                      "reference_reserved_empty_node": reference.nodes[0].tolist() == [0, -1, -1],
                      "raw_plane_bytes_match_reference_prefix": reference.lumps[1].startswith(raw.lumps[1]),
                      "all_repaired_plane_bytes_identical": rebuilt.lumps[1] == reference.lumps[1],
                      "all_repaired_nodes_identical_after_offset": np.array_equal(shifted, reference.nodes[1:]),
                      "hull_roots_match_after_offset": np.array_equal(rebuilt.models[0]["head"][1:] + 1, reference.models[0]["head"][1:]),
                      "bounds_identical": all(np.array_equal(rebuilt.models[0][field], reference.models[0][field]) for field in ("mins", "maxs")),
                      "rebuilt_clipnodes_sha256": hashlib.sha256(rebuilt.lumps[9]).hexdigest(),
                      "normalized_reference_clipnodes_sha256": hashlib.sha256(normalized.astype("<i8").tobytes()).hexdigest(),
                      "rebuilt_nodes_int64_sha256": hashlib.sha256(rebuilt.nodes.astype("<i8").tobytes()).hexdigest(),
                      "plane_lump_sha256": hashlib.sha256(rebuilt.lumps[1]).hexdigest()}
            records.append(record)
            print(path.name, "planes", record["all_repaired_plane_bytes_identical"],
                  "nodes", record["all_repaired_nodes_identical_after_offset"], flush=True)
    pipeline.save_json(root / "archive_comparison.json", {"archive": str(Path(args.archive).resolve()),
                                                         "archive_sha256": pipeline.sha256(args.archive),
                                                         "archive_files": len(entries), "files": records})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("repair", "repair-chunks", "validate", "compare"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--archive")
    parser.add_argument("--samples", type=int, default=50000)
    parser.add_argument("--rays", type=int, default=4096)
    parser.add_argument("--seed", type=int, default=20261023)
    parser.add_argument("--variants", nargs="+", choices=("raw", "repaired", "archive"), default=("raw", "repaired", "archive"))
    parser.add_argument("--report-stem", default="validation")
    args = parser.parse_args()
    config = json.loads((Path(args.output).resolve() / "config.json").read_text())
    if args.command == "compare":
        if not args.archive:
            parser.error("compare requires --archive")
        compare_archive(args, config)
    elif args.command == "repair-chunks":
        repair_chunks(args, config)
    elif args.command == "repair":
        repair(args, config)
    else:
        validate(args, config)


if __name__ == "__main__":
    main()
