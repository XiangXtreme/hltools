"""Repair one witnessed false SOLID occurrence using legacy brush volumes.

This is an isolated diagnostic BSP experiment, not a production cliptype.
Only one path to a SOLID leaf, inside a bounded box, can change. It uses the
supplied brushbsp.py's fragment operations and local BSP builder.
"""

import argparse
import hashlib
import importlib.util
import itertools
from pathlib import Path
import json
import struct
import time

from legacy_clip_diagnose import convex_hit, margin, ray_errors, save_json, sha256, tops, volume_tops
from source_clip_pipeline import HullFile
from source_clip_validate import structure, tree_first_hit
import numpy as np


def vertices(planes):
    triples = np.array(list(itertools.combinations(range(len(planes)), 3)))
    matrices = planes[triples, :3]
    valid = np.abs(np.linalg.det(matrices)) > 1e-10
    values = np.linalg.solve(matrices[valid], planes[triples[valid], 3, None])[..., 0]
    values = values[np.max(values @ planes[:, :3].T - planes[:, 3], axis=1) < 1e-6]
    return np.unique(np.round(values, 9), axis=0)


def compact(tree, nodes, roots):
    ordered, mapped, visiting = [], {}, set()

    def visit(index):
        if index < 0:
            return index
        if index in mapped:
            return mapped[index]
        if index in visiting:
            raise ValueError("Cycle")
        visiting.add(index)
        plane, front, back = nodes[index]
        front, back = visit(front), visit(back)
        if front == back and index >= len(tree.nodes):
            result = front
        else:
            result = len(ordered)
            ordered.append((plane, front, back))
        mapped[index] = result
        visiting.remove(index)
        return result

    heads = [visit(root) for root in roots]
    last = len(ordered) - 1
    remap = lambda index: last - index if index >= 0 else index
    result = [(p, remap(f), remap(b)) for p, f, b in reversed(ordered)]
    if len(result) > 32767:
        raise ValueError("Clipnode budget exceeded")
    return result, [remap(index) for index in heads]


def graph_hash(tree, hull):
    memo = {}

    def visit(index):
        if index < 0:
            return struct.pack("<i", index)
        if index not in memo:
            plane, front, back = tree.nodes[index]
            record = tree.lumps[1][int(plane) * 20:(int(plane) + 1) * 20]
            memo[index] = hashlib.sha256(record + visit(int(front)) + visit(int(back))).digest()
        return memo[index]

    return visit(int(tree.models[0]["head"][hull])).hex()


def encode_bsp(tree, planes, nodes, heads):
    lumps = list(tree.lumps)
    lumps[1] += b"".join(planes)
    lumps[9] = b"".join(struct.pack("<ihh", *node) for node in nodes)
    models = bytearray(lumps[14])
    struct.pack_into("<3i", models, 40, *heads)
    lumps[14] = bytes(models)
    data = bytearray(124)
    struct.pack_into("<i", data, 0, 30)
    for index, lump in enumerate(lumps):
        data.extend(b"\0" * (-len(data) % 4))
        struct.pack_into("<ii", data, 4 + 8 * index, len(data), len(lump))
        data.extend(lump)
    return bytes(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path)
    parser.add_argument("--brushbsp", required=True, type=Path)
    parser.add_argument("--radius", type=float, default=32)
    args = parser.parse_args()
    root = args.experiment.resolve()
    output = root / "legacy_local_probe.bsp"
    if output.exists() or args.radius <= 0:
        raise ValueError("Use a fresh experiment or a positive radius")
    report = json.loads((root / "summary.json").read_text())
    witness = np.array(report["hulls"]["1"]["bsp_error_witness"]["point"])
    volumes = [{key: np.array(value) for key, value in volume.items()}
               for volume in json.loads((root / "volumes.json").read_text())["1"]["legacy"]]
    source = root / "legacy.bsp"
    source_hash, script_hash = sha256(source), sha256(args.brushbsp)
    tree = HullFile(source)
    if tree.contents(witness[None], 1)[0] != -2 or margin(witness[None], volumes)[0] <= .05:
        raise ValueError("Witness is not a robust extra-solid point")
    spec = importlib.util.spec_from_file_location("brushbsp_local_probe", args.brushbsp.resolve())
    bb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bb)
    start = time.perf_counter()
    table, nodes = bb.Planes(), tree.nodes.tolist()
    lo = np.maximum(witness - args.radius, [report["xy"][0], report["xy"][0], report["z"][0]])
    hi = np.minimum(witness + args.radius, [report["xy"][1], report["xy"][1], report["z"][1]])
    normals = np.concatenate((np.eye(3), -np.eye(3)))
    distances = np.r_[hi, -lo]
    region = bb.make_frag(table, np.array(list(itertools.product(*zip(lo, hi)))), normals, distances)
    path, current = [], int(tree.models[0]["head"][1])
    while current >= 0:
        plane, front, back = nodes[current]
        p = tree.planes[plane]
        pid = table.add(p[:3], p[3])
        fragments = bb.split_frag(region, pid, table.n[pid], table.d[pid])
        side = int(witness @ p[:3] < p[3])
        region = fragments[side]
        if region is None or len(region.verts) < 4:
            raise ValueError("Lost witness region")
        path.append((current, side))
        current = (front, back)[side]
    if current != -2:
        raise ValueError("Witness path did not end SOLID")
    fragments = []
    candidates = 0
    for volume in volumes:
        if np.any(volume["bounds"][0] > hi) or np.any(volume["bounds"][1] < lo):
            continue
        candidates += 1
        p = volume["planes"]
        fragment = bb.make_frag(table, vertices(p), p[:, :3], p[:, 3])
        for pid, orientation in region.sides.items():
            parts = bb.split_frag(fragment, pid, table.n[pid], table.d[pid])
            fragment = parts[1 if orientation == 1 else 0]
            if fragment is None:
                break
        if fragment is not None and len(fragment.verts) >= 4:
            fragments.append(bb.Frag(fragment.verts, fragment.inc, fragment.sides,
                                     fragment.used | set(region.sides)))
    local = []
    local_root = bb.build(fragments, table, local, np.random.default_rng(0))
    extra_planes, plane_ids = [], {}

    def file_plane(normal, distance):
        record = struct.pack("<4fi", *normal, distance, 3 + int(np.argmax(np.abs(normal))))
        if record not in plane_ids:
            plane_ids[record] = len(tree.planes) + len(extra_planes)
            extra_planes.append(record)
        return plane_ids[record]

    base = len(nodes)
    nodes += [(file_plane(table.n[p], table.d[p]),
               f + base if f >= 0 else f, b + base if b >= 0 else b) for p, f, b in local]
    replacement = local_root + base if local_root >= 0 else local_root
    # The selected original leaf remains SOLID outside this diagnostic box.
    for axis in range(3):
        normal = np.eye(3)[axis]
        nodes.append((file_plane(normal, hi[axis]), -2, replacement))
        replacement = len(nodes) - 1
        nodes.append((file_plane(normal, lo[axis]), replacement, -2))
        replacement = len(nodes) - 1
    for index, side in reversed(path):
        plane, front, back = nodes[index]
        nodes.append((plane, replacement if side == 0 else front, replacement if side else back))
        replacement = len(nodes) - 1
    heads = [int(head) for head in tree.models[0]["head"][1:]]
    heads[0] = replacement
    nodes, heads = compact(tree, nodes, heads)
    data = encode_bsp(tree, extra_planes, nodes, heads)
    elapsed = time.perf_counter() - start
    fixed = HullFile(data=data)
    checked = structure(fixed)
    if fixed.contents(witness[None], 1)[0] != -1:
        raise ValueError("Local repair did not remove witnessed false solid")
    if any(graph_hash(tree, hull) != graph_hash(fixed, hull) for hull in (2, 3)):
        raise ValueError("Unselected hull changed")
    if any(tree.lumps[index] != fixed.lumps[index] for index in range(15) if index not in (1, 9, 14)):
        raise ValueError("Noncollision lump changed")
    rng = np.random.default_rng(20261023)
    local_points = rng.uniform(lo, hi, (100000, 3))
    probes = np.load(root / "probes.npz")
    points = np.concatenate((probes["points"], local_points))
    before, after = tree.contents(points, 1), fixed.contents(points, 1)
    target = margin(points, volumes)
    changed = before != after
    if np.any(changed & (before != -2)) or np.any(changed & (after != -1)):
        raise ValueError("A change was not SOLID to EMPTY")
    outside = np.any(points < lo - 1e-4, axis=1) | np.any(points > hi + 1e-4, axis=1)
    if np.any(changed & outside) or np.any(changed & (target < -.05)):
        raise ValueError("Repair removed legacy target coverage or changed outside its box")
    ray_results = {}
    expected = convex_hit(probes["origins"], probes["directions"],
                          2 * (report["z"][1] - report["z"][0]), volumes)
    for name, value in (("before", tree), ("after", fixed)):
        ray_results[name] = ray_errors(tree_first_hit(value, probes["origins"], probes["directions"],
            2 * (report["z"][1] - report["z"][0]), 1), expected)
    result = {"output": str(output), "input_sha256": source_hash,
              "brushbsp_sha256": script_hash, "witness": witness.tolist(), "box": [lo.tolist(), hi.tolist()],
              "repair_seconds": elapsed, "path_nodes": len(path), "candidate_brushes": candidates,
              "clipped_fragments": len(fragments), "local_subtree_nodes": len(local),
              "before": structure(tree), "after": checked, "probes": len(points),
              "changed_solid_to_empty": int(changed.sum()),
              "removed_inside_legacy_margin_0_05": int(np.sum(changed & (target < -.05))),
              "extra_solid_before": int(np.sum((before == -2) & (target > .05))),
              "extra_solid_after": int(np.sum((after == -2) & (target > .05))),
              "unchanged_hulls_2_3_graph_hash": True, "noncollision_lumps_identical": True,
              "rays": ray_results}
    result["probe_sets"] = {}
    for name, selection in (("global", slice(0, len(probes["points"]))),
                            ("local", slice(len(probes["points"]), None))):
        result["probe_sets"][name] = {
            "count": len(points[selection]),
            "extra_solid_before": int(np.sum((before[selection] == -2) & (target[selection] > .05))),
            "extra_solid_after": int(np.sum((after[selection] == -2) & (target[selection] > .05))),
            "missing_solid_after": int(np.sum((after[selection] != -2) & (target[selection] < -.05)))}
    expansion_point = np.array(report["hulls"]["1"]["expansion_witness"]["point"])
    result["expansion_witness_before_after"] = [int(value.contents(expansion_point[None], 1)[0])
                                                for value in (tree, fixed)]
    if sha256(source) != source_hash or sha256(args.brushbsp) != script_hash:
        raise ValueError("Input changed")
    output.write_bytes(data)
    save_json(root / "local_probe.json", result)
    plot(root, tree, fixed, volumes, witness, report)
    print(json.dumps(result, indent=2), flush=True)


def plot(root, before, after, volumes, witness, report):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x = np.linspace(max(witness[0] - 48, report["xy"][0]), min(witness[0] + 48, report["xy"][1]), 1001)
    xy = np.column_stack((x, np.full(len(x), witness[1])))
    fig, ax = plt.subplots(figsize=(9, 4), dpi=160)
    ax.plot(x, tops(before, 1, xy, *report["z"]), color="#c43c39", label="Original legacy BSP")
    ax.plot(x, tops(after, 1, xy, *report["z"]), color="#16875b", linestyle="--", label="One-leaf local repair")
    ax.plot(x, volume_tops(volumes, xy, *report["z"]), color="#286bb2", linestyle=":", label="Legacy brush union")
    ax.scatter(*witness[[0, 2]], color="#151515", marker="x", label="False-solid witness")
    ax.set(title=f"Hull 1, Y = {witness[1]:.4f}", xlabel="X", ylabel="Player origin Z")
    ax.legend(fontsize=8)
    ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(root / "local_probe.png")
    plt.close(fig)


if __name__ == "__main__":
    main()
