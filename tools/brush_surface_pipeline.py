"""Compare closed brushes and their top triangles through the reference hull-0 pipeline.

This is an isolated experiment, not a production cliptype. It snapshots inputs,
uses the real CSG brush geometry, and validates sampled collision against the
unsnapped Minkowski sums. Local leaf repair follows brushbsp.py's fragment rule.
"""

import argparse
import importlib.util
import itertools
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import time
from types import SimpleNamespace

os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

import numpy as np
from scipy.spatial import ConvexHull

from legacy_clip_diagnose import convex_hit, margin, sha256, save_json
from source_clip_pipeline import HullFile, SCRIPTS
from source_clip_validate import structure, tree_first_hit
from remove_map_group_volume import parse_map

HALF = {1: np.array([16., 16., 36.]), 2: np.array([32., 32., 32.]),
        3: np.array([16., 16., 18.])}
TOL = .01


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def command(args, log):
    started = time.perf_counter()
    with log.open("w", encoding="utf-8") as stream:
        result = subprocess.run(list(map(str, args)), stdout=stream, stderr=subprocess.STDOUT)
    elapsed = time.perf_counter() - started
    save_json(log.with_suffix(".command.json"),
              {"command": list(map(str, args)), "seconds": elapsed, "returncode": result.returncode})
    return result.returncode, elapsed


def require_command(args, log):
    code, elapsed = command(args, log)
    if code:
        raise RuntimeError(f"Command failed ({code}): {log}")
    return elapsed


def compact(nodes, root):
    ordered, memo, cache, visiting = [], {}, {}, set()

    def visit(index):
        if index < 0:
            if index not in (-1, -2):
                raise ValueError(f"Unexpected contents: {index}")
            return index
        if index in memo:
            return memo[index]
        if index in visiting or not 0 <= index < len(nodes):
            raise ValueError("Invalid collision graph")
        visiting.add(index)
        p, f, b = map(int, nodes[index])
        f, b = visit(f), visit(b)
        key = (p, f, b)
        if f == b:
            value = f
        else:
            if key not in cache:
                cache[key] = len(ordered)
                ordered.append(key)
            value = cache[key]
        visiting.remove(index)
        memo[index] = value
        return value

    root = visit(int(root))
    remap = lambda index: len(ordered) - 1 - index if index >= 0 else index
    return [(p, remap(f), remap(b)) for p, f, b in reversed(ordered)], remap(root)


class Tree:
    def __init__(self, planes, nodes, root):
        self.planes = np.array(planes, dtype=float)
        self.nodes = [tuple(map(int, node)) for node in nodes]
        self.root = int(root)

    @classmethod
    def draw(cls, path):
        bsp = HullFile(path)
        leaves = [struct.unpack_from("<i", bsp.lumps[10], i)[0]
                  for i in range(0, len(bsp.lumps[10]), 28)]
        nodes = []
        for i in range(0, len(bsp.lumps[5]), 24):
            p, f, b = struct.unpack_from("<ihh", bsp.lumps[5], i)
            nodes.append((p, leaves[-f - 1] if f < 0 else f, leaves[-b - 1] if b < 0 else b))
        root = int(bsp.models[0]["head"][0])
        return cls(bsp.planes, nodes, leaves[-root - 1] if root < 0 else root)

    def view(self):
        return SimpleNamespace(nodes=np.array(self.nodes, dtype=np.int64).reshape(-1, 3),
                               planes=self.planes, models=[{"head": [self.root] * 4}])

    def contents(self, points):
        return HullFile.contents(self.view(), points)

    def hit(self, origins, directions, length):
        return tree_first_hit(self.view(), origins, directions, length, 1)

    def compact(self):
        self.nodes, self.root = compact(self.nodes, self.root)

    def bound(self, low, high):
        # Leaf repair uses a finite region; its assumed box must also exist in
        # the emitted tree, otherwise a rebuilt fragment can extend to infinity.
        for axis in range(3):
            normal = np.eye(3)[axis]
            p = len(self.planes)
            self.planes = np.r_[self.planes, [[*normal, high[axis], axis], [*normal, low[axis], axis]]]
            self.nodes.append((p, -1, self.root))
            self.nodes.append((p + 1, len(self.nodes) - 1, -1))
            self.root = len(self.nodes) - 1

    def save(self, path):
        np.savez_compressed(path, planes=self.planes, nodes=np.array(self.nodes).reshape(-1, 3), root=self.root)

    @classmethod
    def load(cls, path):
        data = np.load(path)
        return cls(data["planes"], data["nodes"], data["root"])


def expanded(shapes, half, corners):
    volumes = []
    for shape in shapes:
        points = (shape[:, None] + corners[None] * half).reshape(-1, 3)
        hull = ConvexHull(points)
        equations = np.c_[hull.equations[:, :3], -hull.equations[:, 3]]
        _, indices = np.unique(np.round(equations, 9), axis=0, return_index=True)
        vertices = points[hull.vertices]
        volumes.append({"planes": equations[np.sort(indices)], "vertices": vertices,
                        "bounds": np.array([vertices.min(0), vertices.max(0)])})
    return volumes


def extract(path, entity, require_single_top=True):
    brushes = [brush for brush in json.loads(path.read_text())["brushes"] if brush["entity"] == entity]
    if not brushes:
        raise ValueError("Selected entity has no built brushes")
    closed, triangles = [], []
    for brush in brushes:
        if brush["contents"] != -2:
            raise ValueError("This experiment requires solid brushes")
        vertices = np.concatenate([face["vertices"] for face in brush["faces"]])
        # Remove duplicate winding points without changing their coordinates.
        _, indices = np.unique(np.round(vertices, 9), axis=0, return_index=True)
        closed.append(vertices[np.sort(indices)])
        top = [face for face in brush["faces"] if face["normal"][2] > 1e-6]
        if require_single_top and len(top) != 1:
            raise ValueError("Selected brushes must each have one top face")
        for face in top:
            polygon = np.array(face["vertices"])
            for i in range(1, len(polygon) - 1):
                tri = polygon[[0, i, i + 1]]
                if np.linalg.norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])) < 1e-8:
                    raise ValueError("Degenerate top triangle")
                triangles.append(tri)
    return closed, np.array(triangles)


def compile_expanded(path, shapes, hull, exact, compiler):
    started = time.perf_counter()
    brushes = exact.canonical_brushes(shapes, hull)
    changes, compile_seconds = [], 0.
    for attempt in range(65):
        body = ["{\n" + "\n".join(exact.plane_face(n, d) for n, d in brush) + "\n}"
                for brush in brushes]
        path.write_text('{\n"classname" "worldspawn"\n"mapversion" "220"\n'
                        + f'"wad" "{exact.WAD}"\n' + "\n".join(body) + "\n}\n", encoding="ascii")
        log = path.with_suffix(f".csg.{attempt:02}.txt")
        code, elapsed = command([compiler, "csg", path, "-noclip", "-threads", "2"], log)
        compile_seconds += elapsed
        if not code:
            break
        error = re.search(r"entity 0, brush (\d+), side (\d+): has a coplanar plane", log.read_text())
        if error is None or attempt == 64:
            raise RuntimeError(f"Expanded CSG failed: {log}")
        owner, side = map(int, error.groups())
        changes.append([owner, side])
        del brushes[owner][side]
    compile_seconds += require_command(
        [compiler, "bsp", path, "-noclip", "-nofill", "-noinsidefill", "-nonulltex", "-threads", "2"],
        path.with_suffix(".bsp.txt"))
    return Tree.draw(path.with_suffix(".bsp")), {
        "compile_seconds": compile_seconds, "generation_and_compile_seconds": time.perf_counter() - started,
        "dropped_coplanar_sides": changes}


def ray_stats(actual, expected):
    both = np.isfinite(actual) & np.isfinite(expected)
    errors = np.abs(actual[both] - expected[both])
    return {"hit_miss_differences": int(np.sum(np.isfinite(actual) != np.isfinite(expected))),
            "errors_over_0_01": int(np.sum(errors > TOL)),
            "max_distance_error": float(errors.max()) if len(errors) else 0.}


def probes(volumes, seed, count, rays):
    rng = np.random.default_rng(seed)
    low = np.min([v["bounds"][0] for v in volumes], axis=0) - 2
    high = np.max([v["bounds"][1] for v in volumes], axis=0) + 2
    points = rng.uniform(low, high, (count, 3))
    directions = rng.normal(size=(rays, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    length = float(np.linalg.norm(high - low) * 3)
    centers = rng.uniform(low, high, (rays, 3))
    origins = centers - directions * length / 2
    return points, origins, directions, length


def leaf_edges(tree, points):
    nodes = np.array(tree.nodes, dtype=np.int64).reshape(-1, 3)
    current, parents, slots = np.full(len(points), tree.root), np.full(len(points), -1), np.zeros(len(points), int)
    for _ in range(len(nodes) + 1):
        active = np.flatnonzero(current >= 0)
        if not len(active):
            return current, parents, slots
        parents[active] = current[active]
        rows = nodes[current[active]]
        p = tree.planes[rows[:, 0]]
        slots[active] = 1 + (np.sum(points[active] * p[:, :3], axis=1) < p[:, 3]).astype(int)
        current[active] = rows[np.arange(len(rows)), slots[active]]
    raise ValueError("Clipnode cycle")


def region_plane_union(tree, region, volumes):
    """Direct half-space tests inside a leaf, avoiding fragment vertex incidence."""
    root = -1
    low, high = region.verts.min(0), region.verts.max(0)
    for volume in volumes:
        if np.any(volume["bounds"][0] > high + 1e-5) or np.any(volume["bounds"][1] < low - 1e-5):
            continue
        planes = volume["planes"]
        distances = region.verts @ planes[:, :3].T - planes[:, 3]
        if np.any(distances.min(0) > 1e-5):
            continue
        active = planes[distances.max(0) >= -1e-5]
        inside = -2
        extra = []
        for p in reversed(active):
            normal, distance = p[:3], p[3]
            pid = len(tree.planes) + len(extra)
            extra.append([*normal, distance, 3 + int(np.argmax(np.abs(normal)))])
            tree.nodes.append((pid, root, inside))
            inside = len(tree.nodes) - 1
        if extra:
            disk = np.array(extra)
            disk[:, :4] = disk[:, :4].astype(np.float32)
            tree.planes = np.r_[tree.planes, disk]
        root = inside
        if root == -2:
            break
    return root


def repair(tree, volumes, bb, tolerance=TOL, extra_rays=None, max_rounds=5, local_builder="fragments"):
    started = time.perf_counter()
    points, origins, directions, length = probes(volumes, 20261023, 100000, 4096)
    if extra_rays is not None:
        additional_origins, additional_directions, additional_length = extra_rays
        origins = np.r_[origins, additional_origins]
        directions = np.r_[directions, additional_directions]
        length = max(length, additional_length)
    expected = convex_hit(origins, directions, length, volumes)
    low = np.min([v["bounds"][0] for v in volumes], axis=0) - 2
    high = np.max([v["bounds"][1] for v in volumes], axis=0) + 2
    rounds = []
    for iteration in range(max_rounds):
        table = bb.Planes()
        translation = [table.add(p[:3], p[3]) for p in tree.planes]
        region = bb.make_frag(table, np.array(list(itertools.product(*zip(low, high)))),
                              np.r_[np.eye(3), -np.eye(3)], np.r_[high, -low])
        pending, leaves, samples, owners = [(tree.root, region, -1, 0)], [], [], []
        while pending:
            node, region, parent, slot = pending.pop()
            if node >= 0:
                p, f, b = tree.nodes[node]
                pid = translation[p]
                front, back = bb.split_frag(region, pid, table.n[pid], table.d[pid])
                for child, fragment, side in ((f, front, 1), (b, back, 2)):
                    if fragment is not None and len(fragment.verts) >= 4:
                        pending.append((child, fragment, node, side))
            else:
                center = region.verts.mean(0)
                values = np.r_[center[None], center + .97 * (region.verts - center),
                               center + .5 * (region.verts - center)]
                owners.extend([len(leaves)] * len(values))
                samples.append(values)
                leaves.append((region, parent, slot, node))
        samples, owners = np.concatenate(samples), np.array(owners)
        clearance = margin(samples, volumes)
        labels = np.array([leaf[3] for leaf in leaves])[owners]
        bad = set(owners[((labels == -2) & (clearance > tolerance)) |
                         ((labels == -1) & (clearance < -tolerance))].tolist())
        actual = tree.hit(origins, directions, length)
        both = np.isfinite(actual) & np.isfinite(expected)
        wrong = np.zeros(len(actual), bool)
        wrong[both] = np.abs(actual[both] - expected[both]) > tolerance
        witnesses = [points]
        for fraction in (.05, .25, .5, .75, .95):
            distance = actual[wrong] * fraction + expected[wrong] * (1 - fraction)
            witnesses.append(origins[wrong] + directions[wrong] * distance[:, None])
        # Include missed-hit intervals without doing arithmetic on infinities.
        for got, wanted in ((actual, expected), (expected, actual)):
            missed = ~np.isfinite(got) & np.isfinite(wanted)
            for delta in (.02, .1, 1.):
                distance = np.minimum(wanted[missed] + delta, length)
                witnesses.append(origins[missed] + directions[missed] * distance[:, None])
        witnesses = np.concatenate(witnesses)
        labels, parents, slots = leaf_edges(tree, witnesses)
        clearance = margin(witnesses, volumes)
        wrong = ((labels == -2) & (clearance > tolerance)) | ((labels == -1) & (clearance < -tolerance))
        edges = set(zip(parents[wrong].tolist(), slots[wrong].tolist()))
        bad.update(i for i, (_, p, s, _) in enumerate(leaves) if (p, s) in edges)
        print(f"  repair {iteration}: {len(leaves)} leaves, {len(bad)} bad regions", flush=True)
        if not bad:
            break
        fragments = ([bb.make_frag(table, v["vertices"], v["planes"][:, :3], v["planes"][:, 3]) for v in volumes]
                     if local_builder == "fragments" else [])
        before = len(tree.nodes)
        new_planes = []
        for i in sorted(bad):
            region, parent, slot, _ = leaves[i]
            if local_builder == "planes":
                root = region_plane_union(tree, region, volumes)
                if parent < 0:
                    tree.root = root
                else:
                    node = list(tree.nodes[parent])
                    node[slot] = root
                    tree.nodes[parent] = tuple(node)
                continue
            rlo, rhi = region.verts.min(0), region.verts.max(0)
            local_frags = []
            for volume, fragment in zip(volumes, fragments):
                if np.any(volume["bounds"][0] > rhi + tolerance) or np.any(volume["bounds"][1] < rlo - tolerance):
                    continue
                clipped = fragment
                for pid, orientation in region.sides.items():
                    front, back = bb.split_frag(clipped, pid, table.n[pid], table.d[pid])
                    clipped = back if orientation == 1 else front
                    if clipped is None:
                        break
                if clipped is not None and len(clipped.verts) >= 4:
                    local_frags.append(bb.Frag(clipped.verts, clipped.inc, clipped.sides,
                                               clipped.used | set(region.sides)))
            local = []
            root = bb.build(local_frags, table, local, np.random.default_rng(0))
            offset = len(tree.nodes)
            remap = lambda child: child + offset if child >= 0 else child
            for pid, front, back in local:
                n, d = table.n[pid], table.d[pid]
                p = len(tree.planes) + len(new_planes)
                new_planes.append([*n, d, 3 + int(np.argmax(np.abs(n)))])
                tree.nodes.append((p, remap(front), remap(back)))
            if parent < 0:
                tree.root = remap(root)
            else:
                node = list(tree.nodes[parent])
                node[slot] = remap(root)
                tree.nodes[parent] = tuple(node)
        if new_planes:
            disk = np.array(new_planes)
            disk[:, :4] = disk[:, :4].astype(np.float32)
            tree.planes = np.r_[tree.planes, disk]
        rounds.append({"bad_regions": len(bad), "appended_nodes": len(tree.nodes) - before})
    tree.bound(low, high)
    tree.compact()
    return {"seconds": time.perf_counter() - started, "rounds": rounds, "nodes": len(tree.nodes)}


def validate(tree, volumes):
    points, origins, directions, length = probes(volumes, 872025, 250000, 8192)
    clearance = margin(points, volumes)
    actual = tree.contents(points) == -2
    result = {"points": len(points), "rays": len(origins),
              "extra_solid": int(np.sum(actual & (clearance > TOL))),
              "missing_solid": int(np.sum(~actual & (clearance < -TOL))),
              "ray_checks": ray_stats(tree.hit(origins, directions, length),
                                      convex_hit(origins, directions, length, volumes))}
    low = np.min([v["bounds"][0] for v in volumes], axis=0)
    high = np.max([v["bounds"][1] for v in volumes], axis=0)
    rng = np.random.default_rng(120872)
    exterior = rng.uniform(low - (high - low), high + (high - low), (100000, 3))
    exterior = exterior[np.any(exterior < low - 3, axis=1) | np.any(exterior > high + 3, axis=1)]
    result["exterior_points"] = len(exterior)
    result["exterior_extra_solid"] = int(np.sum(tree.contents(exterior) == -2))
    checked = structure(tree.view())
    result["depth"] = checked["hulls"]["1"]["depth"]
    return result


def splice(background, terrain):
    base = HullFile(background)
    if len(base.models) != 1:
        raise ValueError("Experiment currently supports one merged world model")
    planes = base.lumps[1]
    nodes, heads = [], []
    for hull in (1, 2, 3):
        tree = terrain[hull]
        pshift = len(planes) // 20
        planes += b"".join(struct.pack("<4fi", *p[:4], int(p[4])) for p in tree.planes)
        combined = [(p + pshift, f, b) for p, f, b in tree.nodes]
        terrain_root, memo = tree.root, {}

        def overlay(index):
            if index == -1:
                return terrain_root
            if index < 0:
                if index != -2:
                    raise ValueError("Background must contain only EMPTY and SOLID")
                return index
            if index not in memo:
                p, f, b = map(int, base.nodes[index])
                front, back = overlay(f), overlay(b)
                memo[index] = len(combined)
                combined.append((p, front, back))
            return memo[index]

        root = overlay(int(base.models[0]["head"][hull]))
        local, root = compact(combined, root)
        offset = len(nodes)
        heads.append(root + offset if root >= 0 else root)
        nodes.extend((p, f + offset if f >= 0 else f, b + offset if b >= 0 else b) for p, f, b in local)
    if len(nodes) > 32767:
        raise ValueError("GoldSrc clipnode budget exceeded")
    lumps = list(base.lumps)
    lumps[1] = planes
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
    result = HullFile(data=bytes(data))
    checked = structure(result)
    for i in range(15):
        if i not in (1, 9, 14) and result.lumps[i] != base.lumps[i]:
            raise ValueError("Visible geometry or lighting changed during collision splice")
    rng = np.random.default_rng(33524)
    points = rng.uniform(base.models[0]["mins"] - 128, base.models[0]["maxs"] + 128, (150000, 3))
    entities = parse_map(base.lumps[0].rstrip(b"\0").decode("latin-1")).entities
    starts = []
    for entity in entities:
        if entity.classname in ("info_player_start", "info_player_deathmatch", "info_player_coop"):
            origin = np.array(entity.keyvalues.get("origin", "").split(), dtype=float)
            if origin.shape != (3,) or not np.isfinite(origin).all():
                raise ValueError("Invalid player start origin")
            starts.append(origin)
    for hull in (1, 2, 3):
        expected = (base.contents(points, hull) == -2) | (terrain[hull].contents(points) == -2)
        if np.any((result.contents(points, hull) == -2) != expected):
            raise ValueError("Background union mismatch")
        if starts and hull in (1, 3) and np.any(result.contents(np.array(starts), hull) != -1):
            raise ValueError(f"Player start is blocked in hull {hull}")
    checked["player_starts"] = [point.tolist() for point in starts]
    return bytes(data), checked


def surface_compare(closed, surface, closed_volumes, surface_volumes, triangles, half):
    xylo, xyhi = triangles.min((0, 1))[:2], triangles.max((0, 1))[:2]
    zlo, zhi = triangles[:, :, 2].min() - half[2] - 40, triangles[:, :, 2].max() + half[2] + 80
    xs = np.linspace(xylo[0] - half[0] - 1, xyhi[0] + half[0] + 1, 257) + .0173205
    ys = np.linspace(xylo[1] - half[1] - 1, xyhi[1] + half[1] + 1, 257) + .0271828
    xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
    grid_origins = np.c_[xy, np.full(len(xy), zhi)]
    grid_directions = np.tile([0., 0., -1.], (len(xy), 1))
    rng = np.random.default_rng(204182)
    chosen = triangles[rng.integers(len(triangles), size=8192)]
    weights = rng.dirichlet([1, 1, 1], size=len(chosen))
    targets = np.sum(chosen * weights[:, :, None], axis=1)
    origins = np.c_[rng.uniform(xylo + 1, xyhi - 1, (len(chosen), 2)), np.full(len(chosen), zhi)]
    directions = targets - origins
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    length = float(np.linalg.norm([*(xyhi - xylo), zhi - zlo]) * 2)
    result, grid = {}, None
    for name, o, d, limit in (("vertical_grid", grid_origins, grid_directions, zhi - zlo),
                              ("oblique_above", origins, directions, length)):
        a, b = convex_hit(o, d, limit, closed_volumes), convex_hit(o, d, limit, surface_volumes)
        ca, cb = closed.hit(o, d, limit), surface.hit(o, d, limit)
        result[name] = {"rays": len(o), "geometry_closed_vs_surface": ray_stats(a, b),
                        "bsp_closed_vs_geometry": ray_stats(ca, a),
                        "bsp_surface_vs_geometry": ray_stats(cb, b),
                        "bsp_closed_vs_surface": ray_stats(ca, cb)}
        if name == "vertical_grid":
            grid = {"x": xs, "y": ys, "closed_top": (zhi - ca).reshape(257, 257),
                    "surface_top": (zhi - cb).reshape(257, 257)}
    return result, grid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "compiler", "dumper", "scripts", "wad", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--entity", type=int, required=True)
    parser.add_argument("--mode", choices=("compare", "closed"), default="compare")
    args = parser.parse_args()
    for name in ("source", "compiler", "dumper", "scripts", "wad", "output"):
        setattr(args, name, getattr(args, name).resolve())
    root = args.output
    root.mkdir(parents=True, exist_ok=True)
    manifest = {"source": str(args.source), "source_sha256": sha256(args.source), "entity": args.entity,
                "compiler_sha256": sha256(args.compiler), "dumper_sha256": sha256(args.dumper),
                "runner_sha256": sha256(__file__), "wad_sha256": sha256(args.wad),
                "scripts": {name: sha256(args.scripts / name) for name in SCRIPTS},
                "tolerance": TOL, "numpy": np.__version__}
    manifest["mode"] = args.mode
    if (root / "manifest.json").exists():
        if json.loads((root / "manifest.json").read_text()) != manifest:
            raise ValueError("Existing experiment has different inputs; choose a new output directory")
    else:
        save_json(root / "manifest.json", manifest)
        for folder in ("scripts", "compiler"):
            (root / folder).mkdir()
        shutil.copy2(args.source, root / "source_snapshot.map")
        shutil.copy2(args.compiler, root / "compiler/hltools.exe")
        shutil.copy2(args.dumper, root / "compiler/brush_geometry_dump.exe")
        shutil.copy2(__file__, root / "runner_snapshot.py")
        for name in SCRIPTS:
            shutil.copy2(args.scripts / name, root / "scripts" / name)
    compiler = root / "compiler/hltools.exe"
    exact = load_module("exacthull_experiment", root / "scripts/exacthull.py")
    bb = load_module("brushbsp_experiment", root / "scripts/brushbsp.py")
    exact.HALF = HALF
    exact.WAD = args.wad.as_posix()
    background = root / "background"
    if not (root / "geometry.json").exists() or not background.with_suffix(".bsp").exists():
        require_command([root / "compiler/brush_geometry_dump.exe", root / "source_snapshot.map",
                         root / "geometry.json", "--background", args.entity, background], root / "geometry.txt")
    closed, triangles = extract(root / "geometry.json", args.entity, args.mode == "compare")
    np.save(root / "top_triangles.npy", triangles)
    print(f"Input: {len(closed)} closed brushes, {len(triangles)} top triangles", flush=True)
    report = {"brushes": len(closed), "top_triangles": len(triangles), "hulls": {},
              "scope": "Top contact comparison; closed volumes and top-only shells can differ below or at sides.",
              "validation": "Independent seeds and unsnapped convex-volume oracle; sampling is not a whole-space proof."}
    variants = ("closed", "surface") if args.mode == "compare" else ("closed",)
    trees = {name: {} for name in variants}
    for hull in (1, 2, 3):
        volumes = {}
        record = {"half_size": HALF[hull].tolist(), "variants": {}}
        for name, shapes in (("closed", closed), ("surface", triangles)):
            if name not in variants:
                continue
            volumes[name] = expanded(shapes, HALF[hull], exact.CORNERS)
            prefix = root / f"{name}_h{hull}"
            print(f"{name} hull {hull}", flush=True)
            if prefix.with_suffix(".npz").exists():
                tree = Tree.load(prefix.with_suffix(".npz"))
                result = json.loads(prefix.with_suffix(".json").read_text())
            else:
                tree, result = compile_expanded(prefix.with_suffix(".map"), shapes, hull, exact, compiler)
                result["raw_drawnodes"] = len(tree.nodes)
                result["raw_validation"] = validate(tree, volumes[name])
                result["repair"] = repair(tree, volumes[name], bb)
                result["holdout"] = validate(tree, volumes[name])
                tree.save(prefix.with_suffix(".npz"))
                save_json(prefix.with_suffix(".json"), result)
            trees[name][hull] = tree
            record["variants"][name] = result
            print(json.dumps(result), flush=True)
        if args.mode == "compare":
            comparison, grid = surface_compare(trees["closed"][hull], trees["surface"][hull],
                                               volumes["closed"], volumes["surface"], triangles, HALF[hull])
            np.savez_compressed(root / f"surface_grid_h{hull}.npz", **grid)
            record["comparison"] = comparison
            print(json.dumps(comparison), flush=True)
        report["hulls"][str(hull)] = record
        save_json(root / "results.json", report)
    if not (root / "background_lit.bsp").exists():
        report["background_seconds"] = {}
        for stage in ("bsp", "vis", "rad"):
            report["background_seconds"][stage] = require_command(
                [compiler, stage, background.with_suffix(".bsp"), "-threads", "2"], root / f"background.{stage}.txt")
        shutil.copy2(background.with_suffix(".bsp"), root / "background_lit.bsp")
    else:
        report["background_seconds"] = {stage: json.loads((root / f"background.{stage}.command.json").read_text())["seconds"]
                                         for stage in ("bsp", "vis", "rad")}
    lit = HullFile(root / "background_lit.bsp")
    if len(lit.lumps[8]) <= 1 or not any(lit.lumps[8]):
        raise ValueError("Background has no nonzero lighting samples")
    report["playable"] = {}
    for name in variants:
        data, checked = splice(root / "background_lit.bsp", trees[name])
        path = root / f"brush_{name}_test.bsp"
        path.write_bytes(data)
        checked.update(path=str(path), sha256=sha256(path),
                       lighting_bytes=len(HullFile(data=data).lumps[8]),
                       visibility_bytes=len(HullFile(data=data).lumps[4]))
        report["playable"][name] = checked
    if sha256(args.source) != manifest["source_sha256"]:
        raise ValueError("Source MAP changed during the experiment")
    save_json(root / "results.json", report)
    print(json.dumps(report["playable"], indent=2), flush=True)


if __name__ == "__main__":
    main()
