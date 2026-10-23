"""Compose cropped source solids from exposed 3D boundary half-spaces.

Only outward half-spaces of real source boundary faces may enter a term.
Unrealizable affine-plane sign patterns are unspecified. This bounded
experiment verifies the source composition before applying legacy offsets.
"""

import argparse
from fractions import Fraction
import hashlib
import itertools
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from brush_exterior_pipeline import extract_exterior, split_polygon
from brush_surface_pipeline import HALF, load_module, ray_stats, repair, validate
from concave_support_diagnose import shift_legacy
from legacy_clip_diagnose import convex_hit, margin, save_json, sha256
from legacy_union_pipeline import compile_volumes
from surface_seam_audit import EPSILONS, pair_origins

HULL_RECOVERIES = 0
RECOVERY_EXAMPLE = None


def exact_facets(points):
    """Use rational orientation predicates for a small failed hull."""
    faces = set()
    values = [[Fraction.from_float(float(c)) for c in p] for p in points]
    for ids in itertools.combinations(range(len(points)), 3):
        p, q, r = [values[i] for i in ids]
        a, b = [q[k] - p[k] for k in range(3)], [r[k] - p[k] for k in range(3)]
        normal = [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]
        if not any(normal):
            continue
        distances = [sum(normal[k] * (v[k] - p[k]) for k in range(3)) for v in values]
        if min(distances) >= 0 or max(distances) <= 0:
            faces.add(tuple(i for i, d in enumerate(distances) if not d))
    triangles = []
    for face in sorted(faces):
        if len(face) == 3:
            triangles.append(face)
        else:
            cloud = points[list(face)]
            centered = cloud - cloud.mean(0)
            _, scales, axes = np.linalg.svd(centered, full_matrices=False)
            projected = (centered @ axes[:2].T) / scales[:2]
            order = ConvexHull(projected).vertices
            triangles.extend((face[order[0]], face[order[i]], face[order[i+1]]) for i in range(1, len(order)-1))
    if not triangles:
        raise ValueError("Exact orientation recovery found no bounded hull")
    triangles = np.array(triangles, dtype=int)
    center = points.mean(0)
    tetrahedra = points[triangles] - center
    volume = float(abs(np.linalg.det(tetrahedra)).sum() / 6)
    return triangles, volume


def box_planes(bounds):
    low, high = np.asarray(bounds)
    return np.r_[np.c_[np.eye(3), high], np.c_[-np.eye(3), -low]]


def polyhedron(points):
    global HULL_RECOVERIES
    unique = []
    for point in points:
        if not unique or np.max(abs(np.array(unique) - point), axis=1).min() > 1e-10:
            unique.append(point)
    points = np.array(unique)
    if len(points) < 4:
        return None
    # Affine preconditioning preserves coordinates and avoids Qhull failures
    # on long, extremely thin source slivers without perturbing their points.
    centered = points - points.mean(0)
    _, scales, axes = np.linalg.svd(centered, full_matrices=False)
    if scales[-1] < 1e-11:
        return None
    conditioned = (centered @ axes.T) / scales
    try:
        hull = ConvexHull(conditioned)
        vertices, simplices = hull.vertices, hull.simplices
        volume = float(hull.volume * np.prod(scales))
    except QhullError:
        if len(points) > 64:
            raise ValueError("Failed hull is too large for exact-predicate recovery")
        if RECOVERY_EXAMPLE and not RECOVERY_EXAMPLE.exists():
            save_json(RECOVERY_EXAMPLE, points.tolist())
        simplices, volume = exact_facets(points)
        vertices = np.unique(simplices)
        HULL_RECOVERIES += 1
    if volume < 1e-8:
        return None
    remap = np.full(len(points), -1, dtype=int)
    remap[vertices] = np.arange(len(vertices))
    simplices = remap[simplices]
    points = points[vertices]
    return {"vertices": points, "volume": volume, "simplices": simplices,
            "bounds": np.array([points.min(0), points.max(0)])}


def box(bounds):
    low, high = np.asarray(bounds)
    return polyhedron(np.array(list(itertools.product(*zip(low, high)))))


def clip(volume, plane):
    if volume is None:
        return None
    points = volume["vertices"]
    offsets = points @ plane[:3] - plane[3]
    if offsets.max() <= 1e-8:
        return volume
    if offsets.min() >= -1e-8:
        return None
    edges = set()
    for tri in volume["simplices"]:
        for a, b in itertools.combinations(tri, 2):
            edges.add(tuple(sorted((int(a), int(b)))))
    cuts = []
    for a, b in edges:
        if offsets[a] * offsets[b] < 0:
            cuts.append(points[a] + (points[b] - points[a]) * offsets[a] / (offsets[a] - offsets[b]))
    retained = points[offsets <= 0]
    return polyhedron(np.r_[retained, np.array(cuts).reshape(-1, 3)])


def intersect(bounds, planes):
    result = box(bounds)
    for plane in planes:
        result = clip(result, plane)
        if result is None:
            return None
    return result


def outside_parts(volume, planes):
    """Partition the portion outside a convex source brush into convex pieces."""
    remaining, outside = volume, []
    for plane in planes:
        if remaining is None:
            break
        points = remaining["vertices"]
        offsets = points @ plane[:3] - plane[3]
        if offsets.max() <= 1e-8:
            continue
        if offsets.min() >= -1e-8:
            outside.append(remaining)
            return outside
        front = clip(remaining, -plane)
        if front is not None:
            outside.append(front)
        remaining = clip(remaining, plane)
    return outside


def outside_fragments(volume, source):
    remaining = [volume]
    for brush in source:
        pieces = []
        for piece in remaining:
            if np.any(piece["bounds"][0] >= brush["bounds"][1] - 1e-8) or np.any(piece["bounds"][1] <= brush["bounds"][0] + 1e-8):
                pieces.append(piece)
            else:
                pieces.extend(outside_parts(piece, brush["planes"]))
        remaining = pieces
        if not remaining:
            break
        if len(remaining) > 10000:
            raise ValueError("Source subtraction exceeded the bounded experiment budget")
    return remaining


def outside_volume(volume, source):
    return float(sum(p["volume"] for p in outside_fragments(volume, source)))


def compose_source(planes, bounds, source, max_pieces=20000, checkpoint=None, resume=None,
                   containment_source=None):
    """Cover source brushes by verified boundary terms, splitting only as needed."""
    pending = []
    for brush in source:
        volume = intersect(bounds, brush["planes"])
        if volume is not None:
            pending.append(volume)
    accepted, cache, splits, visited = {}, {}, 0, 0
    maximum_split_error = 0.
    signature = hashlib.sha256(np.array(planes).tobytes() + np.array(bounds).tobytes()
                               + b"".join(b["planes"].tobytes() for b in source)).hexdigest()
    allowed = source if containment_source is None else containment_source
    if containment_source is not None:
        signature = hashlib.sha256(signature.encode() + b"".join(b["planes"].tobytes() for b in allowed)).hexdigest()
    if resume:
        saved = json.loads(Path(resume).read_text())
        if saved["input_signature"] != signature:
            raise ValueError("Adaptive checkpoint source/boundary inputs differ")
        pending = [{k: np.array(v) if isinstance(v, list) else v for k, v in p.items()} for p in saved["pending"]]
        # Recondition checkpoint geometry with the current clipping backend.
        pending = [polyhedron(p["vertices"]) for p in pending]
        if any(p is None for p in pending):
            raise ValueError("Adaptive checkpoint contains a degenerate source piece")
        accepted = {tuple(t["members"]): t["excess"] for t in saved["accepted"]}
        splits, visited = saved["splits"], saved["visited"]
        maximum_split_error = saved["maximum_split_error"]
    while pending:
        piece = pending.pop()
        visited += 1
        if visited > max_pieces:
            raise ValueError("Adaptive boundary composition exceeded the piece budget")
        offsets = piece["vertices"] @ planes[:, :3].T - planes[:, 3]
        members = tuple(np.flatnonzero(offsets.max(0) <= 1e-7).tolist())
        if members in accepted:
            continue
        # Fewer constraints mean a larger closure. A previously accepted
        # subset covers this piece, including after the common plane offsets.
        member_set = set(members)
        if any(member_set.issuperset(term) for term in accepted):
            continue
        if members not in cache:
            closure = intersect(bounds, planes[list(members)])
            if closure is None:
                raise ValueError("Boundary closure lost its source piece")
            fragments = outside_fragments(closure, allowed)
            excess = sum(f["volume"] for f in fragments)
            witness = max(fragments, key=lambda f: f["volume"])["vertices"].mean(0) if fragments else None
            cache[members] = (excess, witness)
        excess, witness = cache[members]
        if excess <= .001:
            accepted[members] = excess
        else:
            outside = witness @ planes[:, :3].T - planes[:, 3]
            mixed = (offsets.min(0) < -1e-7) & (offsets.max(0) > 1e-7) & (outside > 1e-7)
            if not mixed.any():
                raise ValueError("Source piece cannot be separated from empty space by outward boundary terms")
            scores = np.where(mixed, outside / np.maximum(offsets.max(0) - offsets.min(0), 1e-8), -np.inf)
            halves, fallback = None, None
            for index in np.argsort(scores)[::-1]:
                if not mixed[index]:
                    break
                candidate = [clip(piece, planes[index]), clip(piece, -planes[index])]
                valid = [p for p in candidate if p is not None]
                if len(valid) == 2:
                    halves = valid
                    break
                if valid and abs(valid[0]["volume"] - piece["volume"]) <= .001:
                    fallback = valid
            if halves is None:
                halves = fallback
            if halves is None:
                raise ValueError("Adaptive split did not produce valid source pieces")
            split_error = abs(sum(p["volume"] for p in halves) - piece["volume"])
            maximum_split_error = max(maximum_split_error, split_error)
            if split_error > .001:
                raise ValueError("Adaptive split lost source volume")
            pending.extend(halves)
            splits += 1
        if visited % 50 == 0:
            print(f"Boundary composition: {visited} pieces, {len(pending)} pending, {len(accepted)} terms", flush=True)
            if checkpoint:
                save_json(checkpoint, {"input_signature": signature, "visited": visited, "splits": splits,
                    "maximum_split_error": maximum_split_error,
                    "pending": [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in p.items()} for p in pending],
                    "accepted": [{"members": list(t), "excess": excess} for t, excess in accepted.items()]})
    kept = []
    for term in sorted(accepted, key=lambda t: (len(t), t)):
        if not any(set(other).issubset(term) for other in kept):
            kept.append(list(term))
    print(f"Boundary composition: {len(kept)} terms after absorption; checking source coverage", flush=True)
    target = term_volumes(planes, kept, bounds)
    missing = 0.
    for index, brush in enumerate(source):
        original = intersect(bounds, brush["planes"])
        if original is not None:
            missing += outside_volume(original, target)
        if (index + 1) % 50 == 0:
            print(f"Source coverage: {index + 1}/{len(source)} brushes checked", flush=True)
    if missing > .001:
        raise ValueError(f"Adaptive composition loses source geometry: {missing} cubic units")
    excess = float(sum(accepted[tuple(t)] for t in kept))
    if excess > .001:
        raise ValueError(f"Adaptive composition exceeds its containment union: {excess} cubic units")
    return kept, {"visited_source_pieces": visited, "adaptive_splits": splits,
                  "containment": "source" if containment_source is None else "supplied_envelope",
                  "maximum_split_volume_error": maximum_split_error,
                  "term_excess_volume_sum": excess,
                  "missing_source_volume_sum": missing, "source_coverage_checked": True}


def boundary_planes(triangles, normals, bounds):
    found = []
    sides = box_planes(bounds)
    for tri, normal in zip(triangles, normals):
        polygon = tri
        for side in sides:
            _, polygon = split_polygon(polygon, side[:3], side[3])
            if polygon is None:
                break
        if polygon is None:
            continue
        plane = np.r_[normal, tri[0] @ normal]
        if not any(np.max(abs(plane - p)) < 1e-7 for p in found):
            found.append(plane)
    if not found:
        raise ValueError("The crop contains no source boundary")
    return np.array(found)


def arrangement(planes, bounds, source, max_cells=20000):
    cells = [(box(bounds), ())]
    for plane in planes:
        divided = []
        for volume, signs in cells:
            for sign in (True, False):
                piece = clip(volume, plane if sign else -plane)
                if piece is not None:
                    divided.append((piece, (*signs, sign)))
        cells = divided
        if len(cells) > max_cells:
            raise ValueError("Boundary arrangement exceeded the bounded experiment budget")
    centers = np.array([v["vertices"].mean(0) for v, _ in cells])
    distances = margin(centers, source)
    if np.any(abs(distances) < 1e-7):
        raise ValueError("An arrangement cell has an ambiguous source classification")
    signs = np.array([s for _, s in cells], dtype=bool)
    solid = distances < 0
    total = sum(v["volume"] for v, _ in cells)
    expected = float(np.prod(np.asarray(bounds)[1] - np.asarray(bounds)[0]))
    if abs(total - expected) > .001:
        raise ValueError(f"Arrangement lost source-domain volume: {total} vs {expected}")
    source = [b for b in source if np.all(b["bounds"][1] > np.asarray(bounds)[0])
              and np.all(b["bounds"][0] < np.asarray(bounds)[1])]
    maximum_inconsistent_volume = 0.
    for (volume, _), is_solid in zip(cells, solid):
        if is_solid:
            inconsistent = outside_volume(volume, source)
        else:
            inconsistent = 0.
            for brush in source:
                if np.any(volume["bounds"][0] >= brush["bounds"][1]) or np.any(volume["bounds"][1] <= brush["bounds"][0]):
                    continue
                overlap = volume
                for p in brush["planes"]:
                    overlap = clip(overlap, p)
                    if overlap is None:
                        break
                if overlap is not None:
                    inconsistent += overlap["volume"]
        maximum_inconsistent_volume = max(maximum_inconsistent_volume, inconsistent)
    if maximum_inconsistent_volume > .001:
        raise ValueError(f"Boundary planes do not separate source occupancy: {maximum_inconsistent_volume} cubic units")
    return signs, solid, {"cells": len(cells), "solid_cells": int(solid.sum()),
                          "volume_partition_error": abs(total - expected),
                          "max_cell_classification_inconsistent_volume": maximum_inconsistent_volume}


def compose(signs, solid):
    inside, outside = signs[solid], signs[~solid]
    if not len(inside) or not len(outside):
        raise ValueError("Choose a crop containing both solid and empty space")
    terms = set()
    for seed in inside:
        members = np.flatnonzero(seed).tolist()
        if np.any(outside[:, members].all(1)):
            raise ValueError("Outward boundary half-spaces cannot represent this source arrangement monotonically")
        # Remove constraints only if no realizable empty cell becomes covered.
        for index in members.copy():
            candidate = [j for j in members if j != index]
            if not np.any(outside[:, candidate].all(1)):
                members = candidate
        terms.add(frozenset(members))
    kept = []
    for term in sorted(terms, key=lambda t: (len(t), sorted(t))):
        if not any(t.issubset(term) for t in kept):
            kept.append(term)
    # Prefer terms that cover many still-uncovered solid cells per constraint.
    pending, chosen = np.ones(len(inside), bool), []
    while pending.any():
        candidates = [(np.sum(inside[pending][:, sorted(t)].all(1)) / max(1, len(t)), t) for t in kept]
        score, term = max(candidates, key=lambda item: (item[0], -len(item[1]), tuple(sorted(item[1]))))
        if score <= 0:
            raise ValueError("Boundary terms do not cover the source")
        chosen.append(sorted(term))
        pending &= ~inside[:, sorted(term)].all(1)
        kept.remove(term)
    represented = np.any([signs[:, term].all(1) for term in chosen], axis=0)
    if np.any(represented != solid):
        raise ValueError("Boundary composition changed an arrangement cell")
    return chosen


def supporting_planes(volume, candidates):
    active = []
    for p in candidates:
        face = volume["vertices"][abs(volume["vertices"] @ p[:3] - p[3]) < 1e-6]
        # Nearby vertices can make a real supporting face have numerical rank
        # three. Additional rank must not delete an essential half-space.
        if len(face) >= 3 and np.linalg.matrix_rank(face[1:] - face[0], tol=1e-7) >= 2:
            active.append(p)
    return np.array(active).reshape(-1, 4)


def term_volumes(planes, terms, bounds):
    result = []
    for term in terms:
        volume = intersect(bounds, planes[term])
        if volume is None:
            raise ValueError("An essential boundary term is empty")
        # Keep exact source plane coefficients instead of refitting Qhull faces.
        candidates = np.r_[planes[term], box_planes(bounds)]
        active = supporting_planes(volume, candidates)
        reconstructed = intersect(bounds, active)
        if (reconstructed is None or
                np.max(reconstructed["vertices"] @ candidates[:, :3].T - candidates[:, 3]) > 1e-5):
            raise ValueError("Supporting-plane reduction changes a boundary term")
        volume["planes"] = active
        result.append(volume)
    return result


def verify_source(target, source, signs, solid, terms):
    excess = sum(outside_volume(v, source) for v in target)
    if excess > .001:
        raise ValueError(f"Boundary composition extends outside the source: {excess} cubic units")
    represented = np.any([signs[:, term].all(1) for term in terms], axis=0)
    if np.any(solid & ~represented):
        raise ValueError("Boundary composition loses a solid arrangement cell")
    return {"term_excess_volume_sum": excess, "solid_cells_covered": True}


def main():
    global RECOVERY_EXAMPLE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--entity", type=int, required=True)
    parser.add_argument("--probe", type=float, nargs=2, action="append", required=True)
    parser.add_argument("--radius", type=float, default=64.)
    parser.add_argument("--method", choices=("arrangement", "adaptive"), default="arrangement")
    parser.add_argument("--whole-entity", action="store_true")
    parser.add_argument("--geometry-only", action="store_true")
    parser.add_argument("--resume-composition", type=Path)
    args = parser.parse_args()
    if args.radius <= 1.:
        parser.error("Choose a positive crop radius greater than one")
    if args.whole_entity and (args.method != "adaptive" or len(args.probe) != 1):
        parser.error("Whole-entity experiments require adaptive composition and one probe")
    base, root = args.build.resolve(), args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    RECOVERY_EXAMPLE = root / "exact_recovery_example.json"
    for name in ("geometry.json", "hltools.exe", "exacthull.py", "brushbsp.py"):
        shutil.copy2(base / name, root / name)
    shutil.copy2(__file__, root / "runner_snapshot.py")
    geometry = json.loads((root / "geometry.json").read_text())
    triangles, normals, source, extraction = extract_exterior(geometry, args.entity)
    low, high = triangles.min((0, 1)), triangles.max((0, 1))
    zlo, zhi = float(low[2] - 100), float(high[2] + 200)
    exact = load_module("boundary_exacthull", root / "exacthull.py")
    exact.WAD = ""
    exact.fmt = lambda p: "( " + " ".join(f"{v:.12f}" for v in p) + " )"
    bb = load_module("boundary_brushbsp", root / "brushbsp.py")
    bb.EPS = 1e-5
    report = {"reference_build": str(base), "entity": args.entity, "method": args.method,
              "inputs": {name: sha256(root / name) for name in ("geometry.json", "hltools.exe", "exacthull.py", "brushbsp.py")},
              "runner_sha256": sha256(__file__), "extraction": extraction,
              "whole_entity": args.whole_entity,
              "scope": ("Whole-entity boundary composition within a padded domain. No game installation."
                        if args.whole_entity else
                        "Cropped full-boundary experiments, with artificial unshifted crop walls. No game installation."),
              "patches": []}
    for index, (x, y) in enumerate(args.probe):
        bounds = np.array([[x - args.radius, y - args.radius, zlo], [x + args.radius, y + args.radius, zhi]])
        if args.whole_entity:
            bounds = np.array([low - 100., high + 100.])
            bounds[0, 2], bounds[1, 2] = zlo, zhi
        planes = boundary_planes(triangles, normals, bounds)
        if args.method == "adaptive":
            terms, record = compose_source(planes, bounds, source,
                                            checkpoint=root / "composition_checkpoint.json", resume=args.resume_composition)
        else:
            signs, solid, record = arrangement(planes, bounds, source)
            terms = compose(signs, solid)
        record.update({"xy": [x, y], "bounds": bounds.tolist(), "planes": planes.tolist(), "terms": terms, "hulls": {}})
        save_json(root / "construction.json", record)
        unshifted = term_volumes(planes, terms, bounds)
        if args.method == "arrangement":
            record.update(verify_source(unshifted, source, signs, solid, terms))
        xs = np.linspace(bounds[0, 0] + .001, bounds[1, 0] - .001, 129)
        ys = np.linspace(bounds[0, 1] + .001, bounds[1, 1] - .001, 129)
        grid = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
        origins = np.c_[grid, np.full(len(grid), zhi - 1)]
        directions = np.tile([0., 0., -1.], (len(grid), 1))
        original = convex_hit(origins, directions, zhi - zlo - 2, source)
        comparison = ray_stats(convex_hit(origins, directions, zhi - zlo - 2, unshifted), original)
        if comparison["hit_miss_differences"] or comparison["max_distance_error"] > 1e-4:
            raise ValueError(f"Boundary source contacts do not match closed brushes: {comparison}")
        record["source_contact_grid"] = comparison
        rng = np.random.default_rng(742761)
        points = rng.uniform(bounds[0], bounds[1], (250000, 3))
        a, b = margin(points, source), margin(points, unshifted)
        away = (abs(a) > 1e-5) & (abs(b) > 1e-5)
        mismatches = int(np.sum(away & ((a < 0) != (b < 0))))
        if mismatches:
            raise ValueError(f"Unshifted source holdout failed: {mismatches} points")
        record["unshifted_point_mismatches"] = mismatches
        if args.method == "arrangement":
            np.savez_compressed(root / f"patch{index}_source.npz", signs=signs, solid=solid)
        save_json(root / "construction.json", record)
        for hull in (1, 2, 3):
            shifted = shift_legacy(planes, HALF[hull])
            target = term_volumes(shifted, terms, bounds)
            save_json(root / f"patch{index}_h{hull}.json", [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in t.items()} for t in target])
            if args.geometry_only:
                record["hulls"][str(hull)] = {"convex_terms": len(target),
                    "active_plane_count": sum(len(v["planes"]) for v in target)}
                print(f"Patch {index}, hull {hull}: {len(planes)} boundary planes, {len(terms)} terms, geometry only", flush=True)
                continue
            tree, seconds = compile_volumes(root / f"patch{index}_h{hull}.map", target, exact, root / "hltools.exe")
            checks = validate(tree, target)
            if (checks["extra_solid"] or checks["missing_solid"] or checks["exterior_extra_solid"]
                    or checks["ray_checks"]["hit_miss_differences"] or checks["ray_checks"]["errors_over_0_01"]):
                repair(tree, target, bb, tolerance=1e-4, local_builder="planes",
                       extra_rays=(origins, directions, zhi - zlo - 2))
                tree.compact()
                checks = validate(tree, target)
            got = tree.hit(origins, directions, zhi - zlo - 2)
            wanted = convex_hit(origins, directions, zhi - zlo - 2, target)
            top_checks = ray_stats(got, wanted)
            if (checks["extra_solid"] or checks["missing_solid"] or checks["exterior_extra_solid"]
                    or checks["ray_checks"]["hit_miss_differences"] or checks["ray_checks"]["errors_over_0_01"]
                    or top_checks["hit_miss_differences"] or top_checks["errors_over_0_01"]
                    or np.any(wanted > original + 1e-5)):
                raise ValueError("Boundary offset failed geometry or source-retention checks")
            pairs = pair_origins([[x, y]], zhi - 1)
            o = pairs.reshape(-1, 3)
            d = np.tile([0., 0., -1.], (len(o), 1))
            tops = (zhi - 1 - tree.hit(o, d, zhi - zlo - 2)).reshape(3, 2)
            record["hulls"][str(hull)] = {"nodes": len(tree.nodes), "compile_seconds": seconds,
                "geometry_checks": checks, "contact_grid": top_checks,
                "witness_top_pairs": tops.tolist(), "epsilons": EPSILONS.tolist()}
            tree.save(root / f"patch{index}_h{hull}.npz")
            print(f"Patch {index}, hull {hull}: {len(planes)} boundary planes, {len(terms)} terms, "
                  f"{len(tree.nodes)} nodes, witness delta {abs(tops[-1, 1] - tops[-1, 0]):.6f}", flush=True)
        report["patches"].append(record)
        report["verified_hull_recoveries"] = HULL_RECOVERIES
        save_json(root / "results.json", report)
    print(f"Results: {root}", flush=True)


if __name__ == "__main__":
    main()
