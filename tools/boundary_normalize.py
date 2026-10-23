"""Conservatively regularize small terrain seams within a vertical envelope.

Source solids remain covered. Every normalized term must lie inside the union
of source brushes extruded upward by the explicit distance limit. Thin walls
are candidates for removal; substantial walls and the footprint are retained.
"""

import argparse
import json
from pathlib import Path
import shutil
import time

import numpy as np
from scipy.spatial import ConvexHull

from boundary_lattice_diagnose import boundary_planes, compose_source, term_volumes
from brush_exterior_pipeline import extract_exterior
from brush_surface_pipeline import HALF, ray_stats
from concave_support_diagnose import shift_legacy
from legacy_clip_diagnose import convex_hit, margin, save_json, sha256


def vertical_envelope(source, distance):
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError("Vertical envelope distance must be finite and positive")
    result = []
    for brush in source:
        planes = brush["planes"].copy()
        planes[:, 3] += distance * np.maximum(planes[:, 2], 0.)
        # Segment extrusion also creates silhouette walls. Shifting existing
        # faces alone would enlarge the XY footprint of a tetrahedron.
        outline = ConvexHull(brush["vertices"][:, :2]).equations
        walls = np.c_[outline[:, :2], np.zeros(len(outline)), -outline[:, 2]]
        bounds = brush["bounds"].copy()
        bounds[1, 2] += distance
        result.append({"planes": np.r_[planes, walls], "bounds": bounds})
    return result


def seam_planes(planes, triangles, normals, limit):
    removed = []
    for index, plane in enumerate(planes):
        if abs(plane[2]) > 1e-8:
            continue
        matching = ((np.max(abs(normals - plane[:3]), axis=1) < 1e-7)
                    & (abs(np.einsum("ij,ij->i", triangles[:, 0], normals) - plane[3]) < 1e-6))
        faces = triangles[matching]
        if not len(faces):
            continue
        edges = faces[:, [1, 2, 0]] - faces
        span = np.linalg.norm(edges[:, :, :2], axis=2).max(1)
        twice_area = np.linalg.norm(np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0]), axis=1)
        height = np.divide(twice_area, span, out=np.full_like(span, np.inf), where=span > 1e-10)
        if height.max() <= limit:
            removed.append({"index": index, "plane": plane.tolist(), "triangles": len(faces),
                            "max_vertical_thickness": float(height.max())})
    return removed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--entity", type=int, required=True)
    parser.add_argument("--distance", type=float, default=.001)
    parser.add_argument("--resume-composition", type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.distance) or not 0 < args.distance <= .001:
        parser.error("Choose an explicit upward normalization bound in (0, 0.001]")
    base, root = args.build.resolve(), args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    for name in ("geometry.json", "hltools.exe", "exacthull.py", "brushbsp.py"):
        shutil.copy2(base / name, root / name)
    shutil.copy2(__file__, root / "runner_snapshot.py")
    geometry = json.loads((root / "geometry.json").read_text())
    triangles, normals, source, extraction = extract_exterior(geometry, args.entity)
    low, high = triangles.min((0, 1)), triangles.max((0, 1))
    bounds = np.array([low - 100., high + 100.])
    bounds[1, 2] += 100.
    original_planes = boundary_planes(triangles, normals, bounds)
    lift = args.distance / 4.
    removed = seam_planes(original_planes, triangles, normals, lift)
    removed_ids = {r["index"] for r in removed}
    planes = np.array([p for i, p in enumerate(original_planes) if i not in removed_ids])
    planes[:, 3] += lift * np.maximum(planes[:, 2], 0.)
    envelope = vertical_envelope(source, args.distance)
    print(f"Normalization: {len(removed)} thin planes removed, {len(planes)} retained; bound {args.distance}", flush=True)
    manifest = {"mode": "bounded_vertical_normalization", "maximum_upward_distance": args.distance,
                "roof_lift": lift, "removed_seam_planes": removed,
                "original_boundary_planes": len(original_planes), "extraction": extraction}
    save_json(root / "normalization.json", manifest)
    terms, proof = compose_source(planes, bounds, source, containment_source=envelope,
        checkpoint=root / "composition_checkpoint.json", resume=args.resume_composition)
    patch = {**proof, "planes": planes.tolist(), "terms": terms, "bounds": bounds.tolist(), "hulls": {}}
    save_json(root / "construction.json", patch)
    unshifted = term_volumes(planes, terms, bounds)
    rng = np.random.default_rng(124101)
    points = rng.uniform(bounds[0], bounds[1], (250000, 3))
    original, normalized, allowed = [margin(points, v) for v in (source, unshifted, envelope)]
    patch["unshifted_missing_source_points"] = int(np.sum((original < -1e-5) & (normalized > 1e-5)))
    patch["unshifted_outside_envelope_points"] = int(np.sum((normalized < -1e-5) & (allowed > 1e-5)))
    xy = np.stack(np.meshgrid(np.linspace(low[0] + .003, high[0] - .003, 257),
                             np.linspace(low[1] + .007, high[1] - .007, 257)), -1).reshape(-1, 2)
    zhi, length = high[2] + 50., high[2] - low[2] + 100.
    origins = np.c_[xy, np.full(len(xy), zhi)]
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    before = convex_hit(origins, directions, length, source)
    after = convex_hit(origins, directions, length, unshifted)
    patch["source_contact_grid"] = ray_stats(before, after)
    finite = np.isfinite(before) & np.isfinite(after)
    gain = before[finite] - after[finite]
    patch["source_contact_minimum_gain"] = float(gain.min())
    patch["source_contact_maximum_gain"] = float(gain.max())
    save_json(root / "construction.json", patch)
    if (patch["unshifted_missing_source_points"] or patch["unshifted_outside_envelope_points"]
            or patch["source_contact_grid"]["hit_miss_differences"]
            or gain.min() < -1e-5 or gain.max() > args.distance + 1e-5):
        raise ValueError("Normalized source failed independent source/envelope checks")
    for hull in (1, 2, 3):
        target = term_volumes(shift_legacy(planes, HALF[hull]), terms, bounds)
        save_json(root / f"patch0_h{hull}.json",
                  [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in t.items()} for t in target])
        patch["hulls"][str(hull)] = {"convex_terms": len(target), "active_plane_count": sum(len(v["planes"]) for v in target)}
        print(f"Hull {hull}: {len(target)} normalized terms exported", flush=True)
    report = {"entity": args.entity, "whole_entity": True, "method": "adaptive", "normalization": manifest,
              "inputs": {name: sha256(root / name) for name in ("geometry.json", "hltools.exe", "exacthull.py", "brushbsp.py")},
              "resume_checkpoint": ({"path": str(args.resume_composition.resolve()),
                                     "sha256": sha256(args.resume_composition)} if args.resume_composition else None),
              "runner_sha256": sha256(__file__), "patches": [patch], "wall_seconds": time.perf_counter() - started}
    save_json(root / "results.json", report)
    print(f"Verified normalized geometry: {root}", flush=True)


if __name__ == "__main__":
    main()
