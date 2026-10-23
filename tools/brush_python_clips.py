"""Feed an exposed MAP brush surface to the unchanged reference Python builders."""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import time
from types import SimpleNamespace

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np

from brush_exterior_pipeline import extract_exterior
import brush_surface_pipeline as surface
import source_clip_pipeline as reference
import source_clip_validate as validator
from source_clip_validate import structure, tree_first_hit


def select_triangles(triangles, normals, selection):
    if selection not in ("terrain", "shell"):
        raise ValueError("Unknown surface selection")
    selected = normals[:, 2] > 1e-6 if selection == "terrain" else np.ones(len(triangles), bool)
    original = triangles[selected]
    # These are exactly the grid and degeneracy test in build_boreas_clips.triangles.
    snapped = np.round(original / (1 / 16)) * (1 / 16)
    keep = np.linalg.norm(np.cross(snapped[:, 1] - snapped[:, 0],
                                   snapped[:, 2] - snapped[:, 0]), axis=1) > 1e-3
    if not keep.any():
        raise ValueError("No triangles survive surface selection and reference snapping")
    return snapped[keep], {
        "selection": selection, "selected_before_snap": len(original),
        "snap_degenerate_triangles": int((~keep).sum()),
        "max_vertex_snap": float(np.linalg.norm(snapped - original, axis=2).max()),
        "triangles": int(keep.sum()), "grid": 1 / 16,
    }


def prepare(args):
    source, geometry, manifest, root, scripts, compiler, wad = [Path(getattr(args, key)).resolve()
        for key in ("source", "geometry", "manifest", "output", "scripts", "compiler", "wad")]
    provenance = json.loads(manifest.read_text())
    if reference.sha256(source) != provenance["source_sha256"]:
        raise ValueError("Geometry export and source MAP hashes differ")
    for path in (geometry, compiler, wad, *(scripts / name for name in reference.SCRIPTS)):
        if not path.is_file():
            raise ValueError(f"Missing input: {path}")
    started = time.perf_counter()
    triangles, normals, _, extraction = extract_exterior(json.loads(geometry.read_text()), args.entity)
    selected, selection = select_triangles(triangles, normals, args.surface)
    root.mkdir(parents=True, exist_ok=False)
    (root / "scripts").mkdir()
    (root / "compiler").mkdir()
    hashes = {}
    for name in reference.SCRIPTS:
        shutil.copy2(scripts / name, root / "scripts" / name)
        hashes[name] = reference.sha256(scripts / name)
    snapshot = root / "compiler/hltools.exe"
    shutil.copy2(compiler, snapshot)
    shutil.copy2(source, root / "source_snapshot.map")
    shutil.copy2(geometry, root / "geometry.json")
    shutil.copy2(manifest, root / "geometry_manifest.json")
    np.savez_compressed(root / "exterior.npz", triangles=triangles, normals=normals)
    np.save(root / "terrain_tris.npy", selected)
    config = {
        "input_kind": "brush_exterior_triangles", "source": str(source),
        "source_sha256": reference.sha256(source), "output": str(root),
        "geometry": str(geometry), "geometry_sha256": reference.sha256(geometry),
        "geometry_manifest": str(manifest), "geometry_manifest_sha256": reference.sha256(manifest),
        "compiler_snapshot": str(snapshot), "compiler_sha256": reference.sha256(compiler),
        "compiler_threads": args.compiler_threads, "wad": str(wad), "script_sha256": hashes,
        "triangle_sha256": reference.sha256(root / "terrain_tris.npy"),
        "adapters": ["Source displacement parsing replaced by exposed brush triangle input",
                     "Reference 1/16 grid and cross-product degeneracy threshold retained",
                     "Original split/compile_chunk/base_template/splice/fix_file functions",
                     "Paths isolated; compiler logs, return codes and thread count recorded",
                     "Missing bsp_hull import supplied by compatibility reader"],
    }
    info = {"entity": args.entity, "extraction": extraction, **selection,
            "triangle_bounds": [selected.min((0, 1)).tolist(), selected.max((0, 1)).tolist()],
            "initial_chunks": math.ceil(len(selected) / 2400),
            "seconds": time.perf_counter() - started}
    reference.save_json(root / "config.json", config)
    reference.save_json(root / "input.json", info)
    print(json.dumps(info, indent=2), flush=True)


def package(args):
    root = Path(args.output).resolve()
    config = json.loads((root / "config.json").read_text())
    if reference.sha256(config["source"]) != config["source_sha256"]:
        raise ValueError("Source MAP changed")
    if reference.sha256(root / "terrain_tris.npy") != config["triangle_sha256"]:
        raise ValueError("Prepared triangles changed")
    paths = sorted((root / "clips_out").glob("bclip*.bsp"))
    if len(paths) != 1:
        raise ValueError("Test-map packaging currently requires exactly one reference clip file")
    collision = reference.HullFile(paths[0])
    raw_structure = structure(collision)
    if collision.models[0]["head"][2] != collision.models[0]["head"][1]:
        raise ValueError("Reference hull 2 no longer aliases hull 1")
    trees = {hull: surface.Tree(collision.planes, collision.nodes,
                               collision.models[0]["head"][hull]) for hull in (1, 2, 3)}
    data, checks = surface.splice(args.background, trees)
    destination = root / args.name
    destination.write_bytes(data)
    packaged = reference.HullFile(data=data)
    background = reference.HullFile(args.background)
    old_path = Path(args.compare).resolve()
    old = reference.HullFile(old_path)
    triangles = np.load(root / "terrain_tris.npy")
    low, high = triangles.min((0, 1)), triangles.max((0, 1))
    xs = np.linspace(low[0] + .173205, high[0] - .271828, 257)
    ys = np.linspace(low[1] + .271828, high[1] - .173205, 257)
    xy = np.stack(np.meshgrid(xs, ys), -1).reshape(-1, 2)
    origins = np.c_[xy, np.full(len(xy), high[2] + 100)]
    directions = np.tile([0., 0., -1.], (len(xy), 1))
    length = float(high[2] - low[2] + 250)
    hits = {name: tree_first_hit(tree, origins, directions, length, 1) for name, tree in
            (("python", packaged), ("previous", old), ("background", background))}
    tmin, tmax = triangles.min(1), triangles.max(1)

    def near(a, b, hull):
        half = surface.HALF[hull]
        return np.flatnonzero(np.all(tmax >= a - half, axis=1) & np.all(tmin <= b + half, axis=1))

    validator.FIX = SimpleNamespace(_tris=triangles, near=near, X=SimpleNamespace(HALF=surface.HALF))
    target = validator.triangle_first_hit(origins, directions, length, 1)
    hits["target"] = np.minimum(target, hits["background"])
    if args.simple:
        simple = reference.HullFile(args.simple)
        structure(simple)
        hits["simple"] = tree_first_hit(simple, origins, directions, length, 1)
    delta = hits["python"] - hits["previous"]
    record = {
        "output": str(destination), "output_sha256": reference.sha256(destination),
        "reference_clip": str(paths[0]), "reference_structure": raw_structure,
        "packaged_structure": checks, "background": str(Path(args.background).resolve()),
        "background_sha256": reference.sha256(args.background),
        "reference_hull2_aliases_hull1": True,
        "packaging": "Union original room collision with reference terrain; preserve render/VIS/light lumps",
        "comparison": {"file": str(old_path), "hull": 1, "rays": len(origins),
                       **surface.ray_stats(hits["python"], hits["previous"])},
        "grid_validation": {name: surface.ray_stats(hits[name], hits["target"])
                            for name in ("python", "previous", "simple") if name in hits},
    }
    finite = np.flatnonzero(np.isfinite(delta))
    if len(finite):
        index = finite[np.argmax(np.abs(delta[finite]))]
        record["comparison"]["largest_difference"] = {
            "xy": xy[index].tolist(), "python_z": float(origins[index, 2] - hits["python"][index]),
            "previous_z": float(origins[index, 2] - hits["previous"][index])}
    np.savez_compressed(root / "comparison_top_h1.npz", x=xs, y=ys,
                        **{name: (origins[:, 2] - hit).reshape(257, 257) for name, hit in hits.items()})
    reference.save_json(root / "package.json", record)
    print(json.dumps(record, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prep = commands.add_parser("prepare")
    for key in ("source", "geometry", "manifest", "scripts", "compiler", "wad", "output"):
        prep.add_argument("--" + key, required=True)
    prep.add_argument("--entity", type=int, required=True)
    prep.add_argument("--surface", choices=("terrain", "shell"), default="terrain")
    prep.add_argument("--compiler-threads", type=int, default=2)
    pack = commands.add_parser("package")
    for key in ("output", "background", "compare", "name"):
        pack.add_argument("--" + key, required=True)
    pack.add_argument("--simple", help="Ordinary simple BSP for the same dense-grid comparison")
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args)
    else:
        package(args)


if __name__ == "__main__":
    main()
