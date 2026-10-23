"""Stage playable Boreas start-area collision comparisons using installed visuals."""

import argparse
import json
from pathlib import Path
import shutil
import struct
import sys
import zipfile

import numpy as np
from scipy.spatial import ConvexHull

import brush_surface_pipeline as surface
import source_clip_pipeline as reference
from install_native_reference_clips import replace_lumps
from source_clip_validate import structure


LOW = np.array([10800., -13200., 12800.])
HIGH = np.array([14800., -8000., 15750.])
PREFIX = "models/brs_start_test/"


def overlaps(low, high):
    return bool(np.all(high >= LOW) and np.all(low <= HIGH))


def entities(bsp):
    return [e.keyvalues for e in surface.parse_map(
        bsp.lumps[0].rstrip(b"\0").decode("ascii")).entities]


def serialize(items):
    for item in items:
        if any(any(c in k + v for c in '\x00"\r\n') for k, v in item.items()):
            raise ValueError("Invalid entity string")
    return ("".join("{\n" + "".join(f'"{k}" "{v}"\n' for k, v in e.items())
                    + "}\n" for e in items) + "\0").encode("ascii")


def model_vertices(path, entity):
    """Read static default-pose vertices; zero header bounds are common here."""
    data = path.read_bytes()
    if struct.unpack_from("<4si", data) != (b"IDST", 10):
        raise ValueError(f"Not MDL v10: {path}")
    bones, bone_index = struct.unpack_from("<2i", data, 140)
    if bones != 1:
        raise ValueError(f"Expected a single static root: {path}")
    if struct.unpack_from("<i", data, bone_index + 32)[0] != -1:
        raise ValueError(f"Invalid root bone: {path}")
    values = np.array(struct.unpack_from("<6f", data, bone_index + 64))
    if np.any(values[3:] != 0):
        raise ValueError(f"Rotated default bone: {path}")
    count, offset = struct.unpack_from("<2i", data, 204)
    vertices = []
    body = int(entity.get("body", "0"))
    for index in range(count):
        models, base, model_index = struct.unpack_from("<3i", data, offset + index * 76 + 64)
        selected = (body // base) % models if base else 0
        submodel = model_index + selected * 112
        number, bone_info, vertex_index = struct.unpack_from("<3i", data, submodel + 80)
        if any(data[bone_info:bone_info + number]):
            raise ValueError(f"Non-root vertex: {path}")
        vertices.append(np.frombuffer(data, "<f4", number * 3, vertex_index).reshape(-1, 3))
    vertices = np.concatenate(vertices).astype(float) + values[:3]
    pitch, yaw, roll = np.radians(np.fromstring(entity.get("angles", "0 0 0"), sep=" "))
    if abs(np.sin(pitch)) > 1e-6 or abs(np.sin(roll)) > 1e-6:
        raise ValueError(f"Unsupported entity rotation: {path}")
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return vertices @ rotation.T + np.fromstring(entity.get("origin", "0 0 0"), sep=" ")


def pack_clips(template, trees):
    # Preserve the native empty draw hull and its reserved node zero.
    planes, clips, heads = template.lumps[1], [(0, -1, -1)], []
    for tree in trees:
        tree.compact()
        plane_offset, node_offset = len(planes) // 20, len(clips)
        heads.append(tree.root + node_offset if tree.root >= 0 else tree.root)
        planes += b"".join(struct.pack("<4fi", *p[:4], int(p[4])) for p in tree.planes)
        clips.extend((p + plane_offset, f + node_offset if f >= 0 else f,
                      b + node_offset if b >= 0 else b) for p, f, b in tree.nodes)
    if len(clips) > 32767:
        raise ValueError("External model exceeds GoldSrc clipnode limit")
    model = bytearray(template.lumps[14])
    # GoldSrc's network coordinate range requires broad-phase bounds to cross 4096.
    struct.pack_into("<6f", model, 0, *np.minimum(LOW - 64, -4000), *np.maximum(HIGH + 64, 4000))
    if len(heads) != 2:
        raise ValueError("Expected one standing and one crouching tree")
    struct.pack_into("<3i", model, 40, heads[0], heads[0], heads[1])
    data = replace_lumps(template, {1: planes, 9: b"".join(struct.pack("<ihh", *n) for n in clips),
                                     14: bytes(model)})
    checked = structure(reference.HullFile(data=data))
    return data, checked


def clip_trees(bsp):
    result = []
    for hull in (1, 3):
        tree = surface.Tree(bsp.planes, bsp.nodes, bsp.models[0]["head"][hull])
        tree.bound(LOW - 64, HIGH + 64)
        result.append(tree)
    return result


def run(args):
    game, build, root, archive = (getattr(args, key).resolve() for key in ("game", "build", "output", "archive"))
    root.mkdir(parents=True, exist_ok=False)
    stage = root / "cstrike"
    (stage / "maps").mkdir(parents=True)
    (stage / PREFIX).mkdir(parents=True)
    main_path = game / "maps/surf_brs_native.bsp"
    main_hash = reference.sha256(main_path)
    main = reference.HullFile(main_path)
    original = entities(main)
    selected, visuals = [], []
    for entity in original:
        model = entity.get("model", "")
        if entity.get("classname") != "env_sprite" or not model.endswith(".mdl"):
            continue
        header = (game / model).read_bytes()[:244]
        if struct.unpack_from("<i", header, 140)[0] != 1:
            point = np.fromstring(entity.get("origin", "0 0 0"), sep=" ")
            if overlaps(point, point):
                selected.append(dict(entity))
                visuals.append({"model": model, "origin": entity["origin"],
                                "selection": "animated effect origin inside region",
                                "sha256": reference.sha256(game / model)})
            continue
        points = model_vertices(game / model, entity)
        if overlaps(points.min(0), points.max(0)):
            selected.append(dict(entity))
            visuals.append({"model": model, "origin": entity["origin"],
                            "bounds": [points.min(0).tolist(), points.max(0).tolist()],
                            "sha256": reference.sha256(game / model)})
    reference.save_json(root / "visuals.json", visuals)
    print(f"Selected {len(selected)} original visual entities", flush=True)
    triangles = np.load(build / "terrain_tris.npy")
    indices = np.flatnonzero(np.all(triangles.max(1) >= LOW - 128, axis=1)
                             & np.all(triangles.min(1) <= HIGH + 128, axis=1))
    triangles = triangles[indices]
    np.save(root / "terrain_tris.npy", triangles)
    chunks = json.loads((build / "build_results.json").read_text())["chunks"]
    index_set = set(indices.tolist())
    chunks = [c for c in chunks if index_set.intersection(c["indices"])]
    print(f"Selected {len(triangles)} triangles and {len(chunks)} author chunks", flush=True)
    template = reference.HullFile(game / "models/brs_native/bclipv2_019.bsp")
    clips, checks = {"ref": []}, {}
    with zipfile.ZipFile(archive) as package:
        for chunk in chunks:
            number = int(Path(chunk["file"]).stem.removeprefix("bclip"))
            name = f"surf_boreas/models/boreasclips/bclipv2_{number:03d}.bsp"
            source = reference.HullFile(data=package.read(name))
            data, check = pack_clips(template, clip_trees(source))
            relative = PREFIX + f"ref_{number:03d}.bsp"
            (stage / relative).write_bytes(data)
            clips["ref"].append(relative)
            checks[relative] = check
            # Bounding and renumbering must preserve original occupancy in the test region.
            rng = np.random.default_rng(number)
            points = rng.uniform(LOW, HIGH, (20000, 3))
            output = reference.HullFile(data=data)
            for hull in (1, 3):
                if not np.array_equal(source.contents(points, hull), output.contents(points, hull)):
                    raise ValueError("Author collision changed within the test region")
    config = json.loads((build / "config.json").read_text())
    compiler = Path(config["compiler_snapshot"])
    if reference.sha256(compiler) != config["compiler_sha256"]:
        raise ValueError("Compiler snapshot hash changed")
    sys.path.insert(0, str(build / "scripts"))
    import exacthull
    brushes = []
    for triangle in triangles:
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        normal /= np.linalg.norm(normal)
        prism = ConvexHull(np.r_[triangle, triangle - normal * 256.])
        planes = np.c_[prism.equations[:, :3], -prism.equations[:, 3]]
        _, unique = np.unique(np.round(planes, 9), axis=0, return_index=True)
        brushes.append("{\n" + "\n".join(exacthull.plane_face(p[:3], p[3]) for p in planes[np.sort(unique)]) + "\n}")
    text = ('{\n"classname" "worldspawn"\n"mapversion" "220"\n'
            + f'"wad" "{config["wad"]}"\n' + "\n".join(brushes) + "\n}\n")
    timings = {}
    for mode in ("simple", "legacy"):
        path = root / f"terrain_{mode}.map"
        path.write_text(text, encoding="ascii")
        timings[mode] = {}
        for command in ([compiler, "csg", path, "-cliptype", mode, "-threads", "2"],
                        [compiler, "bsp", path.with_suffix(".bsp"), "-nofill", "-noinsidefill", "-nonulltex", "-threads", "2"]):
            timings[mode][command[1]] = surface.require_command(command, root / f"{mode}.{command[1]}.txt")
        data, check = pack_clips(template, clip_trees(reference.HullFile(path.with_suffix(".bsp"))))
        relative = PREFIX + mode + ".bsp"
        (stage / relative).write_bytes(data)
        clips[mode] = [relative]
        checks[relative] = check
        print(f"{mode}: {check['stored_nodes']} clipnodes", flush=True)
    # Shared platform collision, start marker, reset triggers and original lighting.
    shared = [dict(e) for e in original if e.get("model") in ("*1", "*31", "*33", "*45",
                                                                 "models/brs_native/brs_detail.bsp")
              or e.get("classname") in ("info_player_start", "info_teleport_destination",
                                         "light_environment", "env_fog")]
    start = next(e for e in shared if e["classname"] == "info_player_start")
    shared.append(dict(start, classname="info_player_deathmatch"))
    variants = {}
    for mode, paths in clips.items():
        helpers = [{"classname": "env_sprite", "targetname": f"__start_precache_{i}",
                    "model": path, "origin": "0 0 0", "spawnflags": "0", "framerate": "0", "scale": "1"}
                   for i, path in enumerate(paths)]
        walls = [{"classname": "func_wall", "model": path, "origin": "0 0 0"} for path in paths]
        items = [dict(original[0]), *helpers, *walls, *selected, *shared]
        data = replace_lumps(main, {0: serialize(items)})
        result = reference.HullFile(data=data)
        if entities(result) != items or any(result.lumps[i] != main.lumps[i] for i in range(1, 15)):
            raise ValueError("Scene container changed unexpectedly")
        name = f"brs_start_{mode}"
        destination = stage / "maps" / (name + ".bsp")
        destination.write_bytes(data)
        variants[mode] = {"map": name, "entities": len(items), "collision_models": paths,
                          "sha256": reference.sha256(destination),
                          "clipnodes": sum(checks[path]["stored_nodes"] for path in paths)}
        for entity in items:
            model = entity.get("model", "")
            if model and not model.startswith("*") and not (stage / model).exists() and not (game / model).is_file():
                raise ValueError(f"Missing model dependency: {model}")
    starts = np.array([np.fromstring(start["origin"], sep=" ")])
    for mode, paths in clips.items():
        for path in paths:
            bsp = reference.HullFile(stage / path)
            for hull in (1, 3):
                if bsp.contents(starts, hull)[0] != -1:
                    raise ValueError(f"Spawn inside terrain: {mode}")
    report = {"source_map": str(main_path), "source_map_sha256": main_hash,
              "archive": str(archive), "archive_sha256": reference.sha256(archive),
              "bounds": [LOW.tolist(), HIGH.tolist()], "triangle_indices": indices.tolist(),
              "author_chunks": [c["file"] for c in chunks], "visuals": visuals,
              "variants": variants, "checks": checks, "compile_seconds": timings,
              "spawn": start, "installed": False,
              "scope": "Original coordinates, models and light lumps. Entity subset; unused original BSP models remain. Author trees bounded 64 units outside region. Brush variants use original front triangles with 256 units of inward material; rear and edge volumes differ."}
    reference.save_json(root / "scene.json", report)
    readme = (
        "Boreas start-area collision comparison\n\n"
        "In the CS 1.6 console, load one of these maps:\n"
        "  map brs_start_ref     original Boreas Python collision BSPs\n"
        "  map brs_start_simple  same local terrain compiled with simple\n"
        "  map brs_start_legacy  same local terrain compiled with legacy\n\n"
        "The maps keep the original start location, nearby models, platform collision, "
        "world render/light lumps, and source coordinates. Only nearby model entities "
        "are kept; some invisible original BSP model data remains unused.\n\n"
        "Generated clipnode counts (including standing, crouch, and the reserved empty node):\n"
        + "".join(f"  {mode}: {item['clipnodes']}\n" for mode, item in variants.items())
        + "\nThe reference variant uses the two original author chunks selected by terrain "
        "triangle bounds and limits their entity bounds to this test area. The simple and "
        "legacy variants compile 222 original terrain triangles as inward 256-unit prisms; "
        "their rear and edge volumes differ from the reference.\n"
        "Compile time: " + json.dumps(timings, ensure_ascii=False) + "\n"
        "BSP structure and collision contents were checked offline. Runtime feel still needs in-game testing.\n"
    )
    (root / "README.txt").write_text(readme, encoding="utf-8")
    (stage / "maps/brs_start_readme.txt").write_text(readme, encoding="utf-8")
    if args.install:
        files = [p for p in stage.rglob("*") if p.is_file()]
        replaceable = {}
        if args.replace_owned_scene:
            previous = json.loads(args.replace_owned_scene.read_text(encoding="utf-8"))
            for item in previous["variants"].values():
                relative = f"maps/{item['map']}.bsp"
                replaceable[relative] = item["sha256"]
        for path in files:
            target = game / path.relative_to(stage)
            if target.exists() and reference.sha256(path) != reference.sha256(target):
                relative = path.relative_to(stage).as_posix()
                if relative == "maps/brs_start_readme.txt" and "Boreas start-area" in target.read_text(encoding="utf-8"):
                    continue
                if relative not in replaceable or reference.sha256(target) != replaceable[relative]:
                    raise ValueError(f"Refusing to overwrite a different existing asset: {target}")
        for path in files:
            target = game / path.relative_to(stage)
            if target.exists() and reference.sha256(path) == reference.sha256(target):
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
            if reference.sha256(path) != reference.sha256(target):
                raise ValueError("Installed file hash mismatch")
        if reference.sha256(main_path) != main_hash:
            raise ValueError("Original map changed")
        report["installed"] = True
        reference.save_json(root / "scene.json", report)
    print(json.dumps({"output": str(root), "variants": variants, "installed": report["installed"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("game", "build", "output", "archive"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--install", action="store_true")
    parser.add_argument("--replace-owned-scene", type=Path,
                        help="Replace only test maps whose current hashes match this earlier scene.json")
    run(parser.parse_args())
