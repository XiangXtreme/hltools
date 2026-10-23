"""Stage and install rebuilt Python clips for the existing surf_brs_native map."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct

import numpy as np

from brush_surface_pipeline import parse_map
from source_clip_pipeline import HullFile, save_json, sha256
from source_clip_validate import structure


def replace_lumps(original, replacements):
    data = bytearray(original.data)
    for index, lump in replacements.items():
        offset, size = struct.unpack_from("<ii", data, 4 + index * 8)
        if size == len(lump):
            data[offset:offset + size] = lump
        else:
            data.extend(b"\0" * (-len(data) % 4))
            struct.pack_into("<ii", data, 4 + index * 8, len(data), len(lump))
            data.extend(lump)
    return bytes(data)


def adapt(native, rebuilt):
    if len(native.models) != 1 or len(rebuilt.models) != 1:
        raise ValueError("Expected one external collision model")
    if native.lumps[7] or rebuilt.lumps[7]:
        raise ValueError("Collision models must not contain render faces")
    if native.nodes[0].tolist() != [0, -1, -1]:
        raise ValueError("Expected the native model's reserved EMPTY node")
    if not np.array_equal(native.models[0]["origin"], rebuilt.models[0]["origin"]):
        raise ValueError("External model origins differ")
    # The template draw tree remains in the native container. Its plane indices
    # must retain their meanings when replacing the collision plane lump.
    for offset in range(0, len(native.lumps[5]), 24):
        plane = struct.unpack_from("<i", native.lumps[5], offset)[0]
        if plane >= len(rebuilt.planes) or not np.array_equal(native.planes[plane], rebuilt.planes[plane]):
            raise ValueError("Template draw plane changed")
    nodes = rebuilt.nodes.copy()
    nodes[:, 1:] = np.where(nodes[:, 1:] >= 0, nodes[:, 1:] + 1, nodes[:, 1:])
    clips = native.lumps[9][:8] + b"".join(struct.pack("<ihh", *node) for node in nodes)
    models = bytearray(native.lumps[14])
    # Retain native broad-phase coverage, extending it if the new chunk is larger.
    low = np.minimum(native.models[0]["mins"], rebuilt.models[0]["mins"])
    high = np.maximum(native.models[0]["maxs"], rebuilt.models[0]["maxs"])
    struct.pack_into("<6f", models, 0, *low, *high)
    heads = rebuilt.models[0]["head"][1:]
    struct.pack_into("<3i", models, 40, *(int(h) + 1 if h >= 0 else int(h) for h in heads))
    data = replace_lumps(native, {1: rebuilt.lumps[1], 9: clips, 14: bytes(models)})
    result = HullFile(data=data)
    checks = structure(result)
    if not np.array_equal(result.nodes[1:], nodes) or result.lumps[1] != rebuilt.lumps[1]:
        raise ValueError("Staged collision does not match the rebuilt file")
    for index in range(15):
        if index not in (1, 9, 14) and result.lumps[index] != native.lumps[index]:
            raise ValueError("Unrelated native lump changed")
    return data, checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("build", "game", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--install", action="store_true")
    args = parser.parse_args()
    build, game, root = (getattr(args, key).resolve() for key in ("build", "game", "output"))
    config = json.loads((build / "config.json").read_text())
    repairs = {r["file"]: r for r in json.loads((build / "repair_results.json").read_text())["files"]}
    chunks = json.loads((build / "build_results.json").read_text())["chunks"]
    if len(chunks) != 39 or set(repairs) != {f"bclip{i:03d}.bsp" for i in range(39)}:
        raise ValueError("Expected the complete 39-file repaired Source terrain build")
    for name, expected in config["script_sha256"].items():
        if sha256(build / "scripts" / name) != expected:
            raise ValueError(f"Reference script snapshot changed: {name}")
    map_path = game / "maps/surf_brs_native.bsp"
    main_hash = sha256(map_path)
    detail = game / "models/brs_native/brs_detail.bsp"
    detail_hash = sha256(detail)
    entities = parse_map(HullFile(map_path).lumps[0].rstrip(b"\0").decode("latin-1")).entities
    helpers = {e.keyvalues.get("model") for e in entities if e.classname == "env_sprite"}
    walls = {e.keyvalues.get("model") for e in entities if e.classname == "func_wall"}
    root.mkdir(parents=True, exist_ok=False)
    stage, backup = root / "staged", root / "backup"
    stage.mkdir()
    backup.mkdir()
    records = []
    for index in range(39):
        source = build / "clips_out" / f"bclip{index:03d}.bsp"
        name = f"bclipv2_{index:03d}.bsp"
        relative = f"models/brs_native/{name}"
        if relative not in helpers or relative not in walls:
            raise ValueError(f"Map does not precache and spawn {relative}")
        if sha256(source) != repairs[source.name]["repaired_sha256"]:
            raise ValueError(f"Repaired source changed: {source}")
        target = game / relative
        old_hash = sha256(target)
        native, rebuilt = HullFile(target), HullFile(source)
        data, checks = adapt(native, rebuilt)
        (stage / name).write_bytes(data)
        shutil.copy2(target, backup / name)
        if sha256(backup / name) != old_hash:
            raise ValueError(f"Backup verification failed: {target}")
        records.append({"file": name, "source": str(source), "target": str(target),
                        "source_sha256": sha256(source), "before_sha256": old_hash,
                        "staged_sha256": hashlib.sha256(data).hexdigest(),
                        "before_nodes": len(native.nodes), "rebuilt_nodes": len(rebuilt.nodes),
                        "installed_nodes": len(rebuilt.nodes) + 1,
                        "collision_data_changed": native.lumps[1] != rebuilt.lumps[1] or native.lumps[9] != HullFile(data=data).lumps[9],
                        "structure": checks})
    report = {"map": str(map_path), "map_sha256": main_hash, "build": str(build),
              "detail_sha256": detail_hash, "backup": str(backup), "installed": False,
              "files": records, "collision_files_changed": sum(r["collision_data_changed"] for r in records),
              "total_before_nodes": sum(r["before_nodes"] for r in records),
              "total_staged_nodes": sum(r["installed_nodes"] for r in records),
              "adapter": "Preserve native container, reserved EMPTY node, draw tree and broad-phase bounds; replace collision planes, nodes and roots"}
    save_json(root / "installation.json", report)
    if args.install:
        for record in records:
            if sha256(record["target"]) != record["before_sha256"]:
                raise ValueError(f"Installed file changed during staging: {record['target']}")
        try:
            for record in records:
                shutil.copyfile(stage / record["file"], record["target"])
                if sha256(record["target"]) != record["staged_sha256"]:
                    raise ValueError(f"Installation verification failed: {record['target']}")
            if sha256(map_path) != main_hash or sha256(detail) != detail_hash:
                raise ValueError("Main map or detail carrier changed during installation")
        except Exception:
            for record in records:
                shutil.copyfile(backup / record["file"], record["target"])
            raise
        report["installed"] = True
        save_json(root / "installation.json", report)
    print(json.dumps({key: value for key, value in report.items() if key != "files"}, indent=2))


if __name__ == "__main__":
    main()
