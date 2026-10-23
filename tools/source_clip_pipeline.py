"""Run the supplied Source terrain collision scripts in an isolated workspace."""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import lzma
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time
import types
from concurrent.futures import ProcessPoolExecutor, as_completed

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

SCRIPTS = (
    "srcbsp.py", "exacthull.py", "brushbsp.py", "clipcheck.py",
    "build_boreas_clips.py", "fix_ambiguous.py", "memcap.py",
)
CONFIG = None


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def decompress_source(source, destination):
    data = bytearray(Path(source).read_bytes())
    if len(data) < 1036 or data[:4] != b"VBSP":
        raise ValueError("Expected a little-endian Source BSP")
    changed = []
    for index in range(64):
        offset, size, version, actual_size = struct.unpack_from("<4I", data, 8 + 16 * index)
        if offset + size > len(data):
            raise ValueError(f"Lump {index} extends outside the file")
        if not actual_size or not size:
            continue
        packed = bytes(data[offset:offset + size])
        magic, expected, compressed_size = struct.unpack_from("<4sII", packed)
        if magic != b"LZMA" or actual_size != expected or compressed_size != size - 17:
            raise ValueError(f"Invalid Source LZMA header in lump {index}")
        prop = packed[12]
        if prop >= 9 * 5 * 5:
            raise ValueError("Invalid LZMA properties")
        filters = [{"id": lzma.FILTER_LZMA1, "dict_size": struct.unpack_from("<I", packed, 13)[0],
                    "lc": prop % 9, "lp": (prop // 9) % 5, "pb": prop // 45}]
        decoder = lzma.LZMADecompressor(format=lzma.FORMAT_RAW, filters=filters)
        # Source streams may omit an end marker; the header supplies the output limit.
        unpacked = decoder.decompress(packed[17:], max_length=expected)
        if len(unpacked) != expected:
            raise ValueError(f"Incorrect decompressed size for lump {index}")
        # Append instead of relocating existing data: game lumps can contain absolute offsets.
        data.extend(b"\0" * (-len(data) % 4))
        new_offset = len(data)
        data.extend(unpacked)
        struct.pack_into("<4I", data, 8 + 16 * index, new_offset, expected, version, 0)
        changed.append({"lump": index, "packed_bytes": size, "unpacked_bytes": expected,
                        "sha256": hashlib.sha256(unpacked).hexdigest()})
    Path(destination).write_bytes(data)
    return changed


class HullFile:
    """Compatibility reader for the unavailable bsp_hull module used by clipcheck."""

    def __init__(self, path=None, data=None):
        import numpy as np
        self.data = Path(path).read_bytes() if data is None else data
        if struct.unpack_from("<i", self.data)[0] != 30:
            raise ValueError("Expected GoldSrc BSP v30")
        self.lumps = []
        for index in range(15):
            offset, size = struct.unpack_from("<ii", self.data, 4 + 8 * index)
            if offset < 0 or size < 0 or offset + size > len(self.data):
                raise ValueError("Invalid BSP lump range")
            self.lumps.append(self.data[offset:offset + size])
        self.planes = np.array(list(struct.iter_unpack("<4fi", self.lumps[1])), dtype=float)
        for plane in self.planes:
            if plane[4] < 3:
                axis = int(plane[4])
                plane[:3] = 0
                plane[axis] = 1
        self.nodes = np.array(list(struct.iter_unpack("<ihh", self.lumps[9])), dtype=np.int64)
        dtype = np.dtype([("mins", "<f4", 3), ("maxs", "<f4", 3), ("origin", "<f4", 3),
                          ("head", "<i4", 4), ("visleafs", "<i4"), ("ff", "<i4"), ("nf", "<i4")])
        self.models = np.frombuffer(self.lumps[14], dtype)

    def contents(self, points, hull=1):
        import numpy as np
        points = np.asarray(points)
        current = np.full(len(points), int(self.models[0]["head"][hull]), dtype=np.int64)
        for _ in range(len(self.nodes) + 1):
            active = np.flatnonzero(current >= 0)
            if not len(active):
                return current
            rows = self.nodes[current[active]]
            planes = self.planes[rows[:, 0]]
            back = np.sum(points[active] * planes[:, :3], axis=1) < planes[:, 3]
            current[active] = rows[np.arange(len(rows)), 1 + back.astype(int)]
        raise ValueError("Cycle in clipnode graph")


def load_scripts(config):
    global CONFIG
    CONFIG = config
    os.chdir(config["output"])
    sys.path.insert(0, str(Path(config["output"]) / "scripts"))
    import exacthull as exact
    compat = types.ModuleType("bsp_hull")
    compat.BSP = HullFile
    compat.HULL_HALF = exact.HALF
    compat.CONTENTS_SOLID = -2
    sys.modules["bsp_hull"] = compat
    import build_boreas_clips as build
    exact.HLTOOLS = config["compiler_snapshot"]
    exact.WAD = config["wad"]
    exact.subprocess = types.SimpleNamespace(run=logged_compile)
    build.BSP = str(Path(config["output"]) / "surf_boreas_unc.bsp")
    build.VMF = str(Path(config["output"]) / "source.vmf")
    build.CSTRIKE = str(Path(config["output"]) / "isolated_install") + "/"
    return build, exact


def logged_compile(args, **kwargs):
    command = [str(arg) for arg in args] + ["-threads", str(CONFIG["compiler_threads"])]
    path = Path(args[2])
    stage = args[1]
    records = path.with_suffix(f".{stage}.commands.jsonl")
    attempt = len(records.read_text().splitlines()) if records.exists() else 0
    if stage == "bsp":
        csg_records = path.with_suffix(".csg.commands.jsonl")
        if csg_records.exists() and json.loads(csg_records.read_text().splitlines()[-1])["returncode"]:
            raise RuntimeError(f"CSG never succeeded: {path}")
    started = time.perf_counter()
    result = subprocess.run(command, **kwargs)
    output = result.stdout or ""
    errors = result.stderr or ""
    if isinstance(output, bytes):
        output = output.decode("utf-8", errors="replace")
    if isinstance(errors, bytes):
        errors = errors.decode("utf-8", errors="replace")
    path.with_suffix(f".{stage}.{attempt:03d}.txt").write_text(output + errors, encoding="utf-8")
    with records.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"command": command, "seconds": time.perf_counter() - started,
                                 "returncode": result.returncode}) + "\n")
    retriable = stage == "csg" and re.search(r"entity 0, brush (\d+), side (\d+)", output + errors)
    if result.returncode and not retriable:
        raise RuntimeError(f"Compiler failed: {path}, stage {stage}, code {result.returncode}")
    return result


def prepare(args):
    root = Path(args.output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root / "scripts").mkdir()
    (root / "compiler").mkdir()
    hashes = {}
    for name in SCRIPTS:
        source = Path(args.scripts) / name
        shutil.copy2(source, root / "scripts" / name)
        hashes[name] = sha256(source)
    compiler = root / "compiler" / "hltools.exe"
    shutil.copy2(args.compiler, compiler)
    shutil.copy2(args.vmf, root / "source.vmf")
    config = {key: str(Path(getattr(args, key)).resolve())
              for key in ("source", "vmf", "scripts", "compiler", "wad", "output")}
    config.update(compiler_snapshot=str(compiler), compiler_threads=args.compiler_threads,
                  source_sha256=sha256(args.source), vmf_sha256=sha256(args.vmf),
                  compiler_sha256=sha256(args.compiler), script_sha256=hashes,
                  adapters=["Top-level Source LZMA decompression, preserving old offsets",
                            "Paths redirected into isolated output; no game installation",
                            "Missing bsp_hull import supplied by compatibility reader",
                            "Compiler thread count, logs, and return-code checks"])
    config["decompressed_lumps"] = decompress_source(args.source, root / "surf_boreas_unc.bsp")
    build, _ = load_scripts(config)
    from srcbsp import SourceBSP
    import numpy as np
    bsp = SourceBSP(build.BSP)
    displacements = bsp.displacements()
    vmf_info = []
    for match in re.finditer(r"dispinfo\s*\{([^{}]*)", Path(build.VMF).read_text(encoding="latin-1")):
        keys = dict(re.findall(r'"(\w+)"\s+"([^"]*)"', match.group(1)))
        vmf_info.append((tuple(np.round(np.array(keys["startposition"].strip("[]").split(), float), 1)),
                         int(keys["power"]), int(keys.get("flags", 0))))
    dtype = np.dtype([("start", "<f4", 3), ("v", "<i4"), ("t", "<i4"),
                      ("power", "<i4"), ("rest", "u1", 152)])
    info = np.frombuffer(bsp.lump(26), dtype)
    bsp_keys = Counter((tuple(np.round(row["start"].astype(float), 1)), int(row["power"])) for row in info)
    vmf_keys = Counter((start, power) for start, power, _ in vmf_info)
    if bsp_keys != vmf_keys:
        raise ValueError("VMF/BSP displacement keys or duplicate counts differ")
    flag_sets = defaultdict(set)
    for start, power, flags in vmf_info:
        flag_sets[(start, power)].add(bool(flags & 4))
    if any(len(values) > 1 for values in flag_sets.values()):
        raise ValueError("Duplicate displacement keys have conflicting collision flags")
    tris = build.triangles(report=True)
    np.save(root / "terrain_tris.npy", tris)
    selected = []
    maximum_snap = 0.0
    no_hull = build.no_hull_starts()
    for disp in displacements:
        row = info[disp.index]
        excluded = any(word in disp.material.lower() for word in build.NOT_SOLID)
        excluded |= (tuple(np.round(row["start"].astype(float), 1)), int(row["power"])) in no_hull
        if not excluded:
            selected.append(disp.index)
            maximum_snap = max(maximum_snap, float(np.linalg.norm(np.round(disp.positions / build.GRID) * build.GRID - disp.positions, axis=1).max()))
    manifest = {"bsp_version": bsp.version, "bsp_displacements": len(displacements),
                "vmf_displacements": len(vmf_info), "matched_displacement_keys": len(bsp_keys),
                "displacement_key_multisets_identical": True,
                "duplicate_keys_with_conflicting_no_hull_flags": 0,
                "vmf_no_hull": sum(bool(flags & 4) for _, _, flags in vmf_info),
                "selected_displacements": selected, "triangles": len(tris),
                "grid": build.GRID, "max_selected_vertex_snap": maximum_snap,
                "triangle_bounds": [tris.min((0, 1)).tolist(), tris.max((0, 1)).tolist()],
                "initial_chunks": math.ceil(len(tris) / build.PER_CHUNK)}
    save_json(root / "input.json", manifest)
    save_json(root / "config.json", config)
    print(json.dumps({key: value for key, value in manifest.items() if key != "selected_displacements"}, indent=2), flush=True)


def compile_worker(name, indices, base):
    import numpy as np
    build, _ = load_scripts(CONFIG)
    started = time.perf_counter()
    tris = np.load("terrain_tris.npy", mmap_mode="r")[indices]
    result = build.compile_chunk(name, tris, base)
    return {"name": name, "nodes": result[1], "triangles": result[2], "note": result[3],
            "seconds": time.perf_counter() - started, "indices": indices}


def init_worker(config):
    load_scripts(config)
    from memcap import limit_memory
    limit_memory(3000)


def run_build(args):
    import numpy as np
    root = Path(args.output).resolve()
    config = json.loads((root / "config.json").read_text())
    build, exact = load_scripts(config)
    tris = np.load("terrain_tris.npy", mmap_mode="r")
    centers = tris.mean(1)
    groups = build.split(centers, range(len(tris)), math.ceil(len(tris) / build.PER_CHUNK))
    todo = [(f"c{index:04d}", group) for index, group in enumerate(groups)]
    template = Path("exact_base/base.bsp")
    completed = template.with_suffix(".bsp.commands.jsonl")
    template_succeeded = completed.exists() and json.loads(completed.read_text().splitlines()[-1])["returncode"] == 0
    if template.exists() and not template_succeeded:
        template.unlink()
    base = exact.base_template()
    done = []
    started = time.perf_counter()
    with ProcessPoolExecutor(args.workers, initializer=init_worker, initargs=(config,)) as pool:
        while todo:
            futures = {pool.submit(compile_worker, name, group, base): (name, group) for name, group in todo}
            todo = []
            for future in as_completed(futures):
                record = future.result()
                name, group = futures[future]
                if record["nodes"] is None:
                    raise RuntimeError(f"Compilation produced no BSP: {name}")
                if record["nodes"] > build.LIMIT:
                    todo.extend((name + suffix, sub) for suffix, sub in zip("ab", build.split(centers, group, 2)))
                    print(f"{name}: {record['nodes']} nodes -> split", flush=True)
                else:
                    done.append(record)
                    save_json("build_progress.json", done)
                    print(f"{len(done)} done: {name}, {record['triangles']} triangles, {record['nodes']} nodes, {record['seconds']:.1f}s {record['note']}", flush=True)
    done.sort(key=lambda record: record["name"])
    Path("clips_raw").mkdir(exist_ok=True)
    Path("clips_out").mkdir(exist_ok=True)
    for index, record in enumerate(done):
        filename = f"bclip{index:03d}.bsp"
        source = Path("exact_chunks") / record["name"] / (record["name"] + ".bsp")
        shutil.copy2(source, Path("clips_raw") / filename)
        if not (Path("clips_out") / filename).exists():
            shutil.copy2(source, Path("clips_out") / filename)
        record["file"] = filename
        record["sha256"] = sha256(source)
    result = {"seconds": time.perf_counter() - started, "workers": args.workers,
              "total_nodes": sum(record["nodes"] for record in done),
              "total_triangles": sum(record["triangles"] for record in done), "chunks": done}
    save_json("build_results.json", result)
    print(f"Finished: {len(done)} files, {result['total_nodes']} nodes, {result['seconds']:.1f}s", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    prep = commands.add_parser("prepare")
    for name in ("source", "vmf", "scripts", "compiler", "wad", "output"):
        prep.add_argument("--" + name, required=True)
    prep.add_argument("--compiler-threads", type=int, default=2)
    build = commands.add_parser("build")
    build.add_argument("--output", required=True)
    build.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args)
    else:
        run_build(args)


if __name__ == "__main__":
    main()
