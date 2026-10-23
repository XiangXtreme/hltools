"""Compile one actual Boreas terrain triangle through simple and the reference scripts."""

import argparse
import json
from pathlib import Path
import shutil
import zipfile

import numpy as np
from scipy.spatial import ConvexHull

import brush_surface_pipeline as surface
import source_clip_pipeline as reference
import source_clip_validate as validator


def run(args):
    build, root, archive = args.build.resolve(), args.output.resolve(), args.archive.resolve()
    original_config = json.loads((build / "config.json").read_text())
    all_triangles = np.load(build / "terrain_tris.npy", mmap_mode="r")
    triangle = np.array(all_triangles[args.triangle])
    chunks = json.loads((build / "build_results.json").read_text())["chunks"]
    matches = [chunk for chunk in chunks if args.triangle in chunk["indices"]]
    if len(matches) != 1:
        raise ValueError("Triangle must belong to exactly one rebuilt chunk")
    chunk = matches[0]
    index = int(Path(chunk["file"]).stem.removeprefix("bclip"))
    archive_name = f"surf_boreas/models/boreasclips/bclipv2_{index:03d}.bsp"
    root.mkdir(parents=True, exist_ok=False)
    for directory in ("scripts", "compiler"):
        (root / directory).mkdir()
    for name in reference.SCRIPTS:
        path = build / "scripts" / name
        if reference.sha256(path) != original_config["script_sha256"][name]:
            raise ValueError(f"Original script snapshot changed: {name}")
        shutil.copy2(path, root / "scripts" / name)
    compiler = root / "compiler/hltools.exe"
    shutil.copy2(original_config["compiler_snapshot"], compiler)
    if reference.sha256(compiler) != original_config["compiler_sha256"]:
        raise ValueError("Original compiler snapshot changed")
    config = dict(original_config, output=str(root), compiler_snapshot=str(compiler),
                  input_kind="selected_boreas_triangle", triangle_index=args.triangle)
    reference.save_json(root / "config.json", config)
    np.save(root / "terrain_tris.npy", triangle[None])
    normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
    normal /= np.linalg.norm(normal)
    build_module, exact = reference.load_scripts(config)
    base = exact.base_template()
    name, nodes, count, note = build_module.compile_chunk("sample", triangle[None], base)
    if nodes is None or nodes > build_module.LIMIT:
        raise ValueError("Reference sample compile failed")
    (root / "clips_raw").mkdir()
    (root / "clips_out").mkdir()
    raw = root / "clips_raw/sample.bsp"
    repaired = root / "clips_out/sample.bsp"
    shutil.copy2(root / "exact_chunks/sample/sample.bsp", raw)
    validator.init_geometry(config)
    repair_note = validator.FIX.fix_file(str(raw), str(repaired))
    # The original triangle is exactly the exposed face of this convex prism.
    # Extra thickness lies behind that face; only outward first contacts are compared.
    prism = np.r_[triangle, triangle - normal * 256.]
    hull = ConvexHull(prism)
    planes = np.c_[hull.equations[:, :3], -hull.equations[:, 3]]
    _, indices = np.unique(np.round(planes, 9), axis=0, return_index=True)
    planes = planes[np.sort(indices)]
    brush = "{\n" + "\n".join(exact.plane_face(p[:3], p[3]) for p in planes) + "\n}"
    text = ('{\n"classname" "worldspawn"\n"mapversion" "220"\n'
            + f'"wad" "{config["wad"]}"\n' + brush + '\n}\n')
    for mode in ("simple", "legacy"):
        path = root / f"sample_{mode}.map"
        path.write_text(text, encoding="ascii")
        surface.require_command([compiler, "csg", path, "-cliptype", mode, "-threads", "2"],
                                root / f"{mode}.csg.txt")
        surface.require_command([compiler, "bsp", path.with_suffix(".bsp"), "-nofill", "-noinsidefill",
                                 "-nonulltex", "-threads", "2"], root / f"{mode}.bsp.txt")
    with zipfile.ZipFile(archive) as package:
        author = reference.HullFile(data=package.read(archive_name))
    files = {"simple": reference.HullFile(root / "sample_simple.bsp"),
             "legacy": reference.HullFile(root / "sample_legacy.bsp"),
             "python_sample": reference.HullFile(repaired),
             "python_full_chunk": reference.HullFile(build / "clips_out" / chunk["file"]),
             "author_full_chunk": author}
    rng = np.random.default_rng(62996)
    weights = .7 / 3 + .3 * rng.dirichlet(np.ones(3), size=1024)
    points = weights @ triangle
    origins = points + normal * 192.
    directions = np.tile(-normal, (len(points), 1))
    tests, probes = {}, {"triangle": triangle, "weights": weights, "origins": origins,
                         "directions": directions}
    for player_hull in (1, 3):
        target = validator.triangle_first_hit(origins, directions, 384., player_hull)
        hits = {name: validator.tree_first_hit(tree, origins, directions, 384., player_hull)
                for name, tree in files.items()}
        record = {"vs_triangle_sat": {name: surface.ray_stats(hit, target) for name, hit in hits.items()},
                  "simple_vs_python_sample": surface.ray_stats(hits["simple"], hits["python_sample"]),
                  "author_vs_full_rebuild": surface.ray_stats(hits["author_full_chunk"], hits["python_full_chunk"])}
        center_origin = triangle.mean(0)[None] + normal * 192
        center_direction = -normal[None]
        record["centroid_outward_contact_distance"] = {
            name: float(192 - validator.tree_first_hit(tree, center_origin, center_direction, 384., player_hull)[0])
            for name, tree in files.items()}
        record["centroid_outward_contact_distance"]["triangle_sat"] = float(
            192 - validator.triangle_first_hit(center_origin, center_direction, 384., player_hull)[0])
        half = exact.HALF[player_hull]
        record["infinite_plane_offsets"] = {"box": float(np.abs(normal) @ half),
                                             "legacy": float(normal**2 @ half)}
        tests[str(player_hull)] = record
        for name, hit in hits.items():
            probes[f"h{player_hull}_{name}"] = hit
        probes[f"h{player_hull}_sat"] = target
    for player_hull, record in tests.items():
        comparison = record["simple_vs_python_sample"]
        if comparison["hit_miss_differences"] or comparison["max_distance_error"] > .01:
            raise ValueError(f"Actual Boreas sample does not agree within .01: hull {player_hull}")
        for name in ("python_full_chunk", "author_full_chunk"):
            comparison = record["vs_triangle_sat"][name]
            if comparison["hit_miss_differences"] or comparison["max_distance_error"] > .01:
                raise ValueError(f"Parent chunk differs from the selected triangle: {name}, hull {player_hull}")
    report = {"build": str(build), "archive": str(archive), "archive_sha256": reference.sha256(archive),
              "triangle_index": args.triangle, "triangle": triangle.tolist(), "normal": normal.tolist(),
              "chunk": chunk["file"], "archive_chunk": archive_name, "rays_per_hull": len(points),
              "prism_depth": 256., "scope": "Outward first contact on the central part of one original surface triangle; not full solid occupancy equality",
              "repair": repair_note, "compiler_sha256": reference.sha256(compiler),
              "script_sha256": config["script_sha256"],
              "structure": {name: validator.structure(tree) for name, tree in files.items()}, "hulls": tests}
    reference.save_json(root / "comparison.json", report)
    np.savez_compressed(root / "probes.npz", **probes)
    lines = ["# Boreas Actual-Sample Comparison", "",
             f"Triangle index: {args.triangle} (zero-based in the saved snapped terrain array).",
             f"Archive parent: `{archive_name}`.", "",
             "Distances below are measured outward along the original triangle's unit normal",
             "from its centroid to the player-centre collision boundary, in map units.", "",
             "| Hull | Author parent BSP | Rebuilt parent BSP | Isolated Python sample | Simple brush | Legacy brush |",
             "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for player_hull, record in tests.items():
        contact = record["centroid_outward_contact_distance"]
        lines.append(f"| {player_hull} | " + " | ".join(f"{contact[name]:.6f}" for name in
                     ("author_full_chunk", "python_full_chunk", "python_sample", "simple", "legacy")) + " |")
    lines += ["", "The same original triangle coordinates are used throughout. The simple/legacy",
              "input is a convex prism with this triangle as its front face and 256 units",
              "of thickness behind it. The compared rays approach the central surface from",
              "outside; the test does not assert full-volume or clipnode-array equality.", "",
              "1024 central normal-directed rays per hull also compare the independent",
              "triangle/box SAT target. Full numerical errors and structures are recorded",
              "in comparison.json; exact probes are saved in probes.npz.", "",
              "The external collision BSPs have no playable room or lighting and are for",
              "collision inspection. The game installation is unchanged."]
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("triangle_index", "triangle", "chunk", "repair", "hulls")}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("build", "archive", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--triangle", type=int, default=62996, help="Zero-based index in the saved terrain array")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
