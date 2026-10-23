"""Compare actual legacy, simple and reference-Python contact on cliptype_sl."""

import argparse
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np

import brush_python_clips as adapter
import brush_surface_pipeline as surface
import source_clip_pipeline as reference
import source_clip_validate as validator


def run(args):
    root, source, compiler, dumper, scripts, wad = [getattr(args, key).resolve() for key in
        ("output", "source", "compiler", "dumper", "scripts", "wad")]
    text = source.read_text(encoding="utf-8")
    document = surface.parse_map(text)
    if len(document.entities) != 3 or len(document.entities[0].brushes) != 7:
        raise ValueError("Expected cliptype_sl with six room brushes and one slope")
    root.mkdir(parents=True, exist_ok=False)
    shutil.copy2(source, root / "source_snapshot.map")
    manifest = {"source": str(source), "source_sha256": reference.sha256(source),
                "compiler_sha256": reference.sha256(compiler), "dumper_sha256": reference.sha256(dumper)}
    reference.save_json(root / "manifest.json", manifest)
    for mode in ("legacy", "simple"):
        path = root / f"cliptype_sl_{mode}.map"
        path.write_text(text, encoding="utf-8")
        surface.require_command([compiler, "csg", path, "-cliptype", mode, "-threads", "2"],
                                root / f"{mode}.csg.txt")
        for stage in ("bsp", "vis", "rad"):
            surface.require_command([compiler, stage, path.with_suffix(".bsp"), "-threads", "2"],
                                    root / f"{mode}.{stage}.txt")
    # Move the same slope into a compile-only detail entity so the existing
    # background exporter retains its render faces while excluding its hulls.
    brush = document.entities[0].brushes[6]
    block = "".join(document.lines[brush.start:brush.end + 1])
    grouped = "".join(document.lines[:brush.start] + document.lines[brush.end + 1:])
    grouped += '\n{\n"classname" "func_detail"\n' + block + '}\n'
    grouped_path = root / "grouped.map"
    grouped_path.write_text(grouped, encoding="utf-8")
    surface.require_command([dumper, grouped_path, root / "geometry.json", "--background", "3",
                             root / "background"], root / "geometry.txt")
    for stage in ("bsp", "vis", "rad"):
        surface.require_command([compiler, stage, root / "background.bsp", "-threads", "2"],
                                root / f"background.{stage}.txt")
    manifest.update(grouped_map_sha256=reference.sha256(grouped_path),
                    geometry_sha256=reference.sha256(root / "geometry.json"),
                    geometry_adapter="Original world brush 6 moved unchanged to func_detail entity 3")
    reference.save_json(root / "manifest.json", manifest)
    python_root = root / "python"
    adapter.prepare(SimpleNamespace(source=source, geometry=root / "geometry.json",
                    manifest=root / "manifest.json", output=python_root, scripts=scripts,
                    compiler=compiler, wad=wad, entity=3, surface="terrain", compiler_threads=2))
    reference.run_build(SimpleNamespace(output=str(python_root), workers=1))
    config = json.loads((python_root / "config.json").read_text())
    validator.repair(SimpleNamespace(workers=1), config)
    external = reference.HullFile(python_root / "clips_out/bclip000.bsp")
    trees = {h: surface.Tree(external.planes, external.nodes, external.models[0]["head"][h])
             for h in (1, 2, 3)}
    encoded, structure = surface.splice(root / "background.bsp", trees)
    packaged = root / "cliptype_sl_python.bsp"
    packaged.write_bytes(encoded)
    files = {mode: reference.HullFile(root / f"cliptype_sl_{mode}.bsp") for mode in ("legacy", "simple")}
    files.update(python_external=external, python_world=reference.HullFile(packaged))
    triangles = np.load(python_root / "terrain_tris.npy")
    validator.init_geometry(config)
    result = {"manifest": manifest, "source_slope_equation": "7*y + 6*z = 144",
              "triangle_count": len(triangles), "structure": {name: validator.structure(tree) for name, tree in files.items()},
              "contacts": []}
    normal = np.array([0., 7., 6.]) / np.sqrt(85)
    for hull, heights in ((1, (4.0625, 20., 40., 60.)), (3, (-13.9375, 4., 20., 40.))):
        for z in heights:
            origin, direction = np.array([[-48., 140., z]]), np.array([[0., -1., 0.]])
            actual = {name: float(140 - validator.tree_first_hit(tree, origin, direction, 270., hull)[0])
                      for name, tree in files.items()}
            actual["triangle_sat"] = float(140 - validator.triangle_first_hit(origin, direction, 270., hull)[0])
            half = surface.HALF[hull]
            shifts = {"legacy": float((normal * normal) @ half), "box": float(np.abs(normal) @ half)}
            predicted = {name: float((144 + shift * np.sqrt(85) - 6*z) / 7) for name, shift in shifts.items()}
            result["contacts"].append({"hull": hull, "x": -48., "z": z,
                                       "direction": "negative Y", "contact_y": actual, "infinite_plane_prediction_y": predicted})
            if abs(actual["python_external"] - actual["python_world"]) > 1e-6:
                raise ValueError("World packaging changed slope contact")
            for name in ("simple", "python_external", "python_world"):
                if abs(actual[name] - actual["triangle_sat"]) > .001:
                    raise ValueError(f"Box-target slope contact changed: {name}")
            if abs(actual["legacy"] - predicted["legacy"]) > .001:
                raise ValueError("Legacy slope contact no longer follows its plane offset")
    result["checks"] = {"contact_cases": len(result["contacts"]), "contact_tolerance": .001,
                        "python_external_equals_world": True, "python_matches_simple_and_sat": True,
                        "legacy_matches_legacy_plane_offset": True}
    if reference.sha256(source) != manifest["source_sha256"]:
        raise ValueError("Source MAP changed")
    reference.save_json(root / "contacts.json", result)
    lines = ["# cliptype_sl Contact Measurements", "",
             "Same input slope; actual BSP interval traces, without engine trace epsilon.",
             "Slope plane: `7*y + 6*z = 144`. Trace from X=-48, Y=140 toward negative Y.",
             "Larger contact Y means the player is held farther away from the slope.", "",
             "| Hull | Player centre Z | Legacy Y | Simple Y | Python external Y | Python world Y | Triangle SAT Y |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for row in result["contacts"]:
        c = row["contact_y"]
        lines.append(f"| {row['hull']} | {row['z']:.4f} | " + " | ".join(f"{c[key]:.6f}" for key in
                      ("legacy", "simple", "python_external", "python_world", "triangle_sat")) + " |")
    lines += ["", "Both conventional builds and the Python world wrapper include VIS and RAD.",
              "The Python external file remains a collision model, not a standalone map.",
              "The slope is moved unchanged to a detail entity for background export only.",
              "Original MAP and game installation are unchanged. No in-game test was performed."]
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(result["contacts"], indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "compiler", "dumper", "scripts", "wad", "output"):
        parser.add_argument("--" + key, required=True, type=Path)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
