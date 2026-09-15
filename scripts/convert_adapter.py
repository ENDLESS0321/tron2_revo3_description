#!/usr/bin/env python3
"""Convert the user's Bambu 3MF adapter, preserving assembly and subtraction.

The input is never modified. Mesh coordinates are meters with the inferred
robot-side shoulder center as origin. CAD axes are retained; +Y points toward
the hand. The shoulder datum is an assembly assumption, not a measured fit.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT.parents[1] / "tron2-revo3转接件_第1版.3mf"
NS = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
PRODUCTION_PATH = "{http://schemas.microsoft.com/3dmanufacturing/production/2015/06}path"


def read_parts(source):
    with zipfile.ZipFile(source) as archive:
        root = ET.fromstring(archive.read("3D/3dmodel.model"))
        if root.attrib.get("unit") != "millimeter":
            raise ValueError("This converter requires an explicit millimeter 3MF")
        settings = ET.fromstring(archive.read("Metadata/model_settings.config"))
        metadata = {}
        for part in settings.findall("./object/part"):
            data = {x.attrib["key"]: x.attrib["value"] for x in part.findall("metadata")}
            metadata[part.attrib["id"]] = {"name": data["name"], "subtype": part.attrib["subtype"]}
        result = []
        snap_count = 0
        snap_max_mm = 0.
        for component in root.findall("./m:resources/m:object/m:components/m:component", NS):
            oid = component.attrib["objectid"]
            member = component.attrib[PRODUCTION_PATH].lstrip("/")
            objects = ET.fromstring(archive.read(member))
            obj = objects.find(f"./m:resources/m:object[@id='{oid}']", NS)
            vertices = np.array([[float(v.attrib[k]) for k in ("x", "y", "z")] for v in obj.findall("./m:mesh/m:vertices/m:vertex", NS)])
            faces = np.array([[int(t.attrib[k]) for k in ("v1", "v2", "v3")] for t in obj.findall("./m:mesh/m:triangles/m:triangle", NS)])
            xf = np.array(component.attrib["transform"].split(), dtype=float).reshape(4, 3)
            vertices = vertices @ xf[:3] + xf[3]
            # STEP tessellation puts the mating faces 5e-7 mm apart. Snap only
            # vertices within 1e-5 mm of this documented shared plane.
            near = np.abs(vertices[:, 1] + 1.1) < 1.e-5
            if near.any():
                snap_count += int(near.sum())
                snap_max_mm = max(snap_max_mm, float(np.abs(vertices[near, 1] + 1.1).max()))
                vertices[near, 1] = -1.1
            mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
            if not mesh.is_volume:
                raise ValueError(f"Part {oid} is not a closed, consistently wound positive volume")
            result.append((oid, metadata[oid], mesh))
        build_transform = root.find("./m:build/m:item", NS).attrib["transform"]
    return result, build_transform, {"shared_plane_y_mm": -1.1, "tolerance_mm": 1e-5, "vertices_snapped": snap_count, "maximum_vertex_movement_mm": snap_max_mm}


def convert(args):
    source = args.source.resolve()
    if args.print_scale <= 0 or args.density_kg_m3 <= 0:
        raise ValueError("Print scale and density must be positive")
    parts, discarded_build_transform, snap = read_parts(source)
    positive = [mesh for _, data, mesh in parts if data["subtype"] == "normal_part"]
    negative = [mesh for _, data, mesh in parts if data["subtype"] == "negative_part"]
    unknown = [data["subtype"] for _, data, _ in parts if data["subtype"] not in {"normal_part", "negative_part"}]
    if unknown or not positive:
        raise ValueError(f"Unsupported part subtypes or missing positive geometry: {unknown}")
    solid = trimesh.boolean.union(positive, engine="manifold", check_volume=True)
    pre_cut_volume = float(solid.volume)
    if negative:
        solid = trimesh.boolean.difference([solid, *negative], engine="manifold", check_volume=True)
    if not solid.is_volume:
        raise ValueError("Boolean result failed closed-volume validation")
    volume_removed_mm3 = pre_cut_volume - float(solid.volume)
    if negative and volume_removed_mm3 <= 0:
        raise ValueError("Negative part did not remove material")
    nominal_bounds = solid.bounds.copy()
    datum = np.array(args.datum_mm, dtype=float)
    solid.apply_translation(-datum)
    solid.apply_scale(args.print_scale * .001)
    solid.density = args.density_kg_m3
    properties = solid.mass_properties
    args.output.mkdir(parents=True, exist_ok=True)
    output_mesh = args.output / "adapter_visual.stl"
    solid.export(output_mesh)
    # STL has independent per-face vertices; process the round trip before
    # checking topology and compare volume to the in-memory boolean result.
    reread = trimesh.load_mesh(output_mesh, process=False)
    # Default trimesh merge tolerance is 1e-8 in the mesh's unit. With meters,
    # this merges distinct very small CAD facets; join identical STL vertices
    # at 1e-12 m instead to preserve topology without altering geometry.
    reread.merge_vertices(digits_vertex=12)
    if not reread.is_volume or not np.isclose(reread.volume, solid.volume, rtol=1e-5):
        raise ValueError("STL round-trip validation failed")
    report = {
        "source": str(source), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "output": str(output_mesh.relative_to(ROOT)) if output_mesh.is_relative_to(ROOT) else str(output_mesh),
        "output_sha256": hashlib.sha256(output_mesh.read_bytes()).hexdigest(),
        "source_unit": "millimeter", "output_unit": "meter",
        "source_parts": [{"id": oid, **data, "faces": len(mesh.faces), "volume_mm3": float(mesh.volume)} for oid, data, mesh in parts],
        "boolean_operation": "union(normal_part) minus every negative_part",
        "volume_removed_by_negative_parts_mm3": volume_removed_mm3,
        "numerical_shared_face_repair": snap,
        "ignored_printer_build_transform": discarded_build_transform,
        "print_scale": args.print_scale,
        "assembly_datum_nominal_mm": datum.tolist(),
        "mounting_reference_nominal_mm": datum.tolist(),
        "datum_status": "inferred robot-side shoulder center; physical fit and insertion depth require measurement",
        "axes": {"x": "original CAD X", "y": "original CAD Y, toward hand", "z": "original CAD Z"},
        "nominal_cad_bounds_mm": nominal_bounds.tolist(),
        "bounds_m": solid.bounds.tolist(), "dimensions_m": solid.extents.tolist(),
        "faces": len(solid.faces), "vertices": len(solid.vertices),
        "watertight": bool(solid.is_watertight), "winding_consistent": bool(solid.is_winding_consistent),
        "euler_number": int(solid.euler_number), "stl_roundtrip_is_volume": bool(reread.is_volume),
        "stl_roundtrip_vertex_merge_digits": 12,
        "density_kg_m3": args.density_kg_m3,
        "mass_model": "homogeneous solid at assumed PLA density; not measured, ignores infill, fasteners and print voids",
        "volume_m3": float(solid.volume), "mass_kg": float(properties.mass),
        "center_mass_m": properties.center_mass.tolist(),
        "inertia_at_com_kg_m2": properties.inertia.tolist(),
        "inertia_kg_m2": properties.inertia.tolist(),
        "inertia_eigenvalues_kg_m2": np.linalg.eigvalsh(properties.inertia).tolist(),
    }
    (args.output / "adapter_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=ROOT / "meshes/adapter")
    parser.add_argument("--print-scale", type=float, default=1.01, help="Bambu project scale; use 1.0 for nominal CAD geometry")
    parser.add_argument("--density-kg-m3", type=float, default=1240., help="Assumed solid PLA density, not a measured adapter mass")
    parser.add_argument("--datum-mm", type=float, nargs=3, default=[0., -12.6, 0.], help="Origin in assembled nominal CAD coordinates")
    convert(parser.parse_args())


if __name__ == "__main__":
    main()
