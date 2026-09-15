#!/usr/bin/env python3
"""Generate side-specific adapter revisions without modifying the source 3MF.

Work in the original nominal CAD assembly frame (millimetres, +Y toward hand).
Only the insertion crown is re-indexed: the wrist, shoulder and hand transforms
remain fixed. Original nut pockets are transported with their radial bores.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import manifold3d as md
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("adapter_source", ROOT / "scripts/convert_adapter.py")
SOURCE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SOURCE)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def solid(mesh):
    # Match the source STL/CAD tessellation precision consistently in CSG.
    return md.Manifold(md.Mesh(np.asarray(mesh.vertices, dtype=np.float32), np.asarray(mesh.faces, dtype=np.uint32)))


def tri(manifold):
    data = manifold.to_mesh()
    mesh = trimesh.Trimesh(np.asarray(data.vert_properties)[:, :3], np.asarray(data.tri_verts), process=False)
    mesh.merge_vertices(digits_vertex=8)
    return mesh


def box(lo, hi):
    return md.Manifold.cube(np.array(hi)-lo).translate(lo)


def write_3mf(mesh, path, title):
    ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
    ET.register_namespace("", ns)
    tag = lambda name: "{" + ns + "}" + name
    model = ET.Element(tag("model"), unit="millimeter", attrib={"{http://www.w3.org/XML/1998/namespace}lang":"en-US"})
    ET.SubElement(model, tag("metadata"), name="Title").text = title
    ET.SubElement(model, tag("metadata"), name="Description").text = "Single baked solid in mm; 1.01 source print compensation already applied. Do not rescale by 1.01 again."
    resources = ET.SubElement(model, tag("resources"))
    obj = ET.SubElement(resources, tag("object"), id="1", type="model", name=title)
    geometry = ET.SubElement(obj, tag("mesh"))
    vertices = ET.SubElement(geometry, tag("vertices"))
    for point in mesh.vertices:
        ET.SubElement(vertices, tag("vertex"), **{k:format(float(v), ".10g") for k,v in zip("xyz", point)})
    faces = ET.SubElement(geometry, tag("triangles"))
    for face in mesh.faces:
        ET.SubElement(faces, tag("triangle"), **{k:str(int(v)) for k,v in zip(("v1","v2","v3"), face)})
    build = ET.SubElement(model, tag("build"))
    ET.SubElement(build, tag("item"), objectid="1")
    types = '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>'
    rels = '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rel0" Target="/3D/3dmodel.model" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>'
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        archive.writestr("[Content_Types].xml", types)
        archive.writestr("_rels/.rels", rels)
        archive.writestr("3D/3dmodel.model", ET.tostring(model, encoding="utf-8", xml_declaration=True))


def build_crown(original, print_scale, theta_degrees, output):
    # Use the actual float32 shoulder plane. Cutting at literal -12.6 would
    # also capture an infinitesimal sliver of the noncircular shoulder.
    plane = float(np.float32(-12.6))
    base, crown = original.split_by_plane([0,1,0], plane)
    if len(crown.decompose()) != 4:
        raise ValueError("Shoulder split must isolate exactly four insertion petals")
    crown_mesh = tri(crown)
    if np.max(np.linalg.norm(crown_mesh.vertices[:,[0,2]],axis=1)) > 28.0:
        raise ValueError("Insertion crown incorrectly includes shoulder material")
    # Original bore height is -17.1 nominal. Match actual wrist Y=-4.5 mm
    # after applying the original print scale. Rigidly raising the whole crown
    # preserves round bores/nut pockets; the 0.045 mm embedded portion unions
    # into the unchanged solid shoulder. Exposed insertion depth shortens by
    # precisely that amount, explicitly recorded below.
    nominal_target_y = -12.6 - 4.5 / print_scale
    hole_shift_y = nominal_target_y - (-17.1)
    edited_crown = crown.rotate([0,-theta_degrees,0]).translate([0,hole_shift_y,0])
    lower = base + edited_crown
    if len(lower.decompose()) != 1 or lower.genus() != original.genus():
        raise ValueError("Crown edit changed connectedness or screw-hole topology")
    report = {
        "edit": "Re-index only insertion crown below nominal CAD Y=-12.6; retain noncircular shoulder and hand mount",
        "theta_convention": "atan2(adapter Z, adapter X); increasing theta equals negative right-handed Y rotation",
        "crown_theta_rotation_deg": theta_degrees,
        "target_bore_angles_deg": [theta_degrees+i*90 for i in range(4)],
        "target_bore_axis_y_adapter_mm": -4.5,
        "nominal_bore_axis_y_mm": nominal_target_y,
        "nominal_bore_and_nut_translation_y_mm": hole_shift_y,
        "original_bore_nominal_diameter_mm": 4.2,
        "baked_bore_diameter_mm": 4.2 * print_scale,
        "original_nut_across_flats_nominal_mm": 6.9,
        "exposed_crown_depth_change_mm": -hole_shift_y*print_scale,
        "original_crown_components": len(crown.decompose()),
        "lower_topology_genus_before_after": [original.genus(), lower.genus()],
        "retained_shoulder_difference_mm3": (lower.trim_by_plane([0,1,0],plane) - original.trim_by_plane([0,1,0],plane)).volume(),
        "key_status": "requires independent comparison against wrist key at theta=30 degrees; not inferred from bore pattern",
    }
    tri(edited_crown).export(output / "trial_crown_nominal_mm.stl")
    return lower, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE.DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=ROOT/"design/revision2")
    parser.add_argument("--print-scale", type=float, default=1.01)
    parser.add_argument("--theta-degrees", type=float, default=15.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    parts, _, source_repair = SOURCE.read_parts(args.source)
    by_id = {oid:solid(mesh) for oid,_,mesh in parts}
    lower, wrist_report = build_crown(by_id['2'], args.print_scale, args.theta_degrees, args.output)
    # Restore the original 10 mm slot before cutting either 15 mm slot. The
    # patch boundaries match the original two cut faces, not guessed contours.
    old_slot = box(np.array([-5,-1.0300004,19.9999695],np.float32).astype(float),
                   np.array([5,5.17000036,27.9250183],np.float32).astype(float))
    upper_filled = by_id['1'] + old_slot
    result = {"schema":"tron2_revo3_adapter_revision2_v1", "source":str(args.source.resolve()),
              "source_sha256":sha(args.source), "producer_sha256":sha(Path(__file__)),
              "print_scale_baked":args.print_scale, "source_shared_plane_repair":source_repair,
              "csg_coordinate_precision":"float32, matching source mesh tessellation; exact shoulder and slot planes use the same representation",
              "wrist":wrist_report, "sides":{}, "status":"geometry_candidate_pending_independent_fit_validation"}
    for side, sign in [('left',1),('right',-1)]:
        cutter = by_id['3'] if sign==1 else by_id['3'].rotate([0,180,0])
        model = ((by_id['1'] if side=='left' else upper_filled)-cutter)+lower
        if model.status()!=md.Error.NoError or len(model.decompose())!=1:
            raise ValueError(f"{side} adapter is not one connected manifold")
        metric_mm = model.translate([0,12.6,0]).scale([args.print_scale]*3)
        mesh = tri(metric_mm)
        if not mesh.is_volume:
            raise ValueError(f"{side} triangle mesh failed closed-volume checks")
        assembly = args.output/f"adapter_{side}_v2_assembly_mm.stl"
        mesh.export(assembly)
        sim = mesh.copy();sim.apply_scale(.001)
        sim_path = args.output/f"adapter_{side}_v2_m.stl";sim.export(sim_path)
        printing = mesh.copy()
        printing.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
        printing.apply_translation(-printing.bounds[0])
        # Binary STL stores float32 coordinates. Quantize before validation,
        # joining coincident vertices and removing only facets that collapse
        # to zero area at that exact output precision. This changes no finite
        # surface patch and avoids delivering an invalid printable STL.
        printing.vertices = np.asarray(printing.vertices,dtype=np.float32).astype(float)
        printing.merge_vertices(digits_vertex=8)
        zero_area = printing.area_faces <= 1e-15
        collapsed_facets = int(np.sum(zero_area))
        printing.update_faces(~zero_area)
        printing.remove_unreferenced_vertices()
        if not printing.is_volume:
            raise ValueError("Float32 print geometry is not a closed positive volume")
        stl = args.output/f"adapter_{side}_v2_print_mm.stl";printing.export(stl)
        three = args.output/f"adapter_{side}_v2.3mf"
        write_3mf(printing, three, f"TRON2 Revo3 {side.upper()} adapter revision 2")
        roundtrip = trimesh.load_mesh(stl,process=False);roundtrip.merge_vertices(digits_vertex=8)
        if not roundtrip.is_volume or not np.isclose(roundtrip.volume,mesh.volume,rtol=1e-5):
            raise ValueError("STL roundtrip validation failed")
        sim.density=1240.;mp=sim.mass_properties
        result['sides'][side]={"back_slot_adapter_z_sign":sign,"slot_nominal_width_mm":15.,
            "slot_baked_width_mm":15*args.print_scale,"bounds_assembly_mm":mesh.bounds.tolist(),
            "volume_mm3":float(mesh.volume),"connected_components":len(model.decompose()),
            "watertight":bool(mesh.is_watertight),"winding_consistent":bool(mesh.is_winding_consistent),
            "print_stl_zero_area_facets_removed":collapsed_facets,
            "print_stl_volume_mm3":float(printing.volume),
            "mass_kg_assumed_solid_PLA":float(mp.mass),"center_mass_m":mp.center_mass.tolist(),"inertia_kg_m2":mp.inertia.tolist(),
            "files":{p.name:sha(p) for p in [assembly,sim_path,stl,three]}}
    (args.output/'design_manifest.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':
    main()
