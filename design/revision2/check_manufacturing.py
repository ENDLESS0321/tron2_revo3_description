#!/usr/bin/env python3
"""Independently check revision-2 manufacturing artifacts without editing them.

Run with the project's .venv/bin/python. Writes only manufacturing_validation.json.
Checks geometry/units/handedness; does not certify print strength or hardware fit.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile

import manifold3d as md
import numpy as np
from scipy.spatial import cKDTree
import trimesh


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = Path(__file__).resolve().parent
CORE = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
NS = {"m": CORE}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def as_solid(mesh):
    result = md.Manifold(md.Mesh64(np.asarray(mesh.vertices, dtype=np.float64), np.asarray(mesh.faces, dtype=np.uint64)))
    if result.status() != md.Error.NoError:
        raise ValueError(f"Cannot construct manifold: {result.status()}")
    return result


def cube(lo, hi):
    return md.Manifold.cube(np.asarray(hi)-lo).translate(lo)


def load_stl(path, digits):
    mesh = trimesh.load_mesh(path, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{path.name}: expected one triangle mesh")
    mesh.merge_vertices(digits_vertex=digits)
    return mesh


def native_3mf(path):
    """Read core XML directly: no slicer transforms or loader auto-scaling."""
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)):
            raise ValueError("3MF contains duplicate ZIP members")
        models = [name for name in names if name.endswith(".model")]
        if models != ["3D/3dmodel.model"]:
            raise ValueError(f"Expected one native model member, got {models}")
        for required in ("[Content_Types].xml", "_rels/.rels"):
            ET.fromstring(archive.read(required))
        model = ET.fromstring(archive.read(models[0]))
        if model.tag != f"{{{CORE}}}model" or model.get("unit") != "millimeter":
            raise ValueError("3MF must declare the core model namespace and millimeter unit")
        objects = model.findall("./m:resources/m:object", NS)
        builds = model.findall("./m:build", NS)
        if len(objects) != 1 or len(builds) != 1:
            raise ValueError("3MF must contain one object and one build element")
        obj = objects[0]
        if obj.get("type") != "model" or obj.find("m:components", NS) is not None:
            raise ValueError("3MF must contain one baked model object, not component instances")
        items = builds[0].findall("m:item", NS)
        if len(items) != 1 or items[0].get("objectid") != obj.get("id"):
            raise ValueError("3MF must contain one build item referencing its model object")
        raw_transform = items[0].get("transform")
        if raw_transform is not None:
            transform = np.fromstring(raw_transform, sep=" ")
            identity = np.r_[np.eye(3).reshape(-1), [0., 0., 0.]]
            if transform.shape != (12,) or not np.allclose(transform, identity, rtol=0, atol=1e-12):
                raise ValueError("3MF build item must be identity; scaling/position is already baked")
        mesh_nodes = obj.findall("m:mesh", NS)
        if len(mesh_nodes) != 1:
            raise ValueError("3MF must have exactly one mesh")
        vertices = np.array([[float(v.get(k)) for k in "xyz"] for v in mesh_nodes[0].findall("./m:vertices/m:vertex", NS)])
        faces = np.array([[int(t.get(k)) for k in ("v1", "v2", "v3")] for t in mesh_nodes[0].findall("./m:triangles/m:triangle", NS)], dtype=np.int64)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
            raise ValueError("3MF vertex array is empty, malformed, or non-finite")
        if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0 or faces.min() < 0 or faces.max() >= len(vertices):
            raise ValueError("3MF has invalid triangle indices")
        mesh = trimesh.Trimesh(vertices, faces, process=False)
        mesh.merge_vertices(digits_vertex=8)
        return mesh, {"model_members": models, "unit": "millimeter", "model_objects": len(objects), "build_elements": len(builds), "build_items": len(items), "identity_build_transform": True, "vertices": len(vertices), "triangles": len(faces)}


def distance_between_point_sets(a, b):
    return float(max(cKDTree(a).query(b)[0].max(), cKDTree(b).query(a)[0].max()))


def compare_meshes(a, b, tolerance_mm):
    """Compare geometry, permitting removal of collapsed zero-area facets.

    A nearest triangle centroid is already a point on the opposing surface.
    Where centroids do not correspond (e.g. a float32-collapsed facet was
    removed), check the actual nearest surface point instead of requiring
    identical tessellation or triangle counts.
    """
    distances_ab = cKDTree(b.triangles_center).query(a.triangles_center)[0]
    distances_ba = cKDTree(a.triangles_center).query(b.triangles_center)[0]
    centroid_distance = float(max(distances_ab.max(), distances_ba.max()))
    fallback_count = 0
    for distances, source, target in ((distances_ab,a,b),(distances_ba,b,a)):
        unmatched = distances > tolerance_mm
        fallback_count += int(unmatched.sum())
        if unmatched.any():
            _, exact, _ = trimesh.proximity.closest_point(target, source.triangles_center[unmatched])
            distances[unmatched] = exact
    result = {
        "vertex_hausdorff_mm": distance_between_point_sets(a.vertices, b.vertices),
        "triangle_centroid_nearest_centroid_distance_mm": centroid_distance,
        "triangle_centroid_surface_distance_bound_mm": float(max(distances_ab.max(),distances_ba.max())),
        "centroids_checked_against_actual_surface": fallback_count,
        "triangle_counts": [len(a.faces), len(b.faces)],
        "bounds_max_error_mm": float(np.abs(a.bounds-b.bounds).max()),
        "volume_relative_error": float(abs(a.volume-b.volume)/abs(a.volume)),
        "coordinate_tolerance_mm": tolerance_mm,
    }
    result["valid"] = (result["vertex_hausdorff_mm"] <= tolerance_mm and result["triangle_centroid_surface_distance_bound_mm"] <= tolerance_mm and result["bounds_max_error_mm"] <= tolerance_mm and result["volume_relative_error"] <= 1e-5)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=OUTPUT/"design_manifest.json")
    parser.add_argument("--output", type=Path, default=OUTPUT/"manufacturing_validation.json")
    parser.add_argument("--coordinate-tolerance-mm", type=float, default=5e-5)
    parser.add_argument("--max-upper-difference-mm3", type=float, default=0.1)
    args = parser.parse_args()
    errors = []
    report = {"schema":"tron2_revo3_manufacturing_validation_v1", "created_utc":datetime.now(timezone.utc).isoformat(), "scope":"Native manufacturing file units, topology, scaling, print orientation, cable-slot handedness, and upper-geometry preservation. Not a mechanical-fit or print-strength certificate.", "errors":errors, "sides":{}}

    def require(condition, message):
        if not condition:
            errors.append(message)

    try:
        manifest_bytes = args.manifest.read_bytes()
        manifest_hash = hashlib.sha256(manifest_bytes).hexdigest()
        manifest = json.loads(manifest_bytes)
        report["manifest_sha256"] = manifest_hash
        report["producer_sha256"] = manifest.get("producer_sha256")
        require(manifest.get("schema") == "tron2_revo3_adapter_revision2_v1", "Unexpected design-manifest schema")
        require(sha(ROOT/"scripts/redesign_adapters.py") == manifest.get("producer_sha256"), "Producer script changed since manifest generation")
        source = Path(manifest["source"])
        require(sha(source) == manifest["source_sha256"], "Source 3MF hash differs from manifest")
        initial_artifacts = {}
        for side in ("left", "right"):
            expected_names = {f"adapter_{side}_v2{suffix}" for suffix in ("_assembly_mm.stl", "_print_mm.stl", "_m.stl", ".3mf")}
            records = manifest["sides"][side]["files"]
            require(set(records) == expected_names, f"{side}: manifest manufacturing file set differs from required outputs")
            for name, expected_hash in records.items():
                path = args.manifest.parent/name
                if Path(name).name != name:
                    raise ValueError(f"Artifact must use a basename: {name}")
                actual = sha(path)
                require(actual == expected_hash, f"Artifact hash mismatch: {name}")
                initial_artifacts[name] = actual
        if errors:
            raise ValueError("Input hashes/schema not stable; refusing geometric checks on unverified artifacts")

        spec = importlib.util.spec_from_file_location("adapter_source_reader", ROOT/"scripts/convert_adapter.py")
        reader = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(reader)
        source_parts, build_transform, source_repair = reader.read_parts(source)
        source_meshes = {oid: mesh for oid, _, mesh in source_parts}
        build_matrix = np.fromstring(build_transform, sep=" ").reshape(4,3)
        source_print_scale = np.linalg.svd(build_matrix[:3], compute_uv=False)
        scale = float(manifest["print_scale_baked"])
        require(np.allclose(source_print_scale, scale, rtol=0, atol=1e-8), "Baked scale must equal the original single source-project print scale")
        config_path = ROOT/"config/assembly.json"
        config_bytes = config_path.read_bytes()
        config = json.loads(config_bytes)
        require(abs(float(config["adapter_print_scale"])-scale)<1e-12, "Assembly config and manufacturing scale disagree")
        datum = np.array(config["adapter_datum_nominal_mm"], dtype=float)
        upper_reference = as_solid(source_meshes["1"]) - as_solid(source_meshes["3"])
        upper_region = cube([-50.,-1.099,-40.], [50.,30.,40.])
        windows = cube([-7.502,-1.032,19.998], [7.502,5.172,27.927]) + cube([-7.502,-1.032,-27.927], [7.502,5.172,-19.998])
        reference_outside = (upper_reference ^ upper_region) - windows
        report["source_project_print_scale_singular_values"] = source_print_scale.tolist()
        report["print_scale_baked"] = scale
        report["assembly_config_sha256"] = hashlib.sha256(config_bytes).hexdigest()
        report["slot_exclusion_windows_nominal_mm"] = [[[-7.502,-1.032,19.998],[7.502,5.172,27.927]],[[-7.502,-1.032,-27.927],[7.502,5.172,-19.998]]]
        report["upper_compare_region_min_y_mm"] = -1.099
        report["upper_difference_tolerance_mm3"] = args.max_upper_difference_mm3
        report["source_numerical_shared_face_repair"] = source_repair

        for side, expected_sign in (("left",1),("right",-1)):
            side_report = {}
            report["sides"][side] = side_report
            assembly = load_stl(args.manifest.parent/f"adapter_{side}_v2_assembly_mm.stl", 8)
            printing = load_stl(args.manifest.parent/f"adapter_{side}_v2_print_mm.stl", 8)
            meters = load_stl(args.manifest.parent/f"adapter_{side}_v2_m.stl", 12)
            three, three_metadata = native_3mf(args.manifest.parent/f"adapter_{side}_v2.3mf")
            side_report["native_3mf"] = three_metadata
            files = {"assembly_mm_stl": assembly, "print_mm_stl": printing, "meter_stl": meters, "native_3mf": three}
            side_report["geometry"] = {}
            for name, mesh in files.items():
                finite = bool(np.isfinite(mesh.vertices).all())
                components = int(mesh.body_count)
                properties = {"finite_vertices":finite, "watertight":bool(mesh.is_watertight), "consistent_winding":bool(mesh.is_winding_consistent), "positive_volume":bool(mesh.is_volume), "connected_components":components, "vertices":len(mesh.vertices), "triangles":len(mesh.faces), "bounds":mesh.bounds.tolist(), "signed_volume":float(mesh.volume)}
                side_report["geometry"][name] = properties
                require(finite and mesh.is_volume and components == 1, f"{side}/{name}: expected one finite closed positive solid")
            require(np.allclose(assembly.bounds, manifest["sides"][side]["bounds_assembly_mm"], rtol=0, atol=args.coordinate_tolerance_mm), f"{side}: assembly bounds differ from manifest")
            require(np.isclose(assembly.volume, manifest["sides"][side]["volume_mm3"], rtol=1e-5, atol=0), f"{side}: assembly volume differs from manifest")

            expected_print = assembly.copy()
            rx90 = np.eye(4); rx90[:3,:3] = [[1,0,0],[0,0,-1],[0,1,0]]
            expected_print.apply_transform(rx90)
            print_translation = -expected_print.bounds[0]
            expected_print.apply_translation(print_translation)
            converted_meters = meters.copy(); converted_meters.apply_scale(1000.)
            comparisons = {"assembly_to_print":compare_meshes(expected_print,printing,args.coordinate_tolerance_mm), "print_stl_to_native_3mf":compare_meshes(printing,three,args.coordinate_tolerance_mm), "assembly_mm_to_meters_times_1000":compare_meshes(assembly,converted_meters,args.coordinate_tolerance_mm)}
            side_report["cross_format_geometry_checks"] = comparisons
            side_report["expected_print_rotation"] = "+90 degrees about assembly X, then translate minimum XYZ to zero"
            side_report["print_translation_mm"] = print_translation.tolist()
            for name, result in comparisons.items():
                require(result["valid"], f"{side}: geometry mismatch in {name}")
            require(np.allclose(printing.bounds[0],0,rtol=0,atol=args.coordinate_tolerance_mm), f"{side}: print mesh is not translated to positive coordinates/build plane")
            inferred_scale_x = assembly.extents[0]/source_meshes["1"].extents[0]
            inferred_scale_top_y = assembly.bounds[1,1]/(source_meshes["1"].bounds[1,1]-datum[1])
            side_report["independent_scale_from_untouched_features"] = {"x_extent_ratio":float(inferred_scale_x),"top_y_ratio":float(inferred_scale_top_y),"expected_single_scale":scale,"double_scale_to_reject":scale**2}
            require(abs(inferred_scale_x-scale)<2e-6 and abs(inferred_scale_top_y-scale)<2e-6, f"{side}: untouched dimensions indicate missing or repeated print scale")

            rpy = config[side]["adapter_to_hand"]["rpy_rad"]
            rotation = trimesh.transformations.euler_matrix(*rpy,axes="sxyz")[:3,:3]
            dorsal = rotation @ np.array([-1.,0.,0.])
            slot_normal = np.array([0.,0.,float(expected_sign)])
            dot = float(dorsal@slot_normal)
            side_report["hand_back_alignment"] = {"dorsal_normal_adapter":dorsal.tolist(),"slot_outward_normal_adapter":slot_normal.tolist(),"dot_product":dot}
            require(dot>1-1e-10, f"{side}: slot is not on the hand dorsal side in current assembly config")
            require(manifest["sides"][side]["back_slot_adapter_z_sign"]==expected_sign, f"{side}: manifest handedness is incorrect")
            nominal = assembly.copy(); nominal.apply_scale(1./scale); nominal.apply_translation(datum)
            open_points = np.array([[x,y,expected_sign*z] for x,y,z in itertools.product([-6.,-3.,0.,3.,6.],[-0.8,0.,2.,4.9],[21.,24.,27.])])
            closed_points = open_points.copy(); closed_points[:,2]*=-1
            side_guard = np.array([[x,y,expected_sign*z] for x,y,z in itertools.product([-7.6,7.6],[0.,2.,4.9],[21.,24.,27.])])
            lower_guard = np.array([[x,-1.08,expected_sign*z] for x,z in itertools.product([-6.,0.,6.],[21.,24.,27.])])
            slot_results = {"intended_slot":nominal.contains(open_points),"opposite_old_slot_region":nominal.contains(closed_points),"slot_width_guard":nominal.contains(side_guard),"slot_lower_edge_guard":nominal.contains(lower_guard)}
            side_report["slot_material_probes"] = {name:{"sample_count":len(values),"solid_count":int(values.sum())} for name,values in slot_results.items()}
            require(not slot_results["intended_slot"].any(), f"{side}: intended 15 mm dorsal slot contains material")
            require(slot_results["opposite_old_slot_region"].all(), f"{side}: opposite wall/old 10 mm slot is not fully closed at sample points")
            require(slot_results["slot_width_guard"].all(), f"{side}: slot removes material beyond +/-7.5 mm width")
            require(slot_results["slot_lower_edge_guard"].all(), f"{side}: slot lower edge/web was removed")
            actual_outside = (as_solid(nominal) ^ upper_region) - windows
            removed = float((reference_outside-actual_outside).volume())
            added = float((actual_outside-reference_outside).volume())
            difference = abs(removed)+abs(added)
            side_report["upper_geometry_outside_slots"] = {"reference":"Original source upper minus original negative part, in nominal CAD mm", "removed_volume_mm3":removed,"added_volume_mm3":added,"symmetric_difference_volume_mm3":difference,"reference_volume_mm3":float(reference_outside.volume()),"valid":bool(np.isfinite(difference) and difference<=args.max_upper_difference_mm3)}
            require(side_report["upper_geometry_outside_slots"]["valid"], f"{side}: upper geometry changed outside the two slot windows ({difference} mm^3)")

        require(sha(args.manifest)==manifest_hash, "Manifest changed during validation; results are stale")
        require(hashlib.sha256(config_path.read_bytes()).hexdigest()==report["assembly_config_sha256"], "Assembly config changed during validation; results are stale")
        for name, initial_hash in initial_artifacts.items():
            require(sha(args.manifest.parent/name)==initial_hash, f"Artifact changed during validation: {name}")
        report["verified_artifact_sha256"] = initial_artifacts
        report["manifest_and_artifacts_stable_during_validation"] = not any("changed during" in e for e in errors)
    except Exception as exc:
        errors.append(f"Validation could not complete: {type(exc).__name__}: {exc}")
    report["valid"] = not errors
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({"valid":report["valid"],"errors":errors,"manifest_sha256":report.get("manifest_sha256"),"report":str(args.output)},indent=2,ensure_ascii=False))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
