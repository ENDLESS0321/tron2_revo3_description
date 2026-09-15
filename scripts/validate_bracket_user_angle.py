#!/usr/bin/env python3
"""V4 audit: the requested angle is BETWEEN mounting plate and support stem.

Measure actual STL plate normals and URDF axes; do not equate a stored '15'
with a 15-degree physical bend. Side-hole checks use local 60/120 -> 330/30.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import validate_thumb_side_mount as side
import validate_new_bracket_mounts as common

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=ROOT/"design/wrist_camera_bracket_v4/manifest.json"


def actual_stl_angle(folder,user_angle):
    plate=common.read_mesh(folder/"parts/plate_mm.stl","mm")
    stem=common.read_mesh(folder/"parts/stem_mm.stl","mm")
    normals=np.asarray(plate.face_normals).copy();areas=np.asarray(plate.area_faces)
    # Group real planar triangle normals, sign-normalized toward +Z. The
    # broad plate front/back dominates the small neck/gusset surfaces.
    normals[normals[:,2]<0]*=-1
    groups=collections.defaultdict(list)
    for i,n in enumerate(normals):
        if abs(n[0])<.01 and np.linalg.norm(n)>.9:
            groups[tuple(np.round(n,3))].append(i)
    if not groups:raise ValueError("No actual plate plane found")
    ids=max(groups.values(),key=lambda ids:areas[ids].sum())
    normal=np.average(normals[ids],axis=0,weights=areas[ids]);normal/=np.linalg.norm(normal)
    plate_direction=np.cross(normal,[1.,0.,0.]);plate_direction/=np.linalg.norm(plate_direction)
    # The measured principal axis of this 80mm prismatic stem is +Z. This
    # establishes the stem direction from the actual part, not its metadata.
    values,vectors=np.linalg.eigh(np.asarray(stem.moment_inertia))
    stem_direction=vectors[:,np.argmin(values)]
    if stem_direction[2]<0:stem_direction*=-1
    dot=float(np.clip(plate_direction@stem_direction,-1,1));angle=math.degrees(math.acos(dot))
    desired=np.array([0.,math.sin(math.radians(user_angle)),math.cos(math.radians(user_angle))])
    direction_error=float(np.linalg.norm(plate_direction-desired))
    return {"plate_part_sha256":common.sha(folder/"parts/plate_mm.stl"),"stem_part_sha256":common.sha(folder/"parts/stem_mm.stl"),"method":"Area-weighted dominant actual plate triangle normal and actual stem principal inertia axis; take their oriented length-direction dot product.","plane_triangle_count":len(ids),"plane_area_mm2":float(areas[ids].sum()),"measured_plate_normal":normal.tolist(),"measured_plate_length_direction":plate_direction.tolist(),"measured_stem_direction":stem_direction.tolist(),"plate_length_dot_stem":dot,"expected_cos_user_angle":math.cos(math.radians(user_angle)),"measured_plate_to_stem_angle_deg":angle,"requested_plate_to_stem_angle_deg":user_angle,"direction_error":direction_error,"stem_bounds_mm":stem.bounds.tolist(),"pass":bool(abs(angle-user_angle)<.01 and direction_error<1e-4 and np.linalg.norm(stem_direction-[0,0,1])<1e-4)}


def actual_urdf_angle(path,user_angle):
    fk=common.urdf_fk(ET.parse(path).getroot());rows=[]
    for name in ("left","right"):
        root=fk[name+"_wrist_camera_mount_frame"];plate=fk[name+"_wrist_camera_plate_frame"]
        plate_y=plate[:3,1];stem_z=root[:3,2]
        dot=float(np.clip(plate_y@stem_z,-1,1));angle=math.degrees(math.acos(dot))
        rows.append({"side":name,"actual_plate_length_world":plate_y.tolist(),"actual_stem_axis_world":stem_z.tolist(),"dot":dot,"expected_cos_15deg":math.cos(math.radians(user_angle)),"actual_angle_deg":angle,"pass":abs(angle-user_angle)<1e-7})
    return {"urdf_sha256":common.sha(path),"sides":rows,"pass":all(row["pass"] for row in rows)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--manifest",type=Path,default=DEFAULT);p.add_argument("--urdf",type=Path);p.add_argument("--user-angle",type=float,default=15.)
    args=p.parse_args();report={"schema":"plate_to_stem_angle_and_thumb_mount_v4_validation","status":"failed","producer_sha256":common.sha(__file__),"side_measurement_helper_sha256":common.sha(Path(side.__file__)),"generic_measurement_helper_sha256":common.sha(Path(common.__file__)),"scope":"V4 actual plate-to-stem angle plus thumb-side small-hole/camera geometry; no physical thread-depth or strength certification."}
    try:
        path=args.manifest.resolve();data=json.loads(path.read_text());folder=path.parent
        if data.get("schema")!="wrist_camera_bracket_side_bend_v4":raise ValueError("Expected V4 user-angle manifest")
        for name,h in data["files"].items():
            if common.sha(folder/name)!=h:raise ValueError("Manufacturing file SHA mismatch: "+name)
        for part in data["parts"].values():
            for name,h in part["files"].items():
                if common.sha(folder/name)!=h:raise ValueError("Component file SHA mismatch: "+name)
        length=float(data["parameters"]["length_mm"]);user=float(data["parameters"]["plate_to_stem_angle_deg"]);physical=float(data["parameters"]["plate_plane_rotation_deg"])
        if abs(user-args.user_angle)>1e-9 or abs(physical-(90-user))>1e-9:raise ValueError("Manifest does not represent the requested plate-to-stem angle")
        meshpath=folder/"whole_bracket_mm.stl";mesh=common.read_mesh(meshpath,"mm")
        report["inputs"]={"manifest_sha256":common.sha(path),"whole_stl_sha256":common.sha(meshpath),"whole_step_sha256":data["files"]["whole_bracket.step"],"length_mm":length,"user_plate_to_stem_angle_deg":user,"internal_CAD_Rx_deg":physical}
        report["solid"]={"positive_volume":bool(mesh.is_volume),"watertight":bool(mesh.is_watertight),"components":int(mesh.body_count),"faces":len(mesh.faces)}
        report["actual_STL_angle"]=actual_stl_angle(folder,user)
        report["actual_new_60_degree_holes"]=side.new_saddle_holes(mesh)
        report["official_wrist_fit"]=side.wrist_profiles_and_targets(mesh)
        report["covered_large_15deg_hole"]=side.covered_big_hole(mesh)
        report["thumb_direction_FK"]=side.thumb_direction()
        shifted=mesh.copy();shifted.apply_translation([0,0,62-(35.8+length)])
        report["camera_plate_holes"]=common.plate_holes(shifted,0,physical)
        report["D405_interface"]=common.camera_rear_geometry()
        report["camera_case_clearance"]=common.camera_case_clearance(shifted,0,physical,report["D405_interface"]["camera_visual_bounds_plate_mm"])
        if args.urdf:
            rig=side.rig_check(args.urdf.resolve(),length,physical,data["files"]["whole_bracket_m.stl"])
            for row in rig["sides"]:row["transform_errors"]["plate_physical_pose"]=row["transform_errors"].pop("plate_15deg")
            report["integrated_rig"]=rig;report["actual_URDF_angle"]=actual_urdf_angle(args.urdf.resolve(),user)
        checks={"single_positive_solid":mesh.is_volume and mesh.body_count==1,"actual_STL_plate_stem_angle":report["actual_STL_angle"]["pass"],"small_holes_60_120_map_330_30":all(r["pass"] for r in report["actual_new_60_degree_holes"]),"both_official_wrists_fit":all(r["pass"] for r in report["official_wrist_fit"].values()),"large_hole_covered":report["covered_large_15deg_hole"]["pass"],"thumb_side_FK":report["thumb_direction_FK"]["pass"],"D405_back_holes_and_case":all(r["pass"] for r in report["camera_plate_holes"]) and report["D405_interface"]["pass"] and report["camera_case_clearance"]["pass"],"manifest_stable":common.sha(path)==report["inputs"]["manifest_sha256"]}
        if args.urdf:checks.update({"actual_URDF_plate_stem_angle":report["actual_URDF_angle"]["pass"],"actual_rig_mesh_and_TF":report["integrated_rig"]["pass"]})
        report["checks"]={k:bool(v) for k,v in checks.items()};report["status"]="v4_user_angle_geometry_pass" if all(checks.values()) else "v4_user_angle_geometry_fail"
    except Exception as exc:report["error"]=str(exc)
    print(json.dumps(report,indent=2,allow_nan=False));return 0 if report["status"]=="v4_user_angle_geometry_pass" else 2


if __name__=="__main__":raise SystemExit(main())
