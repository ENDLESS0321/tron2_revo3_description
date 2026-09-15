#!/usr/bin/env python3
"""Independent V3 thumb-side 60-degree-hole mounting audit.

This intentionally does not call the old 120-degree saddle-hole validator.
Generic circle/plate/optical helpers are shared; all saddle angles and both
actual wrist transforms are independently defined for the new side mount.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import manifold3d
import validate_new_bracket_mounts as common

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=ROOT/"design/wrist_camera_bracket_v3/manifest.json"
R_A_B=common.rot([0,1,0],math.pi/2)
R_WRIST_B=common.rot([1,0,0],-math.pi/2)@R_A_B


def circles_at_axis(mesh,theta,radials,radius):
    a=math.radians(theta);axis=np.array([math.cos(a),0,math.sin(a)])
    u=np.array([0.,1.,0.]);v=np.cross(axis,u);rows=[]
    for radial in radials:
        origin=radial*axis+[0,-4.5,0]
        row=common.pick_circle(common.circle_candidates(mesh,origin,axis,u,v),[0,0],radius)
        row["radial_plane_mm"]=radial;rows.append(row)
    return rows


def new_saddle_holes(mesh):
    result=[]
    for local,actual in ((60.,330.),(120.,30.)):
        rows=circles_at_axis(mesh,local,[31.2,32.7,33.8],1.7)
        counter=circles_at_axis(mesh,local,[35.],3.)[0]
        centers=np.array([r["center_3d_mm"] for r in rows]);axis=centers[-1]-centers[0];axis/=np.linalg.norm(axis)
        a_axis=R_A_B@axis
        desired=np.array([math.cos(math.radians(actual)),0,math.sin(math.radians(actual))])
        error=math.degrees(math.acos(np.clip(abs(a_axis@desired),-1,1)))
        point_B=30.5*np.array([math.cos(math.radians(local)),0,math.sin(math.radians(local))])+[0,-4.5,0]
        point_A=R_A_B@point_B
        result.append({"local_theta_deg":local,"target_actual_small_hole_theta_deg":actual,"point_B_mm":point_B.tolist(),"point_A_mm":point_A.tolist(),"measured_axis_B":axis.tolist(),"measured_axis_A":a_axis.tolist(),"axis_error_deg":error,"shaft_sections":rows,"head_seat_section":counter,"pass":error<.15 and all(r["pass"] for r in rows) and counter["pass"]})
    return result


def wrist_profiles_and_targets(mesh):
    def section(solid,y):
        path=solid.section(plane_origin=[0,y,0],plane_normal=[0,1,0]);polys=[]
        if path is None:raise ValueError("Missing clamp cross-section")
        for e in path.entities:
            q=path.vertices[e.points][:,[0,2]]
            if len(q)<4 or np.ptp(q,axis=0).max()<1e-5:continue
            if np.linalg.norm(q[0]-q[-1])>1e-4:raise ValueError("Open wrist/clamp contour")
            polys.append(q)
        return manifold3d.CrossSection(polys,manifold3d.FillRule.EvenOdd)
    ys=[-10.4,-8.,-5.,-.6];clamp_sections={y:section(mesh,y) for y in ys};output={}
    for side,letter in (("left","L"),("right","R")):
        path=ROOT/f"meshes/tron2/wrist_roll_{letter}_Link.STL"
        native=common.read_mesh(path,"m")
        in_A=native.copy();in_A.vertices=(in_A.vertices-[-31.7,0,-81.2])@common.rot([1,0,0],-math.pi/2)
        in_B=native.copy();in_B.vertices=(in_B.vertices-[-31.7,0,-81.2])@R_WRIST_B
        small={str(theta):circles_at_axis(in_A,theta,[29.],1.25)[0] for theta in (330.,30.)}
        # The larger retention bore has a 4.5mm throat at R~28mm and an
        # expanding countersink farther outward. Do not confuse it with 2.5mm
        # nominal small-hole cores or sample its countersink as its bore.
        large=circles_at_axis(in_A,15.,[28.],2.25)[0]
        sections=[]
        for y in ys:
            area=float((clamp_sections[y]^section(in_B,y)).area())
            sections.append({"y_B_mm":y,"intersection_area_mm2":area,"pass":area<1e-6})
        output[side]={"official_wrist_mesh_sha256":common.sha(path),"measured_actual_small_holes":small,"separate_large_retention_hole_15deg":large,"narrow_saddle_sections":sections,"same_R_wrist_B":R_WRIST_B.tolist(),"pass":all(r["pass"] for r in small.values()) and large["pass"] and all(r["pass"] for r in sections)}
    return output


def covered_big_hole(mesh):
    n=np.array([math.cos(math.radians(15)),0,math.sin(math.radians(15))]);tangent=np.cross(n,[0,1.,0])
    points_A=np.array([r*n+np.array([0,-4.5+dy,0])+dt*tangent for r in (31.2,33.,35.) for dy in (-3.,0.,3.) for dt in (-3.,0.,3.)])
    points_B=points_A@R_A_B
    inside=mesh.contains(points_B)
    return {"actual_retention_hole_theta_A_deg":15,"corresponding_theta_B_deg":105,"sample_count":len(points_B),"solid_point_count":int(inside.sum()),"points_B_mm":points_B.tolist(),"meaning":"The user-selected large-hole maintenance access is intentionally covered; this does not certify the remaining hardware fasteners or remove any robot screw.","pass":bool(inside.all())}


def thumb_direction():
    path=ROOT/"urdf/tron2_dach_revo3_v2.urdf";fk=common.urdf_fk(ET.parse(path).getroot());out={}
    for side,letter in (("left","L"),("right","R")):
        A=fk[f"wrist_roll_{letter}_Link"]@common.matrix([-.0317,0,-.0812],[-math.pi/2,0,0]);Ai=np.linalg.inv(A)
        positions={name:(Ai@fk[f"{side}_{name}"])[:3,3]*1000 for name in ("thumb_CMP_Link","thumb_CMR_Link","little_MCP_Link")}
        out[side]={"positions_A_mm":{k:v.tolist() for k,v in positions.items()},"thumb_is_A_positive_X":bool(positions["thumb_CMP_Link"][0]>0 and positions["thumb_CMR_Link"][0]>0 and positions["little_MCP_Link"][0]<0)}
    return {"source_urdf_sha256":common.sha(path),"zero_hand_joint_state":True,"sides":out,"pass":all(r["thumb_is_A_positive_X"] for r in out.values())}


def rig_check(path,length,tilt,expected_bracket_sha):
    root=ET.parse(path).getroot();fk=common.urdf_fk(root);rows=[]
    for side,letter in (("left","L"),("right","R")):
        prefix=side+"_wrist_camera";mount=prefix+"_mount_frame";plate=prefix+"_plate_frame";bottom=prefix+"_bottom_screw_frame"
        actual_mount=np.linalg.inv(fk[f"wrist_roll_{letter}_Link"])@fk[mount]
        expected_mount=np.eye(4);expected_mount[:3,:3]=R_WRIST_B;expected_mount[:3,3]=[-.0317,0,-.0812]
        plate_T=np.linalg.inv(fk[mount])@fk[plate]
        expected_plate=common.matrix([0,0,(35.8+length)*.001],[math.radians(tilt),0,0])
        bottom_T=np.linalg.inv(fk[plate])@fk[bottom];expected_bottom=np.eye(4)
        expected_bottom[:3,:3]=common.PLATE_BOTTOM_R;expected_bottom[:3,3]=common.PLATE_BOTTOM_T_MM*.001
        optical=np.linalg.inv(fk[plate])@fk[prefix+"_depth_optical_frame"]
        errors={"wrist_side_mount":float(np.max(abs(actual_mount-expected_mount))),"plate_15deg":float(np.max(abs(plate_T-expected_plate))),"D405_back_mount":float(np.max(abs(bottom_T-expected_bottom))),"optical_origin_m":float(np.linalg.norm(optical[:3,3]-[.009,.032,-.0212])),"optical_forward":float(np.linalg.norm(optical[:3,2]-[0,0,-1]))}
        bracket_tag=root.find(f"link[@name='{mount}']/visual/geometry/mesh");bracket_path=(path.parent/bracket_tag.get("filename")).resolve()
        camera_link=root.find(f"link[@name='{prefix}_link']");visual=camera_link.find("visual");camera_tag=visual.find("geometry/mesh");camera_path=(path.parent/camera_tag.get("filename")).resolve()
        origin=visual.find("origin");Tvisual=common.matrix(np.fromstring(origin.get("xyz","0 0 0"),sep=" "),np.fromstring(origin.get("rpy","0 0 0"),sep=" "))
        T=np.linalg.inv(fk[plate])@fk[prefix+"_link"]@Tvisual
        camera=common.read_mesh(camera_path,"m");scale=np.fromstring(camera_tag.get("scale","1 1 1"),sep=" ")
        v=(camera.vertices*scale)@T[:3,:3].T+T[:3,3]*1000
        bounds=np.array([v.min(0),v.max(0)]);case_error=float(np.max(abs(bounds-[[-21.09000015258789,11,-25],[21,53,-2]])))
        mesh_match=common.sha(bracket_path)==expected_bracket_sha
        rows.append({"side":side,"R_wrist_bracket":actual_mount[:3,:3].tolist(),"transform_errors":errors,"bracket_mesh_sha256":common.sha(bracket_path),"bracket_matches_final_manufacturing_mesh":mesh_match,"camera_case_bounds_plate_mm":bounds.tolist(),"camera_bound_error_mm":case_error,"camera_is_separate":bracket_path!=camera_path,"pass":bool(max(errors.values())<2e-8 and case_error<.002 and mesh_match and bracket_path!=camera_path)})
    active=[j for j in root.findall("joint") if j.get("type")!="fixed"]
    return {"urdf_sha256":common.sha(path),"active_robot_joint_count":len(active),"sides":rows,"pass":len(active)==58 and all(r["pass"] for r in rows)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--manifest",type=Path,default=DEFAULT);p.add_argument("--urdf",type=Path)
    args=p.parse_args();report={"schema":"thumb_side_saddle_mount_validation_v1","status":"failed","producer_sha256":common.sha(__file__),"generic_measurement_helper_sha256":common.sha(Path(common.__file__)),"scope":"New 60-degree clamp pair on actual wrist small holes 330/30; no hardware thread-depth or strength certification."}
    try:
        path=args.manifest.resolve();data=json.loads(path.read_text())
        if data.get("schema")!="wrist_camera_bracket_side_saddle_v3":raise ValueError("Expected V3 side-saddle manifest")
        for name,h in data["files"].items():
            if common.sha(path.parent/name)!=h:raise ValueError("Final manufacturing hash mismatch: "+name)
        length=float(data["parameters"]["length_mm"]);tilt=float(data["parameters"]["plate_tilt_deg"])
        if abs(tilt-15)>1e-9:raise ValueError("This user-requested default check requires a 15-degree plate")
        meshpath=path.parent/"whole_bracket_mm.stl";mesh=common.read_mesh(meshpath,"mm")
        report["inputs"]={"manifest_sha256":common.sha(path),"whole_stl_sha256":common.sha(meshpath),"whole_step_sha256":data["files"]["whole_bracket.step"],"length_mm":length,"plate_tilt_deg":tilt}
        report["solid"]={"watertight":bool(mesh.is_watertight),"positive_volume":bool(mesh.is_volume),"components":int(mesh.body_count),"volume_mm3":float(mesh.volume),"faces":len(mesh.faces)}
        report["new_60_degree_saddle_holes"]=new_saddle_holes(mesh)
        report["actual_wrist_small_vs_large_holes"]=wrist_profiles_and_targets(mesh)
        report["big_15deg_access_covered"]=covered_big_hole(mesh)
        report["thumb_side_evidence"]=thumb_direction()
        shifted=mesh.copy();shifted.apply_translation([0,0,62-(35.8+length)])
        report["plate_holes"]=common.plate_holes(shifted,0,tilt)
        report["camera_interface"]=common.camera_rear_geometry()
        report["camera_case_clearance"]=common.camera_case_clearance(shifted,0,tilt,report["camera_interface"]["camera_visual_bounds_plate_mm"])
        if args.urdf:report["rig"]=rig_check(args.urdf.resolve(),length,tilt,data["files"]["whole_bracket_m.stl"])
        checks={"one_positive_solid":mesh.is_volume and mesh.body_count==1,"new_60_degree_pair":all(r["pass"] for r in report["new_60_degree_saddle_holes"]),"targets_are_actual_small_holes_and_narrow_seat_fits":all(r["pass"] for r in report["actual_wrist_small_vs_large_holes"].values()),"specified_large_hole_covered":report["big_15deg_access_covered"]["pass"],"both_thumb_sides_are_A_positive_X":report["thumb_side_evidence"]["pass"],"plate_and_camera_interface":all(r["pass"] for r in report["plate_holes"]) and report["camera_interface"]["pass"],"camera_case_clearance":report["camera_case_clearance"]["pass"],"manifest_stable":common.sha(path)==report["inputs"]["manifest_sha256"]}
        if args.urdf:checks["integrated_symmetric_thumb_mounts"]=report["rig"]["pass"]
        report["checks"]={k:bool(v) for k,v in checks.items()};report["status"]="thumb_side_mount_pass" if all(checks.values()) else "thumb_side_mount_fail"
    except Exception as exc:report["error"]=str(exc)
    print(json.dumps(report,indent=2,allow_nan=False));return 0 if report["status"]=="thumb_side_mount_pass" else 2


if __name__=="__main__":raise SystemExit(main())
