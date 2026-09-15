#!/usr/bin/env python3
"""Independently measure the new common wrist bracket and its camera mounting.

Reads a final assembly-coordinate STL, optional design manifest, and the camera
URDF. Prints JSON to stdout; it never edits CAD, rig configuration or viewers.
Units of the supplied bracket STL must be stated explicitly.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parents[1]
CAMERA_NATIVE = ROOT / "vendor/realsense-description/realsense2_description/meshes/d405.stl"
PLATE_BOTTOM_R = np.array([[0.,1.,0.],[0.,0.,-1.],[-1.,0.,0.]])
PLATE_BOTTOM_T_MM = np.array([0.,53.,-10.35])


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rot(axis, angle):
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
    x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)


def matrix(xyz=(0,0,0),rpy=(0,0,0)):
    T=np.eye(4);T[:3,3]=xyz
    T[:3,:3]=rot([0,0,1],rpy[2])@rot([0,1,0],rpy[1])@rot([1,0,0],rpy[0])
    return T


def read_mesh(path,units):
    mesh=trimesh.load_mesh(path,process=False)
    if not isinstance(mesh,trimesh.Trimesh):raise ValueError("Expected one triangle mesh")
    mesh.merge_vertices(digits_vertex=12 if units=="m" else 9)
    if units=="m":mesh.apply_scale(1000)
    return mesh


def circle_candidates(mesh,origin,normal,basis_u,basis_v):
    section=mesh.section(plane_origin=origin,plane_normal=normal)
    candidates=[]
    if section is None:return candidates
    for entity in section.entities:
        points=section.vertices[entity.points]
        if len(points)<9 or np.linalg.norm(points[0]-points[-1])>1e-5:continue
        delta=points-np.asarray(origin)
        q=np.column_stack([delta@basis_u,delta@basis_v])
        if np.ptp(q,axis=0).max()>12 or np.ptp(q,axis=0).min()<.2:continue
        fit=np.linalg.lstsq(np.column_stack([2*q,np.ones(len(q))]),np.sum(q*q,axis=1),rcond=None)[0]
        center=fit[:2];radius=math.sqrt(max(0.,fit[2]+center@center))
        error=float(np.max(np.abs(np.linalg.norm(q-center,axis=1)-radius)))
        candidates.append({"center_2d_mm":center.tolist(),"radius_mm":radius,"max_circle_error_mm":error,"point_count":len(points),"center_3d_mm":(np.asarray(origin)+center[0]*basis_u+center[1]*basis_v).tolist()})
    return candidates


def pick_circle(candidates,expected_center,radius):
    if not candidates:raise ValueError("No closed small-hole contour found at the required plane")
    expected=np.asarray(expected_center)
    row=min(candidates,key=lambda r:np.linalg.norm(np.asarray(r["center_2d_mm"])-expected)+abs(r["radius_mm"]-radius)+r["max_circle_error_mm"])
    row=dict(row);row["center_error_mm"]=float(np.linalg.norm(np.asarray(row["center_2d_mm"])-expected));row["diameter_error_mm"]=abs(2*row["radius_mm"]-2*radius)
    row["pass"]=row["center_error_mm"]<.04 and row["diameter_error_mm"]<.04 and row["max_circle_error_mm"]<.025
    return row


def radial_holes(mesh):
    holes=[]
    for theta in (30.,150.):
        a=math.radians(theta);normal=np.array([math.cos(a),0.,math.sin(a)])
        u=np.array([0.,1.,0.]);v=np.cross(normal,u)
        slices=[]
        # The final CAD has an outer 6mm head-seat counterbore starting at
        # axis radius 34.4mm. Measure the 3.4mm shaft below that floor.
        for radial in (31.2,32.7,33.8):
            origin=radial*normal+np.array([0.,-4.5,0.])
            row=pick_circle(circle_candidates(mesh,origin,normal,u,v),[0,0],1.7)
            row["radial_plane_mm"]=radial;slices.append(row)
        centers=np.array([row["center_3d_mm"] for row in slices])
        axis=centers[-1]-centers[0];axis/=np.linalg.norm(axis)
        angle=math.degrees(math.acos(np.clip(abs(axis@normal),-1,1)))
        seat_origin=35.*normal+np.array([0.,-4.5,0.])
        seat=pick_circle(circle_candidates(mesh,seat_origin,normal,u,v),[0,0],3.)
        triangles=np.asarray(mesh.triangles)
        radial_values=triangles@normal
        centers=np.asarray(mesh.triangles_center)
        near_plane=np.max(np.abs(radial_values-34.4),axis=1)<.002
        around_hole=(np.abs(centers[:,1]+4.5)<3.1)&(np.abs(centers@v)<3.1)
        oriented=(np.asarray(mesh.face_normals)@normal)>.999
        seat_area=float(np.asarray(mesh.area_faces)[near_plane&around_hole&oriented].sum())
        holes.append({"theta_deg":theta,"reference_at_R30_5_mm":(30.5*normal+[0,-4.5,0]).tolist(),"expected_outward_axis":normal.tolist(),"fitted_axis":axis.tolist(),"axis_error_deg":angle,"measured_slices":slices,"clearance_diameter_mm":3.4,"head_counterbore":seat,"head_seat_plane_radius_mm":34.4,"measured_planar_seat_area_mm2":seat_area,"hardware_thread_confirmed":False,"pass":bool(angle<.15 and all(row["pass"] for row in slices) and seat["pass"] and seat_area>10.)})
    return holes


def plate_holes(mesh,length_mm,tilt_deg):
    R=rot([1,0,0],math.radians(tilt_deg));pivot=np.array([0.,length_mm,62.])
    local=mesh.copy();local.vertices=(local.vertices-pivot)@R
    rows=[]
    for z in (-1.5,0.,1.5):
        candidates=circle_candidates(local,[0,0,z],[0,0,1],np.array([1,0,0]),np.array([0,1,0]))
        pair=[]
        for x in (-10.,10.):
            circle=pick_circle(candidates,[x,32.],1.7);circle["plate_z_mm"]=z;pair.append(circle)
        spacing=np.linalg.norm(np.asarray(pair[1]["center_2d_mm"])-pair[0]["center_2d_mm"])
        rows.append({"plate_z_mm":z,"hole_measurements":pair,"spacing_mm":float(spacing),"pass":bool(abs(spacing-20)<.04 and all(row["pass"] for row in pair))})
    return rows


def camera_rear_geometry():
    camera=read_mesh(CAMERA_NATIVE,"mm")
    candidates=circle_candidates(camera,[0,0,-22.95],[0,0,1],np.array([1,0,0]),np.array([0,1,0]))
    measured=[pick_circle(candidates,[x,0],1.25) for x in (-10.,10.)]
    points=np.array([[-8.35,-10,21],[-8.35,10,21.]])
    plate_points=points@PLATE_BOTTOM_R.T+PLATE_BOTTOM_T_MM
    expected=np.array([[-10,32,-2],[10,32,-2.]])
    # Optical body rotation maps native STL by Rx(pi) after composing the
    # official camera visual origin; this is also checked independently below.
    native_R=np.diag([1.,-1.,-1.]);native_t=np.array([0.,32.,-25.])
    native_in_plate=camera.vertices@native_R.T+native_t
    return {"source":str(CAMERA_NATIVE.relative_to(ROOT)),"source_sha256":sha(CAMERA_NATIVE),"source_native_rear_hole_sections":measured,"rear_thread":"M3 per official datasheet, despite the simplified mesh's approximately 2.5mm thread-core aperture","rear_thread_max_insertion_mm":4.,"T_plate_camera_bottom":{"rotation":PLATE_BOTTOM_R.tolist(),"xyz_mm":PLATE_BOTTOM_T_MM.tolist()},"mapped_rear_M3_points_plate_mm":plate_points.tolist(),"rear_hole_mapping_max_error_mm":float(np.max(np.abs(plate_points-expected))),"camera_visual_bounds_plate_mm":[native_in_plate.min(axis=0).tolist(),native_in_plate.max(axis=0).tolist()],"lens_direction_plate":(PLATE_BOTTOM_R@np.array([1,0,0])).tolist(),"camera_top_direction_plate":(PLATE_BOTTOM_R@np.array([0,0,1])).tolist(),"usb_exit_direction_plate":(PLATE_BOTTOM_R@np.array([0,-1,0])).tolist(),"pass":bool(all(row["pass"] for row in measured) and np.max(np.abs(plate_points-expected))<1e-10)}


def camera_case_clearance(mesh,length_mm,tilt_deg,camera_bounds):
    R=rot([1,0,0],math.radians(tilt_deg));pivot=np.array([0.,length_mm,62.])
    local=mesh.copy();local.vertices=(local.vertices-pivot)@R
    lower,upper=np.asarray(camera_bounds,dtype=float)
    tolerance=.001
    triangles=np.asarray(local.triangles)
    potential=np.all(triangles.max(axis=1)>lower+tolerance,axis=1)&np.all(triangles.min(axis=1)<upper-tolerance,axis=1)
    midpoint=(lower+upper)*.5
    midpoint_inside=bool(local.contains([midpoint])[0])
    return {"method":"Conservative D405 case AABB in plate coordinates. Require no bracket triangle AABB inside its open interior, plus an exterior midpoint test to exclude containment. Contact at the rear mating plane is allowed.","camera_case_bounds_plate_mm":[lower.tolist(),upper.tolist()],"boundary_tolerance_mm":tolerance,"potential_bracket_triangle_count":int(potential.sum()),"camera_case_midpoint_inside_bracket":midpoint_inside,"scope":"Camera case only; external USB connector, cable bend radius and all robot motion are outside this check.","pass":not bool(potential.any()) and not midpoint_inside}


def wrist_section_fit(bracket):
    import manifold3d
    def section(mesh,y):
        path=mesh.section(plane_origin=[0,y,0],plane_normal=[0,1,0])
        if path is None:raise ValueError("Missing required wrist/base cross-section")
        polygons=[]
        for entity in path.entities:
            q=path.vertices[entity.points][:,[0,2]]
            if len(q)<4 or np.ptp(q,axis=0).max()<1e-5:continue
            if np.linalg.norm(q[0]-q[-1])>1e-4:raise ValueError("Required cross-section contains an open contour")
            polygons.append(q)
        return manifold3d.CrossSection(polygons,manifold3d.FillRule.EvenOdd)
    ys=[-10.4,-8.,-5.,-.6];rows=[]
    bracket_sections={y:section(bracket,y) for y in ys}
    for side,letter in (("left","L"),("right","R")):
        path=ROOT/f"meshes/tron2/wrist_roll_{letter}_Link.STL"
        wrist=read_mesh(path,"m")
        R=rot([1,0,0],-math.pi/2)
        if side=="right":R=R@rot([0,1,0],math.pi)
        wrist.vertices=(wrist.vertices-[-31.7,0,-81.2])@R
        for y in ys:
            a,b=bracket_sections[y],section(wrist,y);area=float((a^b).area())
            rows.append({"side":side,"y_bracket_mm":y,"bracket_section_area_mm2":float(a.area()),"wrist_section_area_mm2":float(b.area()),"intersection_area_mm2":area,"closed_contours":True,"pass":area<1e-6})
    return {"method":"Actual whole-bracket and official wrist visual closed 2D contours, transformed into each side's bracket frame; EvenOdd solid polygon intersection.","sample_y_mm":ys,"rows":rows,"maximum_intersection_area_mm2":max(r["intersection_area_mm2"] for r in rows),"scope":"Only the eight specified static sections. The official wrist visual mesh is not a watertight solid, so no 3D intersection-volume or CCD claim is made.","pass":all(r["pass"] for r in rows)}


def step_holes(path,length_mm,tilt_deg):
    # CAD and rendering use separate existing environments. Keep their binary
    # dependencies isolated rather than installing or mixing them for a check.
    import importlib.util
    if importlib.util.find_spec("cadquery") is None:
        interpreter=ROOT/".cad-venv/bin/python"
        if not interpreter.is_file():raise RuntimeError("STEP check needs the existing .cad-venv CadQuery environment")
        code="import sys,json;from pathlib import Path;sys.path.insert(0,sys.argv[1]);from validate_new_bracket_mounts import step_holes;print(json.dumps(step_holes(Path(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]))))"
        env=dict(os.environ);env.pop("PYTHONPATH",None)
        result=subprocess.run([str(interpreter),"-c",code,str(Path(__file__).parent),str(path.resolve()),str(length_mm),str(tilt_deg)],env=env,text=True,capture_output=True,check=False)
        if result.returncode:raise RuntimeError("Independent STEP helper failed: "+result.stderr)
        report=json.loads(result.stdout);report["cad_interpreter"]=str(interpreter)
        return report
    import cadquery as cq
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_SurfaceType
    imported=cq.importers.importStep(str(path));solids=[s for shape in imported.vals() for s in shape.Solids()]
    cylinders=[]
    for solid in solids:
        for face in solid.Faces():
            surface=BRepAdaptor_Surface(face.wrapped)
            if surface.GetType()!=GeomAbs_SurfaceType.GeomAbs_Cylinder:continue
            cylinder=surface.Cylinder();axis=cylinder.Axis();p=axis.Location();d=axis.Direction()
            cylinders.append({"radius_mm":float(cylinder.Radius()),"axis_point_mm":np.array([p.X(),p.Y(),p.Z()]),"axis_direction":np.array([d.X(),d.Y(),d.Z()])})
    R=rot([1,0,0],math.radians(tilt_deg));pivot=np.array([0.,length_mm,62.]);targets=[]
    for theta in (30.,150.):
        a=math.radians(theta);axis=np.array([math.cos(a),0.,math.sin(a)])
        for radius,label in ((1.7,"shaft"),(3.,"head_counterbore")):
            targets.append((f"wrist_{theta:g}_{label}",radius,30.5*axis+[0,-4.5,0],axis))
    for x in (-10.,10.):targets.append((f"camera_plate_{x:g}",1.7,pivot+R@np.array([x,32,0]),R[:,2]))
    results=[]
    for name,radius,point,axis in targets:
        candidates=[]
        for cylinder in cylinders:
            radius_error=abs(cylinder["radius_mm"]-radius)
            direction=cylinder["axis_direction"]
            angle=math.degrees(math.acos(np.clip(abs(direction@axis),-1,1)))
            distance=float(np.linalg.norm(np.cross(cylinder["axis_point_mm"]-point,axis)))
            candidates.append((radius_error+distance+angle,cylinder,radius_error,distance,angle))
        if not candidates:raise ValueError("No analytical cylinder surfaces found in the STEP")
        _,cylinder,re,de,ae=min(candidates,key=lambda row:row[0])
        results.append({"feature":name,"radius_error_mm":float(re),"axis_line_error_mm":de,"axis_angle_error_deg":ae,"measured_axis_point_mm":cylinder["axis_point_mm"].tolist(),"measured_axis_direction":cylinder["axis_direction"].tolist(),"pass":bool(re<1e-5 and de<1e-5 and ae<1e-4)})
    return {"path":str(path.resolve()),"sha256":sha(path),"solid_count":len(solids),"all_solids_valid":all(s.isValid() for s in solids),"analytical_cylinder_face_count":len(cylinders),"measured_features":results,"pass":len(solids)==1 and all(s.isValid() for s in solids) and all(row["pass"] for row in results)}


def urdf_fk(root):
    links={link.get("name") for link in root.findall("link")}
    pending=list(root.findall("joint"));children={j.find("child").get("link") for j in pending}
    roots=links-children
    if len(roots)!=1:raise ValueError("URDF must have one root")
    out={roots.pop():np.eye(4)}
    while pending:
        changed=False
        for joint in list(pending):
            parent=joint.find("parent").get("link");child=joint.find("child").get("link")
            if parent not in out:continue
            origin=joint.find("origin");xyz=np.fromstring(origin.get("xyz","0 0 0"),sep=" ");rpy=np.fromstring(origin.get("rpy","0 0 0"),sep=" ")
            # Zero revolute state is sufficient: all mount comparisons below
            # are relative to their wrist/plate and should be joint-invariant.
            out[child]=out[parent]@matrix(xyz,rpy);pending.remove(joint);changed=True
        if not changed:raise ValueError("URDF is disconnected or cyclic")
    return out


def rig_geometry(path,length_mm,tilt_deg):
    root=ET.parse(path).getroot();fk=urdf_fk(root);rows=[]
    for side,letter in (("left","L"),("right","R")):
        prefix=side+"_wrist_camera";mount=prefix+"_mount_frame";plate=prefix+"_plate_frame";bottom=prefix+"_bottom_screw_frame"
        missing=[name for name in (mount,plate,bottom,"wrist_roll_"+letter+"_Link") if name not in fk]
        if missing:raise ValueError("Required assembled camera frames missing: "+str(missing))
        actual_mount=np.linalg.inv(fk["wrist_roll_"+letter+"_Link"])@fk[mount]
        expected_mount=matrix([-.0317,0,-.0812],[-math.pi/2,0,0])
        if side=="right":expected_mount=expected_mount@matrix(rpy=[0,math.pi,0])
        actual_plate=np.linalg.inv(fk[mount])@fk[plate]
        expected_plate=matrix([0,length_mm*.001,.062],[math.radians(tilt_deg),0,0])
        actual_bottom=np.linalg.inv(fk[plate])@fk[bottom]
        expected_bottom=np.eye(4);expected_bottom[:3,:3]=PLATE_BOTTOM_R;expected_bottom[:3,3]=PLATE_BOTTOM_T_MM*.001
        errors={"wrist_mount":float(np.max(np.abs(actual_mount-expected_mount))),"plate":float(np.max(np.abs(actual_plate-expected_plate))),"camera_rear_mount":float(np.max(np.abs(actual_bottom-expected_bottom)))}
        optical=np.linalg.inv(fk[plate])@fk[prefix+"_depth_optical_frame"]
        forward=optical[:3,2]
        optical_position_error=float(np.linalg.norm(optical[:3,3]-[.009,.032,-.0212]))
        camera_link=root.find(f"link[@name='{prefix}_link']")
        visual=camera_link.find("visual");mesh_node=visual.find("geometry/mesh")
        mesh_path=(path.parent/mesh_node.get("filename")).resolve()
        camera_mesh=trimesh.load_mesh(mesh_path,process=False)
        scale=np.fromstring(mesh_node.get("scale","1 1 1"),sep=" ")
        visual_origin=visual.find("origin")
        T_visual=matrix(np.fromstring(visual_origin.get("xyz","0 0 0"),sep=" "),np.fromstring(visual_origin.get("rpy","0 0 0"),sep=" "))
        T_plate_visual=np.linalg.inv(fk[plate])@fk[prefix+"_link"]@T_visual
        vertices=(np.asarray(camera_mesh.vertices)*scale)@T_plate_visual[:3,:3].T+T_plate_visual[:3,3]
        actual_bounds=np.array([vertices.min(axis=0),vertices.max(axis=0)])*1000
        expected_bounds=np.array([[-21.09000015258789,11,-25],[21,53,-2.]])
        case_bound_error=float(np.max(np.abs(actual_bounds-expected_bounds)))
        camera_asset={"path":str(mesh_path),"sha256":sha(mesh_path),"bounds_plate_mm":actual_bounds.tolist(),"max_bound_error_mm":case_bound_error}
        attachment_joints=[j for j in root.findall("joint") if prefix in j.get("name","")]
        fixed=all(j.get("type")=="fixed" for j in attachment_joints)
        rows.append({"side":side,"T_wrist_mount":actual_mount.tolist(),"T_mount_plate":actual_plate.tolist(),"T_plate_camera_bottom":actual_bottom.tolist(),"transform_max_errors":errors,"optical_forward_plate":forward.tolist(),"optical_position_error_m":optical_position_error,"actual_camera_visual":camera_asset,"new_attachment_joints_all_fixed":fixed,"pass":bool(max(errors.values())<2e-8 and np.linalg.norm(forward-[0,0,-1])<2e-8 and optical_position_error<2e-8 and case_bound_error<.002 and fixed)})
    active=[j.get("name") for j in root.findall("joint") if j.get("type")!="fixed"]
    return {"urdf":str(path.resolve()),"urdf_sha256":sha(path),"actuated_joint_count":len(active),"no_new_physical_hinge":len(active)==58 and all(r["new_attachment_joints_all_fixed"] for r in rows),"sides":rows,"pass":len(active)==58 and all(r["pass"] for r in rows)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembly-stl",type=Path,required=True)
    parser.add_argument("--units",choices=("mm","m"),default="mm")
    parser.add_argument("--length-mm",type=float,required=True)
    parser.add_argument("--tilt-deg",type=float,required=True)
    parser.add_argument("--manifest",type=Path)
    parser.add_argument("--step",type=Path,help="Verify cylinder axes directly from the exported STEP BRep")
    parser.add_argument("--urdf",type=Path,help="Omit until the new solid bracket has been integrated into the camera URDF")
    args=parser.parse_args()
    report={"schema":"new_wrist_camera_bracket_mount_validation_v1","status":"failed","producer_sha256":sha(__file__),"claim_boundary":"New CAD geometry and nominal camera mounting only. This is not recovered STEP, measured thread depth, cable bend/strain validation, or a strength/load certificate."}
    try:
        inputs=[args.assembly_stl,CAMERA_NATIVE,ROOT/"meshes/tron2/wrist_roll_L_Link.STL",ROOT/"meshes/tron2/wrist_roll_R_Link.STL"]+[p for p in (args.manifest,args.step,args.urdf) if p is not None]
        before={str(p.resolve()):sha(p) for p in inputs}
        if args.manifest:
            design=json.loads(args.manifest.read_text())
            if abs(design["parameters"]["length_mm"]-args.length_mm)>1e-9 or abs(design["parameters"]["plate_tilt_deg"]-args.tilt_deg)>1e-9:
                raise ValueError("Command-line geometry parameters differ from the design manifest")
            for p in (args.assembly_stl,args.step):
                if p is not None and design["files"].get(p.name)!=sha(p):
                    raise ValueError("Geometry file does not match the design manifest: "+str(p))
        mesh=read_mesh(args.assembly_stl,args.units)
        report["input"]={"assembly_stl":str(args.assembly_stl.resolve()),"sha256":sha(args.assembly_stl),"declared_units":args.units,"length_mm":args.length_mm,"tilt_deg":args.tilt_deg}
        if args.manifest:report["input"].update({"manifest":str(args.manifest.resolve()),"manifest_sha256":sha(args.manifest)})
        report["solid"]={"watertight":bool(mesh.is_watertight),"winding_consistent":bool(mesh.is_winding_consistent),"positive_volume":bool(mesh.is_volume),"connected_components":int(mesh.body_count),"volume_mm3":float(mesh.volume),"bounds_mm":mesh.bounds.tolist()}
        report["wrist_radial_holes"]=radial_holes(mesh)
        report["camera_plate_holes"]=plate_holes(mesh,args.length_mm,args.tilt_deg)
        report["camera_rear_mount"]=camera_rear_geometry()
        report["camera_case_vs_bracket"]=camera_case_clearance(mesh,args.length_mm,args.tilt_deg,report["camera_rear_mount"]["camera_visual_bounds_plate_mm"])
        report["wrist_saddle_section_fit"]=wrist_section_fit(mesh)
        if args.step:report["step_hole_geometry"]=step_holes(args.step,args.length_mm,args.tilt_deg)
        if args.urdf:report["assembled_rig"]=rig_geometry(args.urdf,args.length_mm,args.tilt_deg)
        checks={"single_positive_watertight_solid":mesh.is_volume and mesh.body_count==1,"wrist_hole_centers_axes_and_clearance":all(r["pass"] for r in report["wrist_radial_holes"]),"plate_two_M3_clearance_holes":all(r["pass"] for r in report["camera_plate_holes"]),"rear_mount_TF_and_camera_direction":report["camera_rear_mount"]["pass"],"camera_case_excludes_bracket":report["camera_case_vs_bracket"]["pass"],"wrist_saddle_sampled_fit":report["wrist_saddle_section_fit"]["pass"]}
        if args.step:checks["step_analytical_hole_geometry"]=report["step_hole_geometry"]["pass"]
        if args.urdf:checks["assembled_fixed_mounts_and_camera_TF"]=report["assembled_rig"]["pass"]
        checks["input_hashes_stable"]=all(sha(Path(p))==h for p,h in before.items())
        report["checked_input_hashes"]=before
        report["checks"]={k:bool(v) for k,v in checks.items()}
        report["status"]="bracket_geometry_pass" if all(checks.values()) else "bracket_geometry_fail"
        report["rig_validation_performed"]=bool(args.urdf)
    except Exception as exc:
        report["error"]=str(exc)
    print(json.dumps(report,indent=2,allow_nan=False))
    return 0 if report["status"]=="bracket_geometry_pass" else 2


if __name__=="__main__":raise SystemExit(main())
