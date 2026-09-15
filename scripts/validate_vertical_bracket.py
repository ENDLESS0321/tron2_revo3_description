#!/usr/bin/env python3
"""Independent V2 vertical-bracket audit; print JSON without editing CAD/rig.

Shared V1 helpers measure unchanged wrist/D405 interfaces. New tests cover the
vertical stem, filled maintenance cuts, and parameter-dependent solid geometry.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import validate_new_bracket_mounts as common

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=ROOT/"design/wrist_camera_bracket_v2/manifest.json"
START_Z=35.8


def checked_manifest(path):
    data=json.loads(path.read_text())
    if data.get("schema")!="wrist_camera_bracket_radial_stem_v2":raise ValueError("Not a vertical-bracket V2 manifest")
    for name,expected in data["files"].items():
        if common.sha(path.parent/name)!=expected:raise ValueError("Stale whole artifact: "+name)
    for part in data["parts"].values():
        for name,expected in part["files"].items():
            if common.sha(path.parent/name)!=expected:raise ValueError("Stale component artifact: "+name)
    return data


def cad_geometry(manifest_path,variant_paths):
    if importlib.util.find_spec("cadquery") is None:
        interpreter=ROOT/".cad-venv/bin/python"
        code="import sys,json;from pathlib import Path;sys.path.insert(0,sys.argv[1]);from validate_vertical_bracket import cad_geometry;print(json.dumps(cad_geometry(Path(sys.argv[2]),[Path(p) for p in json.loads(sys.argv[3])])))"
        env=dict(os.environ);env.pop("PYTHONPATH",None)
        run=subprocess.run([str(interpreter),"-c",code,str(Path(__file__).parent),str(manifest_path),json.dumps([str(p) for p in variant_paths])],env=env,text=True,capture_output=True)
        if run.returncode:raise RuntimeError("Independent STEP audit failed: "+run.stderr)
        result=json.loads(run.stdout);result["cad_interpreter"]=str(interpreter);return result
    import cadquery as cq
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_SurfaceType
    V=cq.Vector
    def read(path):
        work=cq.importers.importStep(str(path));solids=[s for sh in work.vals() for s in sh.Solids()]
        if len(solids)!=1 or not solids[0].isValid():raise ValueError("STEP is not one valid solid: "+str(path))
        return solids[0]
    def volume(shape):return sum(abs(s.Volume()) for s in shape.Solids())
    def difference(a,b):return volume(a.cut(b))+volume(b.cut(a))
    def bounds(shape):
        b=shape.BoundingBox();return np.array([[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]])
    data=checked_manifest(manifest_path);folder=manifest_path.parent
    length=float(data["parameters"]["length_mm"]);tilt=float(data["parameters"]["plate_tilt_deg"]);height=START_Z+length
    whole=read(folder/"whole_bracket.step")
    parts={name:read(folder/"parts"/(name+".step")) for name in ("base","stem","plate")}
    old_path=ROOT/"design/wrist_camera_bracket_v1/whole_bracket.step";old=read(old_path)
    # Restrict to the unchanged wrist-saddle band, excluding the old tall
    # pedestal and all plate/beam changes. This isolates the two filled cuts.
    band=cq.Solid.makeCylinder(35.8,10,V(0,-10.5,0),V(0,1,0))
    old_band=old.intersect(band);new_band=whole.intersect(band)
    added=new_band.cut(old_band);removed=old_band.cut(new_band)
    cutters=[];fills=[]
    for theta in (15.,105.):
        a=math.radians(theta);n=V(math.cos(a),0,math.sin(a))
        cutter=cq.Solid.makeCylinder(4.75,65,V(25*n.x,-4.5,25*n.z),n)
        cutters.append(cutter);added_volume=volume(added.intersect(cutter))
        fills.append({"former_access_theta_deg":theta,"restored_volume_in_saddle_band_mm3":added_volume,"pass":added_volume>.001})
    outside=volume(added.cut(cutters[0].fuse(cutters[1])))
    fill={"v1_step_sha256":common.sha(old_path),"mask":"R<=35.8mm, Y[-10.5,-0.5]mm","added_material_mm3":volume(added),"removed_material_mm3":volume(removed),"addition_outside_two_former_cutters_mm3":outside,"former_cut_regions":fills,"pass":volume(removed)<.001 and outside<.001 and all(row["pass"] for row in fills)}
    cylinders=[]
    for face in whole.Faces():
        s=BRepAdaptor_Surface(face.wrapped)
        if s.GetType()!=GeomAbs_SurfaceType.GeomAbs_Cylinder:continue
        c=s.Cylinder();a=c.Axis();p=a.Location();d=a.Direction()
        cylinders.append((c.Radius(),np.array([p.X(),p.Y(),p.Z()]),np.array([d.X(),d.Y(),d.Z()])))
    R=common.rot([1,0,0],math.radians(tilt));pivot=np.array([0,0,height]);targets=[]
    for theta in (30.,150.):
        a=math.radians(theta);axis=np.array([math.cos(a),0,math.sin(a)])
        targets.extend([(f"wrist_{theta:g}_shaft",1.7,30.5*axis+[0,-4.5,0],axis),(f"wrist_{theta:g}_seat",3.,34.4*axis+[0,-4.5,0],axis)])
    for x in (-10.,10.):targets.append((f"camera_plate_{x:g}",1.7,pivot+R@np.array([x,32,0]),R[:,2]))
    holes=[]
    for name,radius,point,axis in targets:
        options=[]
        for cr,cp,ca in cylinders:
            re=abs(cr-radius);pe=float(np.linalg.norm(np.cross(cp-point,axis)));ae=math.degrees(math.acos(np.clip(abs(ca@axis),-1,1)))
            options.append((re+pe+ae,re,pe,ae))
        _,re,pe,ae=min(options)
        holes.append({"name":name,"radius_error_mm":float(re),"axis_position_error_mm":pe,"axis_angle_error_deg":ae,"pass":bool(re<1e-5 and pe<1e-5 and ae<1e-4)})
    legacy_cylinders=sum(abs(radius-4.75)<1e-5 for radius,_,_ in cylinders)
    stem_bounds=bounds(parts["stem"])
    expected_bounds=np.array([[-12,-8.5,START_Z],[12,3.5,height]])
    stem_error=float(np.max(np.abs(stem_bounds-expected_bounds)))
    # Below the top gussets, any long +Y arm would enlarge these actual
    # sections beyond the 24x12mm upright cross-section.
    stem_volume=volume(parts["stem"])
    stem={"bounds_mm":stem_bounds.tolist(),"expected_bounds_mm":expected_bounds.tolist(),"bounds_error_mm":stem_error,"volume_mm3":stem_volume,"axis":"+Z","prismatic_y_extent_mm":float(stem_bounds[1,1]-stem_bounds[0,1]),"pass":stem_error<1e-5}
    union=parts["base"].fuse(parts["stem"],parts["plate"])
    partition_error=difference(whole,union)
    base_bounds=bounds(parts["base"])
    plate_local=parts["plate"].translate((0,0,-height)).rotate((0,0,0),(1,0,0),-tilt)
    plate_local_bounds=bounds(plate_local)
    # Tie semantic pieces back to the actual whole and bound every piece:
    # an old longitudinal beam cannot hide in a relabeled base/plate region.
    base_is_short=base_bounds[1,2]<=START_Z+1e-5 and base_bounds[1,1]<=3.5+1e-5
    plate_limits_low=np.array([-20.,-20.,-20.]);plate_limits_high=np.array([20.,52.,2.])
    plate_is_local=np.all(plate_local_bounds[0]>=plate_limits_low-1e-5) and np.all(plate_local_bounds[1]<=plate_limits_high+1e-5)
    structure={"whole_vs_component_union_difference_mm3":partition_error,"base_bounds_mm":base_bounds.tolist(),"plate_local_bounds_mm":plate_local_bounds.tolist(),"base_has_no_tall_or_long_arm":bool(base_is_short),"plate_confined_to_board_and_short_gussets":bool(plate_is_local),"no_old_longitudinal_arm":bool(partition_error<1e-4 and base_is_short and plate_is_local and stem["pass"])}
    comparisons=[]
    for path in variant_paths:
        vd=checked_manifest(path);vf=path.parent;vl=float(vd["parameters"]["length_mm"]);vt=float(vd["parameters"]["plate_tilt_deg"])
        vb=read(vf/"parts/base.step");vs=read(vf/"parts/stem.step");vp=read(vf/"parts/plate.step")
        base_delta=difference(parts["base"],vb)
        row={"manifest":str(path),"manifest_sha256":common.sha(path),"length_mm":vl,"tilt_deg":vt,"base_symmetric_difference_mm3":base_delta}
        if abs(vt-tilt)<1e-9 and abs(vl-length)>1e-9:
            dl=vl-length;vbounds=bounds(vs);expected=stem_bounds.copy();expected[1,2]+=dl
            row.update({"kind":"length","expected_top_translation_mm":[0,0,dl],"stem_bounds_change_error_mm":float(np.max(np.abs(vbounds-expected))),"plate_after_undo_Z_translation_difference_mm3":difference(parts["plate"],vp.translate((0,0,-dl)))})
            row["pass"]=base_delta<1e-5 and row["stem_bounds_change_error_mm"]<1e-5 and row["plate_after_undo_Z_translation_difference_mm3"]<1e-4
        elif abs(vl-length)<1e-9 and abs(vt-tilt)>1e-9:
            def board(shape,angle):
                local=shape.translate((0,0,-height)).rotate((0,0,0),(1,0,0),-angle)
                mask=cq.Solid.makeBox(60,90,10,V(-30,10,-5))
                return local.intersect(mask)
            row.update({"kind":"tilt","stem_symmetric_difference_mm3":difference(parts["stem"],vs),"board_after_undo_tilt_difference_mm3":difference(board(parts["plate"],tilt),board(vp,vt))})
            row["pass"]=base_delta<1e-5 and row["stem_symmetric_difference_mm3"]<1e-5 and row["board_after_undo_tilt_difference_mm3"]<1e-4
        else:raise ValueError("Each comparison must change only length or tilt")
        comparisons.append(row)
    return {"whole_step_sha256":common.sha(folder/"whole_bracket.step"),"step_single_valid_solid":True,"access_relief_restoration":fill,"measured_cylinder_interfaces":holes,"legacy_radius_4_75_cylinder_count":legacy_cylinders,"vertical_stem":stem,"component_structure":structure,"parameter_comparisons":comparisons,"pass":fill["pass"] and all(h["pass"] for h in holes) and legacy_cylinders==0 and stem["pass"] and structure["no_old_longitudinal_arm"] and all(r["pass"] for r in comparisons)}


def rig_check(path,length,tilt):
    root=ET.parse(path).getroot();fk=common.urdf_fk(root);rows=[]
    for side,letter in (("left","L"),("right","R")):
        prefix=side+"_wrist_camera";mount=prefix+"_mount_frame";plate=prefix+"_plate_frame";bottom=prefix+"_bottom_screw_frame"
        expected_mount=common.matrix([-.0317,0,-.0812],[-math.pi/2,0,0])
        if side=="right":expected_mount=expected_mount@common.matrix(rpy=[0,math.pi,0])
        mount_T=np.linalg.inv(fk["wrist_roll_"+letter+"_Link"])@fk[mount]
        plate_T=np.linalg.inv(fk[mount])@fk[plate]
        expected_plate=common.matrix([0,0,(START_Z+length)*.001],[math.radians(tilt),0,0])
        bottom_T=np.linalg.inv(fk[plate])@fk[bottom];expected_bottom=np.eye(4)
        expected_bottom[:3,:3]=common.PLATE_BOTTOM_R;expected_bottom[:3,3]=common.PLATE_BOTTOM_T_MM*.001
        optical=np.linalg.inv(fk[plate])@fk[prefix+"_depth_optical_frame"]
        errors={"wrist_mount":float(np.max(abs(mount_T-expected_mount))),"vertical_top_plate":float(np.max(abs(plate_T-expected_plate))),"camera_back_M3_mount":float(np.max(abs(bottom_T-expected_bottom))),"optical_origin_m":float(np.linalg.norm(optical[:3,3]-[.009,.032,-.0212])),"optical_forward":float(np.linalg.norm(optical[:3,2]-[0,0,-1]))}
        body=root.find(f"link[@name='{prefix}_link']");visual=body.find("visual");meshtag=visual.find("geometry/mesh")
        meshpath=(path.parent/meshtag.get("filename")).resolve();mesh=common.read_mesh(meshpath,"m")
        origin=visual.find("origin");scale=np.fromstring(meshtag.get("scale","1 1 1"),sep=" ")
        visual_T=common.matrix(np.fromstring(origin.get("xyz","0 0 0"),sep=" "),np.fromstring(origin.get("rpy","0 0 0"),sep=" "))
        T=np.linalg.inv(fk[plate])@fk[prefix+"_link"]@visual_T
        vertices=(mesh.vertices*scale)@T[:3,:3].T+T[:3,3]*1000
        bounds=np.array([vertices.min(0),vertices.max(0)])
        case_error=float(np.max(abs(bounds-[[-21.09000015258789,11,-25],[21,53,-2]])))
        separate=root.find(f"link[@name='{mount}']/visual/geometry/mesh").get("filename")!=meshtag.get("filename")
        rows.append({"side":side,"T_mount_plate":plate_T.tolist(),"transform_errors":errors,"camera_visual_bounds_plate_mm":bounds.tolist(),"camera_case_bound_error_mm":case_error,"camera_separate_link_and_mesh":separate,"pass":bool(max(errors.values())<2e-8 and case_error<.002 and separate)})
    active=[j for j in root.findall("joint") if j.get("type")!="fixed"]
    return {"urdf_sha256":common.sha(path),"actuated_count":len(active),"sides":rows,"pass":len(active)==58 and all(r["pass"] for r in rows)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--manifest",type=Path,default=DEFAULT);p.add_argument("--variant-manifest",type=Path,action="append",default=[]);p.add_argument("--urdf",type=Path)
    args=p.parse_args();report={"schema":"vertical_bracket_independent_geometry_v1","status":"failed","producer_sha256":common.sha(__file__),"scope":"New V2 vertical CAD geometry and camera mounting; no hardware thread, cable strain or strength certification."}
    try:
        path=args.manifest.resolve();data=checked_manifest(path);length=float(data["parameters"]["length_mm"]);tilt=float(data["parameters"]["plate_tilt_deg"])
        meshpath=path.parent/"whole_bracket_mm.stl";mesh=common.read_mesh(meshpath,"mm")
        report["inputs"]={"manifest":str(path),"manifest_sha256":common.sha(path),"whole_mesh_sha256":common.sha(meshpath),"length_mm":length,"tilt_deg":tilt}
        report["solid"]={"watertight":bool(mesh.is_watertight),"consistent_winding":bool(mesh.is_winding_consistent),"positive_volume":bool(mesh.is_volume),"components":int(mesh.body_count),"volume_mm3":float(mesh.volume),"bounds_mm":mesh.bounds.tolist()}
        report["wrist_mounting_holes"]=common.radial_holes(mesh)
        # Coordinate-only rigid translation adapts invariant V1 plate/case
        # measurement helpers to the new top pivot (0,0,35.8+L).
        shifted=mesh.copy();shifted.apply_translation([0,0,62-(START_Z+length)])
        report["plate_holes"]=common.plate_holes(shifted,0,tilt)
        report["camera_interface"]=common.camera_rear_geometry()
        report["camera_case_clearance"]=common.camera_case_clearance(shifted,0,tilt,report["camera_interface"]["camera_visual_bounds_plate_mm"])
        report["wrist_saddle_fit"]=common.wrist_section_fit(mesh)
        report["step_and_changes"]=cad_geometry(path,[v.resolve() for v in args.variant_manifest])
        if args.urdf:report["rig"]=rig_check(args.urdf.resolve(),length,tilt)
        checks={"single_valid_solid":mesh.is_volume and mesh.body_count==1,"small_mounting_holes_preserved":all(r["pass"] for r in report["wrist_mounting_holes"]),"camera_plate_holes":all(r["pass"] for r in report["plate_holes"]),"camera_separate_clearance":report["camera_case_clearance"]["pass"],"wrist_section_fit":report["wrist_saddle_fit"]["pass"],"step_fill_and_vertical_geometry":report["step_and_changes"]["pass"],"input_manifest_stable":common.sha(path)==report["inputs"]["manifest_sha256"] and common.sha(meshpath)==report["inputs"]["whole_mesh_sha256"]}
        if args.urdf:checks["integrated_rig"]=report["rig"]["pass"]
        report["checks"]={k:bool(v) for k,v in checks.items()};report["status"]="vertical_bracket_geometry_pass" if all(checks.values()) else "vertical_bracket_geometry_fail"
    except Exception as exc:report["error"]=str(exc)
    print(json.dumps(report,indent=2,allow_nan=False));return 0 if report["status"]=="vertical_bracket_geometry_pass" else 2


if __name__=="__main__":raise SystemExit(main())
