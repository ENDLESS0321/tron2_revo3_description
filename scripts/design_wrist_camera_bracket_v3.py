#!/usr/bin/env python3
"""V3: 60-degree side-mount saddle, solid radial stem, and separate camera plate.

V1 and V2 remain frozen. The former +Y horizontal arm and both large access-relief
cuts are absent. Camera geometry is never included in the bracket solid.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cadquery as cq
import design_wrist_camera_bracket as v1

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT=ROOT/"design/wrist_camera_bracket_v3"
START_Z=35.8
V=cq.Vector
box=v1.box
clean=v1.clean
volume=v1.volume
sha=v1.sha


def fillet_parallel(shape,axis,radius,notes,name):
    try:
        result=cq.Workplane("XY").newObject([shape]).edges("|"+axis).fillet(radius).val()
        if result.isValid():
            notes[name]={"applied":True,"radius_mm":radius,"edge_direction":axis}
            return result
    except Exception as exc:
        notes[name]={"applied":False,"radius_mm":radius,"reason":str(exc)}
    return shape


def make_base(notes):
    inner,outer=30.8,35.8
    low,high=math.radians(52),math.radians(128)
    point=lambda r,a:(r*math.cos(a),r*math.sin(a))
    saddle=(cq.Workplane("XZ",origin=(0,-.5,0))
        .moveTo(*point(inner,low)).threePointArc(point(inner,math.pi/2),point(inner,high))
        .lineTo(*point(outer,high)).threePointArc(point(outer,math.pi/2),point(outer,low))
        .close().extrude(10).val())
    saddle=fillet_parallel(saddle,"Y",.8,notes,"saddle_end_edges")
    # A short solid root pad blends the curved saddle into the upright stem.
    # Its top defines Z=35.8; no V1 tall pedestal or horizontal arm is retained.
    pad=fillet_parallel(box(-12,12,-8.5,3.5,32.8,START_Z),"Z",.8,notes,"root_pad_edges")
    base=clean(saddle.fuse(pad))
    inner_clearance=cq.Solid.makeCylinder(inner,30,V(0,-20,0),V(0,1,0))
    base=clean(base.cut(inner_clearance))
    holes=[]
    for theta in [60.,120.]:
        angle=math.radians(theta)
        axis=V(math.cos(angle),0,math.sin(angle))
        base=base.cut(cq.Solid.makeCylinder(1.7,25,V(25*axis.x,-4.5,25*axis.z),axis))
        base=base.cut(cq.Solid.makeCylinder(3.,12,V(34.4*axis.x,-4.5,34.4*axis.z),axis))
        holes.append({"theta_deg":theta,"axis_direction":[axis.x,0,axis.z],"axis_point_at_wrist_r30_5_mm":[30.5*axis.x,-4.5,30.5*axis.z],"clearance_diameter_mm":3.4,"seat_counterbore_diameter_mm":6.0,"seat_plane_axis_radius_mm":34.4,"thread_status":"original robot thread and engagement depth are unconfirmed"})
    # Intentionally no 9.5 mm access-relief cuts at theta 15/105 or elsewhere.
    return clean(base),holes


def make_stem(length,notes):
    top=START_Z+length
    # Solid rectangular section avoids the open-U section's weak torsion.
    stem=box(-12,12,-8.5,3.5,START_Z,top)
    return clean(fillet_parallel(stem,"Z",.8,notes,"stem_vertical_edges"))


def plate_point(point,length,tilt):
    x,y,z=point
    angle=math.radians(tilt)
    return [x,y*math.cos(angle)-z*math.sin(angle),START_Z+length+y*math.sin(angle)+z*math.cos(angle)]


def make_plate(length,tilt):
    local=box(-20,20,10,52,-2,2).fuse(box(-12,12,-6,12,-2,2))
    for x in [-10.,10.]:
        local=local.cut(cq.Solid.makeCylinder(1.7,12,V(x,32,-6),V(0,0,1)))
    raw=clean(local).rotate((0,0,0),(1,0,0),tilt).translate((0,0,START_Z+length))
    # Short true gussets join the top 18 mm of the radial stem to the plate
    # neck. Their plate-local extent ends at Y=8, before the D405 body Y>=11.
    top=START_Z+length
    p8=plate_point([0,8,1.2],length,tilt)
    p2=plate_point([0,2,1.2],length,tilt)
    yz=[(-6.,top-18),(3.,top-18),(p8[1],p8[2]),(p2[1],p2[2])]
    raw=raw.fuse(v1.yz_prism(-11.8,-9.2,yz),v1.yz_prism(9.2,11.8,yz))
    return clean(raw),yz


def build(args):
    if not 20<=args.length_mm<=150:raise ValueError("length_mm must be 20..150 along +Z")
    if not 0<=args.plate_tilt_deg<=75:raise ValueError("plate_tilt_deg must be 0..75")
    if not math.isfinite(args.density_kg_m3) or args.density_kg_m3<=0:raise ValueError("density must be positive")
    output=args.output_directory.resolve();output.mkdir(parents=True,exist_ok=True)
    part_dir=output/"parts";part_dir.mkdir(exist_ok=True)
    notes={}
    base,wrist_holes=make_base(notes)
    stem=clean(make_stem(args.length_mm,notes).cut(base))
    raw_plate,gusset_yz=make_plate(args.length_mm,args.plate_tilt_deg)
    whole=clean(base.fuse(stem,raw_plate))
    plate=clean(raw_plate.cut(base.fuse(stem)))
    fragments={"count":0,"volume_mm3":0.}
    if len(plate.Solids())>1:
        ordered=sorted(plate.Solids(),key=volume,reverse=True)
        amount=sum(volume(s) for s in ordered[1:])
        if amount>.05:raise ValueError(f"Disconnected plate region: {amount} mm3")
        fragments={"count":len(ordered)-1,"volume_mm3":amount}
        plate=ordered[0]
        whole=clean(base.fuse(stem,plate))
    parts={"base":base,"stem":stem,"plate":plate}
    if not whole.isValid() or len(whole.Solids())!=1:
        raise ValueError("V3 whole bracket must be one valid connected solid")
    for name,shape in parts.items():
        if not shape.isValid() or len(shape.Solids())!=1:
            raise ValueError(f"V3 part {name} must be one valid solid")
    whole_volume=volume(whole)
    partition=sum(volume(part) for part in parts.values())
    if abs(partition-whole_volume)>1e-3:raise ValueError("V3 component partition failed")
    step=output/"whole_bracket.step";mm=output/"whole_bracket_mm.stl";metric=output/"whole_bracket_m.stl"
    v1.export_shape(whole,step,"STEP");v1.export_shape(whole,mm,"STL")
    v1.metric_mesh_from_mm(mm,metric)
    step_check=v1.validate_step(step,whole)
    mm_check=v1.validate_stl(mm,whole_volume,"mm")
    metric_check=v1.validate_stl(metric,whole_volume*1e-9,"m")
    if not step_check["valid"] or step_check["solid_count"]!=1 or step_check["volume_absolute_error_mm3"]>1e-3:
        raise ValueError("V3 exported STEP failed reimport")
    if not mm_check["is_volume"] or not metric_check["is_volume"] or max(mm_check["relative_volume_error"],metric_check["relative_volume_error"])>.005:
        raise ValueError("V3 exported STL failed closed-volume verification")
    records={}
    for name,shape in parts.items():
        part_step=part_dir/(name+".step");part_mm=part_dir/(name+"_mm.stl");part_m=part_dir/(name+"_body_m.stl")
        v1.export_shape(shape,part_step,"STEP");v1.export_shape(shape,part_mm,"STL")
        if name=="stem":xyz=[0,0,START_Z];rpy=[0,0,0]
        elif name=="plate":xyz=[0,0,START_Z+args.length_mm];rpy=[args.plate_tilt_deg,0,0]
        else:xyz=[0,0,0];rpy=[0,0,0]
        local=shape.translate(tuple(-x for x in xyz)).rotate((0,0,0),(1,0,0),-rpy[0])
        v1.metric_mesh_from_mm(part_mm,part_m,xyz,rpy[0])
        records[name]={**v1.properties(shape,args.density_kg_m3),"body_local_properties":v1.properties(local,args.density_kg_m3),"body_frame":{"xyz_mm":xyz,"rpy_deg":rpy},"files":{str(p.relative_to(output)):sha(p) for p in [part_step,part_mm,part_m]},"step_roundtrip":v1.validate_step(part_step,shape)}
    angle=math.radians(args.plate_tilt_deg)
    report={
        "schema":"wrist_camera_bracket_side_saddle_v3",
        "design_basis":"user V3 side mounting: local hole pair 60/120 degrees, 60-degree angular spacing; retain the V2 solid radial stem, no horizontal arm or large access reliefs",
        "parameters":{"length_mm":args.length_mm,"plate_tilt_deg":args.plate_tilt_deg,"density_kg_m3":args.density_kg_m3,"length_range_mm":[20,150],"plate_tilt_range_deg":[0,75]},
        "parameter_semantics":{"length":"radial upright length from fixed base top Z=35.8 along +Z; no inherited +Y arm length","plate_tilt":"Rx around the upright top; plate/short neck/gussets regenerated while base and upright stay fixed","parts":"three nonoverlapping semantic regions of one integral part; camera and fasteners separate","regenerate_on_parameter_change":True},
        "coordinate_convention":{"units":"mm except *_m.stl and explicit SI fields","root":"original hand-adapter shoulder datum","axes":{"X":"across wrist","Y":"wrist axis toward fingers","Z":"radially outward and along the V3 upright"},"theta":"atan2(Z,X), positive equals right-handed Ry(-theta)"},
        "plate_frame":{"xyz_mm":[0,0,START_Z+args.length_mm],"rpy_deg":[args.plate_tilt_deg,0,0],"pivot_axis_in_root":[1,0,0]},
        "camera_mount":{"relative_to":"plate_frame","contact_point_mm":[0,32,-2],"contact_plane_normal_plate":[0,0,1],"camera_optical_direction_plate":[0,0,-1],"bottom_screw_frame":{"xyz_m":[0,.053,-.01035],"rotation_matrix":[[0,1,0],[0,0,-1],[-1,0,0]]},"camera_geometry_included":False,"camera_back_hole_points_bottom_screw_mm":[[-8.35,-10,21],[-8.35,10,21]],"camera_thread_depth_reference_mm":4},
        "features":{"wrist_mount_holes":wrist_holes,"existing_adapter_retention_access_reliefs":{"enabled":False,"theta_deg":[],"diameter_mm":0,"removed_from_v1":[15,105]},"longitudinal_arm_present":False,"stem_axis_root":[0,0,1],"stem_start_mm":[0,0,START_Z],"stem_end_mm":[0,0,START_Z+args.length_mm],"stem_prismatic_y_range_mm":[-8.5,3.5],"plate_gusset_yz_root_mm":gusset_yz,"camera_plate_holes":[{"center_plate_mm":[x,32,0],"center_root_mm":plate_point([x,32,0],args.length_mm,args.plate_tilt_deg),"axis_plate":[0,0,1],"axis_root":[0,-math.sin(angle),math.cos(angle)],"diameter_mm":3.4} for x in [-10,10]],"saddle_radial_clearance_mm":.3},
        "strength_geometry":{"section_type":"solid_rectangle_with_vertical_edge_fillets","stem_axis":[0,0,1],"stem_start_z_mm":START_Z,"stem_length_mm":args.length_mm,"stem_width_mm":24,"stem_depth_mm":12,"web_thickness_mm":12,"web_y_range_mm":[-8.5,3.5],"rib_width_mm":0,"rib_added_depth_mm":0,"section_bbox_xy_mm":[[-12,-8.5],[12,3.5]],"plate_width_mm":40,"plate_length_mm":42,"plate_thickness_mm":4,"plate_root_gusset_width_mm":2.6,"plate_root_gusset_vertical_overlap_mm":18,"plate_root_gusset_max_plate_y_mm":8,"camera_com_plate_local_mm":[0,32,-14],"camera_com_status":"explicit review assumption, not a measured camera COM","density_kg_m3":args.density_kg_m3,"saddle_inner_radius_mm":30.8,"saddle_outer_radius_mm":35.8,"saddle_y_range_mm":[-10.5,-.5],"fixed_root_pad_z_mm":[32.8,35.8],"tilt_range_deg":[0,75],"default_tilt_deg":15,"fillets":notes},
        "whole":v1.properties(whole,args.density_kg_m3),"parts":records,
        "files":{p.name:sha(p) for p in [step,mm,metric]},
        "verification":{"whole_step_roundtrip":step_check,"whole_stl_mm":mm_check,"whole_stl_m":metric_check,"component_volume_partition_error_mm3":abs(partition-whole_volume),"plate_partition_fragment_cleanup":fragments,"physical_fit_or_strength_certified":False},
        "generator_sha256":sha(Path(__file__)),"frozen_v1_helper_sha256":sha(Path(v1.__file__)),
        "prototype_status":"material/print process and original robot threads/screw lengths unconfirmed; geometry is not a strength certification"
    }
    report["features"]["wrist_hole_angular_separation_deg"]=60.
    report["features"]["wrist_hole_angles_local_deg"]=[60.,120.]
    report["features"]["saddle_angle_range_local_deg"]=[52.,128.]
    report["strength_geometry"]["saddle_angle_range_deg"]=[52.,128.]
    report["frozen_v2_baseline_sha256"]=sha(ROOT/"scripts/design_wrist_camera_bracket_v2.py")
    report["mounting_reference"]={"external_root_transform_required":True,"adapter_to_bracket_rpy_deg":[0,90,0],"selected_global_wrist_hole_pair_deg":[330.,30.],"positive_y_90_maps_holes_to_deg":[330.,30.],"negative_y_90_maps_holes_to_deg":[150.,210.],"global_pair_selected_by_assembly_config":True,"same_root_rotation_for_left_and_right":True,"covered_large_retention_hole_global_deg":15.,"large_hole_access_relief_present":False}
    (output/"manifest.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"output":str(output),"parameters":report["parameters"],"plate_frame":report["plate_frame"],"whole":report["whole"],"verification":report["verification"]},indent=2))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--length-mm",type=float,default=80.)
    parser.add_argument("--plate-tilt-deg",type=float,default=15.)
    parser.add_argument("--density-kg-m3",type=float,default=1250.)
    parser.add_argument("--output-directory",type=Path,default=DEFAULT_OUTPUT)
    build(parser.parse_args())


if __name__=="__main__":main()
