#!/usr/bin/env python3
"""New reinforced wrist-camera bracket: integral saddle, arm, tilted plate.

This is a new design from reference photos and measured mounting axes, not a
reconstruction of the original bracket STEP. Camera geometry is never included.
All CAD coordinates are millimeters; the root equals the hand-adapter datum.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT=ROOT/"design/wrist_camera_bracket_v1"
DEFAULT_DENSITY=1250.
V=cq.Vector


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def radians(degrees):return math.radians(degrees)


def box(x0,x1,y0,y1,z0,z1):
    return cq.Solid.makeBox(x1-x0,y1-y0,z1-z0,V(x0,y0,z0))


def clean(shape):
    return shape.clean()


def volume(shape):
    return float(shape.Volume(1e-9))


def rotate_plate(shape,length,tilt):
    return shape.rotate((0,0,0),(1,0,0),tilt).translate((0,length,62.))


def plate_point(point,length,tilt):
    x,y,z=point
    a=radians(tilt)
    return [x,length+y*math.cos(a)-z*math.sin(a),62.+y*math.sin(a)+z*math.cos(a)]


def try_longitudinal_fillet(shape,radius,notes,name):
    try:
        result=cq.Workplane("XY").newObject([shape]).edges("|Y").fillet(radius).val()
        if result.isValid():
            notes[name]={"applied":True,"radius_mm":radius}
            return result
    except Exception as exc:
        notes[name]={"applied":False,"radius_mm":radius,"reason":str(exc)}
    return shape


def make_base(notes):
    inner,outer=30.8,35.8
    lo,hi=radians(22),radians(158)
    points=lambda r,a:(r*math.cos(a),r*math.sin(a))
    # Standard XZ workplane has normal -Y, so extrude from the front edge.
    saddle=(cq.Workplane("XZ",origin=(0,-.5,0))
        .moveTo(*points(inner,lo)).threePointArc(points(inner,math.pi/2),points(inner,hi))
        .lineTo(*points(outer,hi)).threePointArc(points(outer,math.pi/2),points(outer,lo))
        .close().extrude(10.).val())
    saddle=try_longitudinal_fillet(saddle,.8,notes,"saddle_end_edge_fillet")
    # Fixed radial pedestal is part of the wrist base, not the adjustable arm.
    # It raises the longitudinal arm clear of the hand-adapter shoulder.
    pedestal=box(-12,12,-8.5,0,33,64)
    pedestal=pedestal.fuse(box(-12,-9,-8.5,0,64,72),box(9,12,-8.5,0,64,72))
    pedestal=try_longitudinal_fillet(pedestal,.8,notes,"pedestal_longitudinal_fillet")
    base=clean(saddle.fuse(pedestal))
    # Keep the wrist envelope free even where the pedestal meets the saddle.
    bore=cq.Solid.makeCylinder(inner,25,V(0,-20,0),V(0,1,0))
    base=clean(base.cut(bore))
    hole_features=[]
    for theta in [30.,150.]:
        a=radians(theta);axis=V(math.cos(a),0,math.sin(a))
        p=V(25*axis.x,-4.5,25*axis.z)
        base=base.cut(cq.Solid.makeCylinder(1.7,25,p,axis))
        # A flat counterbore floor gives a washer/head support perpendicular
        # to each nonparallel hole axis; actual screw selection is unverified.
        seat_r=34.4
        base=base.cut(cq.Solid.makeCylinder(3.0,12,V(seat_r*axis.x,-4.5,seat_r*axis.z),axis))
        hole_features.append({"theta_deg":theta,"axis_direction":[axis.x,axis.y,axis.z],"axis_point_at_wrist_r30_5_mm":[30.5*axis.x,-4.5,30.5*axis.z],"clearance_diameter_mm":3.4,"seat_plane_axis_radius_mm":seat_r,"seat_counterbore_diameter_mm":6.0,"seat_ring_radial_width_mm":1.3,"remaining_nominal_wall_at_seat_mm":seat_r-inner,"thread_status":"original robot thread and engagement depth not confirmed"})
    # Preserve access to the existing adapter-retention screws where this
    # upper saddle would otherwise cover their outer countersinks.
    for theta in [15.,105.]:
        a=radians(theta);axis=V(math.cos(a),0,math.sin(a))
        # 9.5 mm (rather than 9.0) deliberately opens the relief past Y=0;
        # an exactly tangent cut would leave a zero-thickness nonmanifold seam.
        base=base.cut(cq.Solid.makeCylinder(4.75,65,V(25*axis.x,-4.5,25*axis.z),axis))
    return clean(base),hole_features


def make_arm(length,notes):
    arm=box(-12,12,0,length,58,64)
    arm=arm.fuse(box(-12,-9,0,length,64,72),box(9,12,0,length,64,72))
    return clean(try_longitudinal_fillet(arm,.8,notes,"support_arm_longitudinal_fillet"))


def yz_prism(x0,x1,yz):
    wire=cq.Wire.makePolygon([V(x0,y,z) for y,z in yz]+[V(x0,*yz[0])])
    return cq.Solid.extrudeLinear(wire,[],V(x1-x0,0,0))


def make_plate(length,tilt):
    # Camera-body clearance: the camera backside covers local Y≈11..53;
    # no bracket material protrudes below the Z=-2 contact plane there.
    local=box(-20,20,10,52,-2,2).fuse(box(-12,12,-6,12,-2,2))
    for x in [-10.,10.]:
        local=local.cut(cq.Solid.makeCylinder(1.7,12,V(x,32,-6),V(0,0,1)))
    raw=rotate_plate(clean(local),length,tilt)
    # Real side gussets reconnect the tilted plate to the fixed arm ribs.
    # These roots are rebuilt for every angle; this is not a movable hinge.
    p14=plate_point([0,14,1.2],length,tilt)
    p2=plate_point([0,2,1.2],length,tilt)
    yz=[(length-18,63.5),(length-18,71.5),(p14[1],p14[2]),(p2[1],p2[2])]
    gussets=[yz_prism(-12,-9,yz),yz_prism(9,12,yz)]
    raw=clean(raw.fuse(*gussets))
    # Remove the unused rear tongue below the arm's rounded underside. This
    # prevents a disconnected semantic sliver while the ribs/gussets provide
    # the actual integral plate-root connection above the arm web.
    raw=clean(raw.cut(box(-100,100,-100,300,-100,59.0)))
    return raw,yz


def properties(shape,density):
    bounds=shape.BoundingBox()
    props=GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped,props,1e-9)
    amount=float(props.Mass())
    center=props.CentreOfMass()
    # OCP inertia at unit density is mm^5. kg/m^3 * 1e-15 converts to kg m^2.
    matrix=props.MatrixOfInertia()
    inertia=np.array([[matrix.Value(i+1,j+1) for j in range(3)] for i in range(3)])*density*1e-15
    return {"volume_mm3":amount,"mass_kg":amount*density*1e-9,"density_kg_m3":density,"center_of_mass_mm":[center.X(),center.Y(),center.Z()],"inertia_at_com_kg_m2":inertia.tolist(),"bounds_mm":[[bounds.xmin,bounds.ymin,bounds.zmin],[bounds.xmax,bounds.ymax,bounds.zmax]],"brep_valid":bool(shape.isValid()),"solid_count":len(shape.Solids())}


def export_shape(shape,path,kind,tolerance=.025):
    if kind=="STEP":cq.exporters.export(shape,str(path),exportType="STEP")
    elif kind=="STL":cq.exporters.export(shape,str(path),exportType="STL",tolerance=tolerance,angularTolerance=.08)
    else:raise ValueError(kind)


def validate_step(path,expected):
    loaded=cq.importers.importStep(str(path))
    solids=[s for sh in loaded.vals() for s in sh.Solids()]
    amount=sum(volume(s) for s in solids)
    return {"solid_count":len(solids),"valid":all(s.isValid() for s in solids),"volume_mm3":amount,"volume_absolute_error_mm3":abs(amount-volume(expected))}


def validate_stl(path,expected_volume,units):
    import trimesh
    mesh=trimesh.load_mesh(path,process=False)
    mesh.merge_vertices(digits_vertex=9 if units=="mm" else 12)
    good=(mesh.faces[:,0]!=mesh.faces[:,1])&(mesh.faces[:,1]!=mesh.faces[:,2])&(mesh.faces[:,0]!=mesh.faces[:,2])
    mesh.update_faces(good);mesh.update_faces(mesh.unique_faces())
    return {"watertight":bool(mesh.is_watertight),"is_volume":bool(mesh.is_volume),"faces":len(mesh.faces),"volume":float(mesh.volume),"relative_volume_error":abs(float(mesh.volume)-expected_volume)/expected_volume}


def metric_mesh_from_mm(source,destination,body_xyz=(0,0,0),body_roll_deg=0.):
    """Preserve the conformal millimeter triangulation under unit/frame changes."""
    import trimesh
    mesh=trimesh.load_mesh(source,process=False)
    angle=radians(body_roll_deg)
    rotation=np.array([[1,0,0],[0,math.cos(angle),-math.sin(angle)],[0,math.sin(angle),math.cos(angle)]])
    mesh.vertices=(np.asarray(mesh.vertices)-np.array(body_xyz))@rotation*.001
    mesh.export(destination)


def build(args):
    if not 20<=args.length_mm<=150:raise ValueError("length_mm must be 20..150")
    if not 0<=args.plate_tilt_deg<=75:raise ValueError("plate_tilt_deg must be 0..75")
    if not math.isfinite(args.density_kg_m3) or args.density_kg_m3<=0:raise ValueError("density must be positive")
    output=args.output_directory.resolve();output.mkdir(parents=True,exist_ok=True)
    part_dir=output/"parts";part_dir.mkdir(exist_ok=True)
    notes={}
    base,wrist_holes=make_base(notes)
    arm=make_arm(args.length_mm,notes)
    plate_raw,gusset_yz=make_plate(args.length_mm,args.plate_tilt_deg)
    whole=clean(base.fuse(arm,plate_raw))
    # Three nonoverlapping semantic regions of a single manufactured piece.
    # They are not three independently fastened physical components.
    arm=clean(arm.cut(base))
    plate=clean(plate_raw.cut(base.fuse(arm)))
    fragment_cleanup={"count":0,"volume_mm3":0.0}
    if len(plate.Solids())>1:
        ordered=sorted(plate.Solids(),key=volume,reverse=True)
        fragment_volume=sum(volume(item) for item in ordered[1:])
        if fragment_volume>.05:
            raise ValueError("Plate root has a substantial disconnected semantic component")
        # At very low angles the rectangular gusset edge can extend a few
        # microns beyond a rounded arm corner. Remove only those measured
        # tiny edge remnants, then reconstruct the actual complete solid.
        fragment_cleanup={"count":len(ordered)-1,"volume_mm3":fragment_volume}
        plate=ordered[0]
        whole=clean(base.fuse(arm,plate))
    parts={"wrist_mount":base,"support_arm":arm,"camera_plate":plate}
    if not whole.isValid() or len(whole.Solids())!=1:
        raise ValueError("The complete bracket must be one valid connected BRep solid")
    for name,shape in parts.items():
        if not shape.isValid() or len(shape.Solids())!=1:
            raise ValueError(f"Semantic part {name} is not one valid solid")
    part_volume=sum(volume(shape) for shape in parts.values())
    whole_volume=volume(whole)
    if abs(part_volume-whole_volume)>1e-3:
        raise ValueError("Semantic component volumes do not partition the whole bracket")
    whole_step=output/"whole_bracket.step"
    whole_mm=output/"whole_bracket_mm.stl"
    whole_m=output/"whole_bracket_m.stl"
    export_shape(whole,whole_step,"STEP");export_shape(whole,whole_mm,"STL")
    metric_mesh_from_mm(whole_mm,whole_m)
    step_check=validate_step(whole_step,whole)
    stl_mm_check=validate_stl(whole_mm,whole_volume,"mm")
    stl_m_check=validate_stl(whole_m,whole_volume*1e-9,"m")
    if not step_check["valid"] or step_check["solid_count"]!=1 or step_check["volume_absolute_error_mm3"]>1e-3:
        raise ValueError("Exported whole STEP failed independent import verification")
    if not stl_mm_check["is_volume"] or not stl_m_check["is_volume"] or max(stl_mm_check["relative_volume_error"],stl_m_check["relative_volume_error"])>.005:
        raise ValueError("Exported whole STL failed closed-volume or tessellation checks")
    part_records={}
    for name,shape in parts.items():
        step=part_dir/(name+".step");stl=part_dir/(name+"_mm.stl")
        export_shape(shape,step,"STEP");export_shape(shape,stl,"STL")
        if name=="support_arm":
            local=shape.translate((0,0,-61));xyz=[0,0,61];rpy=[0,0,0]
        elif name=="camera_plate":
            local=shape.translate((0,-args.length_mm,-62)).rotate((0,0,0),(1,0,0),-args.plate_tilt_deg)
            xyz=[0,args.length_mm,62];rpy=[args.plate_tilt_deg,0,0]
        else:local=shape;xyz=[0,0,0];rpy=[0,0,0]
        body_mesh=part_dir/(name+"_body_m.stl");metric_mesh_from_mm(stl,body_mesh,xyz,rpy[0])
        part_records[name]={**properties(shape,args.density_kg_m3),"body_local_properties":properties(local,args.density_kg_m3),"body_frame":{"xyz_mm":xyz,"rpy_deg":rpy},"files":{str(p.relative_to(output)):sha(p) for p in [step,stl,body_mesh]},"step_roundtrip":validate_step(step,shape)}
    a=radians(args.plate_tilt_deg)
    report={"schema":"wrist_camera_bracket_new_design_v1","design_basis":"new design from reference images and measured robot/camera interfaces; not a source STEP reconstruction","prototype_status":"uncertified reinforced printable geometry; material, print process and original robot screw threads/engagement require confirmation","coordinate_convention":{"units":"mm except *_m.stl and explicitly metric transforms/inertias","root":"hand-adapter shoulder center at Y=0","axes":{"X":"transverse across wrist","Y":"wrist axis toward fingers/distal","Z":"radially outward on bracket side"},"theta":"atan2(Z,X), positive theta corresponds to standard right-handed Ry(-theta)"},"parameters":{"length_mm":args.length_mm,"plate_tilt_deg":args.plate_tilt_deg,"density_kg_m3":args.density_kg_m3,"length_range_mm":[20,150],"plate_tilt_range_deg":[0,75]},"parameter_semantics":{"length":"changes only longitudinal U-channel arm length along +Y and moves the distal plate/pivot; wrist base is invariant","plate_tilt":"rebuilds only plate and attached root gussets around pivot X; fixed saddle and straight arm do not rotate","parts":"three disjoint semantic regions of one printed/fabricated solid, not an unfastened three-piece assembly","regenerate_on_parameter_change":True},"plate_frame":{"xyz_mm":[0,args.length_mm,62],"rpy_deg":[args.plate_tilt_deg,0,0],"pivot_axis_in_root":[1,0,0]},"camera_mount":{"relative_to":"plate_frame","contact_point_mm":[0,32,-2],"contact_plane_normal_plate":[0,0,1],"camera_optical_direction_plate":[0,0,-1],"bottom_screw_frame":{"xyz_m":[0,.053,-.01035],"rotation_matrix":[[0,1,0],[0,0,-1],[-1,0,0]]},"camera_geometry_included":False,"camera_back_hole_points_bottom_screw_mm":[[-8.35,-10,21],[-8.35,10,21]],"camera_thread_depth_reference_mm":4,"screw_length_status":"select screw/washer stack from actual hardware; avoid exceeding camera rear thread engagement"},"features":{"wrist_mount_holes":wrist_holes,"existing_adapter_retention_access_reliefs":{"theta_deg":[15,105],"diameter_mm":9,"purpose":"clearance/access, not additional mounting fasteners"},"camera_plate_holes":[{"center_plate_mm":[x,32,0],"axis_plate":[0,0,1],"center_root_mm":plate_point([x,32,0],args.length_mm,args.plate_tilt_deg),"axis_root":[0,-math.sin(a),math.cos(a)],"diameter_mm":3.4} for x in [-10,10]],"arm_axis_root":[0,1,0],"arm_start_mm":[0,0,61],"arm_end_mm":[0,args.length_mm,61],"plate_gusset_yz_root_mm":gusset_yz},"strength_geometry":{"beam_width_mm":24,"web_thickness_mm":6,"rib_width_mm":3,"rib_added_height_mm":8,"rib_end_gap_mm":0,"beam_length_mm":args.length_mm,"review_beam_lengths_mm":[80,150],"density_kg_m3":args.density_kg_m3,"camera_com_plate_local_mm":[0,32,-14],"camera_com_status":"review-only approximate load point, not camera mass-property measurement","plate_width_mm":40,"plate_length_mm":42,"plate_thickness_mm":4,"tilt_range_deg":[0,75],"default_tilt_deg":40,"saddle_inner_radius_mm":30.8,"saddle_outer_radius_mm":35.8,"saddle_y_range_mm":[-10.5,-.5],"saddle_angle_range_deg":[22,158],"root_pedestal_y_range_mm":[-8.5,0],"root_pedestal_top_z_mm":64,"gusset_start_before_pivot_mm":18,"fillets":notes},"whole":properties(whole,args.density_kg_m3),"parts":part_records,"files":{p.name:sha(p) for p in [whole_step,whole_mm,whole_m]},"verification":{"whole_step_roundtrip":step_check,"whole_stl_mm":stl_mm_check,"whole_stl_m":stl_m_check,"sum_component_volume_mm3":part_volume,"component_volume_partition_error_mm3":abs(part_volume-whole.Volume()),"physical_fit_or_strength_certified":False},"generator_sha256":sha(Path(__file__)),"reference_wrist_meshes":{side:sha(ROOT/f"meshes/tron2/wrist_roll_{letter}_Link.STL") for side,letter in [("left","L"),("right","R")]}}
    report["features"]["existing_adapter_retention_access_reliefs"]["diameter_mm"]=9.5
    report["verification"]["component_volume_partition_error_mm3"]=abs(part_volume-whole_volume)
    report["verification"]["plate_partition_fragment_cleanup"]=fragment_cleanup
    report["features"]["saddle_radial_clearance_mm"]=.3
    report["features"]["saddle_fit_status"]="0.3 mm radial design clearance, not a measured print fit; verify seating and original screw engagement on hardware"
    (output/"manifest.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"output":str(output),"parameters":report["parameters"],"plate_frame":report["plate_frame"],"whole":report["whole"],"verification":report["verification"]},indent=2))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--length-mm",type=float,default=80.)
    parser.add_argument("--plate-tilt-deg",type=float,default=40.)
    parser.add_argument("--density-kg-m3",type=float,default=DEFAULT_DENSITY)
    parser.add_argument("--output-directory",type=Path,default=DEFAULT_OUTPUT)
    build(parser.parse_args())


if __name__=="__main__":main()
