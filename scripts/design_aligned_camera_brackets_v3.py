#!/usr/bin/env python3
"""V3: fixed wrist holes, centered/level upper support, shorter saddle and tool access."""
import argparse
import json
import math
from pathlib import Path
import cadquery as cq
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf
from design_wrist_camera_bracket import box,properties,sha
from modify_supplied_wrist_mounts import cylinders

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'design/supplied_brackets_aligned_v3'
V=cq.Vector
TOL=1e-5


def ry(angle):
    a=math.radians(angle);c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def radial(degrees):
    a=math.radians(degrees);return V(math.cos(a),0,math.sin(a))


def tool(radius,start,length,u):
    return cq.Solid.makeCylinder(radius,length,V(start*u.x,-1.4,start*u.z),u)


def shape_transform(shape,R,p):
    assert np.allclose(R.T@R,np.eye(3),atol=1e-10) and np.linalg.det(R)>.999999
    trsf=gp_Trsf()
    trsf.SetValues(*[float(v) for i in range(3) for v in [*R[i],p[i]]])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped,trsf,True).Shape())


def common_volume(a,b):
    if not a.Solids() or not b.Solids():return 0.
    op=BRepAlgoAPI_Common(a.wrapped,b.wrapped);op.SetFuzzyValue(TOL);op.Build()
    if not op.IsDone():raise ValueError('CAD intersection did not complete')
    return 0. if op.Shape().IsNull() else cq.Shape.cast(op.Shape()).Volume()


def build(lift):
    OUT.mkdir(parents=True,exist_ok=True)
    original=json.loads((ROOT/'design/imported_camera_brackets_20260914/interfaces.json').read_text())
    # Normalized construction frame has +Z radially outward on both sides.
    # Right output is reflected back into its own supplied CAD coordinates.
    inner,outer,lo,hi=30.5,35.5,46.,134.
    pt=lambda r,a:(r*math.cos(math.radians(a)),r*math.sin(math.radians(a)))
    saddle=(cq.Workplane('XZ',origin=(0,5.9,0)).moveTo(*pt(inner,lo))
        .threePointArc(pt(inner,90),pt(inner,hi)).lineTo(*pt(outer,hi))
        .threePointArc(pt(outer,90),pt(outer,lo)).close().extrude(11.8).val())
    saddle=cq.Workplane('XY').newObject([saddle]).edges('|Y').fillet(.7).val()
    # Broad solid root bridge supports the preserved 25 mm wide upper neck.
    # No large access bore is cut through this load path.
    bridge=box(-12.5,12.5,-.8,5.9,32.5,36+lift)
    bridge=cq.Workplane('XY').newObject([bridge]).edges('|Y').fillet(.8).val()
    base=saddle.fuse(bridge,tol=TOL).clean()
    bore_features=[]
    for a in [60.,120.]:
        u=radial(a)
        base=base.cut(tool(1.65,20,24,u),tool(3.25,34,8,u),tol=TOL).clean()
        bore_features.append({'source_angle_deg':a,'axis_source':list(u.toTuple()),
            'point_at_contact_radius_mm':[30.5*u.x,-1.4,30.5*u.z],
            'clearance_diameter_mm':3.3,'counterbore_diameter_mm':6.5,'counterbore_floor_radius_mm':34.})
    summaries={}
    for side,angle in [('left',12.),('right',-12.)]:
        row=original['sides'][side];path=Path(row['source'])
        assert sha(path)==row['source_sha256']
        source=cq.importers.importStep(str(path)).val()
        U=ry(angle);shift=np.array([0.,0.,lift if side=='left' else -lift])
        aligned=source.rotate((0,0,0),(0,1,0),angle)
        if side=='right':aligned=aligned.mirror('XY')
        upper=aligned.intersect(box(-100,100,-100,100,35,160)).translate((0,0,lift))
        shape=base.fuse(upper,tol=TOL).clean()
        assert shape.isValid() and len(shape.Solids())==1,'Bracket must be one valid connected solid'
        # Preserve the original camera plate and upper neck after the explicit
        # rigid leveling transform. Root changes end below this guard plane.
        guard=box(-100,100,-100,100,38+lift,180)
        delta=common_volume(shape.cut(upper,tol=TOL),guard)+common_volume(upper.cut(shape,tol=TOL),guard)
        assert delta<1e-3
        Rcam=U@np.array(row['camera_bottom_rotation'])
        pcam=U@np.array(row['camera_bottom_xyz_mm'])+shift
        centers=(np.array(row['camera_hole_centers_mm'])@U.T)+shift
        # Conservative camera body envelope extends beyond nominal casing;
        # camera remains a separate simulation mesh and is never exported here.
        # Actual visual-mesh bounds in the D405 bottom frame are
        # [-8.35,-21.09,0] .. [14.65,21,42] mm. The contact-face 0.01 mm
        # guard excludes intended rear-face seating; the access box is larger.
        camera_box=shape_transform(box(-8.34,14.651,-21.091,21.001,-.001,42.001),Rcam,pcam)
        access_box=shape_transform(box(-8.36,18,-21.2,21.2,-.2,42.2),Rcam,pcam)
        if side=='right':
            camera_box=camera_box.mirror('XY');access_box=access_box.mirror('XY')
        camera_overlap=common_volume(shape,camera_box)
        inherited_overlap=common_volume(upper,camera_box)
        new_root_overlap=common_volume(shape.cut(upper,tol=TOL),camera_box)
        # The rectangular housing envelope includes empty rounded corners.
        # Preserve and report its inherited overlap with the source neck;
        # reject any NEW root collision rather than claiming an exact-case test.
        assert new_root_overlap<.02 and abs(camera_overlap-inherited_overlap)<.02
        access=[]
        for a in [60.,120.]:
            shaft=tool(3.25,35.6,80,radial(a))
            b_hit=common_volume(shape,shaft)
            c_hit=common_volume(access_box,shaft)
            assert max(b_hit,c_hit)<1e-3,f'Blocked tool at {a}: bracket={b_hit}, camera={c_hit}; increase lift'
            access.append({'source_normalized_angle_deg':a,'tool_diameter_mm':6.5,
                           'bracket_intersection_mm3':b_hit,'camera_envelope_intersection_mm3':c_hit})
        # In the assembled original adapter frame the free retention screws
        # are 105/195/285; the intentionally covered central 15-degree screw
        # is not claimed accessible. Left A=90-C; right A=C-90.
        exposed=[]
        for a in [105.,195.,285.]:
            c=90-a if side=='left' else a+90
            clearance=tool(4.5,28,90,radial(c))
            hits=common_volume(shape,clearance)+common_volume(access_box,clearance)
            assert hits<1e-3,f'Retention screw {a} remains covered'
            exposed.append({'adapter_angle_deg':a,'clear_diameter_mm':9.,'intersection_mm3':hits})
        # Record in the side's native CAD frame, matching its unchanged URDF mount.
        raw=shape if side=='left' else shape.mirror('XY')
        step=OUT/f'camera_bracket_{side}_aligned_v3.step'
        stl=OUT/f'camera_bracket_{side}_aligned_v3_mm.stl'
        cq.exporters.export(raw,str(step));cq.exporters.export(raw,str(stl),tolerance=.015,angularTolerance=.08)
        check=cq.importers.importStep(str(step)).val()
        assert check.isValid() and len(check.Solids())==1
        # Use converged volume integration on both sides of STEP roundtrip;
        # default quadrature can change with exported curved-face partitioning.
        v_error=abs(check.Volume(1e-9)/raw.Volume(1e-9)-1)
        assert v_error<5e-6,f'STEP roundtrip relative volume error={v_error}'
        bores=cylinders(check,1.65)
        wrist=[h for h in bores if abs(h['u'][1])<.01]
        rear=[h for h in bores if abs(h['u'][1])>.9]
        assert len(wrist)==2 and len(rear)==2
        rear_errors=[min(np.linalg.norm(np.cross(p-h['p'],h['u'])) for h in rear) for p in centers]
        assert max(rear_errors)<1e-3
        hole_features=json.loads(json.dumps(bore_features))
        if side=='right':
            for h in hole_features:
                h['source_angle_deg']=(-h['source_angle_deg'])%360
                h['axis_source'][2]*=-1;h['point_at_contact_radius_mm'][2]*=-1
        horizontal_error=float(np.degrees(np.arccos(np.clip(abs(Rcam[:,1]@np.array([1.,0,0])),0,1))))
        assert horizontal_error<1e-5
        report={'schema':'supplied_bracket_aligned_v3','source':str(path),'source_sha256':sha(path),
            'original_file_modified':False,'source_geometry_modified':True,'camera_separate':True,
            'step':str(step.relative_to(ROOT)),'geometry_sha256':sha(step),'stl_mm':str(stl.relative_to(ROOT)),
            'parameters':{'upper_leveling_rotation_about_source_y_deg':angle,'radial_lift_mm':lift,
                'normalized_saddle_angle_range_deg':[lo,hi],'normalized_saddle_span_deg':hi-lo,
                'saddle_inner_radius_mm':inner,'saddle_outer_radius_mm':outer,'root_bridge_width_mm':25.,
                'root_bridge_axial_depth_mm':6.7,'camera_fore_aft_pitch_preserved_deg':10.},
            'wrist_holes':hole_features,'camera_hole_centers_mm':centers.tolist(),
            'camera_bottom_xyz_mm':pcam.tolist(),'camera_bottom_rotation':Rcam.tolist(),
            'whole':properties(raw,2700),'material_status':'Assumed solid aluminum density only; strength not certified',
            'verification':{'camera_plate_shape_change_after_rigid_transform_mm3':delta,
                'horizontal_edge_error_deg':horizontal_error,'rear_hole_axis_error_mm':list(map(float,rear_errors)),
                'camera_envelope_bracket_intersection_mm3':camera_overlap,
                'inherited_upper_camera_envelope_intersection_mm3':inherited_overlap,
                'new_root_camera_envelope_intersection_mm3':new_root_overlap,'small_screw_tool_access':access,
                'free_retention_screw_access':exposed,'step_roundtrip_relative_volume_error':v_error,
                'valid_connected_solid':True,'physical_strength_certified':False},'generator_sha256':sha(Path(__file__))}
        (OUT/f'{side}_cad_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
        summaries[side]=report['verification']
    print(json.dumps(summaries,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--radial-lift-mm',type=float,default=10.)
    args=p.parse_args();assert 0<=args.radial_lift_mm<=25
    build(args.radial_lift_mm)
