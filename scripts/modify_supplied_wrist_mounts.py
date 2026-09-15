#!/usr/bin/env python3
"""V2: change ONLY the wrist-band bores of the supplied left/right STEP solids.

Original solids and upper support/camera plate are preserved. Output coordinates
remain the original CAD coordinates; the inward assembly rotation is separate.
"""
import json
import math
from pathlib import Path
import cadquery as cq
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType
from design_wrist_camera_bracket import properties,sha

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'design/supplied_brackets_inward_v2'
V=cq.Vector
BOOLEAN_TOL_MM=1e-5


def axis(degrees):
    a=math.radians(degrees)
    return V(math.cos(a),0,math.sin(a))


def cylinder(radius,start,length,direction):
    return cq.Solid.makeCylinder(radius,length,V(start*direction.x,-1.4,start*direction.z),direction)


def cylinders(shape,radius):
    result=[]
    for face in shape.Faces():
        a=BRepAdaptor_Surface(face.wrapped)
        if a.GetType()!=GeomAbs_SurfaceType.GeomAbs_Cylinder:continue
        c=a.Cylinder()
        if abs(c.Radius()-radius)>1e-6:continue
        p=np.array([c.Location().X(),c.Location().Y(),c.Location().Z()])
        u=np.array([c.Axis().Direction().X(),c.Axis().Direction().Y(),c.Axis().Direction().Z()])
        if any(np.linalg.norm(np.cross(p-r['p'],u))<1e-6 and np.linalg.norm(np.cross(u,r['u']))<1e-6 for r in result):continue
        result.append({'p':p,'u':u,'radius':radius})
    return result


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    interfaces=json.loads((ROOT/'design/imported_camera_brackets_20260914/interfaces.json').read_text())
    # Nominal band used to fill the old holes. The allowed connection region
    # also includes its small outer blend/lip, below |Z|=35; upper support and
    # camera plate are protected against every Boolean operation.
    band=cq.Solid.makeCylinder(34.5,11.8,V(0,-5.9,0),V(0,1,0)).cut(
        cq.Solid.makeCylinder(30.5,11.8,V(0,-5.9,0),V(0,1,0)))
    allowed=cq.Solid.makeCylinder(40,11.8,V(0,-5.9,0),V(0,1,0)).cut(
        cq.Solid.makeCylinder(30.5,11.8,V(0,-5.9,0),V(0,1,0))).intersect(
        cq.Solid.makeBox(100,100,70,V(-50,-50,-35)))
    all_results={}
    for side,old_angles,new_angles in [('left',[30,150],[60,120]),('right',[210,330],[240,300])]:
        original=interfaces['sides'][side]
        source=Path(original['source']);assert sha(source)==original['source_sha256']
        before=cq.importers.importStep(str(source)).val()
        after=before
        # Reconstruct only the material formerly removed by each old bore and
        # its counterbore. Slight cutter overlap avoids coincident split seams.
        for angle in old_angles:
            patch=band.intersect(cylinder(3.30,20,30,axis(angle)))
            after=after.fuse(patch,tol=BOOLEAN_TOL_MM).clean()
        closed=after
        new_holes=[]
        for angle in new_angles:
            u=axis(angle)
            # Stop at R40: a long radial drill would also hit the unchanged
            # camera plate farther out. Do clear the band's outer lip.
            cut=cylinder(1.65,20,20,u).fuse(cylinder(3.25,34,6,u))
            after=after.cut(cut,tol=BOOLEAN_TOL_MM).clean()
            new_holes.append({'source_angle_deg':angle,'axis_source':list(u.toTuple()),
                'axis_origin_source_mm':[0,-1.4,0],
                'point_at_contact_radius_mm':[30.5*u.x,-1.4,30.5*u.z],
                'clearance_diameter_mm':3.3,'counterbore_diameter_mm':6.5,
                'counterbore_floor_radius_mm':34.,'nominal_material_below_seat_mm':3.5})
        assert after.isValid() and len(after.Solids())==1,'Expected one valid connected bracket'
        added=after.cut(before,tol=BOOLEAN_TOL_MM);removed=before.cut(after,tol=BOOLEAN_TOL_MM)
        outside=added.cut(allowed,tol=BOOLEAN_TOL_MM).Volume()+removed.cut(allowed,tol=BOOLEAN_TOL_MM).Volume()
        assert outside<1e-3,f'Unauthorized change outside wrist connection region: {outside}'
        # Independent protected upper halfspace and exact camera-hole check.
        protected=cq.Solid.makeBox(300,300,150,V(-150,-150,35 if side=='left' else -185))
        protected_change=added.intersect(protected,tol=BOOLEAN_TOL_MM).Volume()+removed.intersect(protected,tol=BOOLEAN_TOL_MM).Volume()
        assert protected_change<1e-3
        holes=cylinders(after,1.65)
        wrist=[h for h in holes if abs(h['u'][1])<.01]
        rear=[h for h in holes if abs(h['u'][1])>.9]
        assert len(wrist)==2 and len(rear)==2,'Expected exactly two wrist and two camera bores'
        rear_errors=[]
        for old in original['rear_cylinders']:
            error=min(np.linalg.norm(np.cross(np.array(old['p'])-h['p'],h['u'])) for h in rear)
            rear_errors.append(float(error))
        assert max(rear_errors)<1e-6
        for angle in old_angles:
            u=axis(angle)
            assert all(after.isInside(V(r*u.x,-1.4,r*u.z),1e-6) for r in [31,32,33,34.2])
        for angle in new_angles:
            u=axis(angle)
            assert all(not after.isInside(V(r*u.x,-1.4,r*u.z),1e-6) for r in [31,32,33,34.2])
        # Full screw-head swept access is reported, never used to cut the
        # preserved plate. Actual fastener/head/tool selections remain unknown.
        access=[]
        for angle in new_angles:
            u=axis(angle)
            # 0.01 mm numerical guard avoids coincident-face Common artifacts;
            # it is not an additional manufacturing allowance.
            near=after.intersect(cylinder(3.24,34.01,4.98,u),tol=BOOLEAN_TOL_MM).Volume()
            far=after.intersect(cylinder(3.25,34.5,80,u),tol=BOOLEAN_TOL_MM).Volume()
            access.append({'source_angle_deg':angle,'d6_48_head_clearance_R34_01_to38_99_overlap_mm3':near,
                           'long_straight_d6_5_tool_sweep_overlap_mm3':far})
            assert near<1e-5,'No material may block the short screw-head envelope'
        step=OUT/f'camera_bracket_{side}_inward_v2.step'
        stl=OUT/f'camera_bracket_{side}_inward_v2_mm.stl'
        cq.exporters.export(after,str(step))
        cq.exporters.export(after,str(stl),tolerance=.015,angularTolerance=.08)
        reimport=cq.importers.importStep(str(step)).val()
        error=abs(reimport.Volume()-after.Volume())
        relative_error=error/after.Volume()
        assert reimport.isValid() and len(reimport.Solids())==1 and relative_error<5e-6, f'STEP roundtrip: valid={reimport.isValid()}, solids={len(reimport.Solids())}, volume error={error}'
        assert sha(source)==original['source_sha256']
        result={'schema':'supplied_bracket_inward_v2','source':str(source),'source_sha256':sha(source),
                'source_geometry_modified':True,'original_file_modified':False,'camera_separate':True,
                'geometry_sha256':sha(step),'step':str(step.relative_to(ROOT)),'stl_mm':str(stl.relative_to(ROOT)),
                'modification':'Fill original wrist holes; relocate Ø3.3 bores and Ø6.5 seats to 60-degree pair; clear outer wrist-band lip only. Protected upper support begins at |Z|=35 mm.',
                'old_source_angles_deg':old_angles,'new_source_angles_deg':new_angles,
                'wrist_holes':new_holes,'whole':properties(after,2700),
                'material_status':'Assumed solid aluminum 2700 kg/m3; not identified or strength certified',
                'verification':{'added_volume_mm3':added.Volume(),'removed_volume_mm3':removed.Volume(),
                    'change_outside_wrist_connection_region_mm3':outside,'upper_support_plate_change_mm3':protected_change,
                    'rear_hole_axis_errors_mm':rear_errors,'step_roundtrip_volume_error_mm3':error,
                    'step_roundtrip_relative_volume_error':relative_error,'boolean_tolerance_mm':BOOLEAN_TOL_MM,
                    'valid_connected_solid':True,'old_holes_solid_probe_pass':True,'new_holes_open_probe_pass':True,
                    'fastener_access':access,'strength_certified':False},
                'generator_sha256':sha(Path(__file__))}
        (OUT/f'{side}_cad_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
        all_results[side]=result['verification']
    print(json.dumps(all_results,indent=2))


if __name__=='__main__':main()
