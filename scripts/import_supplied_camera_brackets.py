#!/usr/bin/env python3
"""Export the supplied, unmodified STEP solids and their measured interfaces (mm)."""
import hashlib
import json
from pathlib import Path
import cadquery as cq
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'design/imported_camera_brackets_20260914'


def arr(v):
    return np.array([v.X(), v.Y(), v.Z()])


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    result = {'units': 'mm', 'source_geometry_modified': False, 'sides': {}}
    shapes = {}
    for side, suffix in [('left', 'l'), ('right', 'r')]:
        source = ROOT.parents[1] / f'Tron2灵巧手相机支架-{suffix}.stp'
        data = source.read_bytes()
        assert data.startswith(b'ISO-10303-21;')
        shape = cq.importers.importStep(str(source)).val()
        assert shape.isValid() and len(shape.Solids()) == 1
        shapes[side] = shape
        cq.exporters.export(shape, str(OUT / f'{side}_source_mm.stl'), tolerance=.015, angularTolerance=.08)
        cylinders, planes = [], []
        for i, face in enumerate(shape.Faces()):
            s = BRepAdaptor_Surface(face.wrapped)
            if s.GetType() == GeomAbs_SurfaceType.GeomAbs_Cylinder:
                c = s.Cylinder()
                entry = dict(index=i, radius=c.Radius(), p=arr(c.Location()), u=arr(c.Axis().Direction()))
                if not any(abs(t['radius']-entry['radius']) < 1e-6 and
                           np.linalg.norm(np.cross(t['u'], entry['u'])) < 1e-6 and
                           np.linalg.norm(np.cross(t['p']-entry['p'],entry['u'])) < 1e-6 for t in cylinders):
                    cylinders.append(entry)
            if s.GetType() == GeomAbs_SurfaceType.GeomAbs_Plane and face.Area() > 1000:
                planes.append(dict(index=i, p=arr(s.Plane().Location()), n=np.array(face.normalAt().toTuple()), area=face.Area()))
        rear = [c for c in cylinders if abs(c['radius']-1.65) < 1e-5 and abs(c['u'][1]) > .9]
        wrist = [c for c in cylinders if abs(c['radius']-1.65) < 1e-5 and abs(c['u'][1]) < .01]
        assert len(rear) == 2 and len(wrist) == 2
        # The rear of the camera contacts the negative-Y sheet face. Wrist
        # registration reverses source Y; the optical axis then points distally.
        plane = next(p for p in planes if p['n'][1] < -.9)
        points = [h['p'] + h['u'] * (np.dot(plane['p']-h['p'], plane['n']) / np.dot(h['u'], plane['n'])) for h in rear]
        center = np.mean(points, axis=0)
        u = (points[0]-points[1]) / np.linalg.norm(points[0]-points[1])
        x = plane['n']
        y = -u
        z = np.cross(x, y)
        R = np.column_stack([x, y, z])
        assert np.allclose(R.T @ R, np.eye(3)) and np.linalg.det(R) > .999
        # Official D405 rear-hole midpoint, in bottom_screw coordinates.
        bottom = center - R @ np.array([-8.35, 0, 21.])
        bounds = shape.BoundingBox()
        result['sides'][side] = dict(
            source=str(source), source_sha256=hashlib.sha256(data).hexdigest(),
            mesh_mm=str((OUT / f'{side}_source_mm.stl').relative_to(ROOT)),
            valid=True, solids=1, volume_mm3=shape.Volume(),
            bounds_mm=[[bounds.xmin,bounds.ymin,bounds.zmin],[bounds.xmax,bounds.ymax,bounds.zmax]],
            center_of_mass_mm=list(shape.Center().toTuple()),
            wrist_cylinders=wrist, rear_cylinders=rear,
            camera_contact_plane=plane, camera_hole_centers_mm=points,
            camera_hole_spacing_mm=float(np.linalg.norm(points[0]-points[1])),
            camera_bottom_xyz_mm=bottom, camera_bottom_rotation=R,
            camera_optical_direction_source=x,
            wrist_to_source_in_old_adapter=dict(rotation=np.diag([-1.,-1.,1.]), xyz_mm=[0,-5.9,0]),
        )
    mirror = shapes['right'].mirror('XY')
    common = shapes['left'].intersect(mirror).Volume()
    result['comparison'] = dict(
        reflection_plane='source XY (Z sign reversal)', common_volume_mm3=common,
        symmetric_difference_volume_mm3=shapes['left'].Volume()+shapes['right'].Volume()-2*common,
        interpretation='Same envelope and nominal length, mirrored main layout with small local differences; not a long/short pair.')
    def convert(v):
        if isinstance(v,np.ndarray): return v.tolist()
        if isinstance(v,np.generic): return v.item()
        raise TypeError(type(v))
    (OUT / 'interfaces.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,default=convert)+'\n')
    print(json.dumps(result['comparison'],indent=2))


if __name__ == '__main__':
    main()
