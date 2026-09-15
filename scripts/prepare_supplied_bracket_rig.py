#!/usr/bin/env python3
"""Prepare a separate, reproducible STEP-based assembly without changing defaults."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from build_camera_rig import ROOT, sha

OUT=ROOT/'design/imported_camera_brackets_20260914'


def main():
    interfaces=json.loads((OUT/'interfaces.json').read_text())
    cfg=json.loads((ROOT/'config/camera_rig.json').read_text())
    cfg.update(bracket_geometry='supplied_step_20260914', geometry_status='supplied_STEP_registered_candidate',
               parameter_editable=False, hand_mount_overrides={})
    cfg['notes']=[
        'Original STEP solids are unchanged; imported brackets are fixed geometry, not the earlier adjustable photo-reference design.',
        'Left +90 / right -90 degrees about wrist-frame +Z, viewed from arm toward fingertips; adapter and hand rotate together.',
        'Small wrist-hole pairs 30/150 (left) and 210/330 (right), not the earlier 330/30 side pair.',
        'Camera is a separate official mesh, located by rear M3 pair and contact plane.',
        'Nominal geometry only: tolerances, actual fasteners, physical fit and strength remain unverified.'
    ]
    assembly=json.loads((ROOT/'config/assembly_v2.json').read_text())
    for side,row in interfaces['sides'].items():
        mesh=trimesh.load_mesh(ROOT/row['mesh_mm'],process=True)
        mesh.apply_scale(.001)
        dest=ROOT/f'meshes/wrist_camera_brackets/supplied_20260914/{side}_supplied_m.stl'
        dest.parent.mkdir(parents=True,exist_ok=True)
        mesh.export(dest)
        mesh.density=2700 # Explicit assumed aluminum, not a material identification.
        manifest=dict(source=row['source'],source_sha256=row['source_sha256'],
                      source_geometry_modified=False,mesh_m=str(dest.relative_to(ROOT)),mesh_sha256=sha(dest),
                      camera_separate=True,material_status='Assumed solid aluminum 2700 kg/m3; not measured',
                      whole=dict(center_of_mass_mm=row['center_of_mass_mm'],mass_kg=float(mesh.mass),
                                 inertia_at_com_kg_m2=mesh.moment_inertia.tolist(),volume_mm3=row['volume_mm3']))
        manifest_path=OUT/f'{side}_manifest.json'
        manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
        old=assembly[side]['wrist_to_adapter']
        Rwa=Rotation.from_euler('xyz',old['rpy_rad'])
        registration=row['wrist_to_source_in_old_adapter']
        Rsrc=Rotation.from_matrix(registration['rotation'])
        psrc=np.array(registration['xyz_mm'])*.001
        cfg['wrists'][side].update(
            mount_xyz_m=(np.array(old['xyz_m'])+Rwa.apply(psrc)).tolist(),
            mount_rpy_rad=(Rwa*Rsrc).as_euler('xyz').tolist(),
            extension_axis=[0,0,1], extension_origin_m=[0,0,0], length_mm=0,
            plate_zero_rpy_rad=[0,0,0],plate_tilt_axis=[1,0,0],plate_tilt_deg=0,
            camera_bottom_xyz_m=(np.array(row['camera_bottom_xyz_mm'])*.001).tolist(),
            camera_bottom_rpy_rad=Rotation.from_matrix(row['camera_bottom_rotation']).as_euler('xyz').tolist(),
            bracket_manifest=str(manifest_path.relative_to(ROOT)),
            mount_evidence='Source saddle Y in [-5.9,5.9] maps to original adapter Y in [-11.8,0]; hole axis Y=-1.4 maps to -4.5 mm.')
        degrees=90 if side=='left' else -90
        cfg['hand_mount_overrides'][side]=dict(rpy_rad=(Rotation.from_euler('z',degrees,degrees=True)*Rwa).as_euler('xyz').tolist(),rotation_about_wrist_z_deg=degrees)
    output=ROOT/'config/camera_rig_supplied_step_20260914.json'
    output.write_text(json.dumps(cfg,indent=2,ensure_ascii=False)+'\n')
    print(output)


if __name__=='__main__':main()
