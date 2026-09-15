#!/usr/bin/env python3
"""Use wrist-only revised STEP assets and rotate brackets inward, not hands."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from build_camera_rig import ROOT,sha


def main():
    base_path=ROOT/'config/camera_rig_supplied_step_20260914.json'
    cfg=json.loads(base_path.read_text())
    base_digest=sha(base_path)
    frozen_hands=json.dumps(cfg['hand_mount_overrides'],sort_keys=True)
    frozen_pose=json.dumps(cfg['pose'],sort_keys=True)
    interfaces=json.loads((ROOT/'design/imported_camera_brackets_20260914/interfaces.json').read_text())
    assembly=json.loads((ROOT/'config/assembly_v2.json').read_text())
    cfg.update(bracket_geometry='supplied_step_inward_v2',geometry_status='wrist_band_only_revision_inward_small_holes',
               base_config=str(base_path.relative_to(ROOT)),base_config_sha256=base_digest,
               parameter_editable=False)
    cfg['notes']=[
        'User-approved wrist-band-only CAD revision: close old bores and relocate Ø3.3/Ø6.5 bores to the robot fixed small-hole pair.',
        'Both target wrist holes remain at original adapter theta 330/30 degrees and Y=-4.5 mm.',
        'Only camera bracket assemblies rotate inward. Hand and adapter mounts are copied unchanged from the previous +90/-90 assembly.',
        'Original upper support/neck and camera plate preserved outside the wrist connection region; camera and rear mounting holes unchanged.',
        'Do not interpret inertia assumptions or nominal hole alignment as physical fit or strength certification.'
    ]
    for side in ['left','right']:
        directory=ROOT/'design/supplied_brackets_inward_v2'
        cad_path=directory/f'{side}_cad_manifest.json'
        cad=json.loads(cad_path.read_text())
        mesh=trimesh.load_mesh(ROOT/cad['stl_mm'],process=True)
        assert mesh.is_watertight and mesh.is_volume,'CAD STL must be closed and consistently wound'
        relative_error=abs(mesh.volume/cad['whole']['volume_mm3']-1)
        assert relative_error<.005
        mesh.apply_scale(.001)
        path=ROOT/f'meshes/wrist_camera_brackets/supplied_inward_v2/{side}_inward_v2_m.stl'
        path.parent.mkdir(parents=True,exist_ok=True);mesh.export(path)
        manifest={**cad,'cad_manifest':str(cad_path.relative_to(ROOT)),'cad_manifest_sha256':sha(cad_path),
                  'mesh_m':str(path.relative_to(ROOT)),'mesh_sha256':sha(path),
                  'stl_volume_relative_error':float(relative_error),'stl_watertight':True}
        manifest_path=directory/f'{side}_asset_manifest.json'
        manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
        Rwa=Rotation.from_euler('xyz',assembly[side]['wrist_to_adapter']['rpy_rad'])
        old_registration=interfaces['sides'][side]['wrist_to_source_in_old_adapter']
        inward=90 if side=='left' else -90
        Rnew=Rotation.from_euler('y',inward,degrees=True)*Rotation.from_matrix(old_registration['rotation'])
        cfg['wrists'][side].update(
            mount_rpy_rad=(Rwa*Rnew).as_euler('xyz').tolist(),
            bracket_manifest=str(manifest_path.relative_to(ROOT)),inward_rotation_in_original_adapter_y_deg=inward,
            mount_evidence='Revised CAD local 60/120 (left), 240/300 (right) map to fixed robot small holes 330/30. Wrist axis Y=-4.5 mm unchanged.')
    assert frozen_hands==json.dumps(cfg['hand_mount_overrides'],sort_keys=True)
    assert frozen_pose==json.dumps(cfg['pose'],sort_keys=True)
    assert sha(base_path)==base_digest
    path=ROOT/'config/camera_rig_supplied_step_inward_v2.json'
    path.write_text(json.dumps(cfg,indent=2,ensure_ascii=False)+'\n')
    print(path)


if __name__=='__main__':main()
