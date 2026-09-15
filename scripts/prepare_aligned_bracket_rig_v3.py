#!/usr/bin/env python3
"""Install V3 CAD and camera transforms; preserve wrist mounting holes and all hand poses."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from build_camera_rig import ROOT,sha


def main():
    base=ROOT/'config/camera_rig_supplied_step_inward_v2.json'
    old=json.loads(base.read_text());cfg=json.loads(base.read_text())
    cfg.update(bracket_geometry='supplied_step_aligned_v3',geometry_status='level_centered_support_short_saddle',
               base_config=str(base.relative_to(ROOT)),base_config_sha256=sha(base),parameter_editable=False)
    cfg['notes']=[
        'Both wrist hole axes and bracket root transforms remain identical to inward V2; hands and adapters do not move.',
        'Undo the original 12-degree lateral upper-part skew; camera horizontal edges are parallel to the hand/adapter transverse direction.',
        'Keep original 10-degree fore-aft pitch. Upper support and camera move radially outward 10 mm for driver clearance.',
        'Replace the long wrist connection arc with an 88-degree reinforced saddle, exposing the previously covered end retention screw.',
        'Camera remains a separate original model, attached through the unchanged 20 mm rear hole spacing.',
        'CAD tests use explicit driver and camera envelopes, not a hardware fit or strength certification.'
    ]
    directory=ROOT/'design/supplied_brackets_aligned_v3'
    for side in ['left','right']:
        cad_path=directory/f'{side}_cad_manifest.json';cad=json.loads(cad_path.read_text())
        mesh=trimesh.load_mesh(ROOT/cad['stl_mm'],process=True)
        assert mesh.is_watertight and mesh.is_volume
        error=abs(mesh.volume/cad['whole']['volume_mm3']-1);assert error<.005
        mesh.apply_scale(.001)
        output=ROOT/f'meshes/wrist_camera_brackets/supplied_aligned_v3/{side}_aligned_v3_m.stl'
        output.parent.mkdir(parents=True,exist_ok=True);mesh.export(output)
        asset={**cad,'cad_manifest':str(cad_path.relative_to(ROOT)),'cad_manifest_sha256':sha(cad_path),
               'mesh_m':str(output.relative_to(ROOT)),'mesh_sha256':sha(output),'stl_watertight':True,
               'stl_volume_relative_error':float(error)}
        manifest=directory/f'{side}_asset_manifest.json';manifest.write_text(json.dumps(asset,indent=2)+'\n')
        cfg['wrists'][side].update(bracket_manifest=str(manifest.relative_to(ROOT)),
            camera_bottom_xyz_m=(np.array(cad['camera_bottom_xyz_mm'])*.001).tolist(),
            camera_bottom_rpy_rad=Rotation.from_matrix(cad['camera_bottom_rotation']).as_euler('xyz').tolist(),
            mount_evidence='Wrist mounting origin, orientation and holes unchanged from V2; only upper support placement and saddle extent revised.')
        for key in ['mount_xyz_m','mount_rpy_rad']:
            assert cfg['wrists'][side][key]==old['wrists'][side][key]
    assert cfg['hand_mount_overrides']==old['hand_mount_overrides'] and cfg['pose']==old['pose']
    output=ROOT/'config/camera_rig_supplied_step_aligned_v3.json'
    output.write_text(json.dumps(cfg,indent=2,ensure_ascii=False)+'\n');print(output)


if __name__=='__main__':main()
