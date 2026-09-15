#!/usr/bin/env python3
"""Check fixed interfaces and render the requested supplied-STEP assembly."""
import json
import sys
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from PIL import Image
import trimesh
import manifold3d
from camera_rig import CameraRig, ROOT

OUT=ROOT/'reports/cameras/supplied_step_20260914'


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rig=CameraRig(config_path=ROOT/'config/camera_rig_supplied_step_20260914.json')
    interfaces=json.loads((ROOT/'design/imported_camera_brackets_20260914/interfaces.json').read_text())
    old=json.loads((ROOT/'config/assembly.json').read_text())
    sys.path.insert(0,str(ROOT/'design/revision2/analysis'))
    from extract_wrist_features import wrist_mesh, cylinder_candidate
    from check_v2_fit import closed_paths, section_solid, fit_bores
    theta=np.deg2rad(np.linspace(22.766,37.234,100))
    key_sector=manifold3d.CrossSection([np.vstack([[0,0],40*np.column_stack([np.cos(theta),np.sin(theta)]),[0,0]])])
    report={'source_comparison':interfaces['comparison'],'sides':{},'nq':rig.model.nq,'ncam':rig.model.ncam,
            'urdf':str(rig.urdf_path.relative_to(ROOT)), 'scene':str(rig.scene_path.relative_to(ROOT)),
            'physical_fit_and_strength_certified':False}
    for side,letter in [('left','L'),('right','R')]:
        info=interfaces['sides'][side]
        wrist=rig.body_transform(f'wrist_roll_{letter}_Link')
        Rwa=Rotation.from_euler('xyz',old[side]['wrist_to_adapter']['rpy_rad']).as_matrix()
        A=wrist.copy();A[:3,:3]=wrist[:3,:3]@Rwa
        A[:3,3]=wrist[:3,3]+wrist[:3,:3]@np.array(old[side]['wrist_to_adapter']['xyz_m'])
        B=np.linalg.inv(A)@rig.body_transform(f'{side}_wrist_camera_mount_frame')
        camera=rig.body_transform(f'{side}_wrist_camera_bottom_screw_frame')
        plate=rig.body_transform(f'{side}_wrist_camera_mount_frame')
        rear_actual=np.array([camera[:3,3]+camera[:3,:3]@(np.array([-8.35,y,21])*.001) for y in [-10,10]])
        rear_expected=np.array([plate[:3,3]+plate[:3,:3]@(np.array(p)*.001) for p in info['camera_hole_centers_mm']])
        distances=np.linalg.norm(rear_actual[:,None]-rear_expected[None,:],axis=2)
        hole_error=float(max(distances.min(axis=0).max(),distances.min(axis=1).max())*1000)
        wm=wrist_mesh(side)
        checks=[]
        for h in info['wrist_cylinders']:
            p=B[:3,:3]@np.array(h['p'])+B[:3,3]*1000
            u=B[:3,:3]@np.array(h['u'])
            # Choose the occupied radial ray by the source side's Z sign.
            u*=1 if (u[2]>0)==(side=='left') else -1
            angle=float(np.degrees(np.arctan2(u[2],u[0]))%360)
            target=30 if abs(angle-30)<1 else 150 if abs(angle-150)<1 else 210 if abs(angle-210)<1 else 330
            assert abs(angle-target)<.001, 'Unrecognized wrist-hole ray'
            fit=cylinder_candidate(wm,target)
            tangent=np.array([-np.sin(np.deg2rad(target)),0,np.cos(np.deg2rad(target))])
            check=dict(target_angle_deg=target,bracket_angle_deg=angle,
                       axial_error_mm=float(p[1]-fit['axis_y_mm']),
                       tangent_error_mm=float(p@tangent-fit['axis_tangent_offset_mm']),
                       wrist_hole_diameter_mm=fit['cylinder_diameter_mm'],bracket_diameter_mm=h['radius']*2)
            checks.append(check)
        adapter=trimesh.load_mesh(ROOT/f'meshes/adapter_v2/{side}/adapter_{side}_v2_visual.stl',process=True)
        adapter.apply_scale(1000)
        baseline=adapter.copy()
        requested=np.linalg.inv(A)@rig.body_transform(side+'_adapter_link')
        adapter_world=rig.body_transform(side+'_adapter_link')
        actual_wrist_rotation=wrist[:3,:3].T@adapter_world[:3,:3]
        expected=Rotation.from_euler('z',90 if side=='left' else -90,degrees=True).as_matrix()@Rwa
        rotation_error_deg=float(np.degrees(Rotation.from_matrix(actual_wrist_rotation@expected.T).magnitude()))
        assert rotation_error_deg<1e-6
        assert np.linalg.norm(requested[:3,3])<1e-9, 'Flange origin must not translate'
        adapter.vertices=adapter.vertices@requested[:3,:3].T+requested[:3,3]*1000
        bracket=trimesh.load_mesh(ROOT/info['mesh_mm'],process=True)
        bracket.vertices=bracket.vertices@B[:3,:3].T+B[:3,3]*1000
        sections=[]
        for axial in [-.01,-.1,-.5,-1,-2,-3,-4.5,-6,-8,-10]:
            w=section_solid(closed_paths(wm,axial))
            a=section_solid(closed_paths(adapter,axial))
            old_a=section_solid(closed_paths(baseline,axial))
            b=section_solid(closed_paths(bracket,axial))
            sections.append(dict(y_mm=axial,adapter_wrist_overlap_mm2=(a^w).area(),
                                 baseline_adapter_wrist_overlap_mm2=(old_a^w).area(),
                                 adapter_key_overlap_mm2=((a^w)^key_sector).area(),
                                 bracket_wrist_overlap_mm2=(b^w).area(),bracket_adapter_overlap_mm2=(b^a).area()))
        bores=fit_bores(adapter)
        retention=[{k:h[k] for k in ['target_theta_deg','shaft_direction_theta_error_deg',
                    'max_axis_y_error_over_sampled_shaft_mm','max_tangent_offset_over_sampled_shaft_mm','mean_fitted_diameter_mm']} for h in bores]
        hand=rig.body_transform(side+'_hand_base_link')
        optical=rig.body_transform(side+'_wrist_camera_color_optical_frame')
        optical_in_hand=np.linalg.inv(hand)@optical
        palm=rig.body_transform(side+'_palm')[:3,3]
        v=optical[:3,3]-palm
        cos=float(np.dot(hand[:3,0],v)/np.linalg.norm(v))
        palm_in_camera=optical[:3,:3].T@(palm-optical[:3,3])
        spec=next(s for s in rig.camera_specs if s['name']==side+'_wrist')
        K=np.array(spec['color_K']);pixel=K@palm_in_camera;pixel=pixel[:2]/pixel[2]
        item=dict(wrist_hole_fits=checks,rear_hole_max_error_mm=hole_error,
                  requested_mount_rotation_error_deg=rotation_error_deg,
                  axial_section_overlap=sections,rotated_adapter_retention_bores=retention,
                  optical_origin_in_hand_mm=(optical_in_hand[:3,3]*1000).tolist(),
                  palm_front_side=cos>0,palm_normal_to_camera_deg=float(np.degrees(np.arccos(np.clip(cos,-1,1)))),
                  palm_center_optical_m=palm_in_camera.tolist(),palm_center_pixel=pixel.tolist(),
                  palm_center_in_frame=bool(palm_in_camera[2]>0 and 0<=pixel[0]<rig.width and 0<=pixel[1]<rig.height))
        report['sides'][side]=item
        frame=rig.capture_one(side+'_wrist')
        Image.fromarray(frame['rgb']).save(OUT/f'{side}_wrist_rgb.png')
        np.save(OUT/f'{side}_wrist_depth_m.npy',frame['depth'])
    # Side-by-side real-source CAD inspection, plus robot closeups.
    renderer=mujoco.Renderer(rig.model,width=1200,height=900)
    for side in ['left','right']:
        center=rig.body_transform(side+'_palm')[:3,3]
        for i,azimuth in enumerate([0,90,180]):
            cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam)
            cam.lookat[:]=center;cam.distance=.47;cam.azimuth=azimuth;cam.elevation=-20
            renderer.update_scene(rig.data,camera=cam,scene_option=rig.opt)
            Image.fromarray(renderer.render()).save(OUT/f'{side}_assembly_{i}.png')
    renderer.close();rig.close()
    report['legacy_120deg_pair_alignment_pass']=all(
        s['rear_hole_max_error_mm']<.01 and all(abs(h['axial_error_mm'])<.01 and abs(h['tangent_error_mm'])<.01 for h in s['wrist_hole_fits'])
        for s in report['sides'].values())
    report['requested_rotation_and_nominal_hole_alignment_pass']=report['legacy_120deg_pair_alignment_pass'] and all(
        {h['target_angle_deg'] for h in s['wrist_hole_fits']}=={30,330} for s in report['sides'].values())
    report['corrected_user_target']='Both wrists: the adjacent small-hole pair 330/30 is fixed. The previous 120-degree pair selection is not the requested mounting.'
    # Passing these necessary geometric conditions still requires an image
    # occlusion review; never auto-certify a usable view from projection alone.
    report['usable_palm_view_pass']=None if all(s['palm_center_in_frame'] and s['palm_normal_to_camera_deg']<70 for s in report['sides'].values()) else False
    report['view_limitation']='Rendered RGB is strongly occluded by the thumb/hand. Palm-normal viewing angle is about 86.6 degrees; nominal palm center is outside the vertical image boundary. Do not claim a usable palm-facing view.'
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
