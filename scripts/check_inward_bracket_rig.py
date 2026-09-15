#!/usr/bin/env python3
"""Validate the corrected fixed-hole assembly and render CAD/robot/RGB evidence."""
import json
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import mujoco
from PIL import Image
from scipy.spatial.transform import Rotation
from camera_rig import CameraRig,ROOT,sha

OUT=ROOT/'reports/cameras/supplied_step_inward_v2'


def fields(element):
    return [(e.tag,dict(e.attrib)) for e in element.iter()]


def main():
    global OUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/camera_rig_supplied_step_inward_v2.json')
    parser.add_argument('--report-directory',type=Path,default=OUT)
    args=parser.parse_args();OUT=args.report_directory
    OUT.mkdir(parents=True,exist_ok=True)
    cfg_path=args.config.resolve()
    rig=CameraRig(config_path=cfg_path)
    cfg=rig.config
    old_cfg=json.loads((ROOT/cfg['base_config']).read_text())
    assert cfg['hand_mount_overrides']==old_cfg['hand_mount_overrides'] and cfg['pose']==old_cfg['pose']
    before=ET.parse(ROOT/'urdf/tron2_dach_revo3_supplied_step_20260914.urdf').getroot()
    after=ET.parse(rig.urdf_path).getroot()
    comparisons=[]
    for tag in ['joint','link']:
        for element in before.findall(tag):
            name=element.get('name')
            if 'camera' in name:continue
            actual=after.find(f"{tag}[@name='{name}']")
            assert actual is not None and fields(element)==fields(actual),f'Robot/hand changed: {name}'
            comparisons.append(name)
    sys.path.insert(0,str(ROOT/'design/revision2/analysis'))
    from extract_wrist_features import wrist_mesh,cylinder_candidate,section_polylines
    from check_v2_fit import closed_paths,section_solid
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,5.5),layout='constrained')
    report={'config':str(cfg_path.relative_to(ROOT)),'config_sha256':sha(cfg_path),
        'urdf':str(rig.urdf_path.relative_to(ROOT)),'scene':str(rig.scene_path.relative_to(ROOT)),
        'robot_and_hand_elements_unchanged':len(comparisons),'nq':rig.model.nq,'ncam':rig.model.ncam,
        'sides':{},'physical_fit_or_strength_certified':False}
    old_assembly=json.loads((ROOT/'config/assembly.json').read_text())
    interfaces=json.loads((ROOT/'design/imported_camera_brackets_20260914/interfaces.json').read_text())
    for i,side in enumerate(['left','right']):
        manifest=json.loads((ROOT/cfg['wrists'][side]['bracket_manifest']).read_text())
        assert sha(Path(manifest['source']))==manifest['source_sha256']
        old=old_assembly[side]['wrist_to_adapter']
        W=rig.body_transform('wrist_roll_'+side[0].upper()+'_Link')
        A=W.copy();A[:3,:3]=W[:3,:3]@Rotation.from_euler('xyz',old['rpy_rad']).as_matrix()
        A[:3,3]=W[:3,3]+W[:3,:3]@np.array(old['xyz_m'])
        bracket_world=rig.body_transform(side+'_wrist_camera_mount_frame')
        B=np.linalg.inv(A)@bracket_world
        wm=wrist_mesh(side)
        fits=[];points=[]
        for h in manifest['wrist_holes']:
            p=B[:3,:3]@np.array(h['point_at_contact_radius_mm'])+B[:3,3]*1000
            u=B[:3,:3]@np.array(h['axis_source'])
            angle=float(np.degrees(np.arctan2(u[2],u[0]))%360)
            target=min([30,330],key=lambda a:abs((a-angle+180)%360-180))
            assert abs(angle-target)<1e-6
            fitted=cylinder_candidate(wm,target)
            tangent=np.array([-np.sin(np.deg2rad(target)),0,np.cos(np.deg2rad(target))])
            axial_error=float(p[1]-fitted['axis_y_mm'])
            tangent_error=float(p@tangent-fitted['axis_tangent_offset_mm'])
            assert abs(axial_error)<.001 and abs(tangent_error)<.001
            fits.append({'target_theta_deg':target,'axial_error_mm':axial_error,'tangent_error_mm':tangent_error,
                         'bracket_hole_diameter_mm':3.3,'wrist_hole_diameter_mm':fitted['cylinder_diameter_mm']})
            points.append(p)
        assert {h['target_theta_deg'] for h in fits}=={30,330}
        camera=rig.body_transform(side+'_wrist_camera_bottom_screw_frame')
        actual=np.array([camera[:3,3]+camera[:3,:3]@(np.array([-8.35,y,21])*.001) for y in [-10,10]])
        camera_points=manifest.get('camera_hole_centers_mm',interfaces['sides'][side]['camera_hole_centers_mm'])
        expected=np.array([bracket_world[:3,3]+bracket_world[:3,:3]@(np.array(p)*.001) for p in camera_points])
        errors=np.linalg.norm(actual[:,None]-expected[None,:],axis=2)
        error=float(max(errors.min(axis=0).max(),errors.min(axis=1).max())*1000)
        assert error<.001
        bracket=trimesh.load_mesh(ROOT/manifest['stl_mm'],process=True)
        bracket.vertices=bracket.vertices@B[:3,:3].T+B[:3,3]*1000
        adapter=trimesh.load_mesh(ROOT/f'meshes/adapter_v2/{side}/adapter_{side}_v2_visual.stl',process=True)
        T=np.linalg.inv(A)@rig.body_transform(side+'_adapter_link')
        adapter.vertices=adapter.vertices*1000@T[:3,:3].T+T[:3,3]*1000
        overlaps=[]
        for y in [-.01,-.1,-.5,-1,-2,-3,-4.5,-6,-8,-10]:
            w=section_solid(closed_paths(wm,y));a=section_solid(closed_paths(adapter,y));b=section_solid(closed_paths(bracket,y))
            overlaps.append({'y_mm':y,'bracket_wrist_overlap_mm2':float((b^w).area()),'bracket_adapter_overlap_mm2':float((b^a).area())})
        ax=axes[i]
        for mesh,color,label in [(wm,'#596d80','Robot wrist'),(adapter,'#dc8a28','Hand adapter (unchanged)'),(bracket,'#248dca','Revised camera bracket')]:
            for j,p in enumerate(section_polylines(mesh,1,-4.5)):
                ax.plot(p[:,0],p[:,2],c=color,lw=1.3,label=label if j==0 else None)
        points=np.array(points);ax.scatter(points[:,0],points[:,2],c='#ce2828',marker='x',s=110,label='Aligned fixed small holes',zorder=10)
        if cfg['bracket_geometry']=='supplied_step_aligned_v3':
            a=np.deg2rad(285);p=np.array([np.cos(a),np.sin(a)])*30.5
            ax.scatter(*p,c='#16995a',marker='o',s=80,label='Exposed third retention screw',zorder=10)
        ax.set(title=side.capitalize()+' | fixed robot holes 330 / 30 deg',xlabel='Original adapter X [mm]',ylabel='Original adapter Z [mm]',xlim=(-40,40),ylim=(-40,40))
        ax.set_aspect('equal');ax.grid(alpha=.2)
        optical=rig.body_transform(side+'_wrist_camera_color_optical_frame')
        hand=rig.body_transform(side+'_hand_base_link');palm=rig.body_transform(side+'_palm')[:3,3]
        edge_error=float(np.degrees(np.arccos(np.clip(abs(camera[:3,1]@hand[:3,1]),0,1))))
        if cfg['bracket_geometry']=='supplied_step_aligned_v3':assert edge_error<1e-5
        d=optical[:3,3]-palm
        viewing_angle=float(np.degrees(np.arccos(np.clip(hand[:3,0]@d/np.linalg.norm(d),-1,1))))
        p=optical[:3,:3].T@(palm-optical[:3,3])
        K=np.array(next(s['color_K'] for s in rig.camera_specs if s['name']==side+'_wrist'))
        pixel=K@p;pixel=pixel[:2]/pixel[2]
        image=rig.capture_one(side+'_wrist')
        Image.fromarray(image['rgb']).save(OUT/f'{side}_wrist_rgb.png')
        np.save(OUT/f'{side}_wrist_depth_m.npy',image['depth'])
        report['sides'][side]={'wrist_hole_fits':fits,'rear_hole_max_error_mm':error,
            'cad_verification':manifest['verification'],'mesh_watertight':manifest['stl_watertight'],
            'axial_section_overlap':overlaps,'palm_normal_to_camera_deg':viewing_angle,
            'camera_edge_to_hand_transverse_angle_deg':edge_error,
            'palm_center_pixel':pixel.tolist(),'palm_center_in_frame':bool(p[2]>0 and 0<=pixel[0]<rig.width and 0<=pixel[1]<rig.height),
            'valid_depth_fraction':float(np.isfinite(image['depth']).mean())}
    axes[0].legend(fontsize=8,loc='lower left')
    fig.suptitle('Actual revised mesh section Y=-4.5 mm | Original robot holes unchanged')
    fig.savefig(OUT/'hole_alignment.png',dpi=160);plt.close(fig)
    renderer=mujoco.Renderer(rig.model,width=1200,height=900)
    def render_direction(name,center,direction,distance):
        direction=np.asarray(direction,dtype=float);direction/=np.linalg.norm(direction)
        cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam)
        cam.lookat[:]=center;cam.distance=distance
        cam.azimuth=180+np.degrees(np.arctan2(direction[1],direction[0]))
        cam.elevation=-np.degrees(np.arcsin(direction[2]))
        renderer.update_scene(rig.data,camera=cam,scene_option=rig.opt)
        Image.fromarray(renderer.render()).save(OUT/(name+'.png'))
    for side in ['left','right']:
        mount=rig.body_transform(side+'_wrist_camera_mount_frame')[:3,3]
        hand=rig.body_transform(side+'_hand_base_link');palm=rig.body_transform(side+'_palm')[:3,3]
        az=np.degrees(np.arctan2(hand[1,0],hand[0,0]));el=np.degrees(np.arcsin(hand[2,0]))
        for name,offset in [('mount',0),('oblique',35)]:
            cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam)
            cam.lookat[:]=.65*mount+.35*palm;cam.distance=.34
            # MuJoCo free-camera elevation has the opposite sign to the
            # world's eye-offset elevation: negative means looking from above.
            cam.azimuth=az+offset;cam.elevation=-np.clip(el+15,-80,80)
            renderer.update_scene(rig.data,camera=cam,scene_option=rig.opt)
            Image.fromarray(renderer.render()).save(OUT/f'{side}_{name}.png')
        if cfg['bracket_geometry']=='supplied_step_aligned_v3':
            camera=rig.body_transform(side+'_wrist_camera_bottom_screw_frame')
            render_direction(side+'_front_alignment',.5*mount+.5*camera[:3,3],camera[:3,0]+.35*hand[:3,0],.30)
            old=old_assembly[side]['wrist_to_adapter']
            W=rig.body_transform('wrist_roll_'+side[0].upper()+'_Link')
            R=W[:3,:3]@Rotation.from_euler('xyz',old['rpy_rad']).as_matrix()
            p=W[:3,3]+W[:3,:3]@np.array(old['xyz_m'])
            u=np.array([np.cos(np.deg2rad(285)),0,np.sin(np.deg2rad(285))])
            center=p+R@(u*.031+np.array([0,-.0045,0]))
            render_direction(side+'_third_screw',center,R@u+.35*hand[:3,0],.20)
    renderer.close()
    try:
        rig.set_mount('left',0,0)
        raise AssertionError('Fixed geometry must reject a transform-only edit')
    except ValueError:report['fixed_geometry_edit_guard_pass']=True
    rig.close()
    report['requested_hole_alignment_pass']=True
    report['robot_hand_pose_preservation_pass']=True
    report['view_status']='See rendered RGB: geometric projection and depth checks are not a grasping/visibility certification.'
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='sides'},indent=2))
    for side,info in report['sides'].items():
        print(side,json.dumps({k:v for k,v in info.items() if k not in ['cad_verification','axial_section_overlap']},indent=2))


if __name__=='__main__':main()
