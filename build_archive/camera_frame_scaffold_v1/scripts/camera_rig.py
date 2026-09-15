#!/usr/bin/env python3
"""MuJoCo three-camera RGB-D source with explicit optical TF and live mount frames.

Bracket STEP geometry is currently unavailable. Mount parameters are a visible,
documented frame scaffold, not reconstructed CAD or a verified physical bracket.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from build_camera_rig import ROOT, CAMERA_URDF, SCENE, build


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def intrinsic(width,height,fov):
    fx=width/(2*np.tan(np.deg2rad(fov[0])/2));fy=height/(2*np.tan(np.deg2rad(fov[1])/2))
    return np.array([[fx,0,(width-1)/2],[0,fy,(height-1)/2],[0,0,1]],dtype=float)


def origin(element):
    xyz=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
    R=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' '))
    return xyz,R


class CameraRig:
    def __init__(self,width=640,height=480,config_path=None):
        self.config_path=Path(config_path or ROOT/'config/camera_rig.json').resolve()
        self.config=json.loads(self.config_path.read_text())
        if width is None:width=self.config['width']
        if height is None:height=self.config['height']
        self.width,self.height=int(width),int(height)
        if not (64<=self.width<=1280 and 64<=self.height<=960):raise ValueError('Render resolution outside supported bounds')
        manifest_path=ROOT/'reports/cameras/build_manifest.json'
        fresh=False
        if manifest_path.exists() and CAMERA_URDF.exists() and SCENE.exists():
            r=json.loads(manifest_path.read_text())
            fresh=(r['config_sha256']==sha(self.config_path) and r['producer_sha256']==sha(ROOT/'scripts/build_camera_rig.py')
                   and r['urdf_sha256']==sha(CAMERA_URDF) and r['scene_sha256']==sha(SCENE))
        if not fresh:build(self.config_path)
        self.model=mujoco.MjModel.from_xml_path(str(SCENE))
        self.data=mujoco.MjData(self.model)
        for name,value in self.config['pose'].items():
            j=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,name)
            if j<0:raise ValueError('Unknown pose joint '+name)
            self.data.qpos[self.model.jnt_qposadr[j]]=np.clip(value,*self.model.jnt_range[j])
        self.opt=mujoco.MjvOption();mujoco.mjv_defaultOption(self.opt)
        self.opt.geomgroup[:]=[1,1,0,0,0,0]
        self.camera_specs=[]
        self.camera_ids={}
        for name,part in [('head',self.config['head']),('left_wrist',self.config['wrists']['left']),('right_wrist',self.config['wrists']['right'])]:
            model=self.config['models'][part['model']]
            prefix='head_camera' if name=='head' else name+'_camera'
            spec={'name':name,'model':part['model'],'width':self.width,'height':self.height,
                  'frame_id':prefix+'_depth_optical_frame','color_frame_id':prefix+'_color_optical_frame',
                  'depth_frame_id':prefix+'_depth_optical_frame','depth_range_m':model['depth_range_m'],
                  'intrinsics_status':model['intrinsic_status']}
            self.camera_ids[name]={}
            for stream in ('color','depth'):
                k=intrinsic(self.width,self.height,model[stream+'_fov_deg']);spec[stream+'_K']=k.tolist()
                cid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,name+'_'+stream)
                self.camera_ids[name][stream]=cid
                # MuJoCo stores focal length in sensor units, not pixels.
                self.model.cam_resolution[cid]=[self.width,self.height]
                self.model.cam_sensorsize[cid]=[1,1]
                self.model.cam_intrinsic[cid]=[k[0,0]/self.width,k[1,1]/self.height,0,0]
            spec['K']=spec['depth_K'];self.camera_specs.append(spec)
        self.joints=[];self.static_transforms=[];self.dynamic_joints=[]
        mutable={f'{side}_wrist_camera_{stage}_frame' for side in ('left','right') for stage in ('rod_end','plate')}
        for joint in ET.parse(CAMERA_URDF).getroot().findall('joint'):
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            xyz,R=origin(joint.find('origin'))
            row={'parent':parent,'child':child,'xyz':xyz.tolist(),'quat_xyzw':R.as_quat().tolist()}
            if joint.get('type')=='fixed' and child not in mutable:self.static_transforms.append(row)
            else:self.dynamic_joints.append((parent,child))
        self.tick=0
        self.renderer=mujoco.Renderer(self.model,height=self.height,width=self.width)
        mujoco.mj_forward(self.model,self.data)

    def set_mount(self,side,length_mm,tilt_deg):
        if side not in ('left','right'):raise ValueError('side must be left or right')
        length_mm=float(length_mm);tilt_deg=float(tilt_deg)
        if not 20<=length_mm<=150 or not -30<=tilt_deg<=75:raise ValueError('Mount parameters outside preview bounds')
        cfg=self.config['wrists'][side];cfg['length_mm']=length_mm;cfg['plate_tilt_deg']=tilt_deg
        end=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,side+'_wrist_camera_rod_end_frame')
        plate=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,side+'_wrist_camera_plate_frame')
        self.model.body_pos[end]=np.array(cfg['extension_axis'])*length_mm*.001
        r=Rotation.from_euler('xyz',cfg['plate_zero_rpy_rad'])*Rotation.from_rotvec(np.array(cfg['plate_tilt_axis'])*np.deg2rad(tilt_deg))
        self.model.body_quat[plate]=r.as_quat()[[3,0,1,2]]
        mujoco.mj_forward(self.model,self.data)

    def body_transform(self,name):
        bid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,name)
        if bid<0:raise ValueError('Missing TF body '+name)
        t=np.eye(4);t[:3,:3]=self.data.xmat[bid].reshape(3,3);t[:3,3]=self.data.xpos[bid]
        return t

    def transforms(self):
        rows=[]
        for parent,child in self.dynamic_joints:
            p,c=self.body_transform(parent),self.body_transform(child)
            R=p[:3,:3].T@c[:3,:3];xyz=p[:3,:3].T@(c[:3,3]-p[:3,3])
            rows.append({'parent':parent,'child':child,'xyz':xyz.tolist(),'quat_xyzw':Rotation.from_matrix(R).as_quat().tolist()})
        return rows

    def capture_one(self,name):
        ids=self.camera_ids[name]
        self.renderer.disable_depth_rendering();self.renderer.update_scene(self.data,camera=ids['color'],scene_option=self.opt)
        rgb=self.renderer.render().copy()
        self.renderer.enable_depth_rendering();self.renderer.update_scene(self.data,camera=ids['depth'],scene_option=self.opt)
        depth=self.renderer.render().astype(np.float32,copy=True)
        self.renderer.disable_depth_rendering()
        lo,hi=next(s for s in self.camera_specs if s['name']==name)['depth_range_m']
        # Preserve physical metric optical-Z depth. Mask unavailable ranges,
        # never scale a display colormap into a ROS depth payload.
        depth[(depth<lo)|(depth>hi)|(~np.isfinite(depth))]=np.nan
        return {'rgb':rgb,'depth':depth}

    def step_and_capture(self):
        self.tick+=1
        stamp_ns=round(self.tick*1e9/self.config['fps'])
        self.data.time=stamp_ns/1e9
        mujoco.mj_forward(self.model,self.data)
        return {'stamp_ns':stamp_ns,'frames':{s['name']:self.capture_one(s['name']) for s in self.camera_specs},
                'transforms':self.transforms()}

    def save_config(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(self.config,indent=2,ensure_ascii=False)+'\n')

    def close(self):
        if self.renderer is not None:self.renderer.close();self.renderer=None


def create_source(width=640,height=480,config_path=None):
    return CameraRig(width,height,config_path)


def snapshot(source,output):
    from PIL import Image
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    packet=source.step_and_capture();summary={}
    for spec in source.camera_specs:
        name=spec['name'];frame=packet['frames'][name]
        Image.fromarray(frame['rgb']).save(output/(name+'_rgb.png'))
        np.save(output/(name+'_depth_m.npy'),frame['depth'])
        valid=np.isfinite(frame['depth']);display=np.zeros(frame['depth'].shape+(3,),dtype=np.uint8)
        if valid.any():
            from matplotlib import colormaps
            lo,hi=spec['depth_range_m'];norm=np.clip((np.nan_to_num(frame['depth'],nan=hi)-lo)/(hi-lo),0,1)
            display=(colormaps['turbo'](1-norm)[...,:3]*255).astype(np.uint8);display[~valid]=0
        Image.fromarray(display).save(output/(name+'_depth_preview.png'))
        summary[name]={'rgb_std':float(frame['rgb'].std()),'valid_depth_fraction':float(valid.mean()),
                       'min_depth_m':float(np.nanmin(frame['depth'])) if valid.any() else None,
                       'max_depth_m':float(np.nanmax(frame['depth'])) if valid.any() else None}
    report={'stamp_ns':packet['stamp_ns'],'geometry_status':source.config['geometry_status'],'camera_specs':source.camera_specs,
            'frames':summary,'dynamic_tf_count':len(packet['transforms']),'static_tf_count':len(source.static_transforms)}
    (output/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path);p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480)
    p.add_argument('--output',type=Path,default=ROOT/'reports/cameras/nominal')
    args=p.parse_args();src=create_source(args.width,args.height,args.config)
    try:print(json.dumps(snapshot(src,args.output),indent=2))
    finally:src.close()
