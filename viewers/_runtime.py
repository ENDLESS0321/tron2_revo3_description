"""Shared, read-only runtime for the final assets. No CAD or legacy build dependency."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'assets'


def config():return json.loads((ASSETS/'runtime.json').read_text())


def verify_assets():
    manifest=json.loads((ASSETS/'manifest.json').read_text())
    for name,record in manifest['files'].items():
        path=(ROOT/name).resolve()
        if not path.is_relative_to(ASSETS) or not path.is_file():raise ValueError('Missing or invalid asset: '+name)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:raise ValueError('Asset changed: '+name)
    urdf=ET.parse(ASSETS/'assembly.urdf')
    scene=ET.parse(ASSETS/'scene.xml')
    a={e.get('filename') for e in urdf.findall('.//mesh')}
    b={e.get('file') for e in scene.findall('asset/mesh')}
    if a!=b or len(a)!=manifest['referenced_mesh_count']:raise ValueError('URDF/MJCF mesh sets disagree')
    if any(Path(p).is_absolute() or not (ASSETS/p).resolve().is_relative_to(ASSETS) for p in a):raise ValueError('Nonportable mesh reference')
    return {'verified_files':len(manifest['files']),'referenced_meshes':len(a)}


def set_pose(model,data,pose):
    data.qpos[:]=model.qpos0
    for name,value in pose.items():
        jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
        if jid<0:raise ValueError('Unknown pose joint '+name)
        if not model.jnt_range[jid,0]<=value<=model.jnt_range[jid,1]:raise ValueError('Pose outside joint limit: '+name)
        data.qpos[model.jnt_qposadr[jid]]=value
    mujoco.mj_forward(model,data)


def assembly():
    cfg=config();model=mujoco.MjModel.from_xml_path(str(ASSETS/'scene.xml'));data=mujoco.MjData(model)
    if model.nq!=58 or model.ncam!=6:raise ValueError('Expected 58 robot joints and six render cameras')
    set_pose(model,data,cfg['pose'])
    return model,data,cfg


def parts(kind):
    cfg=config();root=ET.Element('mujoco',model=kind)
    ET.SubElement(root,'compiler',angle='radian')
    ET.SubElement(root,'option',gravity='0 0 0')
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='1200',offheight='850')
    ET.SubElement(visual,'headlight',ambient='.4 .4 .4',diffuse='.7 .7 .7')
    ET.SubElement(visual,'map',znear='.0001',zfar='10')
    assets=ET.SubElement(root,'asset')
    ET.SubElement(assets,'texture',name='sky',type='skybox',builtin='gradient',rgb1='.75 .81 .88',rgb2='.94 .96 .98',width='64',height='256')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='.1 -.2 .3',dir='-.3 .5 -1',diffuse='.7 .7 .7')
    info={}
    for i,row in enumerate(cfg['parts'][kind]):
        side=row['side'];path=ASSETS/row['mesh'];scale=row['scale']
        rotation=Rotation.from_euler('xyz',row['display_rpy_deg'],degrees=True)
        mesh=trimesh.load_mesh(path,process=False)
        vertices=rotation.apply(mesh.vertices*scale);lo=vertices.min(0);hi=vertices.max(0);center=(lo+hi)/2
        x=(i-.5)*.12;shift=[x-center[0],-center[1],-lo[2]]
        quat=rotation.as_quat()[[3,0,1,2]]
        ET.SubElement(assets,'mesh',name=side,file=str(path),scale=' '.join([str(scale)]*3))
        body=ET.SubElement(world,'body',name=side,pos=' '.join(map(str,shift)))
        ET.SubElement(body,'geom',name=side,mesh=side,type='mesh',quat=' '.join(map(str,quat)),contype='0',conaffinity='0',group=str(i+1),
                      rgba='.94 .43 .12 1' if side=='left' else '.16 .52 .85 1')
        info[side]={'center':[x,0,float(center[2]-lo[2])],'group':i+1}
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    if model.nq!=0 or model.ngeom!=2:raise ValueError('Parts mode must contain exactly two static meshes')
    return model,data,info


class Cameras:
    def __init__(self,width=320,height=240):
        self.model,self.data,self.config=assembly()
        self.width,self.height=width,height
        self.option=mujoco.MjvOption();mujoco.mjv_defaultOption(self.option);self.option.geomgroup[:]=[1,1,0,0,0,0]
        self.specs=[]
        for original in self.config['cameras']:
            row=dict(original)
            for stream in ['color','depth']:
                h,v=row[stream+'_fov_deg'];fx=width/(2*np.tan(np.deg2rad(h)/2));fy=height/(2*np.tan(np.deg2rad(v)/2))
                cid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,row['name']+'_'+stream)
                if cid<0:raise ValueError('Missing render camera')
                row[stream+'_id']=cid;row[stream+'_K']=[[fx,0,(width-1)/2],[0,fy,(height-1)/2],[0,0,1]]
                self.model.cam_resolution[cid]=[width,height];self.model.cam_sensorsize[cid]=[1,1]
                self.model.cam_intrinsic[cid]=[fx/width,fy/height,0,0]
            self.specs.append(row)
        self.renderer=mujoco.Renderer(self.model,width=width,height=height);self.tick=0

    def capture(self):
        self.tick+=1;self.data.time=self.tick/self.config['fps'];frames={}
        for row in self.specs:
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(self.data,camera=row['color_id'],scene_option=self.option)
            rgb=self.renderer.render().copy()
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(self.data,camera=row['depth_id'],scene_option=self.option)
            depth=self.renderer.render().astype(np.float32,copy=True);self.renderer.disable_depth_rendering()
            lo,hi=row['depth_range_m'];depth[(depth<lo)|(depth>hi)|~np.isfinite(depth)]=np.nan
            frames[row['name']]={'rgb':rgb,'depth':depth}
        return {'stamp_ns':round(self.data.time*1e9),'frames':frames,'specs':self.specs}

    def close(self):self.renderer.close()
