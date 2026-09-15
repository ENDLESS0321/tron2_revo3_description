#!/usr/bin/env python3
"""Add explicit RealSense model/optical frames to v2 without inventing STEP geometry."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import mujoco

ROOT=Path(__file__).resolve().parents[1]
REPORTS=ROOT/'reports/cameras'
CAMERA_URDF=ROOT/'urdf/tron2_dach_revo3_cameras.urdf'
SCENE=ROOT/'simulation/cameras/scene.xml'
OPTICAL_RPY=[-np.pi/2,0,-np.pi/2]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def vec(values):return ' '.join(f'{float(x):.12g}' for x in values)


def fixed(robot,name,parent,child,xyz=(0,0,0),rpy=(0,0,0)):
    ET.SubElement(robot,'link',name=child)
    j=ET.SubElement(robot,'joint',name=name,type='fixed')
    ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child)
    ET.SubElement(j,'origin',xyz=vec(xyz),rpy=vec(rpy))
    return robot.find(f"link[@name='{child}']")


def camera_model(robot,name,parent,model,mesh_path):
    fixed(robot,name+'_mount_joint',parent,name+'_bottom_screw_frame')
    link=fixed(robot,name+'_link_joint',name+'_bottom_screw_frame',name+'_link',model['bottom_to_link_xyz_m'])
    visual=ET.SubElement(link,'visual')
    ET.SubElement(visual,'origin',xyz=vec(model['visual_xyz_m']),rpy=vec(model['visual_rpy_rad']))
    ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename='../'+str(mesh_path.relative_to(ROOT)))
    mat=ET.SubElement(visual,'material',name=name+'_camera_dark')
    ET.SubElement(mat,'color',rgba='0.16 0.18 0.21 1')
    # Imaging scaffold: retain camera shape but do not assert unreliable source
    # inertia or add uncalibrated hardware payload dynamics.
    for stream in ('depth','color'):
        xyz=model['link_to_color_xyz_m'] if stream=='color' else [0,0,0]
        fixed(robot,name+'_'+stream+'_joint',name+'_link',name+'_'+stream+'_frame',xyz)
        fixed(robot,name+'_'+stream+'_optical_joint',name+'_'+stream+'_frame',name+'_'+stream+'_optical_frame',rpy=OPTICAL_RPY)


def prepare_meshes():
    source=ROOT/'vendor/realsense-description/realsense2_description/meshes'
    output=ROOT/'meshes/cameras';output.mkdir(parents=True,exist_ok=True)
    catalog=json.loads((ROOT/'config/camera_model_catalog.json').read_text())
    result={}
    for name,file,scale in [('D405','d405.stl',.001),('D435i','d435.dae',1.0)]:
        src=ROOT/catalog['models'][name]['meshes']['visual_m']
        # MuJoCo's STL decoder caps files at 200k triangles. Preserve all of
        # the D435 source geometry in OBJ rather than decimating its 231k faces.
        dest=output/(name.lower()+'_visual_m'+('.obj' if name=='D435i' else '.stl'))
        if sha(src)!=catalog['models'][name]['meshes']['visual_sha256']:
            raise ValueError('Camera source mesh hash mismatch')
        stamp=dest.with_suffix('.json')
        if not dest.exists() or not stamp.exists() or json.loads(stamp.read_text())['source_sha256']!=sha(src):
            if dest.suffix=='.obj':trimesh.load_mesh(src,process=False).export(dest)
            else:shutil.copy2(src,dest)
            stamp.write_text(json.dumps({'source':str(src.relative_to(ROOT)),'source_sha256':sha(src),'scale':1,'output_sha256':sha(dest)},indent=2)+'\n')
        result[name]=dest
    return result


def build(config_path=ROOT/'config/camera_rig.json'):
    config_path=Path(config_path);cfg=json.loads(config_path.read_text())
    source=ROOT/cfg['source_urdf'];robot=ET.parse(source).getroot()
    robot.set('name','tron2_dach_revo3_camera_frame_scaffold')
    robot.insert(0,ET.Comment('Camera-frame scaffold: actual bracket STEP geometry is pending; this is NOT a measured bracket assembly.'))
    meshes=prepare_meshes();frames={};mutable=[]
    head=cfg['head'];head_ref='head_camera_mount_frame'
    fixed(robot,'head_camera_reference_joint',head['parent'],head_ref,head['mount_xyz_m'],head['mount_rpy_rad'])
    camera_model(robot,'head_camera',head_ref,cfg['models'][head['model']],meshes[head['model']])
    frames['head']={'prefix':'head_camera','model':head['model']}
    for side,row in cfg['wrists'].items():
        prefix=side+'_wrist_camera';mount=prefix+'_mount_frame';end=prefix+'_rod_end_frame';plate=prefix+'_plate_frame'
        fixed(robot,prefix+'_reference_joint',row['parent'],mount,row['mount_xyz_m'],row['mount_rpy_rad'])
        delta=np.array(row['extension_axis'])*row['length_mm']*.001
        fixed(robot,prefix+'_extension_joint',mount,end,delta)
        rotation=Rotation.from_euler('xyz',row['plate_zero_rpy_rad'])*Rotation.from_rotvec(np.array(row['plate_tilt_axis'])*np.deg2rad(row['plate_tilt_deg']))
        fixed(robot,prefix+'_plate_joint',end,plate,rpy=rotation.as_euler('xyz'))
        camera_model(robot,prefix,plate,cfg['models'][row['model']],meshes[row['model']])
        frames[side+'_wrist']={'prefix':prefix,'model':row['model']}
        mutable.extend([end,plate])
    ET.indent(robot,space='  ');ET.ElementTree(robot).write(CAMERA_URDF,encoding='utf-8',xml_declaration=True)
    model=mujoco.MjModel.from_xml_path(str(CAMERA_URDF))
    SCENE.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='camera_rig_mjcf_') as tmp:
        raw=Path(tmp)/'scene.xml';mujoco.mj_saveLastXML(str(raw),model);scene=ET.parse(raw).getroot()
    compiler=scene.find('compiler');compiler.attrib.pop('meshdir',None)
    compiler.set('strippath','false');compiler.set('discardvisual','false');compiler.set('fusestatic','false')
    for mesh in scene.findall('asset/mesh'):
        path=Path(mesh.get('file'));path=path if path.is_absolute() else (CAMERA_URDF.parent/path).resolve()
        mesh.set('file',os.path.relpath(path,SCENE.parent))
    for geom in scene.findall('.//geom'):
        visual=int(geom.get('contype','1'))==0 and int(geom.get('conaffinity','1'))==0
        geom.set('group','1' if visual else '3')
    visual=scene.find('visual')
    if visual is None:visual=ET.SubElement(scene,'visual')
    ET.SubElement(visual,'global',offwidth='1280',offheight='960')
    ET.SubElement(visual,'map',znear='0.0005',zfar='10')
    ET.SubElement(visual,'headlight',ambient='0.3 0.3 0.3',diffuse='0.65 0.65 0.65')
    assets=scene.find('asset')
    ET.SubElement(assets,'texture',name='camera_scene_floor',type='2d',builtin='checker',rgb1='.24 .28 .33',rgb2='.3 .35 .4',width='256',height='256')
    ET.SubElement(assets,'material',name='camera_scene_floor',texture='camera_scene_floor',texrepeat='8 8')
    world=scene.find('worldbody')
    ET.SubElement(world,'geom',name='camera_floor',type='plane',size='3 3 .1',pos='0 0 -.001',material='camera_scene_floor',group='0')
    ET.SubElement(world,'light',name='camera_key_light',pos='1 -1 3',dir='-.2 .2 -1',diffuse='.9 .9 .9')
    ET.SubElement(world,'light',name='camera_fill_light',pos='-1 1 2',dir='.2 -.2 -1',diffuse='.55 .55 .55')
    ET.SubElement(world,'geom',name='work_table',type='box',size='.45 .48 .025',pos='.72 0 .725',rgba='.53 .4 .26 1',group='0')
    for i,(y,color) in enumerate([(-.22,'.9 .18 .14 1'),(0,'.15 .75 .28 1'),(.22,'.1 .35 .9 1')]):
        ET.SubElement(world,'geom',name=f'test_cube_{i}',type='box',size='.028 .028 .035',pos=f'.64 {y} .785',rgba=color,group='0')
    for name,item in frames.items():
        m=cfg['models'][item['model']]
        for stream in ('color','depth'):
            body=scene.find(f".//body[@name='{item['prefix']}_{stream}_optical_frame']")
            h,v=m[stream+'_fov_deg'];w,height=cfg['width'],cfg['height']
            fx=w/(2*np.tan(np.deg2rad(h)/2));fy=height/(2*np.tan(np.deg2rad(v)/2))
            # ROS optical +Z/+Y-down to OpenGL -Z/+Y-up is Rx(pi).
            ET.SubElement(body,'camera',name=name+'_'+stream,quat='0 1 0 0',resolution=f'{w} {height}',sensorsize='1 1',focalpixel=vec([fx,fy]),principalpixel='0 0')
    ET.indent(scene,space='  ');ET.ElementTree(scene).write(SCENE,encoding='utf-8',xml_declaration=True)
    checked=mujoco.MjModel.from_xml_path(str(SCENE))
    if checked.nq!=58 or checked.ncam!=6:raise ValueError('Expected 58 robot joints and six color/depth render cameras')
    REPORTS.mkdir(parents=True,exist_ok=True)
    report={'source_urdf':str(source.relative_to(ROOT)),'source_urdf_sha256':sha(source),'config_path':str(config_path.resolve()),'config_sha256':sha(config_path),
            'urdf_sha256':sha(CAMERA_URDF),'scene_sha256':sha(SCENE),'producer_sha256':sha(Path(__file__)),
            'geometry_status':cfg['geometry_status'],'hardware_model_status':cfg['hardware_model_status'],
            'camera_names':list(frames),'frames':frames,'mutable_mount_frames':mutable,'nq':checked.nq,'ncam':checked.ncam,
            'complete_bracket_assembly':False,'legacy_body_camera_retained':True}
    (REPORTS/'build_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,default=ROOT/'config/camera_rig.json')
    print(json.dumps(build(parser.parse_args().config),indent=2))
