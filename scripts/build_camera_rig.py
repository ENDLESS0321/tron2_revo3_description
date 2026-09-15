#!/usr/bin/env python3
"""Add RealSense cameras and either a frame study or the new parametric bracket CAD."""
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
FIXED_STEP_GEOMETRIES=('supplied_step_20260914','supplied_step_inward_v2','supplied_step_aligned_v3')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def vec(values):return ' '.join(f'{float(x):.12g}' for x in values)


def fixed(robot,name,parent,child,xyz=(0,0,0),rpy=(0,0,0)):
    ET.SubElement(robot,'link',name=child)
    j=ET.SubElement(robot,'joint',name=name,type='fixed')
    ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child)
    ET.SubElement(j,'origin',xyz=vec(xyz),rpy=vec(rpy))
    return robot.find(f"link[@name='{child}']")


def camera_model(robot,name,parent,model,mesh_path,bottom_xyz=(0,0,0),bottom_rpy=(0,0,0)):
    fixed(robot,name+'_mount_joint',parent,name+'_bottom_screw_frame',bottom_xyz,bottom_rpy)
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


def output_paths(config_path):
    config_path=Path(config_path).resolve()
    if config_path==(ROOT/'config/camera_rig.json').resolve():
        return CAMERA_URDF,SCENE,REPORTS/'build_manifest.json'
    if config_path==(ROOT/'config/camera_rig_supplied_step_20260914.json').resolve():
        return (ROOT/'urdf/tron2_dach_revo3_supplied_step_20260914.urdf',
                ROOT/'simulation/cameras/supplied_step_20260914/scene.xml',
                REPORTS/'supplied_step_20260914/build_manifest.json')
    if config_path==(ROOT/'config/camera_rig_supplied_step_inward_v2.json').resolve():
        return (ROOT/'urdf/tron2_dach_revo3_supplied_step_inward_v2.urdf',
                ROOT/'simulation/cameras/supplied_step_inward_v2/scene.xml',
                REPORTS/'supplied_step_inward_v2/build_manifest.json')
    if config_path==(ROOT/'config/camera_rig_supplied_step_aligned_v3.json').resolve():
        return (ROOT/'urdf/tron2_dach_revo3_supplied_step_aligned_v3.urdf',
                ROOT/'simulation/cameras/supplied_step_aligned_v3/scene.xml',
                REPORTS/'supplied_step_aligned_v3/build_manifest.json')
    key=sha(config_path)[:16]
    return (ROOT/f'urdf/tron2_dach_revo3_cameras_{key}.urdf',
            ROOT/f'simulation/cameras/variants/{key}/scene.xml',
            ROOT/f'reports/cameras/variants/{key}/build_manifest.json')


def build(config_path=ROOT/'config/camera_rig.json'):
    config_path=Path(config_path);cfg=json.loads(config_path.read_text())
    urdf_path,scene_path,report_path=output_paths(config_path)
    imported=cfg.get('bracket_geometry') in FIXED_STEP_GEOMETRIES
    physical=imported or cfg.get('bracket_geometry') in ('parametric_photo_reference_v1','parametric_vertical_v2','parametric_side_mount_v3','parametric_side_bend_v4')
    source=ROOT/cfg['source_urdf'];robot=ET.parse(source).getroot()
    robot.set('name','tron2_dach_revo3_photo_brackets' if physical else 'tron2_dach_revo3_camera_frame_scaffold')
    robot.insert(0,ET.Comment('New photo-reference bracket design; camera is a separate object; material/physical fit remain uncalibrated.' if physical else 'Camera-frame scaffold only; bracket geometry is absent.'))
    if imported:
        robot.set('name','tron2_dach_revo3_supplied_step_brackets')
        robot[0].text='STEP-derived brackets, separate D405 cameras; see config and CAD manifests for geometry revisions. Nominal model, not hardware certified.'
    # Rotate the adapter and the complete hand together about the existing
    # flange origin. All hand-to-adapter transforms and joint values stay intact.
    for side,override in cfg.get('hand_mount_overrides',{}).items():
        joint=robot.find(f"joint[@name='{side}_adapter_mount']")
        if joint is None:raise ValueError('Missing adapter mount for '+side)
        joint.find('origin').set('rpy',vec(override['rpy_rad']))
    meshes=prepare_meshes();frames={};mutable=[]
    bracket_assets={}
    head=cfg['head'];head_ref='head_camera_mount_frame'
    fixed(robot,'head_camera_reference_joint',head['parent'],head_ref,head['mount_xyz_m'],head['mount_rpy_rad'])
    camera_model(robot,'head_camera',head_ref,cfg['models'][head['model']],meshes[head['model']])
    frames['head']={'prefix':'head_camera','model':head['model']}
    for side,row in cfg['wrists'].items():
        prefix=side+'_wrist_camera';mount=prefix+'_mount_frame';end=prefix+'_rod_end_frame';plate=prefix+'_plate_frame'
        mount_link=fixed(robot,prefix+'_reference_joint',row['parent'],mount,row['mount_xyz_m'],row['mount_rpy_rad'])
        if physical:
            if imported:
                manifest_path=ROOT/row['bracket_manifest']
                manifest=json.loads(manifest_path.read_text())
                asset={'manifest':manifest,'manifest_path':manifest_path,'mesh':ROOT/manifest['mesh_m'], 'key':manifest.get('geometry_sha256',manifest['source_sha256'])[:16]}
                if sha(asset['mesh'])!=manifest['mesh_sha256']:raise ValueError('Imported bracket mesh changed')
            else:
                from bracket_assets import ensure_bracket
                asset=ensure_bracket(row['length_mm'],row['plate_tilt_deg'],cfg['bracket_geometry'])
            manifest=asset['manifest'];whole=manifest['whole']
            v=ET.SubElement(mount_link,'visual')
            ET.SubElement(ET.SubElement(v,'geometry'),'mesh',filename='../'+str(asset['mesh'].relative_to(ROOT)))
            material=ET.SubElement(v,'material',name=side+'_camera_bracket_material')
            ET.SubElement(material,'color',rgba='0.16 0.2 0.24 1')
            inertial=ET.SubElement(mount_link,'inertial')
            ET.SubElement(inertial,'origin',xyz=vec(np.array(whole['center_of_mass_mm'])*.001),rpy='0 0 0')
            ET.SubElement(inertial,'mass',value=str(whole['mass_kg']))
            inertia=whole['inertia_at_com_kg_m2']
            ET.SubElement(inertial,'inertia',**{k:str(inertia[i][j]) for k,i,j in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
            bracket_assets[side]={'key':asset['key'],'manifest':str(asset['manifest_path'].relative_to(ROOT)),
                                  'manifest_sha256':sha(asset['manifest_path']),'mesh':str(asset['mesh'].relative_to(ROOT)),
                                  'mesh_sha256':sha(asset['mesh']),'camera_separate':True}
        delta=np.array(row.get('extension_origin_m',[0,0,0]))+np.array(row['extension_axis'])*row['length_mm']*.001
        fixed(robot,prefix+'_extension_joint',mount,end,delta)
        rotation=Rotation.from_euler('xyz',row['plate_zero_rpy_rad'])*Rotation.from_rotvec(np.array(row['plate_tilt_axis'])*np.deg2rad(row['plate_tilt_deg']))
        fixed(robot,prefix+'_plate_joint',end,plate,rpy=rotation.as_euler('xyz'))
        camera_model(robot,prefix,plate,cfg['models'][row['model']],meshes[row['model']],row.get('camera_bottom_xyz_m',[0,0,0]),row.get('camera_bottom_rpy_rad',[0,0,0]))
        frames[side+'_wrist']={'prefix':prefix,'model':row['model']}
        if not imported:mutable.extend([end,plate])
    ET.indent(robot,space='  ');ET.ElementTree(robot).write(urdf_path,encoding='utf-8',xml_declaration=True)
    model=mujoco.MjModel.from_xml_path(str(urdf_path))
    scene_path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='camera_rig_mjcf_') as tmp:
        raw=Path(tmp)/'scene.xml';mujoco.mj_saveLastXML(str(raw),model);scene=ET.parse(raw).getroot()
    compiler=scene.find('compiler');compiler.attrib.pop('meshdir',None)
    compiler.set('strippath','false');compiler.set('discardvisual','false');compiler.set('fusestatic','false')
    for mesh in scene.findall('asset/mesh'):
        path=Path(mesh.get('file'));path=path if path.is_absolute() else (urdf_path.parent/path).resolve()
        mesh.set('file',os.path.relpath(path,scene_path.parent))
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
    ET.indent(scene,space='  ');ET.ElementTree(scene).write(scene_path,encoding='utf-8',xml_declaration=True)
    checked=mujoco.MjModel.from_xml_path(str(scene_path))
    if checked.nq!=58 or checked.ncam!=6:raise ValueError('Expected 58 robot joints and six color/depth render cameras')
    report_path.parent.mkdir(parents=True,exist_ok=True)
    report={'source_urdf':str(source.relative_to(ROOT)),'source_urdf_sha256':sha(source),'config_path':str(config_path.resolve()),'config_sha256':sha(config_path),
            'urdf_path':str(urdf_path.relative_to(ROOT)),'scene_path':str(scene_path.relative_to(ROOT)),
            'urdf_sha256':sha(urdf_path),'scene_sha256':sha(scene_path),'producer_sha256':sha(Path(__file__)),
            'geometry_status':cfg['geometry_status'],'hardware_model_status':cfg['hardware_model_status'],
            'camera_names':list(frames),'frames':frames,'mutable_mount_frames':mutable,'nq':checked.nq,'ncam':checked.ncam,
            'complete_bracket_assembly':physical,'bracket_assets':bracket_assets,
            'physical_fit_or_strength_certified':False,'legacy_body_camera_retained':True}
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,default=ROOT/'config/camera_rig.json')
    print(json.dumps(build(parser.parse_args().config),indent=2))
