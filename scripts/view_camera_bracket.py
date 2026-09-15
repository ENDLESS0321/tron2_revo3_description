#!/usr/bin/env python3
"""Inspect the new one-piece camera bracket, without camera or robot geometry."""
import argparse
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from bracket_assets import ROOT,ensure_bracket


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--length-mm',type=float,default=80)
    parser.add_argument('--plate-tilt-deg',type=float,default=15)
    parser.add_argument('--design',choices=('parametric_side_bend_v4','parametric_side_mount_v3','parametric_vertical_v2','parametric_photo_reference_v1'),default='parametric_side_bend_v4')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    os.environ['MUJOCO_GL']='egl' if args.check else 'glfw'
    import mujoco
    import numpy as np
    from scipy.spatial.transform import Rotation
    asset=ensure_bracket(args.length_mm,args.plate_tilt_deg,args.design)
    record=asset['manifest'];directory=asset['manifest_path'].parent
    root=ET.Element('mujoco',model='Camera bracket | one piece, three functional regions | NO camera included')
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='1200',offheight='900')
    ET.SubElement(visual,'headlight',ambient='.4 .4 .4',diffuse='.7 .7 .7')
    ET.SubElement(visual,'map',znear='.0001')
    assets=ET.SubElement(root,'asset')
    ET.SubElement(assets,'texture',name='sky',type='skybox',builtin='gradient',rgb1='.75 .82 .9',rgb2='.94 .96 .98',width='64',height='256')
    world=ET.SubElement(root,'worldbody')
    ET.SubElement(world,'light',pos='.2 -.2 .4',dir='-.4 .4 -1',diffuse='.7 .7 .7')
    bend_mode=args.design=='parametric_side_bend_v4'
    vertical=args.design in ('parametric_vertical_v2','parametric_side_mount_v3','parametric_side_bend_v4')
    side_mount=args.design in ('parametric_side_mount_v3','parametric_side_bend_v4')
    colors={'wrist_mount':'.15 .4 .72 1','base':'.15 .4 .72 1','stem':'.15 .4 .72 1',
            'support_arm':'.15 .4 .72 1' if vertical else '.95 .45 .12 1',
            'support_stem':'.15 .4 .72 1','camera_plate':'.22 .65 .4 1','plate':'.22 .65 .4 1'}
    for name,part in record['parts'].items():
        path=directory/'parts'/f'{name}_body_m.stl'
        ET.SubElement(assets,'mesh',name=name,file=str(path))
        frame=part['body_frame'];q=Rotation.from_euler('xyz',frame['rpy_deg'],degrees=True).as_quat()[[3,0,1,2]]
        body=ET.SubElement(world,'body',name=name,pos=' '.join(map(str,np.array(frame['xyz_mm'])*.001)),quat=' '.join(map(str,q)))
        ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,rgba=colors[name],contype='0',conaffinity='0')
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    if model.ngeom!=3 or model.nq!=0:raise ValueError('Only three static bracket regions are expected')
    bounds=np.array(record['whole']['bounds_mm'])*.001
    center=bounds.mean(0);distance=float(np.linalg.norm(bounds[1]-bounds[0])*2)
    def setup(cam,azimuth=130,elevation=-25):
        cam.lookat[:]=center;cam.distance=distance;cam.azimuth=azimuth;cam.elevation=elevation
    if args.check:
        from PIL import Image,ImageDraw,ImageFont
        out=ROOT/'design'/('wrist_camera_bracket_v4' if bend_mode else 'wrist_camera_bracket_v3' if side_mount else 'wrist_camera_bracket_v2' if vertical else 'wrist_camera_bracket_v1')
        canvas=Image.new('RGB',(1800,950),(239,243,247));draw=ImageDraw.Draw(canvas)
        font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23)
        draw.text((24,15),f'Camera bracket {"V4" if bend_mode else "V3" if side_mount else "V2" if vertical else "V1"} | length {args.length_mm:g} mm | {"plate-to-stem" if bend_mode else "plate"} {args.plate_tilt_deg:g} deg',(24,38,55),font=font)
        draw.text((24,48),'Blue: wrist saddle and vertical stem   Green: camera plate   NO horizontal arm' if vertical else 'Blue: wrist saddle   Orange: support arm   Green: camera plate',(24,38,55),font=font)
        with mujoco.Renderer(model,height=850,width=900) as renderer:
            for i,(az,el) in enumerate([(130,-25),(-40,-20)]):
                cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam);setup(cam,az,el)
                renderer.update_scene(data,cam)
                canvas.paste(Image.fromarray(renderer.render()),(i*900,100))
        canvas.save(out/'bracket_overview.png')
        print(out/'bracket_overview.png')
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model,data,show_left_ui=False,show_right_ui=False) as viewer:
            setup(viewer.cam)
            viewer.set_texts((None,None,'Camera bracket ONLY\nOne connected solid\n'+('Blue saddle/stem / Green plate\nNO horizontal arm' if vertical else 'Blue wrist / Orange arm / Green plate')+'\nCamera is NOT included',f'L={args.length_mm:g} mm\nPlate={args.plate_tilt_deg:g} deg'))
            while viewer.is_running():viewer.sync();time.sleep(.02)


if __name__=='__main__':main()
