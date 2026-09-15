#!/usr/bin/env python3
"""View the robot, real bracket solids and separate cameras at the RGB-D pose."""
import argparse
import os
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--brackets-only',action='store_true',help='Show only the two camera-bracket solids, without robot, cameras or table')
    args=parser.parse_args()
    os.environ['MUJOCO_GL']='egl' if args.check else 'glfw'
    import mujoco
    from camera_rig import CameraRig
    source=CameraRig(config_path=args.config)
    source.close()  # This window only needs the loaded model, not an RGB renderer.
    bracket_body_ids=[]
    if args.brackets_only:
        import numpy as np
        source.model.geom_group[:]=2
        for side in ['left','right']:
            bid=mujoco.mj_name2id(source.model,mujoco.mjtObj.mjOBJ_BODY,side+'_wrist_camera_mount_frame')
            selected=np.flatnonzero((source.model.geom_bodyid==bid)&(source.model.geom_type==mujoco.mjtGeom.mjGEOM_MESH))
            if not len(selected):raise ValueError('This configuration has no bracket mesh for '+side)
            source.model.geom_group[selected]=1
            bracket_body_ids.append(bid)
    if args.check:
        assert source.model.nq==58 and source.model.ncam==6
        print('Robot + generated brackets + separate cameras loaded; 58 DOF, 6 RGB/depth viewpoints.')
        if args.brackets_only:print('Brackets-only mode: both solids selected; robot, cameras and table hidden.')
        return
    import mujoco.viewer
    with mujoco.viewer.launch_passive(source.model,source.data,show_left_ui=False,show_right_ui=False) as viewer:
        viewer.cam.lookat[:]=[.25,0,1.0]
        viewer.cam.distance=2.35
        viewer.cam.azimuth=135
        viewer.cam.elevation=-15
        viewer.opt.geomgroup[:]=[1,1,0,0,0,0]
        if args.brackets_only:
            viewer.opt.geomgroup[:]=[0,1,0,0,0,0]
            viewer.cam.lookat[:]=source.data.xpos[bracket_body_ids].mean(axis=0)
            viewer.cam.distance=.85
        imported=source.config.get('bracket_geometry') in ('supplied_step_20260914','supplied_step_inward_v2','supplied_step_aligned_v3')
        detail='Supplied STEP brackets | L +90 / R -90\nInspection candidate: RGB is occluded; not a verified grasping rig' if imported else 'Photo-reference design; not original STEP reconstruction'
        if source.config.get('bracket_geometry')=='supplied_step_inward_v2':
            detail='Inward V2: wrist holes revised to fixed small-hole pair\nOriginal support and camera plate retained | Hands unchanged'
        if source.config.get('bracket_geometry')=='supplied_step_aligned_v3':
            detail='Aligned V3: level camera edges | centered support +10 mm\nShort saddle | third retention screw exposed | hands unchanged'
        if args.brackets_only:detail='BRACKETS ONLY | '+detail
        viewer.set_texts((None,None,'Robot + brackets + separate cameras\n'+detail+'\nMouse drag: rotate/pan | Wheel: zoom','Fixed STEP geometry; no old length/tilt sliders' if imported else 'Use view_cameras.sh for RGB-D and CAD parameters'))
        while viewer.is_running():viewer.sync();time.sleep(.02)


if __name__=='__main__':main()
