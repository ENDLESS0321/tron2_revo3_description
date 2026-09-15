#!/usr/bin/env python3
"""Four public viewers share this final-model-only implementation."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
VIEWS={'front':(105,-12),'back':(-75,-12),'top':(90,-89),'wrist':(105,65)}


def depth_image(depth,limits):
    import numpy as np
    from PIL import Image
    lo,hi=limits;valid=np.isfinite(depth)&(depth>0)
    unit=np.clip((np.where(valid,depth,lo)-lo)/(hi-lo),0,1)
    colors=np.array([[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]])
    rgb=np.stack([np.interp(unit,np.linspace(0,1,len(colors)),colors[:,i]) for i in range(3)],axis=-1).astype(np.uint8)
    rgb[~valid]=0;return Image.fromarray(rgb)


def save_packet(packet,output):
    import numpy as np
    from PIL import Image
    output.mkdir(parents=True,exist_ok=True);report={'stamp_ns':packet['stamp_ns'],'depth_unit':'meter','cameras':{}}
    for spec in packet['specs']:
        name=spec['name'];row=packet['frames'][name];rgb,depth=row['rgb'],row['depth'];valid=np.isfinite(depth)&(depth>0)
        if rgb.dtype!=np.uint8 or rgb.shape[:2]!=depth.shape or not np.isfinite(rgb).all():raise ValueError('Invalid frame '+name)
        Image.fromarray(rgb).save(output/(name+'_rgb.png'))
        depth_image(depth,spec['depth_range_m']).save(output/(name+'_depth.png'))
        np.save(output/(name+'_depth_m.npy'),depth,allow_pickle=False)
        report['cameras'][name]={k:v for k,v in spec.items() if not k.endswith('_id')}
        report['cameras'][name].update(rgb_shape=list(rgb.shape),rgb_std=float(rgb.std()),valid_depth_pixels=int(valid.sum()))
    (output/'capture.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def camera_window(args):
    import tkinter as tk
    from tkinter import ttk,messagebox
    from PIL import Image,ImageTk
    from _runtime import Cameras,config
    root=tk.Tk();root.title('TRON2 · 三相机 RGB / 深度');root.geometry('1220x790')
    toolbar=ttk.Frame(root,padding=10);toolbar.pack(fill='x')
    ttk.Label(toolbar,text='三相机 RGB / 米制深度 · 最终 V3 装配',font=('Sans',17,'bold')).pack(side='left')
    latest=[None];stopping=threading.Event();updates=queue.Queue(maxsize=1)
    def save():
        if latest[0] is None:return
        path=args.output/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        try:save_packet(latest[0],path);status.set('已保存 RGB、深度 PNG、原始深度 NPY 和相机参数：'+str(path))
        except Exception as exc:messagebox.showerror('保存失败',str(exc))
    ttk.Button(toolbar,text='保存当前图像与深度',command=save).pack(side='right')
    ttk.Label(root,text='固定模型预览，不连接硬件。深度黑色区域为无效值；原始深度未与彩色图配准。',padding=10).pack(fill='x')
    grid=ttk.Frame(root,padding=8);grid.pack(fill='both',expand=True)
    panels={};images={}
    for col,spec in enumerate(config()['cameras']):
        name=spec['name'];grid.columnconfigure(col,weight=1)
        frame=ttk.LabelFrame(grid,text=spec['label'],padding=6);frame.grid(row=0,column=col,sticky='nsew',padx=4)
        for stream,label in [('rgb','RGB'),('depth',f"深度 {spec['depth_range_m'][0]:g}–{spec['depth_range_m'][1]:g} m")]:
            ttk.Label(frame,text=label).pack(anchor='w')
            panel=tk.Label(frame,bg='#172331',fg='white',text='等待图像…');panel.pack(fill='both',expand=True,pady=(3,8))
            panels[name,stream]=panel
    grid.rowconfigure(0,weight=1)
    status=tk.StringVar(value='正在加载最终相机场景…');ttk.Label(root,textvariable=status,padding=10,wraplength=1170).pack(fill='x')
    def put(item):
        try:updates.get_nowait()
        except queue.Empty:pass
        updates.put_nowait(item)
    def capture_worker():
        source=None
        try:
            source=Cameras(args.width,args.height)
            while not stopping.is_set():
                start=time.monotonic();put(('frame',source.capture()))
                stopping.wait(max(0,1/args.rate-(time.monotonic()-start)))
        except Exception as exc:put(('error',str(exc)))
        finally:
            if source is not None:source.close()
    worker=threading.Thread(target=capture_worker,name='rgbd-renderer',daemon=True);worker.start()
    def poll():
        try:
            kind,item=updates.get_nowait()
            if kind=='error':
                status.set('相机加载失败：'+item);messagebox.showerror('相机加载失败',item)
            else:
                latest[0]=item
                for spec in item['specs']:
                    name=spec['name'];row=item['frames'][name]
                    for stream,im in [('rgb',Image.fromarray(row['rgb'])),('depth',depth_image(row['depth'],spec['depth_range_m']))]:
                        panel=panels[name,stream];w=max(200,min(420,panel.winfo_width()));h=round(w*im.height/im.width)
                        photo=ImageTk.PhotoImage(im.resize((w,h),Image.Resampling.BILINEAR if stream=='rgb' else Image.Resampling.NEAREST))
                        images[name,stream]=photo;panel.configure(image=photo,text='')
                if status.get().startswith('正在'):status.set('三路 RGB/深度已就绪；关闭窗口退出。')
        except queue.Empty:pass
        if not stopping.is_set():root.after(60,poll)
    def close():stopping.set();root.destroy()
    root.protocol('WM_DELETE_WINDOW',close);root.after(60,poll)
    try:root.mainloop()
    finally:stopping.set();worker.join(timeout=15)


def scene_view(args):
    import mujoco
    import numpy as np
    from PIL import Image
    from _runtime import assembly,parts,set_pose
    is_parts=args.mode!='assembly'
    if is_parts:model,data,info=parts(args.mode.replace('-','_'))
    else:model,data,cfg=assembly();info=None
    state={'side':args.side,'view':'front','dirty':True,'pose':None};lock=threading.Lock()
    def settings(cam,opt,side,view):
        if is_parts:
            opt.geomgroup[:]=[0,side in ('left','both'),side in ('right','both'),0,0,0]
            cam.lookat[:]=[0,0,info['left']['center'][2]] if side=='both' else info[side]['center']
            cam.distance=.36 if side=='both' else .20;cam.azimuth,cam.elevation=VIEWS[view]
        else:
            opt.geomgroup[:]=[1,1,0,0,0,0];cam.lookat[:]=[.25,0,1.0]
            cam.distance=2.35;cam.azimuth=135;cam.elevation=-15
    if args.check:
        args.output.mkdir(parents=True,exist_ok=True)
        cam=mujoco.MjvCamera();mujoco.mjv_defaultCamera(cam);opt=mujoco.MjvOption();mujoco.mjv_defaultOption(opt)
        combinations=[(s,v) for s in ['left','right','both'] for v in VIEWS] if is_parts else [('both','overview')]
        with mujoco.Renderer(model,width=1100,height=800) as renderer:
            for side,view in combinations:
                settings(cam,opt,side,view);renderer.update_scene(data,camera=cam,scene_option=opt)
                pixels=renderer.render()
                if pixels.std()<3:raise ValueError('Empty render')
                if is_parts:
                    visible={int(g.objid) for g in renderer.scene.geoms[:renderer.scene.ngeom] if g.objtype==mujoco.mjtObj.mjOBJ_GEOM}
                    expected={'both':{0,1},'left':{0},'right':{1}}[side]
                    if visible!=expected:raise ValueError('Wrong visible parts')
                Image.fromarray(pixels).save(args.output/(args.mode+'_'+side+'_'+view+'.png'))
        print(json.dumps({'mode':args.mode,'checks':len(combinations),'nq':model.nq,'ngeom':model.ngeom,'status':'pass'}));return
    import mujoco.viewer
    def key(k):
        with lock:
            if is_parts and k in [ord('1'),ord('2'),ord('3')]:state['side']={49:'left',50:'right',51:'both'}[k]
            elif is_parts and k in [ord('F'),ord('B'),ord('T'),ord('U')]:state['view']={70:'front',66:'back',84:'top',85:'wrist'}[k]
            elif not is_parts and k in [ord('0'),ord('1')]:state['pose']='zero' if k==ord('0') else 'display'
            elif k==ord('R'):state['view']='front'
            else:return
            state['dirty']=True
    with mujoco.viewer.launch_passive(model,data,key_callback=key,show_left_ui=False,show_right_ui=False) as viewer:
        viewer.set_texts((None,None,'Final model | '+args.mode+'\nMouse: rotate / pan / zoom',
            '1 left | 2 right | 3 both | F/B/T/U views | R reset' if is_parts else '0 zero pose | 1 display pose | R reset view | kinematic only'))
        while viewer.is_running():
            with lock:current=dict(state);state['dirty']=False;state['pose']=None
            with viewer.lock():
                if current['pose'] is not None:set_pose(model,data,{} if current['pose']=='zero' else cfg['pose'])
                if current['dirty']:
                    mujoco.mjv_defaultOption(viewer.opt);settings(viewer.cam,viewer.opt,current['side'],current['view'])
                if is_parts:viewer.opt.geomgroup[:]=[0,current['side'] in ('left','both'),current['side'] in ('right','both'),0,0,0]
                else:viewer.opt.geomgroup[:]=[1,1,0,0,0,0]
                mujoco.mj_forward(model,data)
            viewer.sync();time.sleep(.02)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',required=True,choices=['adapters','camera-brackets','assembly','cameras'])
    p.add_argument('--side',choices=['left','right','both'],default='both')
    p.add_argument('--check',action='store_true',help='Validate assets and render without opening a window')
    p.add_argument('--output',type=Path,default=ROOT/'outputs')
    p.add_argument('--width',type=int,default=320);p.add_argument('--height',type=int,default=240)
    p.add_argument('--rate',type=float,default=8)
    args=p.parse_args()
    if not (64<=args.width<=1280 and 64<=args.height<=960 and 0<args.rate<=60):p.error('Invalid resolution/rate')
    if not args.check and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):p.error('No desktop display; use --check for offscreen images')
    os.environ.setdefault('MUJOCO_GL','egl' if args.check or args.mode=='cameras' else 'glfw')
    from _runtime import verify_assets,Cameras
    print(json.dumps(verify_assets()))
    if args.mode!='cameras':scene_view(args)
    elif args.check:
        source=Cameras(args.width,args.height)
        try:
            report=save_packet(source.capture(),args.output)
            if set(report['cameras'])!={'head','left_wrist','right_wrist'}:raise ValueError('Missing camera')
            for name,row in report['cameras'].items():
                if row['rgb_std']==0 or row['valid_depth_pixels']==0:raise ValueError('Empty camera '+name)
            print(json.dumps({'mode':'cameras','cameras':list(report['cameras']),'status':'pass'}))
        finally:source.close()
    else:camera_window(args)


if __name__=='__main__':main()
