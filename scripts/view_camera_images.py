#!/usr/bin/env python3
"""Local RGB-D camera viewer; Tk UI, or headless --check without opening a GUI."""
from __future__ import annotations

import argparse
import copy
from datetime import datetime
import importlib
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time

import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1]
CAMERAS=("head","left_wrist","right_wrist")
CAMERA_LABELS={"head":"Head / 头部","left_wrist":"Left wrist / 左腕","right_wrist":"Right wrist / 右腕"}
SCAFFOLD_BANNER="当前仅相机安装坐标框架：STP 尚未成功读取，未包含真实相机支架几何。"
PHYSICAL_BANNER="按照片新设计的一体支架；相机独立；修改参数将重建 CAD，不代表已验证材料强度。"
STARTING_BANNER="正在读取相机和支架配置；首次生成 CAD 可能需要若干秒。"
REBUILD_TIMEOUT_S=60.
DEFAULT_USER_CONFIG=ROOT/"config/camera_rig_user.json"


def json_text(value):
    return json.dumps(value,ensure_ascii=False,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else float(x))+"\n"


def geometry_banner(config):
    if config is None:
        return STARTING_BANNER
    if config.get('bracket_geometry')=='supplied_step_aligned_v3':
        return 'V3：相机横边摆正、支撑居中外移10mm、连接弧缩短；腕部孔位和手姿态不变，第三颗固定螺钉已露出。'
    if config.get('bracket_geometry')=='supplied_step_inward_v2':
        return '向内安装 V2：仅改腕部孔组；原支撑段和相机板不变，手姿态不变。固定 STEP 几何，实物装配待验证。'
    if config.get('bracket_geometry')=='supplied_step_20260914':
        return '原始左右 STEP 支架：相机独立、孔位配准；固定几何，禁用旧版长度/倾角滑条。装配与视角仍需复核。'
    if config.get("bracket_geometry")=="parametric_side_bend_v4":
        return "V4侧面两小孔安装：界面角度是板与支撑杆的夹角，默认15°（几何内部Rx75°）；相机独立。"
    if config.get("bracket_geometry")=="parametric_side_mount_v3":
        return "V3：安装到侧面两小孔，覆盖中间的大固定孔；板角默认15°，长度与板角会重建实际CAD。"
    if config.get("bracket_geometry")=="parametric_vertical_v2":
        return "V2：蓝色竖段沿 +Z 加长，无橙色横臂、无大避让孔；绿色板单独调角，改参数会重建 CAD。"
    if config.get("bracket_geometry")=="parametric_photo_reference_v1" or "parametric_photo_reference" in str(config.get("geometry_status","")):
        return PHYSICAL_BANNER
    return SCAFFOLD_BANNER


def mount_parameter_limits(config):
    if config.get('parameter_editable') is False:
        return {'length_mm':[0.,150.],'plate_tilt_deg':[0.,90.]}
    limits=config.get("parameter_limits",{})
    result={}
    for key,fallback in (("length_mm",[20.,150.]),("plate_tilt_deg",[0.,75.])):
        lo,hi=map(float,limits.get(key,fallback))
        if key=="plate_tilt_deg":
            lo=max(0.,lo)
        if not np.isfinite([lo,hi]).all() or lo>=hi:
            raise ValueError(f"Invalid configured mount limits for {key}: {lo}, {hi}")
        result[key]=[lo,hi]
    return result


def image_font(size=18):
    for path in ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc","/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]:
        if Path(path).is_file():
            return ImageFont.truetype(path,size)
    return ImageFont.load_default()


def depth_color(depth,near=.1,far=3.):
    """Fixed metric color scale: dark purple near, yellow far; invalid black."""
    values=np.asarray(depth,dtype=np.float32)
    valid=np.isfinite(values)&(values>0)
    unit=np.clip((np.where(valid,values,near)-near)/(far-near),0,1)
    anchors=np.array([[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]],dtype=float)
    indices=np.linspace(0,1,len(anchors))
    rgb=np.stack([np.interp(unit,indices,anchors[:,i]) for i in range(3)],axis=-1).astype(np.uint8)
    rgb[~valid]=0
    return Image.fromarray(rgb,"RGB")


def camera_depth_limits(config,name,near=None,far=None):
    if near is not None and far is not None:
        return float(near),float(far)
    camera=config.get("head",{}) if name=="head" else config.get("wrists",{}).get(name.split("_")[0],{})
    values=config.get("models",{}).get(camera.get("model"),{}).get("depth_range_m",[.1,3.])
    return float(values[0]),float(values[1])


def validate_packet(packet):
    frames=packet["frames"]
    if set(frames)!=set(CAMERAS):
        raise ValueError(f"Expected exactly these cameras: {CAMERAS}")
    for name in CAMERAS:
        rgb=np.asarray(frames[name]["rgb"])
        depth=np.asarray(frames[name]["depth"])
        if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[-1]!=3:
            raise ValueError(f"{name}: RGB must be uint8 HxWx3")
        if depth.shape!=rgb.shape[:2] or not np.issubdtype(depth.dtype,np.floating):
            raise ValueError(f"{name}: depth must be a floating-point HxW array in meters")
    return packet


def freeze_packet(packet):
    result=dict(validate_packet(packet))
    result["frames"]={name:{**frame,"rgb":np.array(frame["rgb"],copy=True),"depth":np.array(frame["depth"],copy=True)} for name,frame in packet["frames"].items()}
    return result


def make_montage(packet,config,near,far,cell_width=400):
    validate_packet(packet)
    first=packet["frames"][CAMERAS[0]]["rgb"]
    cell_height=round(cell_width*first.shape[0]/first.shape[1])
    gap=14
    top=116
    row_label=28
    canvas=Image.new("RGB",(3*cell_width+4*gap,top+2*(cell_height+row_label)+62),(241,244,248))
    draw=ImageDraw.Draw(canvas)
    font=image_font(17)
    title=image_font(24)
    draw.text((gap,8),"TRON2 · three-camera RGB-D",font=title,fill=(24,37,52))
    draw.rectangle((gap,46,canvas.width-gap,83),fill=(255,236,198))
    draw.text((gap+10,51),geometry_banner(config),font=image_font(15),fill=(112,66,8))
    for column,name in enumerate(CAMERAS):
        x=gap+column*(cell_width+gap)
        frame=packet["frames"][name]
        display_near,display_far=camera_depth_limits(config,name,near,far)
        draw.text((x,89),CAMERA_LABELS[name],font=font,fill=(24,37,52))
        draw.text((x,top),"RGB",font=font,fill=(24,37,52))
        rgb=Image.fromarray(np.asarray(frame["rgb"]),"RGB").resize((cell_width,cell_height),Image.Resampling.BILINEAR)
        canvas.paste(rgb,(x,top+row_label))
        depth_y=top+cell_height+row_label
        draw.text((x,depth_y),f"Depth {display_near:g}–{display_far:g} m",font=font,fill=(24,37,52))
        colored=depth_color(frame["depth"],display_near,display_far).resize((cell_width,cell_height),Image.Resampling.NEAREST)
        canvas.paste(colored,(x,depth_y+row_label))
    footer=top+2*(cell_height+row_label)+7
    wrists=config.get("wrists",{})
    values=[]
    for side,label in [("left","L"),("right","R")]:
        params=wrists.get(side,{})
        values.append(f"{label}: length {params.get('length_mm','?')} mm, plate tilt {params.get('plate_tilt_deg','?')} deg")
    draw.text((gap,footer)," | ".join(values),font=image_font(15),fill=(51,68,82))
    draw.text((gap,footer+24),f"Simulation stamp: {packet.get('stamp_ns',0)/1e9:.3f} s  ·  black depth pixels = invalid",font=image_font(14),fill=(72,88,102))
    return canvas


def save_snapshot(packet,config,directory,near,far):
    directory.mkdir(parents=True,exist_ok=True)
    make_montage(packet,config,near,far).save(directory/"rgbd_montage.png")
    summary={"stamp_ns":packet.get("stamp_ns"),"depth_unit":"meter","geometry_status":config.get("geometry_status","unspecified"),"bracket_geometry":config.get("bracket_geometry","unspecified"),"geometry_banner":geometry_banner(config),"cameras":{}}
    for name in CAMERAS:
        frame=packet["frames"][name]
        rgb=np.asarray(frame["rgb"])
        depth=np.asarray(frame["depth"],dtype=np.float32)
        display_near,display_far=camera_depth_limits(config,name,near,far)
        Image.fromarray(rgb,"RGB").save(directory/f"{name}_rgb.png")
        depth_color(depth,display_near,display_far).save(directory/f"{name}_depth_color.png")
        np.save(directory/f"{name}_depth_m.npy",depth,allow_pickle=False)
        valid=np.isfinite(depth)&(depth>0)
        summary["cameras"][name]={"depth_display_limits_m":[display_near,display_far],"rgb_shape":list(rgb.shape),"rgb_std":float(rgb.std()),"positive_depth_pixels":int(valid.sum()),"finite_depth_pixels":int(np.isfinite(depth).sum()),"depth_min_m":float(depth[valid].min()) if valid.any() else None,"depth_max_m":float(depth[valid].max()) if valid.any() else None}
    (directory/"camera_config.json").write_text(json_text(config),encoding="utf-8")
    (directory/"snapshot.json").write_text(json_text(summary),encoding="utf-8")
    return directory/"rgbd_montage.png"


def create_source(args):
    os.environ.setdefault("MUJOCO_GL","egl")
    module=importlib.import_module("camera_rig")
    return module.create_source(width=args.width,height=args.height,config_path=args.config)


class CaptureWorker(threading.Thread):
    """The worker owns source construction, OpenGL context, capture and close."""
    def __init__(self,args):
        super().__init__(name="camera-capture",daemon=True)
        self.args=args
        self.commands=queue.Queue()
        self.events=queue.Queue()
        self.stop_requested=threading.Event()
        self.lock=threading.Lock()
        self.latest=None
        self.wake_requested=threading.Event()
        self.lifecycle=[]

    def emit(self,kind,payload,new_latest=None):
        record={"kind":kind,"wall_time_monotonic":time.monotonic()}
        if isinstance(payload,dict):
            record.update({k:copy.deepcopy(v) for k,v in payload.items() if k!="config"})
        elif kind=="error":
            record["message"]=str(payload)
        with self.lock:
            if new_latest is not None:
                self.latest=new_latest
            self.lifecycle.append(record)
        self.events.put((kind,payload))

    def run(self):
        source=None
        try:
            source=create_source(self.args)
            # Keep the ready payload compatible with the UI: it is the live
            # configuration, including the actual geometry mode and limits.
            self.events.put(("ready",copy.deepcopy(source.config)))
            while not self.stop_requested.is_set():
                start=time.monotonic()
                pending_mounts={}
                while True:
                    try:
                        side,length,tilt=self.commands.get_nowait()
                        pending_mounts[side]=(length,tilt)
                    except queue.Empty:
                        break
                pending_mounts={side:values for side,values in pending_mounts.items()
                                if abs(source.config["wrists"][side]["length_mm"]-values[0])>1e-9
                                or abs(source.config["wrists"][side]["plate_tilt_deg"]-values[1])>1e-9}
                if pending_mounts:
                    rebuilding_started=time.monotonic()
                    self.emit("rebuilding",{"physical_CAD":source.config.get("bracket_geometry") in ("parametric_photo_reference_v1","parametric_vertical_v2","parametric_side_mount_v3","parametric_side_bend_v4"),
                                             "requests":{side:{"length_mm":length,"plate_tilt_deg":tilt} for side,(length,tilt) in pending_mounts.items()},
                                             "previous_frame_retained":self.latest is not None})
                for side,(length,tilt) in pending_mounts.items():
                    source.set_mount(side,length_mm=length,tilt_deg=tilt)
                if self.stop_requested.is_set():
                    break
                packet=freeze_packet(source.step_and_capture())
                new_latest=(packet,copy.deepcopy(source.config))
                if pending_mounts:
                    # Announce completion only after a frame from the new CAD
                    # is ready; the old frame/config pair remains untouched
                    # throughout rebuilding and renderer replacement.
                    self.emit("rebuilt",{"elapsed_seconds":time.monotonic()-rebuilding_started,
                                          "stamp_ns":packet.get("stamp_ns"),"config":copy.deepcopy(source.config),
                                          "geometry_status":source.config.get("geometry_status","unspecified")},new_latest=new_latest)
                else:
                    with self.lock:
                        self.latest=new_latest
                self.wake_requested.wait(max(0.,1/self.args.rate-(time.monotonic()-start)))
                self.wake_requested.clear()
        except Exception as exc:
            self.emit("error",f"{type(exc).__name__}: {exc}")
        finally:
            if source is not None:
                source.close()


class CameraWindow:
    def __init__(self,args):
        import tkinter as tk
        from tkinter import ttk,messagebox
        from PIL import ImageTk
        self.tk,self.ttk,self.messagebox,self.ImageTk=tk,ttk,messagebox,ImageTk
        self.args=args
        self.root=tk.Tk()
        self.root.title("TRON2 · 三相机 RGB-D")
        self.root.geometry("1290x1000")
        self.root.minsize(930,760)
        self.root.configure(bg="#edf2f7")
        self.root.protocol("WM_DELETE_WINDOW",self.close)
        self.current=None
        self.last_stamp=None
        self.images={}
        self.variables={}
        self.parameter_captions={}
        self.scales={}
        self.mount_jobs={}
        self.initializing=True
        self.rebuilding=False
        self.notice_until=0.
        top=ttk.Frame(self.root,padding=12)
        top.pack(fill="x")
        ttk.Label(top,text="TRON2 · 三相机 RGB-D",font=("Sans",18,"bold")).pack(side="left")
        ttk.Button(top,text="保存截图与原始深度",command=self.save_images).pack(side="right",padx=5)
        ttk.Button(top,text="保存相机参数",command=self.save_config).pack(side="right",padx=5)
        self.banner=tk.Label(self.root,text=geometry_banner(None),bg="#fff0d0",fg="#71420a",anchor="w",padx=18,pady=9,wraplength=1200)
        self.banner.pack(fill="x",padx=12)
        grid=ttk.Frame(self.root,padding=(12,8))
        grid.pack(fill="both",expand=True)
        self.panels={}
        self.depth_labels={}
        self.depth_ranges={name:(args.depth_min or .1,args.depth_max or 3.) for name in CAMERAS}
        for column,name in enumerate(CAMERAS):
            grid.columnconfigure(column,weight=1)
            panel=ttk.LabelFrame(grid,text=CAMERA_LABELS[name],padding=7)
            panel.grid(row=0,column=column,sticky="nsew",padx=4)
            for row,label in [(0,"RGB"),(1,"深度色图 / 米")]:
                caption=ttk.Label(panel,text=label)
                caption.pack(anchor="w")
                if row==1:self.depth_labels[name]=caption
                image_panel=tk.Label(panel,bg="#172331",text="等待图像…",fg="white")
                image_panel.pack(fill="both",expand=True,pady=(3,9))
                self.panels[(name,row)]=image_panel
        grid.rowconfigure(0,weight=1)
        controls=ttk.Frame(self.root,padding=(16,6))
        controls.pack(fill="x")
        for column,side in enumerate(["left","right"]):
            controls.columnconfigure(column,weight=1)
            group=ttk.LabelFrame(controls,text=("左腕" if side=="left" else "右腕")+" · 安装参数",padding=8)
            group.grid(row=0,column=column,sticky="ew",padx=5)
            group.columnconfigure(1,weight=1)
            values={"length":tk.DoubleVar(value=80.),"tilt":tk.DoubleVar(value=40.)}
            self.variables[side]=values
            self.scales[side]={}
            for row,(key,label,lo,hi) in enumerate([("length","支撑杆长度 (mm)",20,150),("tilt","相机连接板倾角 (°)",0,75)]):
                caption=ttk.Label(group,text=label)
                caption.grid(row=row,column=0,sticky="w",padx=(0,10))
                self.parameter_captions[(side,key)]=caption
                scale=tk.Scale(group,from_=lo,to=hi,resolution=1,orient="horizontal",variable=values[key],command=lambda _value,s=side:self.queue_mount(s),length=360,bg="#edf2f7",highlightthickness=0,state="disabled")
                scale.grid(row=row,column=1,sticky="ew")
                self.scales[side][key]=scale
        self.status=tk.StringVar(value="正在启动本地相机仿真；首次生成/加载 CAD 可能需要若干秒…")
        ttk.Label(self.root,textvariable=self.status,padding=(17,10),wraplength=1200).pack(fill="x")
        self.worker=CaptureWorker(args)
        self.worker.start()
        self.root.after(60,self.poll)

    def queue_mount(self,side):
        if self.initializing:
            return
        if side in self.mount_jobs:
            self.root.after_cancel(self.mount_jobs[side])
        self.mount_jobs[side]=self.root.after(140,lambda:self.apply_mount(side))

    def apply_mount(self,side):
        self.mount_jobs.pop(side,None)
        values=self.variables[side]
        self.worker.commands.put((side,float(values["length"].get()),float(values["tilt"].get())))
        self.worker.wake_requested.set()

    def poll(self):
        try:
            while True:
                kind,payload=self.worker.events.get_nowait()
                if kind=="ready":
                    self.banner.configure(text=geometry_banner(payload))
                    limits=mount_parameter_limits(payload)
                    for name in CAMERAS:
                        lo,hi=camera_depth_limits(payload,name,self.args.depth_min,self.args.depth_max)
                        self.depth_ranges[name]=(lo,hi)
                        self.depth_labels[name].configure(text=f"深度色图 / {lo:g}–{hi:g} m")
                    for side in ["left","right"]:
                        self.parameter_captions[(side,"tilt")].configure(text="板与支撑杆夹角 (°)" if payload.get("bracket_geometry")=="parametric_side_bend_v4" else "相机连接板倾角 (°)")
                        self.parameter_captions[(side,"length")].configure(text="蓝色竖段长度 (mm)" if payload.get("bracket_geometry")=="parametric_vertical_v2" else "支撑杆长度 (mm)")
                        for key,config_key in (("length","length_mm"),("tilt","plate_tilt_deg")):
                            lo,hi=limits[config_key]
                            initial=float(payload["wrists"][side][config_key])
                            if not lo<=initial<=hi:
                                raise ValueError(f"{side} {config_key}={initial} is outside GUI limits [{lo},{hi}]")
                            self.scales[side][key].configure(from_=lo,to=hi,state="disabled" if payload.get('parameter_editable') is False else "normal")
                        self.variables[side]["length"].set(payload["wrists"][side]["length_mm"])
                        self.variables[side]["tilt"].set(payload["wrists"][side]["plate_tilt_deg"])
                        if payload.get('parameter_editable') is False:
                            self.parameter_captions[(side,'length')].configure(text='长度：由原始 STEP 实体决定')
                            self.parameter_captions[(side,'tilt')].configure(text='倾角：由原始 STEP 实体决定')
                            for scale in self.scales[side].values():scale.grid_remove()
                    self.initializing=False
                elif kind=="rebuilding":
                    self.rebuilding=True
                    changes="、".join(("左腕" if side=="left" else "右腕")+f" {values['length_mm']:g} mm / {values['plate_tilt_deg']:g}°" for side,values in payload["requests"].items())
                    action="重建 STEP/STL 并重载相机场景" if payload["physical_CAD"] else "更新相机安装坐标"
                    self.status.set(f"正在{action}：{changes}。保留上一帧图像，首次构型可能需要若干秒…")
                elif kind=="rebuilt":
                    self.rebuilding=False
                    self.banner.configure(text=geometry_banner(payload["config"]))
                    self.notice_until=time.monotonic()+5.
                    self.status.set(f"新构型与图像已就绪，更新耗时 {payload['elapsed_seconds']:.1f} 秒。")
                elif kind=="error":
                    self.rebuilding=False
                    self.notice_until=float("inf")
                    self.status.set("相机启动/采集失败："+payload)
                    self.messagebox.showerror("相机采集失败",payload)
        except queue.Empty:
            pass
        with self.worker.lock:
            latest=self.worker.latest
        if latest is not None and latest[0].get("stamp_ns")!=self.last_stamp:
            self.current=latest
            packet,_config=latest
            self.last_stamp=packet.get("stamp_ns")
            for name in CAMERAS:
                frame=packet["frames"][name]
                for row,img in [(0,Image.fromarray(frame["rgb"],"RGB")),(1,depth_color(frame["depth"],*self.depth_ranges[name]))]:
                    width=max(250,min(420,self.panels[(name,row)].winfo_width()))
                    img=img.resize((width,round(width*img.height/img.width)),Image.Resampling.BILINEAR if row==0 else Image.Resampling.NEAREST)
                    photo=self.ImageTk.PhotoImage(img)
                    self.images[(name,row)]=photo
                    self.panels[(name,row)].configure(image=photo,text="")
            if not self.rebuilding and time.monotonic()>self.notice_until:
                self.status.set(f"本地仿真时间 {packet.get('stamp_ns',0)/1e9:.3f} s · 三路RGB与米制深度 · 黑色深度像素为无效值 · 保存参数写入 {self.args.user_config.name}")
        if not self.worker.stop_requested.is_set():
            self.root.after(70,self.poll)

    def save_images(self):
        if self.current is None:
            self.messagebox.showinfo("尚无图像","等待第一组相机图像后再保存。")
            return
        path=self.args.output/datetime.now().strftime("snapshot_%Y%m%d_%H%M%S_%f")
        try:
            result=save_snapshot(*self.current,path,self.args.depth_min,self.args.depth_max)
            self.notice_until=time.monotonic()+6.
            self.status.set(f"截图、三路RGB和原始米制深度已保存：{result.parent}")
        except Exception as exc:
            self.messagebox.showerror("保存截图失败",str(exc))

    def save_config(self):
        if self.current is None:
            self.messagebox.showinfo("尚无参数","等待相机启动后再保存。")
            return
        # Save the values associated with the displayed captured bundle. Flush
        # pending slider edits first so saved and displayed parameters agree.
        for side in list(self.mount_jobs):
            self.root.after_cancel(self.mount_jobs[side])
            self.apply_mount(side)
        requested={side:(float(self.variables[side]["length"].get()),float(self.variables[side]["tilt"].get())) for side in ["left","right"]}
        self.status.set("正在等待所选参数完成 CAD 重建与新图像采集，随后保存…")
        self._save_config_when_applied(requested,time.monotonic()+REBUILD_TIMEOUT_S)

    def _save_config_when_applied(self,requested,deadline):
        with self.worker.lock:
            latest=self.worker.latest
        if latest is None:
            return
        config=latest[1]
        if any(abs(config["wrists"][s]["length_mm"]-v[0])>1e-6 or abs(config["wrists"][s]["plate_tilt_deg"]-v[1])>1e-6 for s,v in requested.items()):
            if time.monotonic()<deadline:
                self.root.after(100,lambda:self._save_config_when_applied(requested,deadline))
            else:
                self.messagebox.showerror("参数尚未应用","等待采集状态更新后重试保存。")
            return
        try:
            self.args.user_config.parent.mkdir(parents=True,exist_ok=True)
            self.args.user_config.write_text(json_text(config),encoding="utf-8")
            self.notice_until=time.monotonic()+6.
            self.status.set(f"已保存独立参数文件：{self.args.user_config}")
        except Exception as exc:
            self.messagebox.showerror("保存参数失败",str(exc))

    def close(self):
        self.worker.stop_requested.set()
        self.worker.wake_requested.set()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        self.worker.stop_requested.set()
        self.worker.wake_requested.set()
        self.worker.join(timeout=REBUILD_TIMEOUT_S)


def wait_for_worker(worker,predicate,timeout=REBUILD_TIMEOUT_S):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            while True:
                kind,payload=worker.events.get_nowait()
                if kind=="error":raise RuntimeError(payload)
        except queue.Empty:
            pass
        with worker.lock:
            latest=worker.latest
        if latest is not None and predicate(*latest):
            return latest
        if not worker.is_alive():raise RuntimeError("Capture worker stopped before producing the requested frame")
        time.sleep(.025)
    raise TimeoutError(f"Timed out after {timeout:g} s waiting for real CAD rebuilding / camera capture")


def image_change(before,after):
    result={}
    for name in CAMERAS:
        a,b=before["frames"][name],after["frames"][name]
        rgb_changed=np.any(a["rgb"]!=b["rgb"],axis=-1)
        da,db=a["depth"],b["depth"]
        va,vb=np.isfinite(da),np.isfinite(db)
        common=va&vb
        difference=np.abs(da[common]-db[common])
        result[name]={"changed_rgb_pixels":int(rgb_changed.sum()),"rgb_mean_absolute_change":float(np.abs(a["rgb"].astype(float)-b["rgb"].astype(float)).mean()),"changed_depth_validity_pixels":int((va!=vb).sum()),"changed_common_depth_pixels":int((difference>1e-6).sum()),"mean_common_depth_change_m":float(difference.mean()) if difference.size else None}
    return result


def run_offscreen_check(args):
    """Exercise the same worker and command queue as the GUI, without Tk."""
    worker=CaptureWorker(args)
    config_path=args.config or ROOT/"config/camera_rig.json"
    config_hash_before=hashlib.sha256(config_path.read_bytes()).hexdigest()
    worker.start()
    try:
        baseline=wait_for_worker(worker,lambda packet,_config:True)
        baseline_path=save_snapshot(*baseline,args.output/"viewer_check",args.depth_min,args.depth_max)
        if baseline[1].get('parameter_editable') is False:
            report={'status':'fixed_STEP_capture_verified','opened_gui':False,'capture_worker_used':True,
                    'baseline_montage':str(baseline_path),'live_mount_edits':'disabled_for_fixed_geometry',
                    'input_config_unchanged':hashlib.sha256(config_path.read_bytes()).hexdigest()==config_hash_before}
            assert report['input_config_unchanged']
            (args.output/'viewer_check'/'mount_controls_check.json').write_text(json_text(report))
            print(json_text(report))
            return
        requested={}
        limits=mount_parameter_limits(baseline[1])
        for side in ["left","right"]:
            original=baseline[1]["wrists"][side]
            lo,hi=limits["length_mm"]
            candidate=float(original["length_mm"])+15
            if candidate>hi:candidate=float(original["length_mm"])-15
            if not lo<=candidate<=hi:raise ValueError("Configured length range is too narrow for the live +15/-15 mm check")
            requested[side]=(candidate,float(original["plate_tilt_deg"]))
            worker.commands.put((side,*requested[side]))
        worker.wake_requested.set()
        def applied(packet,config):
            return all(abs(config["wrists"][side]["length_mm"]-values[0])<1e-7 and abs(config["wrists"][side]["plate_tilt_deg"]-values[1])<1e-7 for side,values in requested.items())
        length=wait_for_worker(worker,applied)
        length_path=save_snapshot(*length,args.output/"viewer_length_check",args.depth_min,args.depth_max)
        for side in ["left","right"]:
            length_value,tilt_value=requested[side]
            lo,hi=limits["plate_tilt_deg"]
            candidate=tilt_value+12
            if candidate>hi:candidate=tilt_value-12
            if not lo<=candidate<=hi:raise ValueError("Configured tilt range is too narrow for the live +12/-12 degree check")
            requested[side]=(length_value,candidate)
            worker.commands.put((side,*requested[side]))
        worker.wake_requested.set()
        tilted=wait_for_worker(worker,applied)
        tilt_path=save_snapshot(*tilted,args.output/"viewer_tilt_check",args.depth_min,args.depth_max)
        length_changes=image_change(baseline[0],length[0])
        tilt_changes=image_change(length[0],tilted[0])
        if any(changes[name]["changed_rgb_pixels"]==0 for changes in [length_changes,tilt_changes] for name in ["left_wrist","right_wrist"]):
            raise AssertionError("At least one wrist RGB view did not change after a live length/plate-tilt command")
        config_hash_after=hashlib.sha256(config_path.read_bytes()).hexdigest()
        if config_hash_after!=config_hash_before:
            raise AssertionError("The check changed the input configuration file")
        with worker.lock:
            lifecycle=copy.deepcopy(worker.lifecycle)
        report={"status":"real_offscreen_capture_and_live_mount_commands_verified","opened_gui":False,"capture_worker_used":True,"baseline_montage":str(baseline_path),"length_montage":str(length_path),"tilt_montage":str(tilt_path),"baseline_parameters":baseline[1]["wrists"],"length_parameters":length[1]["wrists"],"tilt_parameters":tilted[1]["wrists"],"length_only_image_changes":length_changes,"tilt_only_image_changes":tilt_changes,"input_config_path":str(config_path),"input_config_sha256_before":config_hash_before,"input_config_sha256_after":config_hash_after,"saved_user_config":False,"geometry_status":tilted[1].get("geometry_status","unspecified"),"bracket_geometry":tilted[1].get("bracket_geometry","unspecified"),"geometry_status_by_stage":{stage:config.get("geometry_status","unspecified") for stage,(_packet,config) in (("baseline",baseline),("length",length),("tilt",tilted))},"parameter_limits":limits,"rebuild_wait_timeout_seconds":REBUILD_TIMEOUT_S,"capture_lifecycle":lifecycle}
        (args.output/"viewer_check"/"mount_controls_check.json").write_text(json_text(report),encoding="utf-8")
        print(json_text(report))
    finally:
        worker.stop_requested.set()
        worker.wake_requested.set()
        worker.join(timeout=REBUILD_TIMEOUT_S)
        if worker.is_alive():raise RuntimeError("Capture worker did not close after check")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,help="Camera rig config; uses a saved user config if present, otherwise module defaults")
    parser.add_argument("--user-config",type=Path,default=DEFAULT_USER_CONFIG,help="Save target; never overwrites the default rig config")
    parser.add_argument("--width",type=int,default=320)
    parser.add_argument("--height",type=int,default=240)
    parser.add_argument("--rate",type=float,default=8.)
    parser.add_argument("--depth-min",type=float,help="Override all depth colormap minima in meters; use with --depth-max")
    parser.add_argument("--depth-max",type=float,help="Override all depth colormap maxima; defaults use each camera's range")
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/cameras")
    parser.add_argument("--check",action="store_true",help="Verify real offscreen captures and live length/plate-tilt commands; opens no GUI")
    args=parser.parse_args()
    if args.width<64 or args.height<64 or not np.isfinite(args.rate) or args.rate<=0:
        parser.error("Invalid dimensions, rate, or metric depth display bounds")
    if (args.depth_min is None)!=(args.depth_max is None):
        parser.error("Set both --depth-min and --depth-max, or neither")
    if args.depth_min is not None and (not np.isfinite([args.depth_min,args.depth_max]).all() or args.depth_min<0 or args.depth_max<=args.depth_min):
        parser.error("Invalid metric depth display bounds")
    if args.user_config.resolve()==(ROOT/"config/camera_rig.json").resolve():
        parser.error("--user-config must be an independent file, not config/camera_rig.json")
    if args.config is None and args.user_config.is_file():
        args.config=args.user_config
    if args.check:
        run_offscreen_check(args)
    else:
        CameraWindow(args).run()


if __name__=="__main__":
    main()
