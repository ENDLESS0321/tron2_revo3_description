#!/usr/bin/env python3
"""View only the two revision-2 printable STL solids, without the robot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PARTS = ROOT / "design/revision2"
SPACING = 0.11
VIEWS = {"front": (105, -12), "back": (-75, -12), "top": (90, -89), "wrist": (105, 65)}
MODES = {ord("1"): "left", ord("2"): "right", ord("3"): "both"}
VIEW_KEYS = {ord("F"): "front", ord("B"): "back", ord("T"): "top", ord("U"): "wrist"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_model(mujoco, np, trimesh):
    manifest = json.loads((PARTS / "design_manifest.json").read_text())
    robot = ET.Element("mujoco", model="Printed adapters v2 | LEFT orange / RIGHT blue")
    ET.SubElement(robot, "compiler", angle="radian")
    ET.SubElement(robot, "option", gravity="0 0 0")
    visual = ET.SubElement(robot, "visual")
    ET.SubElement(visual, "global", offwidth="1200", offheight="850")
    ET.SubElement(visual, "headlight", ambient="0.4 0.4 0.4", diffuse="0.7 0.7 0.7")
    ET.SubElement(visual, "map", znear="0.0001", zfar="10")
    assets = ET.SubElement(robot, "asset")
    ET.SubElement(assets, "texture", name="sky", type="skybox", builtin="gradient", rgb1="0.75 0.81 0.88", rgb2="0.94 0.96 0.98", width="64", height="256")
    world = ET.SubElement(robot, "worldbody")
    ET.SubElement(world, "light", pos="0.1 -0.2 0.3", dir="-0.3 0.5 -1", diffuse="0.7 0.7 0.7")
    info = {}
    for index, side in enumerate(("left", "right")):
        path = PARTS / f"adapter_{side}_v2_print_mm.stl"
        actual = digest(path)
        if actual != manifest["sides"][side]["files"][path.name]:
            raise ValueError(f"打印件文件与清单不符：{path}")
        mesh = trimesh.load_mesh(path, process=False)
        bounds = mesh.bounds * .001
        center = (bounds[0] + bounds[1]) / 2
        offset_x = (index - .5) * SPACING
        shift = [-center[0], -center[1], -bounds[0, 2]]
        ET.SubElement(assets, "mesh", name=f"{side}_print_mesh", file=str(path), scale="0.001 0.001 0.001")
        body = ET.SubElement(world, "body", name=f"{side}_printed_adapter", pos=f"{offset_x} 0 0")
        ET.SubElement(body, "geom", name=f"{side}_printed_solid", type="mesh", mesh=f"{side}_print_mesh",
                      pos=" ".join(map(str, shift)), contype="0", conaffinity="0", group=str(index+1),
                      rgba="0.94 0.43 0.12 1" if side=="left" else "0.16 0.52 0.85 1")
        info[side] = {"file": str(path.relative_to(ROOT)), "sha256": actual,
                      "camera_center_m": [offset_x, 0, float(center[2]-bounds[0, 2])], "geom_group": index+1}
    model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if model.nq != 0 or model.ngeom != 2 or model.nbody != 3:
        raise ValueError("打印件查看场景应仅有两个静态实体")
    return model, data, info


def set_mode(opt, mode):
    opt.geomgroup[:] = [0, mode in ("left", "both"), mode in ("right", "both"), 0, 0, 0]


def set_camera(cam, mode, view, info):
    cam.lookat[:] = [0, 0, info["left"]["camera_center_m"][2]] if mode=="both" else info[mode]["camera_center_m"]
    cam.distance = .32 if mode=="both" else .165
    cam.azimuth, cam.elevation = VIEWS[view]


def check(mujoco, np, model, data, info):
    from PIL import Image
    output = ROOT / "reports/revision2/parts_viewer"
    output.mkdir(parents=True, exist_ok=True)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    rows = []
    with mujoco.Renderer(model, height=850, width=1200) as renderer:
        for mode in ("both", "left", "right"):
            for view in VIEWS:
                set_mode(opt, mode)
                set_camera(camera, mode, view, info)
                renderer.update_scene(data, camera=camera, scene_option=opt)
                visible = {int(g.objid) for g in renderer.scene.geoms[:renderer.scene.ngeom] if g.objtype==mujoco.mjtObj.mjOBJ_GEOM}
                expected = {0,1} if mode=="both" else ({0} if mode=="left" else {1})
                if visible != expected:
                    raise ValueError(f"{mode}/{view} 的显示实体不正确：{visible}")
                pixels = renderer.render()
                if not np.isfinite(pixels).all() or np.std(pixels) < 8:
                    raise ValueError("渲染结果异常或为空")
                image_path = output / f"{mode}_{view}.png"
                Image.fromarray(pixels).save(image_path)
                rows.append({"mode": mode, "view": view, "visible_geom_ids": sorted(visible), "image": str(image_path.relative_to(ROOT)), "sha256": digest(image_path)})
    report = {"status": "pass", "scope": "Model loading and offscreen rendering of all part/view combinations; desktop interaction not launched",
              "producer_sha256": digest(Path(__file__)), "parts": info, "nq": model.nq, "ngeom": model.ngeom, "views": rows}
    (output / "viewer_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"只看打印件模式检查通过：{len(rows)} 个组合。结果：{output}")


def interactive(mujoco, model, data, info, mode):
    import mujoco.viewer
    state = {"mode": mode, "view": "front", "dirty": True}
    state_lock = threading.Lock()

    def keyboard(key):
        with state_lock:
            if key in MODES:
                state["mode"] = MODES[key]
            elif key in VIEW_KEYS:
                state["view"] = VIEW_KEYS[key]
            elif key == ord("R"):
                state["view"] = "front"
            else:
                return
            state["dirty"] = True

    print("只看打印件：左件橙色，右件蓝色。")
    print("1=左件  2=右件  3=并排；F/B=两侧视图  T=手侧俯视  U=腕侧仰视  R=复位视角。")
    print("鼠标左键拖动旋转，右键拖动平移，滚轮缩放；关闭窗口退出。")
    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard, show_left_ui=False, show_right_ui=False) as viewer:
        while viewer.is_running():
            with state_lock:
                current = dict(state)
                state["dirty"] = False
            with viewer.lock():
                if current["dirty"]:
                    # MuJoCo also handles these keys (e.g. T transparency,
                    # number keys geom groups) before the Python callback.
                    # Restore the preset's options so those actions cannot
                    # accidentally hide a part or make it transparent.
                    mujoco.mjv_defaultOption(viewer.opt)
                    set_camera(viewer.cam, current["mode"], current["view"], info)
                set_mode(viewer.opt, current["mode"])
            if current["dirty"]:
                selected = {"both":"BOTH | LEFT orange / RIGHT blue", "left":"LEFT part | orange", "right":"RIGHT part | blue"}[current["mode"]]
                viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                 "Printed parts v2\nMode\nSelect\nViews\nMouse",
                                 f"\n{selected}\n1 left / 2 right / 3 both\nF front / B back / T top / U underside / R reset\nLeft drag: rotate | Right drag: pan | Wheel: zoom"))
            viewer.sync()
            time.sleep(.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--check", action="store_true", help="Run offscreen checks without opening a desktop window")
    args = parser.parse_args()
    os.environ["MUJOCO_GL"] = "egl" if args.check else "glfw"
    if not args.check and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("当前终端没有图形显示。请在本机桌面终端运行，或用 --check 输出静态图。")
    import mujoco
    import numpy as np
    import trimesh
    model, data, info = load_model(mujoco, np, trimesh)
    if args.check:
        check(mujoco, np, model, data, info)
    else:
        interactive(mujoco, model, data, info, args.side)


if __name__ == "__main__":
    main()
