#!/usr/bin/env python3
"""Render the actual revision-2 meshes from dorsal and wrist-facing views."""
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'design/revision2'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    size=(880,720)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    canvas=Image.new('RGB',(size[0]*2,size[1]*2+170),(239,242,246))
    draw=ImageDraw.Draw(canvas)
    draw.text((28,20),'TRON2 / Revo3 adapter revision 2',(26,39,54),font=font)
    draw.text((28,57),'Actual STL meshes | Identical camera viewpoints in each row expose the left/right difference',(62,75,91),font=small)
    inputs={}
    for col,side in enumerate(('left','right')):
        path=OUT/f'adapter_{side}_v2_m.stl'
        inputs[side]={'path':str(path.relative_to(ROOT)),'sha256':sha(path)}
        # Rotate the display camera around the right part so that each back
        # slot faces the viewer. These display rotations never enter URDF.
        xml=f'''<mujoco><compiler angle="radian"/><visual><global offwidth="880" offheight="720"/><headlight ambient="0.45 0.45 0.45" diffuse="0.7 0.7 0.7"/><map znear="0.0001"/></visual><asset><mesh name="adapter" file="{path}"/><texture name="sky" type="skybox" builtin="gradient" rgb1="0.8 0.85 0.9" rgb2="0.94 0.96 0.98" width="64" height="256"/></asset><worldbody><light pos="0.1 -0.2 0.2" dir="-0.3 0.5 -1" diffuse="0.7 0.7 0.7"/><body euler="1.5707963267948966 0 0"><geom type="mesh" mesh="adapter" contype="0" conaffinity="0" rgba="0.90 0.43 0.13 1"/></body></worldbody></mujoco>'''
        model=mujoco.MjModel.from_xml_string(xml)
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        with mujoco.Renderer(model,height=size[1],width=size[0]) as renderer:
            for row,elevation in enumerate((-12,-12)):
                camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
                camera.lookat[:]=[0,0,.011]
                camera.distance=.155
                camera.azimuth=105 if row==0 else -75
                camera.elevation=elevation
                renderer.update_scene(data,camera)
                image=Image.fromarray(renderer.render())
                x,y=col*size[0],110+row*size[1]
                canvas.paste(image,(x,y))
                open_slot=(side=='left' and row==0) or (side=='right' and row==1)
                draw.text((x+25,y+18),f'{side.upper()} | '+('Dorsal cable slot: OPEN' if open_slot else 'Palm-facing side: CLOSED'),(20,32,45),font=small)
    draw.text((28,canvas.height-43),'Wrist/hand mount frames unchanged. Only lower petals re-indexed; right original slot filled.',(62,75,91),font=small)
    output=OUT/'adapter_pair.png';canvas.save(output)
    (OUT/'adapter_pair_render.json').write_text(json.dumps({'producer_sha256':sha(Path(__file__)),'inputs':inputs,'output_sha256':sha(output),'method':'MuJoCo EGL render of actual meshes; identical camera viewpoint across both columns of each row'},indent=2)+'\n')
    print(output)


if __name__=='__main__':main()
