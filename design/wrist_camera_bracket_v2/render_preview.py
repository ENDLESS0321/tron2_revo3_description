"""Render V2's actual base, radial upright and top plate, without a camera."""
from pathlib import Path
import json
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE=Path(__file__).resolve().parent
m=json.loads((HERE/"manifest.json").read_text())
colors={"base":"#267bd1","stem":"#438dd8","plate":"#52bd84"}
meshes={name:trimesh.load_mesh(HERE/"parts"/(name+"_mm.stl"),process=False) for name in colors}
fig=plt.figure(figsize=(15,8),constrained_layout=True)
ax=fig.add_subplot(1,2,1,projection="3d")
vertices=[]
for name,mesh in meshes.items():
    ax.add_collection3d(Poly3DCollection(mesh.triangles,facecolors=colors[name],shade=True))
    vertices.append(mesh.vertices)
v=np.vstack(vertices);low=v.min(axis=0);high=v.max(axis=0)
ax.set(xlim=(low[0]-3,high[0]+3),ylim=(low[1]-3,high[1]+3),zlim=(low[2]-3,high[2]+3),xlabel="X [mm]",ylabel="Y [mm]",zlabel="Z radial upright [mm]")
ax.set_box_aspect(high-low)
ax.view_init(elev=19,azim=-55)
ax.set_title("V2: base + solid radial stem + directly connected plate")
side=fig.add_subplot(1,2,2)
for name,mesh in meshes.items():
    segments=trimesh.intersections.mesh_plane(mesh,plane_normal=[1,0,0],plane_origin=[10.5,0,0])
    if len(segments):side.add_collection(LineCollection(segments[:,:,[1,2]],colors=colors[name],linewidths=1.7,label=name))
p=m["plate_frame"]["xyz_mm"]
side.plot(p[1],p[2],"ro",ms=5,label="Plate pivot")
side.axhline(35.8,color="#555",ls="--",lw=.8,label="Length origin Z=35.8")
side.autoscale();side.set_aspect("equal");side.grid(alpha=.2)
side.set(xlabel="Y [mm]",ylabel="Z [mm]",title="Side section: no longitudinal arm")
side.legend(loc="lower right")
fig.suptitle(f"V2 | radial length {m['parameters']['length_mm']:g} mm | plate {m['parameters']['plate_tilt_deg']:g} degrees | camera separate")
fig.savefig(HERE/"cad_preview.png",dpi=190)
