"""Preview the generated CAD segments and actual mounting frames."""
from pathlib import Path
import json
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.collections import LineCollection

HERE=Path(__file__).resolve().parent
manifest=json.loads((HERE/"manifest.json").read_text())
colors={"wrist_mount":"#758698","support_arm":"#387cb0","camera_plate":"#d89945"}
meshes={name:trimesh.load_mesh(HERE/"parts"/(name+"_mm.stl"),process=False) for name in colors}
fig=plt.figure(figsize=(17,7),constrained_layout=True)
ax=fig.add_subplot(1,2,1,projection="3d")
allv=[]
for name,m in meshes.items():
    ax.add_collection3d(Poly3DCollection(m.triangles,facecolors=colors[name],alpha=1,shade=True))
    allv.append(m.vertices)
v=np.vstack(allv);lo=v.min(axis=0);hi=v.max(axis=0)
ax.set(xlim=(lo[0]-3,hi[0]+3),ylim=(lo[1]-3,hi[1]+3),zlim=(lo[2]-3,hi[2]+3),xlabel="X [mm]",ylabel="Y toward fingers [mm]",zlabel="Z outward [mm]")
ax.set_box_aspect(hi-lo)
ax.view_init(elev=22,azim=-53)
ax.set_title("Integral bracket: base / longitudinal arm / tilted plate")
ax2=fig.add_subplot(1,2,2)
for name,m in meshes.items():
    segments=trimesh.intersections.mesh_plane(m,plane_origin=[10.5,0,0],plane_normal=[1,0,0])
    if len(segments):
        ax2.add_collection(LineCollection(segments[:,:,[1,2]],colors=colors[name],linewidths=1.4,label=name))
p=np.array(manifest["plate_frame"]["xyz_mm"])
ax2.plot(p[1],p[2],"ro",ms=5,label="Plate pivot (X axis)")
for hole in manifest["features"]["camera_plate_holes"]:
    point=hole["center_root_mm"]
    ax2.plot(point[1],point[2],"kx",ms=6)
ax2.axvline(0,color="#777",ls="--",lw=.7)
ax2.set_aspect("equal")
ax2.autoscale()
ax2.set(xlabel="Y [mm]",ylabel="Z [mm]",title="Side-rib section X=10.5 mm; camera omitted")
ax2.grid(alpha=.2)
ax2.legend(fontsize=9,loc="upper left")
fig.suptitle(f"New wrist camera bracket | L={manifest['parameters']['length_mm']:g} mm, plate={manifest['parameters']['plate_tilt_deg']:g}° | Not strength-certified")
fig.savefig(HERE/"cad_preview.png",dpi=190)
