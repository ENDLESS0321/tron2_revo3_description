"""Cross-section visualization of the inferred shoulder-seated mounting pose."""
from inspect_adapter import OUT
import numpy as np
import matplotlib.pyplot as plt
import trimesh

ROOT = OUT.parent
adapter = trimesh.load_mesh(ROOT / "meshes/adapter/adapter_visual.stl", process=False)
adapter.apply_scale(1000)
hand = trimesh.load_mesh(ROOT / "vendor/brainco-revo3/revo3_system/meshes/hands/visual/right/base_link.STL", process=False)
hand.apply_scale(1000)
rotation = np.array([[0,1,0],[0,0,1],[1,0,0]])
hand.vertices = hand.vertices @ rotation.T + [0,23.855,0]
fig, axes = plt.subplots(1,3,figsize=(15,7),constrained_layout=True)
for ax, (axis, coord, others) in zip(axes, [(0,0,[2,1]),(2,0,[0,1]),(1,28.855,[0,2])]):
    for mesh, color, label in [(adapter,"#b77232","Printed adapter"),(hand,"#306daf","Official right palm")]:
        origin, normal = np.zeros(3), np.zeros(3)
        origin[axis]=coord
        normal[axis]=1
        section=mesh.section(plane_origin=origin,plane_normal=normal)
        first=True
        for entity in section.entities:
            points=section.vertices[entity.points]
            ax.plot(points[:,others[0]],points[:,others[1]],c=color,lw=1, label=label if first else None)
            first=False
    ax.set(xlabel=f"Adapter {'XYZ'[others[0]]} [mm]",ylabel=f"Adapter {'XYZ'[others[1]]} [mm]",title=f"Section {'XYZ'[axis]}={coord:g} mm")
    if axis != 1:
        ax.axhline(35.855,c="green",ls="--",lw=.8,label="Sleeve lip / hand shoulder")
        ax.set_ylim(-13,65)
        ax.set_xlim(-43,43)
    ax.set_aspect("equal")
    ax.grid(alpha=.2)
    ax.legend(fontsize=8,loc="upper right")
fig.suptitle("Inferred pose: hand base Y=23.855 mm; hand shoulder Z=12 mm on sleeve lip. Hole mismatch remains.")
fig.savefig(OUT / "mount_fit_sections.png",dpi=190)
