"""Independent 2D solid intersections for the proposed local lobe rotation."""
from extract_wrist_features import ROOT,OUT,wrist_mesh,section_polylines
import numpy as np
import json
import trimesh
import manifold3d
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

adapter=trimesh.load_mesh(ROOT/"meshes/adapter/adapter_visual.stl",process=False)
adapter.apply_scale(1000)
rotated=adapter.copy()
rotated.vertices=rotated.vertices@Rotation.from_euler("y",-15,degrees=True).as_matrix().T

def polygons(mesh,y):
    lines=section_polylines(mesh,1,y)
    closed=[p[:,[0,2]] for p in lines if np.linalg.norm(p[0]-p[-1])<1.e-6]
    if len(closed)!=len(lines):
        raise ValueError(f"Unclosed section at Y={y}")
    return manifold3d.CrossSection(closed,manifold3d.FillRule.EvenOdd)

angles=np.deg2rad(np.linspace(22.766,37.234,100))
wedge=manifold3d.CrossSection([np.vstack([[0,0],40*np.column_stack([np.cos(angles),np.sin(angles)]),[0,0]])])
report={"scope":"original production adapter at scale1.01, negative-Y sections only; theta+15 degrees lobe rotation; no axial hole shift applied", "key_sector_deg":[22.766,37.234],"sampled_y_mm":[-.01,-.1,-.3,-.5,-1,-2,-3,-4.5,-6,-8,-10,-10.39,-10.4],"sides":{}}
fig,axes=plt.subplots(2,3,figsize=(15,10),constrained_layout=True)
for row,side in enumerate(["right","left"]):
    wrist=wrist_mesh(side)
    results=[]
    for y in report["sampled_y_mm"]:
        a=polygons(adapter,y)
        b=polygons(rotated,y)
        w=polygons(wrist,y)
        intersection=b^w
        key=intersection^wedge
        results.append({"y_mm":y,"rotated_key_sector_overlap_area_mm2":key.area(),"rotated_full_section_overlap_area_mm2":intersection.area(),"original_full_section_overlap_area_mm2":(a^w).area(),"rotated_key_sector_adapter_area_mm2":(b^wedge).area()})
        if y in [-.5,-4.5,-10]:
            col=[-.5,-4.5,-10].index(y)
            ax=axes[row,col]
            for model,color,label in [(a,"#c0c0c0","Original adapter"),(b,"#ba7c32","Rotated lower lobes"),(w,"#2f6898","Official wrist")]:
                for i,p in enumerate(model.to_polygons()):
                    ax.plot(p[:,0],p[:,1],c=color,lw=1,label=label if i==0 else None)
            for p in intersection.to_polygons():
                ax.fill(p[:,0],p[:,1],color="red",alpha=.6)
            ax.set(title=f"{side}: Y={y:g}; key overlap={key.area():.3f} mm2",xlabel="Adapter X [mm]",ylabel="Adapter Z [mm]",xlim=(17,32),ylim=(4,22))
            ax.set_aspect("equal")
            ax.grid(alpha=.25)
            ax.legend(fontsize=7,loc="lower left")
    report["sides"][side]=results
fig.suptitle("Local lower-lobe theta+15 degree trial; 1.01 scale; no production files modified")
fig.savefig(OUT/"rotated_lobes_key_clearance.png",dpi=190)
(OUT/"rotated_lobes_check.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
print(json.dumps(report,indent=2))
