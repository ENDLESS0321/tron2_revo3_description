"""Independent read-only section check of configured wrist/adapter geometry."""
from inspect_adapter import OUT
import json
import numpy as np
import matplotlib.pyplot as plt
import trimesh
import manifold3d
from scipy.spatial.transform import Rotation

ROOT=OUT.parent

def radial_intervals(polygons, angle):
    direction=np.array([np.cos(angle),np.sin(angle)])
    hits=[]
    for polygon in polygons:
        start=polygon[:-1]
        delta=polygon[1:]-start
        cross=direction[0]*delta[:,1]-direction[1]*delta[:,0]
        mask=np.abs(cross)>1.e-10
        distance=np.divide(start[:,0]*delta[:,1]-start[:,1]*delta[:,0],cross,out=np.zeros(len(start)),where=mask)
        fraction=np.divide(start[:,0]*direction[1]-start[:,1]*direction[0],cross,out=np.zeros(len(start)),where=mask)
        hits.extend(distance[mask&(fraction>=0)&(fraction<1)&(distance>0)])
    hits=sorted(hits)
    unique=[]
    for hit in hits:
        if not unique or hit-unique[-1]>1.e-7:
            unique.append(float(hit))
    return [(unique[j],unique[j+1]) for j in range(0,len(unique)-1,2)]

config=json.loads((ROOT/"config/assembly.json").read_text())
adapter=trimesh.load_mesh(ROOT/"meshes/adapter/adapter_visual.stl",process=False)
adapter.merge_vertices(digits_vertex=12)
adapter.apply_scale(1000)
results={}
for side, letter in [("right","R"),("left","L")]:
    wrist=trimesh.load_mesh(ROOT/f"meshes/tron2/wrist_roll_{letter}_Link.STL",process=False)
    wrist.merge_vertices(digits_vertex=12)
    mount=config[side]["wrist_to_adapter"]
    rotation=Rotation.from_euler("xyz",mount["rpy_rad"]).as_matrix()
    wrist.vertices=(wrist.vertices-np.array(mount["xyz_m"]))@rotation*1000
    report={"wrist_to_adapter":mount,"wrist_bounds_adapter_frame_mm":wrist.bounds.tolist(),"wrist_is_volume":bool(wrist.is_volume),"sections":[]}
    fig,axes=plt.subplots(2,4,figsize=(19,10),constrained_layout=True)
    selected=[(0,0,[2,1]),(2,0,[0,1]),(1,-0.05,[0,2]),(1,-1,[0,2]),(1,-3,[0,2]),(1,-5,[0,2]),(1,-8,[0,2]),(1,-10,[0,2])]
    for ax,(axis,coord,others) in zip(axes.flat,selected):
        record={"axis":"xyz"[axis],"coordinate_mm":coord,"meshes":{}}
        polygons={}
        for mesh,color,label in [(adapter,"#b77232","Adapter"),(wrist,"#2d70af",f"TRON2 {side} wrist")]:
            normal,origin=np.zeros(3),np.zeros(3)
            normal[axis]=1
            origin[axis]=coord
            section=mesh.section(plane_normal=normal,plane_origin=origin)
            entities=[]
            closed=[]
            if section is not None:
                for index,entity in enumerate(section.entities):
                    pts=section.vertices[entity.points][:,others]
                    if len(pts)<3 or np.ptp(pts,axis=0).max()<1.e-5:
                        continue
                    ax.plot(pts[:,0],pts[:,1],c=color,lw=.75,label=label if not entities else None)
                    fit=np.linalg.lstsq(np.column_stack([2*pts,np.ones(len(pts))]),np.sum(pts**2,axis=1),rcond=None)[0]
                    center=fit[:2]
                    radius=float(np.sqrt(max(0,fit[2]+np.dot(center,center))))
                    error=float(np.max(np.abs(np.linalg.norm(pts-center,axis=1)-radius)))
                    item={"bounds_mm":[pts.min(axis=0).tolist(),pts.max(axis=0).tolist()],"circle_fit_center_mm":center.tolist(),"circle_fit_radius_mm":radius,"circle_fit_max_error_mm":error,"points":len(pts)}
                    item["closed"] = bool(np.linalg.norm(pts[0]-pts[-1]) < 1.e-5)
                    if item["closed"]:
                        closed.append(pts)
                    entities.append(item)
                    if error<.03 and len(pts)>15 and radius>.5:
                        ax.annotate(f"r={radius:.2f}",center,fontsize=6,c=color)
            record["meshes"][label]=entities
            polygons[label]=closed
        if axis==1 and all(polygons.values()):
            solids=[manifold3d.CrossSection(p,manifold3d.FillRule.EvenOdd) for p in polygons.values()]
            intersection=solids[0]^solids[1]
            record["closed_contour_intersection_area_mm2"]=intersection.area()
            record["adapter_section_area_mm2"]=solids[0].area()
            largest={"radial_overlap_mm":0.,"angle_deg":None}
            contours=list(polygons.values())
            for degrees in np.arange(.13,360,.5):
                intervals=[radial_intervals(p,np.deg2rad(degrees)) for p in contours]
                for first in intervals[0]:
                    for second in intervals[1]:
                        width=max(0.,min(first[1],second[1])-max(first[0],second[0]))
                        if width>largest["radial_overlap_mm"]:
                            largest={"radial_overlap_mm":width,"angle_deg":float(degrees)}
            record["largest_radial_overlap_sampled_every_half_degree"]=largest
            for poly in intersection.to_polygons():
                ax.fill(poly[:,0],poly[:,1],c="red",alpha=.7)
        ax.set(title=f"{side} mount: adapter {'XYZ'[axis]}={coord:g} mm",xlabel=f"Adapter {'XYZ'[others[0]]} [mm]",ylabel=f"Adapter {'XYZ'[others[1]]} [mm]")
        if axis!=1:
            ax.axhline(0,c="green",ls="--",lw=.7,label="Shoulder datum")
            ax.set(xlim=(-45,45),ylim=(-55,40))
        else:
            ax.set(xlim=(-43,43),ylim=(-33,33))
        ax.set_aspect("equal")
        ax.grid(alpha=.2)
        ax.legend(fontsize=7,loc="upper right")
        report["sections"].append(record)
    fig.suptitle("Configured wrist/adapter geometry: source visual meshes; millimeters; shoulder datum Y=0")
    fig.savefig(OUT/f"{side}_wrist_fit_sections.png",dpi=175)
    plt.close(fig)
    if wrist.is_volume:
        try:
            overlap=trimesh.boolean.intersection([adapter,wrist],engine="manifold",check_volume=True)
            report["visual_solid_intersection_volume_mm3"]=float(overlap.volume)
            report["intersection_bounds_mm"]=overlap.bounds.tolist()
        except Exception as exc:
            report["intersection_error"]=str(exc)
    results[side]=report
(OUT/"wrist_fit_geometry.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
print(json.dumps({side:{k:v for k,v in d.items() if k!="sections"} for side,d in results.items()},indent=2))
