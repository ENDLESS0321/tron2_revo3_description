"""Independently validate actual revision-2 STL holes, nut cavities and key fit."""
from extract_wrist_features import ROOT,OUT,wrist_mesh,section_polylines
from pathlib import Path
import hashlib
import json
import numpy as np
import trimesh
import manifold3d
import matplotlib.pyplot as plt

DESIGN=OUT.parent
AXIAL_SAMPLES=[-.01,-.1,-.3,-.5,-1,-2,-3,-4.5,-6,-8,-10]
KEY_RANGE=[22.766,37.234]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def closed_paths(mesh,y):
    paths=section_polylines(mesh,1,y)
    if any(np.linalg.norm(p[0]-p[-1])>1.e-5 for p in paths):
        raise ValueError(f"Open contour at Y={y}")
    return [p[:,[0,2]] for p in paths]

def section_solid(paths):
    return manifold3d.CrossSection(paths,manifold3d.FillRule.EvenOdd)

def intervals(paths,degrees):
    radians=np.deg2rad(degrees)
    direction=np.array([np.cos(radians),np.sin(radians)])
    hits=[]
    for polygon in paths:
        start=polygon[:-1]
        delta=polygon[1:]-start
        cross=direction[0]*delta[:,1]-direction[1]*delta[:,0]
        nonzero=np.abs(cross)>1.e-10
        distance=np.divide(start[:,0]*delta[:,1]-start[:,1]*delta[:,0],cross,out=np.zeros(len(start)),where=nonzero)
        fraction=np.divide(start[:,0]*direction[1]-start[:,1]*direction[0],cross,out=np.zeros(len(start)),where=nonzero)
        hits.extend(distance[nonzero&(fraction>=0)&(fraction<1)&(distance>0)])
    hits=sorted(hits)
    unique=[]
    for h in hits:
        if not unique or h-unique[-1]>1.e-6:
            unique.append(float(h))
    return [(unique[i],unique[i+1]) for i in range(0,len(unique)-1,2)]

def radial_loops(mesh,degrees,radial):
    radians=np.deg2rad(degrees)
    normal=np.array([np.cos(radians),0,np.sin(radians)])
    tangent=np.array([-np.sin(radians),0,np.cos(radians)])
    sec=mesh.section(plane_normal=normal,plane_origin=normal*radial)
    loops=[]
    if sec is None:
        return loops
    for entity in sec.entities:
        pts=sec.vertices[entity.points]
        if len(pts)<8 or np.linalg.norm(pts[0]-pts[-1])>1e-5:
            continue
        xy=np.column_stack([pts@tangent,pts[:,1]])
        lo,hi=xy.min(axis=0),xy.max(axis=0)
        if not (lo[0]>-6 and hi[0]<6 and lo[1]>-10 and hi[1]<0):
            continue
        fitted=np.linalg.lstsq(np.column_stack([2*xy,np.ones(len(xy))]),np.sum(xy**2,axis=1),rcond=None)[0]
        center=fitted[:2]
        radius=float(np.sqrt(max(0.,fitted[2]+np.dot(center,center))))
        error=np.abs(np.linalg.norm(xy-center,axis=1)-radius)
        loops.append({"radial_plane_mm":radial,"center_tangent_y_mm":center.tolist(),"fitted_diameter_mm":2*radius,"max_circle_fit_error_mm":float(error.max()),"bounds_tangent_y_mm":[lo.tolist(),hi.tolist()],"points":len(xy)})
    return loops

def fit_bores(mesh):
    results=[]
    for degrees in [15,105,195,285]:
        records=[]
        for radial in [24,25,26,27,27.7]:
            selected=[r for r in radial_loops(mesh,degrees,radial) if r["max_circle_fit_error_mm"]<.01 and abs(r["fitted_diameter_mm"]-4.242)<.02]
            if len(selected)!=1:
                raise ValueError(f"Expected one round 4.242 mm bore at angle {degrees}, r {radial}; got {selected}")
            records.append(selected[0])
        radial=np.array([r["radial_plane_mm"] for r in records])
        centers=np.array([r["center_tangent_y_mm"] for r in records])
        coefficients=np.linalg.lstsq(np.column_stack([radial,np.ones(len(radial))]),centers,rcond=None)[0]
        axis_angle=degrees+np.degrees(np.arctan(coefficients[0,0]))
        axis_ytilt=np.degrees(np.arctan(coefficients[0,1]))
        nominal_error_y=float(np.max(np.abs(centers[:,1]+4.5)))
        nominal_error_tangent=float(np.max(np.abs(centers[:,0])))
        nut=[r for r in radial_loops(mesh,degrees,22.22) if r["max_circle_fit_error_mm"]>.05]
        if len(nut)!=1:
            raise ValueError(f"Expected one nut cavity at angle {degrees}; got {nut}")
        bounds=np.array(nut[0]["bounds_tangent_y_mm"])
        results.append({"target_theta_deg":degrees,"target_axis_y_mm":-4.5,"section_circle_fits":records,"shaft_direction_theta_deg":float(axis_angle%360),"shaft_direction_theta_error_deg":float(axis_angle-degrees),"shaft_axial_tilt_deg":float(axis_ytilt),"max_axis_y_error_over_sampled_shaft_mm":nominal_error_y,"max_tangent_offset_over_sampled_shaft_mm":nominal_error_tangent,"mean_fitted_diameter_mm":float(np.mean([r["fitted_diameter_mm"] for r in records])),"nut_section":nut[0],"nut_across_axial_flats_mm":float(bounds[1,1]-bounds[0,1]),"nut_across_tangent_corners_mm":float(bounds[1,0]-bounds[0,0]),"nut_axis_y_midpoint_mm":float(np.mean(bounds[:,1]))})
    return results

def main():
    manifest_path=DESIGN/"design_manifest.json"
    manifest=json.loads(manifest_path.read_text())
    old_path=ROOT/"meshes/adapter/adapter_visual.stl"
    old=trimesh.load_mesh(old_path,process=False)
    old.apply_scale(1000)
    a=np.deg2rad(np.linspace(KEY_RANGE[0],KEY_RANGE[1],100))
    wedge=manifold3d.CrossSection([np.vstack([[0,0],40*np.column_stack([np.cos(a),np.sin(a)]),[0,0]])])
    report={"scope":"actual exported revision-2 assembly-mm STLs; independent mesh sections, circle fits and polygon booleans", "manifest_sha256":sha(manifest_path),"original_3mf_sha256":manifest["source_sha256"],"source_hash_current_matches_manifest":sha(Path(manifest["source"]))==manifest["source_sha256"],"old_adapter_stl_sha256":sha(old_path),"key_sector_deg":KEY_RANGE,"fixed_axial_samples_y_mm":AXIAL_SAMPLES,"additional_axial_samples":"each actual mesh minimum Y plus 0.008 and 0.001 mm, giving 13 nonempty layers","current_design_depth_change_from_manifest_mm":manifest["wrist"].get("exposed_crown_depth_change_mm"),"sides":{}}
    fig,axes=plt.subplots(2,3,figsize=(18,12),constrained_layout=True)
    for row,side in enumerate(["left","right"]):
        path=DESIGN/f"adapter_{side}_v2_assembly_mm.stl"
        mesh=trimesh.load_mesh(path,process=False)
        mesh.merge_vertices(digits_vertex=9)
        wrist=wrist_mesh(side)
        axial_samples=AXIAL_SAMPLES+[float(mesh.bounds[0,1]+.008),float(mesh.bounds[0,1]+.001)]
        section_results=[]
        for y in axial_samples:
            wp=closed_paths(wrist,y)
            vp=closed_paths(mesh,y)
            op=closed_paths(old,y)
            if not wp or not vp:
                raise ValueError(f"Empty wrist/adapter contour at actual validation sample Y={y}")
            w,v,o=section_solid(wp),section_solid(vp),section_solid(op)
            all_overlap=v^w
            key_overlap=all_overlap^wedge
            gap=float("inf")
            gap_theta=None
            key_tip_gap=float("inf")
            for theta in np.arange(KEY_RANGE[0]+.013,KEY_RANGE[1],.05):
                vi=intervals(vp,theta)
                wi=intervals(wp,theta)
                if not vi or not wi:
                    continue
                # In this sector the center is empty, adapter petals sit inside
                # the wrist shell; first wrist interval begins at its inner wall.
                wrist_inner=wi[0][0]
                adapter_outer=max(z[1] for z in vi)
                candidate=wrist_inner-adapter_outer
                if candidate<gap:
                    gap,gap_theta=float(candidate),float(theta)
                if wrist_inner<27.5:
                    key_tip_gap=min(key_tip_gap,float(candidate))
            section_results.append({"y_mm":y,"key_overlap_area_mm2":float(key_overlap.area()),"full_overlap_area_mm2":float(all_overlap.area()),"v1_full_overlap_area_mm2":float((o^w).area()),"minimum_key_sector_radial_gap_sampled_mm":None if np.isinf(gap) else gap,"minimum_gap_theta_deg":gap_theta,"minimum_protruding_key_radial_gap_mm":None if np.isinf(key_tip_gap) else key_tip_gap})
            if y in [-.5,-4.5,-10]:
                col=[-.5,-4.5,-10].index(y)
                ax=axes[row,col]
                for geom,color,label in [(o,"#aaa","v1 adapter"),(v,"#b47731","v2 actual STL"),(w,"#356c9e","Official wrist")]:
                    for j,p in enumerate(geom.to_polygons()):
                        ax.plot(p[:,0],p[:,1],c=color,lw=.9,label=label if j==0 else None)
                for p in all_overlap.to_polygons():
                    ax.fill(p[:,0],p[:,1],c="red",alpha=.5)
                ax.set(title=f"{side} Y={y:g}; key overlap={key_overlap.area():.4f} mm2",xlabel="Adapter X [mm]",ylabel="Adapter Z [mm]",xlim=(-33,33),ylim=(-33,33))
                ax.set_aspect("equal")
                ax.grid(alpha=.2)
                ax.legend(fontsize=8,loc="lower left")
        shoulder=[]
        for y in [.01,3,8,11.60]:
            old_s=section_solid(closed_paths(old,y))
            new_s=section_solid(closed_paths(mesh,y))
            shoulder.append({"y_mm":y,"symmetric_difference_area_mm2":float((old_s-new_s).area()+(new_s-old_s).area()),"old_area_mm2":float(old_s.area()),"new_area_mm2":float(new_s.area())})
        bore_results=fit_bores(mesh)
        report["sides"][side]={"file":str(path.relative_to(ROOT)),"sha256":sha(path),"matches_manifest_hash":sha(path)==manifest["sides"][side]["files"][path.name],"watertight":bool(mesh.is_watertight),"is_volume":bool(mesh.is_volume),"bounds_mm":mesh.bounds.tolist(),"bores":bore_results,"key_and_wrist_sections":section_results,"unrotated_shoulder_sections":shoulder,"max_key_overlap_area_mm2":max(s["key_overlap_area_mm2"] for s in section_results),"max_full_overlap_area_mm2":max(s["full_overlap_area_mm2"] for s in section_results)}
    report["limitations"]=["13 axial sections and sampled radial directions are not a full continuous collision proof or a printed fit test.","Expected residual outer-wall interference from source 1.01 print compensation remains; zero key interference must not be presented as zero total interference.","Bore diameter is fitted to the polygonal mesh and may be slightly below nominal due to tessellation; nut flats are measured directly from contour bounds.","The original bores contain small inherited axis tilt/off-centering; measured departures are reported rather than silently assuming perfect cardinal cylinders."]
    (OUT/"v2_fit_validation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    fig.suptitle("Independent validation of actual V2 files: gray V1, orange V2, blue wrist, red overlap")
    fig.savefig(OUT/"v1_v2_wrist_sections.png",dpi=180)
    print(json.dumps({side:{"sha256":r["sha256"],"max_key_overlap_area_mm2":r["max_key_overlap_area_mm2"],"max_full_overlap_area_mm2":r["max_full_overlap_area_mm2"],"bores":[{k:b[k] for k in ["target_theta_deg","shaft_direction_theta_error_deg","max_axis_y_error_over_sampled_shaft_mm","max_tangent_offset_over_sampled_shaft_mm","mean_fitted_diameter_mm","nut_across_axial_flats_mm","nut_axis_y_midpoint_mm"]} for b in r["bores"]],"unrotated_shoulder_sections":r["unrotated_shoulder_sections"]} for side,r in report["sides"].items()},indent=2))

if __name__=="__main__":
    main()
