"""Mesh-based wrist interface feature extraction, in configured adapter axes."""
from pathlib import Path
import sys
import json
import hashlib
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
import mpl_toolkits
local_mpl=Path(sys.prefix)/f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/mpl_toolkits"
if local_mpl.is_dir() and str(local_mpl) not in mpl_toolkits.__path__:
    mpl_toolkits.__path__.insert(0,str(local_mpl))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
config=json.loads((ROOT/"config/assembly.json").read_text())

def wrist_mesh(side):
    letter=side[0].upper()
    mesh=trimesh.load_mesh(ROOT/f"meshes/tron2/wrist_roll_{letter}_Link.STL",process=False)
    mesh.merge_vertices(digits_vertex=12)
    mount=config[side]["wrist_to_adapter"]
    R=Rotation.from_euler("xyz",mount["rpy_rad"]).as_matrix()
    mesh.vertices=(mesh.vertices-np.array(mount["xyz_m"]))@R*1000
    return mesh

def normal_histogram(mesh):
    n=mesh.face_normals
    centers=mesh.triangles_center
    radii=np.linalg.norm(centers[:,[0,2]],axis=1)
    mask=(centers[:,1]>-12)&(centers[:,1]<-.01)&(radii>24)&(radii<32)&(np.abs(n[:,1])>.15)&(np.abs(n[:,1])<.98)
    theta=(np.degrees(np.arctan2(n[:,2],n[:,0]))+90)%180
    hist,bins=np.histogram(theta[mask],bins=360,range=(0,180),weights=mesh.area_faces[mask])
    peaks=np.argsort(hist)[-14:][::-1]
    return [{"axis_angle_deg":float((bins[i]+bins[i+1])/2),"weighted_area_mm2":float(hist[i])} for i in peaks]

def cylinder_candidate(mesh,degrees):
    angle=np.deg2rad(degrees)
    axis=np.array([np.cos(angle),0.,np.sin(angle)])
    tangent=np.array([-np.sin(angle),0.,np.cos(angle)])
    c=mesh.triangles_center
    n=mesh.face_normals
    radius=np.linalg.norm(c[:,[0,2]],axis=1)
    theta=np.arctan2(c[:,2],c[:,0])
    angular=np.abs(np.arctan2(np.sin(theta-angle),np.cos(theta-angle)))
    mask=(c[:,1]>-11)&(c[:,1]<-.05)&(radius>27.9)&(radius<32)&(angular<np.deg2rad(10))&(np.abs(n@axis)<.003)
    vertices=mesh.vertices[np.unique(mesh.faces[mask])]
    if len(vertices)<6:
        return {"angle_deg":degrees,"faces":int(mask.sum())}
    points=np.column_stack([vertices@tangent,vertices[:,1]])
    if degrees in [30,150,210,330]:
        # The unique key at 30 degrees adds unrelated nearby planar faces.
        # Its radial cross-section identifies the 1.25 mm circular hole; fit
        # only vertices lying on that local circle, rather than the key wall.
        keep=np.abs(np.linalg.norm(points-[0,-4.5],axis=1)-1.25)<.02
        points=points[keep]
        vertices=vertices[keep]
    fit=np.linalg.lstsq(np.column_stack([2*points,np.ones(len(points))]),np.sum(points**2,axis=1),rcond=None)[0]
    center=fit[:2]
    r=np.sqrt(max(0,fit[2]+np.dot(center,center)))
    error=np.abs(np.linalg.norm(points-center,axis=1)-r)
    covariance=(n[mask]*mesh.area_faces[mask,None]).T@n[mask]
    _,eig=np.linalg.eigh(covariance)
    fitted_axis=eig[:,0]
    if fitted_axis@axis<0:
        fitted_axis=-fitted_axis
    return {"angle_deg":degrees,"axis_angle_from_normals_deg":float(np.degrees(np.arctan2(fitted_axis[2],fitted_axis[0]))%360),"axis_unit_from_normals":fitted_axis.tolist(),"faces":int(mask.sum()),"vertices":len(vertices),"cylinder_radius_mm":float(r),"cylinder_diameter_mm":float(2*r),"axis_tangent_offset_mm":float(center[0]),"axis_y_mm":float(center[1]),"radial_bounds_mm":[float((vertices@axis).min()),float((vertices@axis).max())],"max_fit_error_mm":float(error.max()),"rms_fit_error_mm":float(np.sqrt(np.mean(error**2)))}

def radial_section(mesh,degrees,distance):
    a=np.deg2rad(degrees)
    u=np.array([np.cos(a),0.,np.sin(a)])
    tangent=np.array([-np.sin(a),0.,np.cos(a)])
    section=mesh.section(plane_normal=u,plane_origin=u*distance)
    result=[]
    if section is None:
        return result
    for entity in section.entities:
        v=section.vertices[entity.points]
        pts=np.column_stack([v@tangent,v[:,1]])
        if len(pts)<8 or pts[:,1].max()< -10 or pts[:,1].min()>0:
            continue
        fit=np.linalg.lstsq(np.column_stack([2*pts,np.ones(len(pts))]),np.sum(pts**2,axis=1),rcond=None)[0]
        center=fit[:2]
        radius=np.sqrt(max(0.,fit[2]+np.dot(center,center)))
        error=np.abs(np.linalg.norm(pts-center,axis=1)-radius)
        result.append({"circle_radius_mm":float(radius),"center_tangent_y_mm":center.tolist(),"max_fit_error_mm":float(error.max()),"bounds_mm":[pts.min(axis=0).tolist(),pts.max(axis=0).tolist()],"points":len(pts)})
    return result

def key_sections(mesh):
    result=[]
    for y in [-.01,-.5,-1,-2,-3,-4.5,-6,-8,-10,-11,-12]:
        s=mesh.section(plane_normal=[0,1,0],plane_origin=[0,y,0])
        if s is None:
            continue
        v=s.vertices
        angle=np.degrees(np.arctan2(v[:,2],v[:,0]))
        radius=np.linalg.norm(v[:,[0,2]],axis=1)
        mask=(angle>20)&(angle<40)&(radius<28.1)&(radius>20)
        if mask.any():
            pts=v[mask]
            a=np.deg2rad(30)
            radial=pts[:,0]*np.cos(a)+pts[:,2]*np.sin(a)
            tangent=-pts[:,0]*np.sin(a)+pts[:,2]*np.cos(a)
            result.append({"y_mm":y,"bounds_xyz_mm":[pts.min(axis=0).tolist(),pts.max(axis=0).tolist()],"angle_span_deg":[float(angle[mask].min()),float(angle[mask].max())],"radial_span_about_30deg_mm":[float(radial.min()),float(radial.max())],"tangent_span_about_30deg_mm":[float(tangent.min()),float(tangent.max())]})
    return result

def key_cylinder(mesh):
    c=mesh.triangles_center
    n=mesh.face_normals
    radius=np.linalg.norm(c[:,[0,2]],axis=1)
    angle=np.degrees(np.arctan2(c[:,2],c[:,0]))
    mask=(c[:,1]>-12.5)&(c[:,1]<-.3)&(radius<27.65)&(radius>25)&(angle>25)&(angle<35)&(np.abs(n[:,1])<.003)
    v=mesh.vertices[np.unique(mesh.faces[mask])]
    pts=v[:,[0,2]]
    fit=np.linalg.lstsq(np.column_stack([2*pts,np.ones(len(pts))]),np.sum(pts**2,axis=1),rcond=None)[0]
    center=fit[:2]
    radius=float(np.sqrt(max(0,fit[2]+np.dot(center,center))))
    errors=np.abs(np.linalg.norm(pts-center,axis=1)-radius)
    return {"faces":int(mask.sum()),"radius_mm":radius,"axis_center_xz_mm":center.tolist(),"axis_center_polar_radius_mm":float(np.linalg.norm(center)),"axis_center_angle_deg":float(np.degrees(np.arctan2(center[1],center[0]))),"vertex_y_bounds_mm":[float(v[:,1].min()),float(v[:,1].max())],"max_fit_error_mm":float(errors.max())}

def section_polylines(mesh,axis,coordinate):
    origin=np.zeros(3)
    origin[axis]=coordinate
    normal=np.zeros(3)
    normal[axis]=1
    section=mesh.section(plane_origin=origin,plane_normal=normal)
    return [] if section is None else [section.vertices[e.points] for e in section.entities if len(e.points)>2]

def plots_and_key_profiles(wrists,adapter):
    fig,axes=plt.subplots(2,2,figsize=(13,12),constrained_layout=True)
    for row,(side,mesh) in enumerate(wrists.items()):
        for col,y in enumerate([-.5,-4.5]):
            ax=axes[row,col]
            for model,color,label in [(adapter,"#bc8b53","Existing adapter"),(mesh,"#3a699a","Official wrist")]:
                for j,pts in enumerate(section_polylines(model,1,y)):
                    ax.plot(pts[:,0],pts[:,2],c=color,lw=.9,label=label if j==0 else None)
            for degrees in [15,105,195,285]:
                a=np.deg2rad(degrees)
                ax.plot([18*np.cos(a),35*np.cos(a)],[18*np.sin(a),35*np.sin(a)],c="green",ls="--",lw=.8)
                ax.text(36*np.cos(a),36*np.sin(a),f"{degrees}°\n4.5 mm",ha="center",va="center",fontsize=8,c="green")
            for degrees in [30,150,210,330]:
                a=np.deg2rad(degrees)
                ax.plot(29*np.cos(a),29*np.sin(a),marker="o",mfc="none",mec="purple",ms=5)
            a=np.deg2rad(30)
            ax.annotate("Unique inward key\n30°; separate from large holes",xy=(26*np.cos(a),26*np.sin(a)),xytext=(-35,38),arrowprops={"arrowstyle":"->","color":"red"},color="red",fontsize=8)
            ax.set(title=f"{side.title()} wrist, adapter Y={y:g} mm",xlabel="Adapter X [mm]",ylabel="Adapter Z [mm]",xlim=(-41,41),ylim=(-41,42))
            ax.set_aspect("equal")
            ax.grid(alpha=.2)
            ax.legend(fontsize=8,loc="lower left")
    fig.suptitle("Green: mounting holes at 15 + 90n degrees; purple: separate 2.5 mm holes; red: unique key")
    fig.savefig(OUT/"wrist_holes_and_key.png",dpi=190)
    plt.close(fig)

    profiles={}
    fig,axes=plt.subplots(1,2,figsize=(12,7),constrained_layout=True)
    for ax,(side,mesh) in zip(axes,wrists.items()):
        profiles[side]={}
        for y in [-.01,-.3,-1,-3,-8,-10.4]:
            curves=[]
            for points in section_polylines(mesh,1,y):
                s=points[:,0]*np.cos(np.pi/6)+points[:,2]*np.sin(np.pi/6)
                t=-points[:,0]*np.sin(np.pi/6)+points[:,2]*np.cos(np.pi/6)
                valid=(s>25)&(s<29)&(np.abs(t)<4.5)
                if valid.any():
                    # Record vertices in mesh contour order; use NaN gaps in plots.
                    coords=np.column_stack([t,s])
                    curves.append(coords[valid].tolist())
                    coords[~valid]=np.nan
                    ax.plot(coords[:,0],coords[:,1],lw=1,label=f"Y={y:g}" if len(curves)==1 else None)
            profiles[side][str(y)]=curves
        t=np.linspace(-4.5,4.5,180)
        ax.plot(t,np.sqrt(28**2-t**2),c="black",ls="--",lw=.8,label="Nominal inner circle R=28")
        ax.set(title=f"{side.title()} key at theta=30°",xlabel="Local tangent t [mm]",ylabel="Local radial coordinate s [mm]",xlim=(-4.5,4.5),ylim=(25.3,29))
        ax.set_aspect("equal")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8,loc="upper right")
    fig.savefig(OUT/"key_local_profiles.png",dpi=190)
    plt.close(fig)
    return profiles

def main():
    report={}
    wrists={}
    for side in ["right","left"]:
        mesh=wrist_mesh(side)
        wrists[side]=mesh
        report[side]={"bounds_mm":mesh.bounds.tolist(),"normal_histogram_peaks":normal_histogram(mesh),"candidate_cylinders":[cylinder_candidate(mesh,degrees) for degrees in [15,30,105,150,195,210,285,330]],"large_hole_radial_sections":{str(s):radial_section(mesh,15,s) for s in [28.05,28.25,28.5,29,29.5,30,30.4]},"key_sections":key_sections(mesh),"key_cylinder_fit":key_cylinder(mesh)}
        report[side]["mounting_holes"]=[x for x in report[side]["candidate_cylinders"] if x["angle_deg"] in [15,105,195,285]]
        report[side]["separate_small_holes"]=[x for x in report[side]["candidate_cylinders"] if x["angle_deg"] in [30,150,210,330]]
    adapter=trimesh.load_mesh(ROOT/"meshes/adapter/adapter_visual.stl",process=False)
    adapter.apply_scale(1000)
    report["key_profiles_tangent_radial_mm"]=plots_and_key_profiles(wrists,adapter)
    report["coordinate_convention"]={"frame":"current configured adapter frame; mm; origin at nominal original CAD (0,-12.6,0)","theta":"atan2(Z,X), degrees in [0,360)","view":"X right, Z up, viewer on -Y looking toward +Y; positive theta is counterclockwise in this view","rotation_warning":"A positive theta placement is standard right-handed Ry(-theta), not Ry(+theta)","mounts":{side:config[side]["wrist_to_adapter"] for side in wrists}}
    report["key_interpretation"]={"center_angle_deg":30,"nominal_inner_bore_radius_mm":28.0,"minimum_key_radial_coordinate_mm_approx":25.75,"inward_protrusion_mm_approx":2.25,"tangential_root_extent_mm_approx":[-3.526,3.526],"root_width_mm_approx":7.052,"root_angle_range_deg_approx":[22.766,37.234],"body_y_range_mm_approx":[-12.5,-.3],"mouth_chamfer_axial_extent_mm_approx":.3,"geometry":"unique inward rounded key, not a separate radial mounting hole; noncircular profile, use sampled profile/envelope rather than the rejected single-circle fit","photo":"photo arrow is morphologically consistent with this unique inward key; photos not used for dimensional inference","precision_limit":"Approximate mesh-envelope dimensions, not engineering tolerances or a recovered CAD sketch; radial 2.5 mm small hole interrupts its section at Y=-4.5"}
    sys.path.insert(0,str(ROOT/"scripts"))
    from convert_adapter import read_parts,DEFAULT_SOURCE
    parts,_,_=read_parts(DEFAULT_SOURCE)
    old_part=parts[1][2].copy()
    old_part.apply_translation([0,12.6,0])
    old_nut_sections={str(rad):radial_section(old_part,0,rad) for rad in [20,21,22,22.9,23.1,25,27.8]}
    report["existing_adapter"]={"source_sha256":hashlib.sha256(DEFAULT_SOURCE.read_bytes()).hexdigest(),"source":"original 3MF part 2, before print scaling","nominal_hole_theta_deg":[0,90,180,270],"nominal_hole_y_relative_shoulder_mm":-4.5,"nominal_hole_diameter_mm":4.2,"current_print_scale":config["adapter_print_scale"],"current_hole_y_relative_shoulder_mm":-4.5*config["adapter_print_scale"],"current_hole_diameter_mm":4.2*config["adapter_print_scale"],"nominal_nut_pocket_across_flats_mm":6.9,"nominal_nut_pocket_across_corners_mm_approx":7.97,"nominal_nut_seat_radial_position_mm_approx":23.0,"nominal_nut_pocket_depth_from_inner_bore_mm_approx":3.0,"radial_nut_section_evidence":old_nut_sections,"key_relation":"the old four-lobed adapter already leaves a wide open sector near 30 degrees; moving holes/nut bosses must preserve a separate key clearance there"}
    report["conclusions"]={"mounting_hole_target_theta_deg":[15,105,195,285],"target_hole_axis_y_mm":-4.5,"target_wrist_inner_cylinder_diameter_mm":4.5,"target_wrist_countersink":"approximately 45 degree radial expansion begins at axial hole coordinate r=28.25 mm; diameter at r=28.5/29/29.5/30 is 5/6/7/8 mm","global_adapter_rotation_required":False,"hole_pattern_rotation_in_theta_deg":15,"small_hole_pattern_is_not_target":True,"key_must_be_treated_separately":True,"left_right_wrist_interface_same_in_configured_frames":True,"source_cad_y_for_target_with_current_print_scale_mm":-12.6-4.5/config["adapter_print_scale"]}
    (OUT/"wrist_features.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps({side:{"mounting_holes":report[side]["mounting_holes"],"separate_small_holes":report[side]["separate_small_holes"]} for side in wrists},indent=2))

if __name__=="__main__":
    main()
