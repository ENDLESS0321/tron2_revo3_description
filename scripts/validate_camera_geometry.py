#!/usr/bin/env python3
"""Independent checks of new parameterized bracket CAD and optical geometry.

Creates only reports/cameras/geometry_validation.json. Test planes and colored
markers exist only in this process's renderer scene. The source URDF, MJCF,
camera modules, saved rig configuration and other viewers are never edited.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import traceback
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from scipy import ndimage

from camera_rig import create_source


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "urdf/tron2_dach_revo3_cameras.urdf"
SCENE = ROOT / "simulation/cameras/scene.xml"
OUTPUT = ROOT / "reports/cameras/geometry_validation.json"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def axis_rotation(axis, angle):
    x, y, z = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)


def rpy_rotation(values):
    r, p, y = values
    return axis_rotation([0,0,1], y) @ axis_rotation([0,1,0], p) @ axis_rotation([1,0,0], r)


def transform(xyz=(0,0,0), rotation=None):
    matrix = np.eye(4)
    matrix[:3,3] = xyz
    if rotation is not None:
        matrix[:3,:3] = rotation
    return matrix


def independent_fk(rig):
    root = ET.parse(Path(getattr(rig,"urdf_path",URDF))).getroot()
    values = {mujoco.mj_id2name(rig.model, mujoco.mjtObj.mjOBJ_JOINT, j): float(rig.data.qpos[rig.model.jnt_qposadr[j]]) for j in range(rig.model.njnt)}
    links = {link.get("name") for link in root.findall("link")}
    children = {joint.find("child").get("link") for joint in root.findall("joint")}
    roots = links-children
    if len(roots) != 1:
        raise ValueError("URDF is not single-rooted")
    result = {roots.pop(): np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        changed = False
        for joint in list(pending):
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in result:
                continue
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            motion = np.eye(4)
            if joint.get("type") == "revolute":
                motion[:3,:3] = axis_rotation(np.fromstring(joint.find("axis").get("xyz"), sep=" "), values[joint.get("name")])
            elif joint.get("type") != "fixed":
                raise ValueError("Unexpected joint type in camera scaffold")
            result[child] = result[parent] @ transform(xyz, rpy_rotation(rpy)) @ motion
            pending.remove(joint)
            changed = True
        if not changed:
            raise ValueError("Disconnected or cyclic URDF")
    return result


def camera_pose(rig, cid):
    return transform(rig.data.cam_xpos[cid].copy(), rig.data.cam_xmat[cid].reshape(3,3).copy())


def expected_K(rig, name, stream):
    part = rig.config["head"] if name == "head" else rig.config["wrists"][name.split("_")[0]]
    horizontal, vertical = rig.config["models"][part["model"]][stream+"_fov_deg"]
    return np.array([[rig.width/(2*math.tan(math.radians(horizontal)/2)),0,(rig.width-1)/2], [0,rig.height/(2*math.tan(math.radians(vertical)/2)),(rig.height-1)/2], [0,0,1.]])


def frame_checks(rig):
    fk = independent_fk(rig)
    rows = []
    for name, ids in rig.camera_ids.items():
        prefix = "head_camera" if name == "head" else name+"_camera"
        for stream, cid in ids.items():
            optical = fk[prefix+"_"+stream+"_optical_frame"]
            expected = optical @ transform(rotation=np.diag([1.,-1.,-1.]))
            actual = camera_pose(rig, cid)
            position_error = float(np.linalg.norm(actual[:3,3]-expected[:3,3]))
            rotation_error = float(np.max(np.abs(actual[:3,:3]-expected[:3,:3])))
            forward_dot = float(np.dot(-actual[:3,2], optical[:3,2]))
            up_dot = float(np.dot(actual[:3,1], -optical[:3,1]))
            rows.append({"camera": name, "stream": stream, "optical_frame": prefix+"_"+stream+"_optical_frame", "T_world_optical_independent_fk": optical.tolist(), "T_world_mujoco_camera": actual.tolist(), "position_error_m": position_error, "rotation_matrix_error": rotation_error, "optical_forward_dot_mujoco_forward": forward_dot, "optical_up_dot_mujoco_up": up_dot, "pass": position_error < 2e-5 and rotation_error < 2e-5 and forward_dot > 1-1e-9 and up_dot > 1-1e-9})
    return rows, fk


def add_box(scene, position, rotation, half_size, rgba):
    index = scene.ngeom
    if index >= scene.maxgeom:
        raise ValueError("Analytical fixture exceeds renderer scene capacity")
    geom = scene.geoms[index]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_BOX, np.asarray(half_size, dtype=float), np.asarray(position, dtype=float), np.asarray(rotation, dtype=float).reshape(-1), np.asarray(rgba, dtype=np.float32))
    geom.specular = 0
    geom.shininess = 0
    geom.reflectance = 0
    scene.ngeom += 1


def analytical_fixture(rig, cid, optical, kind):
    """Render independent world-space geometry through an actual rig camera."""
    rig.renderer.disable_depth_rendering()
    rig.renderer.update_scene(rig.data, camera=cid, scene_option=rig.opt)
    scene = rig.renderer.scene
    scene.ngeom = 0  # Only this renderer's in-memory scene is changed.
    scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
    for light in scene.lights[:scene.nlight]:
        light.ambient[:] = [1/max(1, scene.nlight)]*3
        light.diffuse[:] = 0
        light.specular[:] = 0
    R, p = optical[:3,:3], optical[:3,3]
    if kind in ("flat", "tilted"):
        z0, sx, sy = (.42,0.,0.) if kind == "flat" else (.39,.08,-.06)
        normal = np.array([-sx,-sy,1.]); normal /= np.linalg.norm(normal)
        tangent = np.array([1.,0.,sx]); tangent /= np.linalg.norm(tangent)
        plane_R = np.column_stack([tangent, np.cross(normal,tangent), normal])
        thickness = .0005
        center = np.array([0.,0.,z0])+thickness*normal
        add_box(scene, p+R@center, R@plane_R, [.8,.8,thickness], [.55,.55,.55,1])
        rig.renderer.enable_depth_rendering()
        pixels = rig.renderer.render().copy()
        rig.renderer.disable_depth_rendering()
        return pixels, {"z0_m": z0, "slope_x": sx, "slope_y": sy}
    add_box(scene, p+R@np.array([0.,0.,.441]), R, [.8,.8,.001], [.20,.20,.20,1])
    if kind == "grid":
        centers = np.array([[x,y,.4] for y in np.linspace(-.095,.095,5) for x in np.linspace(-.15,.15,7)])
        markers = [(str(index), center, [1.,.02,.02,1.]) for index, center in enumerate(centers)]
    else:
        markers = [("red_upper_right", [.10,-.080,.4], [1,.02,.02,1]), ("green_lower_left", [-.09,.07,.4], [.02,1,.02,1]), ("blue_upper_left", [-.07,-.065,.4], [.02,.02,1,1]), ("yellow_lower_right", [.08,.055,.4], [1,1,.02,1])]
    for _, center, rgba in markers:
        add_box(scene, p+R@(np.asarray(center)+[0,0,.0001]), R, [.006,.006,.0001], rgba)
    return rig.renderer.render().copy(), markers


def color_weights(image, color):
    r, g, b = np.moveaxis(image.astype(float), -1, 0)
    if color == "red":
        score = r-np.maximum(g,b)
    elif color == "green":
        score = g-np.maximum(r,b)
    elif color == "blue":
        score = b-np.maximum(r,g)
    elif color == "yellow":
        score = np.minimum(r,g)-b
    else:
        raise ValueError(color)
    return np.maximum(score,0.)


def weighted_centroid(weights):
    yy, xx = np.indices(weights.shape)
    total = weights.sum()
    if total <= 0:
        raise ValueError("Expected colored marker missing from rendered image")
    return np.array([(weights*xx).sum()/total, (weights*yy).sum()/total])


def pixel_checks(rig, fk):
    rows = []
    for name, ids in rig.camera_ids.items():
        prefix = "head_camera" if name == "head" else name+"_camera"
        metadata = next(spec for spec in rig.camera_specs if spec["name"] == name)
        for stream, cid in ids.items():
            K = expected_K(rig,name,stream)
            optical = fk[prefix+"_"+stream+"_optical_frame"]
            flat, _ = analytical_fixture(rig,cid,optical,"flat")
            margin = 12
            interior = flat[margin:-margin,margin:-margin]
            flat_error = float(np.max(np.abs(interior-.42)))
            sample_uv = [(rig.width//2,rig.height//2),(int(.15*rig.width),int(.2*rig.height)),(int(.85*rig.width),int(.8*rig.height))]
            samples = []
            for u,v in sample_uv:
                radial_range = .42*math.sqrt(1+((u-K[0,2])/K[0,0])**2+((v-K[1,2])/K[1,1])**2)
                samples.append({"u":u,"v":v,"rendered_depth_m":float(flat[v,u]),"expected_axial_z_m":.42,"contrasting_euclidean_range_m":radial_range})
            tilted, plane = analytical_fixture(rig,cid,optical,"tilted")
            us = np.linspace(20,rig.width-21,19).astype(int)
            vs = np.linspace(20,rig.height-21,15).astype(int)
            U,V = np.meshgrid(us,vs)
            expected = plane["z0_m"]/(1-plane["slope_x"]*(U-K[0,2])/K[0,0]-plane["slope_y"]*(V-K[1,2])/K[1,1])
            tilted_error = float(np.max(np.abs(tilted[V,U]-expected)))
            image, markers = analytical_fixture(rig,cid,optical,"orientation")
            orientation = []
            for label, point, _ in markers:
                score = color_weights(image,label.split("_")[0])
                observed = weighted_centroid(score)
                q = K@np.asarray(point); projected = q[:2]/q[2]
                error = float(np.linalg.norm(observed-projected))
                orientation.append({"marker":label,"optical_xyz_m":point,"expected_pixel_uv":projected.tolist(),"measured_pixel_uv":observed.tolist(),"error_px":error,"pass":error<.85})
            grid, markers = analytical_fixture(rig,cid,optical,"grid")
            weights = color_weights(grid,"red")
            labels, count = ndimage.label(weights>5)
            centers = []
            for label in range(1,count+1):
                selected = weights*(labels==label)
                if np.count_nonzero(selected)>4:
                    centers.append(weighted_centroid(selected))
            if len(centers) != 35:
                raise ValueError(f"{name}/{stream}: expected 35 projection markers, found {len(centers)}")
            centers = sorted(centers,key=lambda p:p[1])
            centers = np.asarray([p for row in range(5) for p in sorted(centers[row*7:(row+1)*7],key=lambda p:p[0])])
            points = np.asarray([marker[1] for marker in markers])
            rays = np.column_stack([points[:,0]/points[:,2],points[:,1]/points[:,2],np.ones(35)])
            fitted = np.linalg.lstsq(rays,centers,rcond=None)[0].T
            principal_error = float(np.max(np.abs(fitted[:,2]-K[:2,2])))
            focal_relative_error = max(abs(fitted[0,0]/K[0,0]-1),abs(fitted[1,1]/K[1,1]-1))
            metadata_error = float(np.max(np.abs(np.asarray(metadata[stream+"_K"])-K)))
            rows.append({"camera":name,"stream":stream,"width":rig.width,"height":rig.height,"expected_K_from_FOV":K.tolist(),"reported_K_difference":metadata_error,"render_fitted_projection_2x3":fitted.tolist(),"principal_error_px":principal_error,"focal_relative_error":float(focal_relative_error),"half_pixel_convention":"integer pixel indices, principal=((W-1)/2,(H-1)/2)","flat_plane_max_axial_depth_error_m":flat_error,"flat_plane_samples":samples,"tilted_plane_equation":"Z = z0 + slope_x*X + slope_y*Y in ROS optical coordinates","tilted_plane":plane,"tilted_plane_max_depth_error_m":tilted_error,"orientation_markers":orientation,"grid_marker_count":len(centers),"fixture_image_sha256":{"orientation_rgb":array_sha(image),"projection_grid_rgb":array_sha(grid),"flat_depth":array_sha(flat),"tilted_depth":array_sha(tilted)},"pass":flat_error<2e-5 and tilted_error<5e-5 and principal_error<.18 and focal_relative_error<.002 and metadata_error<1e-10 and all(row["pass"] for row in orientation)})
    return rows


def all_camera_poses(rig):
    return {name+"/"+stream:camera_pose(rig,cid) for name,ids in rig.camera_ids.items() for stream,cid in ids.items()}


def rgb_delta(before,after):
    return float(np.mean(np.abs(before.astype(float)-after.astype(float))))


def joint_values(rig):
    return {mujoco.mj_id2name(rig.model,mujoco.mjtObj.mjOBJ_JOINT,j):float(rig.data.qpos[rig.model.jnt_qposadr[j]]) for j in range(rig.model.njnt)}


def calibration_values(rig):
    result={}
    for spec in rig.camera_specs:
        for stream,cid in rig.camera_ids[spec["name"]].items():
            result[spec["name"]+"/"+stream]={"K":copy.deepcopy(spec[stream+"_K"]),"resolution":rig.model.cam_resolution[cid].tolist(),"sensor_size":rig.model.cam_sensorsize[cid].tolist(),"intrinsic":rig.model.cam_intrinsic[cid].tolist()}
    return result


def active_bracket_assets(rig):
    urdf=Path(rig.urdf_path);root=ET.parse(urdf).getroot();result={}
    for side in ("left","right"):
        link=root.find(f"link[@name='{side}_wrist_camera_mount_frame']")
        mesh=link.find("visual/geometry/mesh") if link is not None else None
        if mesh is None:raise ValueError("The new bracket CAD must exist as an actual visual mesh")
        path=(urdf.parent/mesh.get("filename")).resolve()
        result[side]={"path":str(path.relative_to(ROOT)),"sha256":sha(path)}
    return {"urdf":str(urdf.relative_to(ROOT)),"urdf_sha256":sha(urdf),"scene":str(Path(rig.scene_path).relative_to(ROOT)),"scene_sha256":sha(rig.scene_path),"config":str(Path(rig.config_path).relative_to(ROOT)),"config_sha256":sha(rig.config_path),"brackets":result}


def mount_checks(rig):
    cfg=copy.deepcopy(rig.config["wrists"]["left"])
    if rig.config.get("bracket_geometry")!="parametric_photo_reference_v1":raise ValueError("Expected the new parameterized CAD, not an empty frame scaffold")
    # Give every joint a non-default, bounded value so rebuilding from the
    # configuration pose alone cannot accidentally satisfy state preservation.
    original_joints=joint_values(rig)
    for j in range(rig.model.njnt):
        adr=rig.model.jnt_qposadr[j];q=rig.data.qpos[adr];lo,hi=rig.model.jnt_range[j]
        amount=.013+.001*(j%7)
        rig.data.qpos[adr]=q+amount if hi-q>amount else q-amount
        if not lo<=rig.data.qpos[adr]<=hi:raise ValueError("Test joint perturbation is outside limits")
    rig.tick=37
    mujoco.mj_forward(rig.model,rig.data)
    rig.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=True
    nominal=rig.step_and_capture();poses0=all_camera_poses(rig)
    names={stage:"left_wrist_camera_"+stage+"_frame" for stage in ("mount","rod_end","plate")}
    bodies0={stage:rig.body_transform(name).copy() for stage,name in names.items()}
    calibration0=calibration_values(rig);assets0=active_bracket_assets(rig)

    def update(length,tilt):
        before_joints=joint_values(rig);before_tick=rig.tick
        object_ids=(id(rig.model),id(rig.data),id(rig.renderer))
        rig.set_mount("left",length,tilt)
        # Always reacquire from rig; set_mount replaces all three objects.
        after_joints=joint_values(rig)
        keys_match=set(before_joints)==set(after_joints)
        error=max((abs(after_joints[k]-v) for k,v in before_joints.items()),default=0.) if keys_match else None
        calibration=calibration_values(rig)
        calibration_error=max(float(np.max(np.abs(np.asarray(calibration[k][field])-calibration0[k][field]))) for k in calibration0 for field in calibration0[k])
        objects_replaced=all(old!=new for old,new in zip(object_ids,(id(rig.model),id(rig.data),id(rig.renderer))))
        frames,_=frame_checks(rig)
        result={"joint_names_preserved":keys_match,"tested_joint_count":len(before_joints),"max_joint_position_error_rad":error,"tick_before":before_tick,"tick_after_reload":rig.tick,"tick_preserved":rig.tick==before_tick,"model_data_renderer_replaced":objects_replaced,"all_camera_K_sensor_settings_max_error":calibration_error,"independent_variant_FK_pass":all(row["pass"] for row in frames),"variant_artifacts":active_bracket_assets(rig)}
        result["pass"]=keys_match and len(before_joints)==58 and error<1e-12 and rig.tick==before_tick and objects_replaced and calibration_error<1e-12 and result["independent_variant_FK_pass"]
        return result

    length_reload=update(cfg["length_mm"]+20,cfg["plate_tilt_deg"])
    longer=rig.step_and_capture();poses1=all_camera_poses(rig)
    translation=bodies0["mount"][:3,:3]@np.asarray(cfg["extension_axis"])*.020
    position_error=rotation_error=unaffected_pose_error=0.
    for key,T0 in poses0.items():
        expected=T0.copy()
        if key.startswith("left_wrist/"):expected[:3,3]+=translation
        else:unaffected_pose_error=max(unaffected_pose_error,float(np.max(np.abs(poses1[key]-T0))))
        position_error=max(position_error,float(np.linalg.norm(poses1[key][:3,3]-expected[:3,3])))
        rotation_error=max(rotation_error,float(np.max(np.abs(poses1[key][:3,:3]-expected[:3,:3]))))
    body_errors={}
    for stage,name in names.items():
        expected=bodies0[stage].copy()
        if stage in ("rod_end","plate"):expected[:3,3]+=translation
        body_errors[stage]=float(np.max(np.abs(rig.body_transform(name)-expected)))
    length_assets=length_reload["variant_artifacts"]["brackets"]
    length_geometry_changed=length_assets["left"]["sha256"]!=assets0["brackets"]["left"]["sha256"] and length_assets["right"]["sha256"]==assets0["brackets"]["right"]["sha256"]
    length_rgb_change=rgb_delta(nominal["frames"]["left_wrist"]["rgb"],longer["frames"]["left_wrist"]["rgb"])
    length={"delta_length_mm":20,"expected_world_translation_m":translation.tolist(),"camera_position_max_error_m":position_error,"camera_rotation_max_error":rotation_error,"frame_max_transform_errors":body_errors,"head_right_pose_max_error":unaffected_pose_error,"left_rgb_mean_absolute_change_8bit":length_rgb_change,"left_mesh_changed_right_mesh_preserved":length_geometry_changed,"reload":length_reload,"pass":position_error<2e-6 and rotation_error<5e-6 and max(body_errors.values())<5e-6 and unaffected_pose_error<1e-10 and length_rgb_change>.05 and length_geometry_changed and length_reload["pass"]}
    rig.set_mount("left",cfg["length_mm"],cfg["plate_tilt_deg"])
    tilt_reload=update(cfg["length_mm"],cfg["plate_tilt_deg"]+15)
    tilted=rig.step_and_capture();poses2=all_camera_poses(rig)
    pivot=bodies0["rod_end"][:3,3]
    delta_world=bodies0["plate"][:3,:3]@axis_rotation(cfg["plate_tilt_axis"],math.radians(15))@bodies0["plate"][:3,:3].T
    p_error=r_error=unaffected_pose_error=0.
    for key,T0 in poses0.items():
        expected=T0.copy()
        if key.startswith("left_wrist/"):
            expected[:3,3]=pivot+delta_world@(T0[:3,3]-pivot);expected[:3,:3]=delta_world@T0[:3,:3]
        else:unaffected_pose_error=max(unaffected_pose_error,float(np.max(np.abs(poses2[key]-T0))))
        p_error=max(p_error,float(np.linalg.norm(poses2[key][:3,3]-expected[:3,3])))
        r_error=max(r_error,float(np.max(np.abs(poses2[key][:3,:3]-expected[:3,:3]))))
    pivot_error=float(np.max(np.abs(rig.body_transform(names["rod_end"])-bodies0["rod_end"])))
    mount_error=float(np.max(np.abs(rig.body_transform(names["mount"])-bodies0["mount"])))
    plate_expected=bodies0["plate"].copy();plate_expected[:3,:3]=delta_world@plate_expected[:3,:3]
    plate_error=float(np.max(np.abs(rig.body_transform(names["plate"])-plate_expected)))
    tilt_assets=tilt_reload["variant_artifacts"]["brackets"]
    tilt_geometry_changed=tilt_assets["left"]["sha256"]!=assets0["brackets"]["left"]["sha256"] and tilt_assets["right"]["sha256"]==assets0["brackets"]["right"]["sha256"]
    tilt_rgb_change=rgb_delta(nominal["frames"]["left_wrist"]["rgb"],tilted["frames"]["left_wrist"]["rgb"])
    tilt={"delta_tilt_deg":15,"rod_end_world_pivot_m":pivot.tolist(),"rod_end_transform_error":pivot_error,"mount_transform_error":mount_error,"plate_transform_error":plate_error,"camera_position_max_error_m":p_error,"camera_rotation_max_error":r_error,"head_right_pose_max_error":unaffected_pose_error,"left_rgb_mean_absolute_change_8bit":tilt_rgb_change,"left_mesh_changed_right_mesh_preserved":tilt_geometry_changed,"reload":tilt_reload,"pass":p_error<2e-6 and r_error<5e-6 and max(pivot_error,mount_error,plate_error)<5e-6 and unaffected_pose_error<1e-10 and tilt_rgb_change>.05 and tilt_geometry_changed and tilt_reload["pass"]}
    unaffected={name:{"length_rgb_change":rgb_delta(nominal["frames"][name]["rgb"],longer["frames"][name]["rgb"]),"tilt_rgb_change":rgb_delta(nominal["frames"][name]["rgb"],tilted["frames"][name]["rgb"])} for name in ("head","right_wrist")}
    rig.set_mount("left",cfg["length_mm"],cfg["plate_tilt_deg"])
    for j in range(rig.model.njnt):rig.data.qpos[rig.model.jnt_qposadr[j]]=original_joints[mujoco.mj_id2name(rig.model,mujoco.mjtObj.mjOBJ_JOINT,j)]
    mujoco.mj_forward(rig.model,rig.data)
    return {"baseline_artifacts":assets0,"length":length,"tilt":tilt,"other_views_image_diagnostics":unaffected,"timestamp_sequence_ns":[nominal["stamp_ns"],longer["stamp_ns"],tilted["stamp_ns"]],"rounding_tolerances":{"position_m":2e-6,"rotation_matrix":5e-6,"unchanged_camera_pose_matrix":1e-10,"reason":"mj_saveLastXML rounds independently regenerated plate quaternions; tolerances bound that serialization, not a physical motion allowance."},"scope":"New parameterized solid bracket CAD is regenerated. Isolation gates concern other-camera poses and K; shared-scene pixels can change when moved bracket solids change visibility.","geometry_status":rig.config["geometry_status"],"new_bracket_CAD_present":True,"physical_fit_or_strength_certified":False}


def input_hashes():
    paths=[ROOT/"scripts/camera_rig.py",ROOT/"scripts/build_camera_rig.py",ROOT/"scripts/bracket_assets.py",ROOT/"scripts/design_wrist_camera_bracket.py",Path(__file__),ROOT/"config/camera_rig.json",ROOT/"config/camera_model_catalog.json",ROOT/"design/wrist_camera_bracket_v1/manifest.json",URDF,SCENE,ROOT/"reports/cameras/build_manifest.json"]
    hashes={str(path.relative_to(ROOT)):sha(path) for path in paths}
    for mesh in ET.parse(SCENE).getroot().findall("asset/mesh"):
        if mesh.get("file"):
            path=(SCENE.parent/mesh.get("file")).resolve()
            hashes[str(path.relative_to(ROOT))]=sha(path)
    return hashes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width",type=int,default=640)
    parser.add_argument("--height",type=int,default=480)
    parser.add_argument("--second-resolution",type=int,nargs=2,default=[800,450])
    args=parser.parse_args()
    report={"schema":"three_camera_geometry_validation_v2","status":"failed","mujoco_version":mujoco.__version__,"scope":"新建参数化支架CAD及光学几何验证","method":"Independent URDF FK plus rendered analytical world planes and colored markers, followed by real CAD/model/renderer rebuilds.","claim_boundary":"Validates new parameterized bracket CAD and imaging geometry. Original STEP recovery, measured physical fit, material strength, real sensor calibration and physical dynamics are not certified."}
    before=None
    try:
        before=input_hashes()
        manifest=json.loads((ROOT/"reports/cameras/build_manifest.json").read_text())
        if manifest["config_sha256"]!=sha(ROOT/"config/camera_rig.json") or manifest["producer_sha256"]!=sha(ROOT/"scripts/build_camera_rig.py") or manifest["urdf_sha256"]!=sha(URDF) or manifest["scene_sha256"]!=sha(SCENE):
            raise RuntimeError("Main camera build is stale; regenerate it before independent validation")
        all_frames=[];all_pixels=[];mounts=None
        for index,(width,height) in enumerate([(args.width,args.height),tuple(args.second_resolution)]):
            rig=create_source(width=width,height=height)
            try:
                frames,fk=frame_checks(rig)
                all_frames.extend(frames)
                all_pixels.extend(pixel_checks(rig,fk))
                if index==0:
                    mounts=mount_checks(rig)
            finally:
                rig.close()
        after=input_hashes()
        unchanged=before==after
        report.update({"input_hashes":before,"input_files_unchanged_during_validation":unchanged,"resolutions_wh":[[args.width,args.height],args.second_resolution],"frame_checks":all_frames,"rendered_pixel_depth_checks":all_pixels,"mount_checks":mounts,"checks":{"all_camera_optical_FK_pass":all(r["pass"] for r in all_frames),"all_rendered_depth_projection_checks_pass":all(r["pass"] for r in all_pixels),"left_length_20mm_CAD_reload_isolation_pass":mounts["length"]["pass"],"left_tilt_15deg_CAD_reload_isolation_pass":mounts["tilt"]["pass"],"joint_state_and_tick_preserved":mounts["length"]["reload"]["pass"] and mounts["tilt"]["reload"]["pass"],"input_hashes_stable":unchanged}})
        report["status"]="camera_geometry_pass" if all(report["checks"].values()) else "camera_geometry_fail"
    except Exception as exc:
        report.update({"error":str(exc),"traceback":traceback.format_exc(),"input_hashes":before})
    OUTPUT.parent.mkdir(parents=True,exist_ok=True)
    OUTPUT.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"status":report["status"],"checks":report.get("checks"),"error":report.get("error"),"report":str(OUTPUT)},indent=2))
    return 0 if report["status"]=="camera_geometry_pass" else 2


if __name__=="__main__":raise SystemExit(main())
