#!/usr/bin/env python3
"""Convert the assembled URDF to a reusable MuJoCo scene and audit its kinematics.

The preview and interactive viewer use forward kinematics, not physical tracking.
No actuator gains, armatures, joint damping, masses, or inertias are invented.
An optional unactuated dynamics smoke test is explicitly separate from assembly
validation and is not a stability or contact-success certificate.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[1]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=PROJECT / "urdf/tron2_dach_revo3.urdf")
    parser.add_argument("--scene", type=Path, default=PROJECT / "simulation/scene.xml")
    parser.add_argument("--report-dir", type=Path, default=PROJECT / "reports")
    parser.add_argument("--expected-dof", type=int, default=58)
    parser.add_argument("--backend", choices=("egl", "osmesa", "glfw"), default=os.environ.get("MUJOCO_GL", "egl"))
    parser.add_argument("--width", type=int, default=1100)
    parser.add_argument("--height", type=int, default=1000)
    parser.add_argument("--skip-render", action="store_true")
    parser.add_argument("--viewer", action="store_true", help="Open a kinematic viewer: 0=zero, 1=display, C=collision geometry, F=gentle finger motion")
    parser.add_argument("--smoke-seconds", type=float, default=0, help="Optional unactuated free dynamics smoke; no hard-setting during integration")
    return parser.parse_args()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vec(text, default="0 0 0"):
    return np.fromstring(text or default, sep=" ", dtype=float)


def rpy_matrix(text):
    r, p, y = vec(text)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def origin_matrix(element):
    out = np.eye(4)
    if element is not None:
        out[:3, :3] = rpy_matrix(element.get("rpy"))
        out[:3, 3] = vec(element.get("xyz"))
    return out


def axis_matrix(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*K + (1-math.cos(angle))*(K@K)


def parse_robot(path):
    root = ET.parse(path).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    rows = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        rows.append({"name": joint.get("name"), "type": joint.get("type"), "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"), "origin": origin_matrix(joint.find("origin")), "axis": vec(joint.find("axis").get("xyz")) if joint.find("axis") is not None else np.array([1., 0., 0.]), "limit": {k: float(v) for k, v in limit.attrib.items()} if limit is not None else {}})
    roots = set(links) - {row["child"] for row in rows}
    if len(roots) != 1 or len(links) != len(root.findall("link")) or len({r["name"] for r in rows}) != len(rows):
        raise ValueError("URDF must have one root and unique link/joint names")
    if len({row["child"] for row in rows}) != len(rows):
        raise ValueError("URDF child link has multiple parents")
    return root, links, rows, roots.pop()


def source_fk(rows, root_name, joint_values):
    transforms = {root_name: np.eye(4)}
    pending = list(rows)
    while pending:
        progressed = False
        for row in list(pending):
            if row["parent"] not in transforms:
                continue
            motion = np.eye(4)
            if row["type"] in ("revolute", "continuous"):
                motion[:3, :3] = axis_matrix(row["axis"], joint_values.get(row["name"], 0.))
            elif row["type"] == "prismatic":
                motion[:3, 3] = row["axis"] * joint_values.get(row["name"], 0.)
            elif row["type"] != "fixed":
                raise ValueError(f"unsupported assembly joint type: {row['type']}")
            transforms[row["child"]] = transforms[row["parent"]] @ row["origin"] @ motion
            pending.remove(row)
            progressed = True
        if not progressed:
            raise ValueError("disconnected or cyclic URDF")
    return transforms


def pose_vectors(model):
    zero = model.qpos0.copy()
    display = zero.copy()
    requested = {"elbow_L_Joint": -0.70, "elbow_R_Joint": -0.70, "proximal_roll_L_Joint": 0.10, "proximal_roll_R_Joint": -0.10, "head_pitch_Joint": 0.05}
    for name, angle in requested.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            lo, hi = model.jnt_range[jid]
            display[model.jnt_qposadr[jid]] = np.clip(angle, lo, hi)
    return {"zero": zero, "display": display}


def convert_scene(args):
    original_model = mujoco.MjModel.from_xml_path(str(args.urdf.resolve()))
    args.scene.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="tron2_mjcf_") as scratch:
        raw_path = Path(scratch) / "model.xml"
        mujoco.mj_saveLastXML(str(raw_path), original_model)
        root = ET.parse(raw_path).getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    raw_meshdir = compiler.get("meshdir", "")
    for mesh in root.findall("asset/mesh"):
        filename = mesh.get("file")
        if filename is None:
            continue
        raw = Path(filename)
        candidates = [raw] if raw.is_absolute() else [args.urdf.parent / raw_meshdir / raw, args.urdf.parent / raw, PROJECT / raw]
        resolved = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
        if resolved is None:
            raise FileNotFoundError(f"cannot rebase imported mesh {filename}; candidates={candidates}")
        mesh.set("file", Path(os.path.relpath(resolved, args.scene.parent)).as_posix())
    compiler.attrib.pop("meshdir", None)
    compiler.set("strippath", "false")
    compiler.set("discardvisual", "false")
    compiler.set("fusestatic", "false")
    root.set("model", "DACH_TRON2A + adapters + bilateral Revo3 | kinematic assembly scene")
    for geom in root.findall(".//geom"):
        is_visual = int(geom.get("contype", "1")) == 0 and int(geom.get("conaffinity", "1")) == 0
        geom.set("group", "1" if is_visual else "3")
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", "0.001")
    option.set("gravity", "0 0 -9.81")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", {"offwidth": str(args.width), "offheight": str(args.height)})
    ET.SubElement(visual, "quality", {"offsamples": "4", "shadowsize": "2048"})
    ET.SubElement(visual, "headlight", {"ambient": "0.35 0.35 0.35", "diffuse": "0.6 0.6 0.6", "specular": "0.15 0.15 0.15"})
    ET.SubElement(visual, "map", {"znear": "0.001", "zfar": "20"})
    assets = root.find("asset")
    if assets is None:
        assets = ET.SubElement(root, "asset")
    ET.SubElement(assets, "texture", {"name": "assembly_sky", "type": "skybox", "builtin": "gradient", "rgb1": "0.20 0.23 0.28", "rgb2": "0.07 0.08 0.10", "width": "256", "height": "1536"})
    ET.SubElement(assets, "material", {"name": "assembly_floor_material", "rgba": "0.30 0.33 0.37 1", "reflectance": "0.03", "shininess": "0.1"})
    world = root.find("worldbody")
    ET.SubElement(world, "geom", {"name": "assembly_ground", "type": "plane", "size": "3 3 0.1", "pos": "0 0 -0.0001", "material": "assembly_floor_material", "group": "0"})
    ET.SubElement(world, "light", {"name": "assembly_key", "pos": "2 -2 3.5", "dir": "-0.5 0.5 -1", "diffuse": "0.8 0.8 0.8", "castshadow": "true"})
    ET.SubElement(world, "light", {"name": "assembly_fill", "pos": "-2 1 2.5", "dir": "0.5 -0.2 -1", "diffuse": "0.4 0.45 0.5", "castshadow": "false"})
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")
    for name, qpos in pose_vectors(original_model).items():
        ET.SubElement(keyframe, "key", {"name": name, "qpos": " ".join(f"{v:.12g}" for v in qpos)})
    root.insert(0, ET.Comment("Generated by scripts/preview_assembly.py. No actuators or physical robot parameters are added or tuned. Use the script's kinematic viewer mode for joint sliders; ordinary physics playback is unactuated."))
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(args.scene, encoding="utf-8", xml_declaration=True)
    return original_model, mujoco.MjModel.from_xml_path(str(args.scene.resolve()))


def body_transform(data, body_id):
    result = np.eye(4)
    result[:3, :3] = data.xmat[body_id].reshape(3, 3)
    result[:3, 3] = data.xpos[body_id]
    return result


def check_relocation(scene):
    """Compile a second copy under an unrelated directory with the same layout."""
    root = ET.parse(scene).getroot()
    mesh_files = sorted({mesh.get("file") for mesh in root.findall("asset/mesh") if mesh.get("file")})
    if any(Path(filename).is_absolute() for filename in mesh_files):
        return {"pass": False, "error": "absolute mesh path in saved scene"}
    with tempfile.TemporaryDirectory(prefix="tron2_scene_relocation_") as scratch:
        relocated_root = Path(scratch).resolve()
        relocated_scene = relocated_root / scene.relative_to(PROJECT)
        relocated_scene.parent.mkdir(parents=True)
        shutil.copy2(scene, relocated_scene)
        for filename in mesh_files:
            original = (scene.parent/filename).resolve(strict=True)
            target = (relocated_scene.parent/filename).resolve()
            if not target.is_relative_to(relocated_root):
                return {"pass": False, "error": f"mesh reference escapes relocatable layout: {filename}"}
            target.parent.mkdir(parents=True, exist_ok=True)
            try:
                os.link(original, target)
            except OSError:
                shutil.copy2(original, target)
        reloaded = mujoco.MjModel.from_xml_path(str(relocated_scene))
        return {"pass": True, "relative_asset_count": len(mesh_files), "relocated_nq": reloaded.nq, "method": "Compiled scene copied into a separate temporary project layout with relative meshes; temporary test files removed after loading."}


def check_mesh_bindings(urdf, scene):
    """Reject basename aliasing by checking the source bytes for each body."""
    source = ET.parse(urdf).getroot()
    output = ET.parse(scene).getroot()
    assets = {mesh.get("name"): (scene.parent/mesh.get("file")).resolve(strict=True) for mesh in output.findall("asset/mesh") if mesh.get("file")}
    bodies = {body.get("name"): body for body in output.findall(".//body")}
    cached = {}
    def sha(path):
        if path not in cached:
            cached[path] = digest(path)
        return cached[path]
    compared = []
    mismatches = []
    source_paths = set()
    for link in source.findall("link"):
        name = link.get("name")
        body = bodies.get(name)
        if body is None:
            if link.findall("visual") or link.findall("collision"):
                mismatches.append({"link": name, "error": "body with source geometry missing from MJCF"})
            continue
        for kind, group in (("visual", "1"), ("collision", "3")):
            expected_paths = [(urdf.parent/geom.find("geometry/mesh").get("filename")).resolve(strict=True) for geom in link.findall(kind) if geom.find("geometry/mesh") is not None]
            actual_paths = [assets[geom.get("mesh")] for geom in body.findall("geom") if geom.get("group") == group and geom.get("mesh") is not None]
            source_paths.update(expected_paths)
            expected = Counter(sha(path) for path in expected_paths)
            actual = Counter(sha(path) for path in actual_paths)
            okay = expected == actual
            compared.append({"link": name, "kind": kind, "source_mesh_geom_count": len(expected_paths), "scene_mesh_geom_count": len(actual_paths), "source_file_sha256_multiset_matches": okay})
            if not okay:
                mismatches.append({"link": name, "kind": kind, "expected_files": [str(path.relative_to(PROJECT)) if path.is_relative_to(PROJECT) else str(path) for path in expected_paths], "actual_files": [str(path.relative_to(PROJECT)) if path.is_relative_to(PROJECT) else str(path) for path in actual_paths]})
    return {"pass": not mismatches, "source_unique_mesh_files": len(source_paths), "scene_mesh_asset_count": len(assets), "compared_body_geometry_groups": len(compared), "comparisons": compared, "mismatches": mismatches, "method": "Per body and visual/collision role: compare multisets of SHA256 over referenced mesh files. Distinct left/right meshes sharing a basename must not alias."}


def finite_state(data):
    names = ("qpos", "qvel", "qacc", "xpos", "xmat", "geom_xpos", "geom_xmat")
    return all(np.isfinite(getattr(data, name)).all() for name in names)


def geometry_contacts(model, data):
    rows = []
    for contact in data.contact[:data.ncon]:
        if float(contact.dist) >= 0:
            continue
        pair = []
        for gid in contact.geom:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(gid))
            body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid]))
            pair.append({"geom": name or f"geom_{int(gid)}", "body": body})
        rows.append({"distance_m": float(contact.dist), "pair": pair})
    rows.sort(key=lambda row: row["distance_m"])
    unique_pairs = {}
    for row in rows:
        pair = tuple(sorted(str(item["body"]) for item in row["pair"]))
        if pair not in unique_pairs:
            unique_pairs[pair] = {"links": list(pair), "minimum_distance_m": row["distance_m"], "negative_contact_count": 0}
        unique_pairs[pair]["negative_contact_count"] += 1
    return {"contact_count": int(data.ncon), "negative_distance_count": len(rows), "minimum_distance_m": rows[0]["distance_m"] if rows else None, "deepest_contacts": rows[:12], "deepest_link_pairs": sorted(unique_pairs.values(), key=lambda row: row["minimum_distance_m"]), "scope": "MuJoCo collision-proxy contacts at one hard-set pose; not a full-surface clearance certificate"}


def validate(args, original_model, model, robot):
    root, links, rows, root_name = robot
    moving = [row for row in rows if row["type"] != "fixed"]
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) for jid in range(model.njnt)]
    expected_names = [row["name"] for row in moving]
    checks = {"expected_dof": model.nq == model.nv == model.njnt == args.expected_dof, "source_joint_names_preserved": set(names) == set(expected_names), "all_joints_hinge": bool(np.all(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE)), "no_free_base_joint": bool(np.all(model.jnt_type != mujoco.mjtJoint.mjJNT_FREE)), "source_link_bodies_retained": all(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0 for name in links), "no_actuators_added": model.nu == 0}
    joint_rows = []
    for row in moving:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, row["name"])
        if jid < 0:
            raise ValueError(f"missing imported joint: {row['name']}")
        normalized_axis = row["axis"] / np.linalg.norm(row["axis"])
        expected_range = [row["limit"]["lower"], row["limit"]["upper"]]
        joint_rows.append({"name": row["name"], "qpos_address": int(model.jnt_qposadr[jid]), "axis": model.jnt_axis[jid].tolist(), "limits_rad": model.jnt_range[jid].tolist(), "axis_error": float(np.max(np.abs(model.jnt_axis[jid]-normalized_axis))), "limit_error_rad": float(np.max(np.abs(model.jnt_range[jid]-expected_range)))})
    checks["joint_axes_and_limits_preserved"] = all(row["axis_error"] <= 2e-6 and row["limit_error_rad"] <= 2e-5 for row in joint_rows)
    mass_errors, inertia_errors = [], []
    for name, link in links.items():
        ins = link.find("inertial")
        if ins is None:
            continue
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        mass = float(ins.find("mass").get("value"))
        mass_errors.append(abs(float(model.body_mass[bid])-mass))
        if mass == 0:
            continue
        a = {k: float(v) for k, v in ins.find("inertia").attrib.items()}
        I = np.array([[a["ixx"], a["ixy"], a["ixz"]], [a["ixy"], a["iyy"], a["iyz"]], [a["ixz"], a["iyz"], a["izz"]]])
        R = origin_matrix(ins.find("origin"))[:3, :3]
        quat_R = np.empty(9)
        mujoco.mju_quat2Mat(quat_R, model.body_iquat[bid])
        Ri = quat_R.reshape(3, 3)
        reconstructed = Ri @ np.diag(model.body_inertia[bid]) @ Ri.T
        inertia_errors.append(float(np.max(np.abs(reconstructed - R@I@R.T))))
    checks["positive_mass_values_preserved"] = max(mass_errors, default=0.) <= 2e-5
    checks["positive_mass_inertias_preserved"] = max(inertia_errors, default=0.) <= 2e-5
    # mj_saveLastXML serializes floating values to finite decimal precision.
    # Compare the compiled URDF directly as well as the reloaded MJCF.
    checks["source_and_scene_dof_match"] = original_model.nq == model.nq and original_model.nv == model.nv
    fixed_mounts = [row for row in rows if row["type"] == "fixed" and ("adapter" in (row["name"]+row["parent"]+row["child"]).lower() or ("wrist_roll" in row["parent"] and "grasper" not in row["child"]))]
    data = mujoco.MjData(model)
    poses = pose_vectors(model)
    fk_worst_position = fk_worst_rotation = 0.
    rigid_worst = 0.

    def evaluate(qpos):
        nonlocal fk_worst_position, fk_worst_rotation, rigid_worst
        mujoco.mj_resetData(model, data)
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        values = {name: float(qpos[model.jnt_qposadr[jid]]) for jid, name in enumerate(names)}
        transforms = source_fk(rows, root_name, values)
        p_error = r_error = 0.
        for name, expected in transforms.items():
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            actual = body_transform(data, bid)
            p_error = max(p_error, float(np.linalg.norm(actual[:3, 3]-expected[:3, 3])))
            r_error = max(r_error, float(np.max(np.abs(actual[:3, :3]-expected[:3, :3]))))
        fk_worst_position = max(fk_worst_position, p_error)
        fk_worst_rotation = max(fk_worst_rotation, r_error)
        for row in fixed_mounts:
            parent = body_transform(data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, row["parent"]))
            child = body_transform(data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, row["child"]))
            actual = np.linalg.inv(parent) @ child
            rigid_worst = max(rigid_worst, float(np.max(np.abs(actual-row["origin"]))))
        return finite_state(data), p_error, r_error

    pose_reports = {}
    for name, qpos in poses.items():
        finite, position_error, rotation_error = evaluate(qpos)
        pose_reports[name] = {"finite": finite, "fk_max_position_error_m": position_error, "fk_max_rotation_matrix_error": rotation_error, "joint_values_rad": {n: float(qpos[model.jnt_qposadr[j]]) for j, n in enumerate(names)}, "collision_proxy_diagnostic": geometry_contacts(model, data)}
    sweep = []
    for jid, name in enumerate(names):
        adr = model.jnt_qposadr[jid]
        lo, hi = model.jnt_range[jid]
        base = float(poses["zero"][adr])
        eps = min(1e-6, (hi-lo)*1e-4)
        values = np.linspace(max(lo+eps, base-0.25), min(hi-eps, base+0.25), 5)
        finite = True
        pmax = rmax = 0.
        rotations = []
        for value in values:
            qpos = poses["zero"].copy()
            qpos[adr] = value
            okay, pe, re = evaluate(qpos)
            finite &= okay
            pmax, rmax = max(pmax, pe), max(rmax, re)
            rotations.append(data.xmat[model.jnt_bodyid[jid]].copy())
        rotation_change = float(np.linalg.norm(rotations[-1]-rotations[0]))
        sweep.append({"joint": name, "sample_values_rad": values.tolist(), "bounded": bool(np.all(values >= lo) and np.all(values <= hi)), "finite": bool(finite), "child_rotation_change_frobenius": rotation_change, "fk_max_position_error_m": pmax, "fk_max_rotation_matrix_error": rmax, "pass": bool(finite and rotation_change > 1e-6 and pmax < 5e-5 and rmax < 5e-5)})
    checks["all_joint_individual_sweeps_pass"] = len(sweep) == args.expected_dof and all(row["pass"] and row["bounded"] for row in sweep)
    checks["source_fk_preserved"] = fk_worst_position < 5e-5 and fk_worst_rotation < 5e-5
    checks["mount_fixed_relationships_preserved"] = len(fixed_mounts) >= 4 and rigid_worst < 5e-5
    relocation = check_relocation(args.scene)
    checks["scene_relocation_pass"] = relocation["pass"] and relocation.get("relocated_nq") == args.expected_dof
    mesh_bindings = check_mesh_bindings(args.urdf, args.scene)
    checks["per_body_source_meshes_preserved"] = mesh_bindings["pass"]
    report = {
        "status": "kinematic_assembly_pass" if all(checks.values()) else "kinematic_assembly_fail",
        "claim_boundary": "Forward-kinematics assembly and load validation only. No hard-set image, joint sweep, or optional finite smoke demonstrates collision freedom, control tracking, grasp success, print fit, or physical stability.",
        "mujoco_version": mujoco.__version__,
        "source_urdf": str(args.urdf.resolve()), "source_urdf_sha256": digest(args.urdf),
        "scene": str(args.scene.resolve()), "scene_sha256": digest(args.scene), "producer_sha256": digest(__file__),
        "model_counts": {"nq": model.nq, "nv": model.nv, "njnt": model.njnt, "nbody": model.nbody, "ngeom": model.ngeom, "nmesh": model.nmesh, "nu": model.nu},
        "checks": checks, "scene_relocation": relocation, "mesh_bindings": mesh_bindings,
        "joint_schema": joint_rows,
        "mass_and_inertia_preservation": {"max_mass_error_kg": max(mass_errors, default=0.), "max_inertia_component_error_kg_m2": max(inertia_errors, default=0.), "roundtrip_note": "Saved MJCF rounds numeric values; tolerances reflect XML serialization, not changed physical parameters."},
        "poses": pose_reports, "joint_sweep": sweep,
        "fk_worst_position_error_m": fk_worst_position, "fk_worst_rotation_matrix_error": fk_worst_rotation,
        "fixed_mount_joint_names": [r["name"] for r in fixed_mounts], "mount_rigidity_max_transform_component_error": rigid_worst,
        "physics_parameters_modified": [],
        "scene_additions": ["static ground plane at z=-0.0001 m", "lights/background", "visual group=1 and collision group=3", "zero/display keyframes", "1 ms simulation timestep and Earth gravity"],
        "dynamics_smoke": {"status": "not_run"}, "render": {"status": "not_run"},
    }
    return report, poses


def make_camera(target, distance, azimuth, elevation):
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = target
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    return camera


def render_images(args, model, poses):
    from PIL import Image, ImageDraw, ImageFont
    options = mujoco.MjvOption()
    mujoco.mjv_defaultOption(options)
    options.geomgroup[:] = [1, 1, 0, 0, 0, 0]
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 28) if Path(font_path).exists() else ImageFont.load_default()
    small = ImageFont.truetype(font_path, 18) if Path(font_path).exists() else ImageFont.load_default()
    data = mujoco.MjData(model)
    outputs = {}
    with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
        def shot(qpos, camera):
            mujoco.mj_resetData(model, data)
            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=options)
            return Image.fromarray(renderer.render().copy())

        def panel(images, labels, target, headline):
            canvas = Image.new("RGB", (len(images)*args.width, args.height+110), (17, 20, 26))
            draw = ImageDraw.Draw(canvas)
            draw.text((26, 13), headline, font=font, fill=(232, 238, 245))
            draw.text((26, 51), "Kinematic assembly preview | authored visual meshes and URDF materials | no dynamics claim", font=small, fill=(160, 175, 195))
            for i, (im, label) in enumerate(zip(images, labels)):
                canvas.paste(im, (i*args.width, 110))
                draw.text((i*args.width+26, 80), label, font=small, fill=(208, 222, 236))
            canvas.save(target)
            outputs[target.name] = {"path": str(target.resolve()), "sha256": digest(target), "size_px": list(canvas.size)}

        camera = make_camera([0., 0., 0.86], 2.75, 145, -12)
        panel([shot(poses["zero"], camera), shot(poses["display"], camera)], ["01 | Zero pose / arms at rest", "02 | Mild elbow flexion (-0.70 rad)"], args.report_dir/"assembly_overview.png", "DACH_TRON2A + printed adapters + bilateral Revo3")
        for side, short in (("left", "L"), ("right", "R")):
            mujoco.mj_resetData(model, data)
            data.qpos[:] = poses["display"]
            mujoco.mj_forward(model, data)
            wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"wrist_roll_{short}_Link")
            hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_hand_base_link")
            if min(wrist_id, hand_id) < 0:
                raise ValueError(f"missing {side} wrist/hand base body for close-up")
            wrist = data.xpos[wrist_id].copy()
            hand = data.xpos[hand_id].copy()
            hand_R = data.xmat[hand_id].reshape(3,3)
            target = 0.3*wrist + 0.7*(hand + hand_R@np.array([0., 0., .065]))
            azimuth = 150 if side == "left" else 30
            cameras = [make_camera(target, .51, azimuth, -15), make_camera(target, .51, azimuth+105, -25)]
            panel([shot(poses["display"], cam) for cam in cameras], [f"{side.title()} wrist / adapter / palm", "Second angle | attachment geometry"], args.report_dir/f"mount_{side}.png", f"{side.title()} Revo3 attachment | visual inspection")
    return {"status": "pass", "backend": os.environ.get("MUJOCO_GL"), "visual_geom_group": 1, "collision_geom_group_hidden": 3, "images": outputs}


def dynamics_smoke(model, poses, seconds):
    data = mujoco.MjData(model)
    data.qpos[:] = poses["display"]
    mujoco.mj_forward(model, data)
    steps = max(1, math.ceil(seconds/model.opt.timestep))
    finite = True
    for _ in range(steps):
        mujoco.mj_step(model, data)
        if not finite_state(data):
            finite = False
            break
    warnings = {str(mujoco.mjtWarning(i)): int(data.warning[i].number) for i in range(int(mujoco.mjtWarning.mjNWARNING)) if data.warning[i].number}
    return {"status": "finite_unactuated_smoke_only" if finite and not warnings else "failed_or_warning", "requested_seconds": seconds, "elapsed_simulation_seconds": float(data.time), "steps_requested": steps, "finite": finite, "warnings": warnings, "maximum_abs_qvel_rad_s": float(np.max(np.abs(data.qvel))), "maximum_abs_qpos_change_rad": float(np.max(np.abs(data.qpos-poses["display"]))), "method": "unactuated mj_step with gravity and collisions, no qpos reset or external holding forces during integration", "claim_boundary": "Finite arithmetic over this interval does not demonstrate stability or control success."}


def interactive(model, poses):
    import mujoco.viewer
    data = mujoco.MjData(model)
    data.qpos[:] = poses["display"]
    mujoco.mj_forward(model, data)
    pending = {"pose": None, "collision": False, "fingers": False}
    finger_joints = [jid for jid in range(model.njnt) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or "").startswith(("left_", "right_")) and any(token in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or "") for token in ("_MCP_", "_PIP_", "_DIP_"))]
    def key_callback(key):
        if key in (ord("0"), ord("1")):
            pending["pose"] = "zero" if key == ord("0") else "display"
            pending["fingers"] = False
        if key in (ord("C"), ord("c")):
            pending["collision"] = not pending["collision"]
        if key in (ord("F"), ord("f")):
            pending["fingers"] = not pending["fingers"]
    print("Kinematic viewer: no physics integration. Joint sliders are supported. 0=zero, 1=display, C=collision meshes, F=gentle finger motion.", flush=True)
    start_time = time.monotonic()
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = [0., 0., .86]
        viewer.cam.distance = 2.75
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -12
        while viewer.is_running():
            with viewer.lock():
                if pending["pose"]:
                    data.qpos[:] = poses[pending["pose"]]
                    pending["pose"] = None
                if pending["fingers"]:
                    bend = .14*(1-math.cos((time.monotonic()-start_time)*1.5))
                    for jid in finger_joints:
                        data.qpos[model.jnt_qposadr[jid]] = np.clip(bend, *model.jnt_range[jid])
                viewer.opt.geomgroup[:] = [1, not pending["collision"], 0, pending["collision"], 0, 0]
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(.02)


def main():
    args = arguments()
    args.urdf = args.urdf.resolve()
    args.scene = args.scene.resolve()
    args.report_dir = args.report_dir.resolve()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    os.environ["MUJOCO_GL"] = args.backend
    global mujoco, np
    import mujoco
    import numpy as np
    report_path = args.report_dir/"mujoco_validation.json"
    report = {"status": "failed", "source_urdf": str(args.urdf), "producer_sha256": digest(__file__)}
    try:
        robot = parse_robot(args.urdf)
        original, model = convert_scene(args)
        report, poses = validate(args, original, model, robot)
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        if not args.skip_render:
            try:
                report["render"] = render_images(args, model, poses)
            except Exception as exc:
                report["render"] = {"status": "failed", "backend": args.backend, "error": str(exc), "traceback": traceback.format_exc()}
        if args.smoke_seconds > 0:
            report["dynamics_smoke"] = dynamics_smoke(model, poses, args.smoke_seconds)
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        print(json.dumps({"status": report["status"], "checks": report["checks"], "render": report["render"]["status"], "report": str(report_path), "scene": str(args.scene)}, indent=2), flush=True)
        if args.viewer:
            interactive(model, poses)
        return 0 if all(report["checks"].values()) and report["render"]["status"] != "failed" else 2
    except Exception as exc:
        report.update({"status": "failed", "error": str(exc), "traceback": traceback.format_exc()})
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        print(json.dumps({"status": "failed", "error": str(exc), "report": str(report_path)}, indent=2), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
