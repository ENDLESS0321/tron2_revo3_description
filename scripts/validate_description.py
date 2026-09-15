#!/usr/bin/env python3
"""Statically validate the assembled DACH_TRON2A + two Revo3 description.

Uses only the Python standard library. This is a structural/model-data check;
it does not certify collision clearance, hardware calibration, or dynamics.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def eigenvalues_symmetric_3x3(matrix: list[list[float]]) -> list[float]:
    """Jacobi diagonalization, with a scale-relative stopping criterion."""
    a = [row[:] for row in matrix]
    for _ in range(40):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(a[ij[0]][ij[1]]))
        scale = max(abs(value) for row in a for value in row)
        if scale == 0.0 or abs(a[p][q]) <= scale * 1e-14:
            break
        theta = 0.5 * math.atan2(2.0 * a[p][q], a[q][q] - a[p][p])
        c, s = math.cos(theta), math.sin(theta)
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = a[q][p] = 0.0
        for k in range(3):
            if k not in (p, q):
                akp, akq = a[k][p], a[k][q]
                a[k][p] = a[p][k] = c * akp - s * akq
                a[k][q] = a[q][k] = s * akp + c * akq
    return sorted(a[i][i] for i in range(3))


def strings_in_json(value: object) -> set[str]:
    if isinstance(value, str):
        return {value}
    if isinstance(value, list):
        return set().union(*(strings_in_json(item) for item in value)) if value else set()
    if isinstance(value, dict):
        return set(value).union(*(strings_in_json(item) for item in value.values()))
    return set()


def validate(urdf_path: Path, joint_map_path: Path, run_check_urdf: bool = True) -> dict:
    errors: list[str] = []
    warnings: list[str] = []
    report: dict = {
        "schema": "tron2_revo3_urdf_validation_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "urdf": str(urdf_path),
        "joint_map": str(joint_map_path),
        "scope": "Static XML, topology, joint identity, mesh references, and inertial data; not a dynamics or clearance certificate.",
        "errors": errors,
        "warnings": warnings,
    }

    def vector(value: str | None, count: int, context: str) -> list[float] | None:
        try:
            values = [float(token) for token in (value or "").split()]
            if len(values) != count or not all(math.isfinite(x) for x in values):
                raise ValueError("wrong length or non-finite value")
            return values
        except (TypeError, ValueError):
            errors.append(f"{context}: expected {count} finite numeric values, got {value!r}")
            return None

    try:
        source_bytes = urdf_path.read_bytes()
        robot = ET.fromstring(source_bytes)
        report["urdf_sha256"] = hashlib.sha256(source_bytes).hexdigest()
    except (OSError, ET.ParseError) as exc:
        errors.append(f"Cannot read/parse generated URDF: {exc}")
        report["valid"] = False
        return report
    if robot.tag != "robot":
        errors.append(f"Expected <robot>, got <{robot.tag}>")

    link_elements = robot.findall("link")
    joint_elements = robot.findall("joint")
    link_counts = Counter(link.get("name", "") for link in link_elements)
    joint_counts = Counter(joint.get("name", "") for joint in joint_elements)
    for kind, counts in (("link", link_counts), ("joint", joint_counts)):
        for name, count in counts.items():
            if not name or count != 1:
                errors.append(f"Invalid/duplicate {kind} name {name!r}: count={count}")
    links = {link.get("name", ""): link for link in link_elements}
    joints = {joint.get("name", ""): joint for joint in joint_elements}
    parent_joints: dict[str, list[ET.Element]] = defaultdict(list)
    children: dict[str, list[str]] = defaultdict(list)
    edges: dict[tuple[str, str], list[ET.Element]] = defaultdict(list)

    for joint in joint_elements:
        name = joint.get("name", "")
        p, c = joint.find("parent"), joint.find("child")
        parent = p.get("link") if p is not None else None
        child = c.get("link") if c is not None else None
        if parent not in links or child not in links:
            errors.append(f"Joint {name}: dangling/missing link reference {parent!r} -> {child!r}")
            continue
        if parent == child:
            errors.append(f"Joint {name}: self cycle on {parent}")
        parent_joints[child].append(joint)
        children[parent].append(child)
        edges[(parent, child)].append(joint)
    for child, incoming in parent_joints.items():
        if len(incoming) != 1:
            errors.append(f"Link {child}: {len(incoming)} incoming joints")
    roots = sorted(set(links) - set(parent_joints))
    if roots != ["world"]:
        errors.append(f"Expected exactly one world root, got {roots}")
    if len(joint_elements) != len(link_elements) - 1:
        errors.append("Tree must have exactly links - 1 joints")
    reached: set[str] = set()
    pending = ["world"] if "world" in links else []
    while pending:
        node = pending.pop()
        if node in reached:
            errors.append(f"Cycle or multiple traversal detected at link {node}")
            continue
        reached.add(node)
        pending.extend(children.get(node, []))
    if reached != set(links):
        errors.append(f"Unreachable links: {sorted(set(links) - reached)}")
    for name in list(links) + list(joints):
        if "grasper" in name.lower() or "gripper" in name.lower():
            errors.append(f"Original grasper branch remains: {name}")

    def require_fixed_edge(parent: str, child: str) -> None:
        found = edges.get((parent, child), [])
        if len(found) != 1 or found[0].get("type") != "fixed":
            errors.append(f"Expected one fixed joint {parent} -> {child}")

    require_fixed_edge("world", "base_Link")
    for side, suffix in (("left", "L"), ("right", "R")):
        require_fixed_edge(f"wrist_roll_{suffix}_Link", f"{side}_adapter_link")
        require_fixed_edge(f"{side}_adapter_link", f"{side}_hand_base_link")

    upstream_paths = {
        "tron2": PROJECT_ROOT / "vendor/tron2-robot-description/tron2a/DACH_TRON2A/urdf/robot.urdf",
        "left_hand": PROJECT_ROOT / "vendor/brainco-revo3/revo3_system/urdf/revo3_left.urdf",
        "right_hand": PROJECT_ROOT / "vendor/brainco-revo3/revo3_system/urdf/revo3_right.urdf",
    }
    expected_groups: dict[str, set[str]] = {}
    expected_upstream_links: set[str] = set()
    for group, path in upstream_paths.items():
        try:
            upstream = ET.parse(path).getroot()
            expected_groups[group] = {joint.get("name", "") for joint in upstream.findall("joint") if joint.get("type") == "revolute"}
            expected_upstream_links.update(link.get("name", "") for link in upstream.findall("link") if "grasper" not in link.get("name", "").lower())
        except (OSError, ET.ParseError) as exc:
            errors.append(f"Cannot read upstream identity source {path}: {exc}")
            expected_groups[group] = set()
    if not expected_upstream_links <= set(links):
        errors.append(f"Upstream link names were removed/changed: {sorted(expected_upstream_links - set(links))}")
    expected_revolute = set().union(*expected_groups.values())
    revolute = {joint.get("name", "") for joint in joint_elements if joint.get("type") == "revolute"}
    counts_by_type = Counter(joint.get("type", "") for joint in joint_elements)
    if len(revolute) != 58 or counts_by_type["revolute"] != 58:
        errors.append(f"Expected 58 unique revolute joints, got {len(revolute)} / {counts_by_type['revolute']} entries")
    if revolute != expected_revolute:
        errors.append(f"Revolute identities differ from upstream; missing={sorted(expected_revolute - revolute)}, extra={sorted(revolute - expected_revolute)}")
    for group, expected_count in (("tron2", 16), ("left_hand", 21), ("right_hand", 21)):
        actual = len(expected_groups[group] & revolute)
        if len(expected_groups[group]) != expected_count or actual != expected_count:
            errors.append(f"{group}: expected {expected_count} upstream revolute names preserved, got {actual}")
    arms = {f"{part}_{suffix}_Joint" for suffix in ("L", "R") for part in ("proximal_pitch", "proximal_roll", "proximal_yaw", "elbow", "wrist_yaw", "wrist_pitch", "wrist_roll")}
    heads = {"head_yaw_Joint", "head_pitch_Joint"}
    if not arms <= revolute or not heads <= revolute:
        errors.append("The 14 arm and 2 head revolute joints are not all present")
    if any(kind not in {"fixed", "revolute"} for kind in counts_by_type):
        errors.append(f"Unexpected joint types: {dict(counts_by_type)}")

    for joint in joint_elements:
        name = joint.get("name", "")
        if joint.get("type") == "revolute":
            axis = joint.find("axis")
            values = vector(axis.get("xyz") if axis is not None else None, 3, f"Joint {name} axis")
            if values is not None:
                norm = math.sqrt(sum(x * x for x in values))
                if norm <= 1e-12:
                    errors.append(f"Joint {name}: zero axis")
                elif abs(norm - 1.0) > 1e-3:
                    warnings.append(f"Joint {name}: axis is not normalized (norm={norm})")
            limit = joint.find("limit")
            numeric_limits = {}
            for key in ("lower", "upper", "effort", "velocity"):
                parsed = vector(limit.get(key) if limit is not None else None, 1, f"Joint {name} limit {key}")
                if parsed is not None:
                    numeric_limits[key] = parsed[0]
            if len(numeric_limits) == 4:
                if numeric_limits["lower"] >= numeric_limits["upper"]:
                    errors.append(f"Joint {name}: lower limit must be below upper limit")
                if numeric_limits["effort"] <= 0 or numeric_limits["velocity"] <= 0:
                    errors.append(f"Joint {name}: effort and velocity limits must be positive")
            mimic = joint.find("mimic")
            if mimic is not None:
                errors.append(f"Joint {name}: unexpected mimic on independently actuated model")

    for index, origin in enumerate(robot.findall(".//origin")):
        for key in ("xyz", "rpy"):
            if key in origin.attrib:
                vector(origin.get(key), 3, f"Origin {index} {key}")
    mesh_paths: set[Path] = set()
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if not filename or "://" in filename or Path(filename).is_absolute():
            errors.append(f"Mesh path must be a nonempty relative filesystem path: {filename!r}")
            continue
        path = (urdf_path.parent / filename).resolve()
        mesh_paths.add(path)
        if not path.is_file() or path.stat().st_size == 0:
            errors.append(f"Mesh missing or empty: {filename}")
        if "scale" in mesh.attrib:
            scale = vector(mesh.get("scale"), 3, f"Mesh {filename} scale")
            if scale is not None and any(x <= 0 for x in scale):
                errors.append(f"Mesh {filename}: scale must be positive")
    for tag, attributes in (("box", {"size": 3}), ("sphere", {"radius": 1}), ("cylinder", {"radius": 1, "length": 1})):
        for geometry in robot.findall(f".//geometry/{tag}"):
            for attribute, length in attributes.items():
                values = vector(geometry.get(attribute), length, f"{tag} {attribute}")
                if values is not None and any(x <= 0 for x in values):
                    errors.append(f"{tag} {attribute} must be positive")

    inertial_details = []
    fixed_without_inertial = []
    total_mass = 0.0
    expected_markers = {f"{side}_palm" for side in ("left", "right")} | {f"{side}_{finger}_tip_Link" for side in ("left", "right") for finger in ("thumb", "index", "middle", "ring", "little")}
    for name, link in links.items():
        inertial = link.find("inertial")
        incoming = parent_joints.get(name, [])
        movable = any(joint.get("type") != "fixed" for joint in incoming)
        if name in expected_markers and inertial is not None:
            errors.append(f"Fixed helper {name}: expected upstream invalid/zero-mass inertial to be removed")
        if inertial is None:
            if movable:
                errors.append(f"Movable link {name}: missing inertial")
            else:
                fixed_without_inertial.append(name)
            continue
        mass_element, inertia_element = inertial.find("mass"), inertial.find("inertia")
        mass = vector(mass_element.get("value") if mass_element is not None else None, 1, f"Link {name} mass")
        if mass is not None:
            total_mass += mass[0]
            if mass[0] <= 0:
                errors.append(f"Link {name}: inertial mass must be positive; remove inertial from massless fixed markers")
        components = {}
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            value = vector(inertia_element.get(key) if inertia_element is not None else None, 1, f"Link {name} {key}")
            if value is not None:
                components[key] = value[0]
        if len(components) == 6:
            a = components
            eigenvalues = eigenvalues_symmetric_3x3([[a["ixx"], a["ixy"], a["ixz"]], [a["ixy"], a["iyy"], a["iyz"]], [a["ixz"], a["iyz"], a["izz"]]])
            positive = eigenvalues[0] > 0
            triangle = eigenvalues[2] <= eigenvalues[0] + eigenvalues[1] + max(abs(x) for x in eigenvalues) * 1e-7
            if not positive:
                errors.append(f"Link {name}: inertia is not positive definite ({eigenvalues})")
            if not triangle:
                errors.append(f"Link {name}: principal moments violate triangle inequality ({eigenvalues})")
            inertial_details.append({"link": name, "mass_kg": mass[0] if mass else None, "principal_moments_kg_m2": eigenvalues, "positive_definite": positive, "triangle_inequality": triangle})
    if not expected_markers <= set(links):
        errors.append(f"Expected fixed hand reference markers missing: {sorted(expected_markers - set(links))}")

    try:
        joint_map_bytes = joint_map_path.read_bytes()
        mapping = json.loads(joint_map_bytes)
        if not isinstance(mapping, (dict, list)):
            errors.append("joint_map.json must be a JSON object or array")
        map_names = strings_in_json(mapping)
        absent = expected_revolute - map_names
        if absent:
            errors.append(f"joint_map.json lacks revolute joint names: {sorted(absent)}")
        report["joint_map_check"] = {"sha256": hashlib.sha256(joint_map_bytes).hexdigest(), "expected_revolute_names_found": len(expected_revolute & map_names), "scope": "All 58 upstream revolute names occur in JSON keys or string values; field semantics and motor ordering require downstream controller validation."}
    except (OSError, ValueError) as exc:
        errors.append(f"Cannot read/parse joint_map.json: {exc}")

    parser_executable = shutil.which("check_urdf") if run_check_urdf else None
    if parser_executable:
        try:
            result = subprocess.run([parser_executable, str(urdf_path)], capture_output=True, text=True, timeout=30, check=False)
            report["check_urdf"] = {"available": True, "executable": parser_executable, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
            if result.returncode != 0:
                errors.append(f"check_urdf failed with exit code {result.returncode}")
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(f"check_urdf could not finish: {exc}")
    else:
        report["check_urdf"] = {"available": False, "requested": run_check_urdf}
        if run_check_urdf:
            warnings.append("check_urdf is unavailable; standard-library XML/topology validation was performed")

    report.update({
        "robot_name": robot.get("name"),
        "counts": {"links": len(link_elements), "joints": len(joint_elements), "joint_types": dict(counts_by_type), "arm_revolute": len(arms & revolute), "head_revolute": len(heads & revolute), "left_hand_revolute": len(expected_groups["left_hand"] & revolute), "right_hand_revolute": len(expected_groups["right_hand"] & revolute), "unique_mesh_files": len(mesh_paths), "mesh_references": len(robot.findall(".//mesh"))},
        "topology": {"roots": roots, "reachable_links": len(reached)},
        "total_declared_mass_kg": total_mass,
        "inertials": inertial_details,
        "fixed_links_without_inertial": sorted(fixed_without_inertial),
        "expected_helper_inertial_removals": sorted(expected_markers),
        "valid": not errors,
    })
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=PROJECT_ROOT / "urdf/tron2_dach_revo3.urdf")
    parser.add_argument("--joint-map", type=Path, default=PROJECT_ROOT / "config/joint_map.json")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "reports/urdf_validation.json")
    parser.add_argument("--skip-check-urdf", action="store_true")
    args = parser.parse_args()
    report = validate(args.urdf.resolve(), args.joint_map.resolve(), not args.skip_check_urdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"valid": report["valid"], "counts": report.get("counts"), "errors": report["errors"], "warnings": report["warnings"], "report": str(args.output.resolve())}, indent=2, ensure_ascii=False))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
