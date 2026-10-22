#!/usr/bin/env python3
"""Replace only a mounted right-hand subtree with an official Revo3 URDF subtree.

This builder preserves the source assembly and its incoming hand mount, keeps
official inertial data unchanged, merges fixed no-inertia geometry with exact
URDF transforms, and copies only the resulting referenced mesh/texture closure.
It writes a new output directory and never edits either input asset.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


HAND_ROOT = "right_hand_base_link"
MOUNT_JOINT = "right_hand_base_joint"
ACTIVE_TYPES = {"revolute", "continuous", "prismatic"}
TIP_SUFFIX = "_tip_Link"
INERTIA_FIELDS = ("mass", "inertia")


class BuildError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify_mesh_manifest(asset_root: Path, records: list[dict[str, str]]) -> None:
    """Fail closed if any copied mesh is missing or differs from its recorded bytes."""
    asset_root = asset_root.resolve(strict=True)
    for record in records:
        relative = Path(record["output_path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise BuildError(f"unsafe mesh manifest path: {relative}")
        path = (asset_root / relative).resolve(strict=True)
        try:
            path.relative_to(asset_root)
        except ValueError as exc:
            raise BuildError(f"mesh manifest path escapes asset root: {relative}") from exc
        if sha256(path) != record["sha256"]:
            raise BuildError(f"mesh manifest hash mismatch: {relative}")


def xml_signature(node: ET.Element) -> tuple[Any, ...]:
    return (node.tag, tuple(sorted(node.attrib.items())), (node.text or "").strip(),
            tuple(xml_signature(child) for child in node))


def _robot(path: Path) -> tuple[ET.ElementTree, ET.Element]:
    try:
        tree = ET.parse(path)
    except (OSError, ET.ParseError) as exc:
        raise BuildError(f"cannot parse URDF {path}: {exc}") from exc
    root = tree.getroot()
    if root.tag != "robot":
        raise BuildError(f"expected <robot> root in {path}")
    return tree, root


def _maps(root: ET.Element) -> tuple[dict[str, ET.Element], dict[str, ET.Element], dict[str, list[ET.Element]]]:
    links = {link.get("name", ""): link for link in root.findall("link")}
    joints = {joint.get("name", ""): joint for joint in root.findall("joint")}
    if "" in links or len(links) != len(root.findall("link")):
        raise BuildError("URDF link names must be present and unique")
    if "" in joints or len(joints) != len(root.findall("joint")):
        raise BuildError("URDF joint names must be present and unique")
    children: dict[str, list[ET.Element]] = {}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise BuildError(f"joint {joint.get('name')} lacks parent/child")
        if parent.get("link") not in links or child.get("link") not in links:
            raise BuildError(f"joint {joint.get('name')} references an absent link")
        children.setdefault(parent.get("link", ""), []).append(joint)
    return links, joints, children


def _hand_descendants(root: ET.Element, root_link: str | None = None) -> tuple[set[str], dict[str, ET.Element]]:
    root_link = HAND_ROOT if root_link is None else root_link
    links, joints, children = _maps(root)
    if root_link not in links:
        raise BuildError(f"URDF has no {root_link}")
    subtree = {root_link}
    parent_joint: dict[str, ET.Element] = {}
    pending = [root_link]
    while pending:
        parent = pending.pop()
        for joint in children.get(parent, []):
            child = joint.find("child").get("link")
            if child in subtree:
                raise BuildError(f"cycle or duplicate parent detected at link {child}")
            subtree.add(child)
            parent_joint[child] = joint
            pending.append(child)
    return subtree, parent_joint


def _pose(origin: ET.Element | None) -> tuple[list[float], list[float]]:
    if origin is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    try:
        xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
        rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
    except ValueError as exc:
        raise BuildError("URDF origin xyz/rpy must contain numeric values") from exc
    if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(x) for x in xyz + rpy):
        raise BuildError("URDF origin xyz/rpy must contain three finite values each")
    return xyz, rpy


def _matrix(xyz: list[float], rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _multiply(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rpy_from_rotation(r: list[list[float]]) -> list[float]:
    pitch = math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0]))
    if abs(math.cos(pitch)) > 1e-12:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-r[0][1], r[1][1])
    return [roll, pitch, yaw]


def _pose_text(matrix: list[list[float]]) -> tuple[str, str]:
    xyz = [matrix[0][3], matrix[1][3], matrix[2][3]]
    rpy = _rpy_from_rotation(matrix)
    return " ".join(format(v, ".17g") for v in xyz), " ".join(format(v, ".17g") for v in rpy)


def _quaternion_wxyz_from_matrix(matrix: list[list[float]]) -> list[float]:
    r = matrix
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm == 0.0:
        raise BuildError("cannot convert virtual-frame rotation to a finite quaternion")
    return [w / norm, x / norm, y / norm, z / norm]


def _set_origin(node: ET.Element, matrix: list[list[float]]) -> None:
    origin = node.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        node.insert(0, origin)
    xyz, rpy = _pose_text(matrix)
    origin.set("xyz", xyz)
    origin.set("rpy", rpy)


def _fixed_chain_transform(parent_name: str, child_name: str,
                           parent_joint: dict[str, ET.Element]) -> tuple[list[list[float]], list[str]]:
    # Walk from child toward the target inertial parent, then compose forward.
    links_and_joints: list[ET.Element] = []
    cursor = child_name
    while cursor != parent_name:
        joint = parent_joint.get(cursor)
        if joint is None or joint.get("type") != "fixed":
            raise BuildError(f"non-inertial attachment {child_name} is not on a fixed chain to {parent_name}")
        links_and_joints.append(joint)
        cursor = joint.find("parent").get("link", "")
    transform = _matrix([0.0] * 3, [0.0] * 3)
    for joint in reversed(links_and_joints):
        xyz, rpy = _pose(joint.find("origin"))
        transform = _multiply(transform, _matrix(xyz, rpy))
    chain = [j.get("name", "") for j in reversed(links_and_joints)]
    return transform, chain


def _inertia_signature(link: ET.Element) -> tuple[Any, ...] | None:
    inertial = link.find("inertial")
    return xml_signature(inertial) if inertial is not None else None


def _alias_link(name: str, old_links: set[str]) -> str:
    if name == HAND_ROOT:
        return HAND_ROOT
    if name.endswith("_link"):
        legacy = name[:-5] + "_Link"
        if legacy in old_links:
            return legacy
    return name


def _joint_order(root: ET.Element, names: set[str]) -> list[str]:
    return [joint.get("name", "") for joint in root.findall("joint")
            if joint.get("name") in names and joint.get("type") in ACTIVE_TYPES]


def _resolve_mesh(source_root: Path, urdf_path: Path, uri: str) -> Path:
    if uri.startswith("package://") or Path(uri).is_absolute():
        raise BuildError(f"unsupported external/absolute mesh URI: {uri}")
    root = source_root.resolve(strict=True)
    candidates = [(urdf_path.parent / uri).resolve()]
    rooted = (root / uri).resolve()
    if rooted not in candidates:
        candidates.append(rooted)
    candidate = next((item for item in candidates if item.is_file()), None)
    if candidate is None:
        raise FileNotFoundError(uri)
    candidate = candidate.resolve(strict=True)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise BuildError(f"mesh URI escapes its declared asset root: {uri}") from exc
    if not candidate.is_file():
        raise BuildError(f"mesh URI is not a file: {uri}")
    return candidate


def _copy_mesh_closure(root: ET.Element, output_stage: Path,
                        source_roots: dict[Path, Path]) -> list[dict[str, str]]:
    records: dict[str, dict[str, str]] = {}
    for node in root.iter():
        if node.tag not in {"mesh", "texture"}:
            continue
        key = "filename"
        uri = node.get(key)
        if not uri:
            raise BuildError(f"{node.tag} has an empty filename")
        matches: list[tuple[Path, Path]] = []
        for urdf_path, source_root in source_roots.items():
            try:
                resolved = _resolve_mesh(source_root, urdf_path, uri)
            except (FileNotFoundError, BuildError):
                continue
            matches.append((resolved, source_root.resolve()))
        unique_sources = {path for path, _ in matches}
        if not unique_sources:
            raise BuildError(f"mesh URI did not resolve inside any declared source asset root: {uri}")
        hashes = {sha256(path) for path in unique_sources}
        if len(hashes) != 1:
            raise BuildError(f"mesh URI resolves to different bytes across input assets: {uri}; matches={list(map(str, unique_sources))}")
        source = sorted(unique_sources, key=str)[0]
        source_root = next(root_path for path, root_path in matches if path == source)
        relative = source.relative_to(source_root)
        target_rel = relative.as_posix()
        target = output_stage / target_rel
        target.parent.mkdir(parents=True, exist_ok=True)
        digest = sha256(source)
        previous = records.get(target_rel)
        if previous and previous["sha256"] != digest:
            raise BuildError(f"mesh closure path collision with different contents: {target_rel}")
        if not target.exists():
            shutil.copy2(source, target)
        records[target_rel] = {"output_path": target_rel, "source_path": str(source), "sha256": digest}
        node.set(key, target_rel)
    return [records[key] for key in sorted(records)]


def build(old_assembly: Path, official_root: Path, output: Path, side: str = "right",
          old_source_root: Path | None = None) -> dict[str, Any]:
    global HAND_ROOT, MOUNT_JOINT
    if side not in {"left", "right"}:
        raise BuildError("side must be left or right")
    HAND_ROOT = f"{side}_hand_base_link"
    MOUNT_JOINT = f"{side}_hand_base_joint"
    old_assembly = old_assembly.expanduser().resolve(strict=True)
    old_source_root = (old_source_root or old_assembly.parent).expanduser().resolve(strict=True)
    official_root = official_root.expanduser().resolve(strict=True)
    output = output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite output directory: {output}")
    official_urdf = official_root / "urdf" / f"revo3_{side}.urdf"
    if not official_urdf.is_file():
        raise BuildError(f"official root must contain urdf/revo3_{side}.urdf: {official_root}")

    _, old_root = _robot(old_assembly)
    _, official_root_xml = _robot(official_urdf)
    old_links, old_joints, _ = _maps(old_root)
    official_links, official_joints, _ = _maps(official_root_xml)
    old_hand, _ = _hand_descendants(old_root)
    official_hand, official_parent_joint = _hand_descendants(official_root_xml)
    official_geometry_counts = {
        "visuals": sum(len(official_links[name].findall("visual")) for name in official_hand),
        "collisions": sum(len(official_links[name].findall("collision")) for name in official_hand),
    }

    mount = old_joints.get(MOUNT_JOINT)
    if mount is None or mount.find("child").get("link") != HAND_ROOT:
        raise BuildError("source assembly must have right_hand_base_joint mounting right_hand_base_link")
    official_mount = official_joints.get(MOUNT_JOINT)
    if official_mount is None or official_mount.find("parent").get("link") not in {"world", "base_link"}:
        raise BuildError("official URDF must provide its root attachment joint")
    if official_mount.find("child").get("link") != HAND_ROOT:
        raise BuildError("official root attachment must target right_hand_base_link")
    old_mount_signature = xml_signature(mount)

    old_hand_active = {name for name, joint in old_joints.items()
                       if joint.get("type") in ACTIVE_TYPES and joint.find("child").get("link") in old_hand}
    official_hand_active = {name for name, joint in official_joints.items()
                            if joint.get("type") in ACTIVE_TYPES and joint.find("child").get("link") in official_hand}
    if len(old_hand_active) != 21 or old_hand_active != official_hand_active:
        raise BuildError(f"hand active-joint name sets must match and contain 21 joints: old={len(old_hand_active)}, official={len(official_hand_active)}")
    old_active_order = _joint_order(old_root, old_hand_active)
    if len(old_active_order) != 21 or set(old_active_order) != old_hand_active:
        raise BuildError("source hand active-joint order is incomplete or duplicated")
    official_active_by_name = {name: joint for name, joint in official_joints.items() if name in official_hand_active}

    # All official inertia-bearing hand links must be precisely base + the 21
    # active-joint child bodies; their inertial XML is copied without edits.
    official_mass_links = {name for name in official_hand
                           if official_links[name].find("inertial") is not None}
    official_child_links = {official_joints[name].find("child").get("link") for name in official_hand_active}
    expected_mass_links = official_child_links | {HAND_ROOT}
    if len(official_mass_links) != 22 or official_mass_links != expected_mass_links:
        raise BuildError("official hand must contain exactly 22 inertial links: base plus 21 active child bodies")

    aliases = {name: _alias_link(name, set(old_links)) for name in official_hand
               if name in official_links and _alias_link(name, set(old_links)) != name}
    alias_inverse: dict[str, str] = {}
    for source, target in aliases.items():
        if target in alias_inverse:
            raise BuildError(f"link alias collision at {target}")
        alias_inverse[target] = source

    virtual_frames: dict[str, dict[str, Any]] = {}
    tip_links: set[str] = set()
    for name in official_hand:
        if not name.endswith(TIP_SUFFIX):
            continue
        tip = official_links[name]
        joint = official_parent_joint.get(name)
        if joint is None or joint.get("type") != "fixed":
            raise BuildError(f"tip frame {name} must be a fixed child")
        if tip.find("inertial") is not None or tip.findall("visual") or tip.findall("collision"):
            raise BuildError(f"tip frame {name} is not empty; refusing to drop geometry or inertia")
        if any(j.find("parent").get("link") == name for j in official_root_xml.findall("joint")):
            raise BuildError(f"tip frame {name} has child joints and cannot be virtualized safely")
        xyz, rpy = _pose(joint.find("origin"))
        parent = joint.find("parent").get("link", "")
        transform = _matrix(xyz, rpy)
        virtual_frames[name] = {
            "parent_link": _alias_link(parent, set(old_links)),
            "xyz": xyz,
            "quat_wxyz": _quaternion_wxyz_from_matrix(transform),
            "transform_matrix": transform,
        }
        tip_links.add(name)
    if len(virtual_frames) != 5:
        raise BuildError(f"expected five empty fixed fingertip frames, found {len(virtual_frames)}")

    # Move visual/collision geometry from fixed links without official inertia
    # to the closest official inertial ancestor using the composed fixed chain.
    fixed_merges: list[dict[str, Any]] = []
    # Keep the five empty fixed tip links as real URDF aliases. Their parents
    # and exact transforms are copied from the official model below.
    removals: set[str] = set()
    for name in sorted(official_hand - official_mass_links - tip_links):
        link = official_links[name]
        if any(j.get("type") in ACTIVE_TYPES and j.find("child").get("link") == name
               for j in official_root_xml.findall("joint")):
            raise BuildError(f"non-inertial hand link {name} is a movable-joint child")
        cursor = name
        chain: list[str] = []
        while cursor not in official_mass_links:
            joint = official_parent_joint.get(cursor)
            if joint is None or joint.get("type") != "fixed":
                raise BuildError(f"cannot find fixed inertial ancestor for official link {name}")
            chain.append(cursor)
            cursor = joint.find("parent").get("link", "")
        physical_parent = cursor
        transform, joint_chain = _fixed_chain_transform(physical_parent, name, official_parent_joint)
        moved = {tag: 0 for tag in ("visual", "collision")}
        for tag in moved:
            for geometry in list(link.findall(tag)):
                original_xyz, original_rpy = _pose(geometry.find("origin"))
                geom_pose = _multiply(transform, _matrix(original_xyz, original_rpy))
                _set_origin(geometry, geom_pose)
                official_links[physical_parent].append(geometry)
                link.remove(geometry)
                moved[tag] += 1
        fixed_merges.append({
            "source_link": name,
            "physical_parent_link": _alias_link(physical_parent, set(old_links)),
            "fixed_joint_chain": joint_chain,
            "source_to_parent_transform_matrix": [list(row) for row in transform],
            "visual_count": moved["visual"],
            "collision_count": moved["collision"],
            "inertial_added": False,
        })
        removals.add(name)

    # Refuse to delete any non-inertial geometry that was not handled.
    for name in removals:
        link = official_links[name]
        if link.findall("visual") or link.findall("collision") or link.find("inertial") is not None:
            raise BuildError(f"removed fixed frame {name} still carries geometry or inertial data")

    # Build an assembly copy: all old non-hand XML is preserved; only links
    # and joints in the old right-hand subtree are replaced.
    result_root = copy.deepcopy(old_root)
    result_links, result_joints, _ = _maps(result_root)
    result_subtree, _ = _hand_descendants(result_root)
    for link in list(result_root.findall("link")):
        name = link.get("name", "")
        if name == HAND_ROOT:
            continue
        if name in result_subtree:
            result_root.remove(link)
    old_joint_index = {joint.get("name"): idx for idx, joint in enumerate(list(result_root.findall("joint")))}
    for joint in list(result_root.findall("joint")):
        name = joint.get("name", "")
        child = joint.find("child").get("link", "")
        if child in result_subtree and name != MOUNT_JOINT and name not in old_hand_active:
            result_root.remove(joint)

    # Replace root body and append every remaining official hand body.
    official_link_copies: dict[str, ET.Element] = {}
    for name in official_hand - removals:
        copied = copy.deepcopy(official_links[name])
        new_name = _alias_link(name, set(old_links))
        copied.set("name", new_name)
        official_link_copies[name] = copied
    result_root.remove(result_links[HAND_ROOT])
    root_copy = official_link_copies.pop(HAND_ROOT)
    current_links = {link.get("name") for link in result_root.findall("link")}
    if root_copy.get("name") in current_links:
        raise BuildError("replacement official root name conflicts with preserved non-hand link")
    # Preserve the original root link's XML position as closely as possible.
    root_old_index = list(old_root).index(old_links[HAND_ROOT])
    result_root.insert(min(root_old_index, len(result_root)), root_copy)
    for name in sorted(official_link_copies):
        copied = official_link_copies[name]
        if copied.get("name") in {link.get("name") for link in result_root.findall("link")}:
            raise BuildError(f"official replacement link name conflicts with preserved assembly link: {copied.get('name')}")
        result_root.append(copied)

    # Preserve joint XML order by replacing each old active joint in place.
    for name in old_active_order:
        new_joint = copy.deepcopy(official_active_by_name[name])
        parent = new_joint.find("parent")
        child = new_joint.find("child")
        parent.set("link", _alias_link(parent.get("link", ""), set(old_links)))
        child.set("link", _alias_link(child.get("link", ""), set(old_links)))
        # The official joint names and all kinematic/limit values are retained.
        original_idx = old_joint_index.get(name)
        if original_idx is None:
            raise BuildError(f"source joint index missing for {name}")
        # Locate the original slot by name in the current tree (other removals shift positions).
        existing = next((j for j in result_root.findall("joint") if j.get("name") == name), None)
        if existing is None:
            raise BuildError(f"source hand joint slot disappeared before replacement: {name}")
        idx = list(result_root).index(existing)
        result_root.remove(existing)
        result_root.insert(idx, new_joint)

    for tip_name in sorted(tip_links):
        source_joint = official_parent_joint[tip_name]
        alias_joint = copy.deepcopy(source_joint)
        alias_joint.find("parent").set("link", _alias_link(alias_joint.find("parent").get("link", ""), set(old_links)))
        alias_joint.find("child").set("link", _alias_link(alias_joint.find("child").get("link", ""), set(old_links)))
        if any(j.get("name") == alias_joint.get("name") for j in result_root.findall("joint")):
            raise BuildError(f"virtual fingertip joint name conflicts with retained assembly joint: {alias_joint.get('name')}")
        result_root.append(alias_joint)

    # Preserve the old incoming mounting joint byte-for-byte in XML semantics.
    if xml_signature(next(j for j in result_root.findall("joint") if j.get("name") == MOUNT_JOINT)) != old_mount_signature:
        raise BuildError("right_hand_base_joint mounting transform changed")

    # Add official global material definitions needed by copied hand geometry.
    material_names = {m.get("name") for m in result_root.findall("material")}
    for material in official_root_xml.findall("material"):
        name = material.get("name")
        if name in material_names:
            existing = next(m for m in result_root.findall("material") if m.get("name") == name)
            if xml_signature(existing) != xml_signature(material):
                raise BuildError(f"official material conflicts with preserved material {name}")
        else:
            result_root.append(copy.deepcopy(material))
            material_names.add(name)

    # Validate active hand order and preserved arm joints before materializing.
    result_links_final, result_joints_final, _ = _maps(result_root)
    output_hand, _ = _hand_descendants(result_root)
    output_hand_active_order = _joint_order(result_root, old_hand_active)
    if output_hand_active_order != old_active_order:
        raise BuildError(f"replacement changed hand active-joint order: {output_hand_active_order}")
    arm_joint_names = {name for name, joint in old_joints.items()
                       if joint.get("type") in ACTIVE_TYPES and joint.find("child").get("link") not in old_hand}
    output_arm_names = {name for name, joint in result_joints_final.items()
                        if joint.get("type") in ACTIVE_TYPES and joint.find("child").get("link") not in output_hand}
    if arm_joint_names != output_arm_names:
        raise BuildError("preserved non-hand active arm-joint set changed")
    for name in arm_joint_names:
        if xml_signature(old_joints[name]) != xml_signature(result_joints_final[name]):
            raise BuildError(f"non-hand active joint changed: {name}")
    for name, link in old_links.items():
        if name not in old_hand and xml_signature(link) != xml_signature(result_links_final[name]):
            raise BuildError(f"non-hand link changed: {name}")
    for name, joint in old_joints.items():
        if joint.find("child").get("link") not in old_hand and name != MOUNT_JOINT:
            if xml_signature(joint) != xml_signature(result_joints_final[name]):
                raise BuildError(f"non-hand or retained fixed joint changed: {name}")
    # The full mount joint, parent, xyz, and rpy remain exactly as authored.
    mount_out = result_joints_final[MOUNT_JOINT]
    if xml_signature(mount_out) != old_mount_signature:
        raise BuildError("incoming physical hand mounting joint changed")

    final_mass_links = {name for name in output_hand if result_links_final[name].find("inertial") is not None}
    expected_output_mass_links = {_alias_link(name, set(old_links)) for name in official_mass_links}
    if len(final_mass_links) != 22 or final_mass_links != expected_output_mass_links:
        raise BuildError("output must contain exactly the 22 official inertial hand links")
    for official_name in official_mass_links:
        output_name = _alias_link(official_name, set(old_links))
        if _inertia_signature(official_links[official_name]) != _inertia_signature(result_links_final[output_name]):
            raise BuildError(f"official inertia changed during assembly for {official_name}")
    output_geometry_counts = {
        "visuals": sum(len(result_links_final[name].findall("visual")) for name in output_hand),
        "collisions": sum(len(result_links_final[name].findall("collision")) for name in output_hand),
    }
    if output_geometry_counts != official_geometry_counts:
        raise BuildError(f"official hand geometry was lost or duplicated: expected {official_geometry_counts}, got {output_geometry_counts}")

    output_urdf_name = "assembly_official_revo3.urdf"
    parent = output.parent
    parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=f".{output.name}.building-", dir=parent))
    try:
        official_source_root = official_root.resolve(strict=True)
        mesh_records = _copy_mesh_closure(
            result_root,
            stage,
            {old_assembly: old_source_root, official_urdf: official_source_root},
        )
        output_urdf = stage / output_urdf_name
        ET.indent(ET.ElementTree(result_root), space="  ")
        ET.ElementTree(result_root).write(output_urdf, encoding="utf-8", xml_declaration=True)

        try:
            official_commit = subprocess.check_output(
                ["git", "-C", str(official_root), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
            ).strip()
        except (OSError, subprocess.CalledProcessError) as exc:
            raise BuildError(f"cannot identify official source commit from {official_root}: {exc}") from exc

        manifest = {
            "schema": "official_revo3_assembly_build/v1",
            "sources": {
                "old_assembly_path": str(old_assembly),
                "old_source_root": str(old_source_root),
                "old_assembly_sha256": sha256(old_assembly),
                "official_root": str(official_root),
                "official_urdf_path": str(official_urdf),
                "official_urdf_sha256": sha256(official_urdf),
                "official_git_commit": official_commit,
            },
            "output": {"urdf": output_urdf_name, "urdf_sha256": sha256(output_urdf)},
            "joint_contract": {
                f"{side}_hand_active_joint_count": 21,
                f"{side}_hand_active_joint_order": output_hand_active_order,
                "preserved_arm_active_joint_names": [name for name in _joint_order(old_root, arm_joint_names)],
                "incoming_mount_joint": {
                    "name": MOUNT_JOINT,
                    "parent": mount_out.find("parent").get("link"),
                    "child": mount_out.find("child").get("link"),
                    "origin_xyz_rpy": {
                        "xyz": mount_out.find("origin").get("xyz"),
                        "rpy": mount_out.find("origin").get("rpy"),
                    },
                    "preserved_exactly": True,
                },
            },
            "official_link_aliases": aliases,
            "official_inertial_links_preserved": [
                {"official_link": name, "output_link": _alias_link(name, set(old_links)),
                 "inertial_xml_sha256": hashlib.sha256(ET.tostring(official_links[name].find("inertial"), encoding="utf-8")).hexdigest()}
                for name in sorted(official_mass_links)
            ],
            "virtual_frames": virtual_frames,
            "fixed_no_inertia_geometry_merges": fixed_merges,
            "mesh_closure": mesh_records,
            "geometry_count": {
                "official_hand_visuals": official_geometry_counts["visuals"],
                "output_hand_visuals": output_geometry_counts["visuals"],
                "official_hand_collisions": official_geometry_counts["collisions"],
                "output_hand_collisions": output_geometry_counts["collisions"],
                "preserved": True,
            },
            "fixed_merge_policy": "Fixed descendants with no official inertial were merged into the nearest official inertial ancestor using composed URDF fixed-joint transforms; no mass or inertia was synthesized.",
            "training_boundary": "Asset construction only. No checkpoint, trajectory, installed model, mount height, arm joint, reward, or training process was modified.",
        }
        manifest_path = stage / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        frames_path = stage / "revo3_virtual_tip_frames.json"
        frames_path.write_text(json.dumps({"schema": "revo3_virtual_tip_frames/v1", "virtual_frames": virtual_frames}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        # Atomic-ish exclusive directory materialization: the requested path is
        # created only after every source and output check has succeeded.
        output.mkdir(parents=True, exist_ok=False)
        for child in stage.iterdir():
            os.replace(child, output / child.name)
    except Exception:
        shutil.rmtree(stage, ignore_errors=True)
        raise
    else:
        stage.rmdir()
    manifest["output"]["directory"] = str(output)
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--old-assembly", required=True, type=Path, help="source combined TRON2 + Revo3 assembly URDF")
    parser.add_argument("--official-root", required=True, type=Path, help="official description root containing urdf/revo3_right.urdf and meshes/")
    parser.add_argument("--output", required=True, type=Path, help="new output directory; must not already exist")
    parser.add_argument("--side", choices=("left", "right"), default="right", help="hand subtree to replace; source mount is preserved")
    parser.add_argument("--old-source-root", type=Path, help="declared asset root containing source meshes; defaults to the URDF directory")
    args = parser.parse_args(argv)
    try:
        result = build(args.old_assembly, args.official_root, args.output, args.side, args.old_source_root)
    except (OSError, BuildError, ET.ParseError, ValueError, KeyError) as exc:
        print(f"official Revo3 assembly build failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
