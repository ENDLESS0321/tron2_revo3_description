#!/usr/bin/env python3
"""Build an independent revision-2 assembly from the frozen v1 URDF.

Only the left/right adapter links' visual geometry, collision geometry and
inertial data change. All joints and all non-adapter links are preserved.
CoACD meshes are simulator contact approximations, never manufacturing meshes.
The source 3MF, v1 configuration, URDFs, meshes, scenes and reports are read-only.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DESIGN = ROOT / "design/revision2/design_manifest.json"
V1_URDF = ROOT / "urdf/tron2_dach_revo3.urdf"
V1_CONFIG = ROOT / "config/assembly.json"
OUTPUT_URDF = ROOT / "urdf/tron2_dach_revo3_v2.urdf"
OUTPUT_CONFIG = ROOT / "config/assembly_v2.json"
REPORTS = ROOT / "reports/revision2"
SCENE = ROOT / "simulation/revision2/scene.xml"
COACD_PARAMETERS = {
    "threshold": 0.035, "max_convex_hull": 48, "preprocess_mode": "auto",
    "resolution": 1500, "mcts_nodes": 15, "mcts_iterations": 80,
    "mcts_max_depth": 3, "merge": True, "seed": 42,
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative(path):
    path = Path(path).resolve()
    return path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)


def numbers(values):
    return " ".join(format(float(value), ".15g") for value in values)


def signature(node):
    """XML meaning without serialization whitespace."""
    return [node.tag, sorted(node.attrib.items()), (node.text or "").strip(), [signature(child) for child in node]]


def snapshot_v1(source):
    paths = {Path(source), V1_URDF, V1_CONFIG, ROOT / "config/joint_map.json"}
    paths.update((ROOT / "urdf").glob("tron2_dach_revo3.ros.urdf"))
    paths.update(path for path in (ROOT / "meshes/adapter").rglob("*") if path.is_file())
    for directory in (ROOT / "reports", ROOT / "simulation"):
        paths.update(path for path in directory.rglob("*") if path.is_file() and path.relative_to(directory).parts[0] != "revision2")
    return {str(path.resolve()): sha(path) for path in sorted(paths)}


def verify_snapshot(before):
    changed = [path for path, expected in before.items() if not Path(path).is_file() or sha(path) != expected]
    if changed:
        raise RuntimeError(f"v1 inputs/outputs changed during revision-2 build: {changed}")
    return {relative(path): expected for path, expected in before.items()}


def build_collision(side, source_text, source_sha, cpu_ids, rebuild, producer_sha):
    # Limit only this worker; do not alter other users' processes or affinity.
    if cpu_ids and hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, cpu_ids)
    os.environ["OMP_NUM_THREADS"] = str(max(1, len(cpu_ids)))
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    import coacd
    import numpy as np
    import trimesh

    source = Path(source_text)
    folder = ROOT / "meshes/adapter_v2" / side
    visual = folder / f"adapter_{side}_v2_visual.stl"
    collision_folder = folder / "collision"
    report_path = folder / "collision_manifest.json"
    if sha(source) != source_sha:
        raise ValueError(f"{side} meter mesh differs from design manifest")
    folder.mkdir(parents=True, exist_ok=True)
    if not visual.exists() or sha(visual) != source_sha:
        shutil.copy2(source, visual)
    mesh = trimesh.load_mesh(visual, process=False)
    mesh.merge_vertices(digits_vertex=12)
    if not mesh.is_volume or mesh.body_count != 1:
        raise ValueError(f"{side} visual is not a single closed positive volume")

    old = json.loads(report_path.read_text()) if report_path.exists() and not rebuild else None
    cache_valid = old is not None and old.get("source_sha256") == source_sha and old.get("parameters") == COACD_PARAMETERS and old.get("producer_sha256") == producer_sha
    if cache_valid:
        for record in old["pieces"]:
            path = ROOT / record["path"]
            if not path.is_file() or sha(path) != record["sha256"]:
                cache_valid = False
                break
    start = time.monotonic()
    if cache_valid:
        candidates = [trimesh.load_mesh(ROOT / row["path"], process=False) for row in old["pieces"]]
        for candidate in candidates:
            candidate.merge_vertices(digits_vertex=12)
    else:
        print(f"[{side}] Starting CoACD with at most 48 pieces on CPU IDs {cpu_ids}", flush=True)
        coacd.set_log_level("warn")
        raw = coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)), **COACD_PARAMETERS)
        candidates = [trimesh.Trimesh(vertices=vertices, faces=faces, process=True).convex_hull for vertices, faces in raw]
    if not 1 <= len(candidates) <= 48:
        raise ValueError(f"{side} collision decomposition produced {len(candidates)} pieces")
    collision_folder.mkdir(parents=True, exist_ok=True)
    probes = np.asarray([[0., y, 0.] for y in (.015, .020, .025, .030, .034)])
    occupied = np.zeros(len(probes), dtype=bool)
    pieces = []
    for index, proxy in enumerate(candidates):
        if not proxy.is_volume or not proxy.is_convex:
            raise ValueError(f"{side} convex piece {index} is invalid")
        path = collision_folder / f"adapter_{side}_v2_convex_{index:03d}.stl"
        if not cache_valid:
            proxy.export(path)
        reread = trimesh.load_mesh(path, process=False)
        reread.merge_vertices(digits_vertex=12)
        if not reread.is_volume or not reread.is_convex:
            raise ValueError(f"{side} serialized convex piece {index} is invalid")
        occupied |= reread.contains(probes)
        pieces.append({"path": relative(path), "sha256": sha(path), "vertices": len(reread.vertices), "faces": len(reread.faces), "volume_m3": float(reread.volume), "watertight": bool(reread.is_watertight), "convex": bool(reread.is_convex)})
    report = {
        "schema": "adapter_revision2_collision_v1", "side": side,
        "source": relative(source), "source_sha256": source_sha,
        "visual": relative(visual), "visual_sha256": sha(visual),
        "producer_sha256": producer_sha, "algorithm": "CoACD",
        "coacd_version": importlib.metadata.version("coacd"), "parameters": COACD_PARAMETERS,
        "worker_cpu_ids": cpu_ids, "cached_pieces_reused": cache_valid,
        "elapsed_seconds": time.monotonic()-start, "pieces": pieces,
        "visual_volume_m3": float(mesh.volume), "sum_convex_volume_m3": sum(row["volume_m3"] for row in pieces),
        "hull_cap_reached": len(pieces) == 48, "requested_concavity_threshold_certified": False,
        "cavity_axis_probes": [{"xyz_m": point.tolist(), "inside_any_proxy": bool(inside)} for point, inside in zip(probes, occupied)],
        "cavity_axis_probes_pass": not bool(occupied.any()),
        "claim_boundary": "Simulator contact approximation for assembly preview. A 48-hull cap may prevent the requested concavity target. Five sleeve-axis probes do not certify the complete cavity, cable slot, screw holes, keyway, or physical contact success.",
    }
    report_path.write_text(json.dumps(report, indent=2)+"\n")
    if occupied.any():
        raise ValueError(f"{side} collision approximation blocks sampled sleeve axis")
    print(f"[{side}] {len(pieces)} valid convex pieces; sampled sleeve axis open", flush=True)
    return report


def read_design():
    import numpy as np
    design = json.loads(DESIGN.read_text())
    if design.get("schema") != "tron2_revo3_adapter_revision2_v1":
        raise ValueError("Unexpected revision-2 design schema")
    if sha(Path(design["source"])) != design["source_sha256"]:
        raise ValueError("Original source 3MF hash differs from design manifest")
    for side in ("left", "right"):
        data = design["sides"][side]
        path = DESIGN.parent / f"adapter_{side}_v2_m.stl"
        if sha(path) != data["files"][path.name]:
            raise ValueError(f"{side} meter mesh is missing or stale")
        mass = float(data["mass_kg_assumed_solid_PLA"])
        center = np.asarray(data["center_mass_m"], dtype=float)
        inertia = np.asarray(data["inertia_kg_m2"], dtype=float)
        if not mass > 0 or center.shape != (3,) or inertia.shape != (3,3) or not np.isfinite(center).all() or not np.isfinite(inertia).all():
            raise ValueError(f"{side} invalid mass, COM or inertia")
        eigenvalues = np.linalg.eigvalsh(inertia)
        if min(eigenvalues) <= 0 or max(eigenvalues) > sum(eigenvalues)/2 + 1e-12:
            raise ValueError(f"{side} inertia is not physically admissible")
    return design


def assemble(design, collision, initial_v1, design_sha):
    source = ET.parse(V1_URDF).getroot()
    robot = copy.deepcopy(source)
    robot.set("name", "tron2_dach_revo3_v2")
    robot.insert(0, ET.Comment("Generated by scripts/build_revision2_urdf.py from the frozen v1 assembly. Only adapter visual, collision and inertial data are replaced. Simulation geometry is not calibrated hardware."))
    for side in ("left", "right"):
        data = design["sides"][side]
        link = robot.find(f"link[@name='{side}_adapter_link']")
        if link is None:
            raise ValueError(f"Missing v1 {side} adapter link")
        visuals = link.findall("visual")
        if len(visuals) != 1 or visuals[0].find("geometry/mesh") is None:
            raise ValueError(f"Expected one {side} adapter visual mesh")
        visuals[0].find("geometry/mesh").set("filename", "../"+collision[side]["visual"])
        for node in list(link.findall("collision")) + list(link.findall("inertial")):
            link.remove(node)
        inertial = ET.Element("inertial")
        ET.SubElement(inertial, "origin", xyz=numbers(data["center_mass_m"]), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=format(data["mass_kg_assumed_solid_PLA"], ".15g"))
        matrix = data["inertia_kg_m2"]
        ET.SubElement(inertial, "inertia", **{key: format(matrix[i][j], ".15g") for key, i, j in (("ixx",0,0),("ixy",0,1),("ixz",0,2),("iyy",1,1),("iyz",1,2),("izz",2,2))})
        link.insert(0, inertial)
        for row in collision[side]["pieces"]:
            element = ET.SubElement(link, "collision", name=Path(row["path"]).stem)
            ET.SubElement(ET.SubElement(element, "geometry"), "mesh", filename="../"+row["path"])
    expected_joints = {joint.get("name"): signature(joint) for joint in source.findall("joint")}
    actual_joints = {joint.get("name"): signature(joint) for joint in robot.findall("joint")}
    if expected_joints != actual_joints:
        raise ValueError("A v1 joint or mount transform changed")
    unchanged_links = 0
    for link in source.findall("link"):
        if link.get("name") in ("left_adapter_link", "right_adapter_link"):
            continue
        if signature(link) != signature(robot.find(f"link[@name='{link.get('name')}']")):
            raise ValueError(f"Non-adapter link changed: {link.get('name')}")
        unchanged_links += 1
    paths = {}
    for node in robot.findall(".//mesh"):
        filename = node.get("filename")
        if not filename or Path(filename).is_absolute() or "://" in filename:
            raise ValueError(f"Expected relative mesh reference: {filename}")
        path = (OUTPUT_URDF.parent / filename).resolve(strict=True)
        paths[path] = sha(path)
    stems = {}
    for path in paths:
        if path.stem in stems and path != stems[path.stem]:
            raise ValueError(f"Ambiguous mesh basename: {stems[path.stem]} and {path}")
        stems[path.stem] = path
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    config = json.loads(V1_CONFIG.read_text())
    config["robot_name"] = robot.get("name")
    config["revision"] = 2
    config["urdf"] = relative(OUTPUT_URDF)
    config["source_v1_urdf"] = relative(V1_URDF)
    config["source_v1_urdf_sha256"] = initial_v1[str(V1_URDF.resolve())]
    config["design_manifest"] = relative(DESIGN)
    config["design_manifest_sha256"] = design_sha
    config["adapter_assets"] = {side: {"link": side+"_adapter_link", "visual": collision[side]["visual"], "collision_manifest": relative(ROOT/"meshes/adapter_v2"/side/"collision_manifest.json"), "mass_kg_assumed_solid_PLA": design["sides"][side]["mass_kg_assumed_solid_PLA"], "center_mass_m": design["sides"][side]["center_mass_m"], "inertia_kg_m2": design["sides"][side]["inertia_kg_m2"], "slot_adapter_z_sign": design["sides"][side]["back_slot_adapter_z_sign"]} for side in ("left", "right")}
    config["revision2_notes"] = ["All v1 joints, wrist-to-adapter and adapter-to-hand transforms are preserved.", "Only the insertion crown is re-indexed by theta+15 degrees and lifted 0.045 mm; exposed insertion depth is shortened by that amount.", "Left and right dorsal cable slots differ. The adapter link reference frame is unchanged.", "Inertial data are the design manifest's homogeneous solid-PLA assumptions, not measurements.", "Convex collision pieces are capped approximations; detailed visual/STL geometry remains separate."]
    OUTPUT_CONFIG.write_text(json.dumps(config, indent=2, ensure_ascii=False)+"\n")
    return {"joints_preserved": len(expected_joints), "non_adapter_links_preserved": unchanged_links, "adapter_links_replaced": ["left_adapter_link", "right_adapter_link"], "counts": {"links": len(robot.findall("link")), "joints": len(robot.findall("joint")), "actuated": sum(j.get("type") != "fixed" for j in robot.findall("joint")), "unique_mesh_files": len(paths)}, "mesh_sources": {relative(path): value for path, value in sorted(paths.items())}}


def execute_checks(skip_preview):
    commands = [[sys.executable, str(ROOT/"scripts/validate_description.py"), "--urdf", str(OUTPUT_URDF), "--joint-map", str(ROOT/"config/joint_map.json"), "--output", str(REPORTS/"urdf_validation.json")]]
    if not skip_preview:
        commands.append([sys.executable, str(ROOT/"scripts/preview_assembly.py"), "--urdf", str(OUTPUT_URDF), "--scene", str(SCENE), "--report-dir", str(REPORTS)])
    ledger = []
    for command in commands:
        name = Path(command[1]).stem
        result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
        log = REPORTS / f"{name}.log"
        log.write_text(result.stdout+"\n"+result.stderr)
        ledger.append({"argv": command, "exit_code": result.returncode, "log": relative(log), "log_sha256": sha(log), "script_sha256": sha(Path(command[1]))})
        print(result.stdout, end="", flush=True)
        if result.returncode:
            raise RuntimeError(f"{name} failed; see {log}")
    return ledger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, choices=(1,2), default=2)
    parser.add_argument("--threads-per-worker", type=int, default=2)
    parser.add_argument("--rebuild-collision", action="store_true")
    parser.add_argument("--skip-preview", action="store_true", help="Run static URDF validation only; keep MuJoCo reports separate")
    args = parser.parse_args()
    if args.threads_per_worker < 1:
        raise ValueError("threads-per-worker must be positive")
    REPORTS.mkdir(parents=True, exist_ok=True)
    report_path = REPORTS / "assembly_manifest.json"
    report = {"schema": "tron2_revo3_assembly_manifest_v2", "created_utc": datetime.now(timezone.utc).isoformat(), "status": "building", "builder": relative(__file__), "builder_sha256": sha(__file__)}
    before = None
    try:
        design = read_design()
        design_sha = sha(DESIGN)
        before = snapshot_v1(design["source"])
        report.update({"source_v1_urdf": relative(V1_URDF), "source_v1_urdf_sha256": sha(V1_URDF), "source_v1_config": relative(V1_CONFIG), "source_v1_config_sha256": sha(V1_CONFIG), "design_manifest": relative(DESIGN), "design_manifest_sha256": design_sha, "design_producer_sha256": design["producer_sha256"], "original_3mf_sha256": design["source_sha256"], "validation_boundary": "Kinematic revision-2 assembly only; not calibrated fit, full collision clearance, dynamic stability or hardware readiness."})
        cpu_ids = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else list(range(os.cpu_count() or 1))
        collision = {}
        with ProcessPoolExecutor(max_workers=args.workers, mp_context=multiprocessing.get_context("spawn")) as pool:
            futures = {}
            for index, side in enumerate(("left", "right")):
                selected_cpus = [cpu_ids[(index*args.threads_per_worker+k) % len(cpu_ids)] for k in range(min(args.threads_per_worker, len(cpu_ids)))]
                source = DESIGN.parent / f"adapter_{side}_v2_m.stl"
                task = pool.submit(build_collision, side, str(source), design["sides"][side]["files"][source.name], selected_cpus, args.rebuild_collision, report["builder_sha256"])
                futures[task] = side
            for future in as_completed(futures):
                side = futures[future]
                collision[side] = future.result()
        if sha(DESIGN) != design_sha:
            raise ValueError("Design manifest changed during convex decomposition")
        report.update(assemble(design, collision, before, design_sha))
        report.update({"urdf": relative(OUTPUT_URDF), "urdf_sha256": sha(OUTPUT_URDF), "config": relative(OUTPUT_CONFIG), "config_sha256": sha(OUTPUT_CONFIG), "collision": {side: {"manifest": relative(ROOT/"meshes/adapter_v2"/side/"collision_manifest.json"), "manifest_sha256": sha(ROOT/"meshes/adapter_v2"/side/"collision_manifest.json"), "piece_count": len(collision[side]["pieces"]), "cavity_axis_probes_pass": collision[side]["cavity_axis_probes_pass"], "hull_cap_reached": collision[side]["hull_cap_reached"], "claim_boundary": collision[side]["claim_boundary"]} for side in ("left", "right")}, "status": "built_pending_validation"})
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
        report["validation_commands"] = execute_checks(args.skip_preview)
        report["v1_hashes_unchanged"] = verify_snapshot(before)
        report["v1_preservation_pass"] = True
        report["status"] = "static_assembly_pass" if args.skip_preview else "kinematic_assembly_pass"
        derived = [REPORTS/"urdf_validation.json"]
        if not args.skip_preview:
            derived.extend([SCENE, REPORTS/"mujoco_validation.json", REPORTS/"assembly_overview.png", REPORTS/"mount_left.png", REPORTS/"mount_right.png"])
        report["derived_artifact_hashes"] = {relative(path): sha(path) for path in derived}
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
        print(json.dumps({"status": report["status"], "urdf": report["urdf"], "counts": report["counts"], "collision_pieces": {side: report["collision"][side]["piece_count"] for side in ("left", "right")}, "v1_preservation_pass": True, "manifest": relative(report_path)}, indent=2), flush=True)
        return 0
    except Exception as exc:
        report.update({"status": "failed", "error": str(exc), "traceback": traceback.format_exc()})
        if before is not None:
            try:
                report["v1_hashes_unchanged"] = verify_snapshot(before)
                report["v1_preservation_pass"] = True
            except Exception as preserve_exc:
                report["v1_preservation_pass"] = False
                report["v1_preservation_error"] = str(preserve_exc)
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
        print(json.dumps({"status": "failed", "error": str(exc), "manifest": relative(report_path)}, indent=2), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
