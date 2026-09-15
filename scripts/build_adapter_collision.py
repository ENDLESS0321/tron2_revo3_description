#!/usr/bin/env python3
"""Build convex adapter contact proxies; keep the detailed cut visual separate."""
import argparse
import hashlib
import json
from pathlib import Path

import coacd
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-only", action="store_true", help="Audit existing hash-matched proxies without rerunning decomposition")
    args = parser.parse_args()
    source = ROOT / "meshes/adapter/adapter_visual.stl"
    mesh = trimesh.load_mesh(source, process=False)
    mesh.merge_vertices(digits_vertex=12)
    if not mesh.is_volume:
        raise ValueError("Adapter is not a valid closed volume")
    # CoACD operates on the final, cut geometry. It produces simulator convex
    # contact approximations, not a replacement for the visual manufacturing mesh.
    manifest_path = ROOT / "meshes/adapter/collision_manifest.json"
    if args.audit_only:
        existing = json.loads(manifest_path.read_text())
        if existing["source_sha256"] != hashlib.sha256(source.read_bytes()).hexdigest():
            raise ValueError("Existing collision proxies are stale")
        result = []
        for item in existing["pieces"]:
            p = ROOT / item["path"]
            if hashlib.sha256(p.read_bytes()).hexdigest() != item["sha256"]:
                raise ValueError("Existing collision proxy hash mismatch")
            proxy = trimesh.load_mesh(p)
            result.append((proxy.vertices, proxy.faces))
    else:
        coacd.set_log_level("warn")
        result = coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)),
                                threshold=0.035, max_convex_hull=48, preprocess_mode="auto",
                                resolution=1500, mcts_nodes=15, mcts_iterations=80,
                                mcts_max_depth=3, merge=True, seed=42)
    output = ROOT / "meshes/adapter/collision"
    output.mkdir(exist_ok=True)
    pieces = []
    probes = np.array([[0, y, 0] for y in [.015, .020, .025, .030, .034]])
    cavity_blocked = np.zeros(len(probes), dtype=bool)
    for index, (vertices, faces) in enumerate(result):
        proxy = trimesh.Trimesh(vertices=vertices, faces=faces, process=True).convex_hull
        path = output / f"adapter_convex_{index:03d}.stl"
        if not args.audit_only:
            proxy.export(path)
        if not proxy.is_volume or not proxy.is_convex:
            raise ValueError("Invalid collision proxy")
        cavity_blocked |= proxy.contains(probes)
        pieces.append({"path": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                       "vertices": len(proxy.vertices), "volume_m3": float(proxy.volume)})
    report = {"source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "algorithm": "CoACD",
              "parameters": {"threshold":0.035,"max_convex_hull":48,"seed":42}, "pieces": pieces,
              "visual_volume_m3": float(mesh.volume), "sum_convex_volume_m3": sum(p["volume_m3"] for p in pieces),
              "hull_cap_reached": len(pieces) == 48, "requested_concavity_threshold_certified": False,
              "cavity_axis_probes": [{"xyz_m": p.tolist(), "inside_any_proxy": bool(b)} for p,b in zip(probes,cavity_blocked)],
              "cavity_axis_probes_pass": not bool(cavity_blocked.any()),
              "claim": "Convex contact approximation for assembly preview; hull cap may prevent the requested concavity threshold. Five sleeve-axis probes do not certify the complete opening or small fastener holes."}
    if cavity_blocked.any():
        raise ValueError("Collision proxies block sampled sleeve axis")
    manifest_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"convex_pieces":len(pieces),"visual_volume_m3":report["visual_volume_m3"],"sum_convex_volume_m3":report["sum_convex_volume_m3"]}, indent=2))


if __name__ == "__main__":
    main()
