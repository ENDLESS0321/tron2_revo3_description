"""Inspect official palm geometry at the candidate sleeve insertion pose."""
import json
from inspect_adapter import OUT
import numpy as np
import matplotlib.pyplot as plt
import trimesh

ROOT = OUT.parent
results = {}
for side in ("right", "left"):
    source = ROOT / f"vendor/brainco-revo3/revo3_system/meshes/hands/visual/{side}/base_link.STL"
    mesh = trimesh.load_mesh(source, process=False)
    mesh.merge_vertices(digits_vertex=12)
    mesh.apply_scale(1000)
    vertices = mesh.vertices
    normals = mesh.face_normals
    areas = mesh.area_faces
    report = {"bounds_hand_frame_mm": mesh.bounds.tolist(), "faces": len(mesh.faces), "watertight": bool(mesh.is_watertight), "sections": []}
    sections = [(0, -17), (0, 17), (1, -24), (1, 24), (2, 0.01), (2, 5), (2, 18), (2, 22)]
    fig, axes = plt.subplots(2, 4, figsize=(18, 9), constrained_layout=True)
    for ax, (axis, coord) in zip(axes.flat, sections):
        origin, normal = np.zeros(3), np.zeros(3)
        origin[axis] = coord
        normal[axis] = 1
        section = mesh.section(plane_origin=origin, plane_normal=normal)
        others = [j for j in range(3) if j != axis]
        record = {"axis": "xyz"[axis], "coordinate_mm": coord, "entities": []}
        if section is not None:
            for entity in section.entities:
                points = section.vertices[entity.points][:, others]
                a = np.column_stack([2*points, np.ones(len(points))])
                fitted = np.linalg.lstsq(a, np.sum(points**2, axis=1), rcond=None)[0]
                center = fitted[:2]
                radius = float(np.sqrt(max(0, fitted[2]+np.dot(center,center))))
                err = np.max(np.abs(np.linalg.norm(points-center,axis=1)-radius))
                bounds = [points.min(axis=0).tolist(), points.max(axis=0).tolist()]
                record["entities"].append({"bounds_mm": bounds,"circle_fit_center_mm":center.tolist(),"circle_fit_radius_mm": radius, "circle_fit_max_error_mm":float(err),"points":len(points)})
                ax.plot(points[:,0],points[:,1],lw=.6)
                if err < .03:
                    ax.annotate(f"r={radius:.3f}\n({center[0]:.3f},{center[1]:.3f})",center,fontsize=7)
        ax.set(title=f"{side} palm: {'XYZ'[axis]}={coord:g} mm",xlabel="XYZ"[others[0]]+" [mm]",ylabel="XYZ"[others[1]]+" [mm]")
        ax.axis("equal")
        ax.grid(alpha=.25)
        report["sections"].append(record)
    fig.savefig(OUT / f"{side}_palm_sections.png", dpi=160)
    plt.close(fig)
    results[side] = report
(OUT / "hand_fit_geometry.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
print(json.dumps({side:{"bounds":r["bounds_hand_frame_mm"], "mounting_holes":[{"axis":s["axis"],"coordinate_mm":s["coordinate_mm"],"circles":[e for e in s["entities"] if e["points"]>15 and e["circle_fit_max_error_mm"] < .03 and e["circle_fit_radius_mm"]>.5]} for s in r["sections"][:4]]} for side,r in results.items()}, indent=2))
