"""Cross-section evidence for mounting frames; no physical-fit claims."""
import contextlib
import io
import json
import sys
import numpy as np
from inspect_adapter import analyze, OUT
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import trimesh

with contextlib.redirect_stdout(io.StringIO()):
    report, geometry = analyze()
sections = [
    ("1", 0, -28.), ("1", 0, 28.), ("1", 2, -22.), ("1", 2, 22.),
    ("2", 0, -25.), ("2", 0, 25.), ("2", 2, -25.), ("2", 2, 25.),
    ("1", 1, 20.), ("1", 1, 0.), ("2", 1, -20.), ("2", 1, -10.),
]
fig, axes = plt.subplots(3, 4, figsize=(18, 14), constrained_layout=True)
summary = []
for ax, (oid, axis, coord) in zip(axes.flat, sections):
    vertices, faces, *_ = geometry[oid]
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    origin = np.zeros(3)
    normal = np.zeros(3)
    origin[axis] = coord
    normal[axis] = 1
    section = mesh.section(plane_origin=origin, plane_normal=normal)
    record = {"part": oid, "axis": "xyz"[axis], "coordinate_mm": coord, "loops": []}
    other = [j for j in range(3) if j != axis]
    if section is not None:
        for j, entity in enumerate(section.entities):
            points = section.vertices[entity.points]
            projected = points[:, other]
            fitted = np.linalg.lstsq(np.column_stack([2 * projected, np.ones(len(points))]), np.sum(projected ** 2, axis=1), rcond=None)[0]
            center = fitted[:2]
            radius = float(np.sqrt(max(0, fitted[2] + np.dot(center, center))))
            radial_errors = np.abs(np.linalg.norm(projected-center, axis=1)-radius)
            record["loops"].append({"index": j, "points": len(points), "bounds_mm": [projected.min(axis=0).tolist(), projected.max(axis=0).tolist()], "circle_fit_center_mm": center.tolist(), "circle_fit_radius_mm": radius, "circle_fit_max_error_mm": radial_errors.max()})
            ax.plot(projected[:, 0], projected[:, 1], linewidth=0.8)
            if radial_errors.max() < .03:
                ax.annotate(f"r={radius:.3f}\n({center[0]:.3f},{center[1]:.3f})", center, fontsize=7)
    ax.set(title=f"Part {oid}: {'XYZ'[axis]}={coord:g} mm", xlabel=f"{'XYZ'[other[0]]} [mm]", ylabel=f"{'XYZ'[other[1]]} [mm]")
    ax.axis("equal")
    ax.grid(alpha=.3)
    summary.append(record)
fig.savefig(OUT / "adapter_sections.png", dpi=180)
(OUT / "adapter_sections.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
print(json.dumps(summary, indent=2))
