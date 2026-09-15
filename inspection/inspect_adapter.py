"""Read-only geometric inspection of the original Bambu Studio 3MF project."""
from pathlib import Path
import collections
import json
import zipfile
import xml.etree.ElementTree as ET
import sys
import numpy as np
import mpl_toolkits
local_mpl = Path(sys.prefix) / f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/mpl_toolkits"
if local_mpl.is_dir() and str(local_mpl) not in mpl_toolkits.__path__:
    mpl_toolkits.__path__.insert(0, str(local_mpl))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SOURCE = Path("/home/endless/Documents/Project/Dexterous_Hand/tron2-revo3转接件_第1版.3mf")
OUT = Path(__file__).resolve().parent
NS = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}

def transform(value):
    data = np.array([float(x) for x in value.split()]).reshape(4, 3)
    return data[:3], data[3]

def analyze():
    with zipfile.ZipFile(SOURCE) as archive:
        root = ET.fromstring(archive.read("3D/3dmodel.model"))
        parts_root = ET.fromstring(archive.read("3D/Objects/object_2.model"))
        settings = ET.fromstring(archive.read("Metadata/model_settings.config"))
        for name in ("plate_1.png", "top_1.png", "plate_no_light_1.png"):
            (OUT / name).write_bytes(archive.read("Metadata/" + name))
    parts = {}
    for elem in settings.findall("./object/part"):
        parts[elem.attrib["id"]] = {
            "subtype": elem.attrib["subtype"],
            "metadata": {m.attrib["key"]: m.attrib["value"] for m in elem.findall("metadata")},
        }
    assembly = {x.attrib["objectid"]: x.attrib["transform"] for x in root.findall("./m:resources/m:object/m:components/m:component", NS)}
    geometry = {}
    report = {"source": str(SOURCE), "unit": root.attrib["unit"], "build_transform": root.find("./m:build/m:item", NS).attrib["transform"], "parts": {}}
    for obj in parts_root.findall("./m:resources/m:object", NS):
        oid = obj.attrib["id"]
        vertices = np.array([[float(v.attrib[a]) for a in ("x", "y", "z")] for v in obj.findall("./m:mesh/m:vertices/m:vertex", NS)])
        faces = np.array([[int(f.attrib[a]) for a in ("v1", "v2", "v3")] for f in obj.findall("./m:mesh/m:triangles/m:triangle", NS)])
        rotation, offset = transform(assembly[oid])
        vertices = vertices @ rotation + offset
        tri = vertices[faces]
        cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        area = np.linalg.norm(cross, axis=1) / 2
        normals = np.divide(cross, 2 * area[:, None], out=np.zeros_like(cross), where=area[:, None] > 1e-12)
        centroid = tri.mean(axis=1)
        planes = {}
        for axis, letter in enumerate("xyz"):
            mask = np.abs(normals[:, axis]) > 0.999999
            buckets = collections.defaultdict(lambda: {"area": 0.0, "triangles": 0, "indices": []})
            for idx in np.flatnonzero(mask):
                key = (round(float(centroid[idx, axis]), 3), int(np.sign(normals[idx, axis])))
                buckets[key]["area"] += area[idx]
                buckets[key]["triangles"] += 1
                buckets[key]["indices"].append(int(idx))
            dominant = sorted(buckets.items(), key=lambda kv: -kv[1]["area"])[:12]
            planes[letter] = [{"coordinate_mm": key[0], "normal_sign": key[1], "area_mm2": val["area"], "triangles": val["triangles"], "bounds_mm": [tri[val["indices"]].min(axis=(0, 1)).tolist(), tri[val["indices"]].max(axis=(0, 1)).tolist()]} for key, val in dominant]
        edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        report["parts"][oid] = {
            **parts[oid], "component_transform": assembly[oid], "vertices": len(vertices), "triangles": len(faces),
            "bounds_mm": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
            "dimensions_mm": np.ptp(vertices, axis=0).tolist(),
            "signed_volume_mm3": np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6,
            "area_mm2": float(area.sum()), "nonmanifold_edge_count": int(np.count_nonzero(counts != 2)),
            "dominant_axis_aligned_planes": planes,
        }
        geometry[oid] = (vertices, faces, normals, area)
    positives = np.concatenate([geometry[k][0] for k in geometry if parts[k]["subtype"] == "normal_part"])
    report["positive_assembly_bounds_mm"] = [positives.min(axis=0).tolist(), positives.max(axis=0).tolist()]
    report["positive_assembly_dimensions_mm"] = np.ptp(positives, axis=0).tolist()
    (OUT / "adapter_geometry.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)
    for ax, (x, y) in zip(axes, [(0, 1), (0, 2), (2, 1)]):
        for oid, (vertices, faces, normals, area) in geometry.items():
            if oid != "3":
                # All projected vertices preserve silhouette and small mounting holes.
                ax.scatter(vertices[:, x], vertices[:, y], s=0.08, c={"1": "#3174b2", "2": "#d18031"}[oid], alpha=0.28, rasterized=True, label=f"Part {oid}")
            else:
                vmin, vmax = vertices.min(axis=0), vertices.max(axis=0)
                ax.add_patch(plt.Rectangle((vmin[x], vmin[y]), vmax[x]-vmin[x], vmax[y]-vmin[y], facecolor="none", edgecolor="red", linewidth=1.5, label="Part 3: negative"))
        ax.set(xlabel="XYZ"[x] + " [mm]", ylabel="XYZ"[y] + " [mm]", title="Assembly frame, nominal CAD units (before 1.01 print scale)")
        ax.axis("equal")
        ax.grid(alpha=0.25)
        ax.legend(markerscale=10, loc="best")
    fig.savefig(OUT / "adapter_orthographic.png", dpi=220)
    plt.close(fig)
    fig = plt.figure(figsize=(10, 8), constrained_layout=True)
    ax = fig.add_subplot(projection="3d")
    for oid, (vertices, faces, normals, area) in geometry.items():
        step = max(1, len(faces) // 16000)
        collection = Poly3DCollection(vertices[faces[::step]], facecolor={"1": "#76a5cc", "2": "#e3a567", "3": "#e44747"}[oid], edgecolor="none", alpha=.08 if oid=="3" else .8)
        ax.add_collection3d(collection)
    vmin, vmax = positives.min(axis=0), positives.max(axis=0)
    ax.set(xlim=(vmin[0], vmax[0]), ylim=(vmin[1], vmax[1]), zlim=(vmin[2], vmax[2]), xlabel="X [mm]", ylabel="Y [mm]", zlabel="Z [mm]", title="Nominal assembly: hand bracket (blue), flange adapter (orange), subtractive slot (red)")
    ax.set_box_aspect(vmax-vmin)
    ax.view_init(elev=24, azim=-54)
    fig.savefig(OUT / "adapter_3d.png", dpi=200)
    plt.close(fig)
    print(json.dumps({"dimensions_mm": report["positive_assembly_dimensions_mm"], "parts": {k: {key: val[key] for key in ("subtype", "bounds_mm", "dimensions_mm", "triangles", "signed_volume_mm3", "nonmanifold_edge_count")} for k, val in report["parts"].items()}}, indent=2))
    return report, geometry

if __name__ == "__main__":
    analyze()
