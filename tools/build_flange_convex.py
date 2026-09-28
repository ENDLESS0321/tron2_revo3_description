"""Build one convex collision mesh per flange from the approved assembly."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build():
    source = ASSETS / "assembly.urdf"
    tree = ET.parse(source)
    root = tree.getroot()
    report = {"source_urdf_sha256": digest(source), "units": "meters",
              "method": "convex hull of all flange collision vertices in each flange link frame",
              "inertials_preserved": True, "visuals_preserved": True,
              "joints_preserved": True, "physics_stepped": False, "flanges": {}}
    for side in ("left", "right"):
        link = root.find(f"link[@name='{side}_adapter_link']")
        collisions = link.findall("collision")
        points = []
        for collision in collisions:
            mesh_node = collision.find("geometry/mesh")
            if mesh_node is None:
                raise ValueError("Expected mesh collision geometry")
            path = (ASSETS / mesh_node.get("filename")).resolve()
            if not path.is_relative_to(ASSETS):
                raise ValueError("Collision mesh must stay inside assets")
            mesh = trimesh.load_mesh(path, process=False)
            vertices = mesh.vertices * np.fromstring(mesh_node.get("scale", "1 1 1"), sep=" ")
            origin = collision.find("origin")
            if origin is not None:
                rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
                vertices = rotation.apply(vertices) + np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            points.append(vertices)
        points = np.unique(np.vstack(points), axis=0)
        hull = trimesh.Trimesh(vertices=points, process=False).convex_hull
        output = ASSETS / f"meshes/adapter_rl/{side}_flange_convex.stl"
        output.parent.mkdir(parents=True, exist_ok=True)
        hull.export(output)
        # Verify the serialized STL, including its float32 rounding.
        serialized = trimesh.load_mesh(output, process=True)
        planes = ConvexHull(serialized.vertices).equations
        max_outside = float(np.max(points @ planes[:, :3].T + planes[:, 3]))
        if not serialized.is_watertight or not serialized.is_convex or max_outside > 1e-8:
            raise ValueError("Invalid convex envelope")
        for collision in collisions:
            link.remove(collision)
        collision = ET.SubElement(link, "collision", name=f"{side}_flange_single_convex")
        ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "mesh", filename=output.relative_to(ASSETS).as_posix())
        report["flanges"][side] = {
            "link": link.get("name"), "source_collision_count": len(collisions),
            "output_collision_count": 1, "mesh": output.relative_to(ASSETS).as_posix(),
            "mesh_sha256": digest(output), "bounds_m": serialized.bounds.tolist(),
            "hull_volume_m3": float(serialized.volume), "hull_faces": len(serialized.faces),
            "watertight": bool(serialized.is_watertight), "convex": bool(serialized.is_convex),
            "max_source_vertex_outside_hull_m": max_outside}
    target = ASSETS / "assembly_rl_convex.urdf"
    ET.indent(tree, space="  ")
    tree.write(target, encoding="utf-8", xml_declaration=True)
    report["output_urdf_sha256"] = digest(target)
    (ASSETS / "flange_convex.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    build()
