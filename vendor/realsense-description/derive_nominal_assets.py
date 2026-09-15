#!/usr/bin/env python3
"""Derive simulator-friendly meshes and explicit nominal camera frames.

Original upstream files are read-only. The D435 COLLADA contains one identity
scene node and 18 triangle geometry instances; this converter rejects a more
general scene rather than silently ignoring transforms. Camera URDF frames are
a static expansion of the inspected official xacros with nominal extrinsics on
and USB plug geometry off. Upstream placeholder inertias remain identified as
unreliable and are not upgraded into a calibrated dynamics claim.
"""
from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
PACKAGE = ROOT / "realsense2_description"
OUT = ROOT / "generated"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(values):
    return " ".join(f"{float(v):.15g}" for v in values)


def collada_triangles(path):
    root = ET.parse(path).getroot()
    if float(root.find("c:asset/c:unit", NS).get("meter")) != 1.:
        raise ValueError("Expected the upstream meter-based D435 COLLADA")
    nodes = root.findall("c:library_visual_scenes/c:visual_scene/c:node", NS)
    if len(nodes) != 1 or any(child.tag.split("}")[-1] in ("matrix", "translate", "rotate", "scale", "node", "instance_node") for child in nodes[0]):
        raise ValueError("Unexpected nonidentity or nested COLLADA scene")
    selected = [instance.get("url")[1:] for instance in nodes[0].findall("c:instance_geometry", NS)]
    lookup = {geometry.get("id"): geometry for geometry in root.findall("c:library_geometries/c:geometry", NS)}
    pieces = []
    for gid in selected:
        mesh = lookup[gid].find("c:mesh", NS)
        sources = {source.get("id"): source for source in mesh.findall("c:source", NS)}
        vertices = {node.get("id"): node.find("c:input[@semantic='POSITION']", NS).get("source")[1:] for node in mesh.findall("c:vertices", NS)}
        if mesh.findall("c:polylist", NS) or mesh.findall("c:polygons", NS):
            raise ValueError("Unexpected nontriangular COLLADA geometry")
        for primitive in mesh.findall("c:triangles", NS):
            inputs = primitive.findall("c:input", NS)
            vertex_input = next(row for row in inputs if row.get("semantic") == "VERTEX")
            source = sources[vertices[vertex_input.get("source")[1:]]]
            accessor = source.find("c:technique_common/c:accessor", NS)
            stride = int(accessor.get("stride", "1"))
            if stride != 3 or int(accessor.get("offset", "0")) != 0:
                raise ValueError("Unexpected position array accessor")
            points = np.fromstring(source.find("c:float_array", NS).text, sep=" ").reshape(-1, 3)
            index_stride = 1+max(int(row.get("offset", "0")) for row in inputs)
            indices = np.fromstring(primitive.find("c:p", NS).text, sep=" ", dtype=np.int64).reshape(-1, index_stride)
            faces = indices[:, int(vertex_input.get("offset", "0"))].reshape(-1, 3)
            if len(faces) != int(primitive.get("count")):
                raise ValueError("COLLADA triangle count mismatch")
            pieces.append(trimesh.Trimesh(vertices=points, faces=faces, process=False))
    return trimesh.util.concatenate(pieces), len(selected)


def fixed(robot, name, parent, child, xyz=(0.,0.,0.), rpy=(0.,0.,0.)):
    joint = ET.SubElement(robot, "joint", name=name, type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=vector(xyz), rpy=vector(rpy))
    ET.SubElement(robot, "link", name=child)


def nominal_urdf(model, mesh_path):
    is405 = model == "D405"
    offset = [.01085,.009,.021] if is405 else [.0106,.0175,.0125]
    visual_offset = [.0038,-.009,0.] if is405 else [.0043,-.0175,0.]
    box_offset = [-.0078,-.009,0.] if is405 else [0.,-.0175,0.]
    box_size = [.023,.042,.042] if is405 else [.02505,.090,.025]
    optical = [-math.pi/2, 0., -math.pi/2]
    robot = ET.Element("robot", name="realsense_"+model.lower()+"_nominal")
    robot.append(ET.Comment("Static nominal expansion of official Apache-2.0 xacros; calibrated runtime extrinsics are not represented. See the upstream LICENSE, NOTICE.md and generated/manifest.json."))
    ET.SubElement(robot, "link", name="base_link")
    fixed(robot,"camera_joint","base_link","camera_bottom_screw_frame")
    fixed(robot,"camera_link_joint","camera_bottom_screw_frame","camera_link",offset)
    body = robot.find("link[@name='camera_link']")
    visual = ET.SubElement(body,"visual")
    ET.SubElement(visual,"origin",xyz=vector(visual_offset),rpy=vector([math.pi/2,0.,math.pi/2]))
    ET.SubElement(ET.SubElement(visual,"geometry"),"mesh",filename=mesh_path.name)
    material = ET.SubElement(visual,"material",name="upstream_aluminum")
    ET.SubElement(material,"color",rgba="0.5 0.5 0.5 1" if is405 else "0.74902 0.74902 0.74902 1")
    collision = ET.SubElement(body,"collision")
    ET.SubElement(collision,"origin",xyz=vector(box_offset),rpy="0 0 0")
    ET.SubElement(ET.SubElement(collision,"geometry"),"box",size=vector(box_size))
    body.append(ET.Comment("Unreliable upstream placeholder mass/inertia retained only for faithful source expansion. Do not treat as calibrated physical parameters."))
    inertial = ET.SubElement(body,"inertial")
    ET.SubElement(inertial,"mass",value="0.072")
    ET.SubElement(inertial,"origin",xyz="0 0 0",rpy="0 0 0")
    ET.SubElement(inertial,"inertia",ixx="0.003881243",ixy="0",ixz="0",iyy="0.000498940",iyz="0",izz="0.003879257")
    offsets = {"depth":[0.,0.,0.],"infra1":[0.,0.,0.],"infra2":[0.,-.018 if is405 else -.050,0.],"color":[0.,0. if is405 else .015,0.]}
    if not is405:
        offsets.update({"accel":[-.01174,-.00552,.0051],"gyro":[-.01174,-.00552,.0051]})
    for sensor, xyz in offsets.items():
        fixed(robot,"camera_"+sensor+"_joint","camera_link","camera_"+sensor+"_frame",xyz)
        fixed(robot,"camera_"+sensor+"_optical_joint","camera_"+sensor+"_frame","camera_"+sensor+"_optical_frame",rpy=optical)
    ET.indent(robot,space="  ")
    path=OUT/(model.lower()+"_nominal.urdf")
    ET.ElementTree(robot).write(path,encoding="utf-8",xml_declaration=True)
    return path


def main():
    OUT.mkdir(exist_ok=True)
    paths = {"D405":PACKAGE/"meshes/d405.stl","D435i":PACKAGE/"meshes/d435.dae"}
    source405 = trimesh.load_mesh(paths["D405"],process=False)
    source405.apply_scale(.001)
    source435, geometry_count = collada_triangles(paths["D435i"])
    rows={}
    for model,mesh in (("D405",source405),("D435i",source435)):
        target=OUT/("realsense_"+model.lower()+"_native_axes_m.stl")
        mesh.export(target)
        reread=trimesh.load_mesh(target,process=False)
        if len(mesh.faces)!=len(reread.faces) or not np.allclose(mesh.bounds,reread.bounds,atol=1e-8,rtol=0):
            raise ValueError("Derived STL differs from source triangle geometry")
        urdf=nominal_urdf(model,target)
        rows[model]={"source":str(paths[model].relative_to(ROOT)),"source_sha256":digest(paths[model]),"source_unit":"millimeter" if model=="D405" else "meter","derived_mesh":str(target.relative_to(ROOT)),"derived_mesh_sha256":digest(target),"derived_unit":"meter","native_axes_bounds_m":mesh.bounds.tolist(),"triangles":len(mesh.faces),"nominal_urdf":str(urdf.relative_to(ROOT)),"nominal_urdf_sha256":digest(urdf),"method":"scale original STL by 0.001" if model=="D405" else f"flatten {geometry_count} original identity-node COLLADA triangle instances; preserve vertex coordinates, omit single gray material from STL"}
    report={"schema":"realsense_nominal_asset_derivatives_v1","producer_sha256":digest(Path(__file__)),"originals_modified":False,"nominal_extrinsics":True,"usb_plug_geometry":False,"inertial_status":"Upstream explicitly marks mass/inertia unreliable; preserved here for provenance only.","models":rows}
    (OUT/"manifest.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(rows,indent=2))


if __name__=="__main__":main()
