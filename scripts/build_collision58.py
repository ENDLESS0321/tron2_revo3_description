"""Merge same-link convex components without changing robot kinematics.

DIP collision surfaces and all non-hand geometry are protected. Greedy merge
ranking minimizes sampled outward displacement from the original convex union.
The report is a sampled geometric approximation check, not a Hausdorff proof.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import heapq
import json
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def signature(node):
    return [node.tag, sorted(node.attrib.items()), (node.text or '').strip(),
            [signature(c) for c in node]]


def hull_mesh(points):
    h = ConvexHull(np.unique(points, axis=0))
    return trimesh.Trimesh(vertices=h.points, faces=h.simplices, process=True).convex_hull


def surface_points(mesh):
    triangles = mesh.triangles
    return np.concatenate([mesh.vertices, triangles.mean(axis=1),
                           (triangles[:, 0] + triangles[:, 1])/2,
                           (triangles[:, 1] + triangles[:, 2])/2,
                           (triangles[:, 2] + triangles[:, 0])/2])


def outside_distance(points, originals):
    # Plane violation ranks candidates; final report uses triangle distances.
    distances = []
    for mesh in originals:
        planes = ConvexHull(mesh.vertices).equations
        distances.append(np.maximum(0, (points @ planes[:, :3].T + planes[:, 3]).max(axis=1)))
    return np.min(distances, axis=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--target', type=int, choices=[58], default=58)
    args = parser.parse_args()
    source_sha = hashlib.sha256(args.urdf.read_bytes()).hexdigest()
    expected_source = '623be410f99dce037117fc7b08c888aaa690ae4796e1a5f187c27b120f9ad41f'
    if source_sha != expected_source:
        raise ValueError('Expected the pinned official 91-collision migration input')
    source = ET.parse(args.urdf)
    robot = source.getroot()
    before_count = len(robot.findall('.//collision'))
    if before_count != 91:
        raise ValueError('Expected exactly 91 source collision elements')
    args.output.mkdir(parents=True, exist_ok=False)
    original = copy.deepcopy(robot)
    groups = {}
    originals = {}
    counter = 0
    heap = []
    def enqueue(name, i, j):
        a, b = groups[name][i], groups[name][j]
        merged = hull_mesh(np.concatenate([a['mesh'].vertices, b['mesh'].vertices]))
        points = surface_points(merged)
        distances = outside_distance(points, originals[name])
        cost = (float(distances.max()), float(np.mean(distances**2)), len(merged.vertices))
        heapq.heappush(heap, (*cost, name, i, j, merged))
    for link in robot.findall('link'):
        name = link.get('name')
        if not (name == 'right_hand_base_link' or name.startswith(('right_thumb_', 'right_index_', 'right_middle_', 'right_ring_', 'right_little_'))):
            continue
        if '_DIP_' in name:  # Preserve the exact grasp-contact geometry.
            continue
        collisions = link.findall('collision')
        if len(collisions) < 2:
            continue
        groups[name] = {}
        originals[name] = []
        for c in collisions:
            g = c.find('geometry/mesh')
            if g is None:
                raise ValueError('Only explicit convex meshes supported')
            mesh = trimesh.load_mesh(args.urdf.parent/g.get('filename'), force='mesh')
            vertices = mesh.vertices*np.fromstring(g.get('scale', '1 1 1'), sep=' ')
            origin = c.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            vertices = Rotation.from_euler('xyz', rpy).apply(vertices)+xyz
            h = hull_mesh(vertices)
            originals[name].append(h)
            groups[name][counter] = {'mesh': h, 'members': [len(originals[name])-1], 'node': c}
            counter += 1
        ids = list(groups[name])
        for ii, i in enumerate(ids):
            for j in ids[ii+1:]:
                enqueue(name, i, j)
    count = len(robot.findall('.//collision'))
    history = []
    while count > args.target:
        maximum, mean, _, name, i, j, mesh = heapq.heappop(heap)
        if i not in groups[name] or j not in groups[name]:
            continue
        a, b = groups[name].pop(i), groups[name].pop(j)
        members = sorted(a['members']+b['members'])
        groups[name][counter] = {'mesh': mesh, 'members': members, 'node': None}
        history.append({'link': name, 'source_components': members, 'sampled_plane_offset_m': maximum})
        for k in list(groups[name]):
            if k != counter:
                enqueue(name, k, counter)
        counter += 1
        count -= 1
        print('remaining', count, 'link', name, 'offset_mm', maximum*1000, flush=True)
    # Keep all existing runtime references shared with the original closure.
    (args.output/'meshes').symlink_to((args.urdf.parent/'meshes').resolve(), target_is_directory=True)
    generated = args.output/'collision58_meshes'
    generated.mkdir()
    reports = []
    for name, components in groups.items():
        link = robot.find(f"link[@name='{name}']")
        if all(len(c['members']) == 1 for c in components.values()):
            continue
        for node in link.findall('collision'):
            link.remove(node)
        merged_components = []
        for index, component in enumerate(sorted(components.values(), key=lambda c: c['members'])):
            if len(component['members']) == 1:
                link.append(copy.deepcopy(original.find(f"link[@name='{name}']").findall('collision')[component['members'][0]]))
                continue
            mesh = component['mesh']
            path = generated/f'{name}_{index:02d}.obj'
            path.write_text(trimesh.exchange.obj.export_obj(mesh, digits=12))
            c = ET.SubElement(link, 'collision')
            ET.SubElement(c, 'origin', xyz='0 0 0', rpy='0 0 0')
            ET.SubElement(ET.SubElement(c, 'geometry'), 'mesh', filename=str(path.relative_to(args.output)))
            points = surface_points(mesh)
            inside = outside_distance(points, originals[name]) < 1e-10
            distances = np.full(len(points), np.inf)
            for old in originals[name]:
                for start in range(0, len(points), 128):
                    _, d, _ = trimesh.proximity.closest_point_naive(old, points[start:start+128])
                    distances[start:start+128] = np.minimum(distances[start:start+128], d)
            distances[inside] = 0
            merged_components.append({'members': component['members'], 'mesh': str(path.relative_to(args.output)),
                                      'vertices': len(mesh.vertices), 'sampled_outward_max_m': float(distances.max()),
                                      'sampled_outward_p95_m': float(np.percentile(distances, 95)), 'sample_count': len(points)})
        reports.append({'link': name, 'before': len(originals[name]), 'after': len(components), 'merged': merged_components})
    def strip_collisions(root):
        r = copy.deepcopy(root)
        for l in r.findall('link'):
            for c in l.findall('collision'):
                l.remove(c)
        return signature(r)
    assert strip_collisions(robot) == strip_collisions(original)
    assert len(robot.findall('.//collision')) == args.target
    ET.indent(robot, space='  ')
    output = args.output/'assembly_collision58.urdf'
    source.write(output, encoding='utf-8', xml_declaration=True)
    report = {'source_sha256': source_sha,
              'output_sha256': hashlib.sha256(output.read_bytes()).hexdigest(), 'before': before_count, 'after': args.target,
              'noncollision_xml_preserved': True, 'all_dip_collision_xml_preserved': True,
              'modified_links': reports, 'merge_steps': history,
              'boundary': 'Conservative convex merging; sampled outward error only, no continuous Hausdorff or physics claim.'}
    (args.output/'collision58_report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
