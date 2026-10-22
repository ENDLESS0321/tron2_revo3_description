import sys, copy, json, hashlib
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np, trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1];a=root/'assets'
s=E.parse(a/'assembly_rl_convex.urdf').getroot();t=E.parse(a/'assembly_retarget58.urdf').getroot()
# Semantic equality of all noncollision XML.
def signature(r):
 r=copy.deepcopy(r)
 for l in r.findall('link'):
  for c in l.findall('collision'):l.remove(c)
 def sig(e):return (e.tag,tuple(sorted(e.attrib.items())),(e.text or '').strip(),tuple(sig(c) for c in e))
 return sig(r)
assert signature(s)==signature(t)
assert len(s.findall('.//collision'))==91 and len(t.findall('.//collision'))==58
report=json.loads((a/'retarget58.json').read_text());qa=[]
for field,key in [('source','source_sha256'),('output','output_sha256')]:
 assert hashlib.sha256((a/report[field]).read_bytes()).hexdigest()==report[key]
def mesh(c):
 m=c.find('geometry/mesh');h=trimesh.load_mesh(a/m.get('filename'),process=False);h.apply_scale(np.fromstring(m.get('scale','1 1 1'),sep=' '))
 o=c.find('origin')
 if o is not None:
  mat=np.eye(4);mat[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix();mat[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');h.apply_transform(mat)
 return h
for row in report['changed_links']:
 name=row['link'];old=[mesh(c) for c in s.find(f"link[@name='{name}']").findall('collision')];new=[mesh(c) for c in t.find(f"link[@name='{name}']").findall('collision')]
 planes=[ConvexHull(m.vertices).equations for m in new]
 vertices=np.vstack([m.vertices for m in old]);outside=np.min(np.stack([np.max(vertices@p[:,:3].T+p[:,3],axis=1) for p in planes]),axis=0)
 assert outside.max()<1e-8,(name,outside.max())
 for original in old:
  assert min(np.max(original.vertices@p[:,:3].T+p[:,3]) for p in planes)<1e-8, name
 # Surface-sample distance to original convex union; zero for points inside any original hull.
 samples=np.vstack([np.vstack([m.vertices,m.triangles_center]) for m in new]);inside=np.zeros(len(samples),bool)
 for m in old:
  p=ConvexHull(m.vertices).equations;inside|=np.max(samples@p[:,:3].T+p[:,3],axis=1)<=1e-9
 distances=np.zeros(len(samples));ix=np.flatnonzero(~inside)
 if len(ix):
  distances[ix]=np.min(np.stack([trimesh.proximity.closest_point_naive(m,samples[ix])[1] for m in old]),axis=0)
 qa.append(dict(link=name,source_vertex_max_plane_violation_m=float(outside.max()),surface_samples=len(samples),sampled_max_expansion_m=float(distances.max())))
 print(name,distances.max(),flush=True)
for m in t.findall('.//mesh'):assert (a/m.get('filename')).is_file()
sys.path.insert(0,str(root/'viewers'))
from _runtime import _mujoco_urdf
import mujoco
temp,staged=_mujoco_urdf(a/'assembly_retarget58.urdf')
spec=mujoco.MjSpec.from_file(str(staged));spec.compiler.balanceinertia=True;model=spec.compile();assert model.nq==28
temp.cleanup()
report['verification']=dict(noncollision_xml_unchanged=True,source_vertices_enclosed_tolerance_m=1e-8,mesh_closure=True,mujoco_kinematic_dof=model.nq,mujoco_balanceinertia_preview_only=True,links=qa,sampled_max_expansion_m=max(x['sampled_max_expansion_m'] for x in qa))
(a/'retarget58.json').write_text(json.dumps(report,indent=2)+'\n')
assert report['verification']['sampled_max_expansion_m'] < 0.0025
print('PASS',report['verification']['sampled_max_expansion_m'])
