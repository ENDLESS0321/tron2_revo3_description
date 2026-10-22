from pathlib import Path
import copy, hashlib, json, xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
A=ROOT/'assets'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def load(c):
 m=c.find('geometry/mesh')
 path=A/m.get('filename')
 record=json.loads((A/'manifest.json').read_text())['files']['assets/'+m.get('filename')]
 assert sha(path)==record['sha256'], 'Source mesh changed: '+str(path)
 mesh=trimesh.load_mesh(path,process=False)
 v=np.asarray(mesh.vertices)*np.fromstring(m.get('scale','1 1 1'),sep=' ')
 o=c.find('origin')
 if o is not None:
  v=v@Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix().T+np.fromstring(o.get('xyz','0 0 0'),sep=' ')
 return trimesh.convex.convex_hull(v)
assert sha(A/'assembly_rl_convex.urdf') == '623be410f99dce037117fc7b08c888aaa690ae4796e1a5f187c27b120f9ad41f', 'Unexpected source URDF'
r=ET.parse(A/'assembly_rl_convex.urdf').getroot()
assert len(r.findall('.//collision')) == 91
groups={}
for l in r.findall('link'):
 cs=l.findall('collision')
 if (l.get('name').startswith('right_') and len(cs)>1
         and l.get('name') not in {'right_hand_base_link', 'right_thumb_MCP_Link', 'right_thumb_PIP_Link'}
         and not l.get('name').endswith('_DIP_Link')):
  groups[l.get('name')]=[dict(mesh=load(c),sources=[i],element=c) for i,c in enumerate(cs)]
steps=[]
# Greedy, deterministic merging within each rigid link. Never merge across joints.
for step in range(33):
 best=None
 for name,gs in groups.items():
  for i in range(len(gs)):
   for j in range(i+1,len(gs)):
    a,b=gs[i]['mesh'],gs[j]['mesh']
    h=trimesh.convex.convex_hull(np.vstack([a.vertices,b.vertices]))
    cost=max(0.,h.volume-a.volume-b.volume)
    key=(cost,name,i,j)
    if best is None or key<best[0]: best=(key,h)
 (cost,name,i,j),h=best
 gs=groups[name]; sources=gs[i]['sources']+gs[j]['sources']
 steps.append(dict(link=name,source_parts=sorted(sources),hull_excess_over_sum_m3=cost))
 gs[i]=dict(mesh=h,sources=sources,element=None);gs.pop(j)
 print(step+1,name,len(gs),cost,flush=True)
D=A/'meshes/retarget58/right';D.mkdir(parents=True,exist_ok=True)
changed=[]
for name,gs in groups.items():
 l=r.find(f"link[@name='{name}']");old=l.findall('collision')
 if len(gs)==len(old):continue
 originals=[load(c) for c in old]
 for c in old:l.remove(c)
 for n,g in enumerate(gs):
  if g['element'] is not None:l.append(g['element']);continue
  path=D/f'{name}_{n:02d}.stl';g['mesh'].export(path)
  c=ET.SubElement(l,'collision',name=f'{name}_retarget58_{n:02d}')
  ET.SubElement(c,'origin',xyz='0 0 0',rpy='0 0 0')
  ET.SubElement(ET.SubElement(c,'geometry'),'mesh',filename=str(path.relative_to(A)))
 changed.append(dict(link=name,before=len(old),after=len(gs)))
ET.indent(r,space='  ')
out=A/'assembly_retarget58.urdf';ET.ElementTree(r).write(out,encoding='utf-8',xml_declaration=True)
report=dict(schema='retarget_collision_reduction/v1',source='assembly_rl_convex.urdf',source_sha256=sha(A/'assembly_rl_convex.urdf'),output=out.name,output_sha256=sha(out),before=91,after=len(r.findall('.//collision')),method='Greedy minimum added convex-hull volume; merges only within one rigid link; palm/base, all DIP fingertip collisions, thumb MCP/PIP, and non-hand collisions are preserved; source mesh vertices enclosed by hull. Cost subtracts summed component volumes and is not a union-volume measurement.',changed_links=changed,merges=steps,limitations=['Convex merging can fill gaps and enlarge contact surfaces.','No trajectory, PhysX, dynamics, checkpoint or hardware validation.'])
(A/'retarget58.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(changed,indent=2));print(report['after'])
