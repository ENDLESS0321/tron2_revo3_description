from pathlib import Path
import copy, shutil, subprocess, tempfile
import xml.etree.ElementTree as ET
import trimesh
from scipy.spatial.transform import Rotation
import mujoco
ROOT=Path(__file__).resolve().parents[1]
assets=ROOT/'assets'
work=Path(tempfile.mkdtemp(prefix='official-scene-build-'))
# Build converter input with an isolated mesh tree; convert only the two official DAE visuals to STL.
shutil.copy2(assets/'assembly.urdf',work/'assembly.urdf')
shutil.copytree(assets/'meshes',work/'meshes',dirs_exist_ok=True)
meshmap={}
for p in (work/'meshes/hands').rglob('*.dae'):
    out=p.with_name(p.stem+'.mujoco.stl')
    trimesh.load(p,force='mesh').export(out)
    uri=out.relative_to(work).as_posix()
    old=p.relative_to(work).as_posix()
    meshmap[old]=uri
text=(work/'assembly.urdf').read_text()
for old,new in meshmap.items(): text=text.replace(old,new)
(work/'assembly.urdf').write_text(text)
model=mujoco.MjModel.from_xml_path(str(work/'assembly.urdf'))
mujoco.mj_saveLastXML(str(work/'assembly_converted.xml'),model)
converted=ET.parse(work/'assembly_converted.xml').getroot()
scene_tree=ET.parse(assets/'scene.xml'); scene=scene_tree.getroot()
asset=scene.find('asset'); converted_asset=converted.find('asset')
map_by_name={m.get('name'):m for m in converted_asset.findall('mesh')}
# Remove only generated collision mesh declarations so reruns are idempotent.
for node in list(asset.findall('mesh')):
    if node.get('name','').startswith('urdfcollision_'): asset.remove(node)
for side in ('left','right'):
    hand=f'{side}_hand_base_link'; adapter=f'{side}_adapter_link'
    src=converted.find(f'.//body[@name="{adapter}"]/body[@name="{hand}"]')
    dst_parent=scene.find(f'.//body[@name="{adapter}"]')
    old=dst_parent.find(f'./body[@name="{hand}"]')
    if src is None or dst_parent is None or old is None: raise RuntimeError(f'missing hand body path for {side}')
    dst_parent.remove(old)
    new=copy.deepcopy(src)
    # Resolve mesh asset names, renaming only on a conflicting definition.
    existing={m.get('name'):m for m in asset.findall('mesh')}
    for geom in new.findall('.//geom[@mesh]'):
        name=geom.get('mesh'); srcmesh=map_by_name[name]
        prior=existing.get(name)
        if prior is not None and prior.attrib != srcmesh.attrib:
            renamed=f'official_{name}'
            while renamed in existing and existing[renamed].attrib != srcmesh.attrib: renamed='official_'+renamed
            geom.set('mesh',renamed)
            if renamed not in existing:
                m=copy.deepcopy(srcmesh);m.set('name',renamed);asset.append(m);existing[renamed]=m
        elif prior is None:
            asset.append(copy.deepcopy(srcmesh));existing[name]=srcmesh
    dst_parent.append(new)
    # MuJoCo's URDF importer carries visuals but omits <collision>; materialize
    # every official collision mesh with its exact URDF origin and scale.
    urdf=ET.parse(assets/'assembly.urdf').getroot()
    bodies={b.get('name'):b for b in new.iter('body')}
    for link in urdf.findall('link'):
        link_name=link.get('name','')
        if not link_name.startswith(side+'_') or link_name not in bodies: continue
        body=bodies[link_name]
        for index,collision in enumerate(link.findall('collision')):
            geom=collision.find('geometry'); mesh=geom.find('mesh') if geom is not None else None
            if mesh is None: raise RuntimeError(f'non-mesh official hand collision on {link_name}')
            uri=mesh.get('filename'); scale=mesh.get('scale','1 1 1')
            mesh_name=f'urdfcollision_{link_name}_{index}'
            ma=ET.Element('mesh',{'name':mesh_name,'file':uri})
            if scale!='1 1 1': ma.set('scale',scale)
            asset.append(ma)
            origin=collision.find('origin'); xyz=(origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split(); rpy=(origin.get('rpy','0 0 0') if origin is not None else '0 0 0').split()
            quat=Rotation.from_euler('xyz',[float(v) for v in rpy]).as_quat()
            q=' '.join(f'{v:.17g}' for v in [quat[3],quat[0],quat[1],quat[2]])
            ET.SubElement(body,'geom',{'name':f'urdfcollision_geom_{link_name}_{index}','type':'mesh','mesh':mesh_name,'pos':' '.join(xyz),'quat':q})
# Drop mesh assets no longer referenced by any geom, then add any lacking definitions.
used={g.get('mesh') for g in scene.findall('.//geom[@mesh]')}
for node in list(asset.findall('mesh')):
    if node.get('name') not in used: asset.remove(node)
scene_names={m.get('name') for m in asset.findall('mesh')}
for name in used-scene_names:
    asset.append(copy.deepcopy(map_by_name[name]))
# Material RGBA is carried on official geoms by the URDF importer; no fake material or mass is introduced.
ET.indent(scene_tree,space='  ')
scene_tree.write(assets/'scene.xml',encoding='utf-8',xml_declaration=True)
# Copy only derived DAE conversions used by MJCF.
for old,new in meshmap.items():
    dest=assets/new;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(work/new,dest)
# Validate compilation and configuration dimensions.
scene_model=mujoco.MjModel.from_xml_path(str(assets/'scene.xml'))
print('compiled',scene_model.nq,scene_model.ncam,'meshes',len(asset.findall('mesh')),'derived',meshmap)
shutil.rmtree(work)
