#!/usr/bin/env python3
"""Read-only geometry audit of the user's fixed 330/30 wrist-hole pair.

No original STEP, assembly configuration or URDF is modified by this audit.
"""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'design/revision2/analysis'))
from extract_wrist_features import wrist_mesh, cylinder_candidate, section_polylines
import matplotlib.pyplot as plt


def point(degrees,radius=30.5):
    a=np.deg2rad(degrees)
    return np.array([radius*np.cos(a),-4.5,radius*np.sin(a)])


def main():
    out=ROOT/'reports/cameras/requested_inward_holes_20260914'
    out.mkdir(parents=True,exist_ok=True)
    info=json.loads((ROOT/'design/imported_camera_brackets_20260914/interfaces.json').read_text())
    report={'scope':'Corrected user target: robot small holes 330/30 stay fixed; retain hand/adapter rotations; inspect bracket inward rotation only.',
            'originals_and_assemblies_modified':False,'sides':{}}
    fig,axes=plt.subplots(2,2,figsize=(11,10),layout='constrained')
    for row,side in enumerate(['left','right']):
        data=info['sides'][side]
        source=Path(data['source'])
        assert hashlib.sha256(source.read_bytes()).hexdigest()==data['source_sha256']
        wm=wrist_mesh(side)
        target=np.array([point(a) for a in [30,330]])
        robot_fits=[cylinder_candidate(wm,a) for a in [30,330]]
        mesh=trimesh.load_mesh(ROOT/data['mesh_mm'],process=True)
        mount=data['wrist_to_source_in_old_adapter']
        R0=np.array(mount['rotation']);t=np.array(mount['xyz_mm'])
        Ri=Rotation.from_euler('y',90 if side=='left' else -90,degrees=True).as_matrix()@R0
        # CAD axis origins can be far from the solid. Resolve the occupied
        # radial ray from the actual cylinder face centroid (not axis sign).
        inventory=json.loads((ROOT/'design/imported_camera_brackets_20260914/source_inspection.json').read_text())
        faces=inventory['sources'][side]['brep']['solids'][0]['faces']
        occupied=[]
        for h in data['wrist_cylinders']:
            face=next(f for f in faces if f['index']==h['index'])
            center=np.array(face['center_mm']);u=np.array(h['u'])
            u*=np.sign(np.dot(u[[0,2]],center[[0,2]]))
            occupied.append(u*30.5+np.array([0,-1.4,0]))
        occupied=np.array(occupied)
        inward=occupied@Ri.T+t
        chord=float(np.linalg.norm(occupied[0]-occupied[1]))
        nearest=np.linalg.norm(inward[:,None]-target[None,:],axis=2)
        report['sides'][side]={'robot_target_angles_deg':[30,330],
            'robot_small_hole_fits':robot_fits,'robot_target_points_at_r30_5_mm':target.tolist(),
            'bracket_source_contact_points_mm':occupied.tolist(),
            'bracket_hole_contact_chord_mm':chord,
            'robot_hole_contact_chord_mm':float(np.linalg.norm(target[0]-target[1])),
            'inward_bracket_points_mm':inward.tolist(),
            'inward_bracket_angles_deg':(np.rad2deg(np.arctan2(inward[:,2],inward[:,0]))%360).tolist(),
            'inward_nearest_hole_position_errors_mm':nearest.min(axis=1).tolist(),
            'rigid_mount_to_requested_holes_possible':bool(abs(chord-np.linalg.norm(target[0]-target[1]))<.01)}
        for col,R in enumerate([R0,Ri]):
            ax=axes[row,col]
            m=mesh.copy();m.vertices=m.vertices@R.T+t
            for model,color,label in [(wm,'#526477','Robot wrist'),(m,'#2b87cf','Supplied bracket')]:
                for i,p in enumerate(section_polylines(model,1,-4.5)):
                    ax.plot(p[:,0],p[:,2],color=color,lw=1.2,label=label if i==0 else None)
            holes=occupied@R.T+t
            ax.scatter(holes[:,0],holes[:,2],c='#2b87cf',s=60,marker='o',label='STEP hole contact points',zorder=5)
            ax.scatter(target[:,0],target[:,2],c='#e33d32',s=110,marker='x',linewidths=2.5,label='Required robot small holes',zorder=6)
            ax.plot(target[:,0],target[:,2],':',c='#e33d32')
            if col:
                for p in holes:
                    q=target[np.argmin(np.linalg.norm(target-p,axis=1))]
                    ax.plot([p[0],q[0]],[p[2],q[2]],'--',c='#e33d32')
                ax.text(-42,-39,'30 deg mismatch at each hole\n15.79 mm at R30.5',fontsize=10,color='#b82018')
            ax.set_title(side.capitalize()+(' | Previous mounting' if col==0 else ' | Bracket rotated inward 90 deg'))
            ax.set(xlim=(-46,46),ylim=(-46,46),xlabel='Original adapter X [mm]',ylabel='Original adapter Z [mm]')
            ax.set_aspect('equal');ax.grid(alpha=.15)
    axes[0,0].legend(fontsize=8,loc='lower left')
    fig.suptitle('Actual mesh section at Y=-4.5 mm | Red target holes remain fixed\nSTEP contact chord 52.83 mm; required contact chord 30.50 mm',fontsize=13)
    fig.savefig(out/'hole_pair_comparison.png',dpi=150)
    (out/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({s:{k:v for k,v in r.items() if k not in ['robot_small_hole_fits','bracket_source_contact_points_mm']} for s,r in report['sides'].items()},indent=2))


if __name__=='__main__':main()
