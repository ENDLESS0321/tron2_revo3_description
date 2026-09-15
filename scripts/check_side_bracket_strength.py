#!/usr/bin/env python3
"""V4 side-mount supplement: user board-to-stem 15 deg means CAD Rx 75 deg.

Reuses the validated V2 spatial +Z beam equations, but loads and validates the
actual versioned solid, plate mass and COM. The narrow saddle is NOT certified.
Historical V3 manifests require an explicit --manifest; default is V4.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from check_vertical_bracket_strength import rectangle_torsion, rounded_rectangle_section, case, self_check


ROOT=Path(__file__).resolve().parents[1]
GENERATORS={
    'wrist_camera_bracket_side_saddle_v3':ROOT/'scripts/design_wrist_camera_bracket_v3.py',
    'wrist_camera_bracket_side_bend_v4':ROOT/'scripts/design_wrist_camera_bracket_v4.py',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_side_snapshot(path):
    initial=sha(path);cad=json.loads(path.read_text())
    if cad['schema'] not in GENERATORS:
        raise ValueError('This supplement requires an explicit supported side-mount manifest')
    generator=GENERATORS[cad['schema']]
    if sha(generator)!=cad['generator_sha256']:
        raise ValueError('Side-mount generator differs from the manifest')
    if cad['schema']=='wrist_camera_bracket_side_bend_v4':
        user_angle=float(cad['parameters']['plate_to_stem_angle_deg'])
        plane_angle=float(cad['parameters']['plate_plane_rotation_deg'])
        if not 15<=user_angle<=90 or abs(float(cad['parameters']['plate_tilt_deg'])-user_angle)>1e-9 or abs(user_angle+plane_angle-90)>1e-9 or not np.allclose(cad['plate_frame']['rpy_deg'],[plane_angle,0,0],rtol=0,atol=1e-9):
            raise ValueError('V4 user board-to-stem angle and actual CAD plane rotation are inconsistent')
    files=dict(cad['files'])
    for part in cad['parts'].values():files.update(part['files'])
    for relative,expected in files.items():
        if sha(path.parent/relative)!=expected:raise ValueError(f'Side-mount file hash mismatch: {relative}')
    if sorted(float(h['theta_deg']) for h in cad['features']['wrist_mount_holes'])!=[60.,120.]:
        raise ValueError('Side saddle must use the 60-degree-spaced local small-hole pair')
    if cad['camera_mount']['camera_geometry_included'] is not False:
        raise ValueError('Camera must remain a separate component')
    g=cad['strength_geometry'];stem=cad['parts']['stem'];local=stem['body_local_properties']
    if g['stem_axis']!=[0,0,1] or g['section_type']!='solid_rectangle_with_vertical_edge_fillets':
        raise ValueError('Expected the unchanged solid +Z stem')
    l=float(cad['parameters']['length_mm']);rho=float(stem['density_kg_m3'])
    if not np.allclose(local['center_of_mass_mm'][2],l/2,atol=1e-6):
        raise ValueError('Stem is not consistent with a constant prismatic segment')
    area=stem['volume_mm3']/l
    inertia=np.array(stem['inertia_at_com_kg_m2'])[:2,:2]/(rho*1e-15*l)-np.eye(2)*area*l*l/12
    bounds=np.array(local['bounds_mm']);center=np.array(local['center_of_mass_mm'])
    width=float(g['stem_width_mm']);depth=float(g['stem_depth_mm'])
    f=g['fillets']['stem_vertical_edges'];radius=float(f['radius_mm']) if f['applied'] else 0.
    analytic=rounded_rectangle_section(width,depth,radius)
    if not np.allclose([area,inertia[0,0],inertia[1,1]],list(analytic.values()),rtol=1e-6):
        raise ValueError('Side-mount CAD section fails independent rounded-rectangle check')
    section={'area_mm2':area,'bending_inertia_matrix_xy_mm4':inertia.tolist(),
             'Ix_mm4':float(inertia[0,0]),'Iy_mm4':float(inertia[1,1]),
             'centroid_xy_root_mm':stem['center_of_mass_mm'][:2],
             'extreme_xy_mm':np.maximum(center[:2]-bounds[0,:2],bounds[1,:2]-center[:2]).tolist(),
             'outer_rectangle':rectangle_torsion(width,depth),
             'inner_rectangle':rectangle_torsion(width-2*radius,depth-2*radius),
             'longitudinal_fillet_radius_mm':radius}
    return cad,section,{'manifest':str(path.resolve()),'manifest_sha256':initial,
                        'generator':str(generator),'generator_sha256':cad['generator_sha256'],'verified_artifact_sha256':files}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,default=ROOT/'design/wrist_camera_bracket_v4/manifest.json')
    p.add_argument('--payload-g',type=float,default=100.)
    p.add_argument('--youngs-moduli-mpa',type=float,nargs='+',default=[1000.,500.])
    p.add_argument('--g-levels',type=float,nargs='+',default=[1.,3.,5.])
    p.add_argument('--poisson-ratio',type=float,default=.35)
    p.add_argument('--stress-factor',type=float,default=2.)
    p.add_argument('--stress-comparison-mpa',type=float,default=5.)
    p.add_argument('--camera-com-mm',type=float,nargs=3)
    p.add_argument('--density-kg-m3',type=float)
    p.add_argument('--format',choices=['json','markdown'],default='json')
    args=p.parse_args();self_check()
    numbers=[args.payload_g,*args.youngs_moduli_mpa,*args.g_levels,args.poisson_ratio,args.stress_factor,args.stress_comparison_mpa]
    numbers+=args.camera_com_mm or []
    if args.density_kg_m3 is not None:numbers.append(args.density_kg_m3)
    if not np.isfinite(numbers).all() or min([args.payload_g,*args.youngs_moduli_mpa,*args.g_levels,args.stress_comparison_mpa])<=0 or not -1<args.poisson_ratio<.5 or args.stress_factor<1 or (args.density_kg_m3 is not None and args.density_kg_m3<=0):
        p.error('Material/load inputs must be finite and physically valid assumptions')
    cad,section,provenance=load_side_snapshot(args.manifest)
    l=float(cad['parameters']['length_mm'])
    is_v4=cad['schema']=='wrist_camera_bracket_side_bend_v4'
    tilt=float(cad['parameters']['plate_plane_rotation_deg'] if is_v4 else cad['parameters']['plate_tilt_deg'])
    rows=[]
    for g in args.g_levels:
        for e in args.youngs_moduli_mpa:rows.extend(case(cad,section,l,tilt,g,e,args))
    if sha(args.manifest)!=provenance['manifest_sha256'] or sha(Path(provenance['generator']))!=provenance['generator_sha256']:
        raise ValueError('Side-mount source changed during calculation')
    report={'schema':'side_mount_bracket_stem_plate_strength_supplement_v4' if is_v4 else 'side_mount_bracket_stem_plate_strength_supplement_v3',
            'status':'conditional_stem_plate_screen_only_no_saddle_or_hardware_certification',
            'source':provenance,'parameters':cad['parameters'],'section':section,
            'user_plate_to_stem_angle_deg':cad['parameters'].get('plate_to_stem_angle_deg'),
            'actual_CAD_plane_rotation_used_deg':tilt,
            'whole_mass_g_at_assumed_density':cad['whole']['mass_kg']*1000,
            'part_mass_g_at_assumed_density':{k:v['mass_kg']*1000 for k,v in cad['parts'].items()},
            'cases':rows,'limitations':[
                'Actual source-version plate/gusset mass and COM; the beam calculation uses CAD plane rotation, never the public board-to-stem angle as Rx.',
                'Load directions refer to bracket-local axes after side mounting, not fixed world axes.',
                'Stem ideal clamp remains at local Z=35.8. The narrower side saddle, 60-degree fastener pair and changed contact area are outside this model.',
                'E/nu, payload/COM and stress multipliers are assumptions. No FEM, threads/preload, contact, print-layer, creep, impact or fatigue certification.',
                'Bending/axial plus conservative-inscribed-rectangle torsion surrogate; direct transverse shear and constrained warping are omitted.'
            ]}
    if args.format=='json':print(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False))
    else:
        print('| Load | g | assumed E MPa | normal bound MPa | torsion N m | displacement mm |')
        print('|---|---:|---:|---:|---:|---:|')
        for r in rows:print(f"| {r['load_direction']} | {r['g_multiple']:g} | {r['E_assumed_MPa']:g} | {r['normal_stress_bound_MPa']:.3f} | {r['torsion_about_Z_N_m']:.4f} | {r['camera_total_surrogate_displacement_norm_mm']:.3f} |")
    return 0


if __name__=='__main__':raise SystemExit(main())
