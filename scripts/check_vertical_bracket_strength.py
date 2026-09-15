#!/usr/bin/env python3
"""V2 +Z solid-stem screening: two-axis bending, torsion and eccentric compression.

This is a new spatial cantilever model, not the V1 +Y horizontal-arm model.
Requires the actual solid-rectangle V2 CAD manifest; never falls back to V1.
No FEM, bolt/contact, fatigue, creep or measured print-material certification.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


ROOT=Path(__file__).resolve().parents[1]
GENERATOR=ROOT/'scripts/design_wrist_camera_bracket_v2.py'
G0=9.80665


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rectangle_torsion(width,depth):
    """Saint-Venant rectangular J, full side lengths in mm, 100 odd terms.

    The shear coefficient is for the ideal sharp-corner rectangular surrogate;
    it is not a proven peak stress for the full bracket or its connections.
    """
    a,b=sorted((float(width),float(depth)),reverse=True)
    if b<=0:
        raise ValueError('Rectangle dimensions must be positive')
    odd=range(1,200,2)
    series=sum(math.tanh(n*math.pi*a/(2*b))/n**5 for n in odd)
    j=a*b**3/3*(1-192*b/(math.pi**5*a)*series)
    sech_sum=0.
    for n in odd:
        exponent=n*math.pi*a/(2*b)
        sech=2*math.exp(-exponent)/(1+math.exp(-2*exponent))
        sech_sum+=sech/n**2
    stress_per_torque=b/j*(1-8/math.pi**2*sech_sum)
    return {'width_mm':a,'depth_mm':b,'J_mm4':j,'max_shear_per_torque_MPa_per_N_mm':stress_per_torque}


def rounded_rectangle_section(width,depth,radius):
    if radius<0 or 2*radius>=min(width,depth):
        raise ValueError('Invalid rectangular corner radius')
    removed_area=radius**2*(1-math.pi/4)
    removed_second=radius**4*(1/3-math.pi/16)
    def corner_loss(center):
        return center**2*removed_area+center*radius**3/3+removed_second
    return {'area_mm2':width*depth-4*removed_area,
            'Ix_mm4':width*depth**3/12-4*corner_loss(depth/2-radius),
            'Iy_mm4':depth*width**3/12-4*corner_loss(width/2-radius)}


def load_v2(path):
    original=path.read_bytes()
    cad=json.loads(original)
    if cad.get('schema')!='wrist_camera_bracket_radial_stem_v2':
        raise ValueError('Expected the V2 radial-stem CAD manifest; no V1 substitution is allowed')
    geometry=cad['strength_geometry']
    if geometry.get('section_type')!='solid_rectangle_with_vertical_edge_fillets':
        raise ValueError('This review requires the final solid 24x12 stem, not an earlier U-section trial')
    if geometry['stem_axis']!=[0,0,1] or geometry.get('rib_width_mm',0)!=0:
        raise ValueError('Expected a solid prismatic stem along +Z')
    if digest(GENERATOR)!=cad['generator_sha256']:
        raise ValueError('V2 generator changed after this CAD snapshot')
    files=dict(cad['files'])
    for part in cad['parts'].values():
        files.update(part['files'])
    for relative,expected in files.items():
        if digest(path.parent/relative)!=expected:
            raise ValueError(f'CAD artifact hash mismatch: {relative}')
    if cad['camera_mount']['camera_geometry_included'] is not False:
        raise ValueError('The camera must remain separate from the manufactured bracket')
    stem=cad['parts']['stem']
    length=float(cad['parameters']['length_mm'])
    z0=float(geometry['stem_start_z_mm'])
    rho=float(stem['density_kg_m3'])
    center=np.asarray(stem['center_of_mass_mm'],dtype=float)
    local=stem['body_local_properties']
    bounds=np.asarray(local['bounds_mm'],dtype=float)
    if abs(center[2]-(z0+length/2))>1e-5 or abs(bounds[0,2])>1e-3 or abs(bounds[1,2]-length)>1e-3:
        raise ValueError('Stem is not consistent with the required constant +Z extrusion')
    width=float(geometry['stem_width_mm']);depth=float(geometry['stem_depth_mm'])
    area=float(stem['volume_mm3'])/length
    if area<.9*width*depth or not np.allclose([width,depth],[24,12]):
        raise ValueError('Actual stem area/dimensions do not match the selected solid-section design')
    inertia=np.asarray(stem['inertia_at_com_kg_m2'],dtype=float)
    bending=inertia[:2,:2]/(rho*1e-15*length)
    bending-=np.eye(2)*area*length**2/12
    if not np.isfinite(bending).all() or np.linalg.eigvalsh(bending).min()<=0:
        raise ValueError('Invalid CAD-derived transverse bending inertia')
    local_center=np.asarray(local['center_of_mass_mm'])
    extreme=np.maximum(local_center[:2]-bounds[0,:2],bounds[1,:2]-local_center[:2])
    fillet=geometry['fillets']['stem_vertical_edges']
    radius=float(fillet['radius_mm']) if fillet['applied'] else 0.
    analytical=rounded_rectangle_section(width,depth,radius)
    if not np.allclose([area,bending[0,0],bending[1,1]],list(analytical.values()),rtol=1e-6,atol=1e-7):
        raise ValueError('Actual CAD section differs from the independent rounded-rectangle area/inertia calculation')
    outer=rectangle_torsion(width,depth)
    inner=rectangle_torsion(width-2*radius,depth-2*radius)
    section={'area_mm2':area,'bending_inertia_matrix_xy_mm4':bending.tolist(),
             'Ix_mm4':float(bending[0,0]),'Iy_mm4':float(bending[1,1]),
             'centroid_xy_root_mm':center[:2].tolist(),'extreme_xy_mm':extreme.tolist(),
             'longitudinal_fillet_radius_mm':radius,'independent_rounded_rectangle_check':analytical,'outer_rectangle':outer,'inner_rectangle':inner,
             'torsion_model':'Use a centered fully-inscribed rectangle (each side reduced by 2r) for conservative ideal-stem torsional compliance; shear stress is only a rectangular surrogate.'}
    return cad,section,{'manifest':str(path.resolve()),'manifest_sha256':hashlib.sha256(original).hexdigest(),
                        'generator_sha256':cad['generator_sha256'],'verified_artifact_sha256':files}


def beam_response(length,youngs,poisson,section,point_loads,line_load,camera_offset):
    """Uniform +Z beam, rigid tip offsets, signed loads in root XYZ.

    point_loads are (tip-relative position mm, force N). Distributed self load
    acts along the centroidal stem axis. Returns linear beam components plus
    a conservative-rectangle torsion component, not whole-bracket compliance.
    """
    axis=np.array([0.,0.,1.]);q=np.asarray(line_load,dtype=float)
    force=sum((np.asarray(f,dtype=float) for _,f in point_loads),start=np.zeros(3))
    end_moment=sum((np.cross(np.asarray(e,dtype=float),np.asarray(f,dtype=float)) for e,f in point_loads),start=np.zeros(3))
    c=np.cross(axis,force);d=np.cross(axis,q);l=length
    root_moment=end_moment+c*l+d*l*l/2
    root_force=force+q*l
    inertia=np.asarray(section['bending_inertia_matrix_xy_mm4'],dtype=float)
    integrated=end_moment*l+c*l*l/2+d*l**3/6
    weighted=end_moment*l*l/2+c*l**3/3+d*l**4/8
    bending_rotation=np.linalg.solve(inertia,integrated[:2])/youngs
    weighted_rotation=np.linalg.solve(inertia,weighted[:2])/youngs
    area=section['area_mm2'];shear_modulus=youngs/(2*(1+poisson))
    torsion=float(end_moment[2]*l/(shear_modulus*section['inner_rectangle']['J_mm4']))
    tip_displacement=np.array([weighted_rotation[1],-weighted_rotation[0],(force[2]*l+q[2]*l*l/2)/(youngs*area)])
    rotation=np.array([*bending_rotation,torsion])
    camera_offset=np.asarray(camera_offset,dtype=float)
    bending_axial=tip_displacement+np.cross(np.r_[bending_rotation,0.],camera_offset)
    torsion_displacement=np.cross([0.,0.,torsion],camera_offset)
    total=bending_axial+torsion_displacement
    stresses=[]
    cx,cy=section['extreme_xy_mm']
    for label,moment,axial in [('root',root_moment,root_force[2]),('tip',end_moment,force[2])]:
        curvature=np.linalg.solve(inertia,moment[:2])/youngs
        bending_bound=youngs*(abs(curvature[0])*cy+abs(curvature[1])*cx)
        stresses.append({'section':label,'axial_stress_MPa':abs(float(axial))/area,
                         'bending_stress_bound_MPa':float(bending_bound),
                         'normal_stress_bound_MPa':abs(float(axial))/area+float(bending_bound)})
    peak=max(stresses,key=lambda s:s['normal_stress_bound_MPa'])
    shear=abs(float(end_moment[2]))*section['inner_rectangle']['max_shear_per_torque_MPa_per_N_mm']
    vm=math.sqrt(peak['normal_stress_bound_MPa']**2+3*shear**2)
    pcr=math.pi**2*youngs*float(np.linalg.eigvalsh(inertia).min())/(4*l*l)
    return {'root_force_N':root_force.tolist(),'root_moment_N_m':(root_moment*.001).tolist(),
            'torsion_about_Z_N_m':float(end_moment[2])*.001,'G_assumed_MPa':shear_modulus,
            'normal_stress_bound_MPa':peak['normal_stress_bound_MPa'],'peak_normal_stress_at':peak['section'],
            'normal_stress_at_root_and_tip':stresses,'torsional_shear_surrogate_MPa':shear,
            'von_mises_surrogate_MPa':vm,'tip_translation_bending_and_axial_mm':tip_displacement.tolist(),
            'tip_rotation_xyz_rad':rotation.tolist(),'camera_bending_axial_component_mm':bending_axial.tolist(),
            'camera_torsion_component_mm':torsion_displacement.tolist(),'camera_total_surrogate_displacement_mm':total.tolist(),
            'camera_total_surrogate_displacement_norm_mm':float(np.linalg.norm(total)),
            'camera_displacement_to_stem_length_ratio':float(np.linalg.norm(total)/l),
            'ideal_fixed_free_Euler_load_N':pcr,'axial_compression_to_Euler_ratio':max(0.,-float(root_force[2]))/pcr,
            'small_displacement_5percent_screen_exceeded':bool(np.linalg.norm(total)/l>.05)}


def case(cad,section,length,tilt,g_level,youngs,args):
    angle=math.radians(tilt);c,s=math.cos(angle),math.sin(angle)
    rotation=np.array([[1,0,0],[0,c,-s],[0,s,c]])
    camera_local=np.array(args.camera_com_mm if args.camera_com_mm is not None else cad['strength_geometry']['camera_com_plate_local_mm'],dtype=float)
    plate=cad['parts']['plate']['body_local_properties']
    plate_local=np.asarray(plate['center_of_mass_mm'],dtype=float)
    pivot=np.asarray(cad['plate_frame']['xyz_mm'],dtype=float).copy()
    z0=float(cad['strength_geometry']['stem_start_z_mm']);pivot[2]=z0+length
    neutral_tip=np.r_[section['centroid_xy_root_mm'],z0+length]
    camera_offset=pivot+rotation@camera_local-neutral_tip
    plate_offset=pivot+rotation@plate_local-neutral_tip
    rho=float(cad['parameters']['density_kg_m3']) if args.density_kg_m3 is None else args.density_kg_m3
    plate_mass=float(plate['volume_mm3'])*rho*1e-9
    rows=[]
    for label,direction in [('X_transverse',[1,0,0]),('Y_transverse',[0,1,0]),('minus_Z_compression',[0,0,-1])]:
        acceleration=np.array(direction,dtype=float)*(G0*g_level)
        loads=[(camera_offset,acceleration*args.payload_g*.001),(plate_offset,acceleration*plate_mass)]
        line_load=acceleration*(rho*section['area_mm2']*1e-9)
        response=beam_response(length,youngs,args.poisson_ratio,section,loads,line_load,camera_offset)
        response.update(length_mm=length,tilt_deg=tilt,load_direction=label,g_multiple=g_level,
                        E_assumed_MPa=youngs,nu_assumed=args.poisson_ratio,density_assumed_kg_m3=rho,
                        camera_and_cable_mass_assumed_g=args.payload_g,plate_gusset_CAD_mass_g=plate_mass*1000,
                        stem_CAD_section_mass_g=rho*section['area_mm2']*length*1e-6,
                        camera_COM_plate_local_assumed_mm=camera_local.tolist(),camera_offset_from_neutral_tip_mm=camera_offset.tolist(),
                        plate_COM_plate_local_CAD_mm=plate_local.tolist(),plate_offset_from_neutral_tip_mm=plate_offset.tolist(),
                        illustrative_stress_factor=args.stress_factor,
                        illustrative_factored_stress_MPa=response['von_mises_surrogate_MPa']*args.stress_factor,
                        assumed_stress_comparison_MPa=args.stress_comparison_mpa,
                        illustrative_stress_comparison_ratio=response['von_mises_surrogate_MPa']*args.stress_factor/args.stress_comparison_mpa,
                        status='conditional_screening_only_not_strength_approval')
        rows.append(response)
    return rows


def self_check():
    square=rectangle_torsion(1,1)['J_mm4']
    if not math.isclose(square,.140577014955,rel_tol=1e-8):
        raise AssertionError('Rectangular torsion square benchmark failed')
    if not math.isclose(rectangle_torsion(48,24)['J_mm4'],16*rectangle_torsion(24,12)['J_mm4'],rel_tol=1e-12):
        raise AssertionError('Torsion dimensional scaling failed')
    section={'area_mm2':24*12,'bending_inertia_matrix_xy_mm4':[[24*12**3/12,0],[0,12*24**3/12]],
             'extreme_xy_mm':[12,6],'inner_rectangle':rectangle_torsion(24,12)}
    l=80.;e=1000.
    for force,coordinate,inertia in [([1,0,0],0,section['bending_inertia_matrix_xy_mm4'][1][1]),([0,1,0],1,section['bending_inertia_matrix_xy_mm4'][0][0])]:
        row=beam_response(l,e,.35,section,[([0,0,0],force)],[0,0,0],[0,0,0])
        if not math.isclose(row['tip_translation_bending_and_axial_mm'][coordinate],l**3/(3*e*inertia),rel_tol=1e-12):
            raise AssertionError('Transverse +Z cantilever limiting case failed')
    row=beam_response(l,e,.35,section,[([0,0,0],[0,0,-1])],[0,0,0],[0,0,0])
    if not math.isclose(row['tip_translation_bending_and_axial_mm'][2],-l/(e*section['area_mm2']),rel_tol=1e-12):
        raise AssertionError('Axial compression limiting case failed')
    row=beam_response(l,e,.35,section,[([0,10,0],[1,0,0])],[0,0,0],[0,10,0])
    expected=-10*l/((e/2.7)*section['inner_rectangle']['J_mm4'])
    if not math.isclose(row['tip_rotation_xyz_rad'][2],expected,rel_tol=1e-12) or row['camera_torsion_component_mm'][0]<=0:
        raise AssertionError('Eccentric force/twist sign check failed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,default=ROOT/'design/wrist_camera_bracket_v2/manifest.json')
    p.add_argument('--lengths-mm',type=float,nargs='+',default=[80.,150.])
    p.add_argument('--tilts-deg',type=float,nargs='+',default=None)
    p.add_argument('--g-levels',type=float,nargs='+',default=[1.,3.,5.])
    p.add_argument('--youngs-moduli-mpa',type=float,nargs='+',default=[1000.,500.])
    p.add_argument('--poisson-ratio',type=float,default=.35)
    p.add_argument('--payload-g',type=float,default=100.)
    p.add_argument('--camera-com-mm',type=float,nargs=3)
    p.add_argument('--density-kg-m3',type=float)
    p.add_argument('--stress-factor',type=float,default=2.)
    p.add_argument('--stress-comparison-mpa',type=float,default=5.)
    p.add_argument('--format',choices=['json','markdown'],default='json')
    p.add_argument('--self-check',action='store_true',help='Analytic checks only; reads no CAD or V1 file')
    args=p.parse_args();self_check()
    if args.self_check:
        print('Passed: rectangle torsion series, dimensional scaling, X/Y bending, axial shortening, eccentric twist sign. No CAD loaded.')
        return 0
    numeric=[*args.lengths_mm,*args.g_levels,*args.youngs_moduli_mpa,args.poisson_ratio,args.payload_g,args.stress_factor,args.stress_comparison_mpa]
    numeric+=args.tilts_deg or [];numeric+=args.camera_com_mm or []
    if args.density_kg_m3 is not None:numeric.append(args.density_kg_m3)
    if not np.isfinite(numeric).all() or min(args.lengths_mm+args.g_levels+args.youngs_moduli_mpa)<=0 or not -1<args.poisson_ratio<.5 or args.payload_g<=0 or args.stress_factor<1 or args.stress_comparison_mpa<=0 or (args.density_kg_m3 is not None and args.density_kg_m3<=0):
        p.error('Require finite positive dimensions/load/E/density, -1<nu<0.5, and stress factor>=1')
    cad,section,provenance=load_v2(args.manifest)
    tilts=args.tilts_deg or [float(cad['parameters']['plate_tilt_deg'])]
    if any(not 0<=value<=75 for value in tilts) or any(not 20<=value<=150 for value in args.lengths_mm):
        p.error('V2 CAD parameter range is L=20..150 mm, angle=0..75 degrees')
    rows=[]
    for length in args.lengths_mm:
        for tilt in tilts:
            for g in args.g_levels:
                for e in args.youngs_moduli_mpa:
                    rows.extend(case(cad,section,length,tilt,g,e,args))
    if digest(args.manifest)!=provenance['manifest_sha256'] or digest(GENERATOR)!=provenance['generator_sha256']:
        raise ValueError('V2 CAD provenance changed during calculation')
    report={'schema':'vertical_wrist_camera_bracket_strength_screen_v2','status':'not_a_structural_strength_certificate',
            'source':provenance,'section':section,'source_CAD_parameters':cad['parameters'],
            'cases':rows,'analytic_self_checks':'passed',
            'assumptions':[
                'Long axis +Z, ideal clamp at z=35.8; no V1 +Y arm formula is used.',
                'Separate +X, +Y and -Z acceleration cases; g multipliers apply to camera/cable, plate/gusset and stem self weight.',
                'Camera/cable COM is an explicit design assumption; stem/plate volume, COM and section inertia come from hash-verified V2 CAD.',
                'Full solid homogeneous isotropic material with E=500/1000 MPa and nu=.35 by default; print direction/infill/creep are not measured.',
                'Bending uses both transverse CAD inertia axes; -Z load includes axial compression and eccentric bending.',
                'Torsion uses the smaller fully-inscribed rectangular Saint-Venant section for an ideal-stem compliance screen; its stress is a surrogate, not a full bracket peak.',
                'Direct transverse shear stress/deformation and restrained-warping stresses are not included in the beam/torsion surrogate.',
                'When angle differs from the source CAD, hold source plate/gusset mass and local COM fixed and rotate it; alternate-angle gusset geometry is not rebuilt here.',
                'Euler compression value assumes a perfectly clamped-free uniform member (effective length factor 2); it is not full-bracket buckling validation.',
                'No FEM, root saddle/pad/contact/bolt compliance, thread proof, fatigue, impact, print-layer failure, or measured hardware load rating.'
            ]}
    if args.format=='json':
        print(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False))
    else:
        print('| L mm | tilt deg | load | g | E MPa assumed | normal bound MPa | Tz N m | shear surrogate MPa | camera displacement mm |')
        print('|---:|---:|---|---:|---:|---:|---:|---:|---:|')
        for row in rows:
            print(f"| {row['length_mm']:g} | {row['tilt_deg']:g} | {row['load_direction']} | {row['g_multiple']:g} | {row['E_assumed_MPa']:g} | {row['normal_stress_bound_MPa']:.3f} | {row['torsion_about_Z_N_m']:.4f} | {row['torsional_shear_surrogate_MPa']:.3f} | {row['camera_total_surrogate_displacement_norm_mm']:.3f} |")
    return 0


if __name__=='__main__':
    raise SystemExit(main())
