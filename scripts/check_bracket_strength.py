#!/usr/bin/env python3
"""First-order wrist-camera cantilever estimates; NOT a strength certificate.

Units internally: N, mm, MPa (= N/mm^2), kg. The ideal U-section is perfectly
bonded and clamped at Y=0. Material properties, payload COM and tip structure
mass are explicit assumptions. No FEA, bolt/contact, creep or layer model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
GRAVITY_M_S2 = 9.80665


def u_section(width_mm, web_mm, rib_width_mm, rib_height_mm):
    if width_mm <= 0 or web_mm <= 0 or rib_height_mm < 0 or not 0 <= 2*rib_width_mm <= width_mm:
        raise ValueError("Invalid U-section dimensions")
    web_area = width_mm*web_mm
    rib_area = rib_width_mm*rib_height_mm
    area = web_area+2*rib_area
    centroid_z = (web_area*web_mm/2 + 2*rib_area*(web_mm+rib_height_mm/2))/area
    ix = width_mm*web_mm**3/12 + web_area*(web_mm/2-centroid_z)**2
    ix += 2*(rib_width_mm*rib_height_mm**3/12 + rib_area*(web_mm+rib_height_mm/2-centroid_z)**2)
    iz = web_mm*width_mm**3/12 + 2*(rib_height_mm*rib_width_mm**3/12 + rib_area*(width_mm/2-rib_width_mm/2)**2)
    extreme_z = max(centroid_z, web_mm+rib_height_mm-centroid_z)
    return {"area_mm2":area,"centroid_z_above_web_bottom_mm":centroid_z,"I_about_X_mm4":ix,"I_about_Z_mm4":iz,"extreme_z_mm":extreme_z,"section_modulus_about_X_mm3":ix/extreme_z}


def cad_properties(manifest_path, args):
    """Explicit opt-in to final CAD mass and prismatic-arm section data."""
    original=manifest_path.read_bytes()
    manifest=json.loads(original)
    if manifest.get('schema')!='wrist_camera_bracket_new_design_v1':
        raise ValueError('Unsupported CAD manifest')
    expected=dict(manifest['files'])
    for part in manifest['parts'].values():
        expected.update(part['files'])
    for relative,expected_hash in expected.items():
        path=manifest_path.parent/relative
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected_hash:
            raise ValueError(f'CAD artifact hash mismatch: {relative}')
    if hashlib.sha256((ROOT/'scripts/design_wrist_camera_bracket.py').read_bytes()).hexdigest()!=manifest['generator_sha256']:
        raise ValueError('CAD generator changed after this manifest was produced')
    if manifest['camera_mount']['camera_geometry_included'] is not False:
        raise ValueError('Manufacturing bracket must not contain the camera solid')
    g=manifest['strength_geometry']
    for attr,key in (('beam_width_mm','beam_width_mm'),('web_thickness_mm','web_thickness_mm'),('rib_width_mm','rib_width_mm'),('rib_added_height_mm','rib_added_height_mm')):
        if not math.isclose(getattr(args,attr),float(g[key]),rel_tol=0,abs_tol=1e-9):
            raise ValueError(f'Explicit section parameter {attr} differs from final CAD')
    if g['rib_end_gap_mm']!=0:
        raise ValueError('Constant-section model requires full-length ribs')
    density=float(manifest['parameters']['density_kg_m3'])
    length=float(manifest['parameters']['length_mm'])
    arm=manifest['parts']['support_arm']['body_local_properties']
    bounds=arm['bounds_mm']; center=arm['center_of_mass_mm']
    if abs(center[1]-length/2)>1e-5 or abs(bounds[0][1])>1e-3 or abs(bounds[1][1]-length)>1e-3:
        raise ValueError('CAD arm is inconsistent with the prismatic span assumption')
    area=arm['volume_mm3']/length
    inertia=arm['inertia_at_com_kg_m2']
    ix=inertia[0][0]/(density*1e-15*length)-area*length**2/12
    iz=inertia[2][2]/(density*1e-15*length)-area*length**2/12
    c=max(center[2]-bounds[0][2],bounds[1][2]-center[2])
    if min(area,ix,iz,c)<=0:
        raise ValueError('Nonphysical CAD-derived prismatic section')
    plate=manifest['parts']['camera_plate']['body_local_properties']
    args.tip_structure_g=plate['mass_kg']*1000
    args.structure_com_y_mm=plate['center_of_mass_mm'][1]
    args.structure_com_z_mm=plate['center_of_mass_mm'][2]
    args.density_kg_m3=density
    section={'area_mm2':area,'centroid_z_above_web_bottom_mm':center[2]-bounds[0][2],
             'I_about_X_mm4':ix,'I_about_Z_mm4':iz,'extreme_z_mm':c,'section_modulus_about_X_mm3':ix/c,
             'source':'Final CAD constant-Y-extrusion mass properties, including longitudinal fillets'}
    provenance={'path':str(manifest_path.resolve()),'sha256':hashlib.sha256(original).hexdigest(),
                'generator_sha256':manifest['generator_sha256'],'verified_artifact_count':len(expected),
                'whole_mass_kg_at_assumed_density':manifest['whole']['mass_kg'],
                'tip_structure_mass_g':args.tip_structure_g,'tip_structure_COM_plate_mm':plate['center_of_mass_mm'],
                'source_CAD_length_mm':length,'source_CAD_tilt_deg':manifest['parameters']['plate_tilt_deg'],
                'camera_geometry_included':False,
                'other_tilt_assumption':f"For angle sensitivity only, hold the final {manifest['parameters']['plate_tilt_deg']:g}-degree plate/gusset mass and plate-local COM fixed while rotating it; alternate-angle gusset masses are not regenerated here.",
                'camera_COM_assumption_override_mm':[0,args.camera_com_y_mm,args.camera_com_z_mm]}
    return section,provenance


def estimate(length_mm, tilt_deg, acceleration_g, youngs_mpa, section, *, density_kg_m3,
             payload_g, tip_structure_g, com_y_mm, com_z_mm, structure_com_y_mm,
             structure_com_z_mm, stress_factor, assumed_allowable_mpa):
    angle = math.radians(tilt_deg)
    camera_overhang = com_y_mm*math.cos(angle)-com_z_mm*math.sin(angle)
    structure_overhang = structure_com_y_mm*math.cos(angle)-structure_com_z_mm*math.sin(angle)
    if length_mm <= 0 or youngs_mpa <= 0 or acceleration_g <= 0 or camera_overhang < 0 or structure_overhang < 0:
        raise ValueError("This model requires positive span/E/load and nonnegative projected overhangs")
    g = acceleration_g*GRAVITY_M_S2
    payload_force = payload_g*.001*g
    structure_force = tip_structure_g*.001*g
    line_weight = density_kg_m3*1e-9*section["area_mm2"]*g
    l, e, s = length_mm, camera_overhang, structure_overhang
    ei = youngs_mpa*section["I_about_X_mm4"]
    root_moment = payload_force*(l+e)+structure_force*(l+s)+line_weight*l*l/2
    tip_slope = (payload_force*(l*l/2+e*l)+structure_force*(l*l/2+s*l)+line_weight*l**3/6)/ei
    tip_deflection = (payload_force*(l**3/3+e*l*l/2)+structure_force*(l**3/3+s*l*l/2)+line_weight*l**4/8)/ei
    camera_deflection = tip_deflection+e*tip_slope
    nominal_stress = root_moment*section["extreme_z_mm"]/section["I_about_X_mm4"]
    corrected_stress = stress_factor*nominal_stress
    return {
        "length_mm":l,"tilt_deg":tilt_deg,"acceleration_multiple_g":acceleration_g,"youngs_modulus_assumed_MPa":youngs_mpa,
        "camera_COM_projected_overhang_mm":e,"tip_structure_COM_projected_overhang_mm":s,
        "camera_COM_projected_radius_from_root_mm":l+e,
        "beam_mass_assumed_g":density_kg_m3*1e-6*section["area_mm2"]*l,
        "payload_mass_assumed_g":payload_g,"tip_structure_mass_assumed_g":tip_structure_g,
        "root_shear_N":payload_force+structure_force+line_weight*l,
        "root_bending_moment_N_m":root_moment*.001,
        "nominal_root_bending_stress_MPa":nominal_stress,
        "illustrative_Kt_root_stress_MPa":corrected_stress,
        "illustrative_Kt":stress_factor,
        "assumed_allowable_stress_MPa":assumed_allowable_mpa,
        "illustrative_stress_to_assumed_allowable_ratio":corrected_stress/assumed_allowable_mpa,
        "arm_tip_bending_deflection_mm":tip_deflection,
        "camera_COM_bending_deflection_mm":camera_deflection,
        "tip_rotation_rad":tip_slope,
        "camera_deflection_to_projected_span_ratio":camera_deflection/(l+e),
        "small_deflection_5percent_screen_exceeded":camera_deflection/(l+e)>.05,
        "status":"conditional_first_order_estimate_not_structural_approval",
    }


def check_artifacts(stl_paths, step_paths):
    """Read final manufacturing files; checks geometry, not load capacity."""
    result = {"stl":[],"step":[]}
    if stl_paths:
        import numpy as np
        import trimesh
        for path in stl_paths:
            mesh = trimesh.load_mesh(path,process=False)
            mesh.merge_vertices(digits_vertex=8)
            record = {"path":str(path.resolve()),"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
                      "finite_vertices":bool(np.isfinite(mesh.vertices).all()),"watertight":bool(mesh.is_watertight),
                      "winding_consistent":bool(mesh.is_winding_consistent),"positive_volume":bool(mesh.is_volume),
                      "connected_components":int(mesh.body_count),"volume_in_file_units_cubed":float(mesh.volume),
                      "bounds_in_file_units":mesh.bounds.tolist()}
            record["single_solid_geometry_valid"] = record["finite_vertices"] and mesh.is_volume and record["connected_components"]==1
            result["stl"].append(record)
    if step_paths:
        cad_python = ROOT/".cad-venv/bin/python"
        program = """import cadquery as cq,json,sys
for filename in sys.argv[1:]:
 shape=cq.importers.importStep(filename).val()
 solids=shape.Solids()
 print(json.dumps({'path':filename,'valid_brep':shape.isValid(),'solid_count':len(solids),'volume_mm3':shape.Volume(),'solid_volumes_mm3':[s.Volume() for s in solids]}))
"""
        call = subprocess.run([str(cad_python),"-B","-c",program,*[str(p.resolve()) for p in step_paths]],text=True,capture_output=True,check=True)
        for line in call.stdout.splitlines():
            if line.startswith("{"):
                record=json.loads(line)
                record["sha256"]=hashlib.sha256(Path(record["path"]).read_bytes()).hexdigest()
                record["single_solid_geometry_valid"]=record["valid_brep"] and record["solid_count"]==1 and record["volume_mm3"]>0
                result["step"].append(record)
        if len(result["step"])!=len(step_paths):
            raise ValueError("STEP reader did not return every input")
    step_by_name={Path(row['path']).stem:row for row in result['step']}
    for row in result['stl']:
        stem=Path(row['path']).stem
        name=stem[:-3] if stem.endswith('_mm') else stem
        if name in step_by_name:
            exact=step_by_name[name]['volume_mm3']
            row['relative_volume_difference_from_STEP']=abs(row['volume_in_file_units_cubed']-exact)/exact
            row['single_solid_geometry_valid'] &= row['relative_volume_difference_from_STEP']<.005
    return result


def self_check():
    """Analytic limiting cases: point load and uniform self weight."""
    section=u_section(24,6,0,0);l=80;e=1000.;mass=100.
    row=estimate(l,0,1,e,section,density_kg_m3=0,payload_g=mass,tip_structure_g=0,com_y_mm=0,com_z_mm=0,structure_com_y_mm=0,structure_com_z_mm=0,stress_factor=1,assumed_allowable_mpa=5)
    force=mass*.001*GRAVITY_M_S2
    expected=force*l**3/(3*e*section['I_about_X_mm4'])
    if not math.isclose(row['camera_COM_bending_deflection_mm'],expected,rel_tol=1e-12):
        raise AssertionError('Point-load cantilever limiting case failed')
    row=estimate(l,0,1,e,section,density_kg_m3=1250,payload_g=0,tip_structure_g=0,com_y_mm=0,com_z_mm=0,structure_com_y_mm=0,structure_com_z_mm=0,stress_factor=1,assumed_allowable_mpa=5)
    q=1250*1e-9*section['area_mm2']*GRAVITY_M_S2
    expected=q*l**4/(8*e*section['I_about_X_mm4'])
    if not math.isclose(row['camera_COM_bending_deflection_mm'],expected,rel_tol=1e-12):
        raise AssertionError('Uniform self-weight limiting case failed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lengths-mm',nargs='+',type=float,default=[80.,150.])
    p.add_argument('--tilts-deg',nargs='+',type=float,default=[20.,40.,70.])
    p.add_argument('--g-levels',nargs='+',type=float,default=[1.,3.,5.])
    p.add_argument('--youngs-moduli-mpa',nargs='+',type=float,default=[1000.,500.])
    p.add_argument('--allowable-stress-mpa',type=float,default=5.)
    p.add_argument('--stress-factor',type=float,default=2.,help='Illustrative concentration multiplier, not measured Kt')
    p.add_argument('--payload-g',type=float,default=100.,help='Camera plus cable work-point assumption; not measured')
    p.add_argument('--tip-structure-g',type=float,default=20.,help='Temporary plate/gusset self-mass assumption; replace using final CAD')
    p.add_argument('--density-kg-m3',type=float,default=1250.)
    p.add_argument('--beam-width-mm',type=float,default=24.)
    p.add_argument('--web-thickness-mm',type=float,default=6.)
    p.add_argument('--rib-width-mm',type=float,default=3.)
    p.add_argument('--rib-added-height-mm',type=float,default=8.)
    p.add_argument('--camera-com-y-mm',type=float,default=32.)
    p.add_argument('--camera-com-z-mm',type=float,default=-13.5)
    p.add_argument('--structure-com-y-mm',type=float,default=32.)
    p.add_argument('--structure-com-z-mm',type=float,default=0.)
    p.add_argument('--manifest',type=Path,help='Attach provenance for the final CAD snapshot; does not silently override explicit dimensions')
    p.add_argument('--use-cad-properties',action='store_true',help='With --manifest, explicitly replace tip mass/COM and use the final rounded prismatic arm section')
    p.add_argument('--stl',type=Path,action='append',default=[])
    p.add_argument('--step',type=Path,action='append',default=[])
    p.add_argument('--format',choices=('json','markdown'),default='json')
    args=p.parse_args()
    numbers=[*args.lengths_mm,*args.tilts_deg,*args.g_levels,*args.youngs_moduli_mpa,args.allowable_stress_mpa,args.stress_factor,args.payload_g,args.tip_structure_g,args.density_kg_m3,args.beam_width_mm,args.web_thickness_mm,args.rib_width_mm,args.rib_added_height_mm,args.camera_com_y_mm,args.camera_com_z_mm,args.structure_com_y_mm,args.structure_com_z_mm]
    if not all(math.isfinite(value) for value in numbers) or args.allowable_stress_mpa<=0 or args.stress_factor<1 or min(args.payload_g,args.tip_structure_g,args.density_kg_m3)<0:
        p.error('Inputs must be finite and physically nonnegative; E/allowable/load/span positive and assumed Kt >= 1')
    self_check()
    section=u_section(args.beam_width_mm,args.web_thickness_mm,args.rib_width_mm,args.rib_added_height_mm)
    provenance=None
    if args.use_cad_properties:
        if args.manifest is None:p.error('--use-cad-properties requires --manifest')
        section,provenance=cad_properties(args.manifest,args)
    rows=[]
    for length in args.lengths_mm:
        for tilt in args.tilts_deg:
            for g in args.g_levels:
                for modulus in args.youngs_moduli_mpa:
                    rows.append(estimate(length,tilt,g,modulus,section,density_kg_m3=args.density_kg_m3,payload_g=args.payload_g,tip_structure_g=args.tip_structure_g,com_y_mm=args.camera_com_y_mm,com_z_mm=args.camera_com_z_mm,structure_com_y_mm=args.structure_com_y_mm,structure_com_z_mm=args.structure_com_z_mm,stress_factor=args.stress_factor,assumed_allowable_mpa=args.allowable_stress_mpa))
    report={"schema":"wrist_camera_bracket_first_order_strength_review_v1","status":"estimates_only_no_structural_certification","section":section,"input_parameters":{key:value for key,value in vars(args).items() if key not in ('stl','step','manifest')},"cases":rows,
            "assumptions":["Uniform full-length rectangular-U section, fully bonded isotropic material; ignores fillet/tessellation area changes.","Perfect clamp at Y=0; root saddle/stand-off and plate/gusset compliance are excluded.","Load is quasi-static normal-to-web Z acceleration; g factors apply to payload and self weight. Not simultaneous multi-axis shock.","Camera COM [0,32,-13.5] mm is assumed; transformed by plate tilt, so projected overhang is included.","100 g payload includes camera and cables; beam distributed self weight and a separate tip structure mass are added.","E=500/1000 MPa, allowable stress=5 MPa and Kt=2 are screening assumptions, not material measurements.","No FEA, layer-adhesion, infill, creep, fatigue, buckling, bolt/thread/contact, cable pull or impact analysis."],
            "analytic_limiting_case_checks":"passed point-load and uniform-self-weight cantilever formulas"}
    if args.manifest:
        report['cad_manifest']=provenance or {'path':str(args.manifest.resolve()),'sha256':hashlib.sha256(args.manifest.read_bytes()).hexdigest()}
    if args.use_cad_properties:
        report['assumptions'][0]='Constant prismatic CAD arm section including 0.8 mm longitudinal fillets; isotropic, perfectly bonded material remains an assumption.'
        report['assumptions'].append(provenance['other_tilt_assumption'])
    if args.stl or args.step:
        report['artifact_geometry_checks']=check_artifacts(args.stl,args.step)
    if args.use_cad_properties:
        if hashlib.sha256(args.manifest.read_bytes()).hexdigest()!=provenance['sha256']:
            raise ValueError('CAD manifest changed during this calculation')
        if hashlib.sha256((ROOT/'scripts/design_wrist_camera_bracket.py').read_bytes()).hexdigest()!=provenance['generator_sha256']:
            raise ValueError('CAD generator changed during this calculation')
        report['cad_snapshot_stable_during_calculation']=True
    if args.format=='json':
        print(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False))
    else:
        print('| L mm | tilt deg | g | E MPa assumed | nominal stress MPa | assumed Kt stress MPa | camera deflection mm |')
        print('|---:|---:|---:|---:|---:|---:|---:|')
        for row in rows:
            print(f"| {row['length_mm']:g} | {row['tilt_deg']:g} | {row['acceleration_multiple_g']:g} | {row['youngs_modulus_assumed_MPa']:g} | {row['nominal_root_bending_stress_MPa']:.3f} | {row['illustrative_Kt_root_stress_MPa']:.3f} | {row['camera_COM_bending_deflection_mm']:.3f} |")
    if args.stl or args.step:
        return 0 if all(row['single_solid_geometry_valid'] for group in report['artifact_geometry_checks'].values() for row in group) else 1
    return 0


if __name__=='__main__':
    raise SystemExit(main())
