#!/usr/bin/env python3
"""V4 keeps the side mount, defining the user angle BETWEEN plate and stem."""
import argparse
import contextlib
import io
import json
from pathlib import Path

import design_wrist_camera_bracket_v3 as v3

ROOT=Path(__file__).resolve().parents[1]


def build(args):
    bend=float(args.plate_tilt_deg)
    if not 15<=bend<=90:
        raise ValueError('Plate-to-stem angle must be 15..90 degrees')
    plane=90-bend
    internal=argparse.Namespace(length_mm=args.length_mm,plate_tilt_deg=plane,
        density_kg_m3=args.density_kg_m3,output_directory=args.output_directory)
    with contextlib.redirect_stdout(io.StringIO()):
        report=v3.build(internal)
    report['schema']='wrist_camera_bracket_side_bend_v4'
    report['design_basis']='Side small-hole mount unchanged; user angle is the acute angle between plate direction and stem, not the horizontal-plane rotation'
    report['parameters']['plate_tilt_deg']=bend
    report['parameters']['plate_to_stem_angle_deg']=bend
    report['parameters']['plate_plane_rotation_deg']=plane
    report['parameters']['plate_tilt_range_deg']=[15,90]
    report['parameter_semantics']['plate_tilt']='User angle between mounting plate and stem. Internal CAD Rx = 90 - user angle; only plate and root transition change.'
    report['strength_geometry']['default_tilt_deg']=plane
    report['strength_geometry']['angle_for_COM_rotation_deg']=plane
    report['strength_geometry']['user_plate_to_stem_angle_deg']=bend
    report['generator_sha256']=v3.sha(Path(__file__))
    report['frozen_v3_geometry_producer_sha256']=v3.sha(Path(v3.__file__))
    report['verification']['plate_to_stem_angle_deg']=90-report['plate_frame']['rpy_deg'][0]
    (args.output_directory/'manifest.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'output':str(args.output_directory),'user_angle_deg':bend,'actual_CAD_Rx_deg':plane,'whole':report['whole']},indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--length-mm',type=float,default=80)
    p.add_argument('--plate-tilt-deg',type=float,default=15,help='Angle BETWEEN plate and stem, in degrees')
    p.add_argument('--density-kg-m3',type=float,default=1250)
    p.add_argument('--output-directory',type=Path,default=ROOT/'design/wrist_camera_bracket_v4')
    build(p.parse_args())
