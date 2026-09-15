#!/usr/bin/env python3
"""Build/cache the photo-reference bracket CAD; never substitute a visual-only rod."""
import fcntl
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ensure_bracket(length_mm,plate_tilt_deg,design='parametric_side_bend_v4'):
    length_mm=float(length_mm);plate_tilt_deg=float(plate_tilt_deg)
    if not all(math.isfinite(x) for x in [length_mm,plate_tilt_deg]):
        raise ValueError('Bracket parameters must be finite')
    versions={
        'parametric_photo_reference_v1':('design_wrist_camera_bracket.py','wrist_camera_bracket_v1'),
        'parametric_vertical_v2':('design_wrist_camera_bracket_v2.py','wrist_camera_bracket_v2'),
        'parametric_side_mount_v3':('design_wrist_camera_bracket_v3.py','wrist_camera_bracket_v3'),
        'parametric_side_bend_v4':('design_wrist_camera_bracket_v4.py','wrist_camera_bracket_v4'),
    }
    if design not in versions:raise ValueError(f'Unknown bracket design: {design}')
    script,folder=versions[design]
    producer=ROOT/'scripts'/script
    if not producer.is_file():
        raise FileNotFoundError('The parametric bracket CAD producer is not available yet')
    request={'design':design,'length_mm':length_mm,'plate_tilt_deg':plate_tilt_deg,'producer_sha256':sha(producer)}
    key=hashlib.sha256(json.dumps(request,sort_keys=True).encode()).hexdigest()[:16]
    output=ROOT/'design'/folder/'variants'/key
    output.mkdir(parents=True,exist_ok=True)
    stamp=output/'asset_cache.json'
    with (output/'build.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        fresh=False
        if stamp.is_file():
            old=json.loads(stamp.read_text())
            fresh=old['request']==request and all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in old['files'].items())
        if not fresh:
            cmd=[str(ROOT/'.cad-venv/bin/python'),str(producer),'--length-mm',str(length_mm),
                 '--plate-tilt-deg',str(plate_tilt_deg),'--output-directory',str(output)]
            with (output/'cad_build.log').open('w') as log:
                result=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            if result.returncode:
                raise RuntimeError(f'Bracket CAD generation failed; see {output / "cad_build.log"}')
            required=[output/'whole_bracket_m.stl',output/'whole_bracket_mm.stl',output/'whole_bracket.step',output/'manifest.json']
            if not all(p.is_file() for p in required):
                raise ValueError('Bracket CAD did not produce the complete solid/manifest bundle')
            meshdir=ROOT/'meshes/wrist_camera_brackets'/key
            meshdir.mkdir(parents=True,exist_ok=True)
            dest=meshdir/f'camera_bracket_{key}.stl'
            shutil.copy2(required[0],dest)
            files={str(p.relative_to(ROOT)):sha(p) for p in [*required,dest]}
            stamp.write_text(json.dumps({'request':request,'mesh':str(dest.relative_to(ROOT)),
                                         'manifest':str(required[-1].relative_to(ROOT)),'files':files},indent=2)+'\n')
        record=json.loads(stamp.read_text())
    return {'key':key,'mesh':ROOT/record['mesh'],'manifest':json.loads((ROOT/record['manifest']).read_text()),
            'manifest_path':ROOT/record['manifest'],'cache_path':stamp}
