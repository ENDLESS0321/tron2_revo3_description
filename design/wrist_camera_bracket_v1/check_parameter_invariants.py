"""Read exported STEP bodies and verify parameter separation and file hashes."""
from pathlib import Path
import hashlib
import json
import math
import cadquery as cq

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
GENERATOR=ROOT/"scripts/design_wrist_camera_bracket.py"
CASES=[HERE,HERE/"checks/L20_T0",HERE/"checks/L150_T75",HERE/"checks/L80_T20",HERE/"checks/L80_T70",HERE/"checks/L150_T40",HERE/"options/L080_T060"]

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def shape(path):return cq.importers.importStep(str(path)).val()
def volume(obj):return float(obj.Volume(1e-9))
def symmetric_volume(first,second):return volume(first.cut(second))+volume(second.cut(first))

reference=json.loads((HERE/"manifest.json").read_text())
base=shape(HERE/"parts/wrist_mount.step")
arm=shape(HERE/"parts/support_arm.step")
plate=shape(HERE/"parts/camera_plate.step").translate((0,-80,-62)).rotate((0,0,0),(1,0,0),-40)
area=reference["parts"]["support_arm"]["volume_mm3"]/80
report={"generator_sha256":sha(GENERATOR),"cases":[],"claims_scope":"Exported BRep comparisons and representative parameter cases; not manufacturing tolerance or mechanical strength certification."}
for directory in CASES:
    item=json.loads((directory/"manifest.json").read_text())
    if item["generator_sha256"]!=report["generator_sha256"]:
        raise ValueError(f"Stale generator fingerprint: {directory}")
    all_files=dict(item["files"])
    for part in item["parts"].values():all_files.update(part["files"])
    if any(sha(directory/path)!=checksum for path,checksum in all_files.items()):
        raise ValueError(f"Asset fingerprint mismatch: {directory}")
    length=item["parameters"]["length_mm"]
    tilt=item["parameters"]["plate_tilt_deg"]
    current_base=shape(directory/"parts/wrist_mount.step")
    current_arm=shape(directory/"parts/support_arm.step")
    base_delta=symmetric_volume(base,current_base)
    area_delta=abs(volume(current_arm)/length-area)
    row={"directory":str(directory.relative_to(HERE)) if directory!=HERE else ".","length_mm":length,"plate_tilt_deg":tilt,"files_verified":len(all_files),"whole_solid_count":item["whole"]["solid_count"],"step_valid":item["verification"]["whole_step_roundtrip"]["valid"],"stl_mm_watertight":item["verification"]["whole_stl_mm"]["watertight"],"stl_m_watertight":item["verification"]["whole_stl_m"]["watertight"],"wrist_base_symmetric_difference_mm3":base_delta,"arm_volume_per_length_difference_mm2":area_delta}
    if length==80:
        row["arm_symmetric_difference_at_equal_length_mm3"]=symmetric_volume(arm,current_arm)
    if tilt==40:
        current_plate=shape(directory/"parts/camera_plate.step").translate((0,-length,-62)).rotate((0,0,0),(1,0,0),-tilt)
        row["plate_local_symmetric_difference_at_equal_tilt_mm3"]=symmetric_volume(plate,current_plate)
    if base_delta>1e-4 or area_delta>1e-6:
        raise AssertionError(f"Parameter coupling failed: {row}")
    if any(row[key]>1e-4 for key in ["arm_symmetric_difference_at_equal_length_mm3","plate_local_symmetric_difference_at_equal_tilt_mm3"] if key in row):
        raise AssertionError(f"Unintended change in fixed semantic part: {row}")
    report["cases"].append(row)
report["passed"]=True
(HERE/"parameter_validation.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
print(json.dumps(report,indent=2))
