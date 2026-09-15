"""Verify exported V2 BRep parameter separation and absence of the old arm."""
from pathlib import Path
import hashlib
import json
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
GENERATOR=ROOT/"scripts/design_wrist_camera_bracket_v2.py"
CASES=[HERE,HERE/"options/L080_T060",HERE/"checks/L100_T040",HERE/"checks/L020_T000",HERE/"checks/L150_T075"]

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def shape(path):return cq.importers.importStep(str(path)).val()
def volume(obj):return float(obj.Volume(1e-9))
def symmetric_volume(a,b):return volume(a.cut(b))+volume(b.cut(a))
def local_plate(obj,length,tilt):return obj.translate((0,0,-35.8-length)).rotate((0,0,0),(1,0,0),-tilt)

reference=json.loads((HERE/"manifest.json").read_text())
base=shape(HERE/"parts/base.step")
stem=shape(HERE/"parts/stem.step")
plate=local_plate(shape(HERE/"parts/plate.step"),80,40)
section_area=volume(stem)/80
report={"generator_sha256":sha(GENERATOR),"frozen_v1_helper_sha256":sha(ROOT/"scripts/design_wrist_camera_bracket.py"),"cases":[]}
for directory in CASES:
    metadata=json.loads((directory/"manifest.json").read_text())
    if metadata["generator_sha256"]!=report["generator_sha256"]:raise ValueError("Stale V2 generation")
    if metadata["frozen_v1_helper_sha256"]!=report["frozen_v1_helper_sha256"]:raise ValueError("V1 helper changed")
    files=dict(metadata["files"])
    for item in metadata["parts"].values():files.update(item["files"])
    if any(sha(directory/path)!=checksum for path,checksum in files.items()):raise ValueError("Export fingerprint mismatch")
    length=metadata["parameters"]["length_mm"];tilt=metadata["parameters"]["plate_tilt_deg"]
    current_base=shape(directory/"parts/base.step")
    current_stem=shape(directory/"parts/stem.step")
    bounds=current_stem.BoundingBox()
    radii=[]
    for face in current_base.Faces():
        surface=BRepAdaptor_Surface(face.wrapped)
        if surface.GetType()==GeomAbs_SurfaceType.GeomAbs_Cylinder:radii.append(surface.Cylinder().Radius())
    if any(abs(radius-4.75)<1e-4 for radius in radii):raise ValueError("Old 9.5 mm relief cylinder remains")
    base_delta=symmetric_volume(base,current_base)
    area_delta=abs(volume(current_stem)/length-section_area)
    if base_delta>1e-4 or area_delta>1e-6:raise ValueError("V2 parameter separation failed")
    if abs(bounds.ymin+8.5)>1e-4 or abs(bounds.ymax-3.5)>1e-4:raise ValueError("Stem is no longer the 12 mm Y-depth upright")
    if abs((bounds.zmax-bounds.zmin)-length)>1e-4:raise ValueError("Length does not control radial Z span")
    row={"directory":str(directory.relative_to(HERE)) if directory!=HERE else ".","length_mm":length,"plate_tilt_deg":tilt,"export_hashes_verified":len(files),"base_symmetric_difference_mm3":base_delta,"stem_volume_per_length_difference_mm2":area_delta,"stem_actual_y_span_mm":[bounds.ymin,bounds.ymax],"stem_actual_z_span_mm":[bounds.zmin,bounds.zmax],"old_9_5_relief_cylinder_count":0,"base_cylinder_radii_mm":sorted(set(round(r,6) for r in radii)),"whole_solid_count":metadata["whole"]["solid_count"],"whole_step_valid":metadata["verification"]["whole_step_roundtrip"]["valid"],"mm_stl_watertight":metadata["verification"]["whole_stl_mm"]["watertight"],"m_stl_watertight":metadata["verification"]["whole_stl_m"]["watertight"]}
    if length==80:
        row["stem_symmetric_difference_at_equal_length_mm3"]=symmetric_volume(stem,current_stem)
    if tilt==40:
        current_plate=local_plate(shape(directory/"parts/plate.step"),length,tilt)
        row["plate_local_symmetric_difference_at_equal_tilt_mm3"]=symmetric_volume(plate,current_plate)
    if any(row[k]>1e-4 for k in ["stem_symmetric_difference_at_equal_length_mm3","plate_local_symmetric_difference_at_equal_tilt_mm3"] if k in row):raise ValueError("V2 fixed part changed unexpectedly")
    report["cases"].append(row)
report["passed"]=True
report["scope"]="Actual exported BRep/hash checks for the listed parameter samples; no structural or physical-fit certification."
(HERE/"parameter_validation.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
print(json.dumps(report,indent=2))
