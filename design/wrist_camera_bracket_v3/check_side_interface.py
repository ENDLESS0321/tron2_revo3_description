"""Read the exported V3 base and check the specified small/large-hole roles."""
from pathlib import Path
import hashlib
import json
import math
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_SurfaceType

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
metadata=json.loads((HERE/"manifest.json").read_text())
base=cq.importers.importStep(str(HERE/"parts/base.step")).val()
cylinders=[]
for face in base.Faces():
    surface=BRepAdaptor_Surface(face.wrapped)
    if surface.GetType()==GeomAbs_SurfaceType.GeomAbs_Cylinder:
        c=surface.Cylinder();a=c.Axis();p=a.Location();d=a.Direction()
        if abs(c.Radius()-1.7)<1e-6 or abs(c.Radius()-3.0)<1e-6:
            cylinders.append({"radius_mm":c.Radius(),"axis_point_mm":[p.X(),p.Y(),p.Z()],"axis_direction":[d.X(),d.Y(),d.Z()]})
if len(cylinders)!=4:raise ValueError("Expected two shafts and two counterbore cylinders")
samples=[]
for local_theta,expected_material,role in [(60,False,"small_global_330"),(120,False,"small_global_30"),(105,True,"covered_large_global_15")]:
    angle=math.radians(local_theta)
    for radius in [31.,32.,33.,34.,35.5]:
        point=cq.Vector(radius*math.cos(angle),-4.5,radius*math.sin(angle))
        inside=bool(base.isInside(point,1e-6))
        if inside!=expected_material:raise AssertionError(f"Incorrect hole/coverage at {role} r={radius}")
        samples.append({"role":role,"local_theta_deg":local_theta,"global_theta_after_Ry90_deg":(local_theta-90)%360,"radius_mm":radius,"material_present":inside})
record={"generator_sha256":metadata["generator_sha256"],"whole_mm_sha256":metadata["files"]["whole_bracket_mm.stl"],"local_small_holes_deg":[60,120],"mapped_global_small_holes_deg":[330,30],"small_hole_axis_angle_deg":60,"small_hole_chord_at_r30_5_mm":30.5,"large_global_15_covered":True,"fitted_brep_cylinders":cylinders,"material_probe_samples":samples,"v2_file_unchanged":hashlib.sha256((ROOT/"scripts/design_wrist_camera_bracket_v2.py").read_bytes()).hexdigest()=="570d99013ac23f787a533d93ebdfaccdba6ddbd91532def837aab58ff44180e5","scope":"actual STEP cylindrical surfaces and centerline material checks; no physical screw/thread or strength certification","passed":True}
(HERE/"interface_validation.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf-8")
print(json.dumps(record,indent=2))
