#!/usr/bin/env python3
"""Read-only STEP preflight and BRep feature inventory for camera brackets.

This currently inspects sources. It does not claim to parameterize geometry
that cannot be read or whose rod/plate segmentation has not been established.
No protected-file decoding is attempted.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SOURCE_ROOT=ROOT.parents[1]
OUTPUT=ROOT/"design/camera_brackets"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xyz(vector):
    return [float(vector.x),float(vector.y),float(vector.z)]


def ocp_xyz(vector):
    return [float(vector.X()),float(vector.Y()),float(vector.Z())]


def brep_inventory(path):
    import cadquery as cq
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_SurfaceType
    imported=cq.importers.importStep(str(path))
    solids=[solid for shape in imported.vals() for solid in shape.Solids()]
    if not solids:
        raise ValueError("STEP imported but contains no solids")
    descriptions=[]
    for index,solid in enumerate(solids):
        bounds=solid.BoundingBox()
        entry={"index":index,"valid":bool(solid.isValid()),"volume_mm3":solid.Volume(),"surface_area_mm2":solid.Area(),"center_mm":xyz(solid.Center()),"bounds_mm":[[bounds.xmin,bounds.ymin,bounds.zmin],[bounds.xmax,bounds.ymax,bounds.zmax]],"faces":[]}
        for face_index,face in enumerate(solid.Faces()):
            surface=BRepAdaptor_Surface(face.wrapped)
            kind=surface.GetType()
            feature={"index":face_index,"surface_type":str(kind),"area_mm2":face.Area(),"center_mm":xyz(face.Center())}
            if kind==GeomAbs_SurfaceType.GeomAbs_Plane:
                feature["plane_normal"]=xyz(face.normalAt())
                feature["plane_origin_mm"]=ocp_xyz(surface.Plane().Location())
            elif kind==GeomAbs_SurfaceType.GeomAbs_Cylinder:
                cylinder=surface.Cylinder()
                feature.update({"cylinder_radius_mm":cylinder.Radius(),"cylinder_axis_origin_mm":ocp_xyz(cylinder.Axis().Location()),"cylinder_axis_direction":ocp_xyz(cylinder.Axis().Direction())})
            entry["faces"].append(feature)
        descriptions.append(entry)
    return {"solid_count":len(solids),"solids":descriptions,"coordinate_unit_note":"CadQuery/OCP working coordinates expected in mm; STEP unit declaration must be verified before manufacturing use"}


def inspect(path,probe_ocp=False):
    data=path.read_bytes()
    record={"path":str(path.resolve()),"filename":path.name,"bytes":len(data),"sha256":hashlib.sha256(data).hexdigest(),"first_16_bytes_hex":data[:16].hex(),"step_p21_magic_offset":data.find(b"ISO-10303-21"),"step_data_token_offset":data.find(b"DATA;"),"source_modified":False}
    if data.startswith(b"%TSD-Header-###%"):
        record.update({"status":"current_read_not_standard_step","wrapper_signature":"%TSD-Header-###%","interpretation":"This read returned a nonstandard header and no plaintext STEP signature; it does not establish whether the asset is readable in another authorized environment.","can_infer_geometry":False})
    elif b"ISO-10303-21" not in data[:256]:
        record.update({"status":"blocked_unrecognized_source_format","can_infer_geometry":False})
    else:
        try:
            record["brep"]=brep_inventory(path)
            record["status"]="brep_readable_requires_component_identification"
            record["can_infer_geometry"]=True
        except Exception as exc:
            record["status"]="blocked_step_import_failed"
            record["can_infer_geometry"]=False
            record["import_error"]=str(exc)
    if probe_ocp:
        try:
            from OCP.STEPControl import STEPControl_Reader
            from OCP.IFSelect import IFSelect_RetDone
            reader=STEPControl_Reader()
            result=reader.ReadFile(str(path))
            record["ocp_raw_read_status"]=str(result)
            record["ocp_raw_read_succeeded"]=bool(result==IFSelect_RetDone)
            record["ocp_raw_root_count"]=reader.NbRootsForTransfer() if result==IFSelect_RetDone else 0
        except Exception as exc:
            record["ocp_probe_error"]=str(exc)
    if digest(path)!=record["sha256"]:
        raise RuntimeError(f"Source changed while being inspected: {path}")
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-source",type=Path,default=SOURCE_ROOT/"Tron2灵巧手相机支架-l.stp")
    parser.add_argument("--right-source",type=Path,default=SOURCE_ROOT/"Tron2灵巧手相机支架-r.stp")
    parser.add_argument("--output",type=Path,default=OUTPUT)
    parser.add_argument("--probe-ocp",action="store_true",help="Also ask the actual STEP parser to read each unmodified original file")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    versions={}
    for package in ["cadquery","cadquery-ocp","numpy"]:
        try:
            versions[package]=importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package]=None
    records={side:inspect(path,args.probe_ocp) for side,path in [("left",args.left_source),("right",args.right_source)]}
    report={"schema":"camera_bracket_source_inspection_v1","sources":records,"environment_versions":versions,"left_right_relationship":"not_determined" if not all(r["can_infer_geometry"] for r in records.values()) else "requires_solid_registration_and_feature_comparison","geometry_production_available":False,"required_next_evidence":"Readable standard STEP export and identification of actual mount/rod/plate BRep features before any parameterized production geometry can be validated."}
    (args.output/"source_inspection.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if all(r["can_infer_geometry"] for r in records.values()) else 2


if __name__=="__main__":
    raise SystemExit(main())
