import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "viewers"))
from build_official_revo3_assembly import BuildError, verify_mesh_manifest  # noqa: E402
from _runtime import _load_urdf_model  # noqa: E402


class OfficialRevo3MigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.full = ET.parse(ASSETS / "assembly.urdf").getroot()
        cls.training = ET.parse(ASSETS / "training/model5200_official_aligned_20261023_importer_runtime.urdf").getroot()
        cls.lineage = json.loads((ASSETS / "official_revo3_migration.json").read_text())

    def test_current_model_lineage_and_training_urdf_hashes(self):
        self.assertEqual(self.lineage["official_revo3_source_commit"], "6db7e2ed1d864ee0a3ab6ff62d43a303787137f6")
        full = ASSETS / "assembly.urdf"
        self.assertEqual(hashlib.sha256(full.read_bytes()).hexdigest(),
                         self.lineage["full_visual_model"]["sha256"])
        training_manifest = json.loads((ASSETS / "training/manifest.json").read_text())
        for record in (training_manifest["training_source_urdf"], training_manifest["isaac_runtime_urdf"]):
            self.assertEqual(hashlib.sha256((ROOT / record["path"]).read_bytes()).hexdigest(), record["sha256"])
        self.assertEqual(training_manifest["mounting"]["table_to_physical_base_gap_m"], 0.45)

    def test_official_hand_inertials_and_fixed_tip_aliases_are_materialized(self):
        for side, root in (("left", self.full), ("right", self.full)):
            links = {link.get("name"): link for link in root.findall("link")}
            joints = {joint.get("name"): joint for joint in root.findall("joint")}
            for finger in ("thumb", "index", "middle", "ring", "little"):
                tip = f"{side}_{finger}_tip_Link"
                self.assertIn(tip, links)
                self.assertIsNone(links[tip].find("inertial"))
                joint = joints[f"{side}_{finger}_tip_joint"]
                self.assertEqual(joint.get("type"), "fixed")
                self.assertEqual(joint.find("child").get("link"), tip)

    def test_mesh_manifest_rejects_tampered_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh = root / "meshes" / "hand.stl"
            mesh.parent.mkdir()
            mesh.write_bytes(b"official-mesh")
            record = {"output_path": "meshes/hand.stl", "sha256": hashlib.sha256(mesh.read_bytes()).hexdigest()}
            verify_mesh_manifest(root, [record])
            mesh.write_bytes(b"tampered-mesh")
            with self.assertRaisesRegex(BuildError, "hash mismatch"):
                verify_mesh_manifest(root, [record])

    def test_scene_fk_matches_migrated_urdf_at_nonzero_joint_pose(self):
        models = (_load_urdf_model(ASSETS / "assembly.urdf"),
                  mujoco.MjModel.from_xml_path(str(ASSETS / "scene.xml")))
        states = tuple(mujoco.MjData(model) for model in models)
        urdf = ET.parse(ASSETS / "assembly.urdf").getroot()
        hand_links = {"left_hand_base_link", "right_hand_base_link"}
        for side in ("left", "right"):
            hand_links.update(link.get("name") for link in urdf.findall("link")
                              if link.get("name", "").startswith(side + "_") and
                              ("thumb" in link.get("name", "") or "index" in link.get("name", "") or
                               "middle" in link.get("name", "") or "ring" in link.get("name", "") or
                               "little" in link.get("name", "")))
        joints = [joint for joint in urdf.findall("joint")
                  if joint.get("type") in {"revolute", "continuous"} and
                  (joint.get("name", "").startswith("left_") or joint.get("name", "").startswith("right_"))]
        for index, joint in enumerate(joints):
            limit = joint.find("limit")
            if limit is None or limit.get("lower") is None:
                value = 0.13 + 0.01 * (index % 4)
            else:
                lo, hi = float(limit.get("lower")), float(limit.get("upper"))
                value = lo + (hi - lo) * (0.37 + 0.02 * (index % 5))
            for model, data in zip(models, states):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint.get("name"))
                data.qpos[model.jnt_qposadr[jid]] = value
        for model, data in zip(models, states):
            mujoco.mj_forward(model, data)
        for name in sorted(hand_links):
            ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for model in models]
            if min(ids) < 0:
                continue
            bases = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link") for model in models]
            relative = []
            for model, data, base, body in zip(models, states, bases, ids):
                rotation = data.xmat[base].reshape(3, 3)
                relative.append((rotation.T @ (data.xpos[body] - data.xpos[base]),
                                 rotation.T @ data.xmat[body].reshape(3, 3)))
            np.testing.assert_allclose(relative[0][0], relative[1][0], atol=5e-7, err_msg=name)
            np.testing.assert_allclose(relative[0][1], relative[1][1], atol=5e-7, err_msg=name)

    def test_training_model_is_the_recorded_importer_runtime(self):
        manifest = json.loads((ASSETS / "training/manifest.json").read_text())
        path = ASSETS / "training/model5200_official_aligned_20261023_importer_runtime.urdf"
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                         manifest["isaac_runtime_urdf"]["sha256"])
        self.assertEqual((len(self.training.findall("link")), len(self.training.findall("joint")),
                          len(self.training.findall(".//collision"))), (32, 31, 58))
        self.assertEqual(sum(j.get("type") == "revolute" for j in self.training.findall("joint")), 28)
        self.assertEqual(manifest["source_commit"], "6db7e2ed1d864ee0a3ab6ff62d43a303787137f6")
