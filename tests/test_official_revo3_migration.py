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
        cls.training = ET.parse(ASSETS / "assembly_rl_convex.urdf").getroot()
        cls.lineage = json.loads((ASSETS / "official_revo3_migration.json").read_text())

    def test_source_commit_output_hashes_mounts_and_joint_order_mapping(self):
        self.assertEqual(self.lineage["official_source"]["commit"], "6db7e2ed1d864ee0a3ab6ff62d43a303787137f6")
        for name, root in (("full_assembly", ASSETS / "assembly.urdf"),
                           ("training_assembly", ASSETS / "assembly_rl_convex.urdf")):
            record = self.lineage["outputs"][name]
            self.assertEqual(hashlib.sha256(root.read_bytes()).hexdigest(), record["sha256"])
        cases = ((self.full, "left_hand_base_joint", "full_left"),
                 (self.full, "right_hand_base_joint", "full_right"),
                 (self.training, "right_hand_base_joint", "training_right"))
        for root, joint_name, manifest_key in cases:
            joint = root.find(f"joint[@name='{joint_name}']")
            recorded = self.lineage["mounts_preserved"][manifest_key]
            self.assertTrue(recorded["preserved_exactly"])
            self.assertEqual(joint.find("parent").get("link"), recorded["parent"])
            self.assertEqual(joint.find("child").get("link"), recorded["child"])
            self.assertEqual(joint.find("origin").get("xyz"), recorded["origin_xyz_rpy"]["xyz"])
            self.assertEqual(joint.find("origin").get("rpy"), recorded["origin_xyz_rpy"]["rpy"])
        for side in ("left", "right", "training_right"):
            mapping = self.lineage["hand_joint_order_mappings"][side]
            source = mapping["source_joint_order"]
            official = mapping["official_joint_order"]
            self.assertEqual(len(source), 21)
            self.assertEqual(source, official)
            self.assertEqual([row["source_joint"] for row in mapping["mapping"]], source)
            self.assertEqual([row["official_joint"] for row in mapping["mapping"]], official)

    def test_official_hand_inertials_and_fixed_tip_aliases_are_materialized(self):
        for side, root in (("left", self.full), ("right", self.full), ("right", self.training)):
            links = {link.get("name"): link for link in root.findall("link")}
            joints = {joint.get("name"): joint for joint in root.findall("joint")}
            for finger in ("thumb", "index", "middle", "ring", "little"):
                tip = f"{side}_{finger}_tip_Link"
                self.assertIn(tip, links)
                self.assertIsNone(links[tip].find("inertial"))
                joint = joints[f"{side}_{finger}_tip_joint"]
                self.assertEqual(joint.get("type"), "fixed")
                self.assertEqual(joint.find("child").get("link"), tip)
        for side in ("left", "right"):
            expected = {item["output_link"]: item["inertial_xml_sha256"]
                        for item in self.lineage["hand_link_inertials"][side]}
            for name, digest in expected.items():
                node = self.full.find(f"link[@name='{name}']/inertial")
                self.assertIsNotNone(node)
                self.assertEqual(hashlib.sha256(ET.tostring(node, encoding="utf-8")).hexdigest(), digest)

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


if __name__ == "__main__":
    unittest.main()
