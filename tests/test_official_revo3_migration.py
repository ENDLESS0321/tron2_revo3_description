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

    def test_collision_reducer_rejects_unpinned_input_before_writing_output(self):
        import subprocess

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "must-not-be-created"
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/build_collision58.py"),
                 "--urdf", str(ASSETS / "assembly_rl_convex.urdf"), "--output", str(output)],
                capture_output=True, text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("pinned official 91-collision", result.stderr)
            self.assertFalse(output.exists())

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

    def test_training_model_keeps_reduced_topology_and_contact_geometry(self):
        def count_moving(root):
            return sum(j.get("type") in {"revolute", "continuous", "prismatic"}
                       for j in root.findall("joint"))

        self.assertEqual((len(self.full.findall("link")), count_moving(self.full),
                          len(self.full.findall(".//collision"))), (114, 58, 280))
        self.assertEqual((len(self.training.findall("link")), count_moving(self.training),
                          len(self.training.findall(".//collision"))), (37, 28, 58))
        self.assertEqual(self.lineage["hand_geometry_counts"]["training_right"]["output_hand_collisions"], 49)
        self.assertFalse(self.lineage["hand_geometry_counts"]["training_right"]["preserved"])
        # Training retains the old non-hand reduction and arm geometry while
        # the active hand keeps non-collision structure and exact DIP geometry.
        def signature(node):
            return [node.tag, sorted(node.attrib.items()), (node.text or "").strip(),
                    [signature(child) for child in node]]

        retained = []
        for node in self.training.findall("link"):
            name = node.get("name", "")
            if name == "right_palm" or (name.startswith("right_") and name != "right_adapter_link"):
                continue
            retained.append(["link", name, signature(node)])
        retained.extend(["joint", node.get("name"), signature(node)]
                        for node in self.training.findall("joint")
                        if (not node.get("name", "").startswith("right_")
                            or node.get("name") == "right_adapter_mount"))
        retained.sort(key=lambda item: (item[0], item[1]))
        preserved = self.lineage["training_topology_preservation"]
        payload = json.dumps(retained, separators=(",", ":"), ensure_ascii=False).encode()
        self.assertEqual(len(retained), preserved["retained_non_hand_nodes"])
        self.assertEqual(hashlib.sha256(payload).hexdigest(),
                         preserved["retained_non_hand_signature_sha256"])
        self.assertEqual(preserved["removed_obsolete_nodes"], ["right_palm", "right_palm_center"])
        def is_active_hand_node(node, tag):
            name = node.get("name", "")
            if tag == "link" and name == "right_hand_base_link":
                return True
            if tag == "joint" and name == "right_hand_base_joint":
                return True
            return name.startswith("right_") and any(
                finger in name for finger in ("thumb", "index", "middle", "ring", "little"))

        for tag in ("link", "joint"):
            def comparable(node):
                import copy
                node = copy.deepcopy(node)
                if tag == "link" and "_DIP_" not in node.get("name", ""):
                    for collision in node.findall("collision"):
                        node.remove(collision)
                return signature(node)
            full_nodes = {node.get("name"): comparable(node)
                          for node in self.full.findall(tag)
                          if is_active_hand_node(node, tag)}
            training_nodes = {node.get("name"): comparable(node)
                              for node in self.training.findall(tag)
                              if is_active_hand_node(node, tag)}
            self.assertEqual(training_nodes, full_nodes)

    def test_training_right_hand_fk_matches_full_model_at_nonzero_pose(self):
        models = (_load_urdf_model(ASSETS / "assembly.urdf"),
                  _load_urdf_model(ASSETS / "assembly_rl_convex.urdf"))
        states = tuple(mujoco.MjData(model) for model in models)
        joints = [joint for joint in self.full.findall("joint")
                  if joint.get("type") in {"revolute", "continuous"}
                  and (joint.get("name", "").startswith("right_")
                       or "_R_Joint" in joint.get("name", ""))]
        for index, joint in enumerate(joints):
            limit = joint.find("limit")
            value = (0.11 + 0.01 * (index % 3) if limit is None or limit.get("lower") is None
                     else float(limit.get("lower")) +
                     (float(limit.get("upper")) - float(limit.get("lower"))) * (0.31 + 0.02 * (index % 5)))
            for model, data in zip(models, states):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint.get("name"))
                self.assertGreaterEqual(jid, 0, joint.get("name"))
                data.qpos[model.jnt_qposadr[jid]] = value
        for model, data in zip(models, states):
            mujoco.mj_forward(model, data)
        links = [node.get("name") for node in self.full.findall("link")
                 if node.get("name") == "right_hand_base_link"
                 or (node.get("name", "").startswith("right_")
                     and any(finger in node.get("name", "")
                             for finger in ("thumb", "index", "middle", "ring", "little")))]
        bases = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link") for model in models]
        for name in links:
            ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for model in models]
            self.assertTrue(all(index >= 0 for index in ids), name)
            relative = []
            for model, data, base, body in zip(models, states, bases, ids):
                rotation = data.xmat[base].reshape(3, 3)
                relative.append((rotation.T @ (data.xpos[body] - data.xpos[base]),
                                 rotation.T @ data.xmat[body].reshape(3, 3)))
            np.testing.assert_allclose(relative[0][0], relative[1][0], atol=5e-7, err_msg=name)
            np.testing.assert_allclose(relative[0][1], relative[1][1], atol=5e-7, err_msg=name)


if __name__ == "__main__":
    unittest.main()
