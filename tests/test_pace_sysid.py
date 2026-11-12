import hashlib
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
TRAINING = ROOT / "assets/training"
sys.path.insert(0, str(ROOT / "scripts"))

from revo3_pace_profile import FIELDS, isaaclab_actuator_kwargs, load_profile, load_sysid  # noqa: E402


HAND_FINGERS = ("thumb", "index", "middle", "ring", "little")
TRAINING_URDFS = ("model5200_official_aligned_20261023_importer_runtime.urdf",
                  "model5200_official_aligned_20261023_icp.source.urdf")


def hand_joints(urdf_name):
    root = ET.parse(TRAINING / urdf_name).getroot()
    return {j.get("name"): j for j in root.findall("joint")
            if j.get("type") == "revolute" and j.get("name").split("_")[1] in HAND_FINGERS}


class PaceSysidTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sysid = load_sysid()
        cls.manifest = json.loads((TRAINING / "manifest.json").read_text())
        cls.joints = load_profile()

    def test_manifest_points_to_hashed_sidecar(self):
        overrides = self.manifest["runtime_only_overrides"]
        self.assertEqual(overrides["per_joint_hand_dynamics"], "revo3_pace_sysid.json")
        self.assertEqual(overrides["joint_velocity_rad_s"], 6.28)
        for stale in ("position_gain", "velocity_gain", "actuator_effort_nm", "per_joint_hand_dynamics_default_profile"):
            self.assertNotIn(stale, overrides)
        record = self.manifest["files"]["revo3_pace_sysid.json"]
        self.assertEqual(hashlib.sha256((TRAINING / "revo3_pace_sysid.json").read_bytes()).hexdigest(),
                         record["sha256"])

    def test_documentation_hashes_are_current(self):
        for doc, digest in self.manifest["documentation_sha256"].items():
            self.assertEqual(hashlib.sha256((ROOT / doc).read_bytes()).hexdigest(), digest, doc)

    def test_parameters_cover_every_runtime_hand_joint(self):
        expected = set(hand_joints(TRAINING_URDFS[0]))
        self.assertEqual(len(expected), 21)
        self.assertEqual(set(self.joints), expected)
        for name, values in self.joints.items():
            self.assertEqual(set(values), set(FIELDS), name)
            self.assertTrue(all(math.isfinite(v) for v in values.values()), name)
            for field in ("kp", "kd", "armature", "effort_limit"):
                self.assertGreater(values[field], 0.0, f"{name}.{field}")
            for field in ("friction", "viscous_damping"):
                self.assertGreaterEqual(values[field], 0.0, f"{name}.{field}")

    def test_section17_fields_replace_fit03(self):
        replaced = self.sysid["parameters"]["replaced_from_fit_03"]
        pairs = {(name, field) for name, fields in replaced.items() for field in fields}
        self.assertEqual(pairs, {
            ("right_thumb_CMP_joint", "armature"), ("right_thumb_CMP_joint", "kp"),
            ("right_thumb_CMP_joint", "kd"), ("right_thumb_CMR_joint", "kp"),
            ("right_thumb_CMR_joint", "kd"), ("right_ring_PIP_joint", "kp"),
            ("right_ring_PIP_joint", "kd")})
        for name, fields in replaced.items():
            for field, record in fields.items():
                self.assertEqual(self.joints[name][field], record["current"], f"{name}.{field}")
                self.assertNotEqual(record["fit_03"], record["current"], f"{name}.{field}")
        self.assertEqual(self.joints["right_thumb_CMP_joint"]["kp"], 23.543235)
        self.assertEqual(self.joints["right_ring_PIP_joint"]["kd"], 0.120927)

    def test_unreplaced_fit03_report_values(self):
        self.assertEqual(self.joints["right_thumb_CMP_joint"]["effort_limit"], 0.593)
        self.assertEqual(self.joints["right_index_MPR_joint"]["kp"], 21.42)
        self.assertEqual(self.joints["right_ring_DIP_joint"]["effort_limit"], 1.35)
        self.assertEqual(self.joints["right_little_DIP_joint"]["armature"], 0.000705)
        timing = self.sysid["model_assumptions"]
        self.assertEqual((timing["command_delay_s"], timing["pd_update_period_s"], timing["physics_dt_s"]),
                         (0.015, 0.005, 0.0025))

    def test_training_urdfs_carry_sidecar_dynamics(self):
        for urdf_name in TRAINING_URDFS:
            for name, joint in hand_joints(urdf_name).items():
                values = self.joints[name]
                limit, dynamics = joint.find("limit"), joint.find("dynamics")
                self.assertIsNotNone(dynamics, f"{urdf_name}:{name}")
                self.assertEqual(float(limit.get("effort")), values["effort_limit"], f"{urdf_name}:{name}")
                self.assertEqual(float(dynamics.get("damping")), values["viscous_damping"], f"{urdf_name}:{name}")
                self.assertEqual(float(dynamics.get("friction")), values["friction"], f"{urdf_name}:{name}")
                self.assertEqual(limit.get("velocity"), "0", f"{urdf_name}:{name}")

    def test_isaaclab_kwargs_use_delayed_pd_field_names(self):
        kwargs = isaaclab_actuator_kwargs()
        self.assertEqual(set(kwargs), {"stiffness", "damping", "armature", "friction",
                                       "viscous_friction", "effort_limit", "command_delay_s"})
        self.assertEqual(set(kwargs["stiffness"]), set(self.joints))
        self.assertEqual(round(kwargs["command_delay_s"] / 0.005), 3)


if __name__ == "__main__":
    unittest.main()
