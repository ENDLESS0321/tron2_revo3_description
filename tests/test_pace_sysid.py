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


class PaceSysidTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sysid = load_sysid()
        cls.manifest = json.loads((TRAINING / "manifest.json").read_text())
        runtime = ET.parse(TRAINING / cls.sysid["applies_to"]["urdf"]).getroot()
        cls.hand_joints = {j.get("name") for j in runtime.findall("joint")
                           if j.get("type") == "revolute" and j.get("name").split("_")[1] in HAND_FINGERS}

    def test_manifest_points_to_hashed_sidecar(self):
        overrides = self.manifest["runtime_only_overrides"]
        self.assertEqual(overrides["per_joint_hand_dynamics"], "revo3_pace_sysid.json")
        self.assertEqual(overrides["per_joint_hand_dynamics_default_profile"], self.sysid["default_profile"])
        self.assertEqual(overrides["joint_velocity_rad_s"], 6.28)
        for stale in ("position_gain", "velocity_gain", "actuator_effort_nm"):
            self.assertNotIn(stale, overrides)
        record = self.manifest["files"]["revo3_pace_sysid.json"]
        self.assertEqual(hashlib.sha256((TRAINING / "revo3_pace_sysid.json").read_bytes()).hexdigest(),
                         record["sha256"])

    def test_documentation_hashes_are_current(self):
        for doc, digest in self.manifest["documentation_sha256"].items():
            self.assertEqual(hashlib.sha256((ROOT / doc).read_bytes()).hexdigest(), digest, doc)

    def test_default_profile_covers_every_runtime_hand_joint(self):
        self.assertEqual(len(self.hand_joints), 21)
        joints = load_profile()
        self.assertEqual(set(joints), self.hand_joints)
        for name, values in joints.items():
            self.assertEqual(set(values), set(FIELDS), name)
            self.assertTrue(all(math.isfinite(v) for v in values.values()), name)
            for field in ("kp", "kd", "armature", "effort_limit"):
                self.assertGreater(values[field], 0.0, f"{name}.{field}")
            for field in ("friction", "viscous_damping"):
                self.assertGreaterEqual(values[field], 0.0, f"{name}.{field}")

    def test_report_values_spot_check(self):
        joints = load_profile("fit_03")
        self.assertEqual(joints["right_thumb_CMP_joint"],
                         {"kp": 7.151, "kd": 0.339, "armature": 0.00532, "friction": 0.004,
                          "viscous_damping": 0.083, "effort_limit": 0.593})
        self.assertEqual(joints["right_index_MPR_joint"]["kp"], 21.42)
        self.assertEqual(joints["right_ring_DIP_joint"]["effort_limit"], 1.35)
        self.assertEqual(joints["right_little_DIP_joint"]["armature"], 0.000705)
        timing = self.sysid["model_assumptions"]
        self.assertEqual((timing["command_delay_s"], timing["pd_update_period_s"], timing["physics_dt_s"]),
                         (0.015, 0.005, 0.0025))

    def test_balanced_overlay_changes_exactly_seven_fields(self):
        parent, balanced = load_profile("fit_03"), load_profile("balanced_20261101_01")
        changed = {(name, field) for name in parent for field in FIELDS
                   if parent[name][field] != balanced[name][field]}
        self.assertEqual(changed, {
            ("right_thumb_CMP_joint", "armature"), ("right_thumb_CMP_joint", "kp"),
            ("right_thumb_CMP_joint", "kd"), ("right_thumb_CMR_joint", "kp"),
            ("right_thumb_CMR_joint", "kd"), ("right_ring_PIP_joint", "kp"),
            ("right_ring_PIP_joint", "kd")})
        self.assertEqual(balanced["right_thumb_CMP_joint"]["kp"], 23.543235)
        self.assertEqual(balanced["right_ring_PIP_joint"]["kd"], 0.120927)

    def test_isaaclab_kwargs_use_delayed_pd_field_names(self):
        kwargs = isaaclab_actuator_kwargs()
        self.assertEqual(set(kwargs), {"stiffness", "damping", "armature", "friction",
                                       "viscous_friction", "effort_limit", "command_delay_s"})
        self.assertEqual(set(kwargs["stiffness"]), self.hand_joints)
        self.assertEqual(round(kwargs["command_delay_s"] / 0.005), 3)

    def test_unknown_profile_is_rejected(self):
        with self.assertRaises(KeyError):
            load_profile("does_not_exist")


if __name__ == "__main__":
    unittest.main()
