import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco
import trimesh


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
sys.path.insert(0, str(ROOT / "viewers"))

from _runtime import verify_assets  # noqa: E402


class ReleaseAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = ET.parse(ASSETS / "assembly.urdf").getroot()
        cls.scene = ET.parse(ASSETS / "scene.xml").getroot()
        cls.runtime = json.loads((ASSETS / "runtime.json").read_text())

    def link(self, name):
        return self.urdf.find(f"./link[@name='{name}']")

    def joint(self, name):
        return self.urdf.find(f"./joint[@name='{name}']")

    def scene_body(self, name):
        return self.scene.find(f".//body[@name='{name}']")

    def color(self, link_name):
        return self.link(link_name).find("./visual/material/color").get("rgba")

    def test_manifest_and_models_load(self):
        self.assertEqual(verify_assets()["referenced_meshes"], 265)
        urdf = mujoco.MjModel.from_xml_path(str(ASSETS / "assembly.urdf"))
        scene = mujoco.MjModel.from_xml_path(str(ASSETS / "scene.xml"))
        self.assertEqual((urdf.nq, scene.nq, scene.ncam), (58, 58, 6))

    def test_reference_palette_is_applied(self):
        self.assertEqual(self.color("base_Link"), "0.15 0.16 0.17 1")
        self.assertEqual(self.color("left_adapter_link"), "0.42 0.20 0.68 1")
        self.assertEqual(self.color("right_adapter_link"), "0.42 0.20 0.68 1")
        self.assertEqual(self.color("left_hand_base_link"), "0.86 0.87 0.92 1")
        self.assertEqual(self.color("left_index_MPR_Link"), "0.46 0.49 0.58 1")
        self.assertEqual(self.color("left_index_MCP_Link"), "0.60 0.62 0.70 1")
        self.assertEqual(self.color("left_index_DIP_Link"), "0.70 0.72 0.80 1")

    def test_head_camera_is_d455(self):
        mesh = self.link("head_camera_link").find("./visual/geometry/mesh")
        self.assertEqual(mesh.get("filename"), "meshes/cameras/d455_visual_mm.stl")
        self.assertEqual(mesh.get("scale"), "0.001 0.001 0.001")
        self.assertEqual(
            self.joint("head_camera_link_joint").find("origin").get("xyz"),
            "0.01115 0.0475 0.0145",
        )
        self.assertEqual(
            self.joint("head_camera_color_joint").find("origin").get("xyz"),
            "0 -0.059 0",
        )
        required_frames = {
            "head_camera_depth_optical_frame",
            "head_camera_color_optical_frame",
            "head_camera_infra1_optical_frame",
            "head_camera_infra2_optical_frame",
            "head_camera_accel_optical_frame",
            "head_camera_gyro_optical_frame",
            "head_camera_imu_optical_frame",
        }
        links = {node.get("name") for node in self.urdf.findall("link")}
        self.assertTrue(required_frames <= links)
        self.assertNotIn("d435i_visual_m.obj", ET.tostring(self.urdf, encoding="unicode"))

    def test_d455_mesh_and_runtime_parameters(self):
        mesh = trimesh.load_mesh(ASSETS / "meshes/cameras/d455_visual_mm.stl", process=False)
        self.assertAlmostEqual(mesh.extents[0], 124.0, delta=0.01)
        self.assertAlmostEqual(mesh.extents[1], 29.0, delta=0.01)
        self.assertAlmostEqual(mesh.extents[2], 26.0, delta=0.01)
        head = next(camera for camera in self.runtime["cameras"] if camera["name"] == "head")
        self.assertEqual(head["model"], "D455")
        self.assertEqual(head["color_fov_deg"], [90, 65])
        self.assertEqual(head["depth_fov_deg"], [87, 58])
        self.assertEqual(head["depth_range_m"], [0.6, 6.0])

    def test_hands_and_wrist_cameras_are_axially_flipped(self):
        expected_urdf_rpy = {
            "left_hand_base_joint": "-1.57079632679 -1.57079632679 0",
            "right_hand_base_joint": "-1.57079632679 1.57079632679 0",
            "left_wrist_camera_reference_joint": "-1.57079632679 0 -1.57079632679",
            "right_wrist_camera_reference_joint": "-1.57079632679 0 1.57079632679",
        }
        for joint_name, expected_rpy in expected_urdf_rpy.items():
            with self.subTest(joint=joint_name):
                self.assertEqual(self.joint(joint_name).find("origin").get("rpy"), expected_rpy)

        expected_scene_quat = {
            "left_hand_base_link": "0.5 -0.5 -0.5 -0.5",
            "right_hand_base_link": "0.5 -0.5 0.5 0.5",
            "left_wrist_camera_mount_frame": "0.5 -0.5 0.5 -0.5",
            "right_wrist_camera_mount_frame": "0.5 -0.5 -0.5 0.5",
        }
        for body_name, expected_quat in expected_scene_quat.items():
            with self.subTest(body=body_name):
                self.assertEqual(self.scene_body(body_name).get("quat"), expected_quat)

        self.assertEqual(
            self.joint("head_camera_reference_joint").find("origin").get("rpy"),
            "0 0 0",
        )


if __name__ == "__main__":
    unittest.main()
