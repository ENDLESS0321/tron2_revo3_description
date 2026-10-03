import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
sys.path.insert(0, str(ROOT / "viewers"))

from _runtime import assembly, load_training_model, set_pose, verify_assets  # noqa: E402


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

    def test_runtime_models_and_release_metadata_are_synchronized(self):
        manifest = json.loads((ASSETS / "manifest.json").read_text())
        self.assertEqual(self.runtime["release"], manifest["release"])
        self.assertEqual(self.runtime["pose"], self.runtime["models"]["full"]["display_pose"])
        for name, expected in (("full", (58, 6)), ("training", (28, 0))):
            model, data, cfg = assembly(name)
            self.assertEqual((model.nq, model.ncam), expected)
            self.assertEqual(len(cfg["cameras"]), 3 if name == "full" else 0)
            self.assertEqual(cfg["selected_asset"], self.runtime["models"][name]["preview"])
            for joint_name, value in cfg["pose"].items():
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                self.assertGreaterEqual(joint_id, 0)
                self.assertAlmostEqual(data.qpos[model.jnt_qposadr[joint_id]], value)
            set_pose(model, data, {})
            np.testing.assert_allclose(data.qpos, model.qpos0)
        self.assertEqual(verify_assets()["training_referenced_meshes"], 128)

    def test_training_camera_selection_is_rejected_before_render(self):
        import subprocess

        result = subprocess.run([sys.executable, str(ROOT / "viewers/_viewer.py"),
                                 "--mode", "cameras", "--model", "training", "--check"],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 2)
        self.assertIn("supported only for assembly preview", result.stderr)

    def test_table_top_is_45_cm_below_robot_origin(self):
        urdf_origin = self.joint("world_to_base").find("origin")
        urdf_base_z = float(urdf_origin.get("xyz").split()[2])
        scene_base_z = float(self.scene_body("base_Link").get("pos").split()[2])
        table = self.scene.find("./worldbody/geom[@name='work_table']")
        table_center_z = float(table.get("pos").split()[2])
        table_half_height = float(table.get("size").split()[2])
        table_top_z = table_center_z + table_half_height
        self.assertAlmostEqual(urdf_base_z, 1.20035)
        self.assertAlmostEqual(scene_base_z - table_top_z, 0.45)
        layout = self.runtime["world_layout"]
        np.testing.assert_allclose(np.fromstring(table.get("size"), sep=" ") * 2, layout["table_size"])
        np.testing.assert_allclose(np.fromstring(table.get("pos"), sep=" "), layout["table_center"])
        np.testing.assert_allclose(np.fromstring(self.scene_body("base_Link").get("pos"), sep=" "), layout["base_pos"])
        np.testing.assert_allclose(np.fromstring(self.scene_body("base_Link").get("quat"), sep=" "), layout["base_quat_wxyz"])
        self.assertAlmostEqual(table_top_z, 0.507)
        for cube in self.scene.findall("./worldbody/geom"):
            if cube.get("name", "").startswith("test_cube_"):
                bottom_z = float(cube.get("pos").split()[2]) - float(cube.get("size").split()[2])
                self.assertAlmostEqual(bottom_z, table_top_z)


    def test_default_urdf_is_exact_approved_artifact(self):
        import hashlib

        expected = "537f31a798ddb05d1f29e2b5eeede63d47af3d9ff519f40907a24e757f5bbf44"
        self.assertEqual(hashlib.sha256((ASSETS / "assembly.urdf").read_bytes()).hexdigest(), expected)
        self.assertFalse((ASSETS / "assembly_bilateral_axis180.urdf").exists())
        origin = self.joint("world_to_base").find("origin")
        self.assertAlmostEqual(float(origin.get("xyz").split()[2]), 1.20035)

    def test_bilateral_mounts_preserve_finger_axis_and_mating_positions(self):
        for side, expected_palm in (("left", [1, 0, 0]), ("right", [1, 0, 0])):
            adapter = self.joint(f"{side}_adapter_mount").find("origin")
            hand = self.joint(f"{side}_hand_base_joint").find("origin")
            np.testing.assert_allclose(np.fromstring(adapter.get("xyz"), sep=" "), [-0.0317, 0, -0.0812])
            combined = Rotation.from_euler("xyz", np.fromstring(adapter.get("rpy"), sep=" ")).as_matrix() @ Rotation.from_euler("xyz", np.fromstring(hand.get("rpy"), sep=" ")).as_matrix()
            np.testing.assert_allclose(combined[:, 2], [0, 0, -1], atol=1e-10)
            np.testing.assert_allclose(combined[:, 0], expected_palm, atol=1e-10)

    def test_urdf_and_scene_mount_poses_match(self):
        models = [mujoco.MjModel.from_xml_path(str(ASSETS / filename))
                  for filename in ("assembly.urdf", "scene.xml")]
        states = [mujoco.MjData(model) for model in models]
        # Exercise both zero and nonzero arm poses using joint names, not ordering.
        for angle in (0.0, 0.2):
            for model, data in zip(models, states):
                for joint_id in range(model.njnt):
                    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                    if name and any(part in name for part in ("shoulder", "elbow", "wrist")):
                        data.qpos[model.jnt_qposadr[joint_id]] = angle
                mujoco.mj_forward(model, data)
            for side in ("left", "right"):
                for suffix in ("adapter_link", "hand_base_link", "wrist_camera_mount_frame"):
                    name = f"{side}_{suffix}"
                    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                           for model in models]
                    self.assertTrue(all(body_id >= 0 for body_id in ids))
                    layout = self.runtime["world_layout"]
                    wxyz = layout["base_quat_wxyz"]
                    rotation = Rotation.from_quat([*wxyz[1:], wxyz[0]]).as_matrix()
                    expected_pos = rotation @ (states[0].xpos[ids[0]] - [0, 0, 1.20035]) + layout["base_pos"]
                    expected_rotation = rotation @ states[0].xmat[ids[0]].reshape(3, 3)
                    np.testing.assert_allclose(expected_pos, states[1].xpos[ids[1]], atol=1e-9)
                    np.testing.assert_allclose(expected_rotation, states[1].xmat[ids[1]].reshape(3, 3), atol=1e-9)

    def test_training_reduced28_hash_topology_and_meshes(self):
        import hashlib

        path = ASSETS / "assembly_rl_convex.urdf"
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                         "2776f52b77dc46ecd27c46894373dfb0dbe41882740f46f9d7b699518d194034")
        training = ET.parse(path).getroot()
        joints = training.findall("joint")
        links = training.findall("link")
        self.assertEqual((len(links), len(joints)), (38, 37))
        moving = [joint for joint in joints if joint.get("type") != "fixed"]
        self.assertEqual(len(moving), 28)
        self.assertTrue(all(joint.get("type") == "revolute" for joint in moving))
        self.assertEqual(sum("_R_" in joint.get("name") for joint in moving), 7)
        self.assertEqual(sum(joint.get("name").startswith("right_") for joint in moving), 21)
        def structure(node):
            return (node.tag, node.attrib, (node.text or "").strip(),
                    [structure(child) for child in node])
        for joint in moving:
            self.assertEqual(structure(joint), structure(self.joint(joint.get("name"))))
        children = [joint.find("child").get("link") for joint in joints]
        self.assertEqual(len(set(children)), len(children))
        self.assertEqual(set(children), {link.get("name") for link in links} - {"world"})
        meshes = {mesh.get("filename") for mesh in training.findall(".//mesh")}
        self.assertEqual(len(meshes), 128)
        for filename in meshes:
            path = (ASSETS / filename).resolve()
            self.assertTrue(path.is_relative_to(ASSETS))
            self.assertTrue(path.is_file())
        self.assertEqual(sum(len(link.findall("collision")) for link in links), 58)
        for name, count in (("right_adapter_link", 1), ("right_hand_base_link", 28)):
            link = training.find(f"link[@name='{name}']")
            self.assertEqual(len(link.findall("collision")), count)
            for collision in link.findall("collision"):
                mesh = trimesh.load_mesh(ASSETS / collision.find("geometry/mesh").get("filename"), process=True)
                self.assertTrue(mesh.is_convex)
                self.assertTrue(mesh.is_watertight)
        for finger in ("thumb", "index", "middle", "ring", "little"):
            inertial = training.find(f"link[@name='right_{finger}_tip_Link']/inertial")
            self.assertAlmostEqual(float(inertial.find("mass").get("value")), 0.001)
            np.testing.assert_allclose(np.fromstring(inertial.find("origin").get("xyz"), sep=" "), [0, 0, 0])
            inertia = inertial.find("inertia")
            np.testing.assert_allclose([float(inertia.get(k)) for k in ("ixx", "iyy", "izz")],
                                       [1e-10, 1e-10, 1e-9], rtol=1e-7, atol=0)

    def test_training_reduced28_import_and_right_chain_fk(self):
        training = load_training_model()
        assembly = mujoco.MjModel.from_xml_path(str(ASSETS / "assembly.urdf"))
        self.assertEqual((training.nq, training.njnt), (28, 28))
        collision_mask = (training.geom_contype != 0) | (training.geom_conaffinity != 0)
        self.assertEqual(int(collision_mask.sum()), 58)
        self.assertEqual(int(collision_mask.sum()), self.runtime["models"]["training"]["collision_shapes"])
        for name, count in (("right_adapter_link", 1), ("right_hand_base_link", 28)):
            body_id = mujoco.mj_name2id(training, mujoco.mjtObj.mjOBJ_BODY, name)
            self.assertGreaterEqual(body_id, 0)
            self.assertEqual(int(((training.geom_bodyid == body_id) & collision_mask).sum()), count)
        moving = [joint for joint in ET.parse(ASSETS / "assembly_rl_convex.urdf").findall("joint")
                  if joint.get("type") == "revolute"]
        states = [mujoco.MjData(model) for model in (training, assembly)]
        rng = np.random.default_rng(20260928)
        bodies = [joint.find("child").get("link") for joint in moving] + ["right_adapter_link", "right_hand_base_link", "right_palm"]
        for _ in range(10):
            for joint in moving:
                limit = joint.find("limit")
                value = float(rng.uniform(float(limit.get("lower")), float(limit.get("upper"))))
                for model, data in zip((training, assembly), states):
                    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint.get("name"))
                    self.assertGreaterEqual(joint_id, 0)
                    data.qpos[model.jnt_qposadr[joint_id]] = value
            for model, data in zip((training, assembly), states):
                mujoco.mj_forward(model, data)
            for name in bodies:
                ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for model in (training, assembly)]
                self.assertTrue(all(body_id >= 0 for body_id in ids))
                np.testing.assert_allclose(states[0].xpos[ids[0]], states[1].xpos[ids[1]], atol=1e-9)
                np.testing.assert_allclose(states[0].xmat[ids[0]], states[1].xmat[ids[1]], atol=1e-9)

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

    def test_hands_and_wrist_camera_brackets_are_flipped_about_finger_axis(self):
        expected_urdf_rpy = {
            "left_hand_base_joint": "-1.57079632679 -1.57079632679 0",
            "right_hand_base_joint": "-1.57079632679 1.57079632679 0",
            "left_wrist_camera_reference_joint": "1.57079632679 0 1.57079632679979",
            "right_wrist_camera_reference_joint": "1.57079632679 0 -1.57079632679979",
        }
        for joint_name, expected_rpy in expected_urdf_rpy.items():
            with self.subTest(joint=joint_name):
                self.assertEqual(self.joint(joint_name).find("origin").get("rpy"), expected_rpy)

        for joint_name in (
            "left_wrist_camera_reference_joint",
            "right_wrist_camera_reference_joint",
        ):
            with self.subTest(joint=joint_name):
                self.assertEqual(
                    self.joint(joint_name).find("origin").get("xyz"),
                    "-0.0317 -1.31006316906e-18 -0.0753",
                )

        expected_scene_quat = {
            "left_hand_base_link": "0.5 -0.5 -0.5 -0.5",
            "right_hand_base_link": "0.5 -0.5 0.5 0.5",
            "left_wrist_camera_mount_frame": "0.5 0.5 0.5 0.5",
            "right_wrist_camera_mount_frame": "0.5 0.5 -0.5 -0.5",
        }
        for body_name, expected_quat in expected_scene_quat.items():
            with self.subTest(body=body_name):
                actual = np.fromstring(self.scene_body(body_name).get("quat"), sep=" ")
                expected = np.fromstring(expected_quat, sep=" ")
                np.testing.assert_allclose(actual, expected, atol=1e-10)

        for body_name in (
            "left_wrist_camera_mount_frame",
            "right_wrist_camera_mount_frame",
        ):
            with self.subTest(body=body_name):
                self.assertEqual(self.scene_body(body_name).get("pos"), "-0.0317 0 -0.0753")

        original_offset = np.array([-0.0317, 0.0, -0.0753])
        original_camera_rpy = {
            "left": [np.pi / 2, 0.0, -np.pi / 2],
            "right": [np.pi / 2, 0.0, np.pi / 2],
        }
        for model_path in (ASSETS / "assembly.urdf", ASSETS / "scene.xml"):
            model = mujoco.MjModel.from_xml_path(str(model_path))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            for side, suffix in (("left", "L"), ("right", "R")):
                wrist_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    f"wrist_roll_{suffix}_Link",
                )
                camera_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    f"{side}_wrist_camera_mount_frame",
                )
                hand_id = mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    f"{side}_hand_base_link",
                )
                wrist_rotation = data.xmat[wrist_id].reshape(3, 3)
                camera_rotation = data.xmat[camera_id].reshape(3, 3)
                hand_rotation = data.xmat[hand_id].reshape(3, 3)
                relative_offset = wrist_rotation.T @ (
                    data.xpos[camera_id] - data.xpos[wrist_id]
                )
                relative_rotation = wrist_rotation.T @ camera_rotation
                finger_axis = (
                    wrist_rotation.T @ hand_rotation @ np.array([0.0, 0.0, 1.0])
                )
                bracket_half_turn = Rotation.from_rotvec(
                    np.pi * finger_axis
                ).as_matrix()

                with self.subTest(model=model_path.name, side=side):
                    np.testing.assert_allclose(
                        finger_axis,
                        [0.0, 0.0, -1.0],
                        atol=1e-9,
                    )
                    np.testing.assert_allclose(
                        relative_offset,
                        original_offset,
                        atol=1e-9,
                    )
                    np.testing.assert_allclose(
                        relative_rotation,
                        bracket_half_turn
                        @ Rotation.from_euler(
                            "xyz", original_camera_rpy[side]
                        ).as_matrix(),
                        atol=1e-9,
                    )

        self.assertEqual(
            self.joint("head_camera_reference_joint").find("origin").get("rpy"),
            "0 0 0",
        )


if __name__ == "__main__":
    unittest.main()
