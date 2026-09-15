#!/usr/bin/env python3
"""Publish the local MuJoCo camera rig as isolated ROS 2 RGB-D topics.

Uses sensor_msgs directly, without cv_bridge. Import CameraPublisher for an
in-process simulation integration; importing this file does not start ROS or
load the simulator. The default domain is 94 and discovery is localhost-only.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CAMERAS = ("head", "left_wrist", "right_wrist")


def add_ros_python_paths():
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    for path in (
        Path("/opt/ros/humble/local/lib")/version/"dist-packages",
        Path("/opt/ros/humble/lib")/version/"site-packages",
    ):
        if path.is_dir() and str(path) not in sys.path:
            sys.path.append(str(path))


add_ros_python_paths()
try:
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
except ImportError as exc:
    raise ImportError("Use this project's .venv/bin/python (Python 3.10) with the installed ROS 2 Humble packages") from exc


def time_message(stamp_ns):
    if not isinstance(stamp_ns, (int, np.integer)) or stamp_ns < 0:
        raise ValueError("stamp_ns must be a nonnegative integer simulation timestamp")
    sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def valid_frame_name(frame):
    if not isinstance(frame, str) or not frame or frame.startswith("/") or any(c.isspace() for c in frame):
        raise ValueError(f"Invalid TF frame name: {frame!r}")
    return frame


def intrinsic_matrix(values, width, height):
    k = np.asarray(values, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("Camera intrinsics must be finite with positive focal lengths")
    if not np.allclose(k[2], [0, 0, 1], rtol=0, atol=1e-10):
        raise ValueError("Camera K must end in [0, 0, 1]")
    if not 0 <= k[0, 2] <= width or not 0 <= k[1, 2] <= height:
        raise ValueError("Principal point is outside the image")
    return k


def tf_message(record, stamp):
    parent = valid_frame_name(record["parent"])
    child = valid_frame_name(record["child"])
    if parent == child:
        raise ValueError(f"Self-parent TF: {child}")
    xyz = np.asarray(record["xyz"], dtype=float)
    quat = np.asarray(record["quat_xyzw"], dtype=float)
    if xyz.shape != (3,) or quat.shape != (4,) or not np.isfinite(xyz).all() or not np.isfinite(quat).all():
        raise ValueError(f"Malformed/non-finite TF: {parent} -> {child}")
    if abs(float(np.linalg.norm(quat))-1) > 1e-5:
        raise ValueError(f"TF quaternion is not normalized: {parent} -> {child}")
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = map(float, xyz)
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = map(float, quat)
    return message


class CameraPublisher(Node):
    """Publish validated synchronized frame bundles from camera_rig.create_source.

    All RGB, depth, CameraInfo and dynamic TF messages in one bundle share its
    simulation stamp_ns. RGB and depth frame IDs and K matrices may differ.
    """

    def __init__(self, camera_specs, static_transforms, topic_prefix="/tron2_sim", **kwargs):
        super().__init__("tron2_sim_camera_bridge", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)], **kwargs)
        self.topic_prefix = "/" + topic_prefix.strip("/")
        self.specs = {spec["name"]: dict(spec) for spec in camera_specs}
        if len(self.specs) != len(camera_specs) or set(self.specs) != set(CAMERAS):
            raise ValueError(f"camera_specs must contain exactly {CAMERAS}")
        self.publishers_by_camera = {}
        for name, spec in self.specs.items():
            width, height = int(spec["width"]), int(spec["height"])
            if width <= 0 or height <= 0:
                raise ValueError(f"{name}: invalid camera resolution")
            spec["width"], spec["height"] = width, height
            for stream in ("color", "depth"):
                spec[f"{stream}_frame_id"] = valid_frame_name(spec.get(f"{stream}_frame_id", spec.get("frame_id")))
                spec[f"{stream}_K"] = intrinsic_matrix(spec.get(f"{stream}_K", spec.get("K")), width, height)
            self.publishers_by_camera[name] = {
                stream: {
                    "image": self.create_publisher(Image, f"{self.topic_prefix}/{name}/{stream}/image_raw", qos_profile_sensor_data),
                    "info": self.create_publisher(CameraInfo, f"{self.topic_prefix}/{name}/{stream}/camera_info", qos_profile_sensor_data),
                }
                for stream in ("color", "depth")
            }
        clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.clock_publisher = self.create_publisher(Clock, "/clock", clock_qos)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_children = {record["child"] for record in static_transforms}
        if len(self.static_children) != len(static_transforms):
            raise ValueError("Duplicate static TF child frame")
        if not static_transforms:
            raise ValueError("The rig must provide its static camera-link/optical transforms")
        self.static_broadcaster.sendTransform([tf_message(record, time_message(0)) for record in static_transforms])
        self.last_stamp_ns = -1
        self.published_bundles = 0
        self.last_depth_stats = {}

    def camera_info(self, spec, stream, stamp, k):
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = spec[f"{stream}_frame_id"]
        message.width, message.height = spec["width"], spec["height"]
        message.distortion_model = "plumb_bob"
        message.d = [0.] * 5
        message.k = k.reshape(-1).tolist()
        message.r = np.eye(3).reshape(-1).tolist()
        projection = np.zeros((3, 4))
        projection[:, :3] = k
        message.p = projection.reshape(-1).tolist()
        return message

    def publish_bundle(self, bundle):
        stamp_ns = bundle["stamp_ns"]
        stamp = time_message(stamp_ns)
        if stamp_ns <= self.last_stamp_ns:
            raise ValueError("Simulation timestamps must increase strictly between frame bundles")
        frames = bundle["frames"]
        if set(frames) != set(CAMERAS):
            raise ValueError("Every bundle must contain head, left_wrist, and right_wrist frames")
        dynamic = bundle["transforms"]
        dynamic_children = [record["child"] for record in dynamic]
        if not dynamic or len(set(dynamic_children)) != len(dynamic_children):
            raise ValueError("Dynamic TF tree must be nonempty and have unique child frames")
        if self.static_children & set(dynamic_children):
            raise ValueError("A frame cannot be published on both /tf and /tf_static")
        tf_messages = [tf_message(record, stamp) for record in dynamic]
        pending = []
        for name in CAMERAS:
            spec, frame = self.specs[name], frames[name]
            width, height = spec["width"], spec["height"]
            rgb, depth = np.asarray(frame["rgb"]), np.asarray(frame["depth"])
            if rgb.dtype != np.uint8 or rgb.shape != (height, width, 3):
                raise ValueError(f"{name}: RGB must be uint8 HxWx3")
            if depth.dtype != np.float32 or depth.shape != (height, width):
                raise ValueError(f"{name}: depth must be metric float32 HxW")
            finite = np.isfinite(depth)
            valid = finite & (depth > 0)
            if np.isinf(depth).any() or (depth[finite] < 0).any() or not valid.any():
                raise ValueError(f"{name}: depth contains Inf/negative finite values or no positive finite pixels")
            declared_range = spec.get("depth_range_m")
            if declared_range is not None:
                lo, hi = map(float, declared_range)
                if not (math.isfinite(lo) and math.isfinite(hi) and 0 <= lo < hi):
                    raise ValueError(f"{name}: invalid configured simulation depth range")
                if (depth[valid] < lo-1e-5).any() or (depth[valid] > hi+1e-5).any():
                    raise ValueError(f"{name}: valid depth exceeds the configured simulation range")
            self.last_depth_stats[name] = {
                "valid_fraction":float(valid.mean()), "nan_fraction":float(np.isnan(depth).mean()),
                "zero_fraction":float((depth == 0).mean()),
                "valid_min_m":float(depth[valid].min()), "valid_max_m":float(depth[valid].max()),
                "configured_simulation_range_m":declared_range,
            }
            for stream, values, encoding, channels, itemsize in (
                ("color", rgb, "rgb8", 3, 1),
                ("depth", depth, "32FC1", 1, 4),
            ):
                k = intrinsic_matrix(frame.get(f"{stream}_K", spec[f"{stream}_K"]), width, height)
                image = Image()
                image.header.stamp = stamp
                image.header.frame_id = spec[f"{stream}_frame_id"]
                image.height, image.width = height, width
                image.encoding = encoding
                image.is_bigendian = 0
                image.step = width * channels * itemsize
                image.data = np.ascontiguousarray(values, dtype="<f4" if stream == "depth" else np.uint8).tobytes()
                pending.append((self.publishers_by_camera[name][stream], image, self.camera_info(spec, stream, stamp, k)))
        # Validate the complete bundle before publishing any of its messages.
        self.clock_publisher.publish(Clock(clock=stamp))
        self.dynamic_broadcaster.sendTransform(tf_messages)
        for publishers, image, info in pending:
            publishers["info"].publish(info)
            publishers["image"].publish(image)
        self.last_stamp_ns = int(stamp_ns)
        self.published_bundles += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=0, help="0 keeps publishing until interrupted")
    parser.add_argument("--rate", type=float, default=30., help="Maximum wall-clock publication rate; simulation timestamps come from the rig")
    parser.add_argument("--startup-wait", type=float, default=2., help="DDS discovery time before the first capture")
    parser.add_argument("--domain-id", type=int, default=94)
    parser.add_argument("--topic-prefix", default="/tron2_sim")
    args = parser.parse_args()
    if args.frames < 0 or args.width <= 0 or args.height <= 0 or not math.isfinite(args.rate) or args.rate <= 0 or args.startup_wait < 0:
        parser.error("Invalid frame count, dimensions, rate or startup wait")
    if not 1 <= args.domain_id <= 100:
        parser.error("Use an isolated non-default domain in 1..100; this project's default is 94")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["MUJOCO_GL"] = "egl"
    if str(ROOT/"scripts") not in sys.path:
        sys.path.insert(0, str(ROOT/"scripts"))
    rig = importlib.import_module("camera_rig")
    rig_hash = hashlib.sha256(Path(rig.__file__).read_bytes()).hexdigest()
    source = None
    node = None
    executor = None
    context = rclpy.context.Context()
    try:
        source = rig.create_source(width=args.width, height=args.height, config_path=args.config)
        loaded_model_files = {}
        for attribute in ("urdf_path", "scene_path"):
            model_path = getattr(source, attribute, None)
            if model_path is not None:
                model_path = Path(model_path).resolve()
                loaded_model_files[attribute] = {"path":str(model_path), "sha256":hashlib.sha256(model_path.read_bytes()).hexdigest()}
        rclpy.init(args=[], context=context, domain_id=args.domain_id)
        executor = SingleThreadedExecutor(context=context)
        node = CameraPublisher(source.camera_specs, source.static_transforms, args.topic_prefix, context=context)
        deadline = time.monotonic()+args.startup_wait
        while time.monotonic() < deadline and context.ok():
            rclpy.spin_once(node, executor=executor, timeout_sec=min(.05, max(0, deadline-time.monotonic())))
        config_path = Path(getattr(source, "config_path", args.config or ROOT/"config/camera_rig.json"))
        print(json.dumps({"event":"camera_bridge_ready", "domain_id":args.domain_id, "localhost_only":True, "camera_names":list(CAMERAS), "width":args.width, "height":args.height, "topic_prefix":node.topic_prefix,
                          "rig_sha256":rig_hash, "bridge_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "config_sha256":hashlib.sha256(config_path.read_bytes()).hexdigest(),
                          "loaded_model_files":loaded_model_files}), flush=True)
        while context.ok() and (args.frames == 0 or node.published_bundles < args.frames):
            started = time.monotonic()
            node.publish_bundle(source.step_and_capture())
            rclpy.spin_once(node, executor=executor, timeout_sec=0)
            remaining = 1./args.rate-(time.monotonic()-started)
            if remaining > 0:
                time.sleep(remaining)
        print(json.dumps({"event":"camera_bridge_finished", "bundles":node.published_bundles, "last_stamp_ns":node.last_stamp_ns, "last_depth_stats":node.last_depth_stats,
                          "rig_unchanged_during_run":hashlib.sha256(Path(rig.__file__).read_bytes()).hexdigest()==rig_hash,
                          "loaded_model_files_unchanged_during_run":all(hashlib.sha256(Path(item["path"]).read_bytes()).hexdigest()==item["sha256"] for item in loaded_model_files.values())}), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
        if source is not None:
            source.close()


if __name__ == "__main__":
    main()
