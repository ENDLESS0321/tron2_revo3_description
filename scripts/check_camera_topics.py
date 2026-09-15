#!/usr/bin/env python3
"""Subscribe to the isolated RGB-D bridge and validate real ROS 2 messages.

Requires at least three synchronized nonzero simulation stamps across all
three cameras, plus exact-time TF reachability and /clock correspondence.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np

from camera_ros_bridge import CAMERAS, ROOT, valid_frame_name, intrinsic_matrix
import rclpy
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Invalid/nonpositive ROS simulation timestamp")
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


class TopicChecker(Node):
    def __init__(self, args, **kwargs):
        super().__init__("tron2_sim_camera_topic_check", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)], **kwargs)
        self.args = args
        config_bytes = args.config.read_bytes()
        config = json.loads(config_bytes)
        self.config_sha256 = hashlib.sha256(config_bytes).hexdigest()
        self.depth_ranges = {}
        for name, part in (("head", config["head"]), ("left_wrist", config["wrists"]["left"]), ("right_wrist", config["wrists"]["right"])):
            self.depth_ranges[name] = list(map(float, config["models"][part["model"]]["depth_range_m"]))
        self.errors = []
        self.counts = defaultdict(int)
        self.last_stamps = {}
        self.received = {name: defaultdict(dict) for name in CAMERAS}
        self.clock_stamps = set()
        self.clock_last = -1
        self.parents = {}
        self.tf_children = {"dynamic": set(), "static": set()}
        self.tf_message_counts = defaultdict(int)
        self.tf_validated = {}
        self.complete_stamps = []
        self.buffer = Buffer(cache_time=Duration(seconds=60.), node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.subscriptions_kept = []
        prefix = "/" + args.topic_prefix.strip("/")
        for name in CAMERAS:
            for stream in ("color", "depth"):
                for suffix, message_type, kind in (("image_raw", Image, "image"), ("camera_info", CameraInfo, "info")):
                    topic = f"{prefix}/{name}/{stream}/{suffix}"
                    self.subscriptions_kept.append(self.create_subscription(message_type, topic, lambda message, n=name, s=stream, k=kind: self.callback(n,s,k,message), qos_profile_sensor_data))
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_kept.append(self.create_subscription(Clock, "/clock", self.clock_callback, reliable))
        self.subscriptions_kept.append(self.create_subscription(TFMessage, "/tf", lambda m:self.tf_callback("dynamic",m), reliable))
        self.subscriptions_kept.append(self.create_subscription(TFMessage, "/tf_static", lambda m:self.tf_callback("static",m), static))

    def error(self, message):
        if message not in self.errors and len(self.errors) < 200:
            self.errors.append(message)

    def clock_callback(self, message):
        try:
            stamp = stamp_ns(message.clock)
            if stamp < self.clock_last:
                raise ValueError("/clock moved backward")
            self.clock_last = stamp
            self.clock_stamps.add(stamp)
            self.counts["/clock"] += 1
        except ValueError as exc:
            self.error(str(exc))

    def tf_callback(self, kind, message):
        self.tf_message_counts[kind] += 1
        for transform in message.transforms:
            try:
                parent = valid_frame_name(transform.header.frame_id)
                child = valid_frame_name(transform.child_frame_id)
                if parent == child:
                    raise ValueError(f"Self-parent TF {child}")
                prior = self.parents.get(child)
                if prior is not None and prior != parent:
                    raise ValueError(f"TF {child} has conflicting parents {prior} and {parent}")
                t, q = transform.transform.translation, transform.transform.rotation
                values = np.array([t.x,t.y,t.z,q.x,q.y,q.z,q.w])
                if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1) > 1e-5:
                    raise ValueError(f"Non-finite TF or non-unit quaternion: {child}")
                timestamp = stamp_ns(transform.header.stamp)
                self.parents[child] = parent
                self.tf_children[kind].add(child)
                if self.tf_children["dynamic"] & self.tf_children["static"]:
                    raise ValueError("A child frame appears in both dynamic and static TF")
            except ValueError as exc:
                self.error(str(exc))

    def callback(self, name, stream, kind, message):
        key = f"{name}/{stream}/{kind}"
        self.counts[key] += 1
        try:
            timestamp = stamp_ns(message.header.stamp)
            previous = self.last_stamps.get(key, -1)
            if timestamp <= previous:
                raise ValueError("timestamps are not strictly increasing")
            self.last_stamps[key] = timestamp
            frame = valid_frame_name(message.header.frame_id)
            if "optical" not in frame.lower():
                raise ValueError(f"image/CameraInfo frame is not named as optical: {frame}")
            if message.width != self.args.width or message.height != self.args.height:
                raise ValueError(f"resolution {message.width}x{message.height} differs from expected {self.args.width}x{self.args.height}")
            record = {"frame_id":frame,"width":message.width,"height":message.height}
            if kind == "image":
                expected_encoding = "rgb8" if stream == "color" else "32FC1"
                pixel_bytes = 3 if stream == "color" else 4
                if message.encoding != expected_encoding or message.is_bigendian != 0:
                    raise ValueError(f"expected little-endian {expected_encoding}, got {message.encoding}/{message.is_bigendian}")
                if message.step != message.width*pixel_bytes or len(message.data) != message.step*message.height:
                    raise ValueError("step or byte count is incorrect")
                record.update(encoding=message.encoding,step=message.step,byte_count=len(message.data))
                if stream == "depth":
                    values = np.frombuffer(message.data,dtype="<f4")
                    finite = np.isfinite(values)
                    valid = finite & (values > 0)
                    nan_count = int(np.isnan(values).sum())
                    inf_count = int(np.isinf(values).sum())
                    negative = int((values[finite]<0).sum())
                    positive = int(valid.sum())
                    if inf_count or negative or not positive:
                        raise ValueError("depth has Inf/negative finite values or no positive finite measurements")
                    lo, hi = self.depth_ranges[name]
                    if (values[valid]<lo-1e-5).any() or (values[valid]>hi+1e-5).any():
                        raise ValueError("valid depth lies outside the configured simulation range")
                    record.update(nan_pixels=nan_count,infinite_pixels=inf_count,negative_pixels=negative,positive_pixels=positive,zero_pixels=int((values==0).sum()),valid_fraction=float(valid.mean()),nan_fraction=float(np.isnan(values).mean()),depth_min_m=float(values[valid].min()),depth_max_m=float(values[valid].max()),configured_simulation_range_m=[lo,hi])
                else:
                    values = np.frombuffer(message.data,dtype=np.uint8)
                    record["pixel_std"] = float(values.std())
            else:
                k = intrinsic_matrix(message.k,message.width,message.height)
                r = np.asarray(message.r).reshape(3,3)
                p = np.asarray(message.p).reshape(3,4)
                if not np.isfinite(message.d).all() or not np.isfinite(r).all() or not np.isfinite(p).all():
                    raise ValueError("CameraInfo calibration contains non-finite values")
                if message.distortion_model != "plumb_bob" or len(message.d) != 5 or not np.allclose(message.d,0,rtol=0,atol=1e-12):
                    raise ValueError("Expected five zero plumb_bob coefficients for the pinhole simulation")
                if not np.allclose(r,np.eye(3),rtol=0,atol=1e-10) or not np.allclose(p[:,:3],k,rtol=0,atol=1e-10) or not np.allclose(p[:,3],0,rtol=0,atol=1e-10):
                    raise ValueError("CameraInfo R/P is inconsistent with the unrectified pinhole K")
                record.update(K=k.reshape(-1).tolist(),distortion_model=message.distortion_model,D=list(message.d))
            self.received[name][timestamp][f"{stream}_{kind}"] = record
        except (ValueError,TypeError,OverflowError) as exc:
            self.error(f"{key}: {exc}")

    def update_complete(self):
        required = {"color_image","depth_image","color_info","depth_info"}
        candidates = set(self.clock_stamps)
        for name in CAMERAS:
            candidates &= {stamp for stamp,records in self.received[name].items() if stamp > 0 and required <= set(records)}
        complete = []
        for timestamp in sorted(candidates):
            valid = True
            frames = set()
            for name in CAMERAS:
                records = self.received[name][timestamp]
                for stream in ("color","depth"):
                    image, info = records[f"{stream}_image"], records[f"{stream}_info"]
                    if image["frame_id"] != info["frame_id"]:
                        self.error(f"{name}/{stream}: image and CameraInfo frame IDs differ at {timestamp}")
                        valid = False
                    frames.add(image["frame_id"])
            tf_records = {}
            for frame in sorted(frames):
                try:
                    transform = self.buffer.lookup_transform(self.args.world_frame,frame,Time(nanoseconds=timestamp,clock_type=ClockType.ROS_TIME))
                    t,q = transform.transform.translation,transform.transform.rotation
                    values = [t.x,t.y,t.z,q.x,q.y,q.z,q.w]
                    if not np.isfinite(values).all():
                        self.error(f"Non-finite composed TF {self.args.world_frame} -> {frame}")
                        valid = False
                    tf_records[frame] = {"xyz":values[:3],"quat_xyzw":values[3:]}
                except TransformException:
                    valid = False
            if valid:
                complete.append(timestamp)
                self.tf_validated[timestamp] = tf_records
        self.complete_stamps = complete
        return len(complete) >= self.args.minimum_frames and bool(self.tf_children["dynamic"]) and bool(self.tf_children["static"])

    def result(self, elapsed):
        enough = self.update_complete()
        if not enough:
            self.error(f"Only {len(self.complete_stamps)} complete synchronized nonzero stamps with /clock and exact-time TF; need {self.args.minimum_frames}")
        for kind in ("dynamic","static"):
            if not self.tf_children[kind]:
                self.error(f"No {kind} TF was received")
        # Check the received aggregate TF graph for a cycle independently of Buffer.
        for start in self.parents:
            seen=set();frame=start
            while frame in self.parents:
                if frame in seen:
                    self.error(f"Cycle in TF tree at {frame}")
                    break
                seen.add(frame);frame=self.parents[frame]
        samples = {}
        chosen = self.complete_stamps[:self.args.minimum_frames]
        for name in CAMERAS:
            samples[name] = [{"stamp_ns":stamp,**self.received[name][stamp]} for stamp in chosen]
        return {
            "schema":"tron2_sim_ros_camera_validation_v1",
            "created_utc":datetime.now(timezone.utc).isoformat(),
            "valid":not self.errors,
            "scope":"Actual ROS 2 subscription: image payloads, calibration metadata, shared simulation stamps, /clock, and exact-time world-to-optical TF. Does not verify physical bracket geometry or depth reprojection; those belong to rig validation.",
            "domain_id":self.args.domain_id,"localhost_only":True,
            "config":str(self.args.config.resolve()),"config_sha256":self.config_sha256,
            "depth_semantics":"32FC1 metric optical-Z; NaN marks invalid/out-of-range pixels and is preserved; Inf and negative finite depth are rejected.",
            "resolution":[self.args.width,self.args.height],"minimum_frames":self.args.minimum_frames,
            "elapsed_seconds":elapsed,"message_counts":dict(self.counts),
            "complete_synchronized_stamp_count":len(self.complete_stamps),
            "validated_stamp_ns":chosen,"camera_samples":samples,
            "tf_message_counts":dict(self.tf_message_counts),
            "tf_children":{kind:sorted(frames) for kind,frames in self.tf_children.items()},
            "tf_parent_map":self.parents,
            "world_frame":self.args.world_frame,
            "world_to_optical_at_validated_stamps":{str(stamp):self.tf_validated[stamp] for stamp in chosen},
            "errors":self.errors,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id",type=int,default=94)
    parser.add_argument("--config",type=Path,default=ROOT/"config/camera_rig.json")
    parser.add_argument("--topic-prefix",default="/tron2_sim")
    parser.add_argument("--width",type=int,default=640)
    parser.add_argument("--height",type=int,default=480)
    parser.add_argument("--minimum-frames",type=int,default=3)
    parser.add_argument("--timeout",type=float,default=45.)
    parser.add_argument("--world-frame",default="world")
    parser.add_argument("--output",type=Path,default=ROOT/"reports/cameras/ros_topic_validation.json")
    args=parser.parse_args()
    if not 1<=args.domain_id<=100 or args.width<=0 or args.height<=0 or args.minimum_frames<3 or not np.isfinite(args.timeout) or args.timeout<=0:
        parser.error("Use domain 1..100, positive dimensions/timeout, and at least three frames")
    os.environ["ROS_DOMAIN_ID"]=str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"]="1"
    context=rclpy.context.Context();node=None;report=None;executor=None
    try:
        rclpy.init(args=[],context=context,domain_id=args.domain_id)
        executor=SingleThreadedExecutor(context=context)
        node=TopicChecker(args,context=context)
        started=time.monotonic()
        print(json.dumps({"event":"camera_checker_ready","domain_id":args.domain_id,"localhost_only":True}),flush=True)
        while context.ok() and time.monotonic()-started<args.timeout:
            rclpy.spin_once(node,executor=executor,timeout_sec=.05)
            if node.update_complete():
                break
        report=node.result(time.monotonic()-started)
    except KeyboardInterrupt:
        if node is not None:
            report=node.result(0.)
            report["errors"].append("Interrupted before normal completion")
            report["valid"]=False
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
    if report is None:
        return 1
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({"valid":report["valid"],"synchronized_frames":report["complete_synchronized_stamp_count"],"errors":report["errors"],"report":str(args.output.resolve())},indent=2,ensure_ascii=False))
    return 0 if report["valid"] else 1


if __name__=="__main__":
    raise SystemExit(main())
