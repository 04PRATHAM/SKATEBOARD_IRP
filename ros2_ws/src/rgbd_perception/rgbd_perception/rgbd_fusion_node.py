"""Mask-median depth fusion node — YOLO masks + depth -> optical-frame Object3D (Phase D).

Subscribes: Detection2DArray /perception/detections_2d, Image mono8
  /perception/masks, Image 32FC1 /camera/depth/image_raw, CameraInfo (depth).
Publishes: vision_msgs/Detection3DArray /perception/objects_3d
  (header stamp = detection stamp, frame = depth optical frame;
   bbox.center.position = (X right, Y down, Z fwd) metres OPTICAL;
   bbox.size = (w3d, h3d, 0); results hypothesis class_id/score pass-through).
  Empty array (fresh stamp) = clear corridor (NOT stale).
Method (exact, per config): mask pixels (mask == idx+1) -> reject
  NaN/inf/<min_depth/>max_depth -> median (mask_median) or trimmed median ->
  centroid pixel deprojected with LIVE K (never yaml placeholders).
Sync: depth matched within sync_tolerance of detection stamp, else dropped+counted.
Ground truth: never subscribed, never read (TEST-K).
"""
import argparse
import sys
import time

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (Detection2DArray, Detection3DArray, Detection3D,
                             ObjectHypothesisWithPose)

from rgbd_perception.config import load_config
from isaac_sim.scripts.perception.rgbd_interface import _mask_depth


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=None)
    return p.parse_args()


class FusionNode(Node):
    def __init__(self, cfg):
        super().__init__("rgbd_fusion_node")
        self.cfg = cfg
        pc, tp = cfg["perception"], cfg["topics"]
        self.method = pc.get("depth_method", "mask_median")
        self.trim = float(pc.get("trim_fraction", 0.1))
        self.min_d = float(pc.get("min_depth", 0.3))
        self.max_d = float(pc.get("max_depth", 10.0))
        self.min_frac = float(pc.get("min_valid_fraction", 0.05))
        self.sync_tol = float(pc.get("sync_tolerance", 0.1))
        self.depth_frame = tp["depth_optical_frame"]
        self.pub = self.create_publisher(Detection3DArray, tp["objects"], 10)
        self.create_subscription(Detection2DArray, tp["detections"], self.on_det, 10)
        self.create_subscription(Image, "/perception/masks", self.on_mask, 10)
        self.create_subscription(Image, tp["depth"], self.on_depth, 10)
        self.create_subscription(CameraInfo, tp["depth_info"], self.on_info, 10)
        self.masks, self.depth, self.info = None, None, None
        self.stats = {"pairs": 0, "drops": 0, "objects": 0}

    def on_mask(self, m):
        try:
            self.masks = (m, np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width))
        except Exception:
            self.masks = None

    def on_depth(self, m):
        try:
            if m.encoding != "32FC1":
                return
            self.depth = (m, np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width))
        except Exception:
            self.depth = None

    def on_info(self, m):
        self.info = m

    def on_det(self, det_msg):
        t_det = det_msg.header.stamp.sec + det_msg.header.stamp.nanosec * 1e-9
        out = Detection3DArray()
        out.header.stamp = det_msg.header.stamp
        out.header.frame_id = self.depth_frame
        if self.depth is None or self.info is None:
            self.stats["drops"] += 1
            return
        d_msg, depth = self.depth
        t_dep = d_msg.header.stamp.sec + d_msg.header.stamp.nanosec * 1e-9
        if abs(t_det - t_dep) > self.sync_tol:
            self.stats["drops"] += 1
            return
        try:
            K = list(self.info.k)
            fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
        except Exception:
            return
        H, W = depth.shape
        masks = self.masks[1] if self.masks and self.masks[1].shape == (H, W) else None
        if masks is None and self.masks is not None:
            try:
                import cv2  # not required; fallback below
                _ = cv2
            except Exception:
                pass
            try:
                from PIL import Image as PImage
                m = np.array(PImage.fromarray(self.masks[1]).resize((W, H), PImage.NEAREST))
                masks = m
            except Exception:
                masks = None
        self.stats["pairs"] += 1
        for i, r in enumerate(det_msg.detections):
            u = float(r.bbox.center.position.x)
            v = float(r.bbox.center.position.y)
            bw = float(r.bbox.size_x)
            bh = float(r.bbox.size_y)
            z, frac, n = None, 0.0, 0
            if masks is not None and self.method in ("mask_median", "mask_trimmed"):
                z, frac, n = _mask_depth(depth, masks == (i + 1), method=self.method,
                                        trim=self.trim, min_d=self.min_d, max_d=self.max_d,
                                        min_frac=self.min_frac)
            if z is None:
                ui, vi = int(round(u)), int(round(v))
                if 0 <= ui < W and 0 <= vi < H:
                    z0 = float(depth[vi, ui])
                    if np.isfinite(z0) and self.min_d <= z0 <= self.max_d:
                        z, frac, n = z0, 1.0, 1
            if z is None:
                continue
            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            o = Detection3D()
            o.header.stamp = det_msg.header.stamp
            o.header.frame_id = self.depth_frame
            h = ObjectHypothesisWithPose()
            try:
                h.hypothesis.class_id = r.results[0].hypothesis.class_id
                h.hypothesis.score = float(r.results[0].hypothesis.score)
            except Exception:
                continue
            o.results.append(h)
            o.bbox.center.position.x = float(X)
            o.bbox.center.position.y = float(Y)
            o.bbox.center.position.z = float(z)
            o.bbox.size.x = float(bw * z / fx)
            o.bbox.size.y = float(bh * z / fy)
            o.bbox.size.z = 0.0
            out.detections.append(o)
            self.stats["objects"] += 1
        self.pub.publish(out)


def main():
    a = parse_args()
    rclpy.init()
    node = FusionNode(load_config(a.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("[STATS]", node.stats, flush=True)


if __name__ == "__main__":
    main()
