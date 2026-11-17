"""Perception Metrics Collector for Live ROS 2 Pipeline.

Subscribes to:
  /camera/rgb/image_raw
  /camera/depth/image_raw
  /camera/depth/camera_info
  /perception/detections_2d
  /perception/masks
  /perception/objects_3d

Records:
  - Exact frame counts and publication rates (Hz)
  - 2D detections (bounding boxes, class labels, confidence scores)
  - 3D fused objects (optical X, Y, Z and Euclidean distance)
  - Time-series progression of pedestrian distance
  - Saves composite annotated PNG images at regular intervals
  - Writes comprehensive metrics JSON
"""
import argparse
import json
import math
import os
import sys
import time
import cv2
import numpy as np
from PIL import Image, ImageDraw

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray

CLASS_COLORS = {
    0: (0, 255, 0),     # person: green
    13: (255, 165, 0),  # bench: orange
    56: (0, 180, 255),  # chair: sky blue
    57: (255, 0, 255),  # couch: magenta
    60: (255, 255, 0),  # table: yellow
}

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
OUT_DIR = os.path.join(ROOT, "data/stationary_moving_test")

def parse_cid(s):
    try:
        return int(str(s).split(":")[0])
    except Exception:
        return -1

def parse_label(s):
    try:
        return str(s).split(":", 1)[1]
    except Exception:
        return str(s)

class PerceptionCollector(Node):
    def __init__(self, out_dir, duration):
        super().__init__("perception_metrics_collector")
        self.out_dir = out_dir
        self.duration = duration
        self.start_wall = time.time()
        
        # Latest caches
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_info = None
        self.latest_dets = None
        self.latest_masks = None
        self.latest_objs = None
        
        # Counts and timestamps for Hz calculation
        self.stats = {
            "rgb": {"count": 0, "first": None, "last": None},
            "depth": {"count": 0, "first": None, "last": None},
            "info": {"count": 0, "first": None, "last": None},
            "det": {"count": 0, "first": None, "last": None},
            "mask": {"count": 0, "first": None, "last": None},
            "obj": {"count": 0, "first": None, "last": None},
        }
        
        # Time-series observations of 3D objects
        self.observations = []
        self.snapshots_saved = 0
        self.last_snapshot_time = 0.0

        # Subscriptions
        self.create_subscription(RosImage, "/camera/rgb/image_raw", self.cb_rgb, 10)
        self.create_subscription(RosImage, "/camera/depth/image_raw", self.cb_depth, 10)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self.cb_info, 10)
        self.create_subscription(Detection2DArray, "/perception/detections_2d", self.cb_det, 10)
        self.create_subscription(RosImage, "/perception/masks", self.cb_mask, 10)
        self.create_subscription(Detection3DArray, "/perception/objects_3d", self.cb_obj, 10)
        
        self.get_logger().info(f"Collector initialized for {duration}s -> {out_dir}")

    def _mark(self, key):
        now = time.time()
        s = self.stats[key]
        s["count"] += 1
        if s["first"] is None:
            s["first"] = now
        s["last"] = now

    def cb_rgb(self, m):
        self._mark("rgb")
        if m.encoding == "rgb8":
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            self.latest_rgb = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, arr)

    def cb_depth(self, m):
        self._mark("depth")
        if m.encoding == "32FC1":
            arr = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
            self.latest_depth = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, arr)

    def cb_info(self, m):
        self._mark("info")
        self.latest_info = m

    def cb_det(self, m):
        self._mark("det")
        self.latest_dets = m

    def cb_mask(self, m):
        self._mark("mask")
        arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width)
        self.latest_masks = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, arr)

    def cb_obj(self, m):
        self._mark("obj")
        self.latest_objs = m
        now = time.time()
        t_sim = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

        # Process detected 3D objects
        for o in m.detections:
            cid = parse_cid(o.results[0].hypothesis.class_id) if o.results else -1
            label = parse_label(o.results[0].hypothesis.class_id) if o.results else "unknown"
            score = float(o.results[0].hypothesis.score) if o.results else 0.0
            
            x = float(o.bbox.center.position.x)
            y = float(o.bbox.center.position.y)
            z = float(o.bbox.center.position.z)
            dist = math.sqrt(x*x + y*y + z*z)
            
            entry = {
                "wall_time": round(now - self.start_wall, 3),
                "sim_time": round(t_sim, 3),
                "class_id": cid,
                "label": label,
                "confidence": round(score, 3),
                "optical_x": round(x, 3),
                "optical_y": round(y, 3),
                "optical_z": round(z, 3),
                "distance": round(dist, 3)
            }
            self.observations.append(entry)

        # Periodically save annotated snapshots (every 2.5s)
        if now - self.last_snapshot_time >= 2.5 and self.latest_rgb is not None:
            self.save_annotated_frame(t_sim)
            self.last_snapshot_time = now

    def save_annotated_frame(self, t_sim):
        if self.latest_rgb is None:
            return
        rgb_t, rgb_np = self.latest_rgb
        vis = Image.fromarray(rgb_np).convert("RGBA")
        ov = Image.new("RGBA", vis.size, (0, 0, 0, 0))
        dr_ov = ImageDraw.Draw(ov)

        masks_np = self.latest_masks[1] if self.latest_masks else None
        dets = list(self.latest_dets.detections) if self.latest_dets else []
        objs = list(self.latest_objs.detections) if self.latest_objs else []

        # Color segmentation masks
        for i, r in enumerate(dets):
            cid = parse_cid(r.results[0].hypothesis.class_id) if r.results else -1
            color = CLASS_COLORS.get(cid, (255, 40, 40))
            if masks_np is not None:
                try:
                    m = Image.fromarray(((masks_np == (i + 1)) * 110).astype(np.uint8), "L")
                    ov.paste(color + (110,), (0, 0), m)
                except Exception:
                    pass

        vis = Image.alpha_composite(vis, ov)
        dr = ImageDraw.Draw(vis)

        # Draw bounding boxes and text
        for i, r in enumerate(dets):
            cid = parse_cid(r.results[0].hypothesis.class_id) if r.results else -1
            label = parse_label(r.results[0].hypothesis.class_id) if r.results else "obj"
            conf = float(r.results[0].hypothesis.score) if r.results else 0.0
            color = CLASS_COLORS.get(cid, (255, 40, 40))

            u = float(r.bbox.center.position.x)
            v = float(r.bbox.center.position.y)
            bw, bh = float(r.bbox.size_x), float(r.bbox.size_y)
            x1, y1 = int(u - bw / 2), int(v - bh / 2)
            x2, y2 = int(u + bw / 2), int(v + bh / 2)

            dist_str = ""
            if i < len(objs):
                ox = float(objs[i].bbox.center.position.x)
                oy = float(objs[i].bbox.center.position.y)
                oz = float(objs[i].bbox.center.position.z)
                d = math.sqrt(ox*ox + oy*oy + oz*oz)
                dist_str = f" {d:.2f}m (Z={oz:.2f}m)"

            dr.rectangle([x1, y1, x2, y2], outline=color + (255,), width=2)
            tag = f"{label} {conf:.2f}{dist_str}"
            dr.text((x1 + 2, max(0, y1 - 12)), tag, fill=(255, 255, 255, 255))

        # Composite raw and annotated
        raw_im = Image.fromarray(rgb_np)
        W, H = raw_im.size
        canvas = Image.new("RGB", (W * 2 + 10, H + 30), (30, 30, 30))
        canvas.paste(raw_im, (0, 30))
        canvas.paste(vis.convert("RGB"), (W + 10, 30))
        cd = ImageDraw.Draw(canvas)
        cd.text((10, 8), f"SIM RAW RGB | t_sim={t_sim:.1f}s", fill=(220, 220, 220))
        cd.text((W + 20, 8), f"YOLO26n-seg + MASK-MEDIAN DEPTH FUSION", fill=(220, 220, 220))

        fname = f"snapshot_{self.snapshots_saved:02d}_t{int(t_sim):02d}s.png"
        fpath = os.path.join(self.out_dir, fname)
        canvas.save(fpath)
        self.snapshots_saved += 1
        self.get_logger().info(f"Saved visual snapshot -> {fname}")

    def compute_summary(self):
        summary = {"rates_hz": {}, "counts": {}}
        for k, v in self.stats.items():
            cnt = v["count"]
            summary["counts"][k] = cnt
            if cnt > 1 and v["first"] is not None and v["last"] is not None:
                dt = v["last"] - v["first"]
                summary["rates_hz"][k] = round(cnt / dt, 2) if dt > 0 else 0.0
            else:
                summary["rates_hz"][k] = 0.0

        summary["total_3d_observations"] = len(self.observations)
        
        # Pedestrian distance progression analysis
        peds = [o for o in self.observations if o["class_id"] == 0 or o["label"] == "person"]
        if peds:
            depths = [p["optical_z"] for p in peds]
            dists = [p["distance"] for p in peds]
            times = [p["sim_time"] for p in peds]
            summary["pedestrian_analysis"] = {
                "count": len(peds),
                "first_detected_sim_time": times[0],
                "last_detected_sim_time": times[-1],
                "initial_depth_m": depths[0],
                "final_depth_m": depths[-1],
                "min_depth_m": min(depths),
                "max_depth_m": max(depths),
                "initial_distance_m": dists[0],
                "final_distance_m": dists[-1],
                "min_distance_m": min(dists),
                "max_distance_m": max(dists),
            }
        else:
            summary["pedestrian_analysis"] = None

        return summary

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--out", default=OUT_DIR)
    args = p.parse_args()

    rclpy.init()
    node = PerceptionCollector(args.out, args.duration)

    t0 = time.time()
    try:
        while time.time() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    summary = node.compute_summary()
    res_path = os.path.join(args.out, "metrics.json")
    with open(res_path, "w") as f:
        json.dump({"summary": summary, "observations": node.observations}, f, indent=2)

    print("\n" + "="*60)
    print(" PERCEPTION METRICS SUMMARY")
    print("="*60)
    print(f"Counts: {summary['counts']}")
    print(f"Rates (Hz): {summary['rates_hz']}")
    if summary["pedestrian_analysis"]:
        pa = summary["pedestrian_analysis"]
        print(f"Pedestrian 3D Observations: {pa['count']}")
        print(f"  First Sim Time: {pa['first_detected_sim_time']}s | Last Sim Time: {pa['last_detected_sim_time']}s")
        print(f"  Depth (Z): initial={pa['initial_depth_m']}m -> min={pa['min_depth_m']}m -> final={pa['final_depth_m']}m")
        print(f"  Distance: initial={pa['initial_distance_m']}m -> min={pa['min_distance_m']}m -> final={pa['final_distance_m']}m")
    else:
        print("[WARN] No pedestrian 3D observations recorded!")
    print(f"Full metrics saved to: {res_path}")
    print("="*60 + "\n")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
