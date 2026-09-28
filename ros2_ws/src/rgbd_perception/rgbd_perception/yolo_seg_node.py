"""YOLO26n-seg ROS node — RGB-ONLY inference (Phase C).

Subscribes: sensor_msgs/Image /camera/rgb/image_raw (rgb8).
Publishes:
  vision_msgs/Detection2DArray /perception/detections_2d (header = RGB stamp/frame;
    results[0].hypothesis.class_id = "<coco_id>:<label>", score = confidence;
    bbox center/size in pixels). Empty array (fresh stamp) = corridor clear.
  sensor_msgs/Image mono8 /perception/masks (pixel = detection_index+1,
    0 = background; same stamp/frame; HxW of RGB).
Never subscribes depth/GT/odom. Rate-limited (drops counted, never blocks).
Run: /home/ubuntu/perception_venv/bin/python yolo_seg_node.py [--config ...]
  (venv inherits rclpy from system site-packages; source /opt/ros/humble first)
"""
import argparse
import sys
import time

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from rgbd_perception.config import load_config
from rgbd_perception.yolo_seg import YoloSegDetector


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=None)
    return p.parse_args()


class YoloSegNode(Node):
    def __init__(self, cfg):
        super().__init__("yolo_seg_node")
        self.cfg = cfg
        pc, tp = cfg["perception"], cfg["topics"]
        self.det = YoloSegDetector(
            model_path=pc["model_path"], device=pc["device"], conf=pc["confidence_threshold"],
            iou=pc["iou_threshold"], imgsz=pc["image_size"], half=pc["half_precision"],
            target_classes=tuple(pc["target_classes"]),
            class_names_override=pc.get("class_names_override") or {})
        self.get_logger().info("YOLO task=segment device=%s classes=%s" % (
            self.det.device, sorted(self.det.target_classes)))
        self.pub_det = self.create_publisher(Detection2DArray, tp["detections"], 10)
        self.pub_mask = self.create_publisher(Image, "/perception/masks", 10)
        self.create_subscription(Image, tp["rgb"], self.on_rgb, 10)
        self.min_period = 1.0 / max(float(pc.get("perception_rate", 10.0)), 0.1)
        self.busy = False
        self.stats = {"frames": 0, "drops": 0, "n_det": 0}

    def on_rgb(self, msg):
        if msg.encoding != "rgb8":
            return
        now = time.time()
        if getattr(self, "_last", 0) and now - self._last < self.min_period:
            return
        if self.busy:
            self.stats["drops"] += 1
            return
        self.busy = True
        try:
            self._last = now
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dets, lat = self.det.predict(rgb, stamp=stamp_s, frame_id=msg.header.frame_id)
            self.stats["frames"] += 1
            self.stats["n_det"] += len(dets)
            out = Detection2DArray()
            out.header = msg.header
            mk = np.zeros((msg.height, msg.width), dtype=np.uint8)
            for i, d in enumerate(dets):
                r = Detection2D()
                r.header = msg.header
                h = ObjectHypothesisWithPose()
                h.hypothesis.class_id = "%d:%s" % (d.class_id, d.class_name)
                h.hypothesis.score = float(d.confidence)
                r.results.append(h)
                x1, y1, x2, y2 = d.bbox
                r.bbox.center.position.x = float((x1 + x2) / 2.0)
                r.bbox.center.position.y = float((y1 + y2) / 2.0)
                r.bbox.size_x = float(x2 - x1)
                r.bbox.size_y = float(y2 - y1)
                out.detections.append(r)
                if d.mask is not None:
                    try:
                        mk[np.asarray(d.mask, dtype=bool)] = i + 1
                    except Exception:
                        pass
            self.pub_det.publish(out)
            mi = Image()
            mi.header = msg.header
            mi.height, mi.width, mi.encoding = msg.height, msg.width, "mono8"
            mi.step = msg.width
            mi.data = mk.tobytes()
            self.pub_mask.publish(mi)
            self.get_logger().info(
                "t=%.1f n=%d infer=%.0fms" % (stamp_s, len(dets), lat.get("yolo_inf_ms", -1)),
                throttle_duration_sec=2.0)
        finally:
            self.busy = False


def main():
    a = parse_args()
    rclpy.init()
    node = YoloSegNode(load_config(a.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("[STATS]", node.stats, flush=True)


if __name__ == "__main__":
    main()
