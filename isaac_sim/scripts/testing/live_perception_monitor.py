"""Live Perception Monitor — SKATEBOARD_IRP.

Displays a live OpenCV window showing:
  - RGB camera image with YOLO bounding boxes, segmentation masks, class labels,
    confidence scores, estimated depth and 3D position for each detection.
  - HUD overlay: frame rates, YOLO latency, 3D detection count, timestamp.

Also writes annotated PNG frames to --out-dir at configurable intervals.
Runs as an external process (system Python 3.10 + perception_venv / rclpy),
separate from Isaac Sim.

Usage (standalone):
  cd /home/ubuntu/SKATEBOARD_IRP
  source /opt/ros/humble/setup.bash
  export PYTHONPATH=/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception:$PYTHONPATH
  /home/ubuntu/perception_venv/bin/python \\
      isaac_sim/scripts/testing/live_perception_monitor.py \\
      --out-dir data/live_monitor --window-title "SKATEBOARD Live Perception"
"""

import argparse
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection3DArray

try:
    from isaac_sim.scripts.scenarios.scenario_registry import resolve_scenario, get_scenario
except Exception:
    resolve_scenario = None
    get_scenario = None

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Colour palette for class IDs (hue-based, COCO)
CLASS_COLORS = {
    0:  (0,   220,  60),  # person: green
    13: (200,  80,  20),  # bench: orange
    56: (50,  100, 220),  # chair: blue
    57: (220,  50, 200),  # couch: magenta
    60: (220, 180,  20),  # dining table: yellow
}
DEFAULT_COLOR = (180, 180, 180)


def parse_class(class_id_str: str):
    """Parse class_id string like '0:person' → (0, 'person')."""
    try:
        parts = str(class_id_str).split(":", 1)
        cid = int(parts[0])
        lbl = parts[1] if len(parts) > 1 else str(cid)
        return cid, lbl
    except Exception:
        return -1, str(class_id_str)


class LiveMonitor(Node):
    def __init__(self, args):
        super().__init__("live_perception_monitor")
        self.args = args
        self.out_dir = Path(args.out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.title = args.window_title
        self.save_every = args.save_every  # save annotated frame every N seconds
        self._last_save = time.time()

        # Latest data
        self._rgb: np.ndarray | None = None
        self._rgb_stamp: float = 0.0
        self._dets = []   # list of Detection2D
        self._objs = []   # list of Detection3D
        self._det_stamp: float = 0.0
        self._obj_stamp: float = 0.0

        # Rate tracking
        self._rgb_times: deque = deque(maxlen=60)
        self._depth_times: deque = deque(maxlen=60)
        self._det_times: deque = deque(maxlen=60)
        self._obj_times: deque = deque(maxlen=60)
        self._det_latencies: deque = deque(maxlen=60)

        self._frame_count: int = 0
        self._save_idx: int = 0

        # Scenario identification & title
        self.scenario_id = getattr(args, "scenario", "s01")
        self.scenario_name = ""
        if resolve_scenario:
            try:
                entry = resolve_scenario(self.scenario_id)
                self.scenario_name = entry.name
            except Exception:
                self.scenario_name = ""

        # Subscriptions
        self.create_subscription(Image, "/camera/rgb/image_raw", self._on_rgb, BEST_EFFORT_QOS)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, BEST_EFFORT_QOS)
        self.create_subscription(Detection2DArray, "/perception/detections_2d",
                                 self._on_dets, RELIABLE_QOS)
        self.create_subscription(Image, "/perception/masks", self._on_mask, RELIABLE_QOS)
        self.create_subscription(Detection3DArray, "/perception/objects_3d",
                                 self._on_objs, RELIABLE_QOS)
        self.create_subscription(String, "/planning/scenario_id", self._on_scenario, RELIABLE_QOS)

        self._mask = None
        if args.show:
            cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.title, 960, 500)
            cv2.moveWindow(self.title, 960, 0)
        self.get_logger().info(f"LiveMonitor started. out_dir={self.out_dir}")

    # -------- callbacks --------

    def _on_mask(self, msg: Image):
        try:
            if msg.encoding == "mono8":
                self._mask = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        except Exception:
            pass

    def _on_rgb(self, msg: Image):
        now = time.time()
        self._rgb_times.append(now)
        self._last_rgb_wall_time = now
        if msg.encoding != "rgb8":
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._rgb = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        self._rgb_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._redraw()

    def _on_depth(self, msg: Image):
        self._depth_times.append(time.time())

    def _on_dets(self, msg: Detection2DArray):
        now = time.time()
        self._det_times.append(now)
        self._dets = list(msg.detections)
        self._det_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Latency between RGB arrival and detection arrival (wall time)
        if getattr(self, "_last_rgb_wall_time", 0.0) > 0.0:
            lat = (now - self._last_rgb_wall_time) * 1000.0
            if 0.0 <= lat < 2000.0:
                self._det_latencies.append(lat)

    def _on_objs(self, msg: Detection3DArray):
        now = time.time()
        self._obj_times.append(now)
        self._objs = list(msg.detections)
        self._obj_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_scenario(self, msg: String):
        self.scenario_id = msg.data.strip().lower()
        if resolve_scenario:
            try:
                entry = resolve_scenario(self.scenario_id)
                self.scenario_name = entry.name
            except Exception:
                self.scenario_name = self.scenario_id
        else:
            self.scenario_name = self.scenario_id
        self.get_logger().info(f"Monitor updated scenario: {self.scenario_id.upper()} ({self.scenario_name})")

    # -------- rendering --------

    def _rate(self, times: deque) -> float:
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        return (len(times) - 1) / dt if dt > 0 else 0.0

    def _redraw(self):
        if self._rgb is None:
            return
        canvas = self._rgb.copy()
        h, w = canvas.shape[:2]

        # --- Draw Segmentation Masks (semi-transparent overlay) ---
        if self._mask is not None and self._mask.shape == (h, w):
            overlay = canvas.copy()
            for i, det in enumerate(self._dets):
                try:
                    cid, _ = parse_class(det.results[0].hypothesis.class_id if det.results else "-1:?")
                    color = CLASS_COLORS.get(cid, DEFAULT_COLOR)
                    pixel_idx = i + 1
                    overlay[self._mask == pixel_idx] = color
                except Exception:
                    pass
            cv2.addWeighted(overlay, 0.40, canvas, 0.60, 0, canvas)

        # --- Draw YOLO 2D detections ---
        # Build robust depth lookup matching exact 2D detection index from 3D objects using obj.id
        depth_by_idx: dict = {}
        xyz_by_idx: dict = {}
        for obj in self._objs:
            try:
                obj_id = getattr(obj, "id", "")
                if obj_id.startswith("yolo_det_"):
                    det_idx = int(obj_id.split("_")[-1])
                    t = obj.bbox.center.position
                    if 0 <= det_idx < len(self._dets) and 0.30 <= t.z <= 10.00:
                        depth_by_idx[det_idx] = math.sqrt(t.x**2 + t.y**2 + t.z**2)
                        xyz_by_idx[det_idx] = (t.x, t.y, t.z)
            except Exception:
                pass

        for i, det in enumerate(self._dets):
            try:
                cid, lbl = parse_class(
                    det.results[0].hypothesis.class_id if det.results else "-1:?"
                )
                conf = det.results[0].hypothesis.score if det.results else 0.0
                bx = det.bbox.center.position.x
                by = det.bbox.center.position.y
                bw = det.bbox.size_x
                bh = det.bbox.size_y
                x1, y1 = int(bx - bw / 2), int(by - bh / 2)
                x2, y2 = int(bx + bw / 2), int(by + bh / 2)
                color = CLASS_COLORS.get(cid, DEFAULT_COLOR)

                # Bounding box
                cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

                # Label background
                dist_str = ""
                if i in depth_by_idx:
                    x3, y3, z3 = xyz_by_idx[i]
                    dist_str = f"  Z={z3:.2f}m d={depth_by_idx[i]:.2f}m"
                label_text = f"[{cid}]{lbl} {conf:.2f}{dist_str}"
                (tw, th), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                ty = max(y1 - 4, th + 4)
                cv2.rectangle(canvas, (x1, ty - th - 4), (x1 + tw + 4, ty), color, -1)
                cv2.putText(canvas, label_text, (x1 + 2, ty - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
            except Exception:
                pass

        # --- Draw Geometric/Unclassified 3D Depth Obstacles (S07 Solid Crate etc.) ---
        fx, fy, cx, cy = 640.0, 640.0, 640.0, 360.0
        for i, obj in enumerate(self._objs):
            try:
                obj_id = getattr(obj, "id", "")
                cid_str = obj.results[0].hypothesis.class_id if obj.results else ""
                # Check if this 3D obstacle is an unclassified depth object or non-YOLO
                if obj_id.startswith("geom_obs_") or "obstacle" in cid_str.lower():
                    tx = obj.bbox.center.position.x
                    ty = obj.bbox.center.position.y
                    tz = obj.bbox.center.position.z
                    if tz > 0.5:
                        u = int(fx * (tx / tz) + cx)
                        v = int(fy * (ty / tz) + cy)
                        # Estimated box dimensions in pixels
                        sx = max(25, min(300, int(fx * (max(0.4, obj.bbox.size.x) / tz))))
                        sy = max(25, min(300, int(fy * (max(0.4, obj.bbox.size.y) / tz))))
                        x1 = max(0, u - sx // 2)
                        y1 = max(35, v - sy // 2)
                        x2 = min(w - 1, u + sx // 2)
                        y2 = min(h - 80, v + sy // 2)
                        
                        geom_color = (0, 220, 255) # Yellow/Cyan
                        cv2.rectangle(canvas, (x1, y1), (x2, y2), geom_color, 2)
                        lbl_text = f"[DEPTH-3D] Solid Obstacle Z={tz:.2f}m d={math.hypot(tx,ty,tz):.2f}m"
                        (tw, th), _ = cv2.getTextSize(lbl_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                        ty_top = max(y1 - 4, th + 36)
                        cv2.rectangle(canvas, (x1, ty_top - th - 4), (x1 + tw + 4, ty_top), geom_color, -1)
                        cv2.putText(canvas, lbl_text, (x1 + 2, ty_top - 2),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
            except Exception:
                pass

        # --- Top HUD banner: Scenario Title & Camera Stream Health ---
        top_h = 32
        now_t = time.time()
        stream_healthy = (now_t - getattr(self, "_last_rgb_wall_time", 0.0) < 1.0) if getattr(self, "_last_rgb_wall_time", 0.0) > 0 else False
        health_color = (0, 220, 0) if stream_healthy else (0, 0, 220)
        health_str = "STREAM: HEALTHY" if stream_healthy else "STREAM: STALE / OFFLINE"

        cv2.rectangle(canvas, (0, 0), (w, top_h), (25, 25, 25), -1)
        scen_title_str = f"SCENARIO {self.scenario_id.upper()}: {self.scenario_name}" if self.scenario_name else f"SCENARIO: {self.scenario_id.upper()}"
        cv2.putText(canvas, scen_title_str, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, health_str, (w - 260, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, health_color, 2, cv2.LINE_AA)

        # --- HUD metrics bar at bottom ---
        rgb_fps = self._rate(self._rgb_times)
        depth_fps = self._rate(self._depth_times)
        det_fps = self._rate(self._det_times)
        obj_fps = self._rate(self._obj_times)
        lat_ms = float(np.mean(list(self._det_latencies))) if self._det_latencies else 0.0
        n_det = len(self._dets)
        n_obj = len(self._objs)

        hud_lines = [
            f"RGB {rgb_fps:.1f} Hz | Depth {depth_fps:.1f} Hz | Det {det_fps:.1f} Hz | Fusion {obj_fps:.1f} Hz",
            f"Detections: {n_det} | 3D Obs: {n_obj} | YOLO~lat {lat_ms:.0f}ms",
            f"Frame {self._frame_count} | {time.strftime('%H:%M:%S')}",
        ]
        bar_h = len(hud_lines) * 22 + 8
        cv2.rectangle(canvas, (0, h - bar_h), (w, h), (20, 20, 20), -1)
        for li, line in enumerate(hud_lines):
            cv2.putText(canvas, line, (8, h - bar_h + 18 + li * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 255, 200), 1, cv2.LINE_AA)

        self._frame_count += 1

        # --- Show and save ---
        if self.args.show:
            cv2.imshow(self.title, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                rclpy.shutdown()
                return

        now = time.time()
        if now - self._last_save >= self.save_every:
            fname = self.out_dir / f"monitor_frame_{self._save_idx:05d}.png"
            cv2.imwrite(str(fname), canvas)
            self._save_idx += 1
            self._last_save = now
            self.get_logger().info(
                f"Saved {fname.name} | RGB {rgb_fps:.1f}Hz Det {det_fps:.1f}Hz "
                f"Fusion {obj_fps:.1f}Hz n_det={n_det} n_3d={n_obj}")


def main():
    ap = argparse.ArgumentParser(description="SKATEBOARD Live Perception Monitor")
    ap.add_argument("--scenario", default="s02", help="Active scenario ID or name")
    ap.add_argument("--out-dir", default="/home/ubuntu/SKATEBOARD_IRP/data/live_monitor")
    ap.add_argument("--window-title", default="Window 2 - Live YOLO26n-seg & 3D Obstacle Monitor")
    ap.add_argument("--save-every", type=float, default=2.0,
                    help="Save annotated frame every N seconds (0 = never)")
    ap.add_argument("--no-show", dest="show", action="store_false", default=True,
                    help="Headless: write frames but do not open a window")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Auto-stop after N seconds (0 = run until Ctrl+C)")
    args = ap.parse_args()

    rclpy.init()
    node = LiveMonitor(args)
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.duration > 0 and (time.time() - t0) > args.duration:
                node.get_logger().info(f"Duration {args.duration}s reached — stopping.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
