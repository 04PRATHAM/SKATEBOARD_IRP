#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Multi-Object Perception System
Module: obstacle_positioning.py

Phase 3: Image-Based Obstacle Positioning & Driving Corridor Analysis
Model: YOLO26n-seg + ByteTrack

Calculates:
- 2D Bounding box geometry (x1, y1, x2, y2, width, height, center)
- Segmentation mask area (exact pixel count)
- Normalized horizontal (x_norm) and vertical (y_norm) image coordinates
- Image-based zone classification (LEFT, CENTER, RIGHT)
- Overlap percentage with the virtual vehicle driving corridor
- Clean interfaces for future RGB-D metric depth integration

CRITICAL PRINCIPLE:
This module operates strictly in 2D image-space coordinates. It does NOT
invent or hallucinate 3D metric distances in meters. True metric depth will
be populated once an RGB-D sensor is physically integrated.

Author: Robotics Software Engineering Team
Permitted Workspace: /home/vat/yolo_vehicle_test/
"""

import os
import sys
import time
import argparse
import signal
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional
import numpy as np
import cv2
import yaml

os.environ["YOLO_CONFIG_DIR"] = "/home/vat/yolo_vehicle_test/.config/Ultralytics"

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ERROR: Ultralytics library not found.")
    sys.exit(1)


@dataclass
class PositionedObstacle:
    """Represents a spatially localized obstacle in image space."""
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    width: int
    height: int
    center_pixel: Tuple[int, int]    # (cx, cy)
    center_normalized: Tuple[float, float]  # (cx / frame_w, cy / frame_h)
    mask_pixel_area: int
    horizontal_zone: str             # "LEFT", "CENTER", "RIGHT"
    corridor_overlap_ratio: float    # 0.0 to 1.0 (overlap with driving corridor)
    in_corridor: bool                # True if corridor_overlap_ratio > threshold
    # Placeholder for future RGB-D metric depth (None until sensor is active)
    metric_depth_meters: Optional[float] = None


class ObstaclePositioner:
    """
    Analyzes detected object masks and bounding boxes to establish their
    relative spatial position across the camera field of view and evaluates
    interference with the vehicle's projected forward path.
    """

    def __init__(
        self,
        model_path: str = "/home/vat/yolo_vehicle_test/yolo26n-seg.pt",
        camera_id: int = 0,
        conf_threshold: float = 0.25,
        target_class_ids: Optional[List[int]] = None,
        classes_config_path: str = "/home/vat/yolo_vehicle_test/config/object_classes.yaml",
        corridor_bounds: Tuple[float, float] = (0.25, 0.75),  # [25%, 75%] of image width
        corridor_overlap_thresh: float = 0.10,               # 10% overlap triggers corridor warning
        headless: bool = False,
        save_video_path: Optional[str] = None,
        save_snapshots: bool = False,
        snapshot_interval: int = 50,
        max_frames: Optional[int] = None,
    ):
        self.model_path = model_path
        self.camera_id = camera_id
        self.conf_threshold = conf_threshold
        self.corridor_left = corridor_bounds[0]
        self.corridor_right = corridor_bounds[1]
        self.corridor_overlap_thresh = corridor_overlap_thresh
        self.headless = headless
        self.save_video_path = save_video_path
        self.save_snapshots = save_snapshots
        self.snapshot_interval = snapshot_interval
        self.max_frames = max_frames
        self.running = True

        # Telemetry
        self.frame_count = 0
        self.latencies: List[float] = []
        self.fps_rolling: float = 0.0

        # Class metadata
        self.class_metadata = self._load_class_metadata(classes_config_path)
        self.target_class_ids = set(target_class_ids) if target_class_ids else {0, 13, 56, 60}

        # Initialize
        self.model = self._initialize_model()
        self.cap = None
        self.video_writer = None
        self._initialize_camera()

        signal.signal(signal.SIGINT, self._sigint_handler)
        signal.signal(signal.SIGTERM, self._sigint_handler)

    def _sigint_handler(self, sig, frame):
        print("\n🛑 Stopping obstacle positioner...")
        self.running = False

    def _load_class_metadata(self, config_path: str) -> Dict:
        default_metadata = {
            0: {"name": "person", "color": (0, 255, 0)},
            13: {"name": "bench", "color": (0, 165, 255)},
            56: {"name": "chair", "color": (255, 144, 30)},
            60: {"name": "dining table", "color": (0, 215, 255)}
        }
        if os.path.exists(config_path):
            try:
                with open(config_path, "r") as f:
                    data = yaml.safe_load(f)
                    metadata = {}
                    if "target_classes" in data:
                        for k, v in data["target_classes"].items():
                            cid = v.get("id")
                            if cid is not None:
                                metadata[cid] = {
                                    "name": v.get("name"),
                                    "color": tuple(v.get("color", [0, 255, 0]))
                                }
                        return metadata
            except Exception:
                pass
        return default_metadata

    def _initialize_model(self) -> YOLO:
        print("=" * 70)
        print("🔍 INITIALIZING OBSTACLE POSITIONING ENGINE (YOLO26n-seg)")
        print("=" * 70)
        t0 = time.perf_counter()
        model = YOLO(self.model_path)
        print(f"✅ Model loaded in {(time.perf_counter() - t0)*1000.0:.1f} ms.")
        return model

    def _initialize_camera(self):
        print(f"📷 Opening Camera Device: /dev/video{self.camera_id}")
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            print(f"❌ FATAL: Cannot open camera /dev/video{self.camera_id}")
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"✅ Camera resolution: {actual_w}x{actual_h}")

        if self.save_video_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.save_video_path)), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(
                self.save_video_path, fourcc, 20.0, (actual_w, actual_h)
            )
            print(f"💾 Saving positioning video to: {self.save_video_path}")

    def analyze_frame(self, frame: np.ndarray, result) -> List[PositionedObstacle]:
        """
        Calculates spatial coordinates, mask areas, zone classification,
        and corridor overlap for every detected object instance.
        """
        h, w = frame.shape[:2]
        c_x1 = int(self.corridor_left * w)
        c_x2 = int(self.corridor_right * w)

        obstacles = []
        if result.boxes is None or len(result.boxes) == 0:
            return obstacles

        boxes = result.boxes
        masks = result.masks
        track_ids = boxes.id.int().cpu().tolist() if boxes.id is not None else [-1] * len(boxes)

        for i, box in enumerate(boxes):
            cls_id = int(box.cls[0])
            if cls_id not in self.target_class_ids:
                continue

            conf = float(box.conf[0])
            class_name = self.model.names.get(cls_id, f"ID_{cls_id}")
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            bw = x2 - x1
            bh = y2 - y1
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cx_norm = cx / float(w)
            cy_norm = cy / float(h)

            # Mask analysis
            mask_area = 0
            corridor_overlap_ratio = 0.0

            if masks is not None and len(masks) > i:
                raw_mask = masks.data[i].cpu().numpy()
                if raw_mask.shape[:2] != (h, w):
                    raw_mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST)
                binary_mask = (raw_mask > 0.5).astype(np.uint8)
                mask_area = int(np.sum(binary_mask))

                # Calculate overlap between binary mask and driving corridor column
                corridor_mask_slice = binary_mask[:, c_x1:c_x2]
                overlap_pixels = int(np.sum(corridor_mask_slice))
                if mask_area > 0:
                    corridor_overlap_ratio = overlap_pixels / float(mask_area)
            else:
                # Fallback to bounding box overlap if segmentation mask is unavailable
                mask_area = bw * bh
                overlap_x1 = max(x1, c_x1)
                overlap_x2 = min(x2, c_x2)
                if overlap_x2 > overlap_x1:
                    corridor_overlap_ratio = (overlap_x2 - overlap_x1) / float(bw)

            # Zone classification based on normalized horizontal center
            if cx_norm < self.corridor_left:
                horizontal_zone = "LEFT"
            elif cx_norm > self.corridor_right:
                horizontal_zone = "RIGHT"
            else:
                horizontal_zone = "CENTER"

            in_corridor = (corridor_overlap_ratio >= self.corridor_overlap_thresh) or (horizontal_zone == "CENTER")

            obs = PositionedObstacle(
                track_id=track_ids[i],
                class_id=cls_id,
                class_name=class_name,
                confidence=conf,
                bbox=(x1, y1, x2, y2),
                width=bw,
                height=bh,
                center_pixel=(cx, cy),
                center_normalized=(cx_norm, cy_norm),
                mask_pixel_area=mask_area,
                horizontal_zone=horizontal_zone,
                corridor_overlap_ratio=corridor_overlap_ratio,
                in_corridor=in_corridor,
                metric_depth_meters=None  # Reserved for RGB-D camera
            )
            obstacles.append(obs)

        return obstacles

    def render_positioning_overlay(
        self, frame: np.ndarray, obstacles: List[PositionedObstacle], latency_ms: float
    ) -> np.ndarray:
        """Renders the virtual driving corridor and obstacle spatial telemetry."""
        canvas = frame.copy()
        h, w = canvas.shape[:2]
        c_x1 = int(self.corridor_left * w)
        c_x2 = int(self.corridor_right * w)

        # 1. Draw Virtual Driving Corridor (Subtle Translucent Guide)
        corridor_overlay = canvas.copy()
        cv2.rectangle(corridor_overlay, (c_x1, 0), (c_x2, h), (180, 50, 180), -1)
        cv2.addWeighted(corridor_overlay, 0.12, canvas, 0.88, 0, canvas)

        # Boundary guideline ticks
        cv2.line(canvas, (c_x1, 0), (c_x1, h), (255, 0, 255), 2, cv2.LINE_AA)
        cv2.line(canvas, (c_x2, 0), (c_x2, h), (255, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "CORRIDOR LEFT", (c_x1 + 6, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 0, 255), 1)
        cv2.putText(canvas, "CORRIDOR RIGHT", (c_x2 - 120, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 0, 255), 1)

        # 2. Render Positioned Obstacles
        for obs in obstacles:
            x1, y1, x2, y2 = obs.bbox
            color = self.class_metadata.get(obs.class_id, {}).get("color", (0, 255, 0))

            # If obstacle intrudes into the driving corridor, highlight box in Bright Red
            box_color = (0, 0, 255) if obs.in_corridor else color
            thickness = 3 if obs.in_corridor else 2
            cv2.rectangle(canvas, (x1, y1), (x2, y2), box_color, thickness)

            # Center crosshair and centroid marker
            cx, cy = obs.center_pixel
            cv2.drawMarker(canvas, (cx, cy), box_color, markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)

            # Telemetry badge
            tag_id_str = f"ID:{obs.track_id} " if obs.track_id != -1 else ""
            status_flag = "⚠️ PATH INTRUSION" if obs.in_corridor else f"Zone: {obs.horizontal_zone}"
            line1 = f"{tag_id_str}{obs.class_name.upper()} ({obs.confidence:.2f})"
            line2 = f"{status_flag} | Area: {obs.mask_pixel_area}px"

            (w1, h1), _ = cv2.getTextSize(line1, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            (w2, h2), _ = cv2.getTextSize(line2, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
            tag_w = max(w1, w2) + 10
            tag_h = h1 + h2 + 12

            badge_y1 = max(0, y1 - tag_h - 4)
            badge_y2 = badge_y1 + tag_h
            cv2.rectangle(canvas, (x1, badge_y1), (x1 + tag_w, badge_y2), (20, 20, 20), -1)
            cv2.rectangle(canvas, (x1, badge_y1), (x1 + tag_w, badge_y2), box_color, 1)

            cv2.putText(canvas, line1, (x1 + 5, badge_y1 + h1 + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)
            cv2.putText(canvas, line2, (x1 + 5, badge_y1 + h1 + h2 + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)

        # 3. Telemetry Header Banner
        banner_h = 42
        banner = canvas.copy()
        cv2.rectangle(banner, (0, 0), (w, banner_h), (20, 20, 20), -1)
        cv2.addWeighted(banner, 0.80, canvas, 0.20, 0, canvas)

        intrusions = sum(1 for o in obstacles if o.in_corridor)
        status_color = (0, 0, 255) if intrusions > 0 else (0, 255, 0)
        status_msg = f"PATH BLOCKED ({intrusions} INTRUDERS)" if intrusions > 0 else "CORRIDOR CLEAR"

        header_str = (
            f"OBSTACLE POSITIONING  |  {status_msg}  |  "
            f"FPS: {self.fps_rolling:.1f}  |  "
            f"Obs: {len(obstacles)}"
        )
        cv2.putText(canvas, header_str, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, status_color, 1, cv2.LINE_AA)

        return canvas

    def run(self):
        print("\n" + "=" * 70)
        print("🚀 RUNNING OBSTACLE POSITIONING & CORRIDOR ANALYSIS")
        print(f"   Corridor Horizontal Bounds: [{self.corridor_left*100:.0f}%, {self.corridor_right*100:.0f}%]")
        print("=" * 70)

        start_wall_time = time.time()
        consecutive_failures = 0

        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    consecutive_failures += 1
                    if consecutive_failures >= 15:
                        print("❌ Camera feed lost.")
                        break
                    time.sleep(0.05)
                    continue

                consecutive_failures = 0
                self.frame_count += 1

                t_start = time.perf_counter()
                results = self.model.track(
                    source=frame,
                    persist=True,
                    tracker="bytetrack.yaml",
                    conf=self.conf_threshold,
                    device="cpu",
                    verbose=False,
                    classes=list(self.target_class_ids)
                )
                t_end = time.perf_counter()
                latency_ms = (t_end - t_start) * 1000.0
                self.latencies.append(latency_ms)

                instant_fps = 1000.0 / latency_ms if latency_ms > 0 else 0.0
                self.fps_rolling = 0.9 * self.fps_rolling + 0.1 * instant_fps if self.frame_count > 1 else instant_fps

                obstacles = self.analyze_frame(frame, results[0])
                annotated_frame = self.render_positioning_overlay(frame, obstacles, latency_ms)

                if self.frame_count % 10 == 0 or any(o.in_corridor for o in obstacles):
                    obs_report = []
                    for o in obstacles:
                        flag = "INTRUDER" if o.in_corridor else o.horizontal_zone
                        obs_report.append(f"{o.class_name}(x={o.center_normalized[0]:.2f}, {flag})")
                    rep_str = ", ".join(obs_report) if obs_report else "No objects"
                    print(
                        f"Frame {self.frame_count:04d} | "
                        f"Latency: {latency_ms:5.1f} ms | "
                        f"FPS: {self.fps_rolling:4.1f} | "
                        f"{rep_str}"
                    )

                if self.save_snapshots and (self.frame_count % self.snapshot_interval == 0):
                    snap_path = f"/home/vat/yolo_vehicle_test/results/detection/position_snap_{self.frame_count:04d}.jpg"
                    cv2.imwrite(snap_path, annotated_frame)

                if self.video_writer is not None:
                    self.video_writer.write(annotated_frame)

                if not self.headless:
                    cv2.imshow("VIRYA Buggy - Obstacle Positioning", annotated_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        break

                if self.max_frames and self.frame_count >= self.max_frames:
                    print(f"\nReached max frames limit ({self.max_frames}).")
                    break

        finally:
            self._cleanup()
            print(f"Positioning session completed. Processed {self.frame_count} frames in {time.time()-start_wall_time:.2f} s.")

    def _cleanup(self):
        if self.cap is not None:
            self.cap.release()
        if self.video_writer is not None:
            self.video_writer.release()
        if not self.headless:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="VIRYA DBW Buggy - Obstacle Positioning & Corridor Analysis")
    parser.add_argument("--model", type=str, default="/home/vat/yolo_vehicle_test/yolo26n-seg.pt")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-video", type=str, default=None)
    parser.add_argument("--save-snapshots", action="store_true")
    parser.add_argument("--snapshot-interval", type=int, default=30)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--corridor-left", type=float, default=0.25)
    parser.add_argument("--corridor-right", type=float, default=0.75)

    args = parser.parse_args()

    if not os.environ.get("DISPLAY"):
        if not args.headless:
            args.headless = True

    positioner = ObstaclePositioner(
        model_path=args.model,
        camera_id=args.camera,
        conf_threshold=args.conf,
        corridor_bounds=(args.corridor_left, args.corridor_right),
        headless=args.headless,
        save_video_path=args.save_video,
        save_snapshots=args.save_snapshots,
        snapshot_interval=args.snapshot_interval,
        max_frames=args.max_frames,
    )
    positioner.run()


if __name__ == "__main__":
    main()
