#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Multi-Object Perception System
Module: object_tracking.py

Phase 2: Multi-Object Tracking (ByteTrack)
Model: YOLO26n-seg (CPU Inference)
Target Categories: Person, Bench, Chair, Dining Table

Assigns persistent tracking IDs, draws motion history trajectories,
and maintains identity across consecutive frames.

Author: Robotics Software Engineering Team
Permitted Workspace: /home/vat/yolo_vehicle_test/
"""

import os
import sys
import time
import argparse
import signal
from collections import defaultdict, deque
from typing import Dict, List, Tuple, Optional
import numpy as np
import cv2
import yaml

os.environ["YOLO_CONFIG_DIR"] = "/home/vat/yolo_vehicle_test/.config/Ultralytics"

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ERROR: Ultralytics library not found in current environment.")
    sys.exit(1)


class ObjectTracker:
    """
    Multi-Object Tracker utilizing YOLO26n-seg and ByteTrack.
    Maintains persistent tracking identities, history trajectories,
    and instance segmentation across video frames.
    """

    def __init__(
        self,
        model_path: str = "/home/vat/yolo_vehicle_test/yolo26n-seg.pt",
        camera_id: int = 0,
        conf_threshold: float = 0.25,
        target_class_ids: Optional[List[int]] = None,
        classes_config_path: str = "/home/vat/yolo_vehicle_test/config/object_classes.yaml",
        headless: bool = False,
        save_video_path: Optional[str] = None,
        save_snapshots: bool = False,
        snapshot_interval: int = 50,
        max_frames: Optional[int] = None,
        track_history_len: int = 30,
    ):
        self.model_path = model_path
        self.camera_id = camera_id
        self.conf_threshold = conf_threshold
        self.headless = headless
        self.save_video_path = save_video_path
        self.save_snapshots = save_snapshots
        self.snapshot_interval = snapshot_interval
        self.max_frames = max_frames
        self.track_history_len = track_history_len
        self.running = True

        # Track history: track_id -> deque of (cx, cy)
        self.track_history = defaultdict(lambda: deque(maxlen=self.track_history_len))
        self.track_class_map = {}
        self.active_tracks_count = 0
        self.total_unique_tracks = set()

        # Telemetry
        self.frame_count = 0
        self.latencies: List[float] = []
        self.fps_rolling: float = 0.0

        # Load class metadata
        self.class_metadata = self._load_class_metadata(classes_config_path)
        self.target_class_ids = set(target_class_ids) if target_class_ids else {0, 13, 56, 60}

        # Initialize Model & Camera
        self.model = self._initialize_model()
        self.cap = None
        self.video_writer = None
        self._initialize_camera()

        signal.signal(signal.SIGINT, self._sigint_handler)
        signal.signal(signal.SIGTERM, self._sigint_handler)

    def _sigint_handler(self, sig, frame):
        print("\n🛑 Termination signal received. Stopping tracker gracefully...")
        self.running = False

    def _load_class_metadata(self, config_path: str) -> Dict:
        default_metadata = {
            0: {"name": "person", "color": (0, 255, 0)},       # Green
            13: {"name": "bench", "color": (0, 165, 255)},     # Orange
            56: {"name": "chair", "color": (255, 144, 30)},    # Blue
            60: {"name": "dining table", "color": (0, 215, 255)} # Yellow
        }
        if os.path.exists(config_path):
            try:
                with open(config_path, "r") as f:
                    data = yaml.safe_load(f)
                    metadata = {}
                    if "target_classes" in data:
                        for key, info in data["target_classes"].items():
                            cid = info.get("id")
                            name = info.get("name")
                            color = tuple(info.get("color", [0, 255, 0]))
                            if cid is not None:
                                metadata[cid] = {"name": name, "color": color}
                        return metadata
            except Exception as e:
                print(f"⚠️ Warning: Could not parse {config_path}: {e}")
        return default_metadata

    def _initialize_model(self) -> YOLO:
        print("=" * 70)
        print("🔍 INITIALIZING YOLO26n-seg + BYTETRACK ENGINE")
        print("=" * 70)
        if not os.path.exists(self.model_path):
            print(f"❌ FATAL ERROR: Model not found at: {self.model_path}")
            sys.exit(1)

        t0 = time.perf_counter()
        model = YOLO(self.model_path)
        print(f"✅ Model loaded in {(time.perf_counter() - t0)*1000.0:.1f} ms.")

        print("📋 Active Tracked Classes:")
        for cid in sorted(self.target_class_ids):
            print(f"   - Class ID {cid:02d}: '{model.names.get(cid, 'UNKNOWN')}'")

        return model

    def _initialize_camera(self):
        print(f"\n📷 Opening Camera Device: /dev/video{self.camera_id}")
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
        print(f"✅ Camera online: {actual_w}x{actual_h}")

        if self.save_video_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.save_video_path)), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(
                self.save_video_path, fourcc, 20.0, (actual_w, actual_h)
            )
            print(f"💾 Saving tracking video to: {self.save_video_path}")

    def run(self):
        print("\n" + "=" * 70)
        print("🚀 STARTING MULTI-OBJECT TRACKING STREAM")
        print("=" * 70)
        if not self.headless:
            print("Keyboard Controls: [Q] Quit  |  [S] Save Snapshot Frame")
            cv2.namedWindow("VIRYA Buggy - Multi-Object Tracking", cv2.WINDOW_NORMAL)

        start_wall_time = time.time()
        consecutive_failures = 0

        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    consecutive_failures += 1
                    if consecutive_failures >= 15:
                        print("❌ FATAL: Camera feed lost.")
                        break
                    time.sleep(0.05)
                    continue

                consecutive_failures = 0
                self.frame_count += 1

                t_start = time.perf_counter()

                # Multi-Object Tracking using ByteTrack with persistence
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

                annotated_frame, tracked_objects = self._render_tracks(frame, results[0], latency_ms)

                # Periodic terminal telemetry
                if self.frame_count % 10 == 0 or len(tracked_objects) > 0:
                    summary_str = ", ".join([f"ID {obj['id']}:{obj['class']}({obj['conf']:.2f})" for obj in tracked_objects])
                    if not summary_str:
                        summary_str = "No active tracks"
                    print(
                        f"Frame {self.frame_count:04d} | "
                        f"Latency: {latency_ms:5.1f} ms | "
                        f"FPS: {self.fps_rolling:4.1f} | "
                        f"Active Tracks ({len(tracked_objects)}): {summary_str}"
                    )

                if self.save_snapshots and (self.frame_count % self.snapshot_interval == 0):
                    snap_path = f"/home/vat/yolo_vehicle_test/results/tracking/track_snap_{self.frame_count:04d}.jpg"
                    cv2.imwrite(snap_path, annotated_frame)

                if self.video_writer is not None:
                    self.video_writer.write(annotated_frame)

                if not self.headless:
                    cv2.imshow("VIRYA Buggy - Multi-Object Tracking", annotated_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        break
                    elif key == ord("s"):
                        snap = f"/home/vat/yolo_vehicle_test/results/tracking/manual_track_{int(time.time())}.jpg"
                        cv2.imwrite(snap, annotated_frame)
                        print(f"📸 Manual snapshot saved: {snap}")

                if self.max_frames and self.frame_count >= self.max_frames:
                    print(f"\nReached max frame limit of {self.max_frames}. Ending test.")
                    break

        finally:
            self._cleanup()
            self._print_final_summary(start_wall_time)

    def _render_tracks(
        self, frame: np.ndarray, result, latency_ms: float
    ) -> Tuple[np.ndarray, List[Dict]]:
        canvas = frame.copy()
        h, w = canvas.shape[:2]
        tracked_objects = []

        # 1. Overlay Segmentation Masks if available
        if result.masks is not None and len(result.masks) > 0:
            mask_overlay = np.zeros_like(canvas, dtype=np.uint8)
            masks_data = result.masks.data.cpu().numpy()

            for i, mask in enumerate(masks_data):
                cls_id = int(result.boxes.cls[i])
                if cls_id not in self.target_class_ids:
                    continue

                if mask.shape[:2] != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

                color = self.class_metadata.get(cls_id, {}).get("color", (0, 255, 0))
                binary_mask = (mask > 0.5).astype(np.uint8)
                mask_overlay[binary_mask == 1] = color

            cv2.addWeighted(mask_overlay, 0.35, canvas, 1.0, 0, canvas)

        # 2. Process Tracked Bounding Boxes & Identifiers
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes
            has_track_ids = boxes.id is not None
            track_ids = boxes.id.int().cpu().tolist() if has_track_ids else [-1] * len(boxes)

            for i, box in enumerate(boxes):
                cls_id = int(box.cls[0])
                if cls_id not in self.target_class_ids:
                    continue

                track_id = track_ids[i]
                conf = float(box.conf[0])
                class_name = self.model.names.get(cls_id, f"ID_{cls_id}")
                color = self.class_metadata.get(cls_id, {}).get("color", (0, 255, 0))

                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                if track_id != -1:
                    self.total_unique_tracks.add(track_id)
                    self.track_history[track_id].append((cx, cy))
                    self.track_class_map[track_id] = class_name
                    id_label = f"ID: {track_id} | {class_name.upper()} {conf:.2f}"
                else:
                    id_label = f"{class_name.upper()} {conf:.2f}"

                # Draw bounding box
                cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

                # Draw track centroid point
                cv2.circle(canvas, (cx, cy), 4, color, -1)

                # Draw motion history trajectory line
                if track_id != -1 and len(self.track_history[track_id]) > 1:
                    points = np.array(self.track_history[track_id], dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(canvas, [points], isClosed=False, color=color, thickness=2)

                # Text label badge
                (txt_w, txt_h), baseline = cv2.getTextSize(id_label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
                tag_y1 = max(0, y1 - txt_h - 6)
                tag_y2 = y1
                cv2.rectangle(canvas, (x1, tag_y1), (x1 + txt_w + 6, tag_y2), color, -1)
                cv2.putText(
                    canvas, id_label, (x1 + 3, tag_y2 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1, cv2.LINE_AA
                )

                tracked_objects.append({
                    "id": track_id,
                    "class": class_name,
                    "class_id": cls_id,
                    "conf": conf,
                    "bbox": [x1, y1, x2, y2],
                    "center": (cx, cy)
                })

        # Telemetry Banner
        header_h = 42
        banner = canvas.copy()
        cv2.rectangle(banner, (0, 0), (w, header_h), (25, 25, 25), -1)
        cv2.addWeighted(banner, 0.75, canvas, 0.25, 0, canvas)

        status_text = (
            f"VIRYA TRACKING  |  ByteTrack  |  "
            f"Latency: {latency_ms:.1f}ms  |  "
            f"FPS: {self.fps_rolling:.1f}  |  "
            f"Active: {len(tracked_objects)}  |  "
            f"Total IDs: {len(self.total_unique_tracks)}"
        )
        cv2.putText(canvas, status_text, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)

        return canvas, tracked_objects

    def _cleanup(self):
        print("\n🧹 Cleaning up tracker resources...")
        if self.cap is not None:
            self.cap.release()
            print("✅ Camera released.")
        if self.video_writer is not None:
            self.video_writer.release()
            print("✅ Tracking video saved.")
        if not self.headless:
            cv2.destroyAllWindows()

    def _print_final_summary(self, start_wall_time: float):
        total_wall_time = time.time() - start_wall_time
        avg_fps = self.frame_count / total_wall_time if total_wall_time > 0 else 0.0

        print("\n" + "=" * 70)
        print("                 TRACKING SESSION SUMMARY")
        print("=" * 70)
        print(f"Total Frames Processed   : {self.frame_count}")
        print(f"Total Unique Objects     : {len(self.total_unique_tracks)}")
        print(f"Unique Track IDs Created : {sorted(list(self.total_unique_tracks))}")
        print(f"Average Pipeline FPS     : {avg_fps:.2f} FPS")

        if self.latencies:
            print(f"Inference Latency (Mean)   : {np.mean(self.latencies):.2f} ms")
            print(f"Inference Latency (Median) : {np.median(self.latencies):.2f} ms")
            print(f"Inference Latency (P95)    : {np.percentile(self.latencies, 95):.2f} ms")

        print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="VIRYA DBW Buggy - Multi-Object Tracking Engine (ByteTrack)")
    parser.add_argument("--model", type=str, default="/home/vat/yolo_vehicle_test/yolo26n-seg.pt")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-video", type=str, default=None)
    parser.add_argument("--save-snapshots", action="store_true")
    parser.add_argument("--snapshot-interval", type=int, default=30)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--config", type=str, default="/home/vat/yolo_vehicle_test/config/object_classes.yaml")

    args = parser.parse_args()

    if not os.environ.get("DISPLAY"):
        if not args.headless:
            print("ℹ️ No graphical DISPLAY detected. Automatically enabling --headless mode.")
            args.headless = True

    tracker = ObjectTracker(
        model_path=args.model,
        camera_id=args.camera,
        conf_threshold=args.conf,
        classes_config_path=args.config,
        headless=args.headless,
        save_video_path=args.save_video,
        save_snapshots=args.save_snapshots,
        snapshot_interval=args.snapshot_interval,
        max_frames=args.max_frames,
    )
    tracker.run()


if __name__ == "__main__":
    main()
