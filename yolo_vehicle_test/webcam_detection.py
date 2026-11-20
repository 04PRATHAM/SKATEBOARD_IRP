#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Multi-Object Perception System
Module: webcam_detection.py

Phase 1: Standalone Webcam Detection and Instance Segmentation
Model: YOLO26n-seg (CPU Inference)
Target Categories: Person, Bench, Chair, Dining Table

Author: Robotics Software Engineering Team
Permitted Workspace: /home/vat/yolo_vehicle_test/
"""

import os
import sys
import time
import argparse
import signal
from typing import Dict, List, Tuple, Optional
import numpy as np
import cv2
import yaml

# Ensure Ultralytics writes configuration inside our permitted workspace
os.environ["YOLO_CONFIG_DIR"] = "/home/vat/yolo_vehicle_test/.config/Ultralytics"

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ERROR: Ultralytics library not found in current environment.")
    print("   Please execute with the local virtual environment:")
    print("   /home/vat/yolo_vehicle_test/venv/bin/python3 webcam_detection.py")
    sys.exit(1)


class WebcamDetector:
    """
    Real-time multi-object perception and instance segmentation engine.
    Processes live video streams, applies class filtering, renders segmentation
    masks and bounding boxes, and records latency performance metrics.
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
    ):
        self.model_path = model_path
        self.camera_id = camera_id
        self.conf_threshold = conf_threshold
        self.headless = headless
        self.save_video_path = save_video_path
        self.save_snapshots = save_snapshots
        self.snapshot_interval = snapshot_interval
        self.max_frames = max_frames
        self.running = True

        # Performance tracking
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.latencies: List[float] = []
        self.fps_rolling: float = 0.0
        self.object_counts_history: List[int] = []

        # Load class metadata
        self.class_metadata = self._load_class_metadata(classes_config_path)

        # Target class IDs (defaults: 0=person, 13=bench, 56=chair, 60=dining table)
        if target_class_ids is not None:
            self.target_class_ids = set(target_class_ids)
        else:
            self.target_class_ids = {0, 13, 56, 60}

        # Initialize Model
        self.model = self._initialize_model()

        # Initialize Video Capture
        self.cap = None
        self.video_writer = None
        self._initialize_camera()

        # Signal handlers for clean termination
        signal.signal(signal.SIGINT, self._sigint_handler)
        signal.signal(signal.SIGTERM, self._sigint_handler)

    def _sigint_handler(self, sig, frame):
        print("\n🛑 Termination signal received. Releasing resources gracefully...")
        self.running = False

    def _load_class_metadata(self, config_path: str) -> Dict:
        """Loads class color and priority metadata from YAML configuration."""
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
                print(f"⚠️ Warning: Could not parse {config_path}: {e}. Using defaults.")
        return default_metadata

    def _initialize_model(self) -> YOLO:
        """Initializes and verifies the YOLO26n-seg segmentation model."""
        print("=" * 70)
        print("🔍 INITIALIZING YOLO26n-seg PERCEPTION ENGINE")
        print("=" * 70)

        if not os.path.exists(self.model_path):
            print(f"❌ FATAL ERROR: Model file not found at: {self.model_path}")
            sys.exit(1)

        print(f"📂 Loading weights from: {self.model_path}")
        t0 = time.perf_counter()
        try:
            model = YOLO(self.model_path)
            load_time = (time.perf_counter() - t0) * 1000.0
            print(f"✅ Model loaded successfully in {load_time:.1f} ms.")
        except Exception as e:
            print(f"❌ FATAL ERROR: Failed to instantiate YOLO model: {e}")
            sys.exit(1)

        # Verify class definitions against model
        print("\n📋 Verifying Target Object Categories:")
        for cid in sorted(self.target_class_ids):
            name_in_model = model.names.get(cid, "UNKNOWN")
            print(f"   - Class ID {cid:02d}: '{name_in_model}' (Filtered Target)")

        return model

    def _initialize_camera(self):
        """Initializes USB webcam with V4L2 backend and safety fallback."""
        print("\n📷 INITIALIZING USB WEBCAM")
        print(f"   Target Device: /dev/video{self.camera_id}")

        # Attempt V4L2 backend first
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("   ⚠️ V4L2 backend failed, attempting default capture backend...")
            self.cap = cv2.VideoCapture(self.camera_id)

        if not self.cap.isOpened():
            print(f"❌ FATAL ERROR: Unable to access camera device /dev/video{self.camera_id}")
            print("   Verify that the USB webcam is connected and user permissions are granted.")
            sys.exit(1)

        # Configure 640x480 resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        print(f"✅ Camera opened successfully: {actual_w}x{actual_h} @ {actual_fps:.1f} FPS")

        # Initialize Video Writer if requested
        if self.save_video_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.save_video_path)), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(
                self.save_video_path, fourcc, 20.0, (actual_w, actual_h)
            )
            print(f"💾 Recording output stream to: {self.save_video_path}")

    def run(self):
        """Main detection and rendering loop."""
        print("\n" + "=" * 70)
        print("🚀 STARTING REAL-TIME WEBCAM DETECTION STREAM")
        print("=" * 70)
        if not self.headless:
            print("Keyboard Controls: [Q] Quit  |  [S] Save Snapshot Frame")
            cv2.namedWindow("VIRYA Buggy - YOLO26n-seg Perception", cv2.WINDOW_NORMAL)

        consecutive_failures = 0
        max_consecutive_failures = 15

        start_wall_time = time.time()

        try:
            while self.running:
                # Frame acquisition
                ret, frame = self.cap.read()

                if not ret:
                    consecutive_failures += 1
                    print(f"⚠️ Warning: Failed to read frame from webcam ({consecutive_failures}/{max_consecutive_failures})")
                    if consecutive_failures >= max_consecutive_failures:
                        print("❌ FATAL: Camera stream lost or disconnected. Halting loop.")
                        break
                    time.sleep(0.05)
                    continue

                consecutive_failures = 0
                self.frame_count += 1

                # Inference
                t_infer_start = time.perf_counter()
                results = self.model.predict(
                    source=frame,
                    conf=self.conf_threshold,
                    device="cpu",
                    verbose=False,
                    classes=list(self.target_class_ids)
                )
                t_infer_end = time.perf_counter()
                latency_ms = (t_infer_end - t_infer_start) * 1000.0
                self.latencies.append(latency_ms)
                self.total_inference_time += (t_infer_end - t_infer_start)

                # Instantaneous and rolling FPS
                instant_fps = 1000.0 / latency_ms if latency_ms > 0 else 0.0
                self.fps_rolling = 0.9 * self.fps_rolling + 0.1 * instant_fps if self.frame_count > 1 else instant_fps

                # Render detections & segmentation masks
                annotated_frame, detections_summary = self._render_results(frame, results[0], latency_ms)
                self.object_counts_history.append(len(detections_summary))

                # Periodic terminal telemetry
                if self.frame_count % 10 == 0 or len(detections_summary) > 0:
                    summary_str = ", ".join([f"{d['class']} ({d['conf']:.2f})" for d in detections_summary])
                    if not summary_str:
                        summary_str = "Clear (0 objects)"
                    print(
                        f"Frame {self.frame_count:04d} | "
                        f"Latency: {latency_ms:5.1f} ms | "
                        f"FPS: {self.fps_rolling:4.1f} | "
                        f"Detected: {summary_str}"
                    )

                # Periodic snapshots if enabled
                if self.save_snapshots and (self.frame_count % self.snapshot_interval == 0):
                    snap_path = f"/home/vat/yolo_vehicle_test/results/detection/snapshot_{self.frame_count:04d}.jpg"
                    cv2.imwrite(snap_path, annotated_frame)

                # Write to video if configured
                if self.video_writer is not None:
                    self.video_writer.write(annotated_frame)

                # GUI Window display
                if not self.headless:
                    cv2.imshow("VIRYA Buggy - YOLO26n-seg Perception", annotated_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:  # 'q' or ESC
                        print("\nUser requested exit via GUI.")
                        break
                    elif key == ord("s"):
                        manual_snap = f"/home/vat/yolo_vehicle_test/results/detection/manual_snap_{int(time.time())}.jpg"
                        cv2.imwrite(manual_snap, annotated_frame)
                        print(f"📸 Manual snapshot saved: {manual_snap}")

                # Check max frames limit (useful for benchmarks/testing)
                if self.max_frames and self.frame_count >= self.max_frames:
                    print(f"\nReached max frame limit of {self.max_frames}. Ending test.")
                    break

        finally:
            self._cleanup()
            self._print_final_summary(start_wall_time)

    def _render_results(
        self, original_frame: np.ndarray, result, latency_ms: float
    ) -> Tuple[np.ndarray, List[Dict]]:
        """
        Draws segmentation masks, bounding boxes, and performance overlays.
        """
        canvas = original_frame.copy()
        h, w = canvas.shape[:2]
        detections = []

        # 1. Overlay Segmentation Masks if available
        if result.masks is not None and len(result.masks) > 0:
            mask_overlay = np.zeros_like(canvas, dtype=np.uint8)
            masks_data = result.masks.data.cpu().numpy()  # [N, H, W]

            for i, mask in enumerate(masks_data):
                cls_id = int(result.boxes.cls[i])
                if cls_id not in self.target_class_ids:
                    continue

                # Resize mask to frame dimensions if necessary
                if mask.shape[:2] != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)

                color = self.class_metadata.get(cls_id, {}).get("color", (0, 255, 0))
                binary_mask = (mask > 0.5).astype(np.uint8)
                mask_overlay[binary_mask == 1] = color

            # Blend mask with alpha transparency
            alpha = 0.40
            cv2.addWeighted(mask_overlay, alpha, canvas, 1.0, 0, canvas)

        # 2. Draw Bounding Boxes and Labels
        if result.boxes is not None and len(result.boxes) > 0:
            for i, box in enumerate(result.boxes):
                cls_id = int(box.cls[0])
                if cls_id not in self.target_class_ids:
                    continue

                conf = float(box.conf[0])
                class_name = self.model.names.get(cls_id, f"ID_{cls_id}")
                color = self.class_metadata.get(cls_id, {}).get("color", (0, 255, 0))

                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w - 1, x2), min(h - 1, y2)

                # Draw bounding box rectangle
                cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

                # Label tag
                label_text = f"{class_name.upper()} {conf:.2f}"
                (txt_w, txt_h), baseline = cv2.getTextSize(
                    label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
                )
                tag_y1 = max(0, y1 - txt_h - 6)
                tag_y2 = y1

                # Background badge for text
                cv2.rectangle(
                    canvas, (x1, tag_y1), (x1 + txt_w + 6, tag_y2), color, -1
                )
                cv2.putText(
                    canvas,
                    label_text,
                    (x1 + 3, tag_y2 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 0),  # Black text for high contrast on badge
                    1,
                    cv2.LINE_AA,
                )

                detections.append({
                    "class": class_name,
                    "class_id": cls_id,
                    "conf": conf,
                    "bbox": [x1, y1, x2, y2],
                    "center": ((x1 + x2) // 2, (y1 + y2) // 2)
                })

        # 3. Draw Telemetry Overlay Header
        header_h = 42
        overlay_banner = canvas.copy()
        cv2.rectangle(overlay_banner, (0, 0), (w, header_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay_banner, 0.75, canvas, 0.25, 0, canvas)

        status_text = (
            f"VIRYA PERCEPTION  |  YOLO26n-seg (CPU)  |  "
            f"Latency: {latency_ms:.1f}ms  |  "
            f"FPS: {self.fps_rolling:.1f}  |  "
            f"Objects: {len(detections)}"
        )
        cv2.putText(
            canvas,
            status_text,
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        return canvas, detections

    def _cleanup(self):
        """Releases all video streams, writers, and GUI windows."""
        print("\n🧹 Executing cleanup sequence...")
        if self.cap is not None:
            self.cap.release()
            print("✅ Webcam device released.")
        if self.video_writer is not None:
            self.video_writer.release()
            print("✅ Output video writer finalized.")
        if not self.headless:
            cv2.destroyAllWindows()
            print("✅ OpenCV GUI windows closed.")

    def _print_final_summary(self, start_wall_time: float):
        """Displays final session statistics and latency metrics."""
        total_wall_time = time.time() - start_wall_time
        avg_fps = self.frame_count / total_wall_time if total_wall_time > 0 else 0.0

        print("\n" + "=" * 70)
        print("                 PERCEPTION SESSION SUMMARY")
        print("=" * 70)
        print(f"Total Frames Processed : {self.frame_count}")
        print(f"Total Session Duration : {total_wall_time:.2f} s")
        print(f"Average Pipeline FPS   : {avg_fps:.2f} FPS")

        if self.latencies:
            mean_lat = np.mean(self.latencies)
            median_lat = np.median(self.latencies)
            p95_lat = np.percentile(self.latencies, 95)
            min_lat = np.min(self.latencies)
            max_lat = np.max(self.latencies)

            print(f"Inference Latency (Mean)   : {mean_lat:.2f} ms")
            print(f"Inference Latency (Median) : {median_lat:.2f} ms")
            print(f"Inference Latency (P95)    : {p95_lat:.2f} ms")
            print(f"Inference Latency (Min/Max): {min_lat:.2f} ms / {max_lat:.2f} ms")

        if self.object_counts_history:
            avg_objects = np.mean(self.object_counts_history)
            max_objects = np.max(self.object_counts_history)
            print(f"Average Objects per Frame  : {avg_objects:.2f}")
            print(f"Peak Objects in a Frame    : {max_objects}")

        print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="VIRYA DBW Buggy - Real-Time Webcam YOLO26n-seg Multi-Object Perception"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="/home/vat/yolo_vehicle_test/yolo26n-seg.pt",
        help="Path to YOLO segmentation weights",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera device index (default: 0 for /dev/video0)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence detection threshold (default: 0.25)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without displaying GUI window (auto-enabled if DISPLAY is absent)",
    )
    parser.add_argument(
        "--save-video",
        type=str,
        default=None,
        help="Path to save output recorded video (e.g. results/detection/webcam_test.mp4)",
    )
    parser.add_argument(
        "--save-snapshots",
        action="store_true",
        help="Save periodic snapshot images into results/detection/",
    )
    parser.add_argument(
        "--snapshot-interval",
        type=int,
        default=30,
        help="Interval of frames between saved snapshots (default: 30)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Exit after processing max frames (useful for automated verification)",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="/home/vat/yolo_vehicle_test/config/object_classes.yaml",
        help="Path to class metadata configuration YAML",
    )

    args = parser.parse_args()

    # Automatically set headless if DISPLAY is not present
    if not os.environ.get("DISPLAY"):
        if not args.headless:
            print("ℹ️ No graphical DISPLAY detected. Automatically enabling --headless mode.")
            args.headless = True

    detector = WebcamDetector(
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

    detector.run()


if __name__ == "__main__":
    main()
