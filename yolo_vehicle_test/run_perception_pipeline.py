#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Integrated Perception Pipeline
Module: run_perception_pipeline.py

Phase 5: Unified Multi-Object Perception, ByteTrack Tracking,
Spatial Corridor Positioning, and Intelligent Safety Decision Engine.

Flow:
  Webcam / Video Source -> Single-Pass YOLO26n-seg + ByteTrack ->
  Mask Geometry & Spatial Positioning -> Corridor Occlusion Analysis ->
  Fail-Safe Decision Reasoning -> Single-Pass HUD Rendering ->
  Structured JSONL Telemetry & Video Output.

CRITICAL VEHICLE SAFETY RESTRICTION:
This pipeline operates strictly in OFFLINE / ISOLATED perception mode.
No vehicle control commands are transmitted to CAN, Modbus, or ROS 2 actuation topics.

Author: Robotics Software Engineering Team
Permitted Workspace: /home/vat/yolo_vehicle_test/
"""

import os
import sys
import time
import json
import argparse
import signal
import threading
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple, Optional
from collections import defaultdict, deque
import numpy as np
import cv2
import yaml
import psutil

# Ensure Ultralytics writes configuration inside our permitted workspace
os.environ["YOLO_CONFIG_DIR"] = "/home/vat/yolo_vehicle_test/.config/Ultralytics"

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ERROR: Ultralytics library not found in environment.")
    sys.exit(1)


class NavigationDecision(str, Enum):
    MOVE_LEFT = "MOVE_LEFT"
    MOVE_RIGHT = "MOVE_RIGHT"
    SLOW_DOWN = "SLOW_DOWN"
    STOP = "STOP"
    PATH_CLEAR = "PATH_CLEAR"
    UNKNOWN = "UNKNOWN"


@dataclass
class TrackedObstacle:
    """Represents a spatially analyzed tracked obstacle instance."""
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]          # (x1, y1, x2, y2)
    center_pixel: Tuple[int, int]            # (cx, cy)
    center_norm: Tuple[float, float]         # (cx / width, cy / height)
    mask_pixel_area: int                     # Number of pixels in segmentation mask
    horizontal_zone: str                     # "LEFT", "CENTER", "RIGHT"
    corridor_overlap_ratio: float            # Ratio [0.0, 1.0] of mask inside corridor
    in_corridor: bool                        # Boolean indicating path intrusion
    metric_depth_meters: Optional[float]     # Reserved for future RGB-D depth


@dataclass
class PipelineTelemetry:
    """Structured telemetry data recorded per frame."""
    frame_id: int
    timestamp: float
    infer_latency_ms: float
    total_pipeline_latency_ms: float
    fps_instant: float
    fps_rolling: float
    cpu_percent: float
    memory_rss_mb: float
    active_objects_count: int
    corridor_intruders_count: int
    decision: str
    decision_reason: str
    suggested_steer: str
    suggested_speed_factor: float
    left_clearance: float
    right_clearance: float
    primary_obstacle: Optional[str]


@dataclass
class DecisionResult:
    """Decision reasoning and navigation recommendation output."""
    decision: NavigationDecision
    reason: str
    primary_obstacle: Optional[str]
    total_objects: int
    corridor_intruders: int
    left_clearance_score: float
    right_clearance_score: float
    suggested_speed_factor: float
    suggested_steer_dir: str


class IntegratedPerceptionPipeline:
    """
    High-performance unified perception engine combining YOLO26n-seg,
    ByteTrack, spatial corridor geometry, and navigation safety decisions.
    """

    def __init__(
        self,
        model_path: str = "/home/vat/yolo_vehicle_test/yolo26n-seg.pt",
        source: str = "0",
        conf_threshold: float = 0.25,
        classes_config: str = "/home/vat/yolo_vehicle_test/config/object_classes.yaml",
        corridor_bounds: Tuple[float, float] = (0.25, 0.75),
        corridor_overlap_thresh: float = 0.10,
        headless: bool = False,
        save_video: Optional[str] = None,
        log_jsonl: str = "/home/vat/yolo_vehicle_test/logs/pipeline_telemetry.jsonl",
        max_frames: Optional[int] = None,
        consecutive_clear_required: int = 3,
    ):
        self.model_path = model_path
        self.source_str = source
        self.conf_threshold = conf_threshold
        self.corridor_left = corridor_bounds[0]
        self.corridor_right = corridor_bounds[1]
        self.corridor_overlap_thresh = corridor_overlap_thresh
        self.headless = headless
        self.save_video_path = save_video
        self.log_jsonl_path = log_jsonl
        self.max_frames = max_frames
        self.consecutive_clear_required = consecutive_clear_required
        self.running = True

        # Process monitor for CPU and RAM utilization
        self.process = psutil.Process(os.getpid())

        # Rolling state
        self.frame_count = 0
        self.consecutive_clear_frames = 0
        self.fps_rolling = 0.0
        self.infer_latencies: List[float] = []
        self.pipeline_latencies: List[float] = []
        self.cpu_usages: List[float] = []
        self.memory_usages: List[float] = []
        self.tracked_history = defaultdict(lambda: deque(maxlen=30))
        self.total_unique_track_ids = set()

        # Load class metadata & targets
        self.class_metadata = self._load_classes(classes_config)
        self.target_class_ids = set(self.class_metadata.keys())

        # Initialize Model
        self.model = self._initialize_model()

        # Initialize Capture Source
        self.cap = None
        self.is_video_file = False
        self._initialize_source()

        # Initialize Output Writers
        self.video_writer = None
        if self.save_video_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.save_video_path)), exist_ok=True)
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(self.save_video_path, fourcc, 20.0, (w, h))

        os.makedirs(os.path.dirname(os.path.abspath(self.log_jsonl_path)), exist_ok=True)
        self.log_file = open(self.log_jsonl_path, "a")

        if threading.current_thread() is threading.main_thread():
            signal.signal(signal.SIGINT, self._sigint_handler)
            signal.signal(signal.SIGTERM, self._sigint_handler)

    def _sigint_handler(self, sig, frame):
        print("\n🛑 Pipeline interrupt signal received. Shutting down gracefully...")
        self.running = False

    def _load_classes(self, config_path: str) -> Dict[int, Dict]:
        defaults = {
            0: {"name": "person", "color": (0, 255, 0), "priority": 1},
            13: {"name": "bench", "color": (0, 165, 255), "priority": 2},
            56: {"name": "chair", "color": (255, 144, 30), "priority": 2},
            60: {"name": "dining table", "color": (0, 215, 255), "priority": 2},
        }
        if os.path.exists(config_path):
            try:
                with open(config_path, "r") as f:
                    data = yaml.safe_load(f)
                    if "target_classes" in data:
                        meta = {}
                        for k, v in data["target_classes"].items():
                            cid = v.get("id")
                            if cid is not None:
                                meta[cid] = {
                                    "name": v.get("name"),
                                    "color": tuple(v.get("color", [0, 255, 0])),
                                    "priority": v.get("priority", 2),
                                }
                        return meta
            except Exception as e:
                print(f"⚠️ Error reading {config_path}: {e}")
        return defaults

    def _initialize_model(self) -> YOLO:
        print("=" * 75)
        print("🔍 INITIALIZING UNIFIED PERCEPTION PIPELINE (YOLO26n-seg + ByteTrack)")
        print("=" * 75)
        if not os.path.exists(self.model_path):
            print(f"❌ Model file not found at: {self.model_path}")
            sys.exit(1)

        t0 = time.perf_counter()
        model = YOLO(self.model_path)
        load_ms = (time.perf_counter() - t0) * 1000.0
        print(f"✅ Neural model initialized in {load_ms:.1f} ms.")

        print("📋 Active Filtered Classes:")
        for cid in sorted(self.target_class_ids):
            cname = model.names.get(cid, "UNKNOWN")
            print(f"   [ID {cid:02d}] {cname:15s} (Priority {self.class_metadata[cid]['priority']})")
        return model

    def _initialize_source(self):
        # Check if source is integer camera index or video path
        if self.source_str.isdigit():
            cam_idx = int(self.source_str)
            print(f"📷 Opening Live USB Camera: /dev/video{cam_idx}")
            self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                print("   ⚠️ V4L2 backend unavailable, falling back to default capture...")
                self.cap = cv2.VideoCapture(cam_idx)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.is_video_file = False
        else:
            if not os.path.exists(self.source_str):
                print(f"❌ Source file not found: {self.source_str}")
                sys.exit(1)
            print(f"🎬 Opening Video Input Stream: {self.source_str}")
            self.cap = cv2.VideoCapture(self.source_str)
            self.is_video_file = True

        if not self.cap.isOpened():
            print(f"❌ FATAL ERROR: Failed to open capture source: {self.source_str}")
            sys.exit(1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"✅ Video stream active: {actual_w}x{actual_h} @ {fps:.1f} FPS")

    def run(self):
        print("\n" + "=" * 75)
        print("🚀 STARTING INTEGRATED PERCEPTION PIPELINE EXECUTION")
        print("   Mode: OFFLINE SAFETY EVALUATION (Actuation Disconnected)")
        print(f"   Virtual Corridor: [{self.corridor_left*100:.0f}%, {self.corridor_right*100:.0f}%] of FOV")
        print("=" * 75)

        if not self.headless:
            cv2.namedWindow("VIRYA Buggy - Integrated Perception Pipeline", cv2.WINDOW_NORMAL)

        consecutive_failures = 0
        start_wall_time = time.time()

        try:
            while self.running:
                t_pipe_start = time.perf_counter()

                # Step 1: Frame Acquisition
                ret, frame = self.cap.read()
                if not ret:
                    consecutive_failures += 1
                    if self.is_video_file or consecutive_failures >= 15:
                        print("\n🏁 Input stream ended or camera disconnected.")
                        break
                    time.sleep(0.05)
                    continue

                consecutive_failures = 0
                self.frame_count += 1

                # Step 2: Unified Inference + ByteTrack (Single-Pass)
                t_infer_start = time.perf_counter()
                results = self.model.track(
                    source=frame,
                    persist=True,
                    tracker="bytetrack.yaml",
                    conf=self.conf_threshold,
                    device="cpu",
                    verbose=False,
                    classes=list(self.target_class_ids),
                )
                t_infer_end = time.perf_counter()
                infer_latency_ms = (t_infer_end - t_infer_start) * 1000.0
                self.infer_latencies.append(infer_latency_ms)

                # Step 3: Spatial Corridor Positioning
                obstacles = self._process_spatial_obstacles(frame, results[0])

                # Step 4: Decision Engine Reasoning
                decision = self._evaluate_navigation_safety(obstacles, frame.shape)

                # Step 5: High-Performance Single-Pass HUD Rendering
                hud_frame = self._render_integrated_hud(frame, results[0], obstacles, decision, infer_latency_ms)

                # Total Pipeline Latency
                t_pipe_end = time.perf_counter()
                pipe_latency_ms = (t_pipe_end - t_pipe_start) * 1000.0
                self.pipeline_latencies.append(pipe_latency_ms)

                # Rolling FPS
                instant_fps = 1000.0 / pipe_latency_ms if pipe_latency_ms > 0 else 0.0
                self.fps_rolling = 0.9 * self.fps_rolling + 0.1 * instant_fps if self.frame_count > 1 else instant_fps

                # System resource utilization
                cpu_p = psutil.cpu_percent()
                mem_rss = self.process.memory_info().rss / (1024 * 1024)
                self.cpu_usages.append(cpu_p)
                self.memory_usages.append(mem_rss)

                # Record Structured Telemetry
                telemetry = PipelineTelemetry(
                    frame_id=self.frame_count,
                    timestamp=time.time(),
                    infer_latency_ms=round(infer_latency_ms, 2),
                    total_pipeline_latency_ms=round(pipe_latency_ms, 2),
                    fps_instant=round(instant_fps, 1),
                    fps_rolling=round(self.fps_rolling, 1),
                    cpu_percent=cpu_p,
                    memory_rss_mb=round(mem_rss, 1),
                    active_objects_count=len(obstacles),
                    corridor_intruders_count=sum(1 for o in obstacles if o.in_corridor),
                    decision=decision.decision.value,
                    decision_reason=decision.reason,
                    suggested_steer=decision.suggested_steer_dir,
                    suggested_speed_factor=decision.suggested_speed_factor,
                    left_clearance=round(decision.left_clearance_score, 2),
                    right_clearance=round(decision.right_clearance_score, 2),
                    primary_obstacle=decision.primary_obstacle,
                )
                self.log_file.write(json.dumps(asdict(telemetry)) + "\n")

                # Terminal Telemetry Line
                if self.frame_count % 10 == 0 or decision.decision == NavigationDecision.STOP:
                    intruders_str = f"{telemetry.corridor_intruders_count} Intruders"
                    print(
                        f"Frame {self.frame_count:04d} | "
                        f"Infer: {infer_latency_ms:5.1f} ms | "
                        f"Total: {pipe_latency_ms:5.1f} ms | "
                        f"FPS: {self.fps_rolling:4.1f} | "
                        f"Dec: {decision.decision.value:10s} | "
                        f"{intruders_str:11s} | {decision.reason[:40]}"
                    )

                if self.video_writer is not None:
                    self.video_writer.write(hud_frame)

                if not self.headless:
                    cv2.imshow("VIRYA Buggy - Integrated Perception Pipeline", hud_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        break

                if self.max_frames and self.frame_count >= self.max_frames:
                    print(f"\nReached max frame benchmark limit ({self.max_frames}).")
                    break

        finally:
            self._cleanup()
            self._print_benchmark_report(start_wall_time)

    def _process_spatial_obstacles(self, frame: np.ndarray, result) -> List[TrackedObstacle]:
        """Calculates precise mask areas, corridor overlap, and image coordinates."""
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

            track_id = track_ids[i]
            conf = float(box.conf[0])
            cname = self.model.names.get(cls_id, f"ID_{cls_id}")

            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            bw = x2 - x1
            bh = y2 - y1
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cx_norm = cx / float(w)
            cy_norm = cy / float(h)

            if track_id != -1:
                self.total_unique_track_ids.add(track_id)
                self.tracked_history[track_id].append((cx, cy))

            # Segmentation Mask Overlap Analysis
            mask_area = 0
            corridor_overlap_ratio = 0.0

            if masks is not None and len(masks) > i:
                raw_mask = masks.data[i].cpu().numpy()
                if raw_mask.shape[:2] != (h, w):
                    raw_mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST)
                binary_mask = (raw_mask > 0.5).astype(np.uint8)
                mask_area = int(np.sum(binary_mask))

                corridor_slice = binary_mask[:, c_x1:c_x2]
                overlap_pixels = int(np.sum(corridor_slice))
                if mask_area > 0:
                    corridor_overlap_ratio = overlap_pixels / float(mask_area)
            else:
                # Bounding box fallback if mask is absent
                mask_area = bw * bh
                ox1 = max(x1, c_x1)
                ox2 = min(x2, c_x2)
                if ox2 > ox1:
                    corridor_overlap_ratio = (ox2 - ox1) / float(bw)

            # Zone determination
            if cx_norm < self.corridor_left:
                zone = "LEFT"
            elif cx_norm > self.corridor_right:
                zone = "RIGHT"
            else:
                zone = "CENTER"

            in_corridor = (corridor_overlap_ratio >= self.corridor_overlap_thresh) or (zone == "CENTER")

            obs = TrackedObstacle(
                track_id=track_id,
                class_id=cls_id,
                class_name=cname,
                confidence=conf,
                bbox=(x1, y1, x2, y2),
                center_pixel=(cx, cy),
                center_norm=(cx_norm, cy_norm),
                mask_pixel_area=mask_area,
                horizontal_zone=zone,
                corridor_overlap_ratio=corridor_overlap_ratio,
                in_corridor=in_corridor,
                metric_depth_meters=None,
            )
            obstacles.append(obs)

        return obstacles

    def _evaluate_navigation_safety(
        self, obstacles: List[TrackedObstacle], frame_shape: Tuple[int, int]
    ):
        """
        Robust fail-safe multi-object decision logic with human-priority gating
        and anti-chatter clear path verification.
        """
        h, w = frame_shape[:2]

        # Case: Empty frame (No detections)
        if not obstacles:
            self.consecutive_clear_frames += 1
            if self.consecutive_clear_frames < self.consecutive_clear_required:
                # Require consecutive clear frames before confirming path is clear
                return DecisionResult(
                    decision=NavigationDecision.SLOW_DOWN,
                    reason=f"Verifying clear path ({self.consecutive_clear_frames}/{self.consecutive_clear_required} frames)",
                    primary_obstacle=None,
                    total_objects=0,
                    corridor_intruders=0,
                    left_clearance_score=1.0,
                    right_clearance_score=1.0,
                    suggested_speed_factor=0.60,
                    suggested_steer_dir="CENTER",
                )
            else:
                return DecisionResult(
                    decision=NavigationDecision.PATH_CLEAR,
                    reason="Driving corridor verified clear",
                    primary_obstacle=None,
                    total_objects=0,
                    corridor_intruders=0,
                    left_clearance_score=1.0,
                    right_clearance_score=1.0,
                    suggested_speed_factor=1.0,
                    suggested_steer_dir="CENTER",
                )

        self.consecutive_clear_frames = 0
        intruders = [o for o in obstacles if o.in_corridor]
        left_obs = [o for o in obstacles if o.horizontal_zone == "LEFT"]
        right_obs = [o for o in obstacles if o.horizontal_zone == "RIGHT"]

        # Clearance scores
        frame_area = float(h * w)
        left_occ = sum(o.mask_pixel_area for o in left_obs) / (0.35 * frame_area)
        right_occ = sum(o.mask_pixel_area for o in right_obs) / (0.35 * frame_area)
        left_clearance = max(0.0, 1.0 - left_occ)
        right_clearance = max(0.0, 1.0 - right_occ)

        # 1. PRIORITY 1: PERSON SAFETY
        persons_in_corridor = [o for o in intruders if o.class_name == "person"]
        if persons_in_corridor:
            p = persons_in_corridor[0]
            return DecisionResult(
                decision=NavigationDecision.STOP,
                reason=f"CRITICAL: Person in path (conf={p.confidence:.2f})",
                primary_obstacle="person",
                total_objects=len(obstacles),
                corridor_intruders=len(intruders),
                left_clearance_score=left_clearance,
                right_clearance_score=right_clearance,
                suggested_speed_factor=0.0,
                suggested_steer_dir="CENTER",
            )

        # 2. SEVERE PATH OCCLUSION: Multiple Intruders, Dual-Side Flanking, or Pinch-Points
        has_left_intruder = any(o.center_norm[0] <= 0.50 for o in intruders)
        has_right_intruder = any(o.center_norm[0] > 0.50 for o in intruders)

        if len(intruders) >= 3 or (has_left_intruder and has_right_intruder) or (len(intruders) >= 1 and left_clearance < 0.40 and right_clearance < 0.40):
            lead = intruders[0]
            return DecisionResult(
                decision=NavigationDecision.STOP,
                reason=f"Corridor blocked ({len(intruders)} intruders, dual-side encroachment)",
                primary_obstacle=f"{lead.class_name} (ID:{lead.track_id})",
                total_objects=len(obstacles),
                corridor_intruders=len(intruders),
                left_clearance_score=left_clearance,
                right_clearance_score=right_clearance,
                suggested_speed_factor=0.0,
                suggested_steer_dir="CENTER",
            )

        # 3. LATERAL AVOIDANCE BYPASS
        if len(intruders) > 0:
            lead = intruders[0]
            avg_x = np.mean([o.center_norm[0] for o in intruders])

            # If obstacle is on left of corridor and right side is sufficiently clear
            if avg_x < 0.48 and right_clearance >= 0.50:
                return DecisionResult(
                    decision=NavigationDecision.MOVE_RIGHT,
                    reason=f"Bypass {lead.class_name} to the right (R_clear={right_clearance:.2f})",
                    primary_obstacle=f"{lead.class_name} (ID:{lead.track_id})",
                    total_objects=len(obstacles),
                    corridor_intruders=len(intruders),
                    left_clearance_score=left_clearance,
                    right_clearance_score=right_clearance,
                    suggested_speed_factor=0.50,
                    suggested_steer_dir="RIGHT",
                )
            # If obstacle is on right of corridor and left side is sufficiently clear
            elif avg_x > 0.52 and left_clearance >= 0.50:
                return DecisionResult(
                    decision=NavigationDecision.MOVE_LEFT,
                    reason=f"Bypass {lead.class_name} to the left (L_clear={left_clearance:.2f})",
                    primary_obstacle=f"{lead.class_name} (ID:{lead.track_id})",
                    total_objects=len(obstacles),
                    corridor_intruders=len(intruders),
                    left_clearance_score=left_clearance,
                    right_clearance_score=right_clearance,
                    suggested_speed_factor=0.50,
                    suggested_steer_dir="LEFT",
                )
            else:
                return DecisionResult(
                    decision=NavigationDecision.STOP,
                    reason=f"Corridor blocked by {lead.class_name}; insufficient lateral clearance to steer",
                    primary_obstacle=f"{lead.class_name} (ID:{lead.track_id})",
                    total_objects=len(obstacles),
                    corridor_intruders=len(intruders),
                    left_clearance_score=left_clearance,
                    right_clearance_score=right_clearance,
                    suggested_speed_factor=0.0,
                    suggested_steer_dir="CENTER",
                )

        # 4. PERIPHERAL CAUTION: Object encroaching corridor edge
        marginal = [
            o for o in obstacles
            if abs(o.center_norm[0] - self.corridor_left) < 0.08
            or abs(o.center_norm[0] - self.corridor_right) < 0.08
        ]
        if marginal:
            lead = marginal[0]
            return DecisionResult(
                decision=NavigationDecision.SLOW_DOWN,
                reason=f"Caution: {lead.class_name} close to corridor edge",
                primary_obstacle=f"{lead.class_name} (ID:{lead.track_id})",
                total_objects=len(obstacles),
                corridor_intruders=0,
                left_clearance_score=left_clearance,
                right_clearance_score=right_clearance,
                suggested_speed_factor=0.70,
                suggested_steer_dir="CENTER",
            )

        return DecisionResult(
            decision=NavigationDecision.PATH_CLEAR,
            reason="Corridor completely clear",
            primary_obstacle=None,
            total_objects=len(obstacles),
            corridor_intruders=0,
            left_clearance_score=left_clearance,
            right_clearance_score=right_clearance,
            suggested_speed_factor=1.0,
            suggested_steer_dir="CENTER",
        )

    def _render_integrated_hud(
        self,
        frame: np.ndarray,
        result,
        obstacles: List[TrackedObstacle],
        decision,
        infer_latency_ms: float,
    ) -> np.ndarray:
        """High-efficiency single-pass HUD and overlay rendering."""
        canvas = frame.copy()
        h, w = canvas.shape[:2]
        c_x1 = int(self.corridor_left * w)
        c_x2 = int(self.corridor_right * w)

        # 1. Driving Corridor Underlay
        corridor_overlay = canvas.copy()
        cv2.rectangle(corridor_overlay, (c_x1, 0), (c_x2, h), (180, 50, 180), -1)
        cv2.addWeighted(corridor_overlay, 0.12, canvas, 0.88, 0, canvas)
        cv2.line(canvas, (c_x1, 0), (c_x1, h), (255, 0, 255), 2, cv2.LINE_AA)
        cv2.line(canvas, (c_x2, 0), (c_x2, h), (255, 0, 255), 2, cv2.LINE_AA)

        # 2. Instance Segmentation Masks
        if result.masks is not None and len(result.masks) > 0:
            mask_overlay = np.zeros_like(canvas, dtype=np.uint8)
            masks_data = result.masks.data.cpu().numpy()
            for i, mask in enumerate(masks_data):
                cid = int(result.boxes.cls[i])
                if cid not in self.target_class_ids:
                    continue
                if mask.shape[:2] != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                color = self.class_metadata.get(cid, {}).get("color", (0, 255, 0))
                mask_overlay[mask > 0.5] = color
            cv2.addWeighted(mask_overlay, 0.35, canvas, 1.0, 0, canvas)

        # 3. Bounding Boxes, Tracks, and Badges
        for obs in obstacles:
            x1, y1, x2, y2 = obs.bbox
            color = self.class_metadata.get(obs.class_id, {}).get("color", (0, 255, 0))
            box_color = (0, 0, 255) if obs.in_corridor else color
            thickness = 3 if obs.in_corridor else 2

            cv2.rectangle(canvas, (x1, y1), (x2, y2), box_color, thickness)
            cx, cy = obs.center_pixel
            cv2.drawMarker(canvas, (cx, cy), box_color, cv2.MARKER_CROSS, 10, 2)

            # Motion trajectory polyline
            if obs.track_id != -1 and len(self.tracked_history[obs.track_id]) > 1:
                pts = np.array(self.tracked_history[obs.track_id], dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(canvas, [pts], isClosed=False, color=box_color, thickness=2)

            # Object Badge
            id_prefix = f"ID:{obs.track_id} " if obs.track_id != -1 else ""
            stat = "[!] PATH BLOCKED" if obs.in_corridor else f"Zone: {obs.horizontal_zone}"
            l1 = f"{id_prefix}{obs.class_name.upper()} {obs.confidence:.2f}"
            l2 = f"{stat} | {obs.mask_pixel_area}px"

            (tw1, th1), _ = cv2.getTextSize(l1, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)
            (tw2, th2), _ = cv2.getTextSize(l2, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
            bw = max(tw1, tw2) + 10
            bh = th1 + th2 + 10

            by1 = max(0, y1 - bh - 4)
            by2 = by1 + bh
            cv2.rectangle(canvas, (x1, by1), (x1 + bw, by2), (20, 20, 20), -1)
            cv2.rectangle(canvas, (x1, by1), (x1 + bw, by2), box_color, 1)
            cv2.putText(canvas, l1, (x1 + 4, by1 + th1 + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.44, box_color, 1, cv2.LINE_AA)
            cv2.putText(canvas, l2, (x1 + 4, by1 + th1 + th2 + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)

        # 4. Top Telemetry Header
        header_h = 42
        header = canvas.copy()
        cv2.rectangle(header, (0, 0), (w, header_h), (25, 25, 25), -1)
        cv2.addWeighted(header, 0.85, canvas, 0.15, 0, canvas)

        cpu_p = self.cpu_usages[-1] if self.cpu_usages else 0.0
        mem_mb = self.memory_usages[-1] if self.memory_usages else 0.0
        pipe_lat = self.pipeline_latencies[-1] if self.pipeline_latencies else 0.0

        top_stats = (
            f"VIRYA PERCEPTION  |  YOLO Infer: {infer_latency_ms:.1f}ms  |  "
            f"Total Pipe: {pipe_lat:.1f}ms  |  "
            f"FPS: {self.fps_rolling:.1f}  |  "
            f"CPU: {cpu_p:.0f}%  RAM: {mem_mb:.0f}MB  |  "
            f"Tracks: {len(obstacles)}"
        )
        cv2.putText(canvas, top_stats, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

        # 5. Bottom Decision Banner
        banner_h = 68
        banner_y1 = h - banner_h
        banner = canvas.copy()
        cv2.rectangle(banner, (0, banner_y1), (w, h), (15, 15, 15), -1)
        cv2.addWeighted(banner, 0.88, canvas, 0.12, 0, canvas)

        decision_colors = {
            NavigationDecision.PATH_CLEAR: (0, 255, 0),
            NavigationDecision.SLOW_DOWN: (0, 215, 255),
            NavigationDecision.MOVE_LEFT: (255, 144, 30),
            NavigationDecision.MOVE_RIGHT: (255, 144, 30),
            NavigationDecision.STOP: (0, 0, 255),
            NavigationDecision.UNKNOWN: (160, 160, 160),
        }
        dec_color = decision_colors.get(decision.decision, (255, 255, 255))

        dec_str = f"DECISION: {decision.decision.value}"
        (dw, dh), _ = cv2.getTextSize(dec_str, cv2.FONT_HERSHEY_SIMPLEX, 0.70, 2)
        cv2.rectangle(canvas, (14, banner_y1 + 10), (24 + dw, banner_y1 + 10 + dh + 10), dec_color, -1)
        cv2.putText(canvas, dec_str, (18, banner_y1 + 10 + dh + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 0, 0), 2, cv2.LINE_AA)

        line_reason = f"Reason: {decision.reason}"
        line_ctrl = f"Proposed Control: Steer {decision.suggested_steer_dir} | Target Speed: {decision.suggested_speed_factor*100:.0f}%"
        cv2.putText(canvas, line_reason, (35 + dw, banner_y1 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(canvas, line_ctrl, (35 + dw, banner_y1 + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.44, dec_color, 1, cv2.LINE_AA)

        # 6. Directional Visual Arrow
        arrow_x = w // 2
        arrow_y = 75
        if decision.decision == NavigationDecision.MOVE_LEFT:
            cv2.arrowedLine(canvas, (arrow_x + 40, arrow_y), (arrow_x - 40, arrow_y), (255, 144, 30), 4, tipLength=0.35)
            cv2.putText(canvas, "STEER LEFT", (arrow_x - 50, arrow_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 144, 30), 2)
        elif decision.decision == NavigationDecision.MOVE_RIGHT:
            cv2.arrowedLine(canvas, (arrow_x - 40, arrow_y), (arrow_x + 40, arrow_y), (255, 144, 30), 4, tipLength=0.35)
            cv2.putText(canvas, "STEER RIGHT", (arrow_x - 55, arrow_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 144, 30), 2)
        elif decision.decision == NavigationDecision.STOP:
            cv2.circle(canvas, (arrow_x, arrow_y), 20, (0, 0, 255), -1)
            cv2.putText(canvas, "STOP", (arrow_x - 22, arrow_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)

        return canvas

    def _cleanup(self):
        print("\n🧹 Releasing perception pipeline resources...")
        if self.cap is not None:
            self.cap.release()
            print("✅ Input stream closed.")
        if self.video_writer is not None:
            self.video_writer.release()
            print("✅ Recorded HUD video saved.")
        if self.log_file is not None and not self.log_file.closed:
            self.log_file.close()
            print("✅ Structured JSONL telemetry finalized.")
        if not self.headless:
            cv2.destroyAllWindows()

    def _print_benchmark_report(self, start_wall_time: float):
        total_time = time.time() - start_wall_time
        avg_fps = self.frame_count / total_time if total_time > 0 else 0.0

        print("\n" + "=" * 75)
        print("          INTEGRATED PERCEPTION PIPELINE BENCHMARK REPORT")
        print("=" * 75)
        print(f"Total Frames Processed     : {self.frame_count}")
        print(f"Total Session Duration     : {total_time:.2f} s")
        print(f"Overall Throughput (FPS)   : {avg_fps:.2f} FPS")
        print(f"Unique Track IDs Created   : {len(self.total_unique_track_ids)}")

        if self.infer_latencies:
            print("\n[ YOLO26n-seg Inference Latency ]")
            print(f"  - Mean Latency           : {np.mean(self.infer_latencies):.2f} ms")
            print(f"  - Median Latency (P50)   : {np.median(self.infer_latencies):.2f} ms")
            print(f"  - P95 Latency            : {np.percentile(self.infer_latencies, 95):.2f} ms")
            print(f"  - Min / Max Latency      : {np.min(self.infer_latencies):.2f} ms / {np.max(self.infer_latencies):.2f} ms")

        if self.pipeline_latencies:
            print("\n[ Full Pipeline Latency (Capture -> Inference -> Decision -> HUD) ]")
            print(f"  - Mean Pipeline Latency  : {np.mean(self.pipeline_latencies):.2f} ms")
            print(f"  - Median Pipeline (P50)  : {np.median(self.pipeline_latencies):.2f} ms")
            print(f"  - P95 Pipeline Latency   : {np.percentile(self.pipeline_latencies, 95):.2f} ms")

        if self.cpu_usages:
            print("\n[ Hardware Resource Utilization ]")
            print(f"  - Mean CPU Utilization   : {np.mean(self.cpu_usages):.1f}%")
            print(f"  - Peak CPU Utilization   : {np.max(self.cpu_usages):.1f}%")
            print(f"  - Memory RSS (Final)     : {self.memory_usages[-1]:.1f} MB")

        print("=" * 75)


def main():
    parser = argparse.ArgumentParser(description="VIRYA DBW Buggy - Integrated Perception Pipeline")
    parser.add_argument("--model", type=str, default="/home/vat/yolo_vehicle_test/yolo26n-seg.pt")
    parser.add_argument("--source", type=str, default="0", help="Camera index ('0') or path to video file")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-video", type=str, default=None)
    parser.add_argument("--log", type=str, default="/home/vat/yolo_vehicle_test/logs/pipeline_telemetry.jsonl")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--corridor-left", type=float, default=0.25)
    parser.add_argument("--corridor-right", type=float, default=0.75)
    parser.add_argument("--config", type=str, default="/home/vat/yolo_vehicle_test/config/object_classes.yaml")

    args = parser.parse_args()

    if not os.environ.get("DISPLAY"):
        if not args.headless:
            args.headless = True

    pipeline = IntegratedPerceptionPipeline(
        model_path=args.model,
        source=args.source,
        conf_threshold=args.conf,
        classes_config=args.config,
        corridor_bounds=(args.corridor_left, args.corridor_right),
        headless=args.headless,
        save_video=args.save_video,
        log_jsonl=args.log,
        max_frames=args.max_frames,
    )
    pipeline.run()


if __name__ == "__main__":
    main()
