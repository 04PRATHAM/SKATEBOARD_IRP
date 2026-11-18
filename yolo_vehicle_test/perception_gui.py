#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Live Multi-Object Perception & Decision GUI
Module: perception_gui.py

Comprehensive Live Perception Dashboard for VIRYA Autonomous Buggy:
  - Live Webcam / Video Feed (YOLO26n-seg + ByteTrack)
  - Color-coded Instance Segmentation Masks & Bounding Boxes
  - Dynamic Vehicle Swept Corridor & Zone Markings (Left, Center, Right)
  - Real-time Detected Objects Panel (Table with IDs, Confidences, Zones, Status)
  - Prominent Decision Banner (STOP, MOVE LEFT, MOVE RIGHT, SLOW DOWN, PATH CLEAR, UNKNOWN)
  - Visual Steering Representation (Directional Arrows, Target Angles, Vehicle Icon)
  - 2D Top-Down Spatial Obstacle Corridor Map
  - Braking Status Panel (Brake Requested / Speed / Offline Safety Notice)
  - System Performance Panel (FPS, Latencies, CPU%, RAM MB, Tracks)
  - Timestamped Scrolling Event & Decision Log
  - Multi-threaded Architecture (Perception Worker Thread + Decoupled UI Loop)
  - Headless / Non-Display Fallback & Dashboard Exporter

CRITICAL VEHICLE SAFETY RESTRICTION:
Operates strictly in OFFLINE / TEST MODE.
Actuation is disconnected; no commands are sent to CAN, Modbus, or ROS 2 actuation topics.

Permitted Workspace: /home/vat/yolo_vehicle_test/
"""

import os
import sys
import time
import queue
import threading
import argparse
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass

import numpy as np
import cv2
from PIL import Image, ImageTk, ImageDraw, ImageFont
import psutil

# Ensure Ultralytics config is contained within permitted workspace
os.environ["YOLO_CONFIG_DIR"] = "/home/vat/yolo_vehicle_test/.config/Ultralytics"

try:
    from ultralytics import YOLO
except ImportError:
    print("❌ ERROR: Ultralytics library not found.")
    sys.exit(1)

from run_perception_pipeline import (
    IntegratedPerceptionPipeline,
    NavigationDecision,
    TrackedObstacle,
    DecisionResult,
    PipelineTelemetry,
)
from vehicle_control_adapter import (
    VehicleControlAdapter,
    ControlMode,
    AdapterTelemetry,
    SubsystemStatus,
    FSMState,
)


@dataclass
class GUIFramePacket:
    """Thread-safe frame & telemetry packet passed from worker to GUI."""
    raw_frame: np.ndarray
    annotated_frame: np.ndarray
    obstacles: List[TrackedObstacle]
    decision: DecisionResult
    infer_latency_ms: float
    pipeline_latency_ms: float
    fps: float
    cpu_percent: float
    memory_rss_mb: float
    timestamp: float
    events: List[str]
    control_telemetry: Optional[AdapterTelemetry] = None


class PerceptionWorker(threading.Thread):
    """
    Background worker thread running camera capture, YOLO26n-seg inference,
    ByteTrack tracking, spatial corridor calculation, and decision engine.
    Completely decoupled from GUI thread to prevent UI freezing.
    """

    def __init__(
        self,
        source: str = "0",
        model_path: str = "/home/vat/yolo_vehicle_test/yolo26n-seg.pt",
        conf_threshold: float = 0.25,
        packet_queue: Optional[queue.Queue] = None,
        max_fps: float = 30.0,
        mode: ControlMode = ControlMode.MODE_OFFLINE_MOCK,
        passkey: Optional[str] = None,
    ):
        super().__init__(daemon=True)
        self.source = source
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.packet_queue = packet_queue or queue.Queue(maxsize=3)
        self.max_fps = max_fps
        self.mode = mode
        self.passkey = passkey

        self.running = False
        self.paused = False
        self.cap = None
        self.pipeline = None
        self.process = psutil.Process(os.getpid())

        self.last_decision = None
        self.last_track_ids = set()
        self.event_history = []
        self._lock = threading.Lock()

        # Initialize Vehicle Control Adapter in requested mode
        self.adapter = VehicleControlAdapter(mode=self.mode, passkey=self.passkey)

    def run(self):
        self.running = True
        self._emit_event("Initializing YOLO26n-seg perception pipeline...")
        self.adapter.start()

        # Initialize pipeline logic
        self.pipeline = IntegratedPerceptionPipeline(
            model_path=self.model_path,
            source=self.source,
            conf_threshold=self.conf_threshold,
            headless=True,
            max_frames=None,
        )

        # Use the pipeline's initialized capture device (avoids duplicate V4L2 camera open)
        self.cap = self.pipeline.cap
        if self.cap is None or not self.cap.isOpened():
            self._emit_event(f"❌ ERROR: Failed to open camera/video source: {self.source}")
            self.adapter.update_perception_decision(None, is_camera_healthy=False)
            fault_packet = GUIFramePacket(
                raw_frame=None,
                annotated_frame=None,
                obstacles=[],
                decision=DecisionResult(
                    decision=NavigationDecision.UNKNOWN,
                    reason=f"CAMERA ERROR: Source '{self.source}' failed to open",
                    primary_obstacle=None,
                    total_objects=0,
                    corridor_intruders=0,
                    left_clearance_score=0.0,
                    right_clearance_score=0.0,
                    suggested_speed_factor=0.0,
                    suggested_steer_dir="CENTER",
                ),
                infer_latency_ms=0.0,
                pipeline_latency_ms=0.0,
                fps=0.0,
                cpu_percent=psutil.cpu_percent(),
                memory_rss_mb=self.process.memory_info().rss / (1024 * 1024),
                timestamp=time.time(),
                events=[f"❌ CAMERA ERROR: /dev/video{self.source} unavailable"],
                control_telemetry=self.adapter.get_telemetry(),
            )
            try:
                self.packet_queue.put_nowait(fault_packet)
            except queue.Full:
                pass
            self.adapter.stop()
            self.running = False
            return

        self._emit_event(f"✅ Video source connected: {self.source}")
        self._emit_event("🚀 Perception worker active (Offline Safety Mode)")

        rolling_fps = 0.0
        frame_idx = 0

        while self.running:
            if self.paused:
                time.sleep(0.05)
                continue

            t_start = time.perf_counter()

            ret, frame = self.cap.read()
            if not ret or frame is None:
                if str(self.source) != "0" and not self.source.isdigit():
                    # Loop recorded video for continuous display
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                    if not ret:
                        self._emit_event("Video playback ended.")
                        break
                else:
                    self._emit_event("⚠️ Camera frame drop or disconnection detected.")
                    self.adapter.update_perception_decision(None, is_camera_healthy=False)
                    fault_packet = GUIFramePacket(
                        raw_frame=None,
                        annotated_frame=None,
                        obstacles=[],
                        decision=DecisionResult(
                            decision=NavigationDecision.UNKNOWN,
                            reason="CAMERA ERROR: /dev/video0 frame read failure",
                            primary_obstacle=None,
                            total_objects=0,
                            corridor_intruders=0,
                            left_clearance_score=0.0,
                            right_clearance_score=0.0,
                            suggested_speed_factor=0.0,
                            suggested_steer_dir="CENTER",
                        ),
                        infer_latency_ms=0.0,
                        pipeline_latency_ms=0.0,
                        fps=0.0,
                        cpu_percent=psutil.cpu_percent(),
                        memory_rss_mb=self.process.memory_info().rss / (1024 * 1024),
                        timestamp=time.time(),
                        events=["[WARN] CAMERA ERROR: Live frame read failed"],
                        control_telemetry=self.adapter.get_telemetry(),
                    )
                    try:
                        self.packet_queue.put_nowait(fault_packet)
                    except queue.Full:
                        pass
                    time.sleep(0.05)
                    continue

            frame_idx += 1

            # 1. Single-Pass Inference + ByteTrack
            t_infer_start = time.perf_counter()
            results = self.pipeline.model.track(
                source=frame,
                persist=True,
                tracker="bytetrack.yaml",
                conf=self.conf_threshold,
                device="cpu",
                verbose=False,
                classes=list(self.pipeline.target_class_ids),
            )
            t_infer_end = time.perf_counter()
            infer_ms = (t_infer_end - t_infer_start) * 1000.0

            # 2. Spatial Corridor Positioning
            obstacles = self.pipeline._process_spatial_obstacles(frame, results[0])

            # 3. Decision Engine Reasoning
            decision = self.pipeline._evaluate_navigation_safety(obstacles, frame.shape)

            # Update Vehicle Control Adapter setpoints
            self.adapter.update_perception_decision(decision, is_camera_healthy=True)

            # 4. Generate Telemetry & Events
            new_events = []
            curr_track_ids = {o.track_id for o in obstacles}
            for o in obstacles:
                if o.track_id not in self.last_track_ids:
                    msg = f"New {o.class_name.upper()} detected (ID: {o.track_id}, conf: {o.confidence:.2f}, zone: {o.horizontal_zone})"
                    new_events.append(msg)
                    self._emit_event(msg)
                if o.in_corridor and (self.last_decision != decision.decision):
                    msg = f"[WARN] Corridor intrusion: {o.class_name.upper()} (ID: {o.track_id}) -> {decision.decision.value}"
                    new_events.append(msg)
                    self._emit_event(msg)

            if decision.decision != self.last_decision:
                msg = f"Decision update: {decision.decision.value} ({decision.reason})"
                new_events.append(msg)
                self._emit_event(msg)
                self.last_decision = decision.decision

            self.last_track_ids = curr_track_ids

            # 5. Render Annotated Camera Frame
            annotated_frame = self._render_camera_canvas(frame, results[0], obstacles, decision)

            # 6. Performance Metrics
            t_end = time.perf_counter()
            pipe_ms = (t_end - t_start) * 1000.0
            instant_fps = 1000.0 / pipe_ms if pipe_ms > 0 else 0.0
            rolling_fps = 0.9 * rolling_fps + 0.1 * instant_fps if frame_idx > 1 else instant_fps

            cpu_p = psutil.cpu_percent()
            mem_rss = self.process.memory_info().rss / (1024 * 1024)

            # 7. Package and Push to Queue
            packet = GUIFramePacket(
                raw_frame=frame,
                annotated_frame=annotated_frame,
                obstacles=obstacles,
                decision=decision,
                infer_latency_ms=infer_ms,
                pipeline_latency_ms=pipe_ms,
                fps=rolling_fps,
                cpu_percent=cpu_p,
                memory_rss_mb=mem_rss,
                timestamp=time.time(),
                events=new_events,
                control_telemetry=self.adapter.get_telemetry(),
            )

            try:
                # Maintain freshest frame in queue
                if self.packet_queue.full():
                    try:
                        self.packet_queue.get_nowait()
                    except queue.Empty:
                        pass
                self.packet_queue.put_nowait(packet)
            except queue.Full:
                pass

        if self.cap:
            self.cap.release()
        self.adapter.stop()
        self._emit_event("🛑 Perception worker stopped. Camera released.")

    def stop(self):
        self.running = False
        self.adapter.stop()

    def toggle_pause(self):
        self.paused = not self.paused
        state_str = "PAUSED" if self.paused else "RESUMED"
        self._emit_event(f"Perception {state_str}")
        return self.paused

    def _emit_event(self, text: str):
        t_str = time.strftime("%H:%M:%S", time.localtime())
        entry = f"[{t_str}] {text}"
        with self._lock:
            self.event_history.append(entry)
            if len(self.event_history) > 100:
                self.event_history.pop(0)

    def _render_camera_canvas(
        self,
        frame: np.ndarray,
        result,
        obstacles: List[TrackedObstacle],
        decision: DecisionResult,
    ) -> np.ndarray:
        """High-clarity overlay rendering for the GUI camera widget."""
        canvas = frame.copy()
        h, w = canvas.shape[:2]
        c_x1 = int(self.pipeline.corridor_left * w)
        c_x2 = int(self.pipeline.corridor_right * w)

        # 1. Driving Corridor Swept Path Underlay
        corridor_overlay = canvas.copy()
        cv2.rectangle(corridor_overlay, (c_x1, 0), (c_x2, h), (180, 50, 180), -1)
        cv2.addWeighted(corridor_overlay, 0.12, canvas, 0.88, 0, canvas)
        cv2.line(canvas, (c_x1, 0), (c_x1, h), (255, 0, 255), 2, cv2.LINE_AA)
        cv2.line(canvas, (c_x2, 0), (c_x2, h), (255, 0, 255), 2, cv2.LINE_AA)

        # Zone Division Guidelines (Left: 0-25%, Center: 25-75%, Right: 75-100%)
        cv2.putText(canvas, "ZONE: LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(canvas, "DRIVING CORRIDOR (CENTER)", (c_x1 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "ZONE: RIGHT", (c_x2 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)

        # 2. Instance Segmentation Masks
        if result.masks is not None and len(result.masks) > 0:
            mask_overlay = np.zeros_like(canvas, dtype=np.uint8)
            masks_data = result.masks.data.cpu().numpy()
            for i, mask in enumerate(masks_data):
                cid = int(result.boxes.cls[i])
                if cid not in self.pipeline.target_class_ids:
                    continue
                if mask.shape[:2] != (h, w):
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                color = self.pipeline.class_metadata.get(cid, {}).get("color", (0, 255, 0))
                mask_overlay[mask > 0.5] = color
            cv2.addWeighted(mask_overlay, 0.35, canvas, 1.0, 0, canvas)

        # 3. Bounding Boxes, Centroids, and Badges
        for obs in obstacles:
            x1, y1, x2, y2 = obs.bbox
            color = self.pipeline.class_metadata.get(obs.class_id, {}).get("color", (0, 255, 0))
            box_color = (0, 0, 255) if obs.in_corridor else color
            thickness = 3 if obs.in_corridor else 2

            cv2.rectangle(canvas, (x1, y1), (x2, y2), box_color, thickness)
            cx, cy = obs.center_pixel
            cv2.drawMarker(canvas, (cx, cy), box_color, cv2.MARKER_CROSS, 10, 2)

            # Motion History Polyline
            if obs.track_id != -1 and len(self.pipeline.tracked_history[obs.track_id]) > 1:
                pts = np.array(self.pipeline.tracked_history[obs.track_id], dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(canvas, [pts], isClosed=False, color=box_color, thickness=2)

            # Object Badge
            id_prefix = f"ID:{obs.track_id} " if obs.track_id != -1 else ""
            stat = "[!] IN PATH" if obs.in_corridor else f"Zone: {obs.horizontal_zone}"
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

        return canvas


class DashboardCompositor:
    """
    Renders the complete GUI dashboard layout into a single OpenCV image.
    Used for both live GUI display, remote video streaming, and headless exports.
    Guarantees consistent 100% feature representation regardless of X11/Tkinter presence.
    """

    def __init__(self, width: int = 1280, height: int = 800):
        self.w = width
        self.h = height

    def render(self, packet: GUIFramePacket, event_log: List[str]) -> np.ndarray:
        dash = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        dash[:] = (30, 34, 42)  # Dark slate background (#1e222a)

        # 1. TOP HEADER (Height: 55px)
        cv2.rectangle(dash, (0, 0), (self.w, 55), (20, 23, 29), -1)
        cv2.putText(dash, "VIRYA DBW BUGGY - MULTI-OBJECT PERCEPTION & DECISION DASHBOARD", (20, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

        # Offline Safety Banner
        safety_text = "OFFLINE SAFETY MODE: ACTUATION DISCONNECTED (NO PHYSICAL COMMANDS)"
        (stw, _), _ = cv2.getTextSize(safety_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(dash, (self.w - stw - 35, 12), (self.w - 15, 42), (0, 0, 180), -1)
        cv2.putText(dash, safety_text, (self.w - stw - 25, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # 2. CENTRAL LIVE CAMERA FEED (Position: [20, 70] to [720, 520])
        cam_x, cam_y, cam_w, cam_h = 20, 70, 680, 470
        cv2.rectangle(dash, (cam_x - 2, cam_y - 2), (cam_x + cam_w + 2, cam_y + cam_h + 2), (60, 68, 80), 2)
        if packet.annotated_frame is not None:
            resized_cam = cv2.resize(packet.annotated_frame, (cam_w, cam_h))
            dash[cam_y:cam_y + cam_h, cam_x:cam_x + cam_w] = resized_cam
        else:
            cv2.rectangle(dash, (cam_x, cam_y), (cam_x + cam_w, cam_y + cam_h), (25, 25, 35), -1)
            cv2.putText(dash, "CAMERA FAULT: LIVE WEBCAM UNAVAILABLE", (cam_x + 90, cam_y + 220),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(dash, "Please verify /dev/video0 connection & permissions", (cam_x + 95, cam_y + 260),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        # 3. RIGHT PANEL - DECISION, STEERING & BRAKING (x: 720, width: 540)
        rx = 720
        rw = 540

        # 3A. CURRENT DECISION BANNER (y: 70, height: 95)
        dec_h = 95
        cv2.rectangle(dash, (rx, 70), (rx + rw, 70 + dec_h), (20, 23, 29), -1)
        cv2.rectangle(dash, (rx, 70), (rx + rw, 70 + dec_h), (60, 68, 80), 1)

        dec_colors = {
            NavigationDecision.STOP: (0, 0, 220),
            NavigationDecision.MOVE_LEFT: (255, 144, 30),
            NavigationDecision.MOVE_RIGHT: (255, 144, 30),
            NavigationDecision.SLOW_DOWN: (0, 215, 255),
            NavigationDecision.PATH_CLEAR: (0, 200, 0),
            NavigationDecision.UNKNOWN: (120, 120, 120),
        }
        dec_color = dec_colors.get(packet.decision.decision, (120, 120, 120))

        cv2.rectangle(dash, (rx + 10, 80), (rx + 220, 120), dec_color, -1)
        dec_str = f"DECISION: {packet.decision.decision.value}"
        cv2.putText(dash, dec_str, (rx + 16, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 2, cv2.LINE_AA)

        cv2.putText(dash, f"Reason: {packet.decision.reason[:48]}", (rx + 10, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (220, 220, 220), 1, cv2.LINE_AA)

        # 3B. STEERING REPRESENTATION & 2D CORRIDOR OBSTACLE MAP (y: 175, height: 175)
        steer_y = 175
        steer_h = 175
        cv2.rectangle(dash, (rx, steer_y), (rx + rw, steer_y + steer_h), (25, 29, 36), -1)
        cv2.rectangle(dash, (rx, steer_y), (rx + rw, steer_y + steer_h), (60, 68, 80), 1)
        cv2.putText(dash, "STEERING & 2D CORRIDOR MAP", (rx + 12, steer_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)

        # Left portion: Vehicle Steering Icon & Angle
        vcx, vcy = rx + 105, steer_y + 90
        cv2.rectangle(dash, (vcx - 20, vcy - 35), (vcx + 20, vcy + 35), (80, 90, 110), -1)
        cv2.rectangle(dash, (vcx - 20, vcy - 35), (vcx + 20, vcy + 35), (140, 150, 170), 2)
        # Front/rear wheels
        cv2.rectangle(dash, (vcx - 26, vcy - 30), (vcx - 20, vcy - 12), (30, 30, 30), -1)
        cv2.rectangle(dash, (vcx + 20, vcy - 30), (vcx + 26, vcy - 12), (30, 30, 30), -1)
        cv2.rectangle(dash, (vcx - 26, vcy + 12), (vcx - 20, vcy + 30), (30, 30, 30), -1)
        cv2.rectangle(dash, (vcx + 20, vcy + 12), (vcx + 26, vcy + 30), (30, 30, 30), -1)

        # Directional Steer Arrows & Text
        ctel = packet.control_telemetry
        slew_str = f" | Slew: {ctel.steer.commanded_value:+.1f} deg" if ctel else ""
        if packet.decision.decision == NavigationDecision.MOVE_LEFT:
            cv2.arrowedLine(dash, (vcx + 25, vcy), (vcx - 60, vcy), (255, 144, 30), 4, tipLength=0.35)
            cv2.putText(dash, f"STEER LEFT (+10.0 deg{slew_str})", (rx + 10, steer_y + 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 144, 30), 1, cv2.LINE_AA)
        elif packet.decision.decision == NavigationDecision.MOVE_RIGHT:
            cv2.arrowedLine(dash, (vcx - 25, vcy), (vcx + 60, vcy), (255, 144, 30), 4, tipLength=0.35)
            cv2.putText(dash, f"STEER RIGHT (-10.0 deg{slew_str})", (rx + 10, steer_y + 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 144, 30), 1, cv2.LINE_AA)
        elif packet.decision.decision == NavigationDecision.STOP:
            cv2.circle(dash, (vcx, vcy), 18, (0, 0, 220), -1)
            cv2.putText(dash, "STOP", (vcx - 16, vcy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 2)
            cv2.putText(dash, f"LOCKED (0.0 deg{slew_str})", (rx + 25, steer_y + 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 220), 1, cv2.LINE_AA)
        else:
            cv2.arrowedLine(dash, (vcx, vcy + 15), (vcx, vcy - 50), (0, 200, 0), 4, tipLength=0.35)
            cv2.putText(dash, f"STRAIGHT (0.0 deg{slew_str})", (rx + 20, steer_y + 155),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 200, 0), 1, cv2.LINE_AA)

        # Right portion: 2D Image-Space Top-Down Obstacle & Corridor Map (Section 8.6)
        mx1, my1, mw, mh = rx + 225, steer_y + 15, 300, 145
        cv2.rectangle(dash, (mx1, my1), (mx1 + mw, my1 + mh), (15, 18, 24), -1)
        cv2.rectangle(dash, (mx1, my1), (mx1 + mw, my1 + mh), (50, 56, 70), 1)

        # Draw Corridor Bounds
        cx1_map = int(mx1 + 0.25 * mw)
        cx2_map = int(mx1 + 0.75 * mw)
        # Corridor shading if intruded
        has_intruder = any(o.in_corridor for o in packet.obstacles)
        corridor_bg_col = (40, 20, 60) if not has_intruder else (20, 20, 70)
        cv2.rectangle(dash, (cx1_map, my1), (cx2_map, my1 + mh), corridor_bg_col, -1)
        cv2.line(dash, (cx1_map, my1), (cx1_map, my1 + mh), (255, 0, 255), 1, cv2.LINE_AA)
        cv2.line(dash, (cx2_map, my1), (cx2_map, my1 + mh), (255, 0, 255), 1, cv2.LINE_AA)

        # Vehicle Footprint at bottom of map
        cv2.rectangle(dash, (mx1 + mw // 2 - 14, my1 + mh - 22), (mx1 + mw // 2 + 14, my1 + mh - 4), (100, 110, 130), -1)
        cv2.putText(dash, "BUGGY", (mx1 + mw // 2 - 12, my1 + mh - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 255), 1)

        # Plot Obstacles on 2D map
        for obs in packet.obstacles:
            ox = int(mx1 + obs.center_norm[0] * mw)
            # Invert y so top of image (far) is at top of map
            oy = int(my1 + (1.0 - obs.center_norm[1]) * (mh - 30) + 10)
            ox = max(mx1 + 6, min(mx1 + mw - 6, ox))
            oy = max(my1 + 6, min(my1 + mh - 26, oy))

            obs_color = (0, 0, 255) if obs.in_corridor else (0, 200, 255)
            cv2.circle(dash, (ox, oy), 5, obs_color, -1)
            cv2.putText(dash, f"{obs.class_name[:3].upper()}:{obs.track_id}", (ox + 6, oy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, obs_color, 1, cv2.LINE_AA)

        # Avoidance Trajectory line
        if packet.decision.decision == NavigationDecision.MOVE_LEFT:
            cv2.arrowedLine(dash, (int(mx1 + mw // 2), int(my1 + mh - 22)), (int(mx1 + 0.12 * mw), int(my1 + 25)), (255, 144, 30), 2, tipLength=0.25)
        elif packet.decision.decision == NavigationDecision.MOVE_RIGHT:
            cv2.arrowedLine(dash, (int(mx1 + mw // 2), int(my1 + mh - 22)), (int(mx1 + 0.88 * mw), int(my1 + 25)), (255, 144, 30), 2, tipLength=0.25)
        elif packet.decision.decision == NavigationDecision.PATH_CLEAR:
            cv2.arrowedLine(dash, (int(mx1 + mw // 2), int(my1 + mh - 22)), (int(mx1 + mw // 2), int(my1 + 15)), (0, 200, 0), 2, tipLength=0.25)

        cv2.putText(dash, "2D CORRIDOR MAP (Conceptual)", (mx1 + 6, my1 + mh - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, (140, 140, 140), 1, cv2.LINE_AA)

        # 3C. VEHICLE CONTROL & DBW FEEDBACK INTERFACE (y: 360, height: 180)
        brake_y = 360
        brake_h = 180
        cv2.rectangle(dash, (rx, brake_y), (rx + rw, brake_y + brake_h), (25, 29, 36), -1)
        cv2.rectangle(dash, (rx, brake_y), (rx + rw, brake_y + brake_h), (60, 68, 80), 1)

        # Header with periodic rate
        cv2.putText(dash, "DBW VEHICLE CONTROL & FEEDBACK INTERFACE (20 Hz PERIODIC)", (rx + 12, brake_y + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1, cv2.LINE_AA)

        if ctel:
            fb_tag = "(REAL)" if ctel.has_real_feedback else "(SIM)"
            # Row 1: Steering Subsystem Status
            s_req = ctel.steer.target_value
            s_cmd = ctel.steer.commanded_value
            s_fb = ctel.steer.feedback_value
            s_stat = ctel.steer.status.value
            s_color = (0, 200, 0) if s_stat in ("ACCEPTED", "EXECUTED") else (0, 215, 255)
            cv2.putText(dash, f"STEER:  Req: {s_req:+5.1f} deg | Cmd: {s_cmd:+5.1f} deg | FB {fb_tag}: {s_fb:+5.1f} deg [{s_stat}]",
                        (rx + 12, brake_y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.36, s_color, 1, cv2.LINE_AA)

            # Row 2: Velocity Subsystem Status
            v_req = ctel.velocity.target_value
            v_cmd = ctel.velocity.commanded_value
            v_fb = ctel.velocity.feedback_value
            v_stat = ctel.velocity.status.value
            v_color = (0, 200, 0) if v_stat in ("ACCEPTED", "EXECUTED") else (0, 215, 255)
            cv2.putText(dash, f"SPEED:  Req: {v_req:4.1f} m/s | Cmd: {v_cmd:4.1f} m/s | FB {fb_tag}: {v_fb:4.1f} m/s [{v_stat}]",
                        (rx + 12, brake_y + 72), cv2.FONT_HERSHEY_SIMPLEX, 0.36, v_color, 1, cv2.LINE_AA)

            # Row 3: Braking Subsystem Status
            b_req_str = "APPLIED" if ctel.brake.target_value > 0.5 else "RELEASED"
            b_cmd_str = "APPLIED" if ctel.brake.commanded_value > 0.5 else "RELEASED"
            b_fb_str = "APPLIED" if ctel.brake.feedback_value > 0.5 else "RELEASED"
            b_color = (0, 0, 220) if ctel.brake.commanded_value > 0.5 else (0, 200, 0)
            cv2.putText(dash, f"BRAKE:  Req: {b_req_str:8s} | Cmd: {b_cmd_str:8s} | FB {fb_tag}: {b_fb_str:8s} [{ctel.brake.status.value}]",
                        (rx + 12, brake_y + 96), cv2.FONT_HERSHEY_SIMPLEX, 0.36, b_color, 1, cv2.LINE_AA)

            # Row 4: DBW Supervisor FSM & Bus Telemetry
            fsm_str = f"FSM: {ctel.fsm_state.value}"
            fsm_col = (0, 200, 0) if ctel.fsm_state.value == "AUTOMODE" else ((0, 0, 220) if ctel.fsm_state.value == "FAULT" else (255, 144, 30))
            dm_str = f"DEADMAN: {ctel.deadman_rate_hz:.0f}Hz ACTIVE" if ctel.deadman_active else "DEADMAN: INACTIVE"
            cv2.putText(dash, f"{fsm_str}  |  {dm_str}", (rx + 12, brake_y + 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, fsm_col, 1, cv2.LINE_AA)

            # Row 5: Modbus & SocketCAN Encodings
            reg3_hex = f"0x{ctel.simulated_modbus_reg3:04X}"
            reg0_hex = f"0x{ctel.simulated_modbus_reg0:04X}"
            cv2.putText(dash, f"MODBUS TCP: Reg3={reg3_hex} (Cmd) | Reg0={reg0_hex} (FB)", (rx + 12, brake_y + 142),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)

            # Row 6: Safety Isolation Banner
            cv2.rectangle(dash, (rx + 10, brake_y + 152), (rx + rw - 10, brake_y + 175), (40, 45, 55), -1)
            if ctel.has_real_feedback:
                stat_banner = "STATUS: HARDWARE CONNECTED - LIVE DBW TOPICS ACTIVE"
                stat_col = (0, 200, 0)
            elif ctel.mode == ControlMode.MODE_MONITOR:
                stat_banner = "STATUS: PASSIVE MONITOR MODE - LISTENING TO DBW TOPICS"
                stat_col = (255, 144, 30)
            else:
                stat_banner = "STATUS: OFFLINE MOCK MODE - ZERO PHYSICAL ACTUATION TRANSMITTED"
                stat_col = (100, 200, 255)
            cv2.putText(dash, stat_banner, (rx + 16, brake_y + 169),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, stat_col, 1, cv2.LINE_AA)
        else:
            cv2.putText(dash, "Adapter Telemetry Initializing...", (rx + 20, brake_y + 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)

        # 4. BOTTOM LEFT - DETECTED OBJECTS TABLE (Position: [20, 555] to [620, 785])
        tbl_x, tbl_y, tbl_w, tbl_h = 20, 555, 600, 230
        cv2.rectangle(dash, (tbl_x, tbl_y), (tbl_x + tbl_w, tbl_y + tbl_h), (25, 29, 36), -1)
        cv2.rectangle(dash, (tbl_x, tbl_y), (tbl_x + tbl_w, tbl_y + tbl_h), (60, 68, 80), 1)

        cv2.putText(dash, f"DETECTED OBJECTS PANEL (Total: {len(packet.obstacles)} | Corridor Intruders: {sum(1 for o in packet.obstacles if o.in_corridor)})",
                    (tbl_x + 12, tbl_y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 200, 200), 1, cv2.LINE_AA)

        # Table Header
        cv2.rectangle(dash, (tbl_x + 8, tbl_y + 32), (tbl_x + tbl_w - 8, tbl_y + 54), (40, 46, 58), -1)
        headers = [("CLASS", 14), ("TRACK ID", 110), ("CONF", 195), ("ZONE", 265), ("CORRIDOR STATUS", 360), ("MASK AREA", 490)]
        for h_text, h_off in headers:
            cv2.putText(dash, h_text, (tbl_x + h_off, tbl_y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)

        # Table Rows
        row_y = tbl_y + 72
        if packet.obstacles:
            for obs in packet.obstacles[:6]:
                stat_str = "BLOCKED" if obs.in_corridor else "CLEAR"
                stat_col = (0, 0, 220) if obs.in_corridor else (0, 200, 0)

                cv2.putText(dash, obs.class_name.upper()[:10], (tbl_x + 14, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
                cv2.putText(dash, f"ID:{obs.track_id}", (tbl_x + 110, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
                cv2.putText(dash, f"{obs.confidence:.2f}", (tbl_x + 195, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
                cv2.putText(dash, obs.horizontal_zone, (tbl_x + 265, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
                cv2.putText(dash, stat_str, (tbl_x + 360, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, stat_col, 1)
                cv2.putText(dash, f"{obs.mask_pixel_area} px", (tbl_x + 490, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)
                row_y += 24
        else:
            cv2.putText(dash, "No active target obstacles detected in camera view", (tbl_x + 110, tbl_y + 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (140, 140, 140), 1, cv2.LINE_AA)

        # 5. BOTTOM RIGHT - EVENT LOG & SYSTEM METRICS (Position: [630, 555] to [1260, 785])
        log_x, log_y, log_w, log_h = 630, 555, 630, 230
        cv2.rectangle(dash, (log_x, log_y), (log_x + log_w, log_y + log_h), (25, 29, 36), -1)
        cv2.rectangle(dash, (log_x, log_y), (log_x + log_w, log_y + log_h), (60, 68, 80), 1)

        # Performance Bar
        perf_str = (
            f"SYSTEM METRICS | FPS: {packet.fps:.1f} | Infer: {packet.infer_latency_ms:.1f}ms | "
            f"Total Pipe: {packet.pipeline_latency_ms:.1f}ms | CPU: {packet.cpu_percent:.0f}% | "
            f"RAM: {packet.memory_rss_mb:.0f}MB"
        )
        cv2.rectangle(dash, (log_x + 6, log_y + 8), (log_x + log_w - 6, log_y + 32), (35, 40, 50), -1)
        cv2.putText(dash, perf_str, (log_x + 12, log_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 220, 255), 1, cv2.LINE_AA)

        cv2.putText(dash, "REAL-TIME PERCEPTION & DECISION EVENT LOG", (log_x + 12, log_y + 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)

        # Events entries
        ev_y = log_y + 70
        recent_events = event_log[-6:] if event_log else ["Awaiting perception events..."]
        for ev in recent_events:
            ev_col = (0, 0, 255) if "Corridor intrusion" in ev or "STOP" in ev else (210, 210, 210)
            cv2.putText(dash, ev[:78], (log_x + 14, ev_y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, ev_col, 1, cv2.LINE_AA)
            ev_y += 24

        return dash


class PerceptionGUIApp:
    """
    Main GUI Window application combining Tkinter interactive controls,
    live OpenCV video rendering, detected objects table, and dashboard compositing.
    """

    def __init__(
        self,
        source: str = "0",
        model_path: str = "/home/vat/yolo_vehicle_test/yolo26n-seg.pt",
        save_video: Optional[str] = None,
        max_test_frames: Optional[int] = None,
        headless: bool = False,
        mode: str = "mock",
        passkey: Optional[str] = None,
    ):
        self.source = source
        self.model_path = model_path
        self.save_video = save_video
        self.max_test_frames = max_test_frames
        self.headless = headless or not os.environ.get("DISPLAY")

        # Map string mode to ControlMode enum
        mode_map = {
            "mock": ControlMode.MODE_OFFLINE_MOCK,
            "monitor": ControlMode.MODE_MONITOR,
            "hardware": ControlMode.MODE_HARDWARE_BRIDGE,
        }
        self.control_mode = mode_map.get(mode.lower(), ControlMode.MODE_OFFLINE_MOCK)
        self.passkey = passkey

        self.packet_queue = queue.Queue(maxsize=3)
        self.worker = PerceptionWorker(
            source=self.source,
            model_path=self.model_path,
            packet_queue=self.packet_queue,
            mode=self.control_mode,
            passkey=self.passkey,
        )
        self.compositor = DashboardCompositor(width=1280, height=800)

        self.video_writer = None
        if self.save_video:
            os.makedirs(os.path.dirname(os.path.abspath(self.save_video)), exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(self.save_video, fourcc, 15.0, (1280, 800))
            print(f"🎬 Recording GUI dashboard to: {self.save_video}")

        self.root = None
        self.canvas_label = None
        self.frames_rendered = 0
        self.latest_packet = None

    def run(self):
        print("=" * 75)
        print("🖥️ LAUNCHING VIRYA DBW MULTI-OBJECT PERCEPTION & DECISION GUI")
        print(f"   Mode: {'HEADLESS / TEST-MODE' if self.headless else 'INTERACTIVE DESKTOP WINDOW'}")
        print(f"   Input Source: {self.source}")
        print("   Actuation: OFFLINE DISCONNECTED")
        print("=" * 75)

        self.worker.start()

        if not self.headless:
            self._build_tk_window()
            try:
                self.root.mainloop()
            except KeyboardInterrupt:
                print("\n🛑 Exiting GUI...")
            finally:
                self._cleanup()
        else:
            self._run_headless_loop()

    def _build_tk_window(self):
        import tkinter as tk
        from tkinter import ttk

        self.root = tk.Tk()
        self.root.title("VIRYA DBW Buggy - Live Multi-Object Perception & Decision GUI")
        self.root.geometry("1280x850")
        self.root.configure(bg="#1e222a")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Header / Controls Frame
        ctrl_frame = tk.Frame(self.root, bg="#14171d", height=45)
        ctrl_frame.pack(side=tk.TOP, fill=tk.X)

        title_lbl = tk.Label(
            ctrl_frame,
            text="VIRYA BUGGY PERCEPTION DASHBOARD",
            font=("Helvetica", 14, "bold"),
            fg="#ffffff",
            bg="#14171d",
            padx=15,
            pady=8,
        )
        title_lbl.pack(side=tk.LEFT)

        btn_exit = tk.Button(
            ctrl_frame,
            text="Exit Application",
            command=self._on_close,
            bg="#e06c75",
            fg="#ffffff",
            font=("Helvetica", 10, "bold"),
            padx=10,
        )
        btn_exit.pack(side=tk.RIGHT, padx=10, pady=6)

        self.btn_pause = tk.Button(
            ctrl_frame,
            text="Pause Display",
            command=self._on_pause_toggle,
            bg="#e5c07b",
            fg="#000000",
            font=("Helvetica", 10, "bold"),
            padx=10,
        )
        self.btn_pause.pack(side=tk.RIGHT, padx=6, pady=6)

        # Central Image Canvas
        self.canvas_label = tk.Label(self.root, bg="#1e222a")
        self.canvas_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Trigger periodic UI update loop (~30 FPS)
        self.root.after(30, self._tk_update_loop)

    def _on_pause_toggle(self):
        is_paused = self.worker.toggle_pause()
        self.btn_pause.config(text="Resume Display" if is_paused else "Pause Display")

    def _on_close(self):
        self._cleanup()
        if self.root:
            self.root.destroy()
        sys.exit(0)

    def _tk_update_loop(self):
        try:
            packet = self.packet_queue.get_nowait()
            self.latest_packet = packet
        except queue.Empty:
            packet = self.latest_packet

        if packet is not None:
            dash_img = self.compositor.render(packet, self.worker.event_history)
            self.frames_rendered += 1

            if self.video_writer:
                self.video_writer.write(dash_img)

            # Convert BGR to RGB for Tkinter
            rgb = cv2.cvtColor(dash_img, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)

            self.canvas_label.configure(image=img_tk)
            self.canvas_label.image = img_tk

        if self.max_test_frames and self.frames_rendered >= self.max_test_frames:
            print(f"🏁 Reached requested test frame limit ({self.max_test_frames}). Closing.")
            self._on_close()
            return

        if self.root:
            self.root.after(30, self._tk_update_loop)

    def _run_headless_loop(self):
        """Runs headless frame-pump for test verification and video export."""
        print("🚀 Running GUI Compositor in Headless / Automated Verification Mode...")
        start_time = time.time()

        try:
            while self.worker.is_alive():
                try:
                    packet = self.packet_queue.get(timeout=1.0)
                except queue.Empty:
                    continue

                dash_img = self.compositor.render(packet, self.worker.event_history)
                self.frames_rendered += 1

                if self.video_writer:
                    self.video_writer.write(dash_img)

                if self.frames_rendered % 10 == 0:
                    print(
                        f"GUI Frame {self.frames_rendered:04d} | "
                        f"FPS: {packet.fps:4.1f} | "
                        f"Dec: {packet.decision.decision.value:10s} | "
                        f"Objects: {len(packet.obstacles)} | "
                        f"Steer: {packet.decision.suggested_steer_dir}"
                    )

                if self.max_test_frames and self.frames_rendered >= self.max_test_frames:
                    print(f"🏁 Reached test frame limit ({self.max_test_frames}).")
                    break

        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user.")
        finally:
            self._cleanup()
            print(f"✅ Headless GUI execution complete. Rendered {self.frames_rendered} frames.")

    def _cleanup(self):
        print("🧹 Cleaning up perception GUI resources...")
        self.worker.stop()
        if self.worker.is_alive() and threading.current_thread() != self.worker:
            self.worker.join(timeout=2.0)
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            print("✅ Output dashboard video saved.")


def main():
    parser = argparse.ArgumentParser(description="VIRYA DBW Multi-Object Perception & Decision GUI")
    parser.add_argument("--source", type=str, default="0", help="Camera device index (e.g. 0) or path to video file")
    parser.add_argument("--model", type=str, default="/home/vat/yolo_vehicle_test/yolo26n-seg.pt", help="Path to YOLO26n-seg model")
    parser.add_argument("--save-video", type=str, default=None, help="Path to record GUI dashboard output video")
    parser.add_argument("--max-frames", type=int, default=None, help="Stop after processing N frames (useful for testing)")
    parser.add_argument("--headless", action="store_true", help="Force headless rendering mode without Tkinter display")
    parser.add_argument("--mode", type=str, choices=["mock", "monitor", "hardware"], default="mock",
                        help="Vehicle control adapter mode: mock (offline simulation), monitor (listen-only), hardware (ROS 2 DBW bridge)")
    parser.add_argument("--passkey", type=str, default=None, help="Safety passkey required for hardware mode ('ACTUATE_VIRYA_CONFIRM')")
    args = parser.parse_args()

    app = PerceptionGUIApp(
        source=args.source,
        model_path=args.model,
        save_video=args.save_video,
        max_test_frames=args.max_frames,
        headless=args.headless,
        mode=args.mode,
        passkey=args.passkey,
    )
    app.run()


if __name__ == "__main__":
    main()
