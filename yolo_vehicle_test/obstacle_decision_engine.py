#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Multi-Object Perception System
Module: obstacle_decision_engine.py

Phase 4: Multi-Object Obstacle Decision Engine (Offline Development)
Input: Image-based positioned obstacles and segmentation masks
Output: Proposed Navigation Decisions:
  - MOVE_LEFT
  - MOVE_RIGHT
  - SLOW_DOWN
  - STOP
  - PATH_CLEAR
  - UNKNOWN

CRITICAL VEHICLE SAFETY RESTRICTION:
This module is for OFFLINE DECISION ANALYSIS ONLY.
It does NOT publish steering, velocity, or brake commands to ROS 2 topics,
CAN bus, or Modbus. It generates, visualizes, and logs proposed autonomy decisions.

Author: Robotics Software Engineering Team
Permitted Workspace: /home/vat/yolo_vehicle_test/
"""

import os
import sys
import time
import json
import argparse
import signal
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple, Optional
import numpy as np
import cv2

# Import positioner components from Phase 3
from obstacle_positioning import ObstaclePositioner, PositionedObstacle


class NavigationDecision(str, Enum):
    MOVE_LEFT = "MOVE_LEFT"
    MOVE_RIGHT = "MOVE_RIGHT"
    SLOW_DOWN = "SLOW_DOWN"
    STOP = "STOP"
    PATH_CLEAR = "PATH_CLEAR"
    UNKNOWN = "UNKNOWN"


@dataclass
class DecisionResult:
    """Encapsulates the decision engine's evaluation for a single frame."""
    decision: NavigationDecision
    reason: str
    primary_obstacle: Optional[str]
    total_objects: int
    corridor_intruders: int
    left_clearance_score: float   # 0.0 (blocked) to 1.0 (completely open)
    right_clearance_score: float  # 0.0 (blocked) to 1.0 (completely open)
    suggested_speed_factor: float # Multiplier [0.0 = full brake, 1.0 = normal cruise]
    suggested_steer_dir: str      # "LEFT", "RIGHT", "CENTER"


class ObstacleDecisionEngine:
    """
    Offline Decision Engine that evaluates detected obstacles against the
    driving corridor and generates safe navigation proposals.
    """

    def __init__(
        self,
        model_path: str = "/home/vat/yolo_vehicle_test/yolo26n-seg.pt",
        camera_id: int = 0,
        conf_threshold: float = 0.25,
        corridor_bounds: Tuple[float, float] = (0.25, 0.75),
        headless: bool = False,
        save_video_path: Optional[str] = None,
        log_jsonl_path: str = "/home/vat/yolo_vehicle_test/logs/decision_log.jsonl",
        max_frames: Optional[int] = None,
    ):
        self.headless = headless
        self.save_video_path = save_video_path
        self.log_jsonl_path = log_jsonl_path
        self.max_frames = max_frames
        self.running = True

        # Initialize upstream positioning engine
        self.positioner = ObstaclePositioner(
            model_path=model_path,
            camera_id=camera_id,
            conf_threshold=conf_threshold,
            corridor_bounds=corridor_bounds,
            headless=True  # Positioner runs headlessly; we render the combined decision UI
        )

        self.cap = self.positioner.cap
        self.corridor_left = corridor_bounds[0]
        self.corridor_right = corridor_bounds[1]

        # Logging setup
        os.makedirs(os.path.dirname(os.path.abspath(self.log_jsonl_path)), exist_ok=True)
        self.log_file = open(self.log_jsonl_path, "a")

        self.video_writer = None
        if self.save_video_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.save_video_path)), exist_ok=True)
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(self.save_video_path, fourcc, 20.0, (actual_w, actual_h))

        self.frame_count = 0
        self.decision_history = []

        signal.signal(signal.SIGINT, self._sigint_handler)
        signal.signal(signal.SIGTERM, self._sigint_handler)

    def _sigint_handler(self, sig, frame):
        print("\n🛑 Stopping decision engine...")
        self.running = False

    def evaluate(self, obstacles: List[PositionedObstacle], frame_shape: Tuple[int, int]) -> DecisionResult:
        """
        Rule-based multi-object spatial reasoning engine.
        Prioritizes human life, evaluates corridor blockage, and checks bypass clearances.
        """
        h, w = frame_shape[:2]

        if not obstacles:
            return DecisionResult(
                decision=NavigationDecision.PATH_CLEAR,
                reason="No obstacles detected in scene",
                primary_obstacle=None,
                total_objects=0,
                corridor_intruders=0,
                left_clearance_score=1.0,
                right_clearance_score=1.0,
                suggested_speed_factor=1.0,
                suggested_steer_dir="CENTER",
            )

        # Categorize obstacles
        intruders = [o for o in obstacles if o.in_corridor]
        left_obstacles = [o for o in obstacles if o.horizontal_zone == "LEFT"]
        right_obstacles = [o for o in obstacles if o.horizontal_zone == "RIGHT"]

        # Calculate clearance scores for potential left/right avoidance
        left_area = sum(o.mask_pixel_area for o in left_obstacles)
        right_area = sum(o.mask_pixel_area for o in right_obstacles)
        frame_area = float(h * w)

        left_clearance = max(0.0, 1.0 - (left_area / (0.35 * frame_area)))
        right_clearance = max(0.0, 1.0 - (right_area / (0.35 * frame_area)))

        # RULE 1: Highest Priority - Pedestrian / Person Safety
        people_intruding = [o for o in intruders if o.class_name == "person"]
        if people_intruding:
            primary = people_intruding[0]
            return DecisionResult(
                decision=NavigationDecision.STOP,
                reason=f"CRITICAL SAFETY: Person in path (conf={primary.confidence:.2f})",
                primary_obstacle="person",
                total_objects=len(obstacles),
                corridor_intruders=len(intruders),
                left_clearance_score=left_clearance,
                right_clearance_score=right_clearance,
                suggested_speed_factor=0.0,
                suggested_steer_dir="CENTER",
            )

        # RULE 2: Multiple Intruding Obstacles or Corridor Saturation -> STOP
        if len(intruders) >= 3 or (len(intruders) >= 1 and left_clearance < 0.20 and right_clearance < 0.20):
            primary = intruders[0]
            return DecisionResult(
                decision=NavigationDecision.STOP,
                reason=f"Path completely blocked ({len(intruders)} intruders, both sides occluded)",
                primary_obstacle=f"{primary.class_name} ({primary.track_id})",
                total_objects=len(obstacles),
                corridor_intruders=len(intruders),
                left_clearance_score=left_clearance,
                right_clearance_score=right_clearance,
                suggested_speed_factor=0.0,
                suggested_steer_dir="CENTER",
            )

        # RULE 3: Single / Manageable Corridor Intrusion -> Evaluate Avoidance Bypass
        if len(intruders) > 0:
            primary = intruders[0]
            # Average horizontal position of intruders
            avg_x = np.mean([o.center_normalized[0] for o in intruders])

            # If intruder is towards the left of corridor and right side is clear
            if avg_x <= 0.50 and right_clearance >= 0.40:
                return DecisionResult(
                    decision=NavigationDecision.MOVE_RIGHT,
                    reason=f"Avoid {primary.class_name} on left side of corridor",
                    primary_obstacle=f"{primary.class_name} (ID:{primary.track_id})",
                    total_objects=len(obstacles),
                    corridor_intruders=len(intruders),
                    left_clearance_score=left_clearance,
                    right_clearance_score=right_clearance,
                    suggested_speed_factor=0.50,
                    suggested_steer_dir="RIGHT",
                )

            # If intruder is towards the right of corridor and left side is clear
            elif avg_x > 0.50 and left_clearance >= 0.40:
                return DecisionResult(
                    decision=NavigationDecision.MOVE_LEFT,
                    reason=f"Avoid {primary.class_name} on right side of corridor",
                    primary_obstacle=f"{primary.class_name} (ID:{primary.track_id})",
                    total_objects=len(obstacles),
                    corridor_intruders=len(intruders),
                    left_clearance_score=left_clearance,
                    right_clearance_score=right_clearance,
                    suggested_speed_factor=0.50,
                    suggested_steer_dir="LEFT",
                )

            # Center blocked with insufficient lateral clearance -> STOP
            else:
                return DecisionResult(
                    decision=NavigationDecision.STOP,
                    reason=f"Corridor blocked by {primary.class_name} with inadequate lateral clearance",
                    primary_obstacle=f"{primary.class_name} (ID:{primary.track_id})",
                    total_objects=len(obstacles),
                    corridor_intruders=len(intruders),
                    left_clearance_score=left_clearance,
                    right_clearance_score=right_clearance,
                    suggested_speed_factor=0.0,
                    suggested_steer_dir="CENTER",
                )

        # RULE 4: Peripheral Obstacles Near Corridor Edge -> SLOW_DOWN Caution
        marginal_obstacles = [
            o for o in obstacles
            if abs(o.center_normalized[0] - self.corridor_left) < 0.10
            or abs(o.center_normalized[0] - self.corridor_right) < 0.10
        ]
        if marginal_obstacles:
            primary = marginal_obstacles[0]
            return DecisionResult(
                decision=NavigationDecision.SLOW_DOWN,
                reason=f"Caution: {primary.class_name} adjacent to corridor boundary",
                primary_obstacle=f"{primary.class_name} (ID:{primary.track_id})",
                total_objects=len(obstacles),
                corridor_intruders=0,
                left_clearance_score=left_clearance,
                right_clearance_score=right_clearance,
                suggested_speed_factor=0.70,
                suggested_steer_dir="CENTER",
            )

        # RULE 5: All checks clear
        return DecisionResult(
            decision=NavigationDecision.PATH_CLEAR,
            reason="Driving corridor fully unobstructed",
            primary_obstacle=None,
            total_objects=len(obstacles),
            corridor_intruders=0,
            left_clearance_score=left_clearance,
            right_clearance_score=right_clearance,
            suggested_speed_factor=1.0,
            suggested_steer_dir="CENTER",
        )

    def render_decision_hud(
        self, canvas: np.ndarray, obstacles: List[PositionedObstacle], decision: DecisionResult, latency_ms: float
    ) -> np.ndarray:
        """Draws heads-up navigation display, decision banner, and directional indicator."""
        h, w = canvas.shape[:2]

        # Base positioning overlay from positioner
        canvas = self.positioner.render_positioning_overlay(canvas, obstacles, latency_ms)

        # Decision Color Mapping
        decision_colors = {
            NavigationDecision.PATH_CLEAR: (0, 255, 0),    # Green
            NavigationDecision.SLOW_DOWN: (0, 215, 255),   # Yellow
            NavigationDecision.MOVE_LEFT: (255, 144, 30),  # Blue
            NavigationDecision.MOVE_RIGHT: (255, 144, 30), # Blue
            NavigationDecision.STOP: (0, 0, 255),          # Bright Red
            NavigationDecision.UNKNOWN: (150, 150, 150)    # Gray
        }
        dec_color = decision_colors.get(decision.decision, (255, 255, 255))

        # Bottom Decision Banner
        banner_h = 70
        banner_y1 = h - banner_h
        banner_overlay = canvas.copy()
        cv2.rectangle(banner_overlay, (0, banner_y1), (w, h), (15, 15, 15), -1)
        cv2.addWeighted(banner_overlay, 0.85, canvas, 0.15, 0, canvas)

        # Decision Pill / Badge
        dec_text = f"DECISION: {decision.decision.value}"
        (tw, th), _ = cv2.getTextSize(dec_text, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
        cv2.rectangle(canvas, (15, banner_y1 + 10), (25 + tw, banner_y1 + 10 + th + 10), dec_color, -1)
        cv2.putText(
            canvas, dec_text, (20, banner_y1 + 10 + th + 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 2, cv2.LINE_AA
        )

        # Sub-status text (Reason & Proposed Speed/Steering)
        reason_text = f"Reason: {decision.reason}"
        ctrl_text = f"Proposed: Steer {decision.suggested_steer_dir} | Speed {decision.suggested_speed_factor*100:.0f}%"
        cv2.putText(canvas, reason_text, (35 + tw, banner_y1 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(canvas, ctrl_text, (35 + tw, banner_y1 + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.44, dec_color, 1, cv2.LINE_AA)

        # Directional Steer Indicator Arrow (Center-Top)
        arrow_cx = w // 2
        arrow_cy = 75
        if decision.decision == NavigationDecision.MOVE_LEFT:
            cv2.arrowedLine(canvas, (arrow_cx + 40, arrow_cy), (arrow_cx - 40, arrow_cy), (255, 144, 30), 4, tipLength=0.35)
            cv2.putText(canvas, "STEER LEFT", (arrow_cx - 50, arrow_cy + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 144, 30), 2)
        elif decision.decision == NavigationDecision.MOVE_RIGHT:
            cv2.arrowedLine(canvas, (arrow_cx - 40, arrow_cy), (arrow_cx + 40, arrow_cy), (255, 144, 30), 4, tipLength=0.35)
            cv2.putText(canvas, "STEER RIGHT", (arrow_cx - 55, arrow_cy + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 144, 30), 2)
        elif decision.decision == NavigationDecision.STOP:
            cv2.circle(canvas, (arrow_cx, arrow_cy), 22, (0, 0, 255), -1)
            cv2.putText(canvas, "STOP", (arrow_cx - 24, arrow_cy + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        return canvas

    def run(self):
        print("\n" + "=" * 70)
        print("🧠 RUNNING MULTI-OBJECT OBSTACLE DECISION ENGINE (OFFLINE)")
        print("   VEHICLE CONTROL: DISCONNECTED (SAFE OFFLINE MODE)")
        print(f"   Log Destination: {self.log_jsonl_path}")
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
                results = self.positioner.model.track(
                    source=frame,
                    persist=True,
                    tracker="bytetrack.yaml",
                    conf=self.positioner.conf_threshold,
                    device="cpu",
                    verbose=False,
                    classes=list(self.positioner.target_class_ids)
                )
                t_end = time.perf_counter()
                latency_ms = (t_end - t_start) * 1000.0

                # Analyze obstacles and generate decision
                obstacles = self.positioner.analyze_frame(frame, results[0])
                decision = self.evaluate(obstacles, frame.shape)
                self.decision_history.append(decision.decision.value)

                # Render HUD
                hud_frame = self.render_decision_hud(frame, obstacles, decision, latency_ms)

                # Periodic terminal output
                if self.frame_count % 10 == 0 or decision.decision == NavigationDecision.STOP:
                    print(
                        f"Frame {self.frame_count:04d} | "
                        f"Latency: {latency_ms:5.1f} ms | "
                        f"Decision: {decision.decision.value:11s} | "
                        f"{decision.reason}"
                    )

                # Save JSONL telemetry log
                log_entry = {
                    "timestamp": time.time(),
                    "frame": self.frame_count,
                    "latency_ms": round(latency_ms, 2),
                    "decision": decision.decision.value,
                    "reason": decision.reason,
                    "primary_obstacle": decision.primary_obstacle,
                    "total_objects": decision.total_objects,
                    "corridor_intruders": decision.corridor_intruders,
                    "left_clearance": round(decision.left_clearance_score, 2),
                    "right_clearance": round(decision.right_clearance_score, 2),
                    "suggested_speed_factor": decision.suggested_speed_factor,
                    "suggested_steer_dir": decision.suggested_steer_dir
                }
                self.log_file.write(json.dumps(log_entry) + "\n")
                self.log_file.flush()

                if self.video_writer is not None:
                    self.video_writer.write(hud_frame)

                if not self.headless:
                    cv2.imshow("VIRYA Buggy - Decision Engine HUD", hud_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        break

                if self.max_frames and self.frame_count >= self.max_frames:
                    print(f"\nReached max frames limit ({self.max_frames}).")
                    break

        finally:
            self._cleanup()
            print(f"Decision session completed. Processed {self.frame_count} frames in {time.time()-start_wall_time:.2f} s.")

    def _cleanup(self):
        if self.cap is not None:
            self.cap.release()
        if self.video_writer is not None:
            self.video_writer.release()
        if self.log_file is not None and not self.log_file.closed:
            self.log_file.close()
            print("✅ Decision telemetry log closed.")
        if not self.headless:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="VIRYA DBW Buggy - Multi-Object Obstacle Decision Engine (Offline)")
    parser.add_argument("--model", type=str, default="/home/vat/yolo_vehicle_test/yolo26n-seg.pt")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-video", type=str, default=None)
    parser.add_argument("--log", type=str, default="/home/vat/yolo_vehicle_test/logs/decision_log.jsonl")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--corridor-left", type=float, default=0.25)
    parser.add_argument("--corridor-right", type=float, default=0.75)

    args = parser.parse_args()

    if not os.environ.get("DISPLAY"):
        if not args.headless:
            args.headless = True

    engine = ObstacleDecisionEngine(
        model_path=args.model,
        camera_id=args.camera,
        conf_threshold=args.conf,
        corridor_bounds=(args.corridor_left, args.corridor_right),
        headless=args.headless,
        save_video_path=args.save_video,
        log_jsonl_path=args.log,
        max_frames=args.max_frames,
    )
    engine.run()


if __name__ == "__main__":
    main()
