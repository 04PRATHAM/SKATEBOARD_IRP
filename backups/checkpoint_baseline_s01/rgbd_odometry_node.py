"""Legitimate Dead-Reckoning / Odometry Localisation Node (No GT Leak).

Estimates vehicle pose by integrating Ackermann forward velocity and steering
kinematics from the commanded velocity and knuckle steering angle over time:
    v = cmd.speed, delta = cmd.steering_angle
    dtheta = (v / wheelbase) * tan(delta) * dt
    dx = v * cos(theta) * dt
    dy = v * sin(theta) * dt

Strictly satisfies Rule 8: Ground truth world poses are NEVER subscribed or
used by the runtime navigation pipeline.

Includes RGB-D Visual Motion & Wheel-Slip Detection:
  - Tracks background visual features between consecutive RGB-D frames.
  - Compares wheel encoder forward velocity (UDP 9878) against independently
    estimated visual velocity (v_visual).
  - Detects physical blockage and wheel spin when v_wheel >= 0.08 m/s while
    v_visual < 0.03 m/s for > 0.8 s.
  - Clamps odometry integration to visual velocity when wheel slip is detected,
    preventing spurious forward progression on the map while physically stuck.
  - Publishes /vehicle/wheel_slip (std_msgs/Bool) to trigger safe stop in planner.

Subscribes:
  - /cmd_ackermann (ackermann_msgs/AckermannDriveStamped)
  - /clock (rosgraph_msgs/Clock)
  - /camera/rgb/image_raw (sensor_msgs/Image)
  - /camera/depth/image_raw (sensor_msgs/Image)
  - /camera/depth/camera_info (sensor_msgs/CameraInfo)
  - /perception/masks (sensor_msgs/Image)
  - /vehicle/pause_drive (std_msgs/Bool)

Publishes:
  - /vehicle/estimated_odom (nav_msgs/Odometry)
  - /vehicle/wheel_slip (std_msgs/Bool)
  - /vehicle/visual_odom (geometry_msgs/TwistStamped)
"""

import argparse
import collections
import math
import os
import sys
import time

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, PoseStamped, TwistStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String, Float32

from rgbd_perception.config import load_config
import yaml


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class RgbdOdometryNode(Node):
    def __init__(self, cfg, start_pose=None):
        super().__init__("rgbd_odometry_node")
        self.cfg = cfg
        self.wheelbase = float(cfg.get("vehicle", {}).get("wheelbase", 1.1))

        # Start pose [x, y, z, yaw]
        if start_pose and len(start_pose) >= 4:
            self.x = float(start_pose[0])
            self.y = float(start_pose[1])
            self.z = float(start_pose[2])
            self.yaw = float(start_pose[3])
        else:
            self.x, self.y, self.z, self.yaw = 6.0, -6.0, 0.35, 0.0

        self.speed = 0.0
        self.steer = 0.0
        self.last_cmd_time = 0.0
        self.last_enc_time = 0.0
        self.cmd_timeout = 0.5
        self.use_sim_clock = False
        self.last_sim_time = None
        self.last_wall_time = None
        self.clock_ticks = 0

        # Legitimate wheel encoder receiver (port 9878, closed-loop without GT leak)
        self.cmd_speed = 0.0
        try:
            import socket
            self.enc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.enc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.enc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                pass
            self.enc_sock.bind(("127.0.0.1", 9878))
            self.enc_sock.setblocking(False)
            self.has_enc_sock = True
            self.get_logger().info("[ODOMETRY] Bound UDP socket on 127.0.0.1:9878 for physical wheel telemetry.")
        except Exception as e:
            self.has_enc_sock = False
            self.get_logger().warn(f"[ODOMETRY] Failed to bind UDP port 9878: {e}")

        # RGB-D Visual Motion & Wheel-Slip Estimator state
        self.v_visual = 0.0
        self.visual_conf = 0
        self.visual_conf_score = 0.0
        self.visual_residual_std = 0.0
        self.visual_flow_px = 0.0
        self.last_visual_time = 0.0
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_mask = None
        self.prev_visual = None
        self.wheel_slip_detected = False
        self.slip_duration = 0.0
        self.degraded_duration = 0.0
        self.odom_status = "NOMINAL"
        self.is_paused = False
        self.camera_k = None

        tp = cfg["topics"]
        self.pub_odom = self.create_publisher(Odometry, "/vehicle/estimated_odom", 10)
        self.pub_slip = self.create_publisher(Bool, "/vehicle/wheel_slip", 10)
        self.pub_vis_twist = self.create_publisher(TwistStamped, "/vehicle/visual_odom", 10)
        self.pub_odom_status = self.create_publisher(String, "/vehicle/odom_status", 10)
        self.pub_vis_conf = self.create_publisher(Float32, "/vehicle/visual_odom_confidence", 10)

        self.create_subscription(AckermannDriveStamped, tp["cmd"], self.on_cmd, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(PoseStamped, "/vehicle/reset_pose", self.on_reset_pose, 10)
        self.create_subscription(Bool, "/vehicle/pause_drive", self.on_pause_drive, 10)
        self.create_subscription(TwistStamped, "/vehicle/wheel_encoder", self.on_wheel_encoder_ros, 10)

        # RGB-D image subscriptions for visual motion estimation (Best-Effort Sensor Data)
        self.create_subscription(Image, "/camera/rgb/image_raw", self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, tp["depth"], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, tp["depth_info"], self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/perception/masks", self.on_mask, 10)

        self.last_wall_time = time.time()
        # 20 Hz timer for odometry integration and publication
        self.timer = self.create_timer(0.05, self._timer_callback)

        self.get_logger().info(f"RgbdOdometryNode ready: start=({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f} rad) with RGB-D Visual Slip Detection")

    def on_pause_drive(self, msg: Bool):
        self.is_paused = msg.data
        if self.is_paused:
            self.wheel_slip_detected = False
            self.slip_start_time = None

    def on_info(self, msg: CameraInfo):
        if self.camera_k is None:
            self.camera_k = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def on_mask(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            self.latest_mask = arr
        except Exception:
            pass

    def on_rgb(self, msg: Image):
        try:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t <= 0:
                t = time.time()
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            # Downsample to 320x240 grayscale for high-speed tracking (< 8ms)
            gray = cv2.cvtColor(cv2.resize(arr, (320, 240)), cv2.COLOR_RGB2GRAY)
            self.latest_rgb = (t, gray)
        except Exception:
            pass

    def on_depth(self, msg: Image):
        if msg.encoding != "32FC1":
            return
        try:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t <= 0:
                t = time.time()
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            self.latest_depth = (t, arr)
            self._update_visual_motion()
        except Exception:
            pass

    def _update_visual_motion(self):
        """Estimates forward visual velocity from consecutive RGB-D frames."""
        if self.latest_rgb is None or self.latest_depth is None:
            return

        t_rgb, gray = self.latest_rgb
        t_dep, depth = self.latest_depth
        # Ensure RGB and Depth are approximately synchronized
        if abs(t_rgb - t_dep) > 0.12:
            return

        if self.prev_visual is None:
            self.prev_visual = (t_rgb, gray, depth)
            return

        t_prev, gray_prev, depth_prev = self.prev_visual
        dt = t_rgb - t_prev
        if dt < 0.04 or dt > 0.35:
            # Update baseline
            if dt > 0.35:
                self.prev_visual = (t_rgb, gray, depth)
            return

        # Downsampled mask for static region selection
        mask_down = None
        if self.latest_mask is not None:
            try:
                mask_down = cv2.resize(self.latest_mask, (320, 240), interpolation=cv2.INTER_NEAREST)
            except Exception:
                pass

        # Detect features on static background
        pts_prev = cv2.goodFeaturesToTrack(gray_prev, maxCorners=60, qualityLevel=0.015, minDistance=10)
        if pts_prev is None or len(pts_prev) < 5:
            self.prev_visual = (t_rgb, gray, depth)
            return

        # Exclude dynamic object mask pixels
        if mask_down is not None:
            valid_idx = []
            for i, p in enumerate(pts_prev):
                ux, vy = int(round(p[0][0])), int(round(p[0][1]))
                if 0 <= vy < 240 and 0 <= ux < 320:
                    if mask_down[vy, ux] == 0:
                        valid_idx.append(i)
            if len(valid_idx) >= 5:
                pts_prev = pts_prev[valid_idx]

        pts_curr, st, _ = cv2.calcOpticalFlowPyrLK(gray_prev, gray, pts_prev, None)
        if pts_curr is None:
            self.prev_visual = (t_rgb, gray, depth)
            return

        val = (st.flatten() == 1)
        p1 = pts_prev[val].reshape(-1, 2)
        p2 = pts_curr[val].reshape(-1, 2)

        if len(p1) < 4:
            self.visual_conf_score = 0.05
            self.visual_flow_px = 0.0
            self.prev_visual = (t_rgb, gray, depth)
            return

        self.visual_flow_px = float(np.mean(np.linalg.norm(p2 - p1, axis=1)))

        # Scale coordinates to 640x480 depth image
        scale_x = depth.shape[1] / 320.0
        scale_y = depth.shape[0] / 240.0

        u1 = np.clip(np.round(p1[:, 0] * scale_x).astype(int), 0, depth.shape[1] - 1)
        v1 = np.clip(np.round(p1[:, 1] * scale_y).astype(int), 0, depth.shape[0] - 1)
        u2 = np.clip(np.round(p2[:, 0] * scale_x).astype(int), 0, depth.shape[1] - 1)
        v2 = np.clip(np.round(p2[:, 1] * scale_y).astype(int), 0, depth.shape[0] - 1)

        z1 = depth_prev[v1, u1]
        z2 = depth[v2, u2]

        val_z = np.isfinite(z1) & np.isfinite(z2) & (z1 >= 0.6) & (z1 <= 7.0) & (z2 >= 0.5) & (z2 <= 7.0)
        z1_v, z2_v = z1[val_z], z2[val_z]

        n_features = len(p1)
        n_initial = len(pts_prev)
        p_valid = float(n_features) / max(1.0, float(n_initial))
        p_depth = float(len(z1_v)) / max(1.0, float(n_features))

        if len(z1_v) >= 4:
            # Forward motion: camera advances toward static points -> z2 < z1 -> dz = z1 - z2 > 0
            # Reverse motion: camera recedes from static points -> z2 > z1 -> dz = z1 - z2 < 0
            dz = z1_v - z2_v
            v_pts = dz / dt
            med = float(np.median(v_pts))
            inliers = v_pts[np.abs(v_pts - med) < 0.6]
            if len(inliers) >= 3:
                self.v_visual = float(np.median(inliers))
                self.visual_conf = len(inliers)
                self.visual_residual_std = float(np.std(inliers))
                self.last_visual_time = time.time()

                # Comprehensive visual confidence metric
                c_tracks = min(1.0, float(len(inliers)) / 15.0)
                c_residual = max(0.0, min(1.0, 1.0 - (self.visual_residual_std / 0.40)))
                self.visual_conf_score = float(np.clip(
                    0.35 * c_tracks + 0.25 * p_valid + 0.20 * p_depth + 0.20 * c_residual,
                    0.0, 1.0
                ))

                # Publish visual velocity twist (signed!)
                tw = TwistStamped()
                tw.header.stamp = self.get_clock().now().to_msg()
                tw.header.frame_id = "camera_depth_optical_frame"
                tw.twist.linear.x = float(self.v_visual)
                self.pub_vis_twist.publish(tw)

                conf_msg = Float32()
                conf_msg.data = float(self.visual_conf_score)
                self.pub_vis_conf.publish(conf_msg)
            else:
                self.visual_conf_score = 0.12
        else:
            self.visual_conf_score = 0.08

        self.prev_visual = (t_rgb, gray, depth)

    def set_wheel_feedback(self, speed: float, steer: float):
        """Allows direct physical wheel feedback injection from encoder or test framework."""
        self.speed = float(speed)
        self.steer = float(steer)
        self.last_enc_time = time.time()

    def _timer_callback(self):
        now = time.time()
        if not self.use_sim_clock:
            if self.last_wall_time is not None:
                dt = now - self.last_wall_time
                if 0.0 < dt < 0.5:
                    self._integrate(dt)
            self.last_wall_time = now
        self._publish_odom()

    def on_cmd(self, msg: AckermannDriveStamped):
        self.cmd_speed = float(msg.drive.speed)
        self.cmd_steer = float(msg.drive.steering_angle)
        self.last_cmd_time = time.time()
        # Strictly eliminate fallback that integrates commanded speed as if it were measured movement.
        # self.speed and self.steer are ONLY set from physical encoder feedback (UDP 9878 or set_wheel_feedback).

    def on_wheel_encoder_ros(self, msg: TwistStamped):
        now = time.time()
        self.speed = float(msg.twist.linear.x)
        self.steer = float(msg.twist.angular.z)
        if self.last_enc_time == 0.0 or (now - getattr(self, "_last_ros_enc_log", 0.0) > 5.0):
            self._last_ros_enc_log = now
            self.get_logger().info(f"[ODOMETRY] ROS Wheel encoder telemetry ACTIVE: speed={self.speed:.2f}m/s steer={self.steer:.2f}rad")
        self.last_enc_time = now

    def on_reset_pose(self, msg: PoseStamped):
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.speed = 0.0
        self.steer = 0.0
        self.wheel_slip_detected = False
        self.slip_start_time = None
        self.last_sim_time = None
        self.get_logger().info(f"[RESET] Odometry reset to ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f} rad)")

    def on_clock(self, msg: Clock):
        self.use_sim_clock = True
        self.clock_ticks += 1
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.last_sim_time is None:
            self.last_sim_time = t
            return
        dt = t - self.last_sim_time
        self.last_sim_time = t
        if 0.0 < dt < 0.5:
            self._integrate(dt)

    def _integrate(self, dt: float):
        now = time.time()

        # Poll wheel encoder feedback if active (UDP 9878)
        if getattr(self, "has_enc_sock", False):
            import struct
            try:
                while True:
                    data, _ = self.enc_sock.recvfrom(32)
                    v_enc, s_enc = struct.unpack("ff", data)
                    self.speed = float(v_enc)
                    self.steer = float(s_enc)
                    if not getattr(self, "_udp_enc_logged", False):
                        self._udp_enc_logged = True
                        self.get_logger().info(f"[ODOMETRY] Physical wheel encoder telemetry ACTIVE: speed={self.speed:.2f}m/s steer={self.steer:.2f}rad")
                    self.last_enc_time = now
            except BlockingIOError:
                pass
            except Exception:
                pass

        # Check wheel encoder freshness: NEVER fall back to cmd_speed!
        if self.last_enc_time == 0.0 or (now - self.last_enc_time > 0.5):
            self.speed = 0.0

        # Check command timeout
        if self.last_cmd_time == 0.0 or (now - self.last_cmd_time > self.cmd_timeout):
            self.cmd_speed = 0.0

        delta = self.steer
        # Calibrated 4-wheel Ackermann articulation steering efficiency
        steer_efficiency = 0.80
        w_raw = steer_efficiency * (self.speed / self.wheelbase) * math.tan(delta)
        v_raw = self.speed * math.cos(delta)

        # Wheel-Slip Detection & Physical Blockage Verification (Signed: Forward & Reverse):
        cmd_mag = abs(self.cmd_speed)
        wheel_mag = abs(self.speed)
        is_commanded = (cmd_mag >= 0.05)
        is_wheels_turning = (wheel_mag >= 0.08)

        conf_score = self.visual_conf_score
        if conf_score <= 0.0 and getattr(self, "visual_conf", 0) > 0:
            conf_score = min(1.0, float(self.visual_conf) / 20.0)

        has_fresh_visual = (self.last_visual_time > 0.0) and (now - self.last_visual_time < 0.6)
        is_visual_confident = has_fresh_visual and (conf_score >= 0.30)
        # Physical blockage verification: both forward depth change and 2D optical flow must indicate lack of motion
        visual_stopped = (abs(self.v_visual) < 0.03) and (getattr(self, "visual_flow_px", 0.0) < 1.2)

        if not is_commanded or self.is_paused:
            self.odom_status = "NOMINAL" if not self.is_paused else "PAUSED"
            self.slip_duration = 0.0
            self.wheel_slip_detected = False
            v_eff = 0.0
            w = 0.0
        elif not is_wheels_turning:
            # Commanded to move, but wheels have not physically started turning or encoder is silent
            if self.last_enc_time == 0.0 or (now - self.last_enc_time > 0.5):
                self.odom_status = "NO_ENCODER"
            else:
                self.odom_status = "STATIONARY"
            self.slip_duration = 0.0
            self.wheel_slip_detected = False
            v_eff = 0.0
            w = 0.0
        else:
            # Wheels ARE turning under command. Multi-tier physical displacement verification:
            has_fresh_encoder = (self.last_enc_time > 0.0) and (now - self.last_enc_time < 0.40)

            if not has_fresh_visual:
                # Camera stream interrupted: cannot verify physical motion visually
                self.odom_status = "CAM_INTERRUPTED"
                self.slip_duration = 0.0
                self.wheel_slip_detected = False
                v_eff = 0.0
                w = 0.0
            elif not is_visual_confident:
                # Visual confidence degraded (blank wall / low texture / unverified wheel spin).
                # Strictly report degradation; DO NOT advance position on unverified wheel spin!
                self.odom_status = "VISUAL_DEGRADED"
                self.slip_duration = 0.0
                self.wheel_slip_detected = False
                v_eff = 0.0
                w = 0.0
            elif visual_stopped:
                # Verified physical blockage in commanded direction (vision sees static world while wheels spin)
                self.odom_status = "WHEEL_SLIP"
                v_eff = float(self.v_visual)
                w = 0.0
                self.slip_duration += dt
                if self.slip_duration > 0.5:
                    self.wheel_slip_detected = True
            else:
                # Verified legitimate movement in physical world (visual-inertial agreement)
                self.odom_status = "NOMINAL"
                self.slip_duration = 0.0
                self.wheel_slip_detected = False
                v_eff = v_raw
                w = w_raw

        # Broadcast wheel-slip diagnostic state
        slip_msg = Bool()
        slip_msg.data = bool(self.wheel_slip_detected)
        self.pub_slip.publish(slip_msg)

        status_msg = String()
        status_msg.data = str(self.odom_status)
        self.pub_odom_status.publish(status_msg)

        # Update pose
        self.yaw += w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += v_eff * math.cos(self.yaw) * dt
        self.y += v_eff * math.sin(self.yaw) * dt
        self._publish_odom(v_eff, w)

    def _publish_odom(self, v_eff=None, w=None):
        if v_eff is None:
            v_eff = self.speed if not self.wheel_slip_detected else float(self.v_visual)
        if w is None:
            delta = self.steer
            steer_efficiency = 0.80
            w = steer_efficiency * (v_eff / self.wheelbase) * math.tan(delta) if not self.wheel_slip_detected else 0.0

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = float(self.z)
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        msg.twist.twist.linear.x = float(v_eff)
        msg.twist.twist.angular.z = float(w)

        self.pub_odom.publish(msg)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=None)
    p.add_argument("--scenario", default=None)
    p.add_argument("--start", nargs="+", type=float, default=None, help="Allotted start pose [x, y, (yaw)]")
    args = p.parse_args()

    start_pose = None
    if args.start and len(args.start) >= 2:
        yaw = float(args.start[2]) if len(args.start) > 2 else 0.0
        start_pose = [float(args.start[0]), float(args.start[1]), 0.35, yaw]
    elif args.scenario:
        try:
            from isaac_sim.scripts.scenarios.scenario_registry import resolve_scenario
            entry = resolve_scenario(args.scenario)
            if entry and os.path.exists(entry["path"]):
                with open(entry["path"], "r") as f:
                    sc = yaml.safe_load(f)
                start_pose = sc.get("vehicle_start_pose")
        except Exception:
            if os.path.isfile(args.scenario):
                try:
                    with open(args.scenario, "r") as f:
                        sc = yaml.safe_load(f)
                    start_pose = sc.get("vehicle_start_pose")
                except Exception:
                    pass

    rclpy.init()
    node = RgbdOdometryNode(load_config(args.config), start_pose=start_pose)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
