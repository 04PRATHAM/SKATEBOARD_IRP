"""Multi-Object Tracking, Unknown Obstacle Detection and Dynamic Trajectory Prediction.

Subscribes:
  - /perception/objects_3d (Detection3DArray from rgbd_fusion_node)
  - /camera/depth/image_raw (sensor_msgs/Image 32FC1)
  - /camera/depth/camera_info (sensor_msgs/CameraInfo)

Performs:
  1. Persistent tracking of 3D objects with unique Track IDs.
  2. Velocity estimation (vx, vy, vz) in optical and vehicle frames.
  3. Dynamic classification (speed > 0.15 m/s -> dynamic; otherwise static).
  4. Multi-step forward trajectory prediction: P(t) = P(0) + v * t (t = 0.5 .. 3.0s).
  5. Unknown obstacle detection: foreground non-floor depth clusters not covered by YOLO.

Publishes:
  - /perception/tracked_obstacles (vision_msgs/Detection3DArray)
  - /perception/predicted_markers (visualization_msgs/MarkerArray)
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point

from rgbd_perception.config import load_config


class Track:
    def __init__(self, track_id: int, class_id: int, class_name: str, x: float, y: float, z: float, t: float, conf: float = 0.90):
        self.track_id = track_id
        self.class_id = class_id
        self.class_name = class_name
        self.confidence = float(conf)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.speed = 0.0
        self.is_dynamic = False
        self.last_update = t
        self.creation_time = t
        self.hits = 1
        self.misses = 0
        self.size_x = 0.6
        self.size_y = 1.6
        self.size_z = 0.5
        self.yaw = 0.0
        self.history = [(t, self.x, self.y, self.z)]

    def update(self, x: float, y: float, z: float, t: float, sx: float = 0.6, sy: float = 1.6, sz: float = 0.5, conf: float = 0.90):
        dt = max(1e-3, t - self.last_update)
        self.confidence = float(conf)
        # Raw velocity in optical frame
        raw_vx = (x - self.x) / dt
        raw_vy = (y - self.y) / dt
        raw_vz = (z - self.z) / dt
        raw_speed = math.hypot(raw_vx, raw_vz)

        # Smooth velocity with exponential filter
        alpha_v = 0.55 if raw_speed < 3.0 else 0.1
        self.vx = (1.0 - alpha_v) * self.vx + alpha_v * raw_vx
        self.vy = (1.0 - alpha_v) * self.vy + alpha_v * raw_vy
        self.vz = (1.0 - alpha_v) * self.vz + alpha_v * raw_vz

        # Filter position
        alpha_p = 0.70
        self.x = (1.0 - alpha_p) * self.x + alpha_p * x
        self.y = (1.0 - alpha_p) * self.y + alpha_p * y
        self.z = (1.0 - alpha_p) * self.z + alpha_p * z

        self.size_x = sx
        self.size_y = sy
        self.size_z = sz
        if self.speed > 0.15:
            # Optical frame horizontal plane: -vx is left (vehicle +Y), vz is forward (vehicle +X)
            self.yaw = math.atan2(-self.vx, self.vz)
        self.last_update = t
        self.hits += 1
        self.misses = 0

        self.speed = math.hypot(self.vx, self.vz)
        if self.hits >= 2 and self.speed > 0.18:
            self.is_dynamic = True
        elif self.speed < 0.08:
            self.is_dynamic = False

        self.history.append((t, self.x, self.y, self.z))
        if len(self.history) > 30:
            self.history.pop(0)

    def predict(self, horizons: List[float] = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]) -> List[Tuple[float, float, float, float]]:
        """Returns list of (t_ahead, x, y, z) future predictions."""
        preds = []
        for h in horizons:
            if self.is_dynamic:
                px = self.x + self.vx * h
                py = self.y + self.vy * h
                pz = self.z + self.vz * h
            else:
                px, py, pz = self.x, self.y, self.z
            preds.append((h, px, py, pz))
        return preds


class DynamicTrackerNode(Node):
    def __init__(self, cfg):
        super().__init__("dynamic_tracker_node")
        self.cfg = cfg
        self.max_match_dist = 1.3  # metres gating
        self.max_age_sec = 1.0     # prune track after 1.0s without detections
        self.next_track_id = 1
        self.tracks: Dict[int, Track] = {}

        tp = cfg["topics"]
        self.depth_frame = tp.get("depth_optical_frame", "camera_depth_optical_frame")

        self.pub_tracked = self.create_publisher(Detection3DArray, "/perception/tracked_obstacles", 10)
        self.pub_velocities = self.create_publisher(DiagnosticArray, "/perception/tracked_velocities", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/perception/predicted_markers", 10)

        self.create_subscription(Detection3DArray, tp["objects"], self.on_objects_3d, 10)
        self.create_subscription(Image, tp["depth"], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, tp["depth_info"], self.on_info, qos_profile_sensor_data)

        self.depth_image = None
        self.camera_k = None
        self.last_depth_stamp = 0.0

        self.timer = self.create_timer(0.05, self.track_maintenance)
        self.get_logger().info("DynamicTrackerNode initialized.")

    def on_info(self, msg):
        self.camera_k = list(msg.k)

    def on_depth(self, msg):
        if msg.encoding != "32FC1":
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            self.depth_image = arr
            self.last_depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception:
            self.depth_image = None

    def on_objects_3d(self, msg: Detection3DArray):
        t = time.time()

        detections = []
        for d in msg.detections:
            try:
                x = float(d.bbox.center.position.x)
                y = float(d.bbox.center.position.y)
                z = float(d.bbox.center.position.z)
                sx = float(d.bbox.size.x) if d.bbox.size.x > 0 else 0.6
                sy = float(d.bbox.size.y) if d.bbox.size.y > 0 else 1.6
                sz = float(d.bbox.size.z) if d.bbox.size.z > 0 else 0.5
                cid_str = d.results[0].hypothesis.class_id if d.results else "0:person"
                parts = str(cid_str).split(":", 1)
                cid = int(parts[0]) if parts[0].lstrip("-").isdigit() else 0
                cname = parts[1] if len(parts) > 1 else "obstacle"
                conf = float(d.results[0].hypothesis.score) if (d.results and d.results[0].hypothesis.score > 0) else 0.92
                detections.append({"x": x, "y": y, "z": z, "sx": sx, "sy": sy, "sz": sz, "cid": cid, "cname": cname, "conf": conf})
            except Exception:
                continue

        # Add unknown depth obstacles if present
        unknown_dets = self._extract_unknown_depth_obstacles(detections)
        detections.extend(unknown_dets)

        # Associate detections with existing tracks
        self._associate_and_update(detections, t)

        # Publish tracked obstacles
        self._publish_tracks(msg.header.stamp)

    def _extract_unknown_depth_obstacles(self, known_dets) -> List[dict]:
        """Detect unmasked non-floor depth clusters in front corridor (only if fusion sent no detections)."""
        if known_dets:
            return []
        if self.depth_image is None or self.camera_k is None:
            return []
        fx, fy, cx, cy = self.camera_k[0], self.camera_k[4], self.camera_k[2], self.camera_k[5]
        H, W = self.depth_image.shape

        # Downsample depth for fast clustering
        step = 16
        small_depth = self.depth_image[::step, ::step]
        sh, sw = small_depth.shape

        candidates = []
        for r in range(sh):
            for c in range(sw):
                z = float(small_depth[r, c])
                if not (1.0 <= z <= 5.5 and np.isfinite(z)):
                    continue
                orig_u = c * step
                orig_v = r * step
                X = (orig_u - cx) * z / fx
                Y = (orig_v - cy) * z / fy

                # Exact height above ground z_v = H_cam - cos(pitch)*Y - sin(pitch)*z
                # H_cam = 1.2m, pitch = -0.15 rad: cos(0.15) = 0.9888, sin(0.15) = 0.1494
                # zv >= 0.28m ensures robust floor ground plane separation despite chassis pitch dynamics
                zv = 1.20 - 0.9888 * Y - 0.1494 * z
                if not (0.28 <= zv <= 1.80):
                    continue
                if abs(X) > 1.8:
                    continue

                # Check if inside any known YOLO detection bbox
                inside_known = False
                for kd in known_dets:
                    if math.hypot(X - kd["x"], z - kd["z"]) < 0.6:
                        inside_known = True
                        break
                if not inside_known:
                    candidates.append((X, Y, z))

        if len(candidates) < 45:
            return []

        # Simple clustering: centroid of candidate points
        xs = [p[0] for p in candidates]
        ys = [p[1] for p in candidates]
        zs = [p[2] for p in candidates]
        med_x = float(np.median(xs))
        med_y = float(np.median(ys))
        med_z = float(np.median(zs))

        # Check existing tracks to prevent duplicate unknown tracks
        for tr in self.tracks.values():
            if math.hypot(med_x - tr.x, med_z - tr.z) < 0.7:
                return []

        return [{
            "x": med_x, "y": med_y, "z": med_z,
            "sx": 0.5, "sy": 0.8, "sz": 0.5,
            "cid": -1, "cname": "unknown_obstacle"
        }]

    def _associate_and_update(self, detections: List[dict], t: float):
        matched_tracks = set()
        matched_dets = set()

        # Greedy closest-distance matching in 3D
        pairs = []
        for di, d in enumerate(detections):
            for tid, tr in self.tracks.items():
                dist = math.hypot(d["x"] - tr.x, d["z"] - tr.z)
                if dist <= self.max_match_dist:
                    pairs.append((dist, di, tid))

        pairs.sort(key=lambda item: item[0])

        for dist, di, tid in pairs:
            if di in matched_dets or tid in matched_tracks:
                continue
            d = detections[di]
            self.tracks[tid].update(d["x"], d["y"], d["z"], t, d["sx"], d["sy"], d.get("sz", 0.5), conf=d.get("conf", 0.90))
            matched_dets.add(di)
            matched_tracks.add(tid)

        # Unmatched detections -> new tracks
        for di, d in enumerate(detections):
            if di not in matched_dets:
                tid = self.next_track_id
                self.next_track_id += 1
                tr = Track(tid, d["cid"], d["cname"], d["x"], d["y"], d["z"], t, conf=d.get("conf", 0.90))
                tr.size_x = d["sx"]
                tr.size_y = d["sy"]
                tr.size_z = d.get("sz", 0.5)
                self.tracks[tid] = tr

    def track_maintenance(self):
        now = time.time()
        to_del = []
        for tid, tr in self.tracks.items():
            if now - tr.last_update > self.max_age_sec:
                to_del.append(tid)
        if to_del:
            for tid in to_del:
                del self.tracks[tid]
            stamp = self.get_clock().now().to_msg()
            self._publish_tracks(stamp)
            self._publish_marker_deletions(to_del, stamp)

    def _publish_marker_deletions(self, deleted_ids: List[int], stamp):
        del_arr = MarkerArray()
        for tid in deleted_ids:
            # Delete obstacle cylinder marker
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.depth_frame
            m.ns = "obstacles"
            m.id = int(tid)
            m.action = Marker.DELETE
            del_arr.markers.append(m)
            # Delete trajectory line marker
            m_traj = Marker()
            m_traj.header.stamp = stamp
            m_traj.header.frame_id = self.depth_frame
            m_traj.ns = "trajectories"
            m_traj.id = int(tid)
            m_traj.action = Marker.DELETE
            del_arr.markers.append(m_traj)
        if del_arr.markers:
            self.pub_markers.publish(del_arr)

    def _publish_tracks(self, stamp):
        out = Detection3DArray()
        out.header.stamp = stamp
        out.header.frame_id = self.depth_frame

        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = stamp
        diag_arr.header.frame_id = self.depth_frame

        marker_arr = MarkerArray()

        for tr in self.tracks.values():
            # Suppress single-frame unclassified/geometric noise:
            # Confirmed YOLO detections (person, table, chair, etc.) and structural depth clusters (wall, pillar) are published on frame 1 (hits >= 1).
            # Unclassified noise clusters (class_id < 0 or unknown_obstacle) require hits >= 2.
            is_confirmed = (tr.class_id >= 0 and tr.class_name not in ("unknown_obstacle", "obstacle") and tr.confidence >= 0.40) or (tr.class_name in ("wall", "pillar"))
            min_hits = 1 if is_confirmed else 2
            if tr.hits < min_hits:
                continue

            # Detection3D message with standard ROS 2 semantics
            d3d = Detection3D()
            d3d.header.stamp = stamp
            d3d.header.frame_id = self.depth_frame
            d3d.id = str(tr.track_id)

            dyn_tag = "dynamic" if tr.is_dynamic else "static"

            # Primary hypothesis: classification and confidence in [0.0, 1.0]
            hyp_class = ObjectHypothesisWithPose()
            hyp_class.hypothesis.class_id = f"{tr.track_id}:{tr.class_id}:{tr.class_name}:{dyn_tag}"
            hyp_class.hypothesis.score = float(tr.confidence)
            hyp_class.pose.pose.position.x = float(tr.x)
            hyp_class.pose.pose.position.y = float(tr.y)
            hyp_class.pose.pose.position.z = float(tr.z)
            d3d.results.append(hyp_class)

            # Secondary hypothesis: dedicated velocity field (speed in score, vx/vy/vz in pose position)
            hyp_vel = ObjectHypothesisWithPose()
            hyp_vel.hypothesis.class_id = "velocity_m_s"
            hyp_vel.hypothesis.score = float(tr.speed)
            hyp_vel.pose.pose.position.x = float(tr.vx)
            hyp_vel.pose.pose.position.y = float(tr.vy)
            hyp_vel.pose.pose.position.z = float(tr.vz)
            d3d.results.append(hyp_vel)

            d3d.bbox.center.position.x = float(tr.x)
            d3d.bbox.center.position.y = float(tr.y)
            d3d.bbox.center.position.z = float(tr.z)
            tr_yaw = getattr(tr, "yaw", 0.0)
            d3d.bbox.center.orientation.z = math.sin(tr_yaw * 0.5)
            d3d.bbox.center.orientation.w = math.cos(tr_yaw * 0.5)
            d3d.bbox.size.x = float(tr.size_x)
            d3d.bbox.size.y = float(tr.size_y)
            d3d.bbox.size.z = float(tr.size_z)
            out.detections.append(d3d)

            # Companion diagnostic tracking status with explicit velocity fields
            ds = DiagnosticStatus()
            ds.name = f"track_{tr.track_id}"
            ds.hardware_id = str(tr.track_id)
            ds.message = f"{tr.class_name} ({dyn_tag})"
            ds.values = [
                KeyValue(key="speed", value=f"{tr.speed:.3f}"),
                KeyValue(key="vx", value=f"{tr.vx:.3f}"),
                KeyValue(key="vy", value=f"{tr.vy:.3f}"),
                KeyValue(key="vz", value=f"{tr.vz:.3f}"),
                KeyValue(key="is_dynamic", value=str(tr.is_dynamic)),
                KeyValue(key="class_name", value=str(tr.class_name)),
                KeyValue(key="confidence", value=f"{tr.confidence:.2f}")
            ]
            diag_arr.status.append(ds)

            # Marker for obstacle
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.depth_frame
            m.ns = "obstacles"
            m.id = int(tr.track_id)
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(tr.x)
            m.pose.position.y = float(tr.y)
            m.pose.position.z = float(tr.z)
            m.scale.x = float(tr.size_x)
            m.scale.y = float(tr.size_x)
            m.scale.z = float(tr.size_y)
            m.color.r = 0.9 if not tr.is_dynamic else 1.0
            m.color.g = 0.2 if not tr.is_dynamic else 0.5
            m.color.b = 0.2 if not tr.is_dynamic else 0.0
            m.color.a = 0.8
            marker_arr.markers.append(m)

            # Predicted trajectory marker (LINE_STRIP)
            preds = tr.predict([0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
            m_pred = Marker()
            m_pred.header.stamp = stamp
            m_pred.header.frame_id = self.depth_frame
            m_pred.ns = "trajectories"
            m_pred.id = int(tr.track_id)
            m_pred.type = Marker.LINE_STRIP
            m_pred.action = Marker.ADD
            m_pred.scale.x = 0.08
            m_pred.color.r = 0.0 if not tr.is_dynamic else 0.9
            m_pred.color.g = 0.8 if not tr.is_dynamic else 0.2
            m_pred.color.b = 0.9 if not tr.is_dynamic else 0.8
            m_pred.color.a = 0.9

            p0 = Point()
            p0.x, p0.y, p0.z = tr.x, tr.y, tr.z
            m_pred.points.append(p0)
            for _, px, py, pz in preds:
                pt = Point()
                pt.x, pt.y, pt.z = float(px), float(py), float(pz)
                m_pred.points.append(pt)
            marker_arr.markers.append(m_pred)

        self.pub_tracked.publish(out)
        self.pub_velocities.publish(diag_arr)
        self.pub_markers.publish(marker_arr)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=None)
    args = p.parse_args()

    rclpy.init()
    node = DynamicTrackerNode(load_config(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
