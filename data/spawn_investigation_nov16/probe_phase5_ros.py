
import sys
import os
import math
import json
import time

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
if "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception" not in sys.path:
    sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import rclpy
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from rgbd_perception.autonomous_planner_node import AutonomousPlannerNode
from rgbd_perception.rgbd_odometry_node import RgbdOdometryNode
from rgbd_perception.config import load_config

rclpy.init()

sx, sy, sz = 6.0, -6.0, 0.35
gx, gy = 13.5, -6.0
yaw = 0.0

# Check Odometry node
full_cfg = load_config()
odom_node = RgbdOdometryNode(full_cfg, start_pose=[sx, sy, sz, yaw])

# Check Planner node
p_cfg = {
    "planner": {"max_steer": 0.45, "cruise_speed": 0.8, "slow_speed": 0.5, "goal_tolerance": 0.3},
    "topics": {"cmd": "/cmd_ackermann", "objects": "/perception/tracked_obstacles", "odom": "/vehicle/estimated_odom"}
}
scen_data = {
    "vehicle_start_pose": [sx, sy, sz, yaw],
    "vehicle_destination_pose": [gx, gy, 0.0]
}
planner = AutonomousPlannerNode(p_cfg, scenario_data=scen_data)
planner.odom_received = True
planner.perception_received = True
planner.clock_received = True
planner.init_time = time.time() - 10.0

# Initial path check
assert planner.start_xy == (sx, sy), "Planner start mismatch"
assert planner.goal_xy == (gx, gy), "Planner goal mismatch"

# Detour check around static table at (10.0, -6.0)
planner.veh_x, planner.veh_y, planner.veh_yaw = sx, sy, yaw
det_msg = Detection3DArray()
det_msg.header.stamp = planner.get_clock().now().to_msg()
det = Detection3D()
# True S02 table coordinate at world (10.0, -6.0)
ox, oy = 10.0, -6.0
dx = ox - sx
dy = oy - sy
x_v = dx * math.cos(yaw) + dy * math.sin(yaw)
y_v = -dx * math.sin(yaw) + dy * math.cos(yaw)
det.bbox.center.position.x = y_v
det.bbox.center.position.z = x_v
det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = 1.0, 0.8, 1.6
hyp = ObjectHypothesisWithPose()
hyp.hypothesis.class_id = "table"
hyp.hypothesis.score = 0.95
det.results.append(hyp)
det_msg.detections.append(det)

planner.on_tracked_obstacles(det_msg)
steer_detour, speed_detour = planner._plan_motion()

res = {
    "odom_init": [odom_node.x, odom_node.y, odom_node.yaw],
    "planner_start": list(planner.start_xy),
    "planner_goal": list(planner.goal_xy),
    "global_waypoints_count": len(planner.global_waypoints),
    "detour_state": planner.state,
    "detour_steer_rad": round(steer_detour, 3),
    "detour_speed_mps": round(speed_detour, 2),
    "status": "PASS" if "REPLAN" in planner.state else "FAIL"
}

with open("/home/ubuntu/SKATEBOARD_IRP/data/spawn_investigation_nov16/phase5_ros_result.json", "w") as f:
    json.dump(res, f, indent=2)

rclpy.shutdown()
