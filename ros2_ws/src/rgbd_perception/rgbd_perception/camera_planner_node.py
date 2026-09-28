"""Camera-only planner node — Object3D -> /cmd_ackermann with watchdog (Phase G).

Subscribes: vision_msgs/Detection3DArray /perception/objects_3d ONLY.
Publishes: ackermann_msgs/AckermannDriveStamped /cmd_ackermann
  (drive.speed, drive.steering_angle; saturated to config; existing
  controller convention cmd_timeout 0.5 s respected downstream too).
Failsafe: no fresh objects within stale_timeout -> publish (steer 0, speed 0).
  Empty array with fresh stamp = CLEAR corridor (cruise) — distinct from stale.
  No detections ever + inputs flowing = cruise. Inputs silent = stop.
No odom/GT/waypoints/scenario reads anywhere (TEST-K).
"""
import argparse
import sys
import time

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from ackermann_msgs.msg import AckermannDriveStamped

from rgbd_perception.config import load_config
from rgbd_perception.planner import plan


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=None)
    return p.parse_args()


def parse_cid(class_id_str):
    try:
        return int(str(class_id_str).split(":")[0])
    except Exception:
        return -1


class PlannerNode(Node):
    def __init__(self, cfg):
        super().__init__("camera_planner_node")
        self.cfg = cfg
        tp = cfg["topics"]
        self.stale = float(cfg["perception"].get("stale_timeout", 0.5))
        self.max_steer = float(cfg["planner"].get("max_steer", 0.6))
        self.pub = self.create_publisher(AckermannDriveStamped, tp["cmd"], 10)
        self.create_subscription(Detection3DArray, tp["objects"], self.on_obj, 10)
        self.last_t, self.last_steer = 0.0, 0.0
        self.timer = self.create_timer(0.1, self.watchdog)
        self.stats = {"cmds": 0, "stops": 0}

    def on_obj(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        objs = []
        for d in msg.detections:
            try:
                objs.append({
                    "x": float(d.bbox.center.position.x),
                    "y": float(d.bbox.center.position.y),
                    "z": float(d.bbox.center.position.z),
                    "class_id": parse_cid(d.results[0].hypothesis.class_id),
                })
            except Exception:
                continue
        steer, speed, dbg = plan(objs, self.cfg)
        self.send(steer, speed, dbg.get("decision", ""))
        self.last_t = time.time()
        self.last_steer = steer

    def send(self, steer, speed, why=""):
        steer = max(-self.max_steer, min(self.max_steer, float(steer)))
        speed = max(0.0, min(2.0, float(speed)))
        m = AckermannDriveStamped()
        m.drive.steering_angle = steer
        m.drive.speed = speed
        self.pub.publish(m)
        self.stats["cmds"] += 1
        self.get_logger().info("steer=%+.2f speed=%.2f %s" % (steer, speed, why),
                               throttle_duration_sec=1.0)

    def watchdog(self):
        if time.time() - self.last_t > self.stale:
            self.send(0.0, 0.0, "STALE_STOP")
            self.stats["stops"] += 1


def main():
    a = parse_args()
    rclpy.init()
    node = PlannerNode(load_config(a.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("[STATS]", node.stats, flush=True)


if __name__ == "__main__":
    main()
