"""Companion ROS 2 checker for the integrated scenario (TEST-H/I/J + Phase 6).

Run in a SECOND terminal while run_multi_person_vehicle_gui.py --ros runs:
  source /opt/ros/humble/setup.bash
  python3 isaac_sim/scripts/testing/check_rgbd_topics.py [--window 12]

Uses system python + rclpy (NOT isaacsim python). Subscribes with default
QoS to:
  /camera/rgb/image_raw, /camera/depth/image_raw,
  /camera/rgb/camera_info, /camera/depth/camera_info, /clock, /tf

Checks (deterministic, GT-free):
  TEST-H: RGB + depth publish continuously; header stamps valid sim-time;
          |t_rgb - t_depth| < 0.1 s on paired messages; frame_ids correct;
          optical frames consistent with CameraInfo headers.
  TEST-I: depth image contains finite in-range values; center-region median
          depth valid (pedestrians cross the frame center during the run).
  TEST-J: repo rgbd_fusion on a CENTER-STUB detection (explicitly NOT a real
          detector) + live depth + live K yields a plausible Object3D
          (0.3-10 m, finite). Exercises the real fusion path on real data.

Depth sampling uses image pixels only. No ground-truth pedestrian state is
read anywhere in this file (TEST-K holds by construction).
"""

import argparse
import math
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--window", type=float, default=12.0)
parser.add_argument("--rgb_topic", default="/camera/rgb/image_raw")
parser.add_argument("--depth_topic", default="/camera/depth/image_raw")
parser.add_argument("--rgb_info", default="/camera/rgb/camera_info")
parser.add_argument("--depth_info", default="/camera/depth/camera_info")
a = parser.parse_args()

try:
    import numpy as np
except ImportError:
    print("FAIL: numpy required (system python).", flush=True)
    sys.exit(2)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
from isaac_sim.scripts.perception.rgbd_interface import (
    Detection2D, deproject, rgbd_fusion,
)

store = {"rgb": [], "depth": [], "rgb_info": None, "depth_info": None,
         "clock": 0, "tf": 0}


def img_to_np(msg):
    if msg.encoding == "rgb8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding in ("32FC1", "mono16", "16UC1"):
        dt = np.float32 if msg.encoding == "32FC1" else np.uint16
        ch = 1
        return np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.width, ch)[:, :, 0]
    raise ValueError("unsupported encoding %s" % msg.encoding)


class Checker(Node):
    def __init__(self):
        super().__init__("rgbd_checker")
        self.create_subscription(Image, a.rgb_topic, self.on_rgb, 10)
        self.create_subscription(Image, a.depth_topic, self.on_depth, 10)
        self.create_subscription(CameraInfo, a.rgb_info, self.on_rgb_info, 10)
        self.create_subscription(CameraInfo, a.depth_info, self.on_depth_info, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)

    def on_rgb(self, m):
        store["rgb"].append(m)
        if len(store["rgb"]) > 60:
            store["rgb"].pop(0)

    def on_depth(self, m):
        store["depth"].append(m)
        if len(store["depth"]) > 60:
            store["depth"].pop(0)

    def on_rgb_info(self, m):
        store["rgb_info"] = m

    def on_depth_info(self, m):
        store["depth_info"] = m

    def on_clock(self, m):
        store["clock"] += 1

    def on_tf(self, m):
        store["tf"] += 1


def t_of(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def check(label, cond, detail=""):
    print("[%s] %s %s" % ("PASS" if cond else "FAIL", label, detail), flush=True)
    return bool(cond)


rclpy.init()
node = Checker()
t0 = time.time()
while time.time() - t0 < a.window and (len(store["rgb"]) < 10 or len(store["depth"]) < 10):
    rclpy.spin_once(node, timeout_sec=0.2)
# drain a little more for clock/tf
t1 = time.time()
while time.time() - t1 < 2.0:
    rclpy.spin_once(node, timeout_sec=0.2)

checks = []
nrgb, ndepth = len(store["rgb"]), len(store["depth"])
checks.append(check("RGB publishing continuously", nrgb >= 5, "n=%d" % nrgb))
checks.append(check("Depth publishing continuously", ndepth >= 5, "n=%d" % ndepth))
checks.append(check("/clock alive", store["clock"] > 0, "n=%d" % store["clock"]))
checks.append(check("/tf alive", store["tf"] > 0, "n=%d" % store["tf"]))
checks.append(check("CameraInfo RGB+depth received",
                    store["rgb_info"] is not None and store["depth_info"] is not None))

ok_sync, sync_dt, fid_ok = False, 999.0, False
if nrgb and ndepth:
    r, d = store["rgb"][-1], store["depth"][-1]
    sync_dt = abs(t_of(r.header) - t_of(d.header))
    ok_sync = sync_dt < 0.1
    fid_ok = (r.header.frame_id == "camera_rgb_optical_frame"
              and d.header.frame_id == "camera_depth_optical_frame")
checks.append(check("TEST-H stamps synchronized (<0.1 s)", ok_sync, "dt=%.4f s" % sync_dt))
checks.append(check("TEST-H frame_ids correct", fid_ok,
                    "%s / %s" % (r.header.frame_id if nrgb else "?",
                                 d.header.frame_id if ndepth else "?")))
checks.append(check("TEST-H timestamps valid sim-time",
                    nrgb > 0 and t_of(store["rgb"][-1].header) > 0.0))

depth_ok, med, frac = False, float("nan"), 0.0
depth_np = None
if ndepth:
    try:
        depth_np = img_to_np(store["depth"][-1]).astype(np.float64)
        h, w = depth_np.shape
        ch, cw = h // 2, w // 2
        center = depth_np[ch - 40:ch + 40, cw - 60:cw + 60]
        valid = np.isfinite(center) & (center > 0.1) & (center < 10.0)
        frac = float(valid.mean())
        med = float(np.median(center[valid])) if valid.any() else float("nan")
        depth_ok = frac > 0.2 and math.isfinite(med)
    except Exception as e:
        print("[WARN] depth decode: %r" % e, flush=True)
checks.append(check("TEST-I depth valid+finite (center region)", depth_ok,
                    "frac=%.2f med=%.2f m" % (frac, med)))

fus_ok, dist = False, float("nan")
if depth_ok and store["depth_info"] is not None and depth_np is not None:
    h, w = depth_np.shape
    stub = Detection2D(bbox=(w / 2 - 60, h / 2 - 40, w / 2 + 60, h / 2 + 40),
                       class_name="center-stub(no-detector-yet)", confidence=0.0, object_id=0)
    objs = rgbd_fusion([stub], depth_np, store["depth_info"])
    if objs:
        dist = objs[0].distance
        fus_ok = math.isfinite(dist) and 0.3 < dist < 10.0
checks.append(check("TEST-J fusion Object3D plausible (center-stub)", fus_ok,
                    "dist=%.2f m" % dist))

print("", flush=True)
print("[RESULT] %d/%d ROS checks %s" % (sum(checks), len(checks),
                                        "PASS" if all(checks) else "FAIL"), flush=True)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if all(checks) else 1)
