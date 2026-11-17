"""Closed-loop camera-only autonomy (perception inside sim process).

All perception (YOLO, fusion, planner) runs in the sim's python process,
sharing the sim's ROS 2 context (same DDS participant as the bridge).
No external nodes, no DDS discovery issues.
"""
import argparse
import math
import sys
import os
import json
import time

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/ros2_ws/src/rgbd_perception")

parser = argparse.ArgumentParser(description="Closed-loop camera-only autonomy")
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--gui", action="store_true", default=False)
parser.add_argument("--duration", type=float, default=35.0)
parser.add_argument("--scenario", type=str,
                    default="/home/ubuntu/SKATEBOARD_IRP/scenarios/scenario_autonomy_stop.yaml")
parser.add_argument("--min-travel", type=float, default=2.5)
parser.add_argument("--slow-frac", type=float, default=0.5)
parser.add_argument("--no-expect-steer", dest="expect_steer", action="store_false", default=True)
args = parser.parse_args()
headless = args.headless and not args.gui
if args.gui:
    headless = False

from isaacsim import SimulationApp
sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})
for _ in range(3):
    try: sim.update()
    except Exception: pass

import yaml
import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

from isaac_sim.scripts.utils.world_setup import build_world, RGBD_CAMERA_PRIM
from isaac_sim.scripts.assets.real_human import (
    enable_real_human_extensions, create_real_human,
    save_and_reopen_stage_for_animation, HumanType as _HT,
)
from isaac_sim.scripts.vehicle.ackermann_controller_ros import AckermannROSController
from isaac_sim.scripts.utils.aim import aim_prim
from isaacsim.core.utils.prims import define_prim as _define_prim

import numpy as np
import ultralytics
import yaml

# --------------------------------------------------------------- Config
with open(args.scenario, "r") as f:
    scen = yaml.safe_load(f)
print("[INFO] scenario: %s (AUTONOMY: in-sim perception)" % scen.get("scenario_name"), flush=True)
vx, vy, vz, vyaw = scen.get("vehicle_start_pose", [12.5, 4.5, 0.35, -2.467])

built = build_world(dynamic=True, with_cameras=True, with_ros=False,
                    start_xy=(vx, vy), create_world=False)
stage, vehicle_cfg = built["stage"], built["vehicle_cfg"]
sim_cfg, camera_cfg = built["sim_cfg"], built["camera_cfg"]
dt = float(sim_cfg["physics"]["dt"])

_vp = stage.GetPrimAtPath("/World/Vehicle")
if _vp.IsValid():
    _vx = UsdGeom.Xformable(_vp)
    _ops = _vx.GetOrderedXformOps()
    for _op in _ops:
        if _op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            _op.Set(Gf.Vec3d(vx, vy, vz))
        elif _op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            _op.Set(Gf.Vec3d(0, 0, math.degrees(vyaw)))
    if not any(_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ for _op in _ops):
        _vx.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, math.degrees(vyaw)))

enable_real_human_extensions()
humans = []
for pcfg in scen.get("pedestrians", []):
    h = create_real_human(stage, pcfg)
    humans.append(h)
    print("[INFO] spawned %s variant=%d" % (h.config.pedestrian_id, h.variant_idx), flush=True)

_cam = _define_prim("/World/LabCamera", "Camera")
_cam.GetAttribute("focalLength").Set(24.0)
_cam.GetAttribute("horizontalAperture").Set(36.0)
from isaac_sim.scripts.utils.aim import aim_prim
aim_prim(stage, "/World/LabCamera", Gf.Vec3d(16.0, -10.0, 9.0), Gf.Vec3d(6.0, 0.0, 0.5))

print("[INFO] Exporting + reopening stage...", flush=True)
want = []
for h in humans:
    want.append(getattr(h, "character_prim_path", ""))
    want.append(getattr(h, "_skelroot_path", ""))
stage = save_and_reopen_stage_for_animation(stage, expected_paths=want)

world = World(stage_units_in_meters=1.0,
              physics_dt=float(sim_cfg["physics"]["dt"]),
              rendering_dt=float(sim_cfg["physics"]["dt"]))
art = Articulation("/World/Vehicle/Chassis")
world.scene.add(art)
world.reset()
art.initialize()

from isaac_sim.scripts.bridge.ros2_bridge import setup_ros2_bridge
setup_ros2_bridge(RGBD_CAMERA_PRIM, camera_cfg, sim_cfg, vehicle_path="/World/Vehicle")
for h in humans:
    h.refresh_after_reopen(stage)

import omni.timeline
omni.timeline.get_timeline_interface().play()
for _ in range(10):
    try: sim.update()
    except Exception: pass

for h in humans:
    if h.config.human_type == 1:
        dest = tuple(h.config.destination) if h.config.destination else None
        if dest is not None:
            try: _pp0 = h.get_world_pos()
            except Exception: _pp0 = h.config.position
            _dist = math.hypot(dest[0] - _pp0[0], dest[1] - _pp0[1])
            ok = h.go_walk_straight(velocity=h.config.velocity, distance=_dist)
        else:
            ok = h.go_walk_straight(velocity=h.config.velocity, distance=8.0)
        print("[%s] walk trigger %s" % ("PASS" if ok else "FAIL", h.config.pedestrian_id), flush=True)
    else:
        ok = h.go_idle()
        print("[%s] idle trigger %s" % ("PASS" if ok else "FAIL", h.config.pedestrian_id), flush=True)
    h.update(0.01)

ctrl = AckermannROSController(art, vehicle_cfg, world, timeout=0.5)

_chassis = stage.GetPrimAtPath("/World/Vehicle/Chassis")

def chassis_pose():
    m = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Vehicle/Chassis")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.Transform(Gf.Vec3d(0, 0, 0))
    fwd = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return (float(p[0]), float(p[1]), float(p[2])), math.atan2(float(fwd[1]), float(fwd[0]))

# ---- YOLO detector (in-sim) -----------------------------------------
with open("/home/ubuntu/SKATEBOARD_IRP/configs/perception.yaml") as f:
    _cfg = yaml.safe_load(f)
_pc = _cfg["perception"]
_planner = _cfg["planner"]

_yolo = ultralytics.YOLO(_pc["model_path"])
if getattr(_yolo.model, "task", "") != "segment":
    raise ValueError("model task is not 'segment'")
_names = dict(_yolo.model.names if hasattr(_yolo.model, "names") else _yolo.names)
_conf = float(_pc["confidence_threshold"])
_iou = float(_pc["iou_threshold"])
_imgsz = int(_pc["image_size"])
_half = bool(_pc["half_precision"]) and True  # CUDA available
_target = set(int(c) for c in _pc["target_classes"])
# Canonical application labels via rgbd_perception.config (single mapping layer).
from rgbd_perception.config import application_label as _canonical_label
def _app_label(cid, model_name):
    return _canonical_label(_pc, cid, model_name)
try:
    _yolo.predict(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False, device="cuda:0", half=True, imgsz=640)
except Exception:
    pass

# ROS 2 (in-sim, shares bridge context) ----------------------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose as _ObjHyp3D
from ackermann_msgs.msg import AckermannDriveStamped

rclpy.init()
_ros_node = rclpy.create_node("autonomy_perception")

# Subscribe to RGB
_latest_rgb = {"msg": None, "np": None, "t": 0.0}
_yolo_busy = False
_last_yolo = 0.0
_yolo_period = 1.0 / max(float(_cfg["perception"].get("perception_rate", 10.0)), 0.1)

def _rgb_cb(msg):
    if msg.encoding == "rgb8":
        _latest_rgb["msg"] = msg
        _latest_rgb["np"] = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        _latest_rgb["t"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

_ros_node.create_subscription(Image, "/camera/rgb/image_raw", _rgb_cb, 10)

# Publishers
_pub_det = _ros_node.create_publisher(Detection2DArray, "/perception/detections_2d", 10)
_pub_mask = _ros_node.create_publisher(Image, "/perception/masks", 10)
_pub_obj = _ros_node.create_publisher(Detection3DArray, "/perception/objects_3d", 10)

# YOLO tick
_yolo_busy = False
_last_yolo = 0.0
_yolo_period = 1.0 / max(float(_cfg["perception"].get("perception_rate", 10.0)), 0.1)

def _yolo_tick():
    global _yolo_busy, _last_yolo
    now = time.time()
    if now - _last_yolo < _yolo_period:
        return
    if _yolo_busy or _latest_rgb["np"] is None:
        return
    _yolo_busy = True
    try:
        _last_yolo = now
        rgb = _latest_rgb["np"]
        stamp_s = _latest_rgb["t"]
        frame_id = _latest_rgb["msg"].header.frame_id
        dets = list(_predict(rgb))
        # Publish Detection2DArray
        out = Detection2DArray()
        out.header.stamp.sec = int(stamp_s)
        out.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        out.header.frame_id = frame_id
        mk = np.zeros((rgb.shape[0], rgb.shape[1]), dtype=np.uint8)
        for i, d in enumerate(dets):
            r = Detection2D()
            r.header.stamp = _latest_rgb["msg"].header.stamp
            r.header.frame_id = frame_id
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = "%d:%s" % (d["class_id"], d["class_name"])
            h.hypothesis.score = float(d["confidence"])
            r.results.append(h)
            x1, y1, x2, y2 = d["bbox"]
            r.bbox.center.position.x = float((x1 + x2) / 2.0)
            r.bbox.center.position.y = float((y1 + y2) / 2.0)
            r.bbox.size_x = float(x2 - x1)
            r.bbox.size_y = float(y2 - y1)
            out.detections.append(r)
            if d["mask"] is not None:
                try:
                    mk[np.asarray(d["mask"], dtype=bool)] = i + 1
                except Exception:
                    pass
        _pub_det.publish(out)
        # Mask image
        mi = Image()
        mi.header.stamp = _latest_rgb["msg"].header.stamp
        mi.header.frame_id = frame_id
        mi.height, mi.width, mi.encoding = rgb.shape[0], rgb.shape[1], "mono8"
        mi.step = rgb.shape[1]
        mi.data = mk.tobytes()
        _ros_node.create_publisher(Image, "/perception/masks", 10).publish(mi)
    finally:
        _yolo_busy = False

# Depth subscriber
_latest_depth = {"msg": None, "np": None, "t": 0.0}
_latest_info = None

def _depth_cb(msg):
    if msg.encoding == "32FC1":
        _latest_depth["msg"] = msg
        _latest_depth["np"] = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        _latest_depth["t"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

def _info_cb(msg):
    global _latest_info
    _latest_info = msg

_ros_node.create_subscription(Image, "/camera/depth/image_raw", _depth_cb, 10)
_ros_node.create_subscription(CameraInfo, "/camera/depth/camera_info", _info_cb, 10)

# Publisher for Object3D
_pub_obj = _ros_node.create_publisher(Detection3DArray, "/perception/objects_3d", 10)

# Planner config
_pcfg = _cfg["perception"]
_planner = _cfg["planner"]
_w = float(_planner.get("corridor_width", 1.2)) / 2.0
_zmin = float(_planner.get("lookahead_min", 1.0))
_zmax = float(_planner.get("lookahead_max", 8.0))
_slow_d = float(_planner.get("slow_distance", 4.0))
_stop_d = float(_planner.get("stop_distance", 1.5))
_cruise = float(_planner.get("cruise_speed", 0.8))
_slow_v = float(_planner.get("slow_speed", 0.35))
_max_steer = float(_planner.get("max_steer", 0.6))
_gain = float(_planner.get("steer_gain", 0.5))
_obstacles = set(int(c) for c in _cfg["perception"].get("obstacle_classes", [0]))

def _plan(objs):
    nearest = {"left": float("inf"), "center": float("inf"), "right": float("inf")}
    for o in objs:
        x, y, z = float(o["x"]), float(o["y"]), float(o["z"])
        cid = int(o.get("class_id", -1))
        if _obstacles and cid not in _obstacles:
            continue
        if not (math.isfinite(x) and math.isfinite(z)) or not (_zmin <= z <= _zmax):
            continue
        sector = "center" if abs(x) <= _w else ("left" if x < 0 else "right")
        if z < nearest[sector]:
            nearest[sector] = z
    if nearest["center"] < _stop_d or (
            nearest["left"] < _stop_d and nearest["right"] < _stop_d
            and nearest["center"] < _slow_d):
        return 0.0, 0.0, "STOP_blocked"
    if nearest["center"] >= _slow_d:
        return 0.0, _cruise, "CRUISE_clear"
    if nearest["right"] >= nearest["left"]:
        return -0.3, _slow_v, "SLOW_right"
    else:
        return 0.3, _slow_v, "SLOW_left"

# Fusion + Planner tick
_last_fusion = 0.0
_fusion_period = 1.0 / 10.0

def _fusion_tick():
    global _last_fusion
    now = time.time()
    if now - _last_fusion < _fusion_period:
        return
    _last_fusion = now
    if _latest_depth["np"] is None or _latest_info is None:
        return
    # For now, use bbox-center depth (mask sync later)
    # We need the latest detections - subscribe to our own detections
    pass

# Subscribe to our own detections for fusion
from vision_msgs.msg import Detection2DArray
_latest_dets = {"msg": None, "t": 0.0}
def _det_cb(msg):
    _latest_dets["msg"] = msg
    _latest_dets["t"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
_ros_node.create_subscription(Detection2DArray, "/perception/detections_2d", _det_cb, 10)

# Depth sync tick
_last_fusion = 0.0
_fusion_period = 1.0 / 10.0

def _fusion_and_plan():
    global _last_fusion
    now = time.time()
    if now - _last_fusion < _fusion_period:
        return
    if _latest_depth["np"] is None or _latest_info is None or _latest_dets["msg"] is None:
        return
    d_msg, depth = _latest_depth["msg"], _latest_depth["np"]
    t_dep = d_msg.header.stamp.sec + d_msg.header.stamp.nanosec * 1e-9
    if abs(_latest_dets["t"] - t_dep) > 0.1:
        return
    try:
        K = list(_latest_info.k)
        fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    except Exception:
        return
    H, W = depth.shape
    dets_msg = _latest_dets["msg"]
    objs = []
    for i, r in enumerate(dets_msg.detections):
        try:
            cid = int(r.results[0].hypothesis.class_id.split(":")[0])
        except Exception:
            continue
        u = float(r.bbox.center.position.x)
        v = float(r.bbox.center.position.y)
        ui, vi = int(round(u)), int(round(v))
        z = None
        if 0 <= ui < W and 0 <= vi < H:
            z0 = float(depth[vi, ui])
            if np.isfinite(z0) and _min_d <= z0 <= _max_d:
                z = z0
        if z is None:
            continue
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        objs.append({"x": float(X), "y": float(Y), "z": float(z), "class_id": cid})
    if objs:
        steer, speed, dbg = _plan(objs)
        m = AckermannDriveStamped()
        m.header.stamp.sec = int(time.time())
        m.header.stamp.nanosec = int((time.time() - int(time.time())) * 1e9)
        m.drive.steering_angle = float(steer)
        m.drive.speed = float(speed)
        _pub_cmd.publish(m)
        ctrl.ros_callback(m)
    _pub_obj.publish(Detection3DArray())  # placeholder

_min_d = float(_cfg["perception"].get("min_depth", 0.3))
_max_d = float(_cfg["perception"].get("max_depth", 10.0))

# Cmd publisher
from ackermann_msgs.msg import AckermannDriveStamped
_pub_cmd = _ros_node.create_publisher(AckermannDriveStamped, "/cmd_ackermann", 10)

# ------------------------------------------------------------------ Main
ctrl = AckermannROSController(art, vehicle_cfg, world, timeout=0.5)
_chassis = stage.GetPrimAtPath("/World/Vehicle/Chassis")

def chassis_pose():
    m = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Vehicle/Chassis")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.Transform(Gf.Vec3d(0, 0, 0))
    fwd = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return (float(p[0]), float(p[1]), float(p[2])), math.atan2(float(fwd[1]), float(fwd[0]))

for h in humans:
    if h.config.human_type == 1:
        dest = tuple(h.config.destination) if h.config.destination else None
        if dest is not None:
            try: _pp0 = h.get_world_pos()
            except Exception: _pp0 = h.config.position
            _dist = math.hypot(dest[0] - _pp0[0], dest[1] - _pp0[1])
            ok = h.go_walk_straight(velocity=h.config.velocity, distance=_dist)
        else:
            ok = h.go_walk_straight(velocity=h.config.velocity, distance=8.0)
        print("[%s] walk trigger %s" % ("PASS" if ok else "FAIL", h.config.pedestrian_id), flush=True)
    else:
        ok = h.go_idle()
        print("[%s] idle trigger %s" % ("PASS" if ok else "FAIL", h.config.pedestrian_id), flush=True)
    h.update(0.01)

cmd_trace = []
gt_veh, gt_ped = [], {h.config.pedestrian_id: [] for h in humans}
max_step_jump, prev_p = 0.0, None
max_steps = int(args.duration / dt)

print("[INFO] Starting closed-loop autonomy loop...", flush=True)

for step in range(int(args.duration / dt)):
    if not sim.is_running():
        break
    try:
        rclpy.spin_once(_ros_node, timeout_sec=0.0)
    except Exception:
        pass
    _yolo_tick()
    _fusion_and_plan()
    dfl, dfr = ctrl.update()
    for h in humans:
        try: h.update(dt)
        except Exception: pass
    world.step(render=True)

    (x, y, z), yaw = (
        (lambda m: (m.Transform(Gf.Vec3d(0,0,0)), math.atan2(m.TransformDir(Gf.Vec3d(1,0,0))[1], m.TransformDir(Gf.Vec3d(1,0,0))[0])))
        (UsdGeom.Xformable(stage.GetPrimAtPath("/World/Vehicle/Chassis")).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    )
    if prev_p is not None:
        max_step_jump = max(max_step_jump, math.hypot(x - prev_p[0], y - prev_p[1]))
    prev_p = (x, y)
    cmd_trace.append((step * dt, float(ctrl.target_steer), float(ctrl.target_speed)))
    gt_veh.append((x, y, yaw, step * dt))
    for h in humans:
        try: gt_ped[h.config.pedestrian_id].append((h.get_world_pos(), step * dt))
        except Exception: pass

    if step % int(2.0 / dt) == 0:
        print("  t=%.1fs x=%.2f y=%.2f yaw=%.1f cmd=(%+.2f, %.2f)" % (
            step * dt, x, y, math.degrees(yaw), ctrl.target_steer, ctrl.target_speed), flush=True)

# Post-hoc evaluation
print("=" * 60, flush=True)

def check(label, cond, detail=""):
    print("[%s] %s %s" % ("PASS" if cond else "FAIL", label, detail), flush=True)
    return bool(cond)

checks = []
travel = sum(math.hypot(gt_veh[i+1][0]-gt_veh[i][0], gt_veh[i+1][1]-gt_veh[i][1])
             for i in range(len(gt_veh)-1))
checks.append(check("H1 forward progress (travel>%.1f m)" % args.min_travel,
                    travel > args.min_travel, "%.2f m" % travel))

min_clear, min_t = 1e9, -1.0
for i, (vx_, vy_, _yw, t) in enumerate(gt_veh):
    for pid, tr in gt_ped.items():
        if i < len(tr):
            (px, py, _pz), _pt = tr[i]
            d = math.hypot(px - vx_, py - vy_)
            if d < min_clear:
                min_clear, min_t = d, t
checks.append(check("H3 no contact (min clearance>0.85 m)", min_clear > 0.85,
                    "min=%.2f m at t=%.1f s" % (min_clear, min_t)))

import math as _m
slow_ok, ahead_n = 0, 0
for i, (vx_, vy_, yw, t) in enumerate(gt_veh):
    fx_, fy_ = math.cos(yw), math.sin(yw)
    blocked = False
    for pid, tr in gt_ped.items():
        if i < len(tr):
            (px, py, _pz), _pt = tr[i]
            dx, dy = px - vx_, py - vy_
            zc = dx * math.cos(yw) + dy * math.sin(yw)
            xc = -dx * math.sin(yw) + dy * math.cos(yw)
            if 0.5 < zc < 4.0 and abs(xc) < zc * math.tan(math.radians(35.0)):
                blocked = True
                break
    if blocked:
        ahead_n += 1
        _t, _s, v = cmd_trace[i]
        if v < 0.8 - 1e-9:
            slow_ok += 1
checks.append(check("H2 slows when ped ahead (GT windows)", ahead_n > 20 and slow_ok > args.slow_frac * ahead_n,
                    "windows=%d slowed=%d" % (ahead_n, slow_ok)))

steers = [s for _t, s, _v in cmd_trace]
variety = (max(steers) - min(steers)) if steers else 0.0
no_teleport = max_step_jump < 2.0 * dt * 3 + 0.02
checks.append(check("H4 planner drove (steer variety + no teleport)",
                    variety > 0.2 and no_teleport,
                    "sweep=%.2f jump=%.3f" % (variety, max_step_jump)))

stops = sum(1 for _t, _s, v in cmd_trace if v < 0.05)
checks.append(check("H5 stop capability (zero-speed cmds issued)", stops > 0, "n=%d" % stops))

print("", flush=True)
print("[RESULT] %d/%d closed-loop checks %s" % (
    sum(checks), len(checks), "PASS" if all(checks) else "FAIL"), flush=True)

if not headless:
    print("[INFO] GUI hold-open...", flush=True)
    try:
        while sim.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass

_ros_node.destroy_node()
rclpy.shutdown()
sim.close()
sys.exit(0 if all(checks) else 1)

# ------------------------------------------------------------ YOLO predict
def _predict(rgb):
    res = _yolo.predict(rgb, verbose=False, device="cuda:0", half=True, imgsz=640,
                        conf=0.35, iou=0.7)[0]
    H, W = rgb.shape[:2]
    dets = []
    boxes = getattr(res, "boxes", None)
    masks = getattr(res, "masks", None)
    mdata = None
    if masks is not None and getattr(masks, "data", None) is not None:
        import torch
        mdata = masks.data.detach().to("cpu").float()
        mdata = torch.nn.functional.interpolate(
            mdata.unsqueeze(0), size=(H, W), mode="nearest").squeeze(0) > 0.5
        mdata = mdata.numpy()
    if boxes is not None and len(boxes) > 0:
        xyxy = boxes.xyxy.detach().to("cpu").numpy()
        cls = boxes.cls.detach().to("cpu").numpy().astype(int)
        cnf = boxes.conf.detach().to("cpu").numpy()
        for i in range(len(xyxy)):
            cid = int(cls[i])
            if _target and cid not in _target:
                continue
            x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
            mk = None
            if mdata is not None and i < len(mdata):
                try:
                    mk = np.asarray(mdata[i], dtype=bool)
                except Exception:
                    mk = None
            yield {
                "bbox": (x1, y1, x2, y2),
                "class_id": cid,
                # Canonical application label (60 -> "table"); IDs unchanged.
                "class_name": _app_label(cid, str(_names.get(cid, "class_%d" % cid))),
                "confidence": float(cnf[i]),
                "mask": mk,
            }

# ------------------------------------------------------------ YOLO tick
_yolo_busy = False
_last_yolo = 0.0
_yolo_period = 1.0 / max(float(_cfg["perception"].get("perception_rate", 10.0)), 0.1)

def _yolo_tick():
    global _yolo_busy, _last_yolo
    now = time.time()
    if now - _last_yolo < _yolo_period:
        return
    if _yolo_busy or _latest_rgb["np"] is None:
        return
    _yolo_busy = True
    try:
        _last_yolo = now
        rgb = _latest_rgb["np"]
        stamp_s = _latest_rgb["t"]
        frame_id = _latest_rgb["msg"].header.frame_id
        dets = list(_predict(rgb))
        out = Detection2DArray()
        out.header.stamp.sec = int(stamp_s)
        out.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        out.header.frame_id = frame_id
        mk = np.zeros((rgb.shape[0], rgb.shape[1]), dtype=np.uint8)
        for i, d in enumerate(dets):
            r = Detection2D()
            r.header.stamp = _latest_rgb["msg"].header.stamp
            r.header.frame_id = frame_id
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = "%d:%s" % (d["class_id"], d["class_name"])
            h.hypothesis.score = float(d["confidence"])
            r.results.append(h)
            x1, y1, x2, y2 = d["bbox"]
            r.bbox.center.position.x = float((x1 + x2) / 2.0)
            r.bbox.center.position.y = float((y1 + y2) / 2.0)
            r.bbox.size_x = float(x2 - x1)
            r.bbox.size_y = float(y2 - y1)
            out.detections.append(r)
            if d["mask"] is not None:
                try:
                    mk[np.asarray(d["mask"], dtype=bool)] = i + 1
                except Exception:
                    pass
        _pub_det.publish(out)
        mi = Image()
        mi.header.stamp = _latest_rgb["msg"].header.stamp
        mi.header.frame_id = frame_id
        mi.height, mi.width, mi.encoding = rgb.shape[0], rgb.shape[1], "mono8"
        mi.step = rgb.shape[1]
        mi.data = mk.tobytes()
        _ros_node.create_publisher(Image, "/perception/masks", 10).publish(mi)
    finally:
        _yolo_busy = False

_min_d = float(_cfg["perception"].get("min_depth", 0.3))
_max_d = float(_cfg["perception"].get("max_depth", 10.0))
_zmin = float(_planner.get("lookahead_min", 1.0))
_zmax = float(_planner.get("lookahead_max", 8.0))
_slow_d = float(_planner.get("slow_distance", 4.0))
_stop_d = float(_planner.get("stop_distance", 1.5))
_cruise = float(_planner.get("cruise_speed", 0.8))
_slow_v = float(_planner.get("slow_speed", 0.35))
_max_steer = float(_planner.get("max_steer", 0.6))
_gain = float(_planner.get("steer_gain", 0.5))
_obstacles = set(int(c) for c in _cfg["perception"].get("obstacle_classes", [0]))
_cruise = float(_planner.get("cruise_speed", 0.8))
_slow_v = float(_planner.get("slow_speed", 0.35))
_max_steer = float(_planner.get("max_steer", 0.6))
_gain = float(_planner.get("steer_gain", 0.5))
_obstacles = set(int(c) for c in _cfg["perception"].get("obstacle_classes", [0]))

if not headless:
    print("[INFO] GUI hold-open...", flush=True)
    try:
        while sim.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass

_ros_node.destroy_node()
rclpy.shutdown()
sim.close()
sys.exit(0 if all(checks) else 1)

