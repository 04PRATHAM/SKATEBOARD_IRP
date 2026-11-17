"""YOLO 12-scenario PERCEPTION-ONLY validation — SIM SIDE (LEFT pane).

Runs each configs/scenarios/scenario_*.yaml through the SAME lab + vehicle +
RGB-D camera + ROS bridge used by every other runner. The vehicle is NEVER
commanded (no planner, no /cmd_ackermann, no drive calls); it sits at the
scenario start pose while pedestrians follow their YAML trajectories.

Frame capture / YOLO / overlay happen in a SEPARATE perception_venv process
(validate_yolo_scenarios_ext.py + the repo's yolo_seg_node /
rgbd_fusion_node) because ROS Humble rclpy is Python-3.10-only and cannot
import inside isaacsim Python 3.11. This file therefore contains NO rclpy,
NO ultralytics, NO cv2, NO Camera-sensor reads — only proven sim APIs.

Per scenario it writes:
  <out>/gt_<NN>.json   — camera pose, pedestrian GT tracks (eval only),
                         YAML expectations, wall-time window
The external validator writes ext_<NN>.json + scenario_<NN>.png; an offline
merge produces results.json + REPORT.md.

Single scenario (interactive GUI = LEFT pane):
  /home/ubuntu/isaacsim/python.sh \\
    isaac_sim/scripts/testing/run_yolo_all_scenarios_gui.py \\
    --scenario 02 --gui --duration 12 --out data/yolo_scenario_validation
(RIGHT pane: perception_venv validator with cv2 window — see its docstring.)

Batch (all 12, headless):
  ... --scenario all --headless --duration 10
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser(description="YOLO validation SIM side (no perception code)")
parser.add_argument("--scenario", default="all", help="'all' or comma list like 01,09,12")
parser.add_argument("--duration", type=float, default=10.0)
parser.add_argument("--gui", action="store_true", default=False)
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--out", default="/home/ubuntu/SKATEBOARD_IRP/data/yolo_scenario_validation")
args = parser.parse_args()
headless = args.headless and not args.gui
if args.gui:
    headless = False
os.makedirs(args.out, exist_ok=True)

SCEN_DIR = "/home/ubuntu/SKATEBOARD_IRP/configs/scenarios"
SCEN_ORDER = ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12"]
SCEN_FILES = {
    "01": "scenario_01_stationary_person.yaml",
    "02": "scenario_02_person_center.yaml",
    "03": "scenario_03_person_left.yaml",
    "04": "scenario_04_person_right.yaml",
    "05": "scenario_05_person_crossing_left_to_right.yaml",
    "06": "scenario_06_person_crossing_right_to_left.yaml",
    "07": "scenario_07_person_approaching.yaml",
    "08": "scenario_08_two_people.yaml",
    "09": "scenario_09_person_plus_chair.yaml",
    "10": "scenario_10_person_plus_table.yaml",
    "11": "scenario_11_narrow_corridor.yaml",
    "12": "scenario_12_multiple_objects.yaml",
}
if args.scenario.strip().lower() == "all":
    selected = list(SCEN_ORDER)
else:
    selected = [s.strip().zfill(2) for s in args.scenario.split(",") if s.strip()]
    selected = [s for s in selected if s in SCEN_FILES]

from isaacsim import SimulationApp  # noqa: E402
sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})
for _ in range(3):
    try:
        sim.update()
    except Exception:
        pass

import yaml  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import omni.timeline  # noqa: E402
from pxr import Usd, UsdGeom, Gf  # noqa: E402

from isaac_sim.scripts.utils.world_setup import (  # noqa: E402
    build_world, RGBD_CAMERA_PRIM)
from isaac_sim.scripts.assets.pedestrian import (  # noqa: E402
    create_pedestrian_from_dict as _create_capsule_from_dict)
try:
    from isaac_sim.scripts.assets.realistic_pedestrian import (  # noqa: E402
        create_realistic_pedestrian_from_dict as _create_segmented_from_dict)
    HAS_SEGMENTED = True
except Exception:
    HAS_SEGMENTED = False
    _create_segmented_from_dict = None


def setup_validation_scenario(scen_path):
    """Build world + bridge, place vehicle, spawn segmented pedestrians.

    Mirrors the capsule/segmented tiers of the legacy scenario runner
    WITHOUT importing it (it creates its own SimulationApp at import).
    NO drive commands, NO planner: vehicle stays at its start pose.
    """
    with open(scen_path, "r") as f:
        scenario = yaml.safe_load(f)
    vx, vy, vz, vyaw = scenario.get("vehicle_start_pose", [6.0, -6.0, 0.35, 0.0])
    built = build_world(dynamic=True, with_cameras=False, with_ros=True,
                        start_xy=(vx, vy), glass_has_collision=True,
                        create_world=True)
    stage, world = built["stage"], built["world"]
    articulation, dt = built["articulation"], float(built["sim_cfg"]["physics"]["dt"])
    try:
        articulation.initialize()
    except Exception:
        pass
    vehicle_prim = stage.GetPrimAtPath("/World/Vehicle")
    if vehicle_prim.IsValid():
        xform = UsdGeom.Xformable(vehicle_prim)
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(vx, vy, vz))
            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                op.Set(Gf.Vec3d(0, 0, math.degrees(vyaw)))
        if not any(op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ
                   for op in xform.GetOrderedXformOps()):
            xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, math.degrees(vyaw)))
    chassis_prim = stage.GetPrimAtPath("/World/Vehicle/Chassis")
    if chassis_prim.IsValid():
        cxform = UsdGeom.Xformable(chassis_prim)
        for op in cxform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                op.Set(Gf.Vec3d(0, 0, 0))
    pedestrians, gt_log = [], {}
    for ped_config in scenario.get("pedestrians", []):
        use_seg = bool(ped_config.get("is_realistic", False)
                       or "variant_idx" in ped_config
                       or scenario.get("is_realistic", False))
        ped = None
        if use_seg and HAS_SEGMENTED:
            try:
                ped = _create_segmented_from_dict(stage, ped_config)
                print("  segmented ped %s at %s" % (
                    ped.config.pedestrian_id, ped.config.initial_position), flush=True)
            except Exception as e:
                print("  [WARN] segmented spawn failed: %s" % e, flush=True)
                ped = None
        if ped is None:
            ped = _create_capsule_from_dict(stage, ped_config)
            print("  capsule ped %s at %s" % (
                ped.config.pedestrian_id, ped.config.initial_position), flush=True)
        pedestrians.append(ped)
        gt_log[getattr(ped.config, "pedestrian_id", str(id(ped)))] = []
    world.reset()
    return stage, world, scenario, pedestrians, gt_log, dt


def step_validation_peds(pedestrians, gt_log, dt, sim_time):
    for ped in pedestrians:
        try:
            if ped.is_active() or getattr(ped.state, "value", "") == "idle":
                ped.update(dt, sim_time)
            else:
                try:
                    ped.update(dt, sim_time)
                except Exception:
                    pass
            _pid = ped.config.pedestrian_id
        except Exception as e:
            print("[WARN] ped update failed %s: %s" % (ped, e), flush=True)
            continue
        gt_log[_pid].append(ped.get_ground_truth())


def camera_pose(stage):
    prim = stage.GetPrimAtPath(RGBD_CAMERA_PRIM)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    eye = m.Transform(Gf.Vec3d(0, 0, 0))
    fwd = m.TransformDir(Gf.Vec3d(0, 0, -1))
    return ((float(eye[0]), float(eye[1]), float(eye[2])),
            math.atan2(float(fwd[1]), float(fwd[0])))


for scen_id in selected:
    scen_path = os.path.join(SCEN_DIR, SCEN_FILES[scen_id])
    print("=" * 70, flush=True)
    print("[SCENARIO %s] %s" % (scen_id, SCEN_FILES[scen_id]), flush=True)
    wall_start = time.time()
    try:
        omni.usd.get_context().new_stage()
        for _ in range(2):
            sim.update()
    except Exception as e:
        print("[WARN] new_stage failed: %s" % e, flush=True)
    stage, world, scenario, pedestrians, gt_log, dt = \
        setup_validation_scenario(scen_path)
    try:
        omni.timeline.get_timeline_interface().play()
    except Exception:
        pass
    for _ in range(int(1.0 / dt)):
        step_validation_peds(pedestrians, gt_log, dt, 0.0)
        world.step(render=True)
    eye, cam_yaw = camera_pose(stage)
    print("[SCENARIO %s] camera eye=(%.2f,%.2f,%.2f) yaw=%.1fdeg peds=%d bridge=ON" % (
        scen_id, eye[0], eye[1], eye[2], math.degrees(cam_yaw), len(pedestrians)), flush=True)
    sim_time, n_steps = 0.0, int(args.duration / dt)
    for step in range(n_steps):
        if not sim.is_running():
            break
        step_validation_peds(pedestrians, gt_log, dt, sim_time)
        world.step(render=True)
        sim_time += dt
        if step % int(2.0 / dt) == 0:
            print("  t=%.1fs" % sim_time, flush=True)
    wall_end = time.time()
    gt_tracks = {}
    for pid, track in gt_log.items():
        gt_tracks[pid] = [{"t": round(float(g.timestamp), 2),
                           "pos": [round(float(g.position[0]), 2),
                                   round(float(g.position[1]), 2),
                                   round(float(g.position[2]), 2)]}
                          for g in track]
    with open(os.path.join(args.out, "gt_%s.json" % scen_id), "w") as f:
        json.dump({"scenario": scenario.get("scenario_name", scen_id),
                   "file": SCEN_FILES[scen_id],
                   "vehicle_start_pose": scenario.get("vehicle_start_pose"),
                   "camera": {"eye": [round(v, 2) for v in eye],
                              "yaw_deg": round(math.degrees(cam_yaw), 1)},
                   "pedestrians": scenario.get("pedestrians", []),
                   "static_obstacles": scenario.get("static_obstacles", []),
                   "wall_window": [wall_start, wall_end],
                   "duration": args.duration,
                   "gt_tracks": gt_tracks}, f, indent=2)
    print("[SCENARIO %s] GT saved (wall %.0f s)" % (scen_id, wall_end - wall_start), flush=True)

print("[DONE] sim side complete", flush=True)
sim.close()
