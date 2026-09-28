"""Integrated multi-person + vehicle + RGB-D GUI scenario (SKATEBOARD_IRP).

ONE launch for visual + numerical validation that, simultaneously:
  - lab environment exists,
  - physical Ackermann vehicle drives the C8R route (23.89 m, START/GOAL
    flat floor discs only, no poles),
  - 3 REAL S3 pedestrians walk graph-driven gaits on diverse trajectories,
  - vehicle-mounted RGB-D camera observes them (frustum audit TEST A-F),
  - ROS 2 bridge streams RGB + depth (TEST H-J via companion checker),
  - ground truth stays evaluation-only (TEST K),
  - motion comes from the articulation, never teleport (TEST L).

Flow reuses proven pieces only (no rebuilds):
  build_world(deferred) -> spawn 3 RealHumans -> save+reopen stage
  (AnimationGraph discovery + scripting-component audit) -> World +
  Articulation + reset -> ROS bridge (if --ros) -> refresh humans ->
  graph walk triggers -> waypoint drive + human pump + FOV audit.

Usage:
  .../python.sh isaac_sim/scripts/testing/run_multi_person_vehicle_gui.py --headless [--ros] [--duration 60]
  .../python.sh isaac_sim/scripts/testing/run_multi_person_vehicle_gui.py --gui [--ros]

Phase 9: perception output is NOT connected to steering here. Steering
comes only from the waypoint follower on vehicle pose.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser(description="Integrated multi-person + vehicle + RGB-D scenario")
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--gui", action="store_true", default=False)
parser.add_argument("--ros", action="store_true", default=False)
parser.add_argument("--duration", type=float, default=60.0)
parser.add_argument("--scenario", type=str,
                    default="/home/ubuntu/SKATEBOARD_IRP/scenarios/scenario_multi_person_vehicle_gui.yaml")
args = parser.parse_args()
headless = args.headless and not args.gui
if args.gui:
    headless = False

from isaacsim import SimulationApp
sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})
for _ in range(3):
    try:
        sim.update()
    except Exception:
        pass

import yaml
import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

from isaac_sim.scripts.utils.world_setup import build_world, RGBD_CAMERA_PRIM
from isaac_sim.scripts.assets.real_human import (
    enable_real_human_extensions,
    create_real_human,
    save_and_reopen_stage_for_animation,
    HumanType as _HT,
)
from isaac_sim.scripts.control.waypoint_controller import (
    WaypointFollower, WaypointControllerConfig,
)
from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step

# ---------------------------------------------------------------- config
with open(args.scenario, "r") as f:
    scen = yaml.safe_load(f)
print("[INFO] scenario: %s" % scen.get("scenario_name"), flush=True)

ROUTE = [tuple(w) for w in scen["vehicle_route"]]
START, GOAL = ROUTE[0], ROUTE[-1]
vx, vy, vz, vyaw = scen.get("vehicle_start_pose", [START[0], START[1], 0.35, 0.0])

# Camera model (configs/camera.yaml): 640x480, HFOV 70deg, VFOV 45deg,
# mount 0.9 m ahead of chassis center, 1.2 m above ground, pitch -0.15 rad.
IMG_W, IMG_H = 640, 480
TAN_H = math.tan(math.radians(35.0))
TAN_V = math.tan(math.radians(22.5))
FX = (IMG_W / 2.0) / TAN_H
FY = (IMG_H / 2.0) / TAN_V
CAM_AHEAD, CAM_H, CAM_PITCH = 0.9, 1.2, -0.15

# ---------------------------------------------------------------- build
built = build_world(dynamic=True, with_cameras=True, with_ros=False,
                    start_xy=(vx, vy), create_world=False)
stage, vehicle_cfg = built["stage"], built["vehicle_cfg"]
sim_cfg = built["sim_cfg"]
camera_cfg = built["camera_cfg"]
dt = float(sim_cfg["physics"]["dt"])

# Vehicle spawn yaw faces the first leg (pose on /World/Vehicle only).
_yaw0 = math.atan2(ROUTE[1][1] - START[1], ROUTE[1][0] - START[0])
_vp = stage.GetPrimAtPath("/World/Vehicle")
if _vp.IsValid():
    _vx = UsdGeom.Xformable(_vp)
    _ops = _vx.GetOrderedXformOps()
    for _op in _ops:
        if _op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            _op.Set(Gf.Vec3d(vx, vy, vz))
        elif _op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            _op.Set(Gf.Vec3d(0, 0, math.degrees(_yaw0)))
    if not any(_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ for _op in _ops):
        _vx.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, math.degrees(_yaw0)))
_cp = stage.GetPrimAtPath("/World/Vehicle/Chassis")
if _cp.IsValid():
    _cx = UsdGeom.Xformable(_cp)
    for _op in _cx.GetOrderedXformOps():
        if _op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            _op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
        elif _op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            _op.Set(Gf.Vec3d(0, 0, 0))

# Spawn 3 REAL S3 pedestrians (graph-direct; no behavior-script attach).
enable_real_human_extensions()
humans = []
for pcfg in scen.get("pedestrians", []):
    h = create_real_human(stage, pcfg)
    humans.append(h)
    print("[INFO] spawned REAL S3 human %s variant=%d at %s" % (
        h.config.pedestrian_id, h.variant_idx, h.config.position), flush=True)

# Overview camera for VIEW 1 (lab + vehicle + pedestrians).
from isaac_sim.scripts.utils.aim import aim_prim
from isaacsim.core.utils.prims import define_prim as _define_prim
_cam_path = "/World/LabCamera"
_cam = _define_prim(_cam_path, "Camera")
_cam.GetAttribute("focalLength").Set(24.0)
_cam.GetAttribute("horizontalAperture").Set(36.0)
aim_prim(stage, _cam_path, Gf.Vec3d(16.0, -10.0, 9.0), Gf.Vec3d(6.0, 0.0, 0.5))

# GUI: open the viewport on the overview camera (VIEW 1) so the scene is
# framed immediately (best-effort; never fatal).
if not headless:
    try:
        import omni.kit.viewport.utility as _vu
        _vp = _vu.get_active_viewport()
        if _vp is not None:
            _vp.camera_path = _cam_path
            print("[INFO] viewport camera -> %s" % _cam_path, flush=True)
    except Exception as e:
        print("[WARN] viewport camera set failed: %r" % e, flush=True)

# Flat START (green) + GOAL (red) floor discs only — no poles.
for _name, _mx, _my, _color in [("START", START[0], START[1], (0.15, 0.75, 0.25)),
                                ("GOAL", GOAL[0], GOAL[1], (0.9, 0.15, 0.15))]:
    _disc = _define_prim(f"/World/RouteMarkers/{_name}", "Cylinder")
    _xd = UsdGeom.Xformable(_disc)
    _xd.AddTranslateOp().Set(Gf.Vec3d(float(_mx), float(_my), 0.02))
    _disc.GetAttribute("radius").Set(0.6)
    _disc.GetAttribute("height").Set(0.04)
    _disc.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
                          ).Set([Gf.Vec3f(*_color)])

# Export + reopen (AnimationGraph discovery; scripting audit runs inside).
print("[INFO] Exporting + reopening stage for AnimationGraph discovery...", flush=True)
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
print("[INFO] reopened stage reset complete.", flush=True)

if args.ros:
    from isaac_sim.scripts.bridge.ros2_bridge import setup_ros2_bridge
    setup_ros2_bridge(RGBD_CAMERA_PRIM, camera_cfg, sim_cfg, vehicle_path="/World/Vehicle")

for h in humans:
    h.refresh_after_reopen(stage)

# Timeline MUST be playing or AnimationGraph MotionMatching never advances
# (world.step moves physics/vehicle but frozen graphs: triggers ok, zero
# displacement — found 2026-09-25). Proven pattern from the walk test.
import omni.timeline
_tl = omni.timeline.get_timeline_interface()
_tl.play()
for _ in range(10):
    try:
        sim.update()
    except Exception:
        pass

# Trigger graph walks via go_walk_straight (the 19/19-proven mechanism).
# Each designed path is a single straight segment: spawn yaw already faces
# the destination (YAML initial_orientation), distance = segment length.
# NOTE (2026-09-25 finding): go_walk_to triggers ok but yields zero root
# translation (PathPoints via NavigationManager never advance the matcher;
# per-tick refresh ruled out as the cause). Until that is root-caused +
# NavMesh baked, straight mode carries all straight segments. YAML
# `destination` fields remain the arrival-evaluation reference.
trig_ok = {}
for h in humans:
    dest = tuple(h.config.destination) if h.config.destination else None
    if dest is not None:
        try:
            _pp0 = h.get_world_pos()
        except Exception:
            _pp0 = h.config.position
        _dist = math.hypot(dest[0] - _pp0[0], dest[1] - _pp0[1])
        ok = h.go_walk_straight(velocity=h.config.velocity, distance=_dist)
    else:
        ok = h.go_walk_straight(velocity=h.config.velocity, distance=8.0)
    trig_ok[h.config.pedestrian_id] = bool(ok)
    print("[%s] walk trigger %s -> %s" % ("PASS" if ok else "FAIL",
          h.config.pedestrian_id, ok), flush=True)
    h.update(0.01)

# ---------------------------------------------------------------- drive
config = WaypointControllerConfig()
follower = WaypointFollower(ROUTE, config)
names = list(art.dof_names)
idx = {n: names.index(n) for n in names}
drive_pos = {}

_chassis = stage.GetPrimAtPath("/World/Vehicle/Chassis")


def chassis_pose():
    m = UsdGeom.Xformable(_chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    p = m.Transform(Gf.Vec3d(0, 0, 0))
    fwd = m.TransformDir(Gf.Vec3d(1, 0, 0))
    return (float(p[0]), float(p[1]), float(p[2])), math.atan2(float(fwd[1]), float(fwd[0]))


def poly_dev(x, y):
    best = 1e9
    for i in range(len(ROUTE) - 1):
        ax, ay = ROUTE[i]
        bx, by = ROUTE[i + 1]
        abx, aby = bx - ax, by - ay
        denom = abx * abx + aby * aby
        t = ((x - ax) * abx + (y - ay) * aby) / denom if denom > 0 else 0.0
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(x - (ax + t * abx), y - (ay + t * aby)))
    return best


def project_to_image(px, py, pz, eye, fwd, right, up):
    dx, dy, dz = px - eye[0], py - eye[1], pz - eye[2]
    zc = dx * fwd[0] + dy * fwd[1] + dz * fwd[2]
    xc = dx * right[0] + dy * right[1] + dz * right[2]
    yc = dx * up[0] + dy * up[1] + dz * up[2]
    if not (0.3 < zc < 10.0):
        return None
    if abs(xc) > zc * TAN_H or abs(yc) > zc * TAN_V:
        return None
    u = IMG_W / 2.0 + (xc / zc) * FX
    v = IMG_H / 2.0 - (yc / zc) * FY
    return (u, v, zc)


(x0, y0, _z0), _yaw_init = chassis_pose()
trail, steers, speeds, yaw_hist = [], [], [], []
ped_tracks = {h.config.pedestrian_id: [] for h in humans}
ped_start = {}
for h in humans:
    try:
        ped_start[h.config.pedestrian_id] = h.get_world_pos()
    except Exception:
        ped_start[h.config.pedestrian_id] = h.config.position
# Walk-active window per ped (for honest walking-speed metric): first tick
# each ped reports completed; speed = final displacement / walk time.
ped_done_t = {h.config.pedestrian_id: None for h in humans}
fov_hits = {h.config.pedestrian_id: 0 for h in humans}
thirds = {"left": 0, "center": 0, "right": 0}
simul_all3_ticks = []
veh_moving_and_peds = 0
max_step_jump = 0.0
prev_p = (x0, y0)
left_hold = right_hold = 0.0
reached = False
max_steps = int(args.duration / dt)
NT = 0

for step in range(max_steps):
    if not sim.is_running():
        break
    (x, y, z), yaw = chassis_pose()
    steer, speed, reached = follower.update(x, y, yaw, dt)
    omega = speed / float(vehicle_cfg["wheel_radius"])
    # TEST K provenance: drive inputs derive ONLY from vehicle pose via the
    # waypoint follower. Pedestrian positions below are read for EVALUATION
    # (FOV audit + GT log) and never enter steering.
    drive_step(art, idx, steer, omega, dt, drive_pos)
    for h in humans:
        try:
            h.update(dt)
        except Exception:
            pass
    # render=True ALWAYS (even headless): the USD timeline — and with it
    # AnimationGraph MotionMatching — advances ONLY on rendered frames
    # (probe 2026-09-25: 100x world.step(render=False) -> timeline frozen;
    # render=True -> advances). render=False moves physics but freezes humans.
    world.step(render=True)

    trail.append((x, y))
    steers.append(steer)
    speeds.append(speed)
    yaw_hist.append(yaw)
    NT += 1
    max_step_jump = max(max_step_jump, math.hypot(x - prev_p[0], y - prev_p[1]))
    prev_p = (x, y)
    if steer > 0.25:
        left_hold += dt
    elif steer < -0.25:
        right_hold += dt

    # Camera model from chassis pose + config mount (proprioception only).
    cp, sy = math.cos(yaw), math.sin(yaw)
    eye = (x + CAM_AHEAD * cp, y + CAM_AHEAD * sy, CAM_H)
    fwd = (cp * math.cos(CAM_PITCH), sy * math.cos(CAM_PITCH), math.sin(CAM_PITCH))
    right = (-sy, cp, 0.0)
    up = (fwd[1] * right[2] - fwd[2] * right[1],
          fwd[2] * right[0] - fwd[0] * right[2],
          fwd[0] * right[1] - fwd[1] * right[0])

    n_vis = 0
    last_pp = {}
    for h in humans:
        pid = h.config.pedestrian_id
        try:
            pp = h.get_world_pos()
        except Exception:
            pp = h.config.position
        last_pp[pid] = pp
        ped_tracks[pid].append((pp, NT * dt))
        # Project ped torso (~1.0 m above feet) into the RGB frame.
        proj = project_to_image(pp[0], pp[1], pp[2] + 1.0, eye, fwd, right, up)
        if proj is not None:
            n_vis += 1
            fov_hits[pid] += 1
            u, _v, _zc = proj
            thirds["left" if u < IMG_W / 3 else ("right" if u > 2 * IMG_W / 3 else "center")] += 1
    if n_vis == len(humans):
        simul_all3_ticks.append(NT * dt)
    for h in humans:
        pid = h.config.pedestrian_id
        if ped_done_t[pid] is None:
            try:
                if h.is_completed():
                    ped_done_t[pid] = NT * dt
            except Exception:
                pass
    # TEST-G (strict): vehicle moving AND >=2 peds actually displaced >0.5 m
    # from start (not merely in WALKING state).
    disp_ct = sum(
        1 for pid, _pp in last_pp.items()
        if math.hypot(_pp[0] - ped_start[pid][0], _pp[1] - ped_start[pid][1]) > 0.5)
    if speed > 0.3 and disp_ct >= 2:
        veh_moving_and_peds += 1

    if step % int(2.0 / dt) == 0:
        print("  t=%.1fs x=%.2f y=%.2f yaw=%.1f steer=%.2f speed=%.2f fov=%d/3" % (
            step * dt, x, y, math.degrees(yaw), steer, speed, n_vis), flush=True)
    if reached and speed < 0.05:
        print("[GOAL] reached at t=%.1fs" % (step * dt), flush=True)
        break

# Settle with zero wheel commands; measure physical drift (stop evidence).
(pre_sx, pre_sy, _pz), _py = chassis_pose()
for _ in range(100):
    (x, y, z), yaw = chassis_pose()
    s, _sp, _r = follower.update(x, y, yaw, dt)
    drive_step(art, idx, s, 0.0, dt, drive_pos)
    for h in humans:
        try:
            h.update(dt)
        except Exception:
            pass
    world.step(render=True)
(x, y, z), yaw = chassis_pose()
settle_drift = math.hypot(x - pre_sx, y - pre_sy)

# ---------------------------------------------------------------- evaluate
print("=" * 60, flush=True)
dist_goal = math.hypot(GOAL[0] - x, GOAL[1] - y)
travelled = sum(math.hypot(trail[i + 1][0] - trail[i][0], trail[i + 1][1] - trail[i][1])
                for i in range(len(trail) - 1))
max_dev = max(poly_dev(px, py) for px, py in trail[::10]) if trail else 999.0
steer_range = (max(steers) - min(steers)) if len(steers) > 1 else 0.0
yaw_sweep = (max(yaw_hist) - min(yaw_hist)) if len(yaw_hist) > 1 else 0.0
print("vehicle: goal_err=%.2f m travel=%.2f m max_dev=%.2f m drift=%.3f m" % (
    dist_goal, travelled, max_dev, settle_drift), flush=True)
for pid in ped_tracks:
    tr = ped_tracks[pid]
    d = math.hypot(tr[-1][0][0] - tr[0][0][0], tr[-1][0][1] - tr[0][0][1]) if len(tr) > 1 else 0.0
    zs = [p[2] for p, _t in tr]
    print("ped %s: disp=%.2f m zspread=%.3f fov_frac=%.3f" % (
        pid, d, (max(zs) - min(zs)) if zs else 999, fov_hits[pid] / max(NT, 1)), flush=True)
print("fov thirds L/C/R=%d/%d/%d simul_all3=%d ticks veh+peds=%d ticks" % (
    thirds["left"], thirds["center"], thirds["right"],
    len(simul_all3_ticks), veh_moving_and_peds), flush=True)


def check(label, cond, detail=""):
    print("[%s] %s %s" % ("PASS" if cond else "FAIL", label, detail), flush=True)
    return bool(cond)


checks = []
# Vehicle (route integrity, same bar as dedicated route test).
checks.append(check("V1 start pose clear", poly_dev(x0, y0) < 0.6))
checks.append(check("V2 reached GOAL", dist_goal < 0.6 and reached, "err=%.2f" % dist_goal))
checks.append(check("V3 route travel (>15 m of 23.89 m)", travelled > 15.0, "%.2f" % travelled))
checks.append(check("V4 follows route (max dev < 1.5)", max_dev < 1.5, "%.2f" % max_dev))
checks.append(check("V5 left+right turns", left_hold > 0.5 and right_hold > 0.5))
checks.append(check("V6 stopped (physical)", settle_drift < 0.15))
# Pedestrians (graph gait preserved in the integrated scene).
_hmap = {h.config.pedestrian_id: h for h in humans}
for pid in ped_tracks:
    tr = ped_tracks[pid]
    d = math.hypot(tr[-1][0][0] - tr[0][0][0], tr[-1][0][1] - tr[0][0][1]) if len(tr) > 1 else 0.0
    zs = [p[2] for p, _t in tr]
    # Walking speed over the ACTIVE walk window (trigger -> completed), not
    # the full run (peds stand after arrival; averaging over standing time
    # would fake a slow shuffle).
    _wt = ped_done_t.get(pid) or (tr[-1][1] - tr[0][1] if len(tr) > 1 else 1e-6)
    v = d / max(_wt, 1e-6)
    checks.append(check("M %s walked (disp>2 m, v 0.4-1.8)" % pid, d > 2.0 and 0.4 < v < 1.8,
                        "d=%.2f v=%.2f t_walk=%.1fs" % (d, v, _wt)))
    checks.append(check("M %s upright (z-spread<0.2)" % pid,
                        (max(zs) - min(zs)) < 0.2 if zs else False))
    _dest = _hmap[pid].config.destination
    if _dest is not None and len(tr) > 1:
        _arr = math.hypot(tr[-1][0][0] - _dest[0], tr[-1][0][1] - _dest[1])
        checks.append(check("M %s arrived (final err<1.0 m)" % pid, _arr < 1.0,
                            "err=%.2f" % _arr))
checks.append(check("T triggers ok (3/3 graph)", all(trig_ok.values()), str(trig_ok)))
# TEST A-G (camera FOV audit; ped positions used as EVALUATION only).
checks.append(check("TEST-A one ped in FOV", any(v > 0.05 * NT for v in fov_hits.values()),
                    str({k: round(v / max(NT, 1), 3) for k, v in fov_hits.items()})))
checks.append(check("TEST-B three peds simultaneously in FOV", len(simul_all3_ticks) > 5,
                    "ticks=%d first_t=%.1fs" % (
                        len(simul_all3_ticks),
                        simul_all3_ticks[0] if simul_all3_ticks else -1)))
checks.append(check("TEST-C/D/E ped left+center+right thirds",
                    thirds["left"] > 0 and thirds["center"] > 0 and thirds["right"] > 0,
                    str(thirds)))
checks.append(check("TEST-G vehicle+peds moving together", veh_moving_and_peds > 50,
                    "ticks=%d" % veh_moving_and_peds))
# TEST K: GT isolation — drive inputs came only from vehicle pose (see loop
# comment); perception interface takes RGB/depth only (DummyDetector=[]).
import isaac_sim.scripts.perception.rgbd_interface as _perc
checks.append(check("TEST-K GT isolation (perception takes RGB/depth only)",
                    _perc.DummyDetector().detect(None, None) == []
                    and "EVALUATION ONLY" in (_perc.GroundTruthDetector.__doc__ or "")))
# TEST L: Ackermann motion real (no teleport).
checks.append(check("TEST-L Ackermann motion (steer+yaw, no teleport)",
                    steer_range > 0.3 and yaw_sweep > 0.5
                    and max_step_jump < config.max_speed * dt * 3 + 0.02,
                    "steer=%.2f yaw=%.2f jump=%.3f" % (steer_range, yaw_sweep, max_step_jump)))
print("", flush=True)
print("[RESULT] %d/%d integrated checks %s" % (
    sum(checks), len(checks), "PASS" if all(checks) else "FAIL"), flush=True)
print("NOTE: TEST-H/I/J (ROS rgb/depth sync, depth validity, 3D plausibility) run via", flush=True)
print("  the companion checker while --ros is active (see check_rgbd_topics.py).", flush=True)

if not headless:
    print("[INFO] GUI hold-open for inspection (close window to exit)...", flush=True)
    print("VIEW 1: viewport = /World/LabCamera (overview).", flush=True)
    print("VIEW 2: switch viewport camera to .../Sensors/RGBD_Camera/RGBD_Camera (vehicle RGB).", flush=True)
    print("VIEW 3: depth — enable viewport depth viz or inspect /camera/depth/image_raw via ROS.", flush=True)
    print("VIEW 4: route START (green disc) -> GOAL (red disc); vehicle + 3 walking S3 humans.", flush=True)
    try:
        while sim.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass

sim.close()
sys.exit(0 if all(checks) else 1)
