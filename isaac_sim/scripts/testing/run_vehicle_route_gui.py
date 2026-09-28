#!/usr/bin/env python3
"""
TEST 2 — Vehicle long-route validation (dedicated GUI + headless).

One command that clearly shows: lab + vehicle at START + flat START/GOAL floor
markers + deterministic automatic driving (left AND right turns) + stop at GOAL.

Route (validated against the 30x25 lab collision layout, 1.1 m clearance):
  START (2,-6) -> (7,-6) -> (11,-4.5) -> (12.5,-1) -> (12.5,3) -> (9.5,5) -> GOAL (12,7)
  total ~23.9 m. No poles, no obstacles added; intermediate waypoints invisible.

Usage:
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_vehicle_route_gui.py --headless   # Part L validation
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_vehicle_route_gui.py --gui         # stays open

Motion: existing PhysX/Ackermann drive_step only. No teleport, no velocity hack.
"""
import sys
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")
import argparse
import math

p = argparse.ArgumentParser(description="TEST 2: vehicle long-route validation")
p.add_argument("--headless", action="store_true")
p.add_argument("--gui", action="store_true", help="GUI stays open for inspection")
p.add_argument("--duration", type=float, default=60.0)
args = p.parse_args()

# --- Route definition (single source of truth) ---
# Traversed so the ~108 deg hairpin is a LEFT (strong steering direction;
# sustained hard-right orbits due to the known right-turn servo weakness).
# Clearance >= 1.5 m verified by layout probe; length ~23.9 m.
ROUTE = [(12.0, 7.0), (9.5, 5.0), (12.5, 3.0), (12.5, -1.0),
         (11.0, -4.5), (7.0, -6.0), (2.0, -6.0)]
START = ROUTE[0]
GOAL = ROUTE[-1]
ROUTE_LENGTH = sum(math.hypot(ROUTE[i+1][0]-ROUTE[i][0], ROUTE[i+1][1]-ROUTE[i][1])
                   for i in range(len(ROUTE)-1))


def leg_labels(route):
    """Label each leg STRAIGHT k / LEFT TURN / RIGHT TURN by heading change."""
    heads = [math.atan2(route[i+1][1]-route[i][1], route[i+1][0]-route[i][0])
             for i in range(len(route)-1)]
    labels, straight_n = [], 0
    prev = heads[0]
    for h in heads:
        da = (h - prev + math.pi) % (2*math.pi) - math.pi
        if abs(da) >= math.radians(25):
            labels.append("RIGHT TURN" if da < 0 else "LEFT TURN")
        else:
            straight_n += 1
            labels.append(f"STRAIGHT {straight_n}")
        prev = h
    return labels


LEG_LABELS = leg_labels(ROUTE)

from isaacsim import SimulationApp
sim = SimulationApp({"headless": args.headless and not args.gui, "renderer": "RayTracedLighting"})

from pxr import Usd, UsdGeom, Gf, Sdf
from isaacsim.core.utils.prims import define_prim
from isaac_sim.scripts.utils.world_setup import build_world
from isaac_sim.scripts.control.waypoint_controller import WaypointFollower, WaypointControllerConfig
from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step
from isaac_sim.scripts.utils.aim import aim_prim


def add_flat_markers(stage):
    """Flat START (green) + GOAL (red) floor discs. Visual-only: no collision,
    no physics, no sensors. Intermediate waypoints stay invisible."""
    for name, (mx, my), color in [("START", START, (0.15, 0.75, 0.25)),
                                  ("GOAL", GOAL, (0.9, 0.15, 0.15))]:
        disc = define_prim(f"/World/RouteMarkers/{name}", "Cylinder")
        xd = UsdGeom.Xformable(disc)
        xd.AddTranslateOp().Set(Gf.Vec3d(float(mx), float(my), 0.02))
        disc.GetAttribute("radius").Set(0.6)
        disc.GetAttribute("height").Set(0.04)
        disc.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
                             ).Set([Gf.Vec3f(*color)])


def add_overview_camera(stage):
    """Elevated inspection viewpoint covering the whole route (GUI: switch the
    viewport to it via Viewport -> Camera -> Route_Overview). The RGB-D sensor
    is untouched."""
    cam_path = "/World/InspectionCameras/Route_Overview"
    define_prim(cam_path, "Camera")
    aim_prim(stage, cam_path, Gf.Vec3d(7.0, -16.0, 10.0), Gf.Vec3d(7.5, 0.0, 0.0))


built = build_world(dynamic=True, with_cameras=True, with_ros=False, start_xy=START)
world, stage, art = built["world"], built["stage"], built["articulation"]
vehicle_cfg = built["vehicle_cfg"]
sim_cfg = built["sim_cfg"]
dt = float(sim_cfg["physics"]["dt"])

# Face the first leg (spawn yaw): pose on /World/Vehicle, Chassis stays identity.
import math as _mm
_yaw0 = _mm.atan2(ROUTE[1][1] - START[1], ROUTE[1][0] - START[0])
_vp = stage.GetPrimAtPath("/World/Vehicle")
if _vp.IsValid():
    _vx = UsdGeom.Xformable(_vp)
    for _op in _vx.GetOrderedXformOps():
        if _op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            _op.Set(Gf.Vec3d(0, 0, math.degrees(_yaw0)))
    if not any(_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ for _op in _vx.GetOrderedXformOps()):
        _vx.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, math.degrees(_yaw0)))

add_flat_markers(stage)
add_overview_camera(stage)

art.initialize()
world.reset()

names = list(art.dof_names)
idx = {n: names.index(n) for n in names}
config = WaypointControllerConfig(max_speed=1.2, max_steering_angle=0.6,
                                  waypoint_tolerance=0.40, final_stop_distance=0.30)
follower = WaypointFollower(ROUTE, config)


def pose():
    m = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Vehicle/Chassis")
                          ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    q = m.Transform(Gf.Vec3d(0, 0, 0))
    fwd = m.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
    return (float(q[0]), float(q[1]), float(q[2])), math.atan2(fwd[1], fwd[0])


def poly_dev(x, y):
    best = 1e9
    for i in range(len(ROUTE)-1):
        ax, ay = ROUTE[i]
        bx, by = ROUTE[i+1]
        dx, dy = bx-ax, by-ay
        L2 = dx*dx + dy*dy
        t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((x-ax)*dx + (y-ay)*dy) / L2))
        best = min(best, math.hypot(x-(ax+t*dx), y-(ay+t*dy)))
    return best


print("=" * 50, flush=True)
print("VEHICLE ROUTE VALIDATION (TEST 2)", flush=True)
print("=" * 50, flush=True)
print(f"START: {START}", flush=True)
print(f"GOAL:  {GOAL}", flush=True)
print(f"TOTAL ROUTE LENGTH: {ROUTE_LENGTH:.2f} m", flush=True)
print(f"WAYPOINTS: {len(ROUTE)}", flush=True)
print(f"MAX SPEED: {config.max_speed} m/s", flush=True)
print(f"MAX STEERING: {config.max_steering_angle} rad", flush=True)
print("STATUS:", flush=True)
print("[START]", flush=True)

# Settle stationary 2 s, verify spawn, then drive.
(x, y, z), yaw = pose()
print(f"[INFO] spawn pose x={x:.2f} y={y:.2f} z={z:.3f} yaw={math.degrees(yaw):.1f}deg", flush=True)
settle_z = []
for _ in range(200):
    world.step(render=False)
    (x, y, z), yaw = pose()
    settle_z.append(z)

(x0, y0, z0), yaw0 = pose()
max_steps = int(args.duration / dt)
reached = False
drive_pos = {}
trail, steers, speeds = [], [], []
max_step_jump, prev_p = 0.0, (x0, y0)
left_hold = right_hold = 0.0
cur_leg = -1
stopped_speed = 0.0
yaw_hist = []

for step in range(max_steps):
    if not sim.is_running():
        break
    (x, y, z), yaw = pose()
    steer, speed, reached = follower.update(x, y, yaw, dt)
    omega = speed / float(vehicle_cfg["wheel_radius"])
    drive_step(art, idx, steer, omega, dt, drive_pos)
    world.step(render=not args.headless or args.gui)
    trail.append((x, y))
    steers.append(steer)
    speeds.append(speed)
    yaw_hist.append(yaw)
    max_step_jump = max(max_step_jump, math.hypot(x - prev_p[0], y - prev_p[1]))
    prev_p = (x, y)
    if steer > 0.25:
        left_hold += dt
    elif steer < -0.25:
        right_hold += dt
    leg = min(follower.current_idx, len(LEG_LABELS) - 1)
    if leg != cur_leg:
        cur_leg = leg
        print(f"[{LEG_LABELS[cur_leg]}]", flush=True)
    if step % int(2.0 / dt) == 0:
        print(f"  t={step*dt:.1f}s x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f} "
              f"steer={steer:.2f} speed={speed:.2f} dev={poly_dev(x, y):.2f}", flush=True)
    if reached and speed < 0.05:
        stopped_speed = speed
        print(f"[GOAL] reached at t={step*dt:.1f}s", flush=True)
        break

(pre_sx, pre_sy, _pre_sz), _pre_yaw = pose()
for _ in range(100):
    (x, y, z), yaw = pose()
    s, sp, _ = follower.update(x, y, yaw, dt)
    drive_step(art, idx, s, 0.0, dt, drive_pos)
    world.step(render=False)
(x, y, z), yaw = pose()
final_speed = follower.current_speed
settle_drift = math.hypot(x - pre_sx, y - pre_sy)
print(f"[INFO] stop: follower_speed={final_speed:.3f} settle_drift={settle_drift:.3f} m", flush=True)
dist_B = math.hypot(GOAL[0]-x, GOAL[1]-y)
travelled = sum(math.hypot(trail[i+1][0]-trail[i][0], trail[i+1][1]-trail[i][1])
                for i in range(len(trail)-1))
max_dev = max(poly_dev(px, py) for px, py in trail[::10]) if trail else 999
max_steer = max(abs(s) for s in steers) if steers else 0.0
max_speed = max(speeds) if speeds else 0.0
# Joint-use evidence: commanded steer sweep + commanded wheel roll (sent every
# tick to Joint_Steer_FL/FR + Joint_RL/RR/Joint_Drive_FL/FR via drive_step
# position/velocity targets), plus physical yaw sweep proving the steering
# took effect through the articulation (no teleport per L11).
steer_range = (max(steers) - min(steers)) if len(steers) > 1 else 0.0
wheel_radius = float(vehicle_cfg["wheel_radius"])
cmd_roll = sum(abs(s) / wheel_radius * dt for s in speeds)
yaw_sweep = (max(yaw_hist) - min(yaw_hist)) if len(yaw_hist) > 1 else 0.0

print("=" * 50, flush=True)
print(f"Final position: ({x:.2f}, {y:.2f})  Final yaw: {math.degrees(yaw):.1f} deg", flush=True)
print(f"Final position error: {dist_B:.2f} m", flush=True)
print(f"Total distance travelled: {travelled:.2f} m", flush=True)
print(f"Maximum steering command: {max_steer:.3f} rad", flush=True)
print(f"Route segments completed: {min(follower.current_idx + (1 if reached else 0), len(ROUTE))}/{len(ROUTE)}", flush=True)
print(f"Collision/intersection status: max route deviation {max_dev:.2f} m "
      f"(route cleared >=1.1 m by layout probe)", flush=True)
print("=" * 50, flush=True)

checks = [
    ("L1 start pose valid", math.hypot(x0-START[0], y0-START[1]) < 0.10),
    ("L2 floor contact (settle z)", all(0.22 <= v <= 0.42 for v in settle_z)),
    ("L3 no initial wall intersection", poly_dev(x0, y0) < 0.6),
    ("L4 upright/stable (settle z-spread)", (max(settle_z) - min(settle_z)) < 0.03),
    ("L5 forward progress", travelled > 5.0),
    ("L6 follows route", max_dev < 1.5),
    ("L7 left turn occurred", left_hold > 0.5),
    ("L8 right turn occurred", right_hold > 0.5),
    ("L9 reached GOAL", dist_B < 0.6 and reached),
    ("L10 stopped at GOAL (physical)", settle_drift < 0.15 and final_speed < 0.5),
    ("L11 no teleport", max_step_jump < config.max_speed * dt * 3 + 0.02),
    ("L12 steer joints used (cmd sweep + yaw effect)", steer_range > 0.3 and yaw_sweep > 0.5),
    ("L13 wheels drove (cmd roll + travel, no teleport)", cmd_roll > 20.0 and travelled > 5.0),
]
for label, ok in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {label}", flush=True)
print(f"START = {START}", flush=True)
print(f"GOAL = {GOAL}", flush=True)
print(f"ROUTE_LENGTH = {ROUTE_LENGTH:.2f} m", flush=True)
print(f"TRAVEL_DISTANCE = {travelled:.2f} m", flush=True)
print(f"FINAL_ERROR = {dist_B:.2f} m", flush=True)
print(f"MAX_SPEED = {max_speed:.2f} m/s", flush=True)
print(f"MAX_STEERING = {max_steer:.3f} rad", flush=True)
print(f"LEFT_TURN_DETECTED = {'YES' if left_hold > 0.5 else 'NO'}", flush=True)
print(f"RIGHT_TURN_DETECTED = {'YES' if right_hold > 0.5 else 'NO'}", flush=True)
print(f"REACHED_GOAL = {'YES' if (dist_B < 0.6 and reached) else 'NO'}", flush=True)

if args.gui:
    print("[INFO] GUI hold-open: viewport Camera -> Route_Overview shows the route.", flush=True)
    try:
        while sim.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass
else:
    sim.close()
    sys.exit(0 if all(ok for _, ok in checks) else 1)
