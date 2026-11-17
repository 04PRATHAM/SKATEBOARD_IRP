"""Wall-collision reproduction probe (Part 2.1/2.2).

Replays the EXACT drive logic of scenario_runner.run_scenario
(drive_step articulation, steer=0, omega=0.5/wheel_radius every physics step)
from the scenario start pose (6,-6), and records chassis world-x over time
against the lab front-wall face (x=+14.9 inner). A second pass with omega=0
proves the stationary default holds still.

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/wall_probe.py --headless
"""
import argparse
import math
import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--gui", action="store_true", default=False)
parser.add_argument("--sim-seconds", type=float, default=30.0)
parser.add_argument("--speed", type=float, default=0.5)
args = parser.parse_args()
headless = args.headless and not args.gui
if args.gui:
    headless = False

from isaacsim import SimulationApp  # noqa: E402
sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, Gf  # noqa: E402
from isaac_sim.scripts.utils.world_setup import build_world  # noqa: E402
from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step  # noqa: E402

WALL_FACE_X = 15.0 - 0.10  # room 30m wide, 0.20 thick front wall (lab_environment.py)
VEH_HALF_LEN = 1.05  # approx nose ahead of chassis origin (conservative)


def chassis_x(stage):
    prim = stage.GetPrimAtPath("/World/Vehicle/Chassis")
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return float(m.Transform(Gf.Vec3d(0, 0, 0))[0])


def run_case(omega, label, sim_seconds):
    built = build_world(dynamic=True, with_cameras=False, with_ros=False,
                        start_xy=(6.0, -6.0), glass_has_collision=True, create_world=True)
    stage, world = built["stage"], built["world"]
    art = built["articulation"]
    try:
        art.initialize()
    except Exception:
        pass
    world.reset()
    # Place exactly like scenario_runner.setup_scenario (pose on /World/Vehicle)
    vp = stage.GetPrimAtPath("/World/Vehicle")
    xf = UsdGeom.Xformable(vp)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(6.0, -6.0, 0.35))
    names = list(art.dof_names)
    idx = {n: names.index(n) for n in names}
    drive_pos = {}
    dt = float(built["sim_cfg"]["physics"]["dt"])
    steps = int(sim_seconds / dt)
    x0 = chassis_x(stage)
    contact_t, min_gap = None, 1e9
    for i in range(steps):
        if not sim.is_running():
            break
        drive_step(art, idx, 0.0, omega, dt, drive_pos)
        world.step(render=True)
        if i % int(0.5 / dt) == 0:
            t = (i + 1) * dt
            gap = WALL_FACE_X - (chassis_x(stage) + VEH_HALF_LEN)
            min_gap = min(min_gap, gap)
            if contact_t is None and gap <= 0.0:
                contact_t = t
                print("[%s] CONTACT at t=%.1fs x=%.2f gap=%.2f" % (label, t, chassis_x(stage), gap), flush=True)
    x1 = chassis_x(stage)
    print("[%s] x0=%.2f x1=%.2f dx=%.2f min_gap=%.2f contact_t=%s" % (
        label, x0, x1, x1 - x0, min_gap, contact_t), flush=True)
    return x0, x1, contact_t


omega = args.speed / 1.0  # placeholder, replaced below
import isaac_sim.scripts.utils.config_loader as _cl
_wheel_r = float(_cl.load_config("configs/vehicle.yaml")["vehicle"]["wheel_radius"])
print("wheel_radius=%.3f wall_face_x=%.1f veh_half_len=%.2f" % (_wheel_r, WALL_FACE_X, VEH_HALF_LEN), flush=True)

results = {}


def verdict(label, ok, detail=""):
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", label, detail), flush=True)
    results[label] = bool(ok)
    return bool(ok)


# Case A: legacy runner behavior (0.5 m/s forward, steer 0) — CHARACTERIZES the
# bug: contact is EXPECTED here (documents why cruise was removed as default).
_, _, contact_a = run_case(args.speed / _wheel_r, "RUNNER-0.5m/s", args.sim_seconds)
verdict("legacy-cruise contacts wall (bug characterization)", contact_a is not None,
        "contact_t=%s" % contact_a)
# Case B: stationary default (omega=0) — must hold still, no contact.
omni.usd.get_context().new_stage()
x0b, x1b, contact_b = run_case(0.0, "STATIONARY-0.0", 10.0)
verdict("stationary hold (dx<0.05m, no contact)", abs(x1b - x0b) < 0.05 and contact_b is None,
        "dx=%.3f contact=%s" % (x1b - x0b, contact_b))

print("[PROBE-DONE] %s" % ("ALL-PASS" if all(results.values()) else "FAILURES-PRESENT"), flush=True)
sim.close()
sys.exit(0 if all(results.values()) else 1)
