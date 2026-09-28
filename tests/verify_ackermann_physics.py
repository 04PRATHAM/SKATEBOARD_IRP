#!/usr/bin/env python3
"""
Deterministic 7-test verifier for TRUE Ackermann physics (PROJECT_PLAN M3/M4).

Pure articulation drive — NO chassis velocity override. All motion comes from
wheel joint velocities + steering joint positions held every step via the
6-DOF articulation (Joint_Steer_FL/FR, Joint_RL/RR, Joint_Drive_FL/FR).

Tests: T1 settle, T2 straight, T3 left (+yaw), T4 right (−yaw, mirrored),
T5 stop, T6 steer clamp, T7 speed clamp. Exit 0 iff ALL pass.

  /home/ubuntu/isaacsim/python.sh tests/verify_ackermann_physics.py --headless
"""

import sys
import math

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def main(headless=True):
    from isaacsim import SimulationApp
    sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})
    import numpy as np
    from pxr import Usd, UsdGeom, Gf
    import omni.usd
    from isaac_sim.scripts.utils.world_setup import build_world
    from isaac_sim.scripts.vehicle.simple_vehicle import apply_ackermann_steering

    built = build_world(dynamic=True, with_cameras=False, with_ros=False)
    world, stage, art = built["world"], built["stage"], built["articulation"]
    vehicle_cfg, sim_cfg = built["vehicle_cfg"], built["sim_cfg"]
    dt = float(sim_cfg["physics"]["dt"])
    R = float(vehicle_cfg["wheel_radius"])
    MAX_STEER = float(vehicle_cfg["max_steering_angle"])
    MAX_SPEED = float(vehicle_cfg["max_speed"])
    art.initialize()
    names = list(art.dof_names)
    I = {n: names.index(n) for n in
         ["Joint_Steer_FL", "Joint_Steer_FR", "Joint_RL", "Joint_RR",
          "Joint_Drive_FL", "Joint_Drive_FR"]}
    from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step
    drive_pos = {}
    results = []

    def hold(steer, omega, seconds):
        for _ in range(int(seconds / dt)):
            drive_step(art, I, steer, omega, dt, drive_pos)
            world.step(render=not headless)

    def check(label, ok, detail=""):
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {label} {detail}", flush=True)

    def pose():
        m = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Vehicle/Chassis")
              ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        p = m.Transform(Gf.Vec3d(0, 0, 0))
        f = m.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
        u = m.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        yaw = math.atan2(f[1], f[0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, u[2]))))
        return (p[0], p[1], p[2]), yaw, tilt

    def spin_max():
        return float(np.max(np.abs(np.array(art.get_joint_positions()).flatten())))

    # ---- T1 settle 2 s ----
    hold(0.0, 0.0, 2.0)
    (x0, y0, z0), yaw0, tilt0 = pose()
    check("T1 settle z in [0.22,0.38]", 0.22 < z0 < 0.38, f"z={z0:.3f}")
    check("T1 level < 3 deg", tilt0 < 3.0, f"tilt={tilt0:.2f}")
    check("T1 6-DOF articulation", art.num_dof == 6, f"dofs={art.num_dof}")

    # ---- T2 straight: 4 rad/s (1 m/s) for 4 s ----
    (xa, ya, _), _, _ = pose()
    hold(0.0, 4.0, 4.0)
    (xb, yb, zb), yawb, tiltb = pose()
    check("T2 straight dx>0.8", (xb - xa) > 0.8, f"dx={xb - xa:.2f} (expect ~v*t*slip)")
    check("T2 lateral < 0.5", abs(yb - ya) < 0.5, f"dy={yb - ya:.3f}")
    check("T2 wheels rolled > 2 rad", spin_max() > 2.0, f"max|pos|={spin_max():.2f}")
    check("T2 stable", 0.15 < zb < 0.5 and tiltb < 6.0, f"z={zb:.3f} tilt={tiltb:.2f}")

    # ---- T3 left 0.4 rad, 5 s; rate measured over final 2 s (steady state) ----
    hold(0.4, 3.2, 4.0)
    (_, _, _), yaw2a, _ = pose()
    (x2, y2, _), _, _ = pose()
    hold(0.4, 3.2, 2.0)
    (x3, y3, _), yaw3, _ = pose()
    dl, yl = math.dist((x2, y2), (x3, y3)), (yaw3 - yaw2a) / 2.0
    check("T3 left yaw rate positive", yl > 0.10, f"rate={yl:+.3f} rad/s")
    check("T3 left travelled", dl > 0.4, f"dist={dl:.2f}")

    # ---- straight gap 1 s to shed turn momentum before mirroring ----
    hold(0.0, 3.2, 3.0)

    # ---- T4 right -0.4 rad, 5 s; steady-state rate ----
    hold(-0.4, 3.2, 4.0)
    (_, _, _), yaw4a, _ = pose()
    (x4, y4, _), _, _ = pose()
    hold(-0.4, 3.2, 2.0)
    (x5, y5, _), yaw5, _ = pose()
    dr, yr = math.dist((x4, y4), (x5, y5)), (yaw5 - yaw4a) / 2.0
    check("T4 right yaw rate negative", yr < -0.10, f"rate={yr:+.3f} rad/s")
    check("T4 mirrored rates", abs(abs(yl) - abs(yr)) < 0.18,
          f"|left|={abs(yl):.3f} |right|={abs(yr):.3f}")

    # ---- T5 stop 5 s: 150 kg coasts; pass = settles (final speed ~0) ----
    (x6, y6, _), _, _ = pose()
    hold(0.0, 0.0, 4.0)
    (xm, ym, _), _, _ = pose()
    hold(0.0, 0.0, 1.0)
    (x7, y7, _), _, _ = pose()
    final_speed = math.dist((xm, ym), (x7, y7)) / 1.0
    check("T5 stop, settles to rest", final_speed < 0.15,
          f"final_speed={final_speed:.3f} m/s total_drift={math.dist((x6, y6), (x7, y7)):.2f} m")

    # ---- T6/T7 saturation (pure math + controller clamp) ----
    dfl, dfr = apply_ackermann_steering(None, 1.5, vehicle_cfg)
    check("T6 steer inner bounded", abs(dfl) < 0.9 and abs(dfr) < 0.9,
          f"FL={dfl:.3f} FR={dfr:.3f}")
    from isaac_sim.scripts.vehicle.ackermann_controller_ros import AckermannROSController
    ctrl = AckermannROSController(art, vehicle_cfg, world)

    class FakeDrive:
        steering_angle = 1.5
        speed = 5.0

    class FakeMsg:
        drive = FakeDrive()

    ctrl.ros_callback(FakeMsg())
    check("T7 speed clamped to 2.0", abs(ctrl.target_speed) <= 2.0 + 1e-9,
          f"target={ctrl.target_speed:.3f}")
    check("T7 steer clamped to 0.6", abs(ctrl.target_steer) <= 0.6 + 1e-9,
          f"target={ctrl.target_steer:.3f}")

    nfail = sum(1 for ok in results if not ok)
    print(f"===== {len(results) - nfail}/{len(results)} CHECKS PASS =====", flush=True)
    sim.close()
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--headless", action="store_true")
    sys.exit(main(headless=p.parse_args().headless))
