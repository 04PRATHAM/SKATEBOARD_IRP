#!/usr/bin/env python3
"""
WASD / Manual Locomotion Test Script for Isaac Sim 5.1.0.

Keeps simulation open indefinitely for vehicle locomotion and inspection.
Viewport Camera: /World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera
(or /World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera)

Controls:
  W = forward (+0.5 m/s)
  S = reverse (-0.5 m/s)
  A = steer left (+0.2 rad)
  D = steer right (-0.2 rad)
  Space = brake (0 speed, 0 steer)
"""
import sys
sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--headless", action="store_true", help="Run headless mode")
    args = p.parse_args()

    from isaacsim import SimulationApp
    sim = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting"})

    import numpy as np
    from pxr import Usd, UsdGeom, Gf
    from isaac_sim.scripts.utils.world_setup import build_world
    from isaac_sim.scripts.testing.wasd_controller import WASDState, subscribe_wasd
    from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step

    print("[INFO] Initializing WASD locomotion world...", flush=True)
    built = build_world(dynamic=True, with_cameras=True, with_ros=False, start_xy=(0.0, -4.0))
    world, stage, art = built["world"], built["stage"], built["articulation"]
    vehicle_cfg, sim_cfg = built["vehicle_cfg"], built["sim_cfg"]
    dt = float(sim_cfg["physics"]["dt"])

    art.initialize()
    names = list(art.dof_names)
    I = {n: names.index(n) for n in names}
    drive_pos = {}
    wasd = WASDState(max_speed=1.5, max_steer=0.6)

    # Event-driven keyboard (carb.input). Viewport must have focus.
    kbd_iface = kbd = kbd_sub = None
    try:
        kbd_iface, kbd, kbd_sub = subscribe_wasd(wasd)
        print("[WASD] Keyboard subscription active (focus the viewport).", flush=True)
    except Exception as e:
        print(f"[WASD] Keyboard subscribe failed ({e}); stdin fallback only.", flush=True)

    print("\n" + "="*70)
    print(" WASD MANUAL LOCOMOTION TEST")
    print(" Viewport Camera → /World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera")
    print("   (or .../Sensors/RGBD_Camera/RGBD_Camera for driver POV)")
    print(" W/S = forward/back, A/D = steer left/right, Space = brake, R = center")
    print(" Press Ctrl+C in terminal or close window to exit.")
    print("="*70 + "\n", flush=True)

    frame = 0
    try:
        while sim.is_running():
            # Optional stdin fallback (only if a terminal char is waiting)
            try:
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if ch and ch.strip():
                        wasd.handle_key(ch.strip())
                        print(f"[WASD] {wasd} (stdin)", flush=True)
            except Exception:
                pass

            omega = float(wasd.speed) / float(vehicle_cfg["wheel_radius"])
            drive_step(art, I, float(wasd.steer), omega, dt, drive_pos)
            world.step(render=True)

            frame += 1
            if frame % int(2.0 / dt) == 0:
                m = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Vehicle/Chassis")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                p = m.Transform(Gf.Vec3d(0, 0, 0))
                yaw = np.arctan2(m.TransformDir(Gf.Vec3d(1, 0, 0))[1], m.TransformDir(Gf.Vec3d(1, 0, 0))[0])
                print(f"[WASD] {wasd} | pose x={p[0]:.2f} y={p[1]:.2f} yaw={np.degrees(yaw):.1f}°", flush=True)
    except KeyboardInterrupt:
        print("[WASD] User requested exit.", flush=True)
    finally:
        try:
            if kbd_iface is not None and kbd is not None and kbd_sub is not None:
                kbd_iface.unsubscribe_to_keyboard_events(kbd, kbd_sub)
        except Exception:
            pass
        try:
            sim.close()
        except Exception:
            pass
        print("[WASD] Simulation closed cleanly.", flush=True)

if __name__ == "__main__":
    main()
