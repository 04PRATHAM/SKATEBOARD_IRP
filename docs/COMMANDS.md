# SKATEBOARD_IRP — Command Reference (Isaac Sim 5.1.0-rc.19 + ROS 2 Humble)

All commands assume `cd /home/ubuntu/SKATEBOARD_IRP` and Isaac Sim at `/home/ubuntu/isaacsim/`.

## 0. Environment (once per terminal)

```bash
# ROS 2 (required for any --ros2 or /cmd_ackermann)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # bridge requires FastDDS
```

## 1. Simulation — Visual Inspection (static lab + coherent vehicle, no physics drive)

**GUI, stays open until you close window or Ctrl+C (120s auto-exit to avoid force-quit dialog):**
```bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --inspect
# Viewport → Camera →
#   /World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera  (RGB-D, front riser 0.9,0,1.2, pitch -8.5° — model input)
#   /World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera  (seat eye -0.25,0,0.75, pitch 0° — human POV, moves 1:1)
#   /World/InspectionCameras/*  (24 static world views, NOT moving, NOT for driving)
```

**Headless quick check (no window, exit 0 = OK, 2–3s):**
```bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --duration 2.0
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --headless --duration 3.0
```

**With ROS 2 bridge (streams RGB, depth, CameraInfo, clock, TF):**
```bash
source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_world.py --ros2 --headless --duration 3.0
# second terminal:
ros2 topic list   # expect 6: /camera/rgb/image_raw, /camera/depth/image_raw, /camera/rgb/camera_info, /camera/depth/camera_info, /clock, /tf
ros2 topic echo --once /camera/rgb/image_raw  # 640×480 rgb8, frame camera_rgb_optical_frame
ros2 topic hz /camera/rgb/image_raw
```

## 2. Simulation — Dynamic Driving (6-DOF Ackermann, true wheel drive)

**Auto demo (GUI, open lane at (0,-4) +X, 12m to front wall):**
```bash
# optional ROS live override:
source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_ackermann_demo.py
# Viewport → Camera → /World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera  (or RGBD)
# auto: settle 2s → straight 2.5s 0.8 m/s → left 2.5s 0.35 rad 0.6 → stop 1s → right 2.5s -0.35 0.6 → hold open (90s timeout)
# second terminal while demo holds open:
ros2 topic pub -r 10 /cmd_ackermann ackermann_msgs/msg/AckermannDriveStamped '{drive: {steering_angle: 0.0, speed: 0.5}}'   # straight
ros2 topic pub -r 10 /cmd_ackermann ackermann_msgs/msg/AckermannDriveStamped '{drive: {steering_angle: 0.3, speed: 0.5}}'   # left
ros2 topic pub -r 10 /cmd_ackermann ackermann_msgs/msg/AckermannDriveStamped '{drive: {steering_angle: -0.3, speed: 0.5}}'  # right
ros2 topic pub --once /cmd_ackermann ackermann_msgs/msg/AckermannDriveStamped '{drive: {steering_angle: 0.0, speed: 0.0}}'  # stop
```

**WASD manual test (isolated, GUI, open corridor (0,-4) +X):**
```bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/run_wasd_test.py
# Viewport → Camera → /World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera
# Focus the viewport first, then:
#   W = forward +0.4 m/s (max 1.5), S = back -0.4 (max -1.0)
#   A = left +0.15 rad, D = right -0.15 rad (max ±0.6)
#   Space = brake (0,0), R = centre steer, Esc/close = quit (120s timeout)
# Console fallback: type w/a/s/d + Enter in the terminal that launched the sim
```

## 3. Tests — Headless (no window, CI-friendly)

```bash
# Full vehicle foundation (56 checks — hierarchy, transforms, wheel contact, Ackermann, camera, visuals)
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/verify_vehicle_complete.py --headless   # expect 56/56

# Physics suite (15 checks — settle, straight, left, right, stop, clamps)
/home/ubuntu/isaacsim/python.sh tests/verify_ackermann_physics.py --headless             # expect 14/15 (mirrored 0.44 vs 0.205 known asymmetry)

# Other physics probes (if needed):
/home/ubuntu/isaacsim/python.sh tests/test_vehicle_physics.py --headless
```

## 4. ROS 2 Inspection (while any --ros2 GUI is running)

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo --once /clock
ros2 topic echo --once /tf | head
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /camera/depth/image_raw
```

## 5. GUI Tips & Troubleshooting

- **Driver POV that moves:** Select `/World/Vehicle/Chassis/Driver_Seat_Camera/DriverSeat_Camera` (seat eye) **or** `/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera` (front riser). **Do NOT** use `Driver_Perspective` under `/World/InspectionCameras` — it is **static world-anchored** at (-4.3,0,1.15) for the inspect spawn (-4,0) and does NOT move with the dynamic vehicle at (6,-6) or (0,-4).
- **Vehicle appears in front of camera / whole vehicle view:** You selected an inspection camera. Switch to one of the two chassis-mounted cameras above.
- **Wheels not visible / hexagonal:** Fixed 2026-09-09: `Wheel_*` Xform now visible (only `Collider` hidden), smooth tire `Cylinder` (no `Tread_*` blocks), white rims. If you still see old hexagonal treads, delete `/tmp` or re-run — stale `Tread_*` prims are auto-hidden.
- **“ISAAC sim not responding” on launch:** **Normal** for 9–11s while `SimulationApp` loads 50+ extensions (`app ready` at ~11s, `Startup Complete` at ~14s). GNOME shows “not responding” after ~5s. **Wait, do NOT click Force Quit.** Use headless (`--headless --duration 2.0`) for quick checks. All GUI runners now auto-exit after 90–120s to avoid force-quit dialog on hold-open.
- **Force-quit on close:** Fixed 2026-09-10: all hold-open loops use `while sim.is_running(): world.step()` + `try/finally: sim.close()` + timeout. Close the window **or** press `Ctrl+C` in the launching terminal; logs should end with `Closed cleanly.`
- **Wall crash / shaking:** Dynamic spawn `(6,-6)` and WASD `(0,-4)` are in open lanes (9–12m to front wall). Demo waypoints `straight 2.5s 0.8` etc. stay clear. If you drive manually into a wall, `drive_step` still sends `omega>0` → PhysX resolves interpenetration → high-frequency shake. **Brake (Space, speed 0)** before wall.

## 6. Quick Smoke Test (30s total)

```bash
cd /home/ubuntu/SKATEBOARD_IRP
source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/verify_vehicle_complete.py --headless
# → 56/56 in ~30s, 0 setDriveTarget errors
```

## 7. File Map

- Core: `isaac_sim/scripts/vehicle/simple_vehicle.py`, `visual_vehicle.py`, `ackermann_controller_ros.py`
- Sensors: `isaac_sim/scripts/sensors/rgbd_camera.py`, `utils/aim.py`
- World: `isaac_sim/scripts/utils/world_setup.py`, `environments/lab_environment.py`, `utils/inspection_cameras.py`
- Bridge: `isaac_sim/scripts/bridge/ros2_bridge.py`
- Testing (isolated): `isaac_sim/scripts/testing/wasd_controller.py`, `run_wasd_test.py`
- Tests: `tests/verify_ackermann_physics.py`, `isaac_sim/scripts/verify_vehicle_complete.py`
- Configs: `configs/vehicle.yaml`, `camera.yaml`, `simulation.yaml`
- Docs: `docs/vehicle_model.md`, `docs/VEHICLE_RECONSTRUCTION_REPORT.md`, `work.md`

