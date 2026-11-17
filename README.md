# Multi-Object Steering Using RGB-D Camera

## Project Overview

This project develops a NVIDIA Isaac Sim simulation for multi-object steering using an RGB-D camera. The system demonstrates a vehicle navigating around people and obstacles in a laboratory environment.

## Team Structure

- **Member 1**: RGB-based perception side (existing/senior project)
- **Member 2**: NVIDIA Isaac Sim simulation (this repository)

## Project Structure

```
multi_object_steering_sim/
├── README.md
├── PROJECT_PLAN.md
├── .gitignore
├── references/
│   ├── vehicle/
│   └── lab/
├── docs/
│   ├── architecture.md
│   ├── simulation_workflow.md
│   ├── vehicle_model.md
│   ├── environment_model.md
│   ├── sensor_setup.md
│   ├── ros2_interfaces.md
│   ├── scenarios.md
│   ├── validation.md
│   └── troubleshooting.md
├── isaac_sim/
│   ├── stages/
│   ├── robots/
│   │   └── vehicle/
│   ├── environments/
│   │   ├── lab/
│   │   ├── furniture/
│   │   └── partitions/
│   ├── sensors/
│   │   └── rgbd_camera/
│   └── scripts/
├── ros2_ws/
│   └── src/
│       ├── simulation_bringup/
│       ├── rgbd_perception/
│       ├── steering_controller/
│       └── simulation_interfaces/
├── configs/
│   ├── vehicle.yaml
│   ├── camera.yaml
│   ├── steering.yaml
│   └── simulation.yaml
├── scenarios/
│   ├── scenario_01_empty.yaml
│   ├── scenario_02_single_person.yaml
│   ├── scenario_03_person_left.yaml
│   ├── scenario_04_person_right.yaml
│   ├── scenario_05_person_center.yaml
│   ├── scenario_06_two_people.yaml
│   ├── scenario_07_person_table.yaml
│   ├── scenario_08_narrow_corridor.yaml
│   ├── scenario_09_multiple_obstacles.yaml
│   └── scenario_10_dynamic_pedestrian.yaml
├── tests/
│   ├── test_camera.py
│   ├── test_depth.py
│   ├── test_topics.py
│   └── test_steering.py
├── data/
│   ├── rgb/
│   ├── depth/
│   ├── detections/
│   └── results/
└── scripts/
    ├── launch_sim.sh
    ├── build.sh
    └── clean.sh
```

## Important Notes

- This is a two-member academic project
- The physical vehicle is fixed in the laboratory and cannot be freely driven
- The simulation is a **compulsory and major** part of the project, not optional visualization
- The sensor is an RGB + DEPTH camera (RGB-D)
- The final architecture supports: RGB + Depth -> ROS 2 -> Perception -> Multi-object reasoning -> Steering decision -> Vehicle controller -> Isaac Sim vehicle
<<<<<<< HEAD- Do not create a generic TurtleBot.
- Do not use differential drive unless justified.
- Do not use LiDAR.
- Do not make RGB or depth optional.
- Do not hard-code dimensions or topic names.

## Current State (2026-11-17)

- **Humans:** Two explicit tiers: `s3` (real skinned S3 assets with graph animation) and `segmented` (primitive assembly for headless fallback). S3 failure raises an error rather than silently downgrading.
- **Vehicle safety:** YAML scenarios hold the vehicle stationary by default through `vehicle_behavior`. The previous unconditional cruise caused a front-wall collision at t=16 seconds. See `docs/WALL_COLLISION_REPORT.md`.
- **Perception:** YOLO26n-seg multi-class `[0,13,56,57,60]` has been standalone-verified. Class 60 is exposed as `table`, with class IDs unchanged. Live ROS RGB/depth is blocked by an Isaac Sim 5.1 Synthetic Data writer regression. See `docs/YOLO_PERCEPTION_VALIDATION_REPORT.md`.
- **Not yet completed:** Live YOLO/fusion/Object3D processing on ROS topics, GUI visual sign-off, and autonomy.
