import sys
import os
import math
import json
import numpy as np
from PIL import ImageGrab

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from isaacsim import SimulationApp
sim = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom
from isaac_sim.scripts.scenarios.scenario_runner import ScenarioRunner

scen_path = "/home/ubuntu/SKATEBOARD_IRP/configs/scenarios/scenario_s02_one_static_obstacle.yaml"
runner = ScenarioRunner(headless=False, enable_ros=False, gui_mode=False, max_duration=10.0)
scenario = runner.load_scenario(scen_path)

v_start = [6.0, -6.0, 0.35, 0.0]
v_goal = [13.5, -6.0, 0.0]

runner.initialize_sim(defer_for_anim=False, start_pose=v_start)
runner.setup_scenario(scenario)
runner.vehicle_start = v_start
runner.vehicle_dest = v_goal
runner.finalize_setup()

# Initial Articulation Pose
positions, orientations = runner.articulation.get_world_poses()
act_pos = [float(x) for x in positions[0]]
act_rot = [float(x) for x in orientations[0]]
qw, qx, qy, qz = act_rot[0], act_rot[1], act_rot[2], act_rot[3]
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
act_yaw = math.atan2(siny_cosp, cosy_cosp)

# Step 20 frames with zero control to verify stability (no spontaneous movement)
for step in range(20):
    runner.world.step(render=True)

pos_after_20, rot_after_20 = runner.articulation.get_world_poses()
p20 = [float(x) for x in pos_after_20[0]]
drift = float(np.linalg.norm(np.array(p20[:2]) - np.array(act_pos[:2])))

# Grab viewport screenshot
try:
    img = ImageGrab.grab()
    img.save("/home/ubuntu/SKATEBOARD_IRP/data/spawn_investigation_nov16/a_sim_physical_spawn.png")
    print(f"[SCREENSHOT] Saved Isaac Sim viewport screenshot to /home/ubuntu/SKATEBOARD_IRP/data/spawn_investigation_nov16/a_sim_physical_spawn.png")
except Exception as e:
    print(f"[SCREENSHOT ERROR] {e}")

# USD Chassis translation
c_prim = runner.stage.GetPrimAtPath("/World/Vehicle/Chassis")
usd_pos = [float(x) for x in UsdGeom.Xformable(c_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()]

out_data = {
    "tc_id": "A",
    "scenario_start": v_start,
    "physical_spawn_pos": act_pos,
    "physical_spawn_rot": act_rot,
    "physical_yaw_rad": act_yaw,
    "physical_yaw_deg": math.degrees(act_yaw),
    "expected_yaw_deg": math.degrees(0.0),
    "usd_chassis_pos": usd_pos,
    "pos_after_20_steps": p20,
    "drift_m": drift,
    "sim_screenshot": "/home/ubuntu/SKATEBOARD_IRP/data/spawn_investigation_nov16/a_sim_physical_spawn.png"
}

with open("/home/ubuntu/SKATEBOARD_IRP/data/spawn_investigation_nov16/result_tc_A.json", "w") as f:
    json.dump(out_data, f, indent=2)

sim.close()
