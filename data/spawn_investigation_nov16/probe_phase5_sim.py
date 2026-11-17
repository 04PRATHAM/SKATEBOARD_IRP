
import sys
import os
import math
import json
import numpy as np

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from isaacsim import SimulationApp
sim = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom
from isaac_sim.scripts.scenarios.scenario_runner import ScenarioRunner
from isaac_sim.scripts.vehicle.ackermann_controller_ros import drive_step

scen_path = "/home/ubuntu/SKATEBOARD_IRP/configs/scenarios/scenario_s02_one_static_obstacle.yaml"
runner = ScenarioRunner(headless=True, enable_ros=False, gui_mode=False, max_duration=15.0)
scenario = runner.load_scenario(scen_path)

v_start = [6.0, -6.0, 0.35, 0.0]
v_dest = [13.5, -6.0, 0.0]

runner.initialize_sim(defer_for_anim=False, start_pose=v_start)
runner.setup_scenario(scenario)
runner.vehicle_start = v_start
runner.vehicle_dest = v_dest
runner.finalize_setup()

names = list(runner.articulation.dof_names)
idx = {n: names.index(n) for n in names}
drive_pos = {}

# Step 1: Initial spawn & active zero hold
p0, _ = runner.articulation.get_world_poses()
init_pos = [float(x) for x in p0[0]]

for _ in range(30):
    drive_step(runner.articulation, idx, 0.0, 0.0, runner.dt, drive_pos)
    runner.world.step(render=False)

p30, _ = runner.articulation.get_world_poses()
hold_pos = [float(x) for x in p30[0]]
hold_drift = math.hypot(hold_pos[0] - init_pos[0], hold_pos[1] - init_pos[1])

# Step 2: Drive forward from selected start
steer_cmd = -0.10
speed_cmd = 0.50
wheel_r = float(runner.vehicle_cfg["wheel_radius"])
omega = speed_cmd / wheel_r

for step in range(100):
    drive_step(runner.articulation, idx, steer_cmd, omega, runner.dt, drive_pos)
    runner.world.step(render=False)

p100, r100 = runner.articulation.get_world_poses()
final_pos = [float(x) for x in p100[0]]
final_rot = [float(x) for x in r100[0]]
displacement = math.hypot(final_pos[0] - init_pos[0], final_pos[1] - init_pos[1])

res = {
    "init_pos": init_pos,
    "hold_pos": hold_pos,
    "hold_drift_mm": round(hold_drift * 1000.0, 3),
    "final_pos": final_pos,
    "final_rot": final_rot,
    "displacement_m": round(displacement, 4),
    "status": "PASS" if (hold_drift < 0.005 and displacement > 0.35) else "FAIL"
}

with open("/home/ubuntu/SKATEBOARD_IRP/data/spawn_investigation_nov16/phase5_sim_result.json", "w") as f:
    json.dump(res, f, indent=2)

runner.cleanup()
sim.close()
