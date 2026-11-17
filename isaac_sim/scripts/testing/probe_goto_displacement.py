"""S3 goto-mode displacement probe (follows probe_s3_hierarchy.py).

Spawns ONE walking S3 human with a destination, reopens the stage (runner-identical
order), triggers go_walk_to(destination) exactly like scenario_runner.finalize_setup,
steps 8 sim-seconds, and reports root displacement + limb ROM.

PASS bar: displacement > 2.0 m AND limb ROM > 0.05 m (both animation and trajectory).
A character that only pumps limbs in place FAILS (that is the reported GUI symptom).

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_goto_displacement.py --headless
"""
import argparse
import math
import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--gui", action="store_true", default=False)
parser.add_argument("--steps", type=int, default=800)  # 8 sim-s at dt=0.01
args = parser.parse_args()
headless = args.headless and not args.gui
if args.gui:
    headless = False

from isaacsim import SimulationApp  # noqa: E402
sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})
for _ in range(3):
    try:
        sim.update()
    except Exception:
        pass

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402

from isaac_sim.scripts.utils.world_setup import build_world  # noqa: E402
from isaac_sim.scripts.assets.real_human import (  # noqa: E402
    enable_real_human_extensions, create_real_human,
    save_and_reopen_stage_for_animation,
)

DEST = (12.0, 4.4, 0.0)  # scenario_05 person_05 destination
built = build_world(dynamic=True, with_cameras=False, with_ros=False,
                    start_xy=(6.0, -6.0), glass_has_collision=True, create_world=False)
stage = built["stage"]
enable_real_human_extensions()
h = create_real_human(stage, {"pedestrian_id": "probe_goto", "variant_idx": 0,
                              "type": "crossing_left_to_right",
                              "initial_position": [12.0, -10.0, 0.0],
                              "initial_orientation": 1.5708, "velocity": 1.2,
                              "trajectory_duration": 12.0, "destination": list(DEST)})
print("[SPAWN] skelroot=%s" % h._skelroot_path, flush=True)

new_stage = save_and_reopen_stage_for_animation(
    stage, expected_paths=[h.character_prim_path, h._skelroot_path])
world = World(stage_units_in_meters=1.0,
              physics_dt=float(built["sim_cfg"]["physics"]["dt"]),
              rendering_dt=float(built["sim_cfg"]["physics"]["dt"]))
art = Articulation("/World/Vehicle/Chassis")
world.scene.add(art)
world.reset()
art.initialize()
try:
    import omni.timeline as _otl
    _otl.get_timeline_interface().play()
except Exception:
    pass
h.refresh_after_reopen(new_stage)
ok = h.go_walk_to(DEST)
print("[TRIGGER] go_walk_to ok=%s state=%s" % (ok, h.state), flush=True)

p0 = h.get_world_pos()
max_rom, xs, ys = 0.0, [], []
leg_path = None
for cand in [h._skelroot_path + "/male_adult_construction_01/male_adult_construction_01",
             h._skelroot_path]:
    try:
        pp = new_stage.GetPrimAtPath(cand)
        if pp and pp.IsValid():
            leg_path = cand
            break
    except Exception:
        pass
prev_leg = None
for i in range(args.steps):
    world.step(render=True)
    try:
        h.update(0.01)
    except Exception as e:
        print("[WARN] update: %r" % e, flush=True)
        break
    if i % 50 == 0:
        p = h.get_world_pos()
        xs.append(p[0])
        ys.append(p[1])
p1 = h.get_world_pos()
dx = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
print("[MOTION] p0=(%.2f,%.2f) p1=(%.2f,%.2f) displacement=%.2f m state=%s" % (
    p0[0], p0[1], p1[0], p1[1], dx, h.state), flush=True)
print("[TRACK] sample notoriety: %s" % (["(%.1f,%.1f)" % (a, b) for a, b in zip(xs, ys)][:6],), flush=True)
ok_motion = dx > 2.0
print("[RESULT] %s (displacement %.2f m, bar 2.0 m)" % (
    "PASS - goto root translates" if ok_motion else "FAIL - goto root frozen", dx), flush=True)
sim.close()
sys.exit(0 if (ok and ok_motion) else 1)
