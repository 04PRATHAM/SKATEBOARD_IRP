"""S3 stage-hierarchy probe: does save/reopen preserve SkelRoot + skinned meshes?

Spawns 3 S3 variants (construction/business/medical) exactly like the GUI runners,
dumps the USD hierarchy BEFORE and AFTER save_and_reopen_stage_for_animation,
and reports: SkelRoot count, Mesh prim count, Skeleton joints, visibility/purpose,
SkelBinding presence, and runtime Character validity.

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_s3_hierarchy.py --headless
"""
import argparse
import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--gui", action="store_true", default=False)
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

from pxr import Usd, UsdGeom, UsdSkel  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402

from isaac_sim.scripts.utils.world_setup import build_world  # noqa: E402
from isaac_sim.scripts.assets.real_human import (  # noqa: E402
    enable_real_human_extensions, create_real_human,
    save_and_reopen_stage_for_animation, RealHumanConfig, HumanType,
)


def dump(stage, tag):
    print("===== HIERARCHY %s =====" % tag, flush=True)
    chars = stage.GetPrimAtPath("/World/Characters")
    print("Characters valid: %s" % (chars.IsValid() if chars else False), flush=True)
    skelroots, meshes, skels = [], [], []
    if chars and chars.IsValid():
        for p in Usd.PrimRange(chars):
            if p.IsA(UsdSkel.Root):
                skelroots.append(str(p.GetPrimPath()))
            if p.GetTypeName() == "Mesh":
                img = UsdGeom.Imageable(p)
                vis = str(img.ComputeVisibility(Usd.TimeCode.Default())) if p.IsValid() else "?"
                pur = str(img.GetPurposeAttr().Get(Usd.TimeCode.Default())) if p.IsValid() else "?"
                meshes.append((str(p.GetPrimPath()), vis, pur))
            if p.GetTypeName() == "Skeleton":
                try:
                    n = len(UsdSkel.Skeleton(p).GetJointsAttr().Get() or [])
                except Exception:
                    n = -1
                skels.append((str(p.GetPrimPath()), n))
    print("SkelRoots (%d): %s" % (len(skelroots), skelroots[:4]), flush=True)
    print("Meshes (%d): showing up to 6:" % len(meshes), flush=True)
    for m in meshes[:6]:
        print("   %s vis=%s purpose=%s" % m, flush=True)
    print("Skeletons: %s" % skels, flush=True)
    # SkelBinding check on first skelroot
    bound = 0
    try:
        from pxr import UsdSkel as _US
        for s in skelroots[:1]:
            sp = stage.GetPrimAtPath(s)
            b = _US.BindingAPI(sp)
            targets = b.GetSkeletonRel().GetTargets() if b else []
            bound = len(targets)
    except Exception as e:
        print("binding check err: %r" % e, flush=True)
    print("binding targets on first SkelRoot: %d" % bound, flush=True)
    return len(skelroots), len(meshes)


built = build_world(dynamic=True, with_cameras=False, with_ros=False,
                    start_xy=(6.0, -6.0), glass_has_collision=True, create_world=False)
stage = built["stage"]
enable_real_human_extensions()
variants = [(0, "probe_construction"), (1, "probe_business"), (2, "probe_medical")]
humans = []
for vidx, pid in variants:
    h = create_real_human(stage, {"pedestrian_id": pid, "variant_idx": vidx,
                                  "type": "stationary",
                                  "initial_position": [2.0 + vidx * 2.0, 0.0, 0.0],
                                  "initial_orientation": 0.0, "velocity": 0.0,
                                  "trajectory_duration": 10.0})
    humans.append(h)
    print("[SPAWN] %s variant=%d skelroot=%s" % (pid, h.variant_idx, h._skelroot_path), flush=True)

n_pre = dump(stage, "BEFORE-REOPEN")

want = []
for h in humans:
    want.append(getattr(h, "character_prim_path", ""))
    want.append(getattr(h, "_skelroot_path", ""))
new_stage = save_and_reopen_stage_for_animation(stage, expected_paths=want)
stage = new_stage
n_post = dump(stage, "AFTER-REOPEN")

# World + reset + refresh (runner-identical order), then runtime check
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
for h in humans:
    h.refresh_after_reopen(new_stage)
ok_idle = [h.go_idle() for h in humans]
print("go_idle results: %s" % ok_idle, flush=True)
for _ in range(30):
    world.step(render=True)
    for h in humans:
        try:
            h.update(0.01)
        except Exception:
            pass
dump(stage, "AFTER-30-STEPS")

print("[VERDICT] skelroots pre=%d post=%d meshes pre=%d post=%d idle=%s" % (
    n_pre[0], n_post[0], n_pre[1], n_post[1], ok_idle), flush=True)
ok = n_post[0] >= 3 and n_post[1] >= 9 and all(ok_idle)
print("[RESULT] %s" % ("PASS - reopen preserves S3 mesh hierarchy" if ok else "FAIL - hierarchy degraded"), flush=True)
sim.close()
