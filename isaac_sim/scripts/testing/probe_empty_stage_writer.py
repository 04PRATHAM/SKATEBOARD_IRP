"""Empty-stage writer probe: does BasicWriter attach work with a trivial scene?

New hypothesis (Part 7): the writer-attach fault might depend on stage complexity
(lab + vehicle + S3 payloads). A bare stage (ground plane + one camera) isolates
systemic-vs-scene-dependent failure.

PASS = attach succeeds AND a PNG is produced. Either way the result is informative:
- PASS => investigate scene simplification / render-product lifetime as workaround path.
- FAIL => fault is systemic to this Isaac Sim install (case closed, external fix needed).

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_empty_stage_writer.py --headless
"""
import argparse
import sys

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", default=False)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402
sim = SimulationApp({"headless": args.headless, "renderer": "RayTracedLighting"})

import traceback  # noqa: E402
from pxr import UsdGeom, Gf  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.prims import define_prim  # noqa: E402


def verdict(label, ok, detail=""):
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", label, detail), flush=True)
    return bool(ok)


world = World(stage_units_in_meters=1.0, physics_dt=0.01, rendering_dt=0.01)
stage = omni.usd.get_context().get_stage()
ground = define_prim("/World/Ground", "Plane")
UsdGeom.Xformable(ground).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
cam = UsdGeom.Camera.Define(stage, "/World/SimpleCam")
cam.CreateFocalLengthAttr().Set(24.0)
cam.CreateHorizontalApertureAttr().Set(20.955)
cam.CreateVerticalApertureAttr().Set(15.29)
cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10.0))
xfc = UsdGeom.Xformable(cam.GetPrim())
xfc.AddTranslateOp().Set(Gf.Vec3d(0, -3, 1.5))
world.reset()
for _ in range(5):
    world.step(render=True)
verdict("bare stage built", True)

ok_all = True
try:
    import os as _os
    import glob as _glob
    import omni.replicator.core as rep  # noqa: E402
    _tmp = "/tmp/rep_empty_stage"
    _os.makedirs(_tmp, exist_ok=True)
    rp = rep.create.render_product("/World/SimpleCam", resolution=(640, 480))
    verdict("render_product created", True, getattr(rp, "path", rp))
    try:
        w = rep.writers.get("BasicWriter")
        try:
            w.initialize(output_dir=_tmp, rgb=True)
        except Exception as e:
            print("[INFO] writer initialize: %r" % e, flush=True)
        w.attach([getattr(rp, "path", rp)])
        verdict("BasicWriter attach (empty stage)", True)
    except Exception:
        verdict("BasicWriter attach (empty stage)", False)
        print(traceback.format_exc(limit=6), flush=True)
        ok_all = False
        w = None
    if w is not None:
        for _ in range(12):
            rep.orchestrator.step(rt_subframes=2)
        cand = sorted(_glob.glob(_os.path.join(_tmp, "**/*.png"), recursive=True))
        verdict("PNG produced (empty stage)", len(cand) > 0, str(cand[:1]))
        ok_all = ok_all and len(cand) > 0
except Exception:
    verdict("empty-stage rep path", False)
    print(traceback.format_exc(limit=10), flush=True)
    ok_all = False

print("[RESULT] %s" % ("PASS - scene-dependent, workaround path exists"
                        if ok_all else "FAIL - systemic install fault"), flush=True)
sim.close()
sys.exit(0 if ok_all else 1)
