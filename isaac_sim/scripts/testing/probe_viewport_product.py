"""Viewport-backed render-product probe (Milestone 11, new variable: product source).

Camera-direct HydraTextures products carry no valid resolution binding (the exact
failing writer attribute). A viewport-backed product may differ. Enumerates
render-product paths exposed by a dedicated viewport window bound to the camera
and attempts BasicWriter attach on each candidate.

PASS bar: attach succeeds AND a PNG is produced for at least one candidate.
FAIL: same dtype error on all candidates (viewport path ruled out too).

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_viewport_product.py --headless
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
define_prim("/World/Ground", "Plane")
cam = UsdGeom.Camera.Define(stage, "/World/SimpleCam")
cam.CreateFocalLengthAttr().Set(24.0)
cam.CreateHorizontalApertureAttr().Set(20.955)
cam.CreateVerticalApertureAttr().Set(15.29)
cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10.0))
UsdGeom.Xformable(cam.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, -3, 1.5))
world.reset()
for _ in range(5):
    world.step(render=True)

import omni.kit.viewport.utility as vu  # noqa: E402
from pxr import Sdf as _Sdf  # noqa: E402
win = vu.create_viewport_window(name="VPProbe", width=640, height=480,
                                camera_path=_Sdf.Path("/World/SimpleCam"))
verdict("viewport window created", win is not None)
api = getattr(win, "viewport_api", win)
names = [n for n in dir(api) if "ender" in n.lower() or "roduct" in n.lower()
         or "ydra" in n.lower() or "exture" in n.lower()]
print("    render-ish attrs: %s" % names, flush=True)
candidates = []
for n in names:
    try:
        v = getattr(api, n)
        val = v() if callable(v) else v
        print("    %s = %r" % (n, str(val)[:120]), flush=True)
        s = str(val)
        if s.startswith("/Render"):
            candidates.append(s)
    except Exception as e:
        print("    %s raised %r" % (n, e), flush=True)
# ViewportAPI commonly exposes .render_product_path directly
try:
    rpp = api.render_product_path
    print("    render_product_path = %r" % (rpp,), flush=True)
    if str(rpp).startswith("/Render") and str(rpp) not in candidates:
        candidates.append(str(rpp))
except Exception as e:
    print("    render_product_path raised %r" % e, flush=True)
print("    candidates: %s" % candidates, flush=True)

any_ok = False
if candidates:
    import os as _os
    import glob as _glob
    import omni.replicator.core as rep  # noqa: E402
    for i, rp_path in enumerate(dict.fromkeys(candidates)):
        _tmp = "/tmp/rep_vp_%d" % i
        _os.makedirs(_tmp, exist_ok=True)
        try:
            w = rep.WriterRegistry.get("BasicWriter")(output_dir=_tmp, rgb=True)
            w.attach([rp_path])
            verdict("BasicWriter attach %s" % rp_path, True)
            for _ in range(12):
                rep.orchestrator.step(rt_subframes=2)
            pngs = sorted(_glob.glob(_os.path.join(_tmp, "**/*.png"), recursive=True))
            got = len(pngs) > 0
            verdict("PNG produced %s" % rp_path, got, str(pngs[:1]))
            any_ok = any_ok or got
        except Exception:
            verdict("BasicWriter attach %s" % rp_path, False)
            print(traceback.format_exc(limit=5), flush=True)
else:
    print("    no render-product candidates exposed; viewport path untestable", flush=True)

print("[RESULT] %s" % ("PASS - viewport-backed product works" if any_ok
                        else "FAIL - viewport path ruled out too"), flush=True)
sim.close()
sys.exit(0 if any_ok else 1)
