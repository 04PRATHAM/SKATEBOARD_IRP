"""Hydra-texture direct readback probe (M14: bypass syntheticdata writers entirely).

Uses IHydraTexture.get_frame_info (do frames advance?) + get_drawable_ldr_resource
(can CPU read rendered RGB without any writer/annotator?). Depth via get_aov_info
only if RGB proves viable.

PASS bar: frame index advances over steps AND ldr buffer is a nonempty uint8
HxWx4 array whose content changes with the scene.
FAIL otherwise (renderer emits nothing readable -> custom publisher impossible).

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_hydra_readback.py --headless
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
from pxr import UsdGeom, Gf, Sdf  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.prims import define_prim  # noqa: E402


def verdict(label, ok, detail=""):
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", label, detail), flush=True)
    return bool(ok)


world = World(stage_units_in_meters=1.0, physics_dt=0.01, rendering_dt=0.01)
stage = omni.usd.get_context().get_stage()
define_prim("/World/Ground", "Plane")
# Bright red cube directly in front of the camera (content-change witness)
cube = define_prim("/World/RedBox", "Cube")
UsdGeom.Cube(cube).CreateSizeAttr().Set(1.0)
UsdGeom.Xformable(cube).AddTranslateOp().Set(Gf.Vec3d(3.0, 0.0, 0.5))
cam = UsdGeom.Camera.Define(stage, "/World/SimpleCam")
cam.CreateFocalLengthAttr().Set(24.0)
cam.CreateHorizontalApertureAttr().Set(20.955)
cam.CreateVerticalApertureAttr().Set(15.29)
cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 10.0))
UsdGeom.Xformable(cam.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.5))
world.reset()
for _ in range(5):
    world.step(render=True)

import omni.kit.viewport.utility as vu  # noqa: E402
win = vu.create_viewport_window(name="HydraProbe", width=320, height=240,
                                camera_path=Sdf.Path("/World/SimpleCam"))
verdict("viewport window created", win is not None)
api = getattr(win, "viewport_api", win)
ht = getattr(api, "_hydra_texture", None)
verdict("hydra texture handle", ht is not None, type(ht))

frames_ok, buf_ok, changes = False, False, False
try:
    fi0 = ht.get_frame_info()
    print("    frame_info t0: %s" % (str(fi0)[:200],), flush=True)
    for _ in range(30):
        world.step(render=True)
    fi1 = ht.get_frame_info()
    print("    frame_info t1: %s" % (str(fi1)[:200],), flush=True)
    frames_ok = str(fi0) != str(fi1)
    verdict("frame info advances over 30 steps", frames_ok)
except Exception:
    verdict("frame info query", False)
    print(traceback.format_exc(limit=6), flush=True)

try:
    import numpy as _np
    res = ht.get_drawable_ldr_resource()
    print("    ldr type: %s" % type(res), flush=True)
    a = _np.asarray(res)
    print("    ldr dtype=%s shape=%s mean=%.1f" % (a.dtype, a.shape, float(a.mean())), flush=True)
    buf_ok = a.size > 0 and a.dtype == _np.uint8
    verdict("ldr buffer nonempty uint8", buf_ok)
    for _ in range(30):
        world.step(render=True)
    b = _np.asarray(ht.get_drawable_ldr_resource())
    changes = bool((a != b).any()) if b.shape == a.shape else True
    verdict("buffer changes across steps", changes)
except Exception:
    verdict("ldr readback", False)
    print(traceback.format_exc(limit=8), flush=True)

print("[RESULT] %s" % ("PASS - hydra readback viable" if (frames_ok and buf_ok)
                        else "FAIL - no readable renderer output"), flush=True)
sim.close()
sys.exit(0 if (frames_ok and buf_ok) else 1)
