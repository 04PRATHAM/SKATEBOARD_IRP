"""Dtype interception probe (M15: exact runtime value at the failing .set()).

Reversible in-process instrumentation ONLY (installed sources untouched):
- wraps AttributeDataValueHelper.set to log declared attr type + exact
  new_value runtime type/dtype/shape/len, then delegates;
- wraps numpy.append to log operand dtypes/shapes and result dtype/shape.

Then performs the minimal failing attach (empty stage + camera + BasicWriter)
and prints the FIRST failing call's details.

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_dtype_intercept.py --headless
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
import numpy as _np  # noqa: E402
from pxr import UsdGeom, Gf  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.prims import define_prim  # noqa: E402

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

# ---- reversible interception (no installed-file edits) ----
from omni.graph.core._impl import attribute_values as _av  # noqa: E402

_orig_set = _av.AttributeDataValueHelper.set
_set_logged = {"n": 0}


def _fmt(v):
    try:
        import numpy as _n
        if isinstance(v, _n.ndarray):
            return "ndarray dtype=%s shape=%s len=%d" % (v.dtype, v.shape, len(v))
        return "%s len=%s" % (type(v), len(v) if hasattr(v, "__len__") else "?")
    except Exception as e:
        return "unformattable %r" % e


def _logged_set(self, new_value, on_gpu=False):
    _set_logged["n"] += 1
    if _set_logged["n"] <= 12:
        try:
            t = self.type
            tdesc = "base=%s array_depth=%s" % (getattr(t, "base_type", "?"),
                                               getattr(t, "array_depth", "?"))
        except Exception:
            tdesc = "?"
        print("[INTERCEPT] set#%d attr_type=(%s) value=%s" % (
            _set_logged["n"], tdesc, _fmt(new_value)), flush=True)
    return _orig_set(self, new_value, on_gpu=on_gpu)


_av.AttributeDataValueHelper.set = _logged_set

_orig_usd_set = _av.AttributeValueHelper.set


def _logged_usd_set(self, new_value, on_gpu=False, update_usd=False):
    _set_logged["n"] += 1
    if _set_logged["n"] <= 12:
        try:
            t = self.type
            tdesc = "base=%s array_depth=%s" % (getattr(t, "base_type", "?"),
                                               getattr(t, "array_depth", "?"))
        except Exception:
            tdesc = "?"
        print("[INTERCEPT-USD] set#%d attr_type=(%s) value=%s" % (
            _set_logged["n"], tdesc, _fmt(new_value)), flush=True)
    return _orig_usd_set(self, new_value, on_gpu=on_gpu, update_usd=update_usd)


_av.AttributeValueHelper.set = _logged_usd_set

_orig_append = _np.append
_app_logged = {"n": 0}


def _logged_append(arr, values, axis=None):
    _app_logged["n"] += 1
    out = _orig_append(arr, values, axis=axis)
    if _app_logged["n"] <= 12:
        def _d(x):
            import numpy as _n
            if isinstance(x, _n.ndarray):
                return "%s%s" % (x.dtype, x.shape)
            return "%s(%s)" % (type(x).__name__, str(x)[:40])
        print("[INTERCEPT] append#%d arr=%s values=%s -> %s%s" % (
            _app_logged["n"], _d(arr), _d(values), out.dtype, out.shape), flush=True)
    return out


_np.append = _logged_append

import omni.replicator.core as rep  # noqa: E402
rp = rep.create.render_product("/World/SimpleCam", resolution=(640, 480))
print("[INFO] product: %s" % getattr(rp, "path", rp), flush=True)
try:
    w = rep.writers.get("BasicWriter")
    try:
        w.initialize(output_dir="/tmp/rep_intercept", rgb=True)
    except Exception as e:
        print("[INFO] writer initialize: %r" % e, flush=True)
    w.attach([getattr(rp, "path", rp)])
    print("[RESULT] attach unexpectedly SUCCEEDED", flush=True)
except Exception:
    import traceback as _tb
    print("[RESULT] attach failed as expected; traceback:", flush=True)
    print(_tb.format_exc(limit=12), flush=True)
print("[INTERCEPT] total set calls=%d append calls=%d" % (_set_logged["n"], _app_logged["n"]), flush=True)
sim.close()
