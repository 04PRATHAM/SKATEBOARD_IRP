"""Viewport-product-fed ROS helper probe (M12 Phase A2).

Last untested combination: OG `ROS2CameraHelper` nodes fed by a VIEWPORT render
product (via Python viewport API) instead of `IsaacCreateRenderProduct`.
All installed, supported APIs; production bridge untouched.

Success signal: NO `unknown dtype` writer errors after graph build + 60 rendered
steps (vs 4 errors in the production path). Data flow would then be validated
externally with ros2 checks in a follow-up run.

Run: /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/probe_ros_viewport_product.py --headless
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
win = vu.create_viewport_window(name="ROSProbe", width=640, height=480,
                                camera_path=Sdf.Path("/World/SimpleCam"))
api = getattr(win, "viewport_api", win)
rp_path = str(api.render_product_path)
verdict("viewport product path", rp_path.startswith("/Render"), rp_path)

import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
try:
    import omni.kit.app as _kapp  # noqa: E402
    _em = _kapp.get_app().get_extension_manager()
    if not _em.is_extension_enabled("isaacsim.ros2.bridge"):
        _em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    print("[INFO] isaacsim.ros2.bridge extension enabled", flush=True)
except Exception as e:
    print("[WARN] bridge extension enable failed: %r" % e, flush=True)
try:
    og.Controller.edit(
        {"graph_path": "/ProbeROS", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RGB.inputs:topicName", "/probe/rgb"),
                ("RGB.inputs:type", "rgb"),
                ("RGB.inputs:frameId", "probe_frame"),
                ("RGB.inputs:renderProductPath", rp_path),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RGB.inputs:execIn"),
            ],
        },
    )
    verdict("probe OG graph built", True, "/ProbeROS + viewport product (no CreateRenderProduct)")
except Exception:
    verdict("probe OG graph built", False)
    print(traceback.format_exc(limit=8), flush=True)

for _ in range(60):
    world.step(render=True)
print("[NOTE] inspect sim log above: zero 'unknown dtype' lines => attach path differs", flush=True)
print("[PROBE-DONE]", flush=True)
sim.close()
