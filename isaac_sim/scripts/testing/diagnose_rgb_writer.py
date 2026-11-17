"""Minimal RGB writer diagnostic (SKATEBOARD_IRP Phase 4).

Isolates WHERE the RGB pipeline stops, without ROS (rclpy cannot import in-sim):

  L1  Camera API: isaacsim.sensors.camera.Camera on the vehicle RGBD prim,
      get_rgb()/get_depth() after rendered steps -> type/dtype/shape/stats.
      (Answers: is the camera rendering? before-or-after ROS?)
  L2  Bridge graph introspection: setup_ros2_bridge() then read back the
      /ROS2_Bridge graph (CreateRenderProduct renderProductPath, helper attrs).
      (Answers: does the failure happen inside the writer/OG attach?)

Run:
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/diagnose_rgb_writer.py --headless [--with-bridge] [--steps 60]
  DISPLAY=:1 ... --gui  (comparison; needs X)

New file because no existing script isolates the camera from the bridge:
check_rgbd_topics.py needs live ROS topics; run_fidelity_* test full stacks.
"""
import argparse
import sys
import traceback

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")

parser = argparse.ArgumentParser(description="RGB writer diagnostic (no ROS)")
parser.add_argument("--headless", action="store_true", default=False)
parser.add_argument("--gui", action="store_true", default=False)
parser.add_argument("--with-bridge", action="store_true", default=False)
parser.add_argument("--skip-l1", action="store_true", default=False,
                    help="skip Camera-API init (clean OG-path isolation)")
parser.add_argument("--steps", type=int, default=60)
args = parser.parse_args()
headless = args.headless and not args.gui
if args.gui:
    headless = False


def verdict(label, ok, detail=""):
    print("[%s] %s %s" % ("OK  " if ok else "FAIL", label, detail), flush=True)
    return bool(ok)


print("=== D0 environment ===", flush=True)
verdict("headless=%s with_bridge=%s steps=%d" % (headless, args.with_bridge, args.steps), True)

from isaacsim import SimulationApp  # noqa: E402
sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting"})
for _ in range(3):
    try:
        sim.update()
    except Exception:
        pass

from isaac_sim.scripts.utils.world_setup import build_world, RGBD_CAMERA_PRIM  # noqa: E402
from isaac_sim.scripts.utils.config_loader import load_config  # noqa: E402

camera_cfg = load_config("configs/camera.yaml")["camera"]
sim_cfg = load_config("configs/simulation.yaml")["simulation"]
built = build_world(dynamic=True, with_cameras=False, with_ros=False,
                    start_xy=(6.0, -6.0), glass_has_collision=True, create_world=True)
stage, world = built["stage"], built["world"]
try:
    built["articulation"].initialize()
except Exception as e:
    print("[WARN] articulation init: %r" % e, flush=True)
world.reset()
verdict("world built, RGBD prim valid",
        stage.GetPrimAtPath(RGBD_CAMERA_PRIM).IsValid(), RGBD_CAMERA_PRIM)

print("=== L1 Camera API (no ROS, no bridge) ===", flush=True)
l1 = {}
if args.skip_l1:
    print("--- L1 Camera init skipped (--skip-l1): warmup steps only ---", flush=True)
    try:
        import omni.timeline as _tlskip  # noqa: E402
        _tlskip.get_timeline_interface().play()
    except Exception as e:
        print("[WARN] timeline play: %r" % e, flush=True)
    for _ in range(10):
        world.step(render=True)
    l1["init"] = None
    cam = None
else:
    cam = None
    # Blessed 5.x pattern per NVIDIA (camera_annotator_device.py): explicit
    # annotator_device. Try cpu/cuda/None, all COLD (no warmup).
    for attempt in ("cpu-cold", "cuda-cold", "none-cold", "none-warmfallback"):
        if attempt == "none-warmfallback":
            try:
                import omni.timeline as _tl  # noqa: E402
                _tl.get_timeline_interface().play()
            except Exception as e:
                print("[WARN] timeline play: %r" % e, flush=True)
            for _ in range(10):
                world.step(render=True)
            print("--- retry after 10 rendered frames + timeline play ---", flush=True)
        _adev = {"cpu-cold": "cpu", "cuda-cold": "cuda"}.get(attempt, None)
        try:
            from isaacsim.sensors.camera import Camera  # noqa: E402
            cam = Camera(prim_path=RGBD_CAMERA_PRIM,
                         resolution=(int(camera_cfg.get("image_width", 640)),
                                     int(camera_cfg.get("image_height", 480))),
                         annotator_device=_adev)
            cam.initialize()
            verdict("Camera init (%s)" % attempt, True)
            l1["init"] = True
            l1["adev"] = _adev
            break
        except Exception:
            verdict("Camera init (%s)" % attempt, False)
            print(traceback.format_exc(limit=8), flush=True)
            l1["init"] = False

if l1.get("init"):
    try:
        import numpy as np  # noqa: E402
        try:
            import omni.timeline as _tl2  # noqa: E402
            _tl2.get_timeline_interface().play()
            print("    timeline playing before poll loop", flush=True)
        except Exception as e:
            print("    timeline play failed: %r" % e, flush=True)
        try:
            print("    render_product: %s" % getattr(cam, "_render_product_path", "?"), flush=True)
        except Exception:
            pass
        rgb, depth = None, None
        for i in range(max(args.steps, 120)):
            world.step(render=True)
            if rgb is None and (i % 5 == 0):
                try:
                    rgb = cam.get_rgb()
                    if rgb is not None:
                        print("    first rgb at step %d" % i, flush=True)
                except Exception as e:
                    print("    get_rgb raised at step %d: %r" % (i, e), flush=True)
                    break
            if rgb is not None and depth is None and (i % 5 == 0):
                try:
                    depth = cam.get_depth()
                    if depth is not None:
                        print("    first depth at step %d" % i, flush=True)
                except Exception as e:
                    print("    get_depth raised at step %d: %r" % (i, e), flush=True)
                    break
            if rgb is not None and depth is not None:
                break
        verdict("get_rgb returns data", rgb is not None, type(rgb))
        if rgb is not None:
            a = np.asarray(rgb)
            print("    rgb type=%s dtype=%s shape=%s min=%s max=%s mean=%s" % (
                type(rgb), a.dtype, a.shape,
                float(a.min()) if a.size else "?", float(a.max()) if a.size else "?",
                float(a.mean()) if a.size else "?"), flush=True)
            l1["rgb_shape"] = str(a.shape)
            l1["rgb_dtype"] = str(a.dtype)
            verdict("rgb 640x480x3 uint8", a.shape == (480, 640, 3), "%s %s" % (a.shape, a.dtype))
            verdict("rgb non-empty (variance>0)", float(a.var()) > 0.0)
        try:
            depth = cam.get_depth()
            verdict("get_depth returns data", depth is not None, type(depth))
            if depth is not None:
                d = np.asarray(depth).astype("float64")
                fin = np.isfinite(d)
                print("    depth dtype=%s shape=%s finite_frac=%.3f median=%.2f" % (
                    d.dtype, d.shape, float(fin.mean()),
                    float(np.median(d[fin])) if fin.any() else float("nan")), flush=True)
        except Exception:
            verdict("get_depth", False)
            print(traceback.format_exc(), flush=True)
    except Exception:
        verdict("L1 capture loop", False)
        print(traceback.format_exc(), flush=True)

if args.with_bridge:
    print("=== L2 bridge graph introspection ===", flush=True)
    try:
        from isaac_sim.scripts.bridge.ros2_bridge import setup_ros2_bridge  # noqa: E402
        gpath = setup_ros2_bridge(RGBD_CAMERA_PRIM, camera_cfg, sim_cfg,
                                  vehicle_path="/World/Vehicle")
        verdict("setup_ros2_bridge returned", True, gpath)
        try:
            import omni.graph.core as og  # noqa: E402
            g = og.get_graph_by_path(gpath)
            names = sorted([n.get_name() for n in g.get_nodes()]) if g else []
            print("    graph nodes: %s" % names, flush=True)
            verdict("graph has RGB+Depth+Infos+Clock+TF",
                    all(k in names for k in ("RGB", "Depth", "RGBInfo", "DepthInfo", "Clock", "PublishTF")))
        except Exception:
            verdict("graph introspection", False)
            print(traceback.format_exc(), flush=True)
        for _ in range(max(args.steps, 30)):
            world.step(render=True)
        print("    (post-bridge steps done; inspect sim log above for writer-attach errors)", flush=True)
    except Exception:
        verdict("setup_ros2_bridge", False)
        print(traceback.format_exc(), flush=True)

print("=== L3b viewport bound to RGBD camera, then writer attach ===", flush=True)
try:
    import omni.kit.viewport.utility as _vu  # noqa: E402
    _vp = _vu.get_active_viewport()
    verdict("active viewport exists", _vp is not None)
    if _vp is not None:
        try:
            _vp.camera_path = RGBD_CAMERA_PRIM
            verdict("viewport camera bound to RGBD", True)
        except Exception:
            verdict("viewport camera bound to RGBD", False)
            print(traceback.format_exc(limit=6), flush=True)
        for _ in range(10):
            world.step(render=True)
        try:
            import omni.replicator.core as rep2  # noqa: E402
            _rp2 = rep2.create.render_product(RGBD_CAMERA_PRIM, resolution=(640, 480),
                                              force_new=True)
            _rp2p = getattr(_rp2, "path", _rp2)
            _w2 = rep2.WriterRegistry.get("BasicWriter")(output_dir=_tmp, rgb=True)
            _w2.attach([_rp2p])
            verdict("BasicWriter attach (viewport-bound)", True)
            for _ in range(12):
                rep2.orchestrator.step(rt_subframes=2)
            cand2 = sorted(_glob.glob(_os.path.join(_tmp, "**/*.png"), recursive=True))
            verdict("BasicWriter PNG produced (viewport-bound)", len(cand2) > 0,
                    str(cand2[:2]))
        except Exception:
            verdict("BasicWriter attach (viewport-bound)", False)
            print(traceback.format_exc(limit=8), flush=True)
except Exception:
    verdict("L3b viewport path", False)
    print(traceback.format_exc(limit=10), flush=True)

print("=== L4 dedicated viewport window + buffer capture ===", flush=True)
try:
    import omni.kit.viewport.utility as _vu4  # noqa: E402
    from pxr import Sdf as _Sdf4  # noqa: E402
    _vp4 = _vu4.create_viewport_window(
        name="RGBD_Diag", width=640, height=480,
        camera_path=_Sdf4.Path(RGBD_CAMERA_PRIM))
    verdict("create_viewport_window(RGBD)", _vp4 is not None)
    _api4 = _vp4.viewport_api if hasattr(_vp4, "viewport_api") else _vp4
    try:
        _aovs = _vu4.get_available_aovs_for_viewport(_api4)
        print("    available AOVs: %s" % (list(_aovs)[:12] if _aovs else _aovs), flush=True)
    except Exception as e:
        print("    AOV query failed: %r" % e, flush=True)
    for _ in range(10):
        world.step(render=True)
    _got = {}
    def _cb(buffer, _st=_got):
        try:
            import numpy as _np4
            _st["arr"] = _np4.asarray(buffer)
        except Exception as e:
            _st["err"] = repr(e)
    try:
        _vu4.capture_viewport_to_buffer(_api4, _cb)
        import time as _t4
        _t0 = _t4.time()
        _tries = 0
        while "arr" not in _got and "err" not in _got and _t4.time() - _t0 < 480.0:
            for _ in range(30):
                world.step(render=True)
            _tries += 1
            print("    capture wait %.0fs (try %d)" % (_t4.time() - _t0, _tries), flush=True)
            try:
                _vu4.capture_viewport_to_buffer(_api4, _cb)
            except Exception as e:
                print("    recapture raised: %r" % e, flush=True)
                break
        if "arr" in _got:
            import numpy as _np4b
            _a = _np4b.asarray(_got["arr"])
            print("    buffer dtype=%s shape=%s mean=%.1f var=%.1f" % (
                _a.dtype, _a.shape, float(_a.mean()), float(_a.var())), flush=True)
            verdict("viewport buffer RGB captured", _a.size > 0 and float(_a.var()) > 0.0)
        else:
            verdict("viewport buffer RGB captured", False, _got.get("err", "timeout"))
    except Exception:
        verdict("capture_viewport_to_buffer", False)
        print(traceback.format_exc(limit=8), flush=True)
except Exception:
    verdict("L4 viewport path", False)
    print(traceback.format_exc(limit=10), flush=True)

print("=== DONE (sim left open only in --gui; headless exits) ===", flush=True)
print("=== L3 rep render_product + BasicWriter + orchestrator ===", flush=True)
try:
    import os as _os
    import glob as _glob
    import omni.replicator.core as rep  # noqa: E402
    _tmp = "/tmp/rep_diag_l3"
    _os.makedirs(_tmp, exist_ok=True)
    for _f in _glob.glob(_os.path.join(_tmp, "**"), recursive=True):
        try:
            if _os.path.isfile(_f):
                _os.remove(_f)
        except Exception:
            pass
    rp = rep.create.render_product(RGBD_CAMERA_PRIM, resolution=(640, 480))
    rp_path = getattr(rp, "path", rp)
    print("    rep render_product: %s" % rp_path, flush=True)
    verdict("rep.create.render_product", bool(rp_path))
    try:
        writer = rep.WriterRegistry.get("BasicWriter")
        w = writer(output_dir=_tmp, rgb=True) if callable(writer) else writer
        w.attach([rp_path])
        verdict("BasicWriter attach", True)
    except Exception:
        verdict("BasicWriter attach", False)
        print(traceback.format_exc(limit=6), flush=True)
        w = None
    if w is not None:
        for _ in range(12):
            rep.orchestrator.step(rt_subframes=2)
        cand = sorted(_glob.glob(_os.path.join(_tmp, "*.png")) +
                      _glob.glob(_os.path.join(_tmp, "rgb/*.png")) +
                      _glob.glob(_os.path.join(_tmp, "**/*.png"), recursive=True))
        verdict("BasicWriter PNG produced", len(cand) > 0, str(cand[:2]))
        if cand:
            try:
                import numpy as _np  # noqa: E402
                from PIL import Image as _Image  # noqa: E402
                _a = _np.asarray(_Image.open(cand[0]).convert("RGB"))
                print("    png dtype=%s shape=%s mean=%.1f var=%.1f" % (
                    _a.dtype, _a.shape, float(_a.mean()), float(_a.var())), flush=True)
                verdict("png 640x480x3 uint8 non-empty",
                        _a.shape == (480, 640, 3) and float(_a.var()) > 0.0)
            except Exception:
                verdict("png decode", False)
                print(traceback.format_exc(limit=6), flush=True)
except Exception:
    verdict("L3 rep path", False)
    print(traceback.format_exc(limit=10), flush=True)

try:
    sim.close()
except Exception:
    pass
