#!/usr/bin/env python3
"""
Real S3 human IDLE test with skeletal proof (SKATEBOARD_IRP).

Proves (headless-deterministic):
  T-H1..H5  character/SkelRoot/mesh/skeleton structure (as before)
  T-H6      runtime Character valid (STRICT — no fallback exists anymore)
  T-H7      Action variable settable
  T-H8      height ~1.4-2.2 m
  T-H9      feet near ground
  T-H10     center near spawn
  T-H11     arms hang (no T-pose): shoulder->hand drop > 0.25 m, both arms
  T-H12     idle micro-motion: foot/hand root-relative ROM > 4 mm over 3 s

AnimationGraph discovery requires the wired characters to be present at stage
attach: the stage is exported + reopened after wiring (proven diag1-diag23),
then the runtime Character is acquired and Idle is driven by the real graph.

Run:
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_character.py --headless
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_character.py --gui   # stays open
"""

import sys
import math
import argparse

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def main(headless: bool = True, capture_path: str = "/home/ubuntu/SKATEBOARD_IRP/data/real_human_idle.png"):
    from isaacsim import SimulationApp
    sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting",
                         "width": 1280, "height": 720})
    app = __import__("omni.kit.app", fromlist=["get_app"]).get_app()
    for _ in range(5):
        app.update()

    from isaac_sim.scripts.assets.real_human import (
        enable_real_human_extensions, get_asset_root, get_character_asset_path,
        RealHuman, RealHumanConfig, HumanType,
        save_and_reopen_stage_for_animation,
    )
    enable_real_human_extensions()

    import omni.usd
    import omni.timeline
    from pxr import Usd, UsdGeom, Gf, Sdf, UsdLux

    from isaacsim.core.utils.prims import define_prim
    stage = omni.usd.get_context().get_stage()

    try:
        from isaacsim.core.prims import GroundPlane
        GroundPlane(prim_path="/World/GroundPlane", z_position=0.0, visible=False)
    except Exception as e:
        import carb as _c; _c.log_warn(f"GroundPlane fallback: {e}")

    floor = define_prim("/World/Floor", "Xform")
    fp = define_prim("/World/Floor/Visual", "Cube")
    UsdGeom.Cube(fp).CreateSizeAttr().Set(1.0)
    x = UsdGeom.Xformable(fp)
    x.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.02))
    x.AddScaleOp().Set(Gf.Vec3d(30, 30, 0.04))
    c = fp.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    c.Set([Gf.Vec3f(0.85, 0.85, 0.86)])

    dl = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dl.CreateIntensityAttr(4000)
    dist = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    dist.CreateIntensityAttr(15000)
    try:
        dist.CreateAngleAttr(2.0)
    except Exception:
        pass

    cam_path = "/World/LabCamera"
    cam = define_prim(cam_path, "Camera")
    cam.GetAttribute("focalLength").Set(35.0)
    cam.GetAttribute("horizontalAperture").Set(36.0)
    from isaac_sim.scripts.utils.aim import aim_prim
    aim_prim(stage, cam_path, Gf.Vec3d(3.5, -2.8, 1.7), Gf.Vec3d(0, 0, 0.85))

    print("[INFO] Spawning real human character (idle)...", flush=True)
    print("[INFO] asset_root=%s" % get_asset_root(), flush=True)
    print("[INFO] character_asset=%s" % get_character_asset_path(0), flush=True)

    cfg = RealHumanConfig(
        pedestrian_id="TestHuman_Idle",
        position=(0.0, 0.0, 0.0),
        yaw_deg=0.0,
        variant_idx=0,
        human_type=HumanType.STATIONARY,
    )
    human = RealHuman(stage, cfg)
    for _ in range(20):
        app.update()

    # Proven discovery sequence: export + reopen, then refresh handles.
    print("[INFO] Exporting + reopening stage for AnimationGraph discovery...", flush=True)
    stage = save_and_reopen_stage_for_animation(stage)
    human.refresh_after_reopen(stage)

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(30):
        app.update()

    ok_idle = human.go_idle()
    print("[%s] go_idle() -> %s (graph-only, no fallback)" % ("PASS" if ok_idle else "FAIL", ok_idle), flush=True)

    import omni.anim.graph.core as ag
    import carb
    ch = human._ensure_runtime_character()
    print("[%s] runtime Character %s" % ("PASS" if ch is not None else "FAIL",
                                        str(ch)[:60] if ch else "None"), flush=True)

    # joint names from composed skeleton
    joint_names = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(human.character_prim_path)):
        if p.GetTypeName() == "Skeleton":
            try:
                joint_names = [str(j) for j in p.GetAttribute("joints").Get()]
            except Exception:
                pass
            break

    def leaf(*cands):
        for cd in cands:
            for j in joint_names:
                if cd.lower() == j.split("/")[-1].lower():
                    return j.split("/")[-1]
        return None

    PROBES = {
        "L_uarm": leaf("L_Upperarm"), "R_uarm": leaf("R_Upperarm"),
        "L_hand": leaf("L_Hand"), "R_hand": leaf("R_Hand"),
        "L_foot": leaf("L_Foot"), "R_foot": leaf("R_Foot"),
        "L_thigh": leaf("L_Thigh"),
    }
    print("[INFO] probes=%s" % PROBES, flush=True)

    def snap():
        out = {}
        p = carb.Float3(0, 0, 0)
        r = carb.Float4(0, 0, 0, 0)
        for k, jn in PROBES.items():
            if not jn:
                continue
            try:
                ch.get_joint_transform(jn, p, r)
                out[k] = (float(p.x), float(p.y), float(p.z))
            except Exception:
                pass
        return out

    snaps = []
    for i in range(180):
        app.update()
        if i % 10 == 0:
            snaps.append(snap())

    def check(label, cond, detail=""):
        print("[%s] %s %s" % ("PASS" if cond else "FAIL", label, detail), flush=True)
        return bool(cond)

    checks = []
    chars_root = stage.GetPrimAtPath("/World/Characters")
    checks.append(check("T-H1 characters folder", chars_root and chars_root.IsValid()))
    h_prim = stage.GetPrimAtPath(human.character_prim_path)
    checks.append(check("T-H2 human Xform exists", h_prim and h_prim.IsValid(), human.character_prim_path))
    checks.append(check("T-H3 SkelRoot valid (real skeleton)", human.is_skelroot_valid(), human._skelroot_path))
    mesh_count = 0
    if h_prim and h_prim.IsValid():
        for p in Usd.PrimRange(h_prim):
            if p.GetTypeName() == "Mesh":
                mesh_count += 1
    checks.append(check("T-H4 body Mesh prims (skinned, not capsules)", mesh_count >= 1,
                        "Mesh count=%d" % mesh_count))
    checks.append(check("T-H5 skeleton joints present", len(joint_names) >= 30,
                        "njoints=%d" % len(joint_names)))
    checks.append(check("T-H6 runtime Character valid (STRICT)", ch is not None))
    try:
        avec = ch.get_variable("Action") if ch is not None else None
        checks.append(check("T-H7 anim graph Action variable", avec is not None, str(avec)[:60]))
    except Exception as e:
        checks.append(check("T-H7 anim graph Action variable", False, str(e)[:100]))

    try:
        from pxr import UsdGeom as _UG2
        skel = stage.GetPrimAtPath(human._skelroot_path) if human._skelroot_path else None
        target = skel if skel and skel.IsValid() else h_prim
        bbox = _UG2.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_]
                              ).ComputeWorldBound(target)
        box = bbox.ComputeAlignedBox()
        bmin, bmax = box.GetMin(), box.GetMax()
        height = float(bmax[2] - bmin[2])
        foot_z = float(bmin[2])
        cx, cy = float((bmin[0] + bmax[0]) * 0.5), float((bmin[1] + bmax[1]) * 0.5)
        checks.append(check("T-H8 height ~1.4-2.2m (real proportions)", 1.4 < height < 2.2,
                            "height=%.2fm" % height))
        checks.append(check("T-H9 feet near ground (foot_z ≈ 0)", abs(foot_z) < 0.22,
                            "foot_z=%.3fm" % foot_z))
        checks.append(check("T-H10 center near spawn", abs(cx) < 1.0 and abs(cy) < 1.0,
                            "center=(%.2f, %.2f)" % (cx, cy)))
    except Exception as e:
        checks.append(check("T-H8 height", False, str(e)[:100]))
        checks.append(check("T-H9 feet", False))
        checks.append(check("T-H10 center", False))

    # T-H11: no T-pose — arms hang (upperarm -> hand drops > 0.25 m, lateral < 0.6)
    tpose_ok = True
    for side in ["L", "R"]:
        a = snaps[-1].get(f"{side}_uarm")
        b = snaps[-1].get(f"{side}_hand")
        if a is None or b is None:
            tpose_ok = False
            continue
        drop = a[2] - b[2]
        lat = math.hypot(b[0] - a[0], b[1] - a[1])
        print("[INFO] %s arm drop=%.3f lateral=%.3f" % (side, drop, lat), flush=True)
        if not (drop > 0.25 and lat < 0.6):
            tpose_ok = False
    checks.append(check("T-H11 arms hang naturally (no T-pose)", tpose_ok))

    # T-H12: idle micro-motion (graph alive, not frozen bind pose)
    rom_max = 0.0
    for k in snaps[0]:
        pts = [s[k] for s in snaps if k in s]
        if len(pts) > 1:
            rom_max = max(rom_max, max(math.dist(pts[0], p) for p in pts[1:]))
    checks.append(check("T-H12 idle joints move (graph alive)", rom_max > 0.004,
                        "maxROM=%.4fm" % rom_max))

    all_pass = all(checks) and ok_idle
    print("", flush=True)
    print("[RESULT] %d/%d human-idle checks %s" % (sum(checks), len(checks), "PASS" if all_pass else "FAIL"), flush=True)

    if not headless:
        try:
            capture_rgb(stage, cam_path, capture_path, None)
            print("[INFO] screenshot saved: %s" % capture_path, flush=True)
        except Exception as e:
            print("[WARN] screenshot capture failed: %r" % e, flush=True)
        print("[INFO] GUI hold-open for inspection (close window to exit)...", flush=True)
        try:
            while sim.is_running():
                app.update()
        except KeyboardInterrupt:
            pass
    else:
        print("[INFO] headless: no screenshot (placeholders are not evidence).", flush=True)

    sim.close()
    return 0 if all_pass else 1


def capture_rgb(stage, cam_path, out_path, world=None):
    """Render one RGB frame from cam_path via Replicator and save as PNG."""
    import os
    import omni.replicator.core as rep
    rp = rep.create.render_product(str(cam_path), resolution=(1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter")
    out_dir = os.path.dirname(out_path) or "."
    os.makedirs(out_dir, exist_ok=True)
    tmp_dir = "/tmp/rep_idle_capture"
    os.makedirs(tmp_dir, exist_ok=True)
    try:
        w = writer(output_dir=tmp_dir, rgb=True) if callable(writer) else writer
        if hasattr(w, "attach"):
            w.attach([rp])
        else:
            w = rep.WriterRegistry.create_writer("BasicWriter", output_dir=tmp_dir, rgb=True)
            w.attach([rp])
    except Exception as _e:
        import carb as _c; _c.log_warn(f"BasicWriter attach fallback: {_e}")
        try:
            writer.attach([rp])
        except Exception:
            pass
    for _ in range(12):
        rep.orchestrator.step(rt_subframes=2)
    try:
        import omni.kit.viewport.utility as vp_util
        import asyncio
        async def do_cap():
            from omni.kit.viewport.utility import capture_viewport_to_file
            await capture_viewport_to_file(out_path)
        asyncio.run(do_cap())
        if os.path.exists(out_path):
            return
    except Exception:
        pass
    import glob
    cand = sorted(glob.glob(os.path.join(tmp_dir, "*.png")) +
                  glob.glob(os.path.join(tmp_dir, "rgb/*.png")) +
                  glob.glob(os.path.join(tmp_dir, "**/*.png"), recursive=True))
    if cand:
        import shutil
        shutil.copy(cand[0], out_path)
        return
    raise RuntimeError("viewport capture produced no image")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=False)
    parser.add_argument("--gui", action="store_true", default=False)
    args = parser.parse_args()
    hl = args.headless and not args.gui
    if args.gui:
        hl = False
    sys.exit(main(headless=hl))
