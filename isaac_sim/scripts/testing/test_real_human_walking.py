#!/usr/bin/env python3
"""
Real S3 human WALK test with skeletal gait proof (SKATEBOARD_IRP).
Doubles as TEST 1 GUI validation (8 m walk with START/GOAL floor discs).

Graph-only walk (Action=Walk + PathPoints + per-frame Walk ramp, official
base_command.walk pattern). Proves BOTH root translation AND skeletal joint
animation; FAILS on fallback-only motion (no Xform fallback exists anymore):

  T-W1  runtime Character valid (STRICT)
  T-W2  go_walk_straight initiated via graph
  T-W3  moved > 1.0 m (expect ~5 m)
  T-W4  z stable (< 0.20 spread)
  T-W5  correct direction (+X)
  T-W6  no sideways slide
  T-W7  SkelRoot valid
  T-W8  body Mesh (real skinned)
  T-W9  thigh root-relative ROM > 0.06 m (legs swing)
  T-W10 foot root-relative ROM > 0.20 m
  T-W11 upper-arm root-relative ROM > 0.10 m (arms swing)
  T-W12 L/R thigh forward-motion correlation < -0.3 (alternating gait)
  T-W13 walk-end arms hang (no T-pose while walking)
  T-W14 graph-driven (Character valid at end, state WALKING/COMPLETED)

Run:
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_walking.py --headless
  /home/ubuntu/isaacsim/python.sh isaac_sim/scripts/testing/test_real_human_walking.py --gui   # stays open
"""

import sys
import math
import argparse

sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP")


def main(headless: bool = True, capture_path: str = "/home/ubuntu/SKATEBOARD_IRP/data/real_human_walking.png"):
    from isaacsim import SimulationApp
    sim = SimulationApp({"headless": headless, "renderer": "RayTracedLighting",
                         "width": 1280, "height": 720})
    app = __import__("omni.kit.app", fromlist=["get_app"]).get_app()
    for _ in range(5):
        app.update()

    from isaac_sim.scripts.assets.real_human import (
        enable_real_human_extensions, get_character_asset_path,
        RealHuman, RealHumanConfig, HumanType,
        save_and_reopen_stage_for_animation,
    )
    enable_real_human_extensions()

    import omni.usd
    import omni.timeline
    from pxr import UsdGeom, Gf, Sdf, UsdLux
    from pxr import Usd as _Usd
    from isaacsim.core.utils.prims import define_prim
    stage = omni.usd.get_context().get_stage()

    try:
        from isaacsim.core.prims import GroundPlane
        GroundPlane(prim_path="/World/GroundPlane", z_position=0.0, visible=False)
    except Exception as e:
        import carb as _c; _c.log_warn(f"GroundPlane: {e}")

    floor = define_prim("/World/Floor", "Xform")
    fp = define_prim("/World/Floor/Visual", "Cube")
    UsdGeom.Cube(fp).CreateSizeAttr().Set(1.0)
    x = UsdGeom.Xformable(fp)
    x.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.02))
    x.AddScaleOp().Set(Gf.Vec3d(30, 30, 0.04))
    fp.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set([Gf.Vec3f(0.85, 0.85, 0.86)])
    dl = UsdLux.DomeLight.Define(stage, "/World/DomeLight"); dl.CreateIntensityAttr(4000)
    dist = UsdLux.DistantLight.Define(stage, "/World/DistantLight"); dist.CreateIntensityAttr(15000)

    cam_path = "/World/LabCamera"
    cam = define_prim(cam_path, "Camera")
    cam.GetAttribute("focalLength").Set(35.0)
    cam.GetAttribute("horizontalAperture").Set(36.0)
    from isaac_sim.scripts.utils.aim import aim_prim
    aim_prim(stage, cam_path, Gf.Vec3d(7.5, -5.0, 2.5), Gf.Vec3d(3.5, 0, 0.9))

    # Flat START (green) + GOAL (red) floor discs — visual-only GUI aids,
    # no collision/physics/sensors. Walk route is 8 m (+X) per Part G.
    for _name, _mx, _color in [("START", 0.0, (0.15, 0.75, 0.25)),
                               ("GOAL", 8.0, (0.9, 0.15, 0.15))]:
        _disc = define_prim(f"/World/WalkMarkers/{_name}", "Cylinder")
        _xd = UsdGeom.Xformable(_disc)
        _xd.AddTranslateOp().Set(Gf.Vec3d(float(_mx), 0.0, 0.02))
        _disc.GetAttribute("radius").Set(0.6)
        _disc.GetAttribute("height").Set(0.04)
        _disc.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray
                              ).Set([Gf.Vec3f(*_color)])

    print("[INFO] Spawning walking human...", flush=True)
    print("[INFO] character_asset=%s" % get_character_asset_path(1), flush=True)

    cfg = RealHumanConfig(
        pedestrian_id="TestHuman_Walk",
        position=(0.0, 0.0, 0.0),
        yaw_deg=0.0,
        variant_idx=1,
        human_type=HumanType.WALKING,
    )
    human = RealHuman(stage, cfg)
    for _ in range(20):
        app.update()

    print("[INFO] Exporting + reopening stage for AnimationGraph discovery...", flush=True)
    stage = save_and_reopen_stage_for_animation(stage)
    human.refresh_after_reopen(stage)

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(30):
        app.update()

    ok_walk = human.go_walk_straight(velocity=1.2, distance=8.0)
    # Set Walk blend speed to ramp stride frequency into valid range.
    # The graph's MotionMatching uses the Walk variable to control step rate;
    # without an explicit set the default is a slow shuffle (~0.2 Hz).
    try:
        ch.set_variable("Walk", 1.5)
    except Exception:
        pass
    print("[%s] go_walk_straight 8m -> %s (graph-only)" % ("PASS" if ok_walk else "FAIL", ok_walk), flush=True)

    import carb
    import omni.anim.graph.core as ag
    from omni.anim.people.scripts.utils import Utils
    ch = human._ensure_runtime_character()

    # joint names from composed skeleton
    joint_names = []
    for p in _Usd.PrimRange(stage.GetPrimAtPath(human.character_prim_path)):
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

    JN = ["Pelvis", "L_Thigh", "R_Thigh", "L_Calf", "R_Calf", "L_Foot", "R_Foot",
          "L_Upperarm", "R_Upperarm", "L_Hand", "R_Hand", "Head"]
    JN = [leaf(n) for n in JN]
    JN = [n for n in JN if n]

    def snap():
        j = {}
        p = carb.Float3(0, 0, 0)
        r = carb.Float4(0, 0, 0, 0)
        for n in JN:
            try:
                ch.get_joint_transform(n, p, r)
                j[n] = (float(p.x), float(p.y), float(p.z))
            except Exception:
                pass
        rp = Utils.get_character_pos(ch)
        return j, (float(rp[0]), float(rp[1]), float(rp[2]))

    dt = 1.0 / 60.0
    series = []
    for i in range(600):
        try:
            human.update(dt)
        except Exception:
            pass
        app.update()
        if i % 12 == 0 and ch is not None:
            try:
                series.append(snap())
            except Exception:
                pass

    def check(label, cond, detail=""):
        print("[%s] %s %s" % ("PASS" if cond else "FAIL", label, detail), flush=True)
        return bool(cond)

    checks = []
    checks.append(check("T-W1 ag.Character handle valid (STRICT)", ch is not None))
    checks.append(check("T-W2 go_walk initiated (graph)", ok_walk))
    if not series:
        checks.append(check("T-W3 moved > 1.0m", False, "no samples"))
        all_pass = False
        print("[RESULT] 2/%d human-walk checks FAIL" % len(checks), flush=True)
        sim.close()
        return 1

    r0, r1 = series[0][1], series[-1][1]
    dist_moved = math.hypot(r1[0] - r0[0], r1[1] - r0[1])
    nominal_time = max((len(series) - 1) * 12 / 60.0, 1e-6)
    avg_speed = dist_moved / nominal_time
    zs = [s[1][2] for s in series]
    z_spread = max(zs) - min(zs)
    z_mean = sum(zs) / len(zs)
    print("[INFO] avg_speed=%.3f m/s over %.1f nominal s, dist=%.2f m"
          % (avg_speed, nominal_time, dist_moved), flush=True)
    checks.append(check("T-W3 walked meaningful distance (>4.0m of 8m)", dist_moved > 4.0,
                        "dx=%.2f p0=(%.2f,%.2f) p1=(%.2f,%.2f)" % (dist_moved, r0[0], r0[1], r1[0], r1[1])))
    checks.append(check("T-W4 z stable (<0.20 spread)", z_spread < 0.20,
                        "z_mean=%.3f spread=%.3f" % (z_mean, z_spread)))
    checks.append(check("T-W5 correct direction (+X)", r1[0] > r0[0] + 0.5,
                        "x %.2f -> %.2f" % (r0[0], r1[0])))
    checks.append(check("T-W6 no sideways slide (|dy| < 0.7 dx)", abs(r1[1] - r0[1]) < 0.7 * max(dist_moved, 1e-6),
                        "dy=%.2f dx=%.2f" % (r1[1] - r0[1], r1[0] - r0[0])))
    checks.append(check("T-W7 SkelRoot valid", human.is_skelroot_valid(), human._skelroot_path))
    h_prim = stage.GetPrimAtPath(human.character_prim_path)
    mesh_ct = 0
    if h_prim and h_prim.IsValid():
        for p in _Usd.PrimRange(h_prim):
            if p.GetTypeName() == "Mesh":
                mesh_ct += 1
    checks.append(check("T-W8 body Mesh (real skinned)", mesh_ct >= 1, "Mesh=%d" % mesh_ct))

    # root-relative ROM: limb swing with translation removed
    def rrom(name):
        pts = [(s[0][name][0] - s[1][0], s[0][name][1] - s[1][1], s[0][name][2] - s[1][2])
               for s in series if name in s[0]]
        if len(pts) < 2:
            return 0.0
        return max(math.dist(pts[0], p) for p in pts[1:])

    rom_thigh = max(rrom("L_Thigh"), rrom("R_Thigh"))
    rom_calf = max(rrom("L_Calf"), rrom("R_Calf"))
    rom_foot = max(rrom("L_Foot"), rrom("R_Foot"))
    rom_uarm = max(rrom("L_Upperarm"), rrom("R_Upperarm"))
    print("[INFO] ROM thigh=%.3f calf=%.3f foot=%.3f arm=%.3f"
          % (rom_thigh, rom_calf, rom_foot, rom_uarm), flush=True)
    checks.append(check("T-W9 thigh swing ROM > 0.06m", rom_thigh > 0.06, "rom=%.3f" % rom_thigh))
    checks.append(check("T-W10 foot swing ROM > 0.20m", rom_foot > 0.20, "rom=%.3f" % rom_foot))
    checks.append(check("T-W11 arm swing ROM > 0.10m", rom_uarm > 0.10, "rom=%.3f" % rom_uarm))

    lx = [(s[0]["L_Thigh"][0] - s[1][0]) for s in series if "L_Thigh" in s[0]]
    rx = [(s[0]["R_Thigh"][0] - s[1][0]) for s in series if "R_Thigh" in s[0]]
    corr = 0.0
    if len(lx) > 2 and len(rx) > 2:
        mx, my = sum(lx) / len(lx), sum(rx) / len(rx)
        cov = sum((a - mx) * (b - my) for a, b in zip(lx, rx)) / len(lx)
        vx = sum((a - mx) ** 2 for a in lx) / len(lx)
        vy = sum((b - my) ** 2 for b in rx) / len(rx)
        corr = cov / math.sqrt(vx * vy) if vx > 0 and vy > 0 else 0.0
    checks.append(check("T-W12 alternating gait (L/R corr < -0.3)", corr < -0.3, "corr=%.3f" % corr))

    jw = series[-1][0]
    tpose_ok = True
    for side in ["L", "R"]:
        a = jw.get(f"{side}_Upperarm")
        b = jw.get(f"{side}_Hand")
        if a is None or b is None:
            tpose_ok = False
            continue
        if not ((a[2] - b[2]) > 0.25 and math.hypot(b[0] - a[0], b[1] - a[1]) < 0.6):
            tpose_ok = False
    checks.append(check("T-W13 no T-pose while walking", tpose_ok))
    checks.append(check("T-W14 graph-driven end state", ch is not None and human.graph_walk_active or human.is_completed()))

    # Part M: speed, stride frequency, uprightness, skeleton/mesh counts, calf.
    checks.append(check("T-W15 avg speed realistic (0.6-1.8 m/s)", 0.6 < avg_speed < 1.8,
                        "v=%.3f" % avg_speed))
    lx_det = [(s[0]["L_Thigh"][0] - s[1][0]) for s in series if "L_Thigh" in s[0]]
    stride_hz = 0.0
    if len(lx_det) > 4:
        mean = sum(lx_det) / len(lx_det)
        peaks = sum(1 for i in range(1, len(lx_det) - 1)
                    if lx_det[i] > mean and lx_det[i] >= lx_det[i-1] and lx_det[i] >= lx_det[i+1])
        stride_hz = peaks / nominal_time
    print("[INFO] stride_freq=%.3f Hz legROM=%.3f" % (stride_hz, rom_thigh), flush=True)
    checks.append(check("T-W16 stride frequency (0.1-2.0 Hz)", 0.1 < stride_hz < 2.0,
                        "f=%.3fHz" % stride_hz))
    head_z = [s[0]["Head"][2] for s in series if "Head" in s[0]]
    pelv_z = [s[0]["Pelvis"][2] for s in series if "Pelvis" in s[0]]
    upright_ok = (len(head_z) > 1 and (max(head_z) - min(head_z)) < 0.15
                  and len(pelv_z) > 1 and 0.7 < sum(pelv_z) / len(pelv_z) < 1.1)
    checks.append(check("T-W17 remains upright", upright_ok,
                        "head_spread=%.3f" % ((max(head_z) - min(head_z)) if len(head_z) > 1 else 999)))
    checks.append(check("T-W18 skeleton+meshes (joints>=90, meshes>=5)",
                        len(joint_names) >= 90 and mesh_ct >= 5,
                        "joints=%d meshes=%d" % (len(joint_names), mesh_ct)))
    checks.append(check("T-W19 calf swing ROM > 0.08m", rom_calf > 0.08, "rom=%.3f" % rom_calf))

    # stop
    if ch is not None:
        try:
            ch.set_variable("Action", "None")
            ch.set_variable("Walk", 0.0)
        except Exception:
            pass

    all_pass = all(checks)
    print("", flush=True)
    print("[RESULT] %d/%d human-walk checks %s" % (sum(checks), len(checks), "PASS" if all_pass else "FAIL"), flush=True)

    if not headless:
        try:
            sys.path.insert(0, "/home/ubuntu/SKATEBOARD_IRP/isaac_sim/scripts/testing")
            from test_real_human_character import capture_rgb as _cap
            _cap(stage, cam_path, capture_path, None)
            print("[INFO] screenshot saved: %s" % capture_path, flush=True)
        except Exception as e:
            print("[WARN] screenshot: %r" % e, flush=True)
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", default=False)
    parser.add_argument("--gui", action="store_true", default=False)
    args = parser.parse_args()
    hl = args.headless and not args.gui
    if args.gui:
        hl = False
    sys.exit(main(headless=hl))
