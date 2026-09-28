"""Camera-only local free-space planner (pure function — no ROS, no GT, no odom).

Input: objects in the DEPTH OPTICAL frame (x right, y down, z forward, metres)
  + config planner section. Only obstacle_classes are avoided; other labels
  are reported in debug but never steered around.

Policy (deterministic, documented):
  sectors by lateral x with corridor half-width w/2: LEFT (x < -w/2),
  CENTER (|x| <= w/2), RIGHT (x > +w/2); nearest forward-z per sector within
  [lookahead_min, lookahead_max].
  - center clear (no obstacle or nearest >= slow_distance): steer 0, cruise.
  - center nearest < stop_distance: STOP (speed 0, steer 0).
  - else (slow_distance band): slow + steer toward the freer side
    (larger nearest-z wins; tie -> RIGHT). steer = +/-0.4 * gain-scaled.
  - both sides blocked (nearest < stop_distance): STOP.
Output: (steer_rad, speed_mps, debug dict). Limits saturated to config.
Steering sign: + = left (matches Ackermann drive_step convention).
Same file runs on the real vehicle.
"""
import math


def plan(objects, cfg):
    p = cfg.get("planner", {})
    w = float(p.get("corridor_width", 1.2)) / 2.0
    zmin = float(p.get("lookahead_min", 1.0))
    zmax = float(p.get("lookahead_max", 8.0))
    slow_d = float(p.get("slow_distance", 4.0))
    stop_d = float(p.get("stop_distance", 1.5))
    cruise = float(p.get("cruise_speed", 0.8))
    slow_v = float(p.get("slow_speed", 0.35))
    max_steer = float(p.get("max_steer", 0.6))
    gain = float(p.get("steer_gain", 0.5))
    obstacles = set(int(c) for c in cfg.get("perception", {}).get("obstacle_classes", [0]))

    # Frame: DEPTH OPTICAL (x right, y down, z forward, metres) — the fusion
    # node outputs optical directly (no TF lookups; portable). Vehicle-left
    # is optical -x; steer + = left.
    def pos(o):
        if isinstance(o, dict):
            return o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0), int(o.get("class_id", -1))
        return (float(o.position[0]), float(o.position[1]), float(o.position[2]),
                int(getattr(o, "class_id", getattr(o, "object_id", -1))))

    nearest = {"left": float("inf"), "center": float("inf"), "right": float("inf")}
    n_obs = 0
    for o in objects or []:
        try:
            x, _y, z, cid = pos(o)
        except Exception:
            continue
        if obstacles and cid not in obstacles:
            continue
        if not (math.isfinite(x) and math.isfinite(z)) or not (zmin <= z <= zmax):
            continue
        n_obs += 1
        sector = "center" if abs(x) <= w else ("left" if x < 0 else "right")
        if z < nearest[sector]:
            nearest[sector] = z

    dbg = {"nearest": dict(nearest), "n_obstacles": n_obs, "decision": ""}
    if nearest["center"] < stop_d or (
            nearest["left"] < stop_d and nearest["right"] < stop_d
            and nearest["center"] < slow_d):
        dbg["decision"] = "STOP_blocked"
        return 0.0, 0.0, dbg
    if nearest["center"] >= slow_d:
        dbg["decision"] = "CRUISE_clear"
        return 0.0, cruise, dbg
    # center needs caution: go to freer side (tie -> right)
    if nearest["right"] >= nearest["left"]:
        steer = -0.4 * (0.5 + 0.5 * gain)
        dbg["decision"] = "SLOW_right"
    else:
        steer = 0.4 * (0.5 + 0.5 * gain)
        dbg["decision"] = "SLOW_left"
    steer = max(-max_steer, min(max_steer, steer))
    return steer, slow_v, dbg
