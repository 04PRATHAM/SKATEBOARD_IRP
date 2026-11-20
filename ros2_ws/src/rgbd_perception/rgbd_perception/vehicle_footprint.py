"""Unified Vehicle Collision Footprint & Kinematic Geometry Model.

Shared across:
- Global A* (astar_grid_path)
- Local detour spline generation (_generate_offset_path)
- SAT OBB swept footprint clearance checking (_check_path_clearance)
- Top-Down Path Planning Visualizer (path_planning_visualizer)
- Offline evaluation and regression test suites

Dimensions based on configs/vehicle.yaml and verified PhysX colliders:
- Chassis length: 1.80m, width: 1.20m, height: 0.40m
- Wheelbase: 1.10m, Front track: 1.00m, Rear track: 1.00m
- Wheel radius: 0.25m, Wheel width: 0.18m
- Camera mount: x = 0.90m, z = 1.20m
- Max steering angle (planner): 0.45 rad (25.78 deg)
- Kinematic curvature limit: kappa = tan(0.45) / 1.10 = 0.4391 rad/m
"""

import math
from typing import List, Tuple, Optional

# Physical vehicle geometry
CHASSIS_LENGTH: float = 1.80
CHASSIS_WIDTH: float = 1.20
WHEELBASE: float = 1.10
TRACK_WIDTH: float = 1.00
WHEEL_RADIUS: float = 0.25
WHEEL_WIDTH: float = 0.18
CAMERA_MOUNT_X: float = 0.90

# Planner motion limits
MAX_STEER_RAD: float = 0.45
MAX_CURVATURE_RAD_M: float = 0.4391  # tan(0.45) / 1.10
CURVATURE_FEASIBILITY_TOL: float = 0.460  # +5% numerical discretization margin

# Unified vehicle bounding box half-extents (including camera riser & steered wheel sweep)
# Chassis half-length 0.90m + front camera housing overhang 0.05m -> 0.95m
VEH_HALF_LENGTH: float = 0.95
# Chassis half-width 0.60m + front outer steered wheel lateral sweep 0.05m -> 0.65m
VEH_HALF_WIDTH: float = 0.65
# Standard safety buffer
SAFETY_MARGIN: float = 0.15

# Effective diagonal A* bounds under maximum Ackermann approach angle (25.8 deg)
# veh_hl_eff = 0.95 * cos(25.8 deg) + 0.65 * sin(25.8 deg) = 0.855 + 0.283 = 1.138m -> 1.15m
# veh_hw_eff = 0.95 * sin(25.8 deg) + 0.65 * cos(25.8 deg) = 0.413 + 0.585 = 0.998m -> 1.00m
ASTAR_HL_EFF: float = 1.15
ASTAR_HW_EFF: float = 1.00


def normalize_angle(a: float) -> float:
    """Wraps angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def sat_obb_clearance(px: float, py: float, heading: float, veh_hl: float, veh_hw: float,
                      ox: float, oy: float, obs_yaw: float, obs_hl: float, obs_hw: float) -> float:
    """Exact Separating Axis Theorem (SAT) clearance between Vehicle OBB and Obstacle OBB in 2D world plane.
    Returns:
        > 0: disjoint separation distance along the most separating normal axis.
        <= 0: intersecting / overlapping.
    """
    cos_v, sin_v = math.cos(heading), math.sin(heading)
    u_v1 = (cos_v, sin_v)
    u_v2 = (-sin_v, cos_v)

    cos_o, sin_o = math.cos(obs_yaw), math.sin(obs_yaw)
    u_o1 = (cos_o, sin_o)
    u_o2 = (-sin_o, cos_o)

    dc = (ox - px, oy - py)

    axes = [u_v1, u_v2, u_o1, u_o2]
    max_sep = -float("inf")

    for ax in axes:
        dist_c = abs(dc[0] * ax[0] + dc[1] * ax[1])
        r1 = veh_hl * abs(u_v1[0] * ax[0] + u_v1[1] * ax[1]) + veh_hw * abs(u_v2[0] * ax[0] + u_v2[1] * ax[1])
        r2 = obs_hl * abs(u_o1[0] * ax[0] + u_o1[1] * ax[1]) + obs_hw * abs(u_o2[0] * ax[0] + u_o2[1] * ax[1])
        sep = dist_c - (r1 + r2)
        if sep > max_sep:
            max_sep = sep

    return max_sep


def compute_path_curvature(dense_path: List[Tuple[float, float]],
                           look_baseline: float = 1.10) -> Tuple[float, List[float]]:
    """Calculates discrete curvature along dense path across the vehicle wheelbase baseline."""
    if len(dense_path) < 3:
        return 0.0, [0.0] * len(dense_path)

    # Estimate heading per sample
    headings = []
    n = len(dense_path)
    for idx in range(n):
        idx_next = min(n - 1, idx + 2)
        idx_prev = max(0, idx - 2)
        dx = dense_path[idx_next][0] - dense_path[idx_prev][0]
        dy = dense_path[idx_next][1] - dense_path[idx_prev][1]
        headings.append(math.atan2(dy, dx) if (dx * dx + dy * dy) > 1e-4 else 0.0)

    # Calculate curvature over look_baseline
    curvatures = [0.0] * n
    max_kappa = 0.0

    # Number of samples spanning look_baseline (dense ds ~ 0.10m -> ~11 samples)
    step_d = math.hypot(dense_path[1][0] - dense_path[0][0], dense_path[1][1] - dense_path[0][1]) if n > 1 else 0.10
    look_k = max(2, int(round(look_baseline / max(0.04, step_d))))

    for idx in range(look_k, n):
        d_head = abs(normalize_angle(headings[idx] - headings[idx - look_k]))
        seg_len = math.hypot(dense_path[idx][0] - dense_path[idx - look_k][0],
                             dense_path[idx][1] - dense_path[idx - look_k][1])
        kappa = d_head / max(0.15, seg_len)
        curvatures[idx] = kappa
        if kappa > max_kappa:
            max_kappa = kappa

    return max_kappa, curvatures
