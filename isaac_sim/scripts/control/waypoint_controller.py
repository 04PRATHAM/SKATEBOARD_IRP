"""
Waypoint Following Controller for Free-Space A-to-B Navigation (M8-M10)
Implements pure-pursuit / heading-error controller that generates
AckermannDriveStamped commands, flowing through existing Ackermann controller
and PhysX articulation. NO teleport, NO chassis velocity hack.

Pipeline (PART 8):
Waypoint Planner → Desired Path/Waypoint → Path Following Controller
 → AckermannDriveStamped → AckermannROSController → PhysX Vehicle

Reuse: drive_step + apply_ackermann_steering not duplicated.

Equations (to be documented in guide):
- desired_heading = atan2(target_y - y, target_x - x)
- heading_error   = normalize_angle(desired_heading - yaw)
- steer_cmd       = clip(kp_steer * heading_error, ±max_steer)
  with steering_rate limiting: steer += clip(target - steer, ±steer_rate*dt)
- speed_cmd: slows near waypoint (dist < slow_dist) and with large heading_error:
    speed = max_speed * (1 - 0.5*|heading_error|/max_steer)  clamped [min_speed, max_speed]
    speed *= min(1, dist/1.5)  when approaching waypoint
  accel/decel limited: speed += clip(target - speed, [-decel*dt, accel*dt])
- waypoint switch when dist < waypoint_tolerance
- final stop when dist to B < final_stop_distance → speed 0, steer 0
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a

@dataclass
class WaypointControllerConfig:
    lookahead_distance: float = 1.5   # m — pure-pursuit lookahead (unused in heading-error variant, kept for interface)
    waypoint_tolerance: float = 0.40   # m — switch to next waypoint
    final_stop_distance: float = 0.30  # m — stop at B
    max_speed: float = 1.2             # m/s — from vehicle.yaml max_speed 2.0 capped
    min_speed: float = 0.0
    max_steering_angle: float = 0.6    # rad
    max_acceleration: float = 1.0      # m/s²
    max_deceleration: float = 2.0      # m/s²
    steering_kp: float = 1.2           # heading_error → steer gain
    steering_rate: float = 2.0         # rad/s — steering rate limit
    slow_down_dist: float = 2.0
    # Collected from vehicle.yaml steering.yaml sync

class WaypointFollower:
    def __init__(self, waypoints: List[Tuple[float,float]], config: Optional[WaypointControllerConfig]=None):
        self.waypoints = [(float(x), float(y)) for x,y in waypoints]
        self.config = config or WaypointControllerConfig()
        self.current_idx = 0
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.reached = False

    def reset(self, waypoints: Optional[List[Tuple[float,float]]] = None):
        if waypoints is not None:
            self.waypoints = [(float(x), float(y)) for x,y in waypoints]
        self.current_idx = 0
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.reached = False

    def _target_heading(self, x: float, y: float) -> Optional[float]:
        if self.current_idx >= len(self.waypoints):
            return None
        tx, ty = self.waypoints[self.current_idx]
        return math.atan2(ty - y, tx - x)

    def update(self, x: float, y: float, yaw: float, dt: float) -> Tuple[float,float,bool]:
        """
        Returns (steer_rad, speed_mps, reached_B_flag).
        x,y,yaw from vehicle pose; dt is physics dt.
        """
        if self.reached or self.current_idx >= len(self.waypoints):
            self.reached = True
            return 0.0, 0.0, True

        # Distance to current waypoint
        tx, ty = self.waypoints[self.current_idx]
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        # Corner anticipation: if the direction change at this waypoint is
        # sharp, switch early (cut the corner like a real driver) and slow
        # down harder on approach. Without this, a >90 deg hairpin is missed
        # at speed and the pure-pursuit orbits (observed 2026-09-24).
        turn_ahead = 0.0
        if self.current_idx + 1 < len(self.waypoints):
            nx, ny = self.waypoints[self.current_idx + 1]
            h1 = math.atan2(ty - y, tx - x)
            h2 = math.atan2(ny - ty, nx - tx)
            turn_ahead = abs(normalize_angle(h2 - h1))
        sharp = turn_ahead > math.radians(60)

        # Waypoint acceptance
        is_final = (self.current_idx == len(self.waypoints)-1)
        tol = self.config.final_stop_distance if is_final else self.config.waypoint_tolerance
        if sharp and not is_final:
            tol = max(tol, 1.2)
        if dist < tol:
            if is_final:
                self.reached = True
                # smooth stop via decel below
            else:
                self.current_idx += 1
                if self.current_idx >= len(self.waypoints):
                    self.reached = True
                    return 0.0, 0.0, True
                # recompute for new waypoint
                tx, ty = self.waypoints[self.current_idx]
                dx = tx - x
                dy = ty - y
                dist = math.hypot(dx, dy)

        desired = math.atan2(dy, dx)
        heading_error = normalize_angle(desired - yaw)

        # Steering: P on heading_error, clipped, rate-limited
        target_steer = max(-self.config.max_steering_angle,
                           min(self.config.max_steering_angle,
                               self.config.steering_kp * heading_error))
        steer_diff = target_steer - self.current_steer
        max_dsteer = self.config.steering_rate * dt
        steer_diff = max(-max_dsteer, min(max_dsteer, steer_diff))
        self.current_steer += steer_diff

        # Speed: base on heading_error and distance to waypoint
        # Slow for large heading error (need to turn), slow when approaching waypoint
        heading_slow = 1.0 - 0.5 * abs(heading_error) / self.config.max_steering_angle
        heading_slow = max(0.3, heading_slow)
        dist_slow = min(1.0, dist / 1.5) if dist < self.config.slow_down_dist else 1.0
        target_speed = self.config.max_speed * heading_slow * dist_slow
        if sharp and dist < 2.5:
            target_speed *= 0.5
        target_speed = max(self.config.min_speed, min(self.config.max_speed, target_speed))
        if self.reached:
            target_speed = 0.0
            target_steer = 0.0

        # Accel/decel limit
        speed_diff = target_speed - self.current_speed
        if speed_diff > 0:
            speed_diff = min(speed_diff, self.config.max_acceleration * dt)
        else:
            speed_diff = max(speed_diff, -self.config.max_deceleration * dt)
        self.current_speed += speed_diff
        if self.reached:
            # force decel faster at final
            self.current_speed = max(0.0, self.current_speed - self.config.max_deceleration * dt * 0.5)
            if self.current_speed < 0.05:
                self.current_speed = 0.0

        return float(self.current_steer), float(self.current_speed), bool(self.reached)

    def get_current_target(self) -> Optional[Tuple[float,float]]:
        if 0 <= self.current_idx < len(self.waypoints):
            return self.waypoints[self.current_idx]
        return None
