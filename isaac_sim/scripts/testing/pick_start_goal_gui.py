"""Pre-Simulation Interactive Start and Goal Allotment GUI.

Allows the user to interactively inspect the scenario map and set BOTH the
Start point and Destination endpoint on the 2D Top-Down Map BEFORE the
simulation begins.

Features:
  - Live hover coordinate display (in meters, world frame) with walkable validation.
  - Left-click to place Start point (cyan vehicle marker with heading arrow).
  - Left-click to place Destination endpoint (gold star marker).
  - Real-time A* route preview avoiding walls and scenario obstacles.
  - Buttons: [Set Start (S)], [Set Goal (G)], [Swap S/G (W)], [Reset Defaults (R)],
             [CONFIRM & LAUNCH SIMULATION (Enter)].
  - On confirmation, launches Isaac Sim 6.2 with vehicle physically spawned at the
    chosen start pose and planner navigating to the chosen destination.

Usage:
  # Standalone interactive picker for scenario S02
  python3 isaac_sim/scripts/testing/pick_start_goal_gui.py --scenario s02

  # Direct launch via integrated perception GUI
  python3 isaac_sim/scripts/testing/run_integrated_perception_gui.py --scenario s02 --pick --gui
"""

import sys
import os
import math
import yaml
import heapq
import argparse
import subprocess
import numpy as np
import cv2
from typing import Tuple, List, Optional

ROOT = "/home/ubuntu/SKATEBOARD_IRP"
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from isaac_sim.scripts.navigation.navmesh import (
    is_point_walkable, LAB_X_MIN, LAB_X_MAX, LAB_Y_MIN, LAB_Y_MAX, OBSTACLES
)


def astar_grid_path(start: Tuple[float, float], goal: Tuple[float, float],
                    obstacles: Optional[List[dict]] = None,
                    res: float = 0.25, safety_margin: float = 0.50) -> Optional[List[Tuple[float, float]]]:
    """Pure Python 2D Grid A* path planner for route preview."""
    sx, sy = start
    gx, gy = goal

    def to_grid(x, y):
        return int(round((x - LAB_X_MIN) / res)), int(round((y - LAB_Y_MIN) / res))

    def to_world(gx_idx, gy_idx):
        return LAB_X_MIN + gx_idx * res, LAB_Y_MIN + gy_idx * res

    start_node = to_grid(sx, sy)
    goal_node = to_grid(gx, gy)

    obs_circles = []
    if obstacles:
        for obs in obstacles:
            ox = obs.get("x_world", obs.get("x", 0.0))
            oy = obs.get("y_world", obs.get("y", 0.0))
            obs_circles.append((ox, oy, safety_margin))

    def is_free(node):
        x_w, y_w = to_world(node[0], node[1])
        if not (LAB_X_MIN <= x_w <= LAB_X_MAX and LAB_Y_MIN <= y_w <= LAB_Y_MAX):
            return False
        if abs(sy - -6.0) < 1.5 and abs(gy - -6.0) < 1.5:
            if y_w < -7.6 or y_w > -4.4:
                return False
        if not is_point_walkable(x_w, y_w):
            return False
        for ox, oy, r in obs_circles:
            if math.hypot(x_w - ox, y_w - oy) < r:
                return False
        return True

    open_heap = []
    h_start = math.hypot(gx - sx, gy - sy)
    heapq.heappush(open_heap, (h_start, 0.0, start_node))
    came_from = {}
    cost_so_far = {start_node: 0.0}

    motions = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
    ]

    max_iter = 6000
    iterations = 0
    closest_node = start_node
    min_dist_to_goal = float("inf")

    while open_heap and iterations < max_iter:
        iterations += 1
        _, current_g, current = heapq.heappop(open_heap)
        cw_x, cw_y = to_world(current[0], current[1])
        d_to_goal = math.hypot(gx - cw_x, gy - cw_y)
        if d_to_goal < min_dist_to_goal:
            min_dist_to_goal = d_to_goal
            closest_node = current
        if d_to_goal < res * 1.5:
            closest_node = current
            break

        for mx, my, mcost in motions:
            nxt = (current[0] + mx, current[1] + my)
            if not is_free(nxt):
                continue
            new_g = current_g + mcost * res
            if nxt not in cost_so_far or new_g < cost_so_far[nxt]:
                cost_so_far[nxt] = new_g
                nw_x, nw_y = to_world(nxt[0], nxt[1])
                h = math.hypot(gx - nw_x, gy - nw_y)
                priority = new_g + h
                heapq.heappush(open_heap, (priority, new_g, nxt))
                came_from[nxt] = current

    if closest_node not in came_from and closest_node != start_node:
        return [(sx, sy), (gx, gy)]

    curr = closest_node
    path_nodes = []
    while curr in came_from:
        path_nodes.append(curr)
        curr = came_from[curr]
    path_nodes.append(start_node)
    path_nodes.reverse()

    waypoints = [to_world(n[0], n[1]) for n in path_nodes]
    if waypoints and math.hypot(waypoints[0][0] - sx, waypoints[0][1] - sy) > 0.05:
        waypoints.insert(0, (sx, sy))
    if waypoints and math.hypot(waypoints[-1][0] - gx, waypoints[-1][1] - gy) > 0.15:
        waypoints.append((gx, gy))

    return waypoints


class PreSimulationPicker:
    def __init__(self, scenario_path: str, win_name="PRE-SIMULATION: ALLOT START & GOAL"):
        self.scenario_path = scenario_path
        self.win_name = win_name
        self.img_w = 1024
        self.img_h = 860

        # Viewport margins
        self.map_x_min = 20
        self.map_x_max = self.img_w - 20
        self.map_y_min = 70
        self.map_y_max = self.img_h - 100

        self.world_min_x = LAB_X_MIN - 0.5
        self.world_max_x = LAB_X_MAX + 0.5
        self.world_min_y = LAB_Y_MIN - 0.5
        self.world_max_y = LAB_Y_MAX + 0.5

        # Load Scenario YAML
        self.scen_data = {}
        try:
            with open(scenario_path, "r") as f:
                self.scen_data = yaml.safe_load(f) or {}
        except Exception as e:
            print(f"[PICKER] Warning loading {scenario_path}: {e}")

        self.scen_name = self.scen_data.get("scenario_name", os.path.basename(scenario_path))
        self.static_obstacles = self.scen_data.get("static_obstacles", []) or []
        self.pedestrians = self.scen_data.get("pedestrians", []) or []

        # Default start and goal
        def_start = self.scen_data.get("vehicle_start_pose", [6.0, -6.0, 0.35, 0.0])
        def_goal = self.scen_data.get("vehicle_destination_pose", [13.5, -6.0, 0.0])
        self.default_start = (float(def_start[0]), float(def_start[1]), float(def_start[3]) if len(def_start) > 3 else 0.0)
        self.default_goal = (float(def_goal[0]), float(def_goal[1]))

        self.start_pose = list(self.default_start)  # [x, y, yaw]
        self.goal_pos = list(self.default_goal)    # [x, y]

        # Interaction Mode: "START", "GOAL", "CONFIRM"
        self.mode = "START"
        self.mouse_pos = (0.0, 0.0)
        self.mouse_px = (0, 0)
        self.is_walkable_hover = True
        self.confirmed = False
        self.status_msg = "CLICK 1: Choose vehicle START position (or press S/G to toggle)"

        # Precomputed preview path
        self.preview_path = []
        self.recompute_preview_path()

    def world_to_pixel(self, wx: float, wy: float) -> Tuple[int, int]:
        px = int(self.map_x_min + (wx - self.world_min_x) / (self.world_max_x - self.world_min_x) * (self.map_x_max - self.map_x_min))
        py = int(self.map_y_min + (self.world_max_y - wy) / (self.world_max_y - self.world_min_y) * (self.map_y_max - self.map_y_min))
        return max(0, min(self.img_w - 1, px)), max(0, min(self.img_h - 1, py))

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        wx = self.world_min_x + (px - self.map_x_min) / float(self.map_x_max - self.map_x_min) * (self.world_max_x - self.world_min_x)
        wy = self.world_max_y - (py - self.map_y_min) / float(self.map_y_max - self.map_y_min) * (self.world_max_y - self.world_min_y)
        return round(wx, 2), round(wy, 2)

    def recompute_preview_path(self):
        """Computes A* preview path using start, goal, and scenario obstacles."""
        obs_dicts = []
        for so in self.static_obstacles:
            pos = so.get("position", [0.0, 0.0])
            obs_dicts.append({"x_world": float(pos[0]), "y_world": float(pos[1])})
        p = astar_grid_path((self.start_pose[0], self.start_pose[1]), (self.goal_pos[0], self.goal_pos[1]),
                            obstacles=obs_dicts, safety_margin=0.45)
        self.preview_path = p or [(self.start_pose[0], self.start_pose[1]), (self.goal_pos[0], self.goal_pos[1])]

    def is_valid_point(self, wx: float, wy: float) -> bool:
        """Validates that a point is inside the lab, outside lab walls, and outside scenario static obstacles."""
        if not is_point_walkable(wx, wy):
            return False
        for so in self.static_obstacles:
            pos = so.get("position", [0.0, 0.0])
            dims = so.get("dimensions", [1.4, 0.8])
            ox, oy = float(pos[0]), float(pos[1])
            w, h = float(dims[0]), float(dims[1])
            if (ox - w / 2 - 0.35 <= wx <= ox + w / 2 + 0.35 and
                oy - h / 2 - 0.35 <= wy <= oy + h / 2 + 0.35):
                return False
        return True

    def on_mouse(self, event, x, y, flags, param):
        self.mouse_px = (x, y)
        wx, wy = self.pixel_to_world(x, y)
        self.mouse_pos = (wx, wy)
        self.is_walkable_hover = self.is_valid_point(wx, wy)

        # Check button clicks (Bottom toolbar: y in [805, 850])
        if event == cv2.EVENT_LBUTTONDOWN:
            if 805 <= y <= 850:
                # Button 1: [ Set Start (S) ] (x: 20..150)
                if 20 <= x <= 150:
                    self.mode = "START"
                    self.status_msg = "MODE: CLICK MAP TO ALLOT START POINT"
                    return
                # Button 2: [ Set Goal (G) ] (x: 160..290)
                elif 160 <= x <= 290:
                    self.mode = "GOAL"
                    self.status_msg = "MODE: CLICK MAP TO ALLOT DESTINATION GOAL"
                    return
                # Button 3: [ Swap S/G (W) ] (x: 300..420)
                elif 300 <= x <= 420:
                    self.start_pose[0], self.goal_pos[0] = self.goal_pos[0], self.start_pose[0]
                    self.start_pose[1], self.goal_pos[1] = self.goal_pos[1], self.start_pose[1]
                    self.start_pose[2] = math.atan2(self.goal_pos[1] - self.start_pose[1], self.goal_pos[0] - self.start_pose[0])
                    self.recompute_preview_path()
                    self.status_msg = "Swapped Start and Goal Points."
                    return
                # Button 4: [ Reset Defaults (R) ] (x: 430..560)
                elif 430 <= x <= 560:
                    self.start_pose = list(self.default_start)
                    self.goal_pos = list(self.default_goal)
                    self.recompute_preview_path()
                    self.status_msg = "Reset to Scenario Default Coordinates."
                    return
                # Button 5: [ CONFIRM & LAUNCH SIMULATION ] (x: 580..1004)
                elif 580 <= x <= 1004:
                    self.confirmed = True
                    return

            # Map clicks
            if self.map_x_min <= x <= self.map_x_max and self.map_y_min <= y <= self.map_y_max:
                if not self.is_valid_point(wx, wy):
                    self.status_msg = f"INVALID: ({wx:.2f}, {wy:.2f}) is inside wall/furniture!"
                    return

                if self.mode == "START":
                    yaw = math.atan2(self.goal_pos[1] - wy, self.goal_pos[0] - wx)
                    self.start_pose = [wx, wy, yaw]
                    self.mode = "GOAL"
                    self.recompute_preview_path()
                    self.status_msg = f"Start set to ({wx:.2f}, {wy:.2f}). CLICK 2: Set Destination Endpoint."
                elif self.mode == "GOAL":
                    self.goal_pos = [wx, wy]
                    # Update start heading towards new goal
                    self.start_pose[2] = math.atan2(self.goal_pos[1] - self.start_pose[1], self.goal_pos[0] - self.start_pose[0])
                    self.mode = "CONFIRM"
                    self.recompute_preview_path()
                    self.status_msg = f"Goal set to ({wx:.2f}, {wy:.2f}). Press Enter or click [CONFIRM & LAUNCH]."
                else:
                    # Normal mode click sets goal
                    self.goal_pos = [wx, wy]
                    self.start_pose[2] = math.atan2(self.goal_pos[1] - self.start_pose[1], self.goal_pos[0] - self.start_pose[0])
                    self.recompute_preview_path()
                    self.status_msg = f"Destination updated to ({wx:.2f}, {wy:.2f})."

        elif event == cv2.EVENT_RBUTTONDOWN:
            # Right-click directly sets start point
            if self.map_x_min <= x <= self.map_x_max and self.map_y_min <= y <= self.map_y_max:
                if self.is_valid_point(wx, wy):
                    yaw = math.atan2(self.goal_pos[1] - wy, self.goal_pos[0] - wx)
                    self.start_pose = [wx, wy, yaw]
                    self.recompute_preview_path()
                    self.status_msg = f"Start point set (Right Click): ({wx:.2f}, {wy:.2f})."
                else:
                    self.status_msg = f"INVALID: Point ({wx:.2f}, {wy:.2f}) inside obstacle!"

    def draw_canvas(self) -> np.ndarray:
        """Draws the complete 2D map, boundaries, obstacles, preview route, and markers."""
        canvas = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        canvas[:] = (22, 26, 34)

        # 1. Top Header Banner
        cv2.rectangle(canvas, (0, 0), (self.img_w, 62), (32, 38, 50), -1)
        cv2.putText(canvas, f"PRE-SIMULATION MAP PICKER: ALLOT START & GOAL POINTS", (20, 26),
                    cv2.FONT_HERSHEY_DUPLEX, 0.65, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Scenario: {self.scen_name}  |  Obstacles: {len(self.static_obstacles)}  |  Pedestrians: {len(self.pedestrians)}",
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 200, 230), 1, cv2.LINE_AA)

        # 2. Draw Lab Boundary and Corridor
        p_top_l = self.world_to_pixel(LAB_X_MIN, LAB_Y_MAX)
        p_bot_r = self.world_to_pixel(LAB_X_MAX, LAB_Y_MIN)
        cv2.rectangle(canvas, p_top_l, p_bot_r, (80, 85, 100), 2)

        # Highlight corridor driving zone (y in [-7.5, -4.5])
        p_c_tl = self.world_to_pixel(LAB_X_MIN, -4.5)
        p_c_br = self.world_to_pixel(LAB_X_MAX, -7.5)
        corridor_overlay = canvas.copy()
        cv2.rectangle(corridor_overlay, p_c_tl, p_c_br, (35, 50, 40), -1)
        cv2.addWeighted(corridor_overlay, 0.45, canvas, 0.55, 0, canvas)
        cv2.rectangle(canvas, p_c_tl, p_c_br, (60, 110, 80), 1)
        cv2.putText(canvas, "MAIN DRIVING CORRIDOR (y in [-7.5, -4.5])", (p_c_tl[0] + 10, p_c_tl[1] + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (100, 180, 130), 1, cv2.LINE_AA)

        # Glass Wall boundary
        p_gw_t = self.world_to_pixel(15.0, LAB_Y_MAX)
        p_gw_b = self.world_to_pixel(15.0, LAB_Y_MIN)
        cv2.line(canvas, p_gw_t, p_gw_b, (220, 200, 120), 2)

        # Background Lab Furniture
        for ox, oy, w, h in OBSTACLES:
            tl = self.world_to_pixel(ox - w / 2, oy + h / 2)
            br = self.world_to_pixel(ox + w / 2, oy - h / 2)
            cv2.rectangle(canvas, tl, br, (40, 44, 55), -1)
            cv2.rectangle(canvas, tl, br, (70, 75, 85), 1)

        # Scenario Static Obstacles (Tables, Chairs, Crates)
        for so in self.static_obstacles:
            pos = so.get("position", [0.0, 0.0])
            dims = so.get("dimensions", [1.4, 0.8])
            ox, oy = float(pos[0]), float(pos[1])
            w, h = float(dims[0]), float(dims[1])
            tl = self.world_to_pixel(ox - w / 2, oy + h / 2)
            br = self.world_to_pixel(ox + w / 2, oy - h / 2)
            cv2.rectangle(canvas, tl, br, (30, 70, 180), -1)
            cv2.rectangle(canvas, tl, br, (60, 120, 255), 2)
            oid = so.get("object_id", "obs")
            cv2.putText(canvas, oid, (tl[0] + 4, tl[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (80, 160, 255), 1)

        # Scenario Pedestrians
        for ped in self.pedestrians:
            init_p = ped.get("initial_position", [0.0, 0.0])
            dest_p = ped.get("destination", init_p)
            p1 = self.world_to_pixel(init_p[0], init_p[1])
            p2 = self.world_to_pixel(dest_p[0], dest_p[1])
            cv2.circle(canvas, p1, 7, (180, 80, 220), -1)
            cv2.arrowedLine(canvas, p1, p2, (200, 120, 240), 2, tipLength=0.2)
            cv2.putText(canvas, f"Ped: {ped.get('pedestrian_id', '')}", (p1[0] + 8, p1[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 120, 240), 1)

        # 3. Draw A* Preview Route
        if len(self.preview_path) > 1:
            pts = np.array([self.world_to_pixel(x, y) for x, y in self.preview_path], dtype=np.int32)
            cv2.polylines(canvas, [pts], isClosed=False, color=(230, 180, 60), thickness=2, lineType=cv2.LINE_AA)
            for pt in pts[::2]:
                cv2.circle(canvas, tuple(pt), 2, (255, 220, 100), -1)

        # 4. Draw Allotted Start Point
        p_s = self.world_to_pixel(self.start_pose[0], self.start_pose[1])
        # Draw vehicle footprint oriented with yaw
        yaw = self.start_pose[2]
        v_len_px = 24
        v_wid_px = 14
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        # 4 corners
        corners = [
            (v_len_px / 2, v_wid_px / 2), (v_len_px / 2, -v_wid_px / 2),
            (-v_len_px / 2, -v_wid_px / 2), (-v_len_px / 2, v_wid_px / 2)
        ]
        c_pts = []
        for cx, cy in corners:
            px = int(p_s[0] + cx * cos_y + cy * sin_y)
            py = int(p_s[1] - cx * sin_y + cy * cos_y)
            c_pts.append((px, py))
        cv2.fillPoly(canvas, [np.array(c_pts, dtype=np.int32)], (200, 140, 40))
        cv2.polylines(canvas, [np.array(c_pts, dtype=np.int32)], True, (255, 200, 80), 2)
        # Heading arrow
        arr_end = (int(p_s[0] + 28 * cos_y), int(p_s[1] - 28 * sin_y))
        cv2.arrowedLine(canvas, p_s, arr_end, (255, 240, 100), 2, tipLength=0.3)
        cv2.circle(canvas, p_s, 6, (255, 220, 50), -1)
        cv2.putText(canvas, f"START: ({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f})",
                    (p_s[0] - 40, p_s[1] - 18), cv2.FONT_HERSHEY_DUPLEX, 0.45, (255, 220, 50), 1, cv2.LINE_AA)

        # 5. Draw Allotted Goal Point
        p_g = self.world_to_pixel(self.goal_pos[0], self.goal_pos[1])
        cv2.circle(canvas, p_g, 14, (60, 220, 80), 2)
        cv2.circle(canvas, p_g, 7, (60, 220, 80), -1)
        cv2.putText(canvas, f"GOAL: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})",
                    (p_g[0] - 40, p_g[1] - 18), cv2.FONT_HERSHEY_DUPLEX, 0.45, (80, 255, 100), 1, cv2.LINE_AA)

        # 6. Live Mouse Hover Coordinates
        mx, my = self.mouse_pos
        col = (80, 240, 100) if self.is_walkable_hover else (60, 60, 240)
        status_text = "CLEAR / WALKABLE" if self.is_walkable_hover else "BLOCKED (INSIDE WALL/OBSTACLE)"
        cv2.putText(canvas, f"Cursor: ({mx:+.2f}m, {my:+.2f}m) - {status_text}",
                    (20, self.img_h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)

        # Current Planned Route Metrics
        dist = 0.0
        for i in range(len(self.preview_path) - 1):
            p1, p2 = self.preview_path[i], self.preview_path[i + 1]
            dist += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        eta = dist / 0.85
        cv2.putText(canvas, f"Route Length: {dist:.2f} m  |  Est. Drive Time: {eta:.1f} s  |  Status: {self.status_msg}",
                    (20, self.img_h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)

        # 7. Render Interactive Bottom Toolbar Buttons
        self._render_buttons(canvas)
        return canvas

    def run(self) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """Runs the interactive GUI loop until confirmed or cancelled."""
        cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win_name, self.on_mouse)

        while not self.confirmed:
            canvas = self.draw_canvas()

            cv2.imshow(self.win_name, canvas)
            key = cv2.waitKey(20) & 0xFF

            if key in (27, ord('q'), ord('Q')):  # Esc / Q to abort
                print("[PICKER] User cancelled point allotment.")
                cv2.destroyAllWindows()
                return None, None
            elif key in (13, 32):  # Enter / Space to confirm & launch
                self.confirmed = True
            elif key in (ord('s'), ord('S')):
                self.mode = "START"
                self.status_msg = "MODE: CLICK MAP TO ALLOT START POINT"
            elif key in (ord('g'), ord('G')):
                self.mode = "GOAL"
                self.status_msg = "MODE: CLICK MAP TO ALLOT DESTINATION GOAL"
            elif key in (ord('w'), ord('W')):
                self.start_pose[0], self.goal_pos[0] = self.goal_pos[0], self.start_pose[0]
                self.start_pose[1], self.goal_pos[1] = self.goal_pos[1], self.start_pose[1]
                self.start_pose[2] = math.atan2(self.goal_pos[1] - self.start_pose[1], self.goal_pos[0] - self.start_pose[0])
                self.recompute_preview_path()
                self.status_msg = "Swapped Start and Goal Points."
            elif key in (ord('r'), ord('R')):
                self.start_pose = list(self.default_start)
                self.goal_pos = list(self.default_goal)
                self.recompute_preview_path()
                self.status_msg = "Reset to Scenario Default Coordinates."

        cv2.destroyAllWindows()
        print(f"\n=======================================================")
        print(f"[PICKER CONFIRMED] Points Allotted Successfully!")
        print(f"  Allotted Start Pose : ({self.start_pose[0]:.2f}, {self.start_pose[1]:.2f}, yaw={self.start_pose[2]:.2f} rad)")
        print(f"  Allotted Goal Point : ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})")
        print(f"=======================================================\n", flush=True)
        return self.start_pose, self.goal_pos

    def _render_buttons(self, canvas):
        y1, y2 = 805, 850
        # Button 1: Set Start
        b1_active = (self.mode == "START")
        col1 = (50, 140, 220) if b1_active else (45, 50, 65)
        cv2.rectangle(canvas, (20, y1), (150, y2), col1, -1)
        cv2.rectangle(canvas, (20, y1), (150, y2), (90, 160, 240), 1)
        cv2.putText(canvas, "Set Start (S)", (32, y1 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        # Button 2: Set Goal
        b2_active = (self.mode == "GOAL")
        col2 = (40, 160, 80) if b2_active else (45, 50, 65)
        cv2.rectangle(canvas, (160, y1), (290, y2), col2, -1)
        cv2.rectangle(canvas, (160, y1), (290, y2), (80, 200, 100), 1)
        cv2.putText(canvas, "Set Goal (G)", (174, y1 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        # Button 3: Swap S/G
        cv2.rectangle(canvas, (300, y1), (420, y2), (45, 50, 65), -1)
        cv2.rectangle(canvas, (300, y1), (420, y2), (90, 100, 120), 1)
        cv2.putText(canvas, "Swap S/G (W)", (312, y1 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1)

        # Button 4: Reset Defaults
        cv2.rectangle(canvas, (430, y1), (560, y2), (45, 50, 65), -1)
        cv2.rectangle(canvas, (430, y1), (560, y2), (90, 100, 120), 1)
        cv2.putText(canvas, "Reset (R)", (460, y1 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1)

        # Button 5: CONFIRM & LAUNCH SIMULATION (PROMINENT GREEN)
        cv2.rectangle(canvas, (580, y1), (1004, y2), (30, 140, 50), -1)
        cv2.rectangle(canvas, (580, y1), (1004, y2), (60, 230, 90), 2)
        cv2.putText(canvas, "CONFIRM & LAUNCH SIMULATION (Enter)", (625, y1 + 28),
                    cv2.FONT_HERSHEY_DUPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)


def pick_points_interactive(scenario_path: str) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    """Public helper function callable by other scripts."""
    picker = PreSimulationPicker(scenario_path)
    return picker.run()


def main():
    parser = argparse.ArgumentParser(description="Pre-Simulation Interactive Start & Goal Allotment")
    parser.add_argument("--scenario", default="s02", help="Scenario ID (s01..s13) or YAML path")
    parser.add_argument("--gui", action="store_true", default=True, help="Launch simulation with GUI after picking")
    args = parser.parse_args()

    from isaac_sim.scripts.scenarios.scenario_registry import resolve_scenario
    scen_info = resolve_scenario(args.scenario)
    scen_path = scen_info["path"]

    start, goal = pick_points_interactive(scen_path)
    if start is None or goal is None:
        print("[PICKER] Point allotment aborted. Exiting.")
        sys.exit(0)

    # Launch integrated perception GUI with the exact allotted points
    launch_cmd = [
        "python3", "isaac_sim/scripts/testing/run_integrated_perception_gui.py",
        "--scenario", scen_info["id"],
        "--start", str(start[0]), str(start[1]), str(start[2]),
        "--goal", str(goal[0]), str(goal[1]),
    ]
    if args.gui:
        launch_cmd.append("--gui")

    print(f"\n[LAUNCHING SIMULATION] Executing command:")
    print(f"  {' '.join(launch_cmd)}\n", flush=True)
    subprocess.run(launch_cmd, cwd=ROOT)


if __name__ == "__main__":
    main()
