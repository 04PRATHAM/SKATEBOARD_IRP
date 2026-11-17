"""Central Scenario Registry for SKATEBOARD_IRP.

Canonical source of truth for all 13 autonomous scenarios (S01 - S13).
Shared by:
  - run_integrated_perception_gui.py (Main launcher)
  - scenario_runner.py (Isaac Sim simulation engine)
  - path_planning_visualizer.py (Window 3 interactive top-down map)
  - live_perception_monitor.py (Window 2 perception monitor)
  - evaluate_all_13_scenarios.py (Batch verifier)
"""

import os
from pathlib import Path
from typing import Dict, List, Any, Optional

ROOT_DIR = "/home/ubuntu/SKATEBOARD_IRP"
SCEN_DIR = os.path.join(ROOT_DIR, "configs/scenarios")

# Master 13-Scenario Matrix
SCENARIOS_DATA: List[Dict[str, Any]] = [
    {
        "id": "s01",
        "name": "S01: Empty Lab Corridor",
        "file": "scenario_s01_empty_lab.yaml",
        "title": "Empty Lab Corridor",
        "description": "Baseline point-to-point navigation with no obstacles.",
        "expected_outcome": "GOAL_REACHED",
        "timeout": 35.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s02",
        "name": "S02: Single Static Table",
        "file": "scenario_s02_one_static_obstacle.yaml",
        "title": "Single Static Table",
        "description": "Unknown static table blocking forward route at (10.0, -6.0). Local Ackermann detour required.",
        "expected_outcome": "GOAL_REACHED",
        "timeout": 45.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s03",
        "name": "S03: Multiple Static Obstacles",
        "file": "scenario_s03_multiple_static_obstacles.yaml",
        "title": "Multiple Static Obstacles (Table + Chair)",
        "description": "Table at (9.5, -6.1) and chair at (11.5, -5.2); weave route required.",
        "expected_outcome": "GOAL_REACHED",
        "timeout": 50.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s04",
        "name": "S04: One Moving Pedestrian",
        "file": "scenario_s04_one_moving_pedestrian.yaml",
        "title": "One Moving Crossing Pedestrian",
        "description": "Crossing pedestrian from (11.0, -4.5) to (11.0, -7.5); dynamic yield and goal arrival.",
        "expected_outcome": "YIELD_AND_GOAL_REACHED",
        "timeout": 50.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s05",
        "name": "S05: Two Moving Pedestrians",
        "file": "scenario_s05_two_moving_pedestrians.yaml",
        "title": "Two Moving Crossing Pedestrians",
        "description": "Two independent pedestrians crossing corridor simultaneously; dynamic tracking and yield.",
        "expected_outcome": "YIELD_AND_GOAL_REACHED",
        "timeout": 55.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s06",
        "name": "S06: Moving Ped + Furniture",
        "file": "scenario_s06_moving_ped_plus_furniture.yaml",
        "title": "Moving Pedestrian + Static Table",
        "description": "Static table at (9.5, -5.8) combined with crossing pedestrian; table detour + pedestrian yield.",
        "expected_outcome": "DETOUR_AND_GOAL_REACHED",
        "timeout": 55.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s07",
        "name": "S07: Unknown Solid Crate",
        "file": "scenario_s07_unknown_unclassified_object.yaml",
        "title": "Unknown Unclassified Solid Crate",
        "description": "COCO-unclassified orange crate at (10.0, -6.0); detected through depth geometry without COCO label.",
        "expected_outcome": "GOAL_REACHED",
        "timeout": 45.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s08",
        "name": "S08: Unexpected Obstacle",
        "file": "scenario_s08_unexpected_obstacle.yaml",
        "title": "Unexpected Obstacle Along Route",
        "description": "Obstacle placed along nominal route at (10.5, -6.0) discovered during movement; real-time replan.",
        "expected_outcome": "GOAL_REACHED",
        "timeout": 45.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s09",
        "name": "S09: Narrow Passage",
        "file": "scenario_s09_narrow_passage.yaml",
        "title": "Narrow Passage Between Two Tables",
        "description": "Narrow corridor between two tables at y=-4.9 and y=-7.1; verify clearance and pass safely.",
        "expected_outcome": "GOAL_REACHED",
        "timeout": 45.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s10",
        "name": "S10: Multi-Obstacle Crossing",
        "file": "scenario_s10_multiple_obstacles_crossing.yaml",
        "title": "Multiple Obstacles and Crossing Trajectories",
        "description": "Complex environment with table, chair, and crossing pedestrian; multi-threat resolution.",
        "expected_outcome": "DETOUR_AND_GOAL_REACHED",
        "timeout": 55.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s11",
        "name": "S11: Obstacle Leaves Route",
        "file": "scenario_s11_obstacle_leaves_route.yaml",
        "title": "Obstacle Leaves Route",
        "description": "Pedestrian blocks route at (10.0, -6.0) then vacates towards y=-8.5; vehicle waits then resumes.",
        "expected_outcome": "RESUME_AND_GOAL_REACHED",
        "timeout": 50.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s12",
        "name": "S12: No Safe Route",
        "file": "scenario_s12_no_safe_route.yaml",
        "title": "No Safe Route (Corridor Blockade)",
        "description": "Complete corridor blockade across x=10.0; verify NO_SAFE_PATH and safe stop before obstacle.",
        "expected_outcome": "SAFE_STOP_NO_SAFE_PATH",
        "timeout": 35.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s13",
        "name": "S13: Perception Failure Injection",
        "file": "scenario_s13_perception_failure.yaml",
        "title": "Perception Failure Injection",
        "description": "Perception loss fault injected; watchdog triggers STALE_STOP safe halt within configured timeout.",
        "expected_outcome": "WATCHDOG_SAFE_STOP",
        "timeout": 30.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
    {
        "id": "s_slip",
        "name": "Slip: Physical Blockage & Wheel-Slip Test",
        "file": "scenario_wheel_slip_blocked.yaml",
        "title": "Physical Blockage & Wheel-Slip Test",
        "description": "Solid obstacle directly contacts vehicle at x=7.2m; verifies RGB-D visual motion halts odometry and triggers WHEEL_SLIP safe stop.",
        "expected_outcome": "WHEEL_SLIP_SAFE_STOP",
        "timeout": 30.0,
        "default_start": (6.0, -6.0, 0.35, 0.0),
        "default_goal": (13.5, -6.0, 0.0),
    },
]

# Aliases map
_ALIASES = {
    # 2-digit numbers
    **{f"{i:02d}": f"s{i:02d}" for i in range(1, 14)},
    # 1-digit numbers
    **{str(i): f"s{i:02d}" for i in range(1, 14)},
    # Upper-case
    **{f"S{i:02d}": f"s{i:02d}" for i in range(1, 14)},
    # Shorthand S1..S13
    **{f"s{i}": f"s{i:02d}" for i in range(1, 14)},
    **{f"S{i}": f"s{i:02d}" for i in range(1, 14)},
    "slip": "s_slip",
    "wheel_slip": "s_slip",
    "s_slip": "s_slip",
}


def get_all_scenarios() -> List[Dict[str, Any]]:
    """Return all 13 scenario dictionaries in sequential order."""
    out = []
    for sc in SCENARIOS_DATA:
        item = dict(sc)
        item["path"] = os.path.normpath(os.path.join(SCEN_DIR, item["file"]))
        out.append(item)
    return out


def resolve_scenario(spec: Any) -> Dict[str, Any]:
    """Strictly resolve a scenario identifier to its metadata and verified YAML file path.

    Rejects unknown scenario IDs with ValueError instead of silently falling back.
    """
    if spec is None:
        raise ValueError("Scenario specification is None. Must provide S01..S13.")

    spec_str = str(spec).strip()

    # If it's already an existing file path, check if it matches a registered scenario
    if os.path.isfile(spec_str):
        fpath = os.path.abspath(spec_str)
        fname = os.path.basename(fpath)
        for sc in SCENARIOS_DATA:
            if sc["file"] == fname or sc["file"] == os.path.basename(spec_str):
                item = dict(sc)
                item["path"] = fpath
                return item
        # If it's a custom YAML file, wrap as custom scenario
        return {
            "id": Path(fpath).stem,
            "name": Path(fpath).stem,
            "file": fname,
            "title": Path(fpath).stem,
            "path": fpath,
            "description": f"Custom scenario file {fname}",
            "expected_outcome": "GOAL_REACHED",
            "timeout": 60.0,
            "default_start": (6.0, -6.0, 0.35, 0.0),
            "default_goal": (13.5, -6.0, 0.0),
        }

    # Normalize key
    clean = spec_str.lower()
    canonical_id = _ALIASES.get(clean, _ALIASES.get(spec_str, clean))

    for sc in SCENARIOS_DATA:
        if sc["id"] == canonical_id or sc["file"] == spec_str or Path(sc["file"]).stem == clean:
            item = dict(sc)
            item["path"] = os.path.normpath(os.path.join(SCEN_DIR, item["file"]))
            if not os.path.isfile(item["path"]):
                raise FileNotFoundError(f"Scenario YAML file missing: {item['path']}")
            return item

    valid_ids = [sc["id"].upper() for sc in SCENARIOS_DATA]
    raise ValueError(
        f"Unknown scenario identifier: {spec!r}. "
        f"Valid identifiers are: {', '.join(valid_ids)} or direct path to YAML."
    )


def get_scenario_by_index(idx: int) -> Dict[str, Any]:
    """Get scenario metadata by 0-based index (0 to 12)."""
    if not (0 <= idx < len(SCENARIOS_DATA)):
        raise IndexError(f"Scenario index {idx} out of range (0..{len(SCENARIOS_DATA)-1})")
    item = dict(SCENARIOS_DATA[idx])
    item["path"] = os.path.normpath(os.path.join(SCEN_DIR, item["file"]))
    return item
