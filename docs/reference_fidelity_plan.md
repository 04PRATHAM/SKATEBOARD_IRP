# Milestone 4.5 — Reference Fidelity Pass Implementation Plan

This plan details the specific implementation steps to improve visual fidelity while preserving the working physics implementation.

---

## Phase 1: Vehicle Visual Improvements

### 1.1 Chassis/Frame — Open Tubular Structure
**File:** `isaac_sim/scripts/vehicle/visual_vehicle.py` (new)
**Approach:** Keep physics chassis as simple box. Add visual frame as child prims.

| Component | Visual Prim | Position (relative to chassis center) | Scale | Color |
|-----------|-------------|--------------------------------------|-------|-------|
| Main longitudinal rails (2x) | Cube | (±length/2, ±width/2, 0) | (length, 0.08, 0.08) | Dark gray [0.1,0.1,0.1] |
| Cross members (3x) | Cube | (0, ±width/2, 0), (±length/3, 0, 0) | (0.08, width, 0.08) | Dark gray |
| Front bumper | Cube | (length/2, 0, -height/2) | (0.1, width, height) | Dark gray |
| Rear bumper | Cube | (-length/2, 0, -height/2) | (0.1, width, height) | Dark gray |

**Implementation:** Add visual prims as children of chassis Xform. No physics/collision on visual prims.

### 1.2 Wheels — Detailed with Rims
**File:** `isaac_sim/scripts/vehicle/visual_vehicle.py`
**Approach:** Keep physics wheels as cylinders. Add visual wheel + rim as child prims.

| Component | Visual Prim | Scale | Color |
|-----------|-------------|-------|-------|
| Tire | Cylinder (same as physics) | radius=0.25, width=0.12 | Dark gray [0.2,0.2,0.2] |
| Rim | Cylinder (inside tire) | radius=0.18, width=0.13 | White [0.9,0.9,0.9] |
| Hub | Cylinder (center) | radius=0.06, width=0.14 | Light gray [0.7,0.7,0.7] |

### 1.3 Steering Wheel
**Position:** Rear area, driver side (left), height ~0.9m from ground
**Visual:** Cylinder (wheel) + Cylinder (column)
- Wheel: radius=0.18, width=0.03, oriented horizontal
- Column: radius=0.02, height=0.4, vertical

### 1.4 Steering Column
**Position:** Front of seat area, connecting to front steering assembly
**Visual:** Cylinder, radius=0.025, height=0.6, vertical

### 1.5 Front Steering Assembly (Ackermann Linkage)
**Components:**
- Steering rack: Horizontal cylinder at front axle, width=track
- Tie rods (2x): Cylinders from rack to wheel knuckles
- Knuckles (2x): Small cubes at wheel attachment points

### 1.6 Sensor Mount
**Position:** Front center, x=0.9m forward of rear axle, z=1.2m
**Visual:** Rectangular box (0.15×0.1×0.1m) + small cylinder for camera

### 1.7 Seat Improvement
**Current:** Simple box
**Improved:** 
- Seat base: Cube (0.6×0.5×0.1) at z=0.5
- Backrest: Cube (0.6×0.1×0.5) at rear of seat base
- Color: Tan [0.6,0.4,0.2]

---

## Phase 2: Environment Improvements

### 2.1 Modular Asset Functions
**File:** `isaac_sim/scripts/environments/lab_assets.py` (new)

**Asset Functions:**
```python
def create_lab_table(stage, path, translation, scale=(1.5, 0.8, 0.75))
def create_lab_chair(stage, path, translation)
def create_slatted_column(stage, path, translation, height=3.0)
def create_glass_partition(stage, path, translation, scale=(0.1, 2.0, 2.5))
def create_yellow_partition(stage, path, translation, scale=(0.15, 2.0, 2.5))
def create_green_partition(stage, path, translation, scale=(0.15, 2.0, 2.5))
def create_wall(stage, path, translation, scale)
def create_floor_tiles(stage, path, tile_size=0.3)
def create_ceiling_lights(stage, path, grid_spacing=2.0)
def create_ceiling_fan(stage, path, translation)
```

### 2.2 Environment Layout (Based on Reference Photos)
**Key Positions (approximate):**
- Tables along walls
- Chairs at tables
- Columns at regular intervals
- Glass partitions defining offices
- Yellow/green accent walls
- Corridors between furniture rows

### 2.3 Replace Obstacle_Box
**Remove:** `/World/Environment/Obstacle_Box` (the 1m test cube)
**Replace with:** Meaningful furniture at semantically correct positions

---

## Phase 3: Lighting Improvements

### 3.1 Ceiling Light Grid
- Grid of rectangular area lights (simulating LED panels)
- Spacing: ~2m × 2m grid
- Intensity: ~1000 each
- Color: Slightly warm [1.0, 0.95, 0.9]

### 3.2 Ceiling Fans
- 4-6 fans distributed across ceiling
- Visual: Central hub + 4 blades
- Position: Between light panels

---

## Phase 4: Implementation Files

### New Files to Create
1. `isaac_sim/scripts/vehicle/visual_vehicle.py` — Vehicle visual geometry
2. `isaac_sim/scripts/environments/lab_assets.py` — Modular lab asset functions
3. `isaac_sim/scripts/environments/lab_environment.py` — Full lab layout
4. `isaac_sim/scripts/run_fidelity_test.py` — Test script for fidelity pass

### Files to Modify
1. `isaac_sim/scripts/vehicle/simple_vehicle.py` — Import visual_vehicle, call after physics creation
2. `isaac_sim/scripts/environment/minimal_env.py` → `lab_environment.py` (replace)
3. `isaac_sim/scripts/run_minimal_world.py` → Use new lab environment
4. `configs/vehicle.yaml` — Add visual parameters if needed

---

## Phase 5: Testing & Verification

### Test Script: `run_fidelity_test.py`
```bash
/home/ubuntu/isaacsim/python.sh isaac_sim/scripts/run_fidelity_test.py --duration 20.0
```

### Verification Checklist
- [ ] Vehicle drives forward (0-5s)
- [ ] Vehicle stops (5-7s)
- [ ] Vehicle turns left with Ackermann (7-12s)
- [ ] Vehicle stops (12-14s)
- [ ] Vehicle turns right with Ackermann (14-19s)
- [ ] Vehicle stops (19-20s)
- [ ] No physics explosions
- [ ] Wheels maintain ground contact
- [ ] Visual geometry renders correctly
- [ ] Environment renders correctly

---

## Configuration Updates

### vehicle.yaml additions (if needed)
```yaml
visual:
  chassis_color: [0.1, 0.1, 0.1, 1.0]
  wheel_rim_color: [0.9, 0.9, 0.9, 1.0]
  seat_color: [0.6, 0.4, 0.2, 1.0]
  frame_color: [0.1, 0.1, 0.1, 1.0]
  tire_color: [0.2, 0.2, 0.2, 1.0]
  hub_color: [0.7, 0.7, 0.7, 1.0]
```

---

## Implementation Order

1. **Create visual_vehicle.py** — Vehicle visual geometry
2. **Create lab_assets.py** — Modular furniture functions
4. **Create lab_environment.py** — Full lab layout
5. **Update simple_vehicle.py** — Integrate visual geometry
6. **Create run_fidelity_test.py** — Test script
7. **Run test** — Verify physics still works
8. **Update documentation**

---

## Key Principles

1. **Preserve Physics** — Never modify physics prims (chassis, wheels, joints)
2. **Separate Visual** — Add visual prims as children, no physics on visuals
3. **Modular Assets** — Reusable functions for furniture
3. **Configurable** - All dimensions in YAML configs
4. **Test After Each Step** - Run physics test after each major change

---

## Estimated Effort

| Task | Estimated Time |
|------|---------------|
| Vehicle visual geometry | 2-3 hours |
| Lab asset functions | 2-3 hours |
| Lab environment layout | 2-3 hours |
| Integration & testing | 1-2 hours |
| Documentation updates | 1 hour |
| **Total** | **~10-12 hours** |

---

*Plan created: 2026-09-05*
*Based on: reference_observations.md, simulation_world_plan.md*