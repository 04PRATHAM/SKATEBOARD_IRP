# Reference Observations

This document summarizes observations from the reference photographs and videos of the physical vehicle and laboratory environment. These observations guide the simulation fidelity improvements in Milestone 4.5.

---

## 1. Physical Vehicle Observations

### 1.1 Overall Vehicle Characteristics
**Source:** `references/vehicle/` (4 photographs)

| Feature | Observation | Status |
|---------|-------------|--------|
| **Vehicle Type** | Compact four-wheeled research/experimental mobility platform (custom electric chassis / mobility scooter style) | Documented |
| **Overall Shape** | Open structural frame/chassis, rectangular footprint | Documented |
| **Wheel Configuration** | 4 large wheels (front: steered, rear: driven) | Documented |
| **Wheel Appearance** | Large diameter, visible tire tread, light/white colored rims | Documented |
| **Chassis/Frame** | Dark/black structural frame, open tubular construction | Documented |
| **Ground Clearance** | Relatively high (approx 15cm visible) | Documented |
| **Seat** | Elevated seat at rear, brown/tan color, with backrest | Documented |
| **Steering** | Visible steering wheel on column, front steering assembly visible | Documented |
| **Front Assembly** | Mechanical steering linkage/assembly at front | Documented |
| **Camera Mount** | Front center area, sensor mounted at approx 1.2m height | Documented |
| **Color Scheme** | Dark frame, light rims, tan/brown seat | Documented |

### 1.2 Dimensions (All ESTIMATED from photos)
| Parameter | Value | Source |
|-----------|-------|--------|
| Length | ~1.8 m | Estimated from photos |
| Width | ~1.2 m | Estimated from photos |
| Height (chassis) | ~1.0 m | Estimated from photos |
| Wheelbase | ~1.1 m | Estimated from photos |
| Front/Rear Track | ~1.0 m | Estimated from photos |
| Wheel Radius | ~0.25 m | Estimated from photos |
| Wheel Width | ~0.12 m | Estimated from photos |
| Ground Clearance | ~0.15 m | Estimated from photos |
| Seat Height | ~0.8-0.9 m from ground | Estimated |
| Camera Mount Height | ~1.2 m from ground | Estimated |
| Camera Mount Position | Front center, ~0.9m forward of rear axle | Estimated |

### 1.3 Current Simulation vs. Reference
| Component | Current Simulation | Reference | Gap |
|-----------|-------------------|-----------|-----|
| Chassis | Simple box (1.8×1.2×1.0) | Open tubular frame, dark | Major visual gap |
| Wheels | Simple cylinders | Large wheels, visible tread, white rims | Moderate gap |
| Seat | Simple box | Elevated with backrest, tan/brown | Minor gap (exists but simple) |
| Steering Wheel | Missing | Visible on column | Missing |
| Steering Column | Missing | Visible at front | Missing |
| Front Assembly | Missing | Visible mechanical linkage | Missing |
| Sensor Mount | Simple cylinder | Structured mount at front | Moderate gap |
| Frame/Chassis Detail | Solid box | Open tubular, dark | Major gap |

---

## 2. Laboratory Environment Observations

### 2.1 Overall Lab Characteristics
**Source:** `references/lab/` (37 photographs)

| Feature | Observation |
|---------|-------------|
| **Space Type** | Indoor academic research lab / workspace |
| **Floor** | Grey/beige vinyl tiles, tiled pattern |
| **Walls** | Concrete perimeter walls, some painted sections |
| **Ceiling** | Drop-ceiling grid with fluorescent/LED panels, ceiling fans |
| **Lighting** | Overhead panel lighting (fluorescent/LED) |

### 2.2 Key Architectural Elements
| Element | Description | Navigation Relevance |
|---------|-------------|---------------------|
| **Glass Partitions** | Floor-to-ceiling glass office walls | Critical for depth perception (refraction) |
| **Wood/Slatted Columns** | Wooden slatted pillars, ~0.6×0.6m | High - obstacle detection |
| **Yellow Accents** | Yellow painted walls/pillars | Visual landmark |
| **Green Elements** | Green painted walls/partitions | Visual landmark |
| **Windows/Blinds** | Large windows with horizontal blinds | Lighting, depth cues |
| **Desks** | Standard office desks (1.5×0.8×0.75m) | Obstacle |
| **Tables** | Lab tables (similar to desks) | Obstacle |
| **Chairs** | Rolling office chairs (0.5m radius seat) | Dynamic obstacle |
| **Corridors** | Open passages between furniture | Navigation paths |

### 2.3 Current Simulation vs. Reference
| Element | Current Simulation | Reference | Gap |
|---------|-------------------|-----------|-----|
| Floor | 50×50m gray plane | Tiled vinyl | Visual only |
| Walls | Single wall at x=5m | Perimeter walls + partitions | Major |
| Lighting | Dome + 1 distant light | Grid LED panels + ceiling fans | Major |
| Furniture | 1 box obstacle | Desks, tables, chairs | Major |
| Partitions | None | Glass, wood slats, yellow/green | Major |
| Columns | None | Slatted wood columns | Major |
| Corridors | Open space | Defined paths | Structural |
| Ceiling | None | Drop ceiling, fans | Visual |

---

## 3. Rectangular Box Identification

### Current Scene Object: `/World/Environment/Obstacle_Box`

| Property | Value |
|----------|-------|
| **Path** | `/World/Environment/Obstacle_Box` |
| **Type** | USD Cube primitive with collision |
| **Position** | (3.0, 1.0, 0.5) |
| **Scale** | (1.0, 1.0, 1.0) → 1m × 1m × 1m cube |
| **Height** | 0.5m above ground (center at 0.5m, so extends 0m to 1m) |
| **Physics** | Kinematic rigid body with collision |

### Purpose
This is the **temporary test obstacle** created in `minimal_env.py` for Milestone 2 physics testing. It serves no semantic purpose in the final environment.

### Why Visible
- Created as a simple 1m cube at position (3, 1)
- Has collision and kinematic rigid body
- No visual material applied (appears as default gray)
- Positioned in the vehicle's forward path for testing

### Action Required
**Replace with meaningful lab furniture** (table, desk, or cabinet) at semantically appropriate locations. Remove this generic test cube once proper furniture assets exist.

---

## 4. Current USD Stage Hierarchy

```
/World
├── /Environment (Xform)
│   ├── /GroundPlane (Cube, 50×50×0.1m, collision)
│   ├── /Wall (Cube, 0.2×6×3m at x=5, kinematic)
│   ├── /Obstacle_Box (Cube, 1×1×1m at (3,1), kinematic)  ← TEMPORARY
│   ├── /Person_01 (Xform at (4,-1))
│   │   ├── /Body (Capsule, radius 0.25, height 1.0)
│   │   └── /Head (Sphere, radius 0.15)
│   └── /Person_02 (Xform at (6,0))
│       └── /Body (Capsule, radius 0.25, height 1.0)
├── /Vehicle (Xform, ArticulationRoot at z=ground_clearance+height/2)
│   ├── /Chassis (Cube 1.8×1.2×1.0, rigid body, 150kg)
│   ├── /Wheel_FL (Cylinder, steering joint Z-axis)
│   ├── /Wheel_FR (Cylinder, steering joint Z-axis)
│   ├── /Wheel_RL (Cylinder, drive joint Y-axis)
│   ├── /Wheel_RR (Cylinder, drive joint Y-axis)
│   ├── /Seat (Cube, visual only)
│   └── /CameraMount (Cylinder, visual marker)
└── /Lighting (Xform)
    ├── /DomeLight (500 intensity)
    └── /DistantLight (3000 intensity, -45°/30°/0°)
```

---

## 5. Gaps Summary for Milestone 4.5

### Vehicle Visual Improvements (Priority Order)
1. **Chassis/Frame** - Replace solid box with open tubular frame structure
2. **Wheels** - Add tire tread visual, proper rim geometry, white rim color
3. **Steering Wheel** - Add on steering column at driver position
4. **Steering Column** - Vertical column from steering wheel to front assembly
5. **Front Steering Assembly** - Visible tie rods, knuckles, Ackermann linkage
6. **Sensor Mount** - Structured mount at front (0.9m forward, 1.2m height)
7. **Seat** - Improve with backrest, proper proportions, tan color

### Environment Improvements (Priority Order)
1. **Floor** - Tiled texture/appearance
2. **Walls** - Perimeter walls + room boundaries
3. **Glass Partitions** - Critical for depth perception testing
4. **Columns** - Wood/slatted columns at realistic positions
5. **Desks/Tables** - Replace Obstacle_Box with proper furniture
6. **Chairs** - Rolling office chairs
7. **Partitions** - Yellow/green accent partitions
8. **Lighting** - Grid ceiling lights + ceiling fans
9. **Corridors** - Defined navigable paths

---

## 6. Implementation Strategy

### Separation of Visual vs. Physics Geometry
| Component | Physics (Collision) | Visual |
|-----------|-------------------|--------|
| Chassis | Simple box (stable) | Tubular frame mesh |
| Wheels | Cylinder (stable) | Detailed wheel + rim |
| Seat | None (visual only) | Box with backrest |
| Steering | Joints only | Column + wheel + linkage |
| Furniture | Simple boxes | Detailed meshes |

### Implementation Order (Milestone 4.5)
1. Create modular asset functions in `isaac_sim/scripts/vehicle/` and `isaac_sim/scripts/environments/`
2. Implement improved vehicle visual geometry (preserving physics)
3. Implement lab environment with modular furniture
4. Replace Obstacle_Box with meaningful furniture
5. Improve lighting
6. Re-run physics test to verify stability

---

## 7. Documentation Updates Needed

After Milestone 4.5:
- `docs/vehicle_model.md` - Detailed vehicle model documentation
- `docs/environment_model.md` - Lab environment documentation
- `docs/simulation_workflow.md` - Updated workflow
- `docs/sensor_setup.md` - Camera setup details
- `docs/ros2_interfaces.md` - ROS 2 topic specifications
- `docs/reference_observations.md` - This document

---

*Document created: 2026-09-05*
*Based on: `references/vehicle/` (4 photos), `references/lab/` (37 photos), `simulation_world_plan.md`*
*All dimensions marked ESTIMATED unless MEASURED*