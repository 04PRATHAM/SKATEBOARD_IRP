"""
Full Laboratory Environment Module (SKATEBOARD_IRP Digital Twin Lab)
Creates a visually recognizable 3D simulation of our actual research laboratory:
- Tiled light beige/grey floor
- Perimeter white walls with yellow architectural accent panels and green partition sections
- Recessed ceiling with linear LED light panels and ceiling fans
- Vertical wooden slatted architectural columns
- Translucent glass office partitions
- Modular furniture: lab work tables, computer desks, office research chairs
- Clean USD hierarchy under /World/Environment

All environment objects have simplified collision geometry to maintain simulation performance.
Uses exact 1:1 metric sizing by setting UsdGeom.Cube size attribute to 1.0.
"""

import math
import numpy as np


def create_lab_environment(world, vehicle_cfg: dict, glass_has_collision: bool = True):
    """
    Create a representative laboratory environment matching reference photographs.
    
    Args:
        world: Isaac Sim World instance
        vehicle_cfg: Vehicle configuration (for sizing reference)
        glass_has_collision: If True, glass partitions are solid obstacles.
            If False, glass is visual-only (for motion-test corridors).
    """
    from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf, Sdf
    import omni.usd
    from isaacsim.core.utils.prims import define_prim, get_prim_at_path
    
    stage = omni.usd.get_context().get_stage()
    
    def safe_define_prim(path: str, prim_type: str = "Xform"):
        prim = get_prim_at_path(path)
        if prim and prim.IsValid():
            return prim
        return define_prim(path, prim_type)
    
    # Color palette matching lab photographs
    color_floor = Gf.Vec3f(0.78, 0.76, 0.72)         # Light beige/grey tile
    color_wall = Gf.Vec3f(0.92, 0.92, 0.90)          # Off-white wall
    color_yellow_accent = Gf.Vec3f(0.95, 0.75, 0.10) # Vibrant yellow architectural panel
    color_green_accent = Gf.Vec3f(0.20, 0.65, 0.35)  # Green wall partition section
    color_wood = Gf.Vec3f(0.55, 0.38, 0.22)          # Warm wood slat
    color_glass = Gf.Vec3f(0.70, 0.88, 0.95)         # Translucent glass tint
    color_desk = Gf.Vec3f(0.35, 0.35, 0.38)          # Dark grey desk surface
    color_chair = Gf.Vec3f(0.20, 0.20, 0.22)         # Black office chair
    
    # ============================================================
    # ENVIRONMENT DIMENSIONS - Large navigable laboratory
    # ============================================================
    room_width = 30.0     # X dimension (front-to-back)
    room_depth = 25.0     # Y dimension (left-to-right)
    ceiling_height = 3.5  # Floor-to-ceiling height (3.5 meters)
    floor_thickness = 0.04
    wall_thickness = 0.20
    
    # Environment root hierarchy
    env_path = "/World/Environment"
    safe_define_prim(env_path, "Xform")
    
    # Sub-groups according to specification
    floor_path = f"{env_path}/Floor"
    walls_path = f"{env_path}/Walls"
    partitions_path = f"{env_path}/Partitions"
    glass_path = f"{env_path}/GlassPartitions"
    arch_path = f"{env_path}/ArchitecturalElements"
    furniture_path = f"{env_path}/Furniture"
    ceiling_path = f"{env_path}/Ceiling"
    
    for p in [floor_path, walls_path, partitions_path, glass_path, arch_path, furniture_path, ceiling_path]:
        safe_define_prim(p, "Xform")
        
    safe_define_prim(f"{furniture_path}/Desks", "Xform")
    safe_define_prim(f"{furniture_path}/Chairs", "Xform")
    safe_define_prim(f"{furniture_path}/Tables", "Xform")

    # Helper for metric cubes; has_collision=False for visual-only deco (tile lines)
    def create_cube(path, translation, scale, color=None, kinematic=True, has_collision=True):
        prim = safe_define_prim(path, "Cube")
        cube_geom = UsdGeom.Cube(prim)
        cube_geom.CreateSizeAttr().Set(1.0)
        xform = UsdGeom.Xformable(prim)
        
        # Set translate - check if op exists, otherwise add
        translate_op = xform.GetTranslateOp()
        if not translate_op:
            translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*translation))
        
        # Set scale - check if op exists, otherwise add
        scale_op = xform.GetScaleOp()
        if not scale_op:
            scale_op = xform.AddScaleOp()
        scale_op.Set(Gf.Vec3d(*scale))
        
        if color:
            disp_attr = prim.GetAttribute("primvars:displayColor")
            if not disp_attr:
                disp_attr = prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
            disp_attr.Set([color])
        if has_collision:
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
            if kinematic:
                rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
                rigid_api.CreateKinematicEnabledAttr().Set(True)
        return prim

    def create_cylinder(path, translation, radius, height, orientation=None, color=None):
        prim = safe_define_prim(path, "Cylinder")
        xform = UsdGeom.Xformable(prim)
        
        # Set translate
        translate_op = xform.GetTranslateOp()
        if not translate_op:
            translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*translation))
        
        # Set rotation if provided
        if orientation:
            rotate_op = xform.GetRotateXYZOp()
            if not rotate_op:
                rotate_op = xform.AddRotateXYZOp()
            rotate_op.Set(Gf.Vec3d(*orientation))
        
        prim.GetAttribute("radius").Set(radius)
        prim.GetAttribute("height").Set(height)
        if color:
            disp_attr = prim.GetAttribute("primvars:displayColor")
            if not disp_attr:
                disp_attr = prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
            disp_attr.Set([color])
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
        return prim

    # ============================================================
    # 1. LIGHT BEIGE/GREY TILED LABORATORY FLOOR (z = -0.10 to 0.0)
    # ============================================================
    # The visual tiled floor is a VISUAL-ONLY cube (has_collision=False). The
    # physics contact surface is a dedicated infinite PhysX ground *plane* at
    # z=0 (created below). A thin/large box collider generated unreliable
    # cylinder-wheel contact for the front (knuckle-chain) wheels — they sank
    # into the box and the chassis pitched nose-down ~6°, which is what caused
    # the "shaking". An infinite ground plane produces perfectly stable contact
    # for all four wheels (verified: chassis z-spread 0.11 m -> 0.00001 m).
    floor_thickness = 0.20
    create_cube(f"{floor_path}/Tile_Ground_Plane", (0.0, 0.0, -floor_thickness/2.0), (room_width, room_depth, floor_thickness), color_floor,
                has_collision=False)

    # Dedicated physics ground plane (infinite, invisible). This is the real
    # collision surface the wheels roll on; the tiled cube above is decoration.
    # Only create if it doesn't exist
    gp_prim = stage.GetPrimAtPath(f"{floor_path}/PhysicsGroundPlane")
    if not gp_prim.IsValid():
        try:
            from isaacsim.core.api.objects import GroundPlane
            GroundPlane(f"{floor_path}/PhysicsGroundPlane", z_position=0.0, visible=False)
            print("[INFO] Physics ground plane (infinite, z=0) created for stable wheel contact.")
        except Exception as e:
            # Fallback: give the visual floor cube collision back so the vehicle
            # still has *something* to rest on if the ground-plane helper is absent.
            print(f"[WARN] GroundPlane helper unavailable ({e}); re-enabling box floor collision.")
            from pxr import UsdPhysics as _UsdPhysics
            _fp = get_prim_at_path(f"{floor_path}/Tile_Ground_Plane")
            if _fp and _fp.IsValid():
                _UsdPhysics.CollisionAPI.Apply(_fp.GetPrim())
    
    # Tile grid lines (thin lines at z = 0.001)
    tile_size = 1.0  # 1-meter tiles
    line_t = 0.01    # 1cm line thickness
    line_h = 0.002   # 2mm height
    line_color = Gf.Vec3f(0.65, 0.63, 0.59)
    
    num_y_lines = int(room_depth / tile_size) + 1
    for iy in range(num_y_lines):
        ly = -room_depth/2.0 + iy * tile_size
        create_cube(f"{floor_path}/Tile_Line_Y_{iy}", (0.0, ly, line_h/2.0), (room_width, line_t, line_h), line_color, has_collision=False)
    num_x_lines = int(room_width / tile_size) + 1
    for ix in range(num_x_lines):
        lx = -room_width/2.0 + ix * tile_size
        create_cube(f"{floor_path}/Tile_Line_X_{ix}", (lx, 0.0, line_h/2.0), (line_t, room_depth, line_h), line_color, has_collision=False)

    # ============================================================
    # 2. PERIMETER WHITE WALLS & ACCENT SECTIONS (z = 0.0 to 3.5m)
    # ============================================================
    wall_h = ceiling_height
    wall_t = wall_thickness
    half_w = room_width / 2.0
    half_d = room_depth / 2.0
    wall_center_z = wall_h / 2.0  # z = 1.75m -> spans from z = 0.0 to z = 3.5m!
    
    # Back Wall (x = -15m)
    create_cube(f"{walls_path}/Wall_Back", (-half_w, 0.0, wall_center_z), (wall_t, room_depth, wall_h), color_wall)
    
    # Front Wall (x = +15m) with 3m doorway opening
    doorway_w = 3.0
    side_w = (room_depth - doorway_w) / 2.0
    create_cube(f"{walls_path}/Wall_Front_L", (half_w, -half_d + side_w/2.0, wall_center_z), (wall_t, side_w, wall_h), color_wall)
    create_cube(f"{walls_path}/Wall_Front_R", (half_w, half_d - side_w/2.0, wall_center_z), (wall_t, side_w, wall_h), color_wall)
    
    # Left Wall (y = -12.5m) & Right Wall (y = +12.5m)
    create_cube(f"{walls_path}/Wall_Left", (0.0, -half_d, wall_center_z), (room_width, wall_t, wall_h), color_wall)
    create_cube(f"{walls_path}/Wall_Right", (0.0, half_d, wall_center_z), (room_width, wall_t, wall_h), color_wall)
    
    # Green Accent Wall Section (Left wall feature)
    create_cube(f"{partitions_path}/Green_Accent_Wall", (-4.0, -half_d + 0.11, wall_center_z), (8.0, 0.02, wall_h), color_green_accent)

    # ============================================================
    # 3. WOODEN SLATTED COLUMNS & YELLOW ACCENT PANELS (z = 0.0 to 3.5m)
    # ============================================================
    # Yellow Architectural Pillar Panels
    create_cube(f"{arch_path}/Yellow_Panel_1", (-8.0, -8.0, wall_center_z), (0.6, 0.15, wall_h), color_yellow_accent)
    create_cube(f"{arch_path}/Yellow_Panel_2", (8.0, 8.0, wall_center_z), (0.6, 0.15, wall_h), color_yellow_accent)
    create_cube(f"{arch_path}/Yellow_Panel_3", (-8.0, 8.0, wall_center_z), (0.6, 0.15, wall_h), color_yellow_accent)
    create_cube(f"{arch_path}/Yellow_Panel_4", (8.0, -8.0, wall_center_z), (0.6, 0.15, wall_h), color_yellow_accent)
    
    # Wooden Vertical Slat Architectural Columns
    col_positions = [
        (-10.0, -8.0), (0.0, -8.0), (10.0, -8.0),
        (-10.0, 8.0), (0.0, 8.0), (10.0, 8.0),
        (-10.0, 0.0), (10.0, 0.0),
    ]
    for i, (cx, cy) in enumerate(col_positions):
        col_group = f"{arch_path}/Wood_Column_{i}"
        safe_define_prim(col_group, "Xform")
        create_cube(f"{col_group}/Core", (cx, cy, wall_center_z), (0.5, 0.5, wall_h), color_wood)
        # Visual slats around column
        for s in range(8):
            angle_rad = math.radians(s * 45.0)
            slat_x = cx + 0.27 * math.cos(angle_rad)
            slat_y = cy + 0.27 * math.sin(angle_rad)
            create_cube(f"{col_group}/Slat_{s}", (slat_x, slat_y, wall_center_z), (0.04, 0.06, wall_h), color_wood)

    # ============================================================
    # 4. TRANSLUCENT GLASS OFFICE PARTITIONS (z = 0.0 to 3.0m)
    # ============================================================
    # Glass now has collision (has_collision=True) so the vehicle
    # cannot pass through. For motion tests that need a clear
    # corridor, the collision can be disabled via the
    # `glass_has_collision` parameter passed to create_lab_environment.
    glass_h = 3.0
    glass_center_z = glass_h / 2.0
    create_cube(f"{glass_path}/Glass_Office_L", (-5.0, -5.0, glass_center_z), (0.08, 10.0, glass_h), color_glass, has_collision=glass_has_collision)
    create_cube(f"{glass_path}/Glass_Office_R", (5.0, 5.0, glass_center_z), (0.08, 10.0, glass_h), color_glass, has_collision=glass_has_collision)
    create_cube(f"{glass_path}/Glass_Office_Rear", (-10.0, 0.0, glass_center_z), (0.08, 8.0, glass_h), color_glass, has_collision=glass_has_collision)

    # ============================================================
    # 5. LAB FURNITURE (Work Tables, Computer Desks, Chairs)
    # ============================================================
    tables_path = f"{furniture_path}/Tables"
    desks_path = f"{furniture_path}/Desks"
    chairs_path = f"{furniture_path}/Chairs"
    
    # Back Work Tables — moved clear of yellow panels at (±8,±8) and wood columns
    # Yellow panels at (-8,-8),(8,8),(-8,8),(8,-8); columns at (-10,±8),(0,±8),(10,±8) etc.
    back_table_configs = [
        ("Back_Table_1", -12.0, -6.0),
        ("Back_Table_2", -12.0, 6.0),
        ("Back_Table_3", -12.0, -8.5),
        ("Back_Table_4", -12.0, 8.5),
    ]
    color_leg = Gf.Vec3f(0.55, 0.55, 0.58)  # lighter aluminium so legs read clearly
    for name, tx, ty in back_table_configs:
        # Top at height 0.75m (top surface 0.75, bottom 0.69)
        create_cube(f"{tables_path}/{name}_Top", (tx, ty, 0.72), (1.6, 0.9, 0.06), color_desk)
        # 4 legs: vertical cylinders, bottom exactly on floor (z 0.0 -> 0.69)
        leg_i = 0
        for lx in [tx - 0.7, tx + 0.7]:
            for ly in [ty - 0.38, ty + 0.38]:
                create_cylinder(f"{tables_path}/{name}_Leg_{leg_i}", (lx, ly, 0.345), 0.05, 0.69, None, color_leg)
                # foot pad so leg contact with floor is visible, not z-fighting
                create_cylinder(f"{tables_path}/{name}_Foot_{leg_i}", (lx, ly, 0.01), 0.07, 0.02, None, Gf.Vec3f(0.2, 0.2, 0.22))
                leg_i += 1

    # Left & Right Office Desks — clear of glass partitions (x=±5) and columns (y=±8)
    desk_configs = [
        ("Left_Desk_1", -6.5, -10.5), ("Left_Desk_2", -2.5, -10.5),
        ("Right_Desk_1", 6.5, 10.5), ("Right_Desk_2", 2.5, 10.5),
    ]
    for name, tx, ty in desk_configs:
        create_cube(f"{desks_path}/{name}_Top", (tx, ty, 0.72), (1.2, 0.7, 0.06), color_desk)
        leg_i = 0
        for lx in [tx - 0.5, tx + 0.5]:
            for ly in [ty - 0.3, ty + 0.3]:
                create_cylinder(f"{desks_path}/{name}_Leg_{leg_i}", (lx, ly, 0.345), 0.05, 0.69, None, color_leg)
                create_cylinder(f"{desks_path}/{name}_Foot_{leg_i}", (lx, ly, 0.01), 0.07, 0.02, None, Gf.Vec3f(0.2, 0.2, 0.22))
                leg_i += 1

    # Center Meeting Table (0,0) — clear of all columns (nearest at ±8-10m)
    create_cube(f"{tables_path}/Meeting_Table_Top", (0.0, 0.0, 0.72), (2.4, 1.2, 0.06), color_desk)
    leg_i = 0
    for lx in [-1.1, 1.1]:
        for ly in [-0.5, 0.5]:
            create_cylinder(f"{tables_path}/Meeting_Table_Leg_{leg_i}", (lx, ly, 0.345), 0.055, 0.69, None, color_leg)
            create_cylinder(f"{tables_path}/Meeting_Table_Foot_{leg_i}", (lx, ly, 0.01), 0.075, 0.02, None, Gf.Vec3f(0.2, 0.2, 0.22))
            leg_i += 1

    # Office Research Chairs - tucked behind their desks/tables, clear of pillars
    chair_configs = [
        ("Chair_B1", -12.0, -5.0), ("Chair_B2", -12.0, 7.0),
        ("Chair_B3", -12.0, -9.5), ("Chair_B4", -12.0, 9.5),
        ("Chair_L1", -6.5, -11.4), ("Chair_L2", -2.5, -11.4),
        ("Chair_R1", 6.5, 11.4), ("Chair_R2", 2.5, 11.4),
        ("Chair_M1", -1.0, -1.5), ("Chair_M2", 1.0, -1.5),
        ("Chair_M3", -1.0, 1.5), ("Chair_M4", 1.0, 1.5),
    ]
    for name, cx, cy in chair_configs:
        chair_path = f"{chairs_path}/{name}"
        safe_define_prim(chair_path, "Xform")
        
        # Seat cushion (comfortable padded seat)
        create_cylinder(f"{chair_path}/Seat", (cx, cy, 0.47), 0.25, 0.06, None, color_chair)
        # Seat cushion bottom (thickness)
        create_cylinder(f"{chair_path}/Seat_Bottom", (cx, cy, 0.44), 0.25, 0.02, None, Gf.Vec3f(0.15, 0.15, 0.15))
        
        # Lumbar support / Backrest (ergonomic curved back)
        create_cube(f"{chair_path}/Backrest_Lumbar", (cx - 0.22, cy, 0.75), (0.08, 0.40, 0.20), color_chair)
        create_cube(f"{chair_path}/Backrest_Upper", (cx - 0.22, cy, 0.95), (0.08, 0.40, 0.25), color_chair)
        # Backrest frame/support
        create_cube(f"{chair_path}/Backrest_Frame", (cx - 0.26, cy, 0.80), (0.04, 0.44, 0.50), Gf.Vec3f(0.15, 0.15, 0.15))
        
        # Seat frame/pan
        create_cube(f"{chair_path}/Seat_Frame", (cx, cy, 0.43), (0.52, 0.52, 0.03), Gf.Vec3f(0.15, 0.15, 0.15))
        
        # Gas lift cylinder (height adjustment)
        create_cylinder(f"{chair_path}/Gas_Lift", (cx, cy, 0.22), 0.035, 0.40, None, Gf.Vec3f(0.4, 0.4, 0.45))
        
        # 5-star base with casters
        base_z = 0.03
        create_cylinder(f"{chair_path}/Base_Main", (cx, cy, base_z), 0.32, 0.02, None, Gf.Vec3f(0.18, 0.18, 0.20))
        # 5 caster wheels on star base
        for i in range(5):
            angle = i * 72.0  # 5 spokes
            caster_x = cx + 0.28 * math.cos(math.radians(angle))
            caster_y = cy + 0.28 * math.sin(math.radians(angle))
            create_cylinder(f"{chair_path}/Caster_{i}", (caster_x, caster_y, 0.025), 0.035, 0.02, None, Gf.Vec3f(0.1, 0.1, 0.1))
            # Caster stem
            create_cylinder(f"{chair_path}/Caster_Stem_{i}", (caster_x, caster_y, 0.015), 0.01, 0.03, None, Gf.Vec3f(0.2, 0.2, 0.2))
        
        # Central column connecting gas lift to base
        create_cylinder(f"{chair_path}/Central_Column", (cx, cy, 0.15), 0.045, 0.45, None, Gf.Vec3f(0.35, 0.35, 0.40))

    # ============================================================
    # 6. CEILING & LIGHTING GRID (z = 3.50m to 3.60m)
    # ============================================================
    ceiling_thickness = 0.10
    ceiling_center_z = wall_h + ceiling_thickness / 2.0  # z = 3.55m -> spans z = 3.50 to 3.60m!
    create_cube(f"{ceiling_path}/Ceiling_Plane", (0.0, 0.0, ceiling_center_z), (room_width, room_depth, ceiling_thickness), color_wall)
    
    # Lighting Root
    lighting_path = "/World/Lighting"
    safe_define_prim(lighting_path, "Xform")
    
    # Dome Light (Bright Ambient indoor fill) - only create if not exists
    dome_light_prim = stage.GetPrimAtPath(f"{lighting_path}/DomeLight")
    if not dome_light_prim.IsValid():
        dome_light = UsdLux.DomeLight.Define(stage, f"{lighting_path}/DomeLight")
        dome_light.CreateIntensityAttr().Set(4000.0)
        dome_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.95))
    
    # Distant Window/Sun Light (Bright directional) - only create if not exists
    distant_light_prim = stage.GetPrimAtPath(f"{lighting_path}/DistantLight")
    if not distant_light_prim.IsValid():
        distant_light = UsdLux.DistantLight.Define(stage, f"{lighting_path}/DistantLight")
        distant_light.CreateIntensityAttr().Set(15000.0)
        distant_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.90))
        xform_distant = UsdGeom.Xformable(distant_light.GetPrim())
        rotate_op = xform_distant.GetRotateXYZOp()
        if not rotate_op:
            rotate_op = xform_distant.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3f(-45.0, 30.0, 0.0))
    
    # Recessed Linear LED Panel Grid across ceiling (tighter spacing at 3m for better coverage)
    led_i = 0
    light_z = wall_h - 0.05  # Flush with ceiling underside (z = 3.45m)
    led_spacing = 3.0
    num_x = int(room_width / led_spacing) + 1
    num_y = int(room_depth / led_spacing) + 1
    start_x = -room_width / 2.0
    start_y = -room_depth / 2.0
    for ix in range(num_x):
        for iy in range(num_y):
            lx = start_x + ix * led_spacing
            ly = start_y + iy * led_spacing
            light_path = f"{lighting_path}/Ceiling_LED_{led_i}"
            led_i += 1
            light_prim = stage.GetPrimAtPath(light_path)
            if not light_prim.IsValid():
                rect_light = UsdLux.RectLight.Define(stage, light_path)
                rect_light.CreateWidthAttr().Set(1.2)
                rect_light.CreateHeightAttr().Set(0.6)
                rect_light.CreateIntensityAttr().Set(4000.0)
                rect_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.95))
                xform_light = UsdGeom.Xformable(rect_light.GetPrim())
                translate_op = xform_light.GetTranslateOp()
                if not translate_op:
                    translate_op = xform_light.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(lx, ly, light_z))
                rotate_op = xform_light.GetRotateXYZOp()
                if not rotate_op:
                    rotate_op = xform_light.AddRotateXYZOp()
                rotate_op.Set(Gf.Vec3d(-90.0, 0.0, 0.0))
    
    # Additional accent lights for work areas (warmer, focused)
    accent_i = 0
    accent_positions = [
        (-12.0, -6.0, 3.2), (-12.0, 6.0, 3.2),   # Back tables
        (-6.0, -10.0, 3.2), (-2.0, -10.0, 3.2),  # Left office
        (6.0, 10.0, 3.2), (2.0, 10.0, 3.2),      # Right office
        (0.0, 0.0, 3.2),                          # Meeting table
    ]
    for ax, ay, az in accent_positions:
        light_path = f"{lighting_path}/Accent_LED_{accent_i}"
        accent_i += 1
        light_prim = stage.GetPrimAtPath(light_path)
        if not light_prim.IsValid():
            accent_light = UsdLux.RectLight.Define(stage, light_path)
            accent_light.CreateWidthAttr().Set(0.8)
            accent_light.CreateHeightAttr().Set(0.8)
            accent_light.CreateIntensityAttr().Set(3000.0)
            accent_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.85))
            xform_accent = UsdGeom.Xformable(accent_light.GetPrim())
            translate_op = xform_accent.GetTranslateOp()
            if not translate_op:
                translate_op = xform_accent.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(ax, ay, az))
            rotate_op = xform_accent.GetRotateXYZOp()
            if not rotate_op:
                rotate_op = xform_accent.AddRotateXYZOp()
            rotate_op.Set(Gf.Vec3d(-90.0, 0.0, 0.0))
    
    # Ceiling Fans hanging 25cm below ceiling
    fan_positions = [(-8.0, -6.0), (8.0, -6.0), (-8.0, 6.0), (8.0, 6.0), (0.0, 0.0)]
    for fi, (fx, fy) in enumerate(fan_positions):
        fan_path = f"{ceiling_path}/Fan_{fi}"
        safe_define_prim(fan_path, "Xform")
        fan_z = wall_h - 0.25
        create_cylinder(f"{fan_path}/Hub", (fx, fy, fan_z), 0.15, 0.10, None, color_chair)
        for b in range(4):
            create_cube(f"{fan_path}/Blade_{b}", (fx, fy, fan_z - 0.05), (0.90, 0.12, 0.02), color_chair)

    print(f"[INFO] Laboratory environment digital twin created under {env_path}")
    print(f"  Floor: Tiled beige/grey ({room_width}x{room_depth})m plane at z=0")
    print(f"  Walls: 3.5m high off-white walls + green accent section")
    print(f"  Architecture: Yellow pillar panels + {len(col_positions)} wooden slatted columns (3.5m high)")
    print(f"  Ceiling: Placed at z=3.50m (bottom surface) to 3.60m (top surface)")


if __name__ == "__main__":
    print("Laboratory environment module loaded successfully")