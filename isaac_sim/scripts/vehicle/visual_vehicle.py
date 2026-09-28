"""
High-Fidelity Vehicle Visual Geometry Module (SKATEBOARD_IRP Digital Twin)
Provides high-fidelity visual digital twin geometry matching the physical research vehicle:
- Open black industrial tubular frame chassis & diamond-plate floor platform
- Tan/brown leather padded seat & backrest with armrests & black mounting frame
- Angled steering column, black steering wheel rim, and front steering console
- Dual front suspension wishbones, shock absorbers, and VIBRANT ORANGE COIL SPRINGS
- Extruded aluminum horizontal front bumper bar
- Large black pneumatic rubber tires with off-white wheel rims and central axle hubs
- Rear black battery & electronics housing enclosure with red/white light assemblies
- Physical RGB-D camera sensor box mounted on front vertical riser post at (0.9, 0, 1.2) m

Uses exact 1:1 metric sizing by setting UsdGeom.Cube size attribute to 1.0.
"""

import math
import numpy as np


def create_vehicle_visuals(vehicle_path: str, vehicle_cfg: dict):
    """
    Create high-fidelity visual digital twin geometry for the vehicle.
    
    Args:
        vehicle_path: USD path to vehicle root (e.g., "/World/Vehicle")
        vehicle_cfg: Vehicle configuration dictionary
    """
    from pxr import Usd, UsdGeom, Gf, Sdf
    import omni.usd
    from isaacsim.core.utils.prims import define_prim, get_prim_at_path
    
    stage = omni.usd.get_context().get_stage()
    
    def safe_define_prim(path: str, prim_type: str = "Xform"):
        prim = get_prim_at_path(path)
        if prim and prim.IsValid():
            return prim
        return define_prim(path, prim_type)
    
    # Vehicle dimensions from config
    length = vehicle_cfg.get("length", 1.8)
    width = vehicle_cfg.get("width", 1.2)
    height = vehicle_cfg.get("height", 0.4)
    wheelbase = vehicle_cfg.get("wheelbase", 1.1)
    front_track = vehicle_cfg.get("front_track", 1.0)
    rear_track = vehicle_cfg.get("rear_track", 1.0)
    wheel_radius = vehicle_cfg.get("wheel_radius", 0.25)
    wheel_width = vehicle_cfg.get("wheel_width", 0.18)
    ground_clearance = vehicle_cfg.get("ground_clearance", 0.15)
    
    # Vehicle root is at world z = ground_clearance + height/2 = 0.35m
    vehicle_root_z = ground_clearance + height / 2.0  # 0.35m
    wheel_local_z = wheel_radius - vehicle_root_z      # 0.25 - 0.35 = -0.10m
    
    # Visual color palettes (Matching real vehicle reference photos)
    color_chassis = Gf.Vec3f(0.10, 0.10, 0.10)       # Dark industrial black metal
    color_floor = Gf.Vec3f(0.15, 0.15, 0.15)         # Diamond-plate black platform
    color_seat = Gf.Vec3f(0.58, 0.38, 0.18)          # Tan/brown padded leather upholstery
    color_seat_frame = Gf.Vec3f(0.12, 0.12, 0.12)    # Black steel seat support structure
    color_tire = Gf.Vec3f(0.18, 0.18, 0.18)          # Dark rubber tire
    color_rim = Gf.Vec3f(0.92, 0.92, 0.90)           # Off-white wheel rim
    color_hub = Gf.Vec3f(0.25, 0.25, 0.25)           # Center axle hub cap
    color_spring = Gf.Vec3f(0.95, 0.45, 0.05)        # Vibrant orange coil spring
    color_shock = Gf.Vec3f(0.30, 0.30, 0.35)         # Metallic shock absorber body
    color_bumper = Gf.Vec3f(0.78, 0.78, 0.80)        # Extruded aluminum silver bar
    color_sw = Gf.Vec3f(0.15, 0.15, 0.15)            # Black steering wheel rim
    color_housing = Gf.Vec3f(0.08, 0.08, 0.08)       # Electronics enclosure box
    color_camera = Gf.Vec3f(0.12, 0.12, 0.12)        # Black camera body
    color_lens = Gf.Vec3f(0.05, 0.05, 0.08)          # Camera optical lens glass
    color_light_red = Gf.Vec3f(0.85, 0.10, 0.10)     # Rear tail light red
    color_light_white = Gf.Vec3f(0.90, 0.90, 0.85)   # Reverse light white
    color_cushion_top = Gf.Vec3f(0.68, 0.48, 0.26)   # Lighter tan comfort pads
    color_armrest = Gf.Vec3f(0.72, 0.72, 0.74)       # Light grey armrest pads (reference)
    color_motor = Gf.Vec3f(0.62, 0.63, 0.66)         # Silver drive motor drum
    color_battery = Gf.Vec3f(0.70, 0.70, 0.72)       # Silver battery box under seat
    color_estop_red = Gf.Vec3f(0.80, 0.08, 0.08)     # E-stop mushroom (reference)
    color_estop_yellow = Gf.Vec3f(0.95, 0.80, 0.10)  # E-stop base ring
    color_mast = Gf.Vec3f(0.55, 0.55, 0.58)          # Grey antenna mast
    color_amber = Gf.Vec3f(0.95, 0.55, 0.05)         # Front amber indicators
    
    # Helper functions for visual prim creation
    def set_color(prim, vec3_color):
        if prim and prim.IsValid():
            disp_attr = prim.GetAttribute("primvars:displayColor")
            if not disp_attr:
                disp_attr = prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
            disp_attr.Set([vec3_color])
            
    def create_cube(path, translation, scale, color=None):
        prim = safe_define_prim(path, "Cube")
        cube_geom = UsdGeom.Cube(prim)
        cube_geom.CreateSizeAttr().Set(1.0)  # Size 1.0 so scale matches exact meter dimensions
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
        xform.AddScaleOp().Set(Gf.Vec3d(*scale))
        if color:
            set_color(prim.GetPrim(), color)
        return prim
    
    def create_cylinder(path, translation, radius, height, orientation=None, color=None):
        prim = safe_define_prim(path, "Cylinder")
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
        if orientation:
            xform.AddRotateXYZOp().Set(Gf.Vec3d(*orientation))
        prim.GetAttribute("radius").Set(radius)
        prim.GetAttribute("height").Set(height)
        if color:
            set_color(prim.GetPrim(), color)
        return prim

    # Visual Group Root - under Chassis so it follows physics (was under Vehicle and stayed behind)
    visual_group_path = f"{vehicle_path}/Chassis/Visuals"
    safe_define_prim(visual_group_path, "Xform")
    
    # ============================================================
    # 1. INDUSTRIAL OPEN TUBULAR CHASSIS & FLOOR PLATFORM
    # ============================================================
    tube_r = 0.035  # 7cm diameter frame tubes
    frame_z = -0.12  # Frame rails near chassis bottom
    
    # Left and Right main longitudinal frame rails
    create_cylinder(f"{visual_group_path}/Frame_Rail_L", (-0.05, width/2.0 - 0.08, frame_z), tube_r, length * 0.90, (0, 90, 0), color_chassis)
    create_cylinder(f"{visual_group_path}/Frame_Rail_R", (-0.05, -width/2.0 + 0.08, frame_z), tube_r, length * 0.90, (0, 90, 0), color_chassis)
    
    # Cross members (Front, Middle, Rear)
    create_cylinder(f"{visual_group_path}/Frame_Cross_Front", (wheelbase/2.0, 0, frame_z), tube_r, width - 0.16, (90, 0, 0), color_chassis)
    create_cylinder(f"{visual_group_path}/Frame_Cross_Mid", (0.0, 0, frame_z), tube_r, width - 0.16, (90, 0, 0), color_chassis)
    create_cylinder(f"{visual_group_path}/Frame_Cross_Rear", (-wheelbase/2.0, 0, frame_z), tube_r, width - 0.16, (90, 0, 0), color_chassis)
    
    # Black Diamond-Plate Floor Deck
    create_cube(f"{visual_group_path}/Floor_Platform", (-0.05, 0.0, -0.08), (length * 0.85, width * 0.85, 0.02), color_floor)

    # ============================================================
    # 2. FRONT ALUMINUM BUMPER BAR
    # ============================================================
    bumper_x = length / 2.0 + 0.02
    create_cube(f"{visual_group_path}/Bumper_Front_Bar", (bumper_x, 0.0, -0.06), (0.08, width * 0.92, 0.06), color_bumper)
    create_cube(f"{visual_group_path}/Bumper_Mount_L", (bumper_x - 0.08, width/4.0, -0.06), (0.12, 0.04, 0.05), color_chassis)
    create_cube(f"{visual_group_path}/Bumper_Mount_R", (bumper_x - 0.08, -width/4.0, -0.06), (0.12, 0.04, 0.05), color_chassis)

    # ============================================================
    # 3. SEAT & BACKREST (Tan/Brown Padded Leather Upholstery)
    # ============================================================
    # Full-width bench per reference photos (long tan bench, tall backrest,
    # LIGHT-GREY armrest pads — not tan). Moved slightly rearward for footwell.
    seat_x = -0.25
    seat_z = 0.12

    # Seat Bottom Cushion (Tan Leather) + top comfort pad (lighter)
    create_cube(f"{visual_group_path}/Seat_Frame_Base", (seat_x, 0.0, 0.04), (0.80, 0.88, 0.04), color_seat_frame)
    create_cube(f"{visual_group_path}/Seat_Cushion", (seat_x, 0.0, seat_z), (0.80, 0.84, 0.12), color_seat)
    create_cube(f"{visual_group_path}/Seat_Cushion_Top", (seat_x, 0.0, seat_z + 0.075), (0.72, 0.76, 0.035), color_cushion_top)

    # Seat Backrest (tall) + front comfort pad
    backrest_x = seat_x - 0.30
    backrest_z = 0.44
    create_cube(f"{visual_group_path}/Backrest_Post_L", (backrest_x, 0.36, 0.26), (0.04, 0.04, 0.38), color_seat_frame)
    create_cube(f"{visual_group_path}/Backrest_Post_R", (backrest_x, -0.36, 0.26), (0.04, 0.04, 0.38), color_seat_frame)
    create_cube(f"{visual_group_path}/Seat_Backrest", (backrest_x, 0.0, backrest_z), (0.10, 0.84, 0.50), color_seat)
    create_cube(f"{visual_group_path}/Backrest_Pad", (backrest_x + 0.055, 0.0, backrest_z), (0.035, 0.76, 0.42), color_cushion_top)

    # Armrests: black posts + LIGHT GREY pads (reference has grey/white pads)
    for side_name, y_dir in [("L", 1), ("R", -1)]:
        arm_y = y_dir * 0.46
        create_cube(f"{visual_group_path}/Armrest_Post_{side_name}", (seat_x, arm_y, 0.24), (0.04, 0.04, 0.20), color_seat_frame)
        create_cube(f"{visual_group_path}/Armrest_Pad_{side_name}", (seat_x + 0.06, arm_y, 0.35), (0.42, 0.07, 0.045), color_armrest)

    # ============================================================
    # 4. STEERING ASSEMBLY & FRONT COWL (big black dash per reference)
    # ============================================================
    # Wide dash cowl spanning the front (reference has a large black cowl with
    # e-stop button, not a small console). Foot sits on deck top (local -0.07).
    create_cube(f"{visual_group_path}/Front_Cowl", (0.32, 0.0, 0.16), (0.44, 0.92, 0.36), color_housing)
    # E-stop: yellow base + red mushroom on cowl top (as in photos)
    create_cylinder(f"{visual_group_path}/EStop_Base", (0.30, 0.12, 0.35), 0.035, 0.02, None, color_estop_yellow)
    create_cylinder(f"{visual_group_path}/EStop_Red", (0.30, 0.12, 0.37), 0.025, 0.03, None, color_estop_red)
    # Indicator stalk on column
    create_cube(f"{visual_group_path}/Steering_Stalk", (0.16, 0.10, 0.52), (0.10, 0.025, 0.025), color_chassis)

    # Angled Steering Column: dash top (0.24, 0, 0.20) up-back to wheel hub.
    # Tilt is about Y (fore-aft pitch). NOTE: (-25, 0, 0) would lean it sideways.
    create_cylinder(f"{visual_group_path}/Steering_Column", (0.18, 0.0, 0.41), 0.03, 0.44, (0, -16, 0), color_chassis)

    # Steering Wheel assembly: parent Xform tilted (0,-65,0) so the wheel plane
    # faces the driver (-X, tipped up). Children are built in wheel-local frame
    # (plane = XY, axle = +Z): outer rim ring, recessed spoke panel, 3 spokes, hub.
    sw_x = 0.12
    sw_z = 0.62
    wheel_path = f"{visual_group_path}/Steering_Wheel"
    wheel_xform = safe_define_prim(wheel_path, "Xform")
    UsdGeom.Xformable(wheel_xform).AddTranslateOp().Set(Gf.Vec3d(sw_x, 0.0, sw_z))
    UsdGeom.Xformable(wheel_xform).AddRotateXYZOp().Set(Gf.Vec3d(0, -65, 0))
    create_cylinder(f"{wheel_path}/Rim", (0.0, 0.0, 0.0), 0.18, 0.035, None, color_sw)
    create_cube(f"{wheel_path}/Spoke_Panel", (0.0, 0.0, 0.0), (0.27, 0.27, 0.038), color_chassis)
    for s in range(3):
        # Explicit op order translate -> rotate -> scale (T*R*S) so the spoke
        # is sized first, then rotated in-plane. (Appending a rotate to the
        # create_cube helper would give T*S*R and silently drop the rotation.)
        sp = safe_define_prim(f"{wheel_path}/Spoke_{s}", "Cube")
        UsdGeom.Cube(sp).CreateSizeAttr().Set(1.0)
        sp_xf = UsdGeom.Xformable(sp)
        sp_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        sp_xf.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, s * 60.0))
        sp_xf.AddScaleOp().Set(Gf.Vec3d(0.30, 0.045, 0.040))
        set_color(sp.GetPrim(), color_sw)
    create_cylinder(f"{wheel_path}/Hub", (0.0, 0.0, 0.0), 0.05, 0.07, None, color_chassis)

    # ============================================================
    # 5. FRONT SUSPENSION & VIBRANT ORANGE COIL SPRINGS
    # ============================================================
    front_x = wheelbase / 2.0
    for side_name, y_dir in [("FL", 1), ("FR", -1)]:
        susp_y = y_dir * (front_track / 2.0 - 0.12)
        
        # Wishbones
        create_cylinder(f"{visual_group_path}/Susp_Wishbone_Lower_{side_name}", (front_x, susp_y, -0.08), 0.018, 0.22, (0, 90, 0), color_chassis)
        create_cylinder(f"{visual_group_path}/Susp_Wishbone_Upper_{side_name}", (front_x, susp_y, 0.05), 0.018, 0.22, (0, 90, 0), color_chassis)
        
        # Shock Absorber & VIBRANT ORANGE COIL SPRINGS
        shock_y = y_dir * (front_track / 2.0 - 0.18)
        create_cylinder(f"{visual_group_path}/Susp_Shock_Body_{side_name}", (front_x, shock_y, 0.0), 0.028, 0.28, (0, y_dir * 15, 0), color_shock)
        create_cylinder(f"{visual_group_path}/Susp_Orange_Spring_{side_name}", (front_x, shock_y, 0.0), 0.045, 0.22, (0, y_dir * 15, 0), color_spring)

    # ============================================================
    # 6. REAR BATTERY & ELECTRONICS HOUSING + REAR LIGHTS
    # ============================================================
    rear_box_x = -0.55
    rear_box_z = 0.12
    create_cube(f"{visual_group_path}/Rear_Electronics_Box", (rear_box_x, 0.0, rear_box_z), (0.45, width * 0.72, 0.30), color_housing)
    create_cube(f"{visual_group_path}/Rear_Box_Lid", (rear_box_x, 0.0, rear_box_z + 0.16), (0.47, width * 0.74, 0.03), color_chassis)
    
    # Rear Lights
    rear_light_x = rear_box_x - 0.23
    for side_name, y_dir in [("L", 1), ("R", -1)]:
        light_y = y_dir * (width * 0.30)
        create_cube(f"{visual_group_path}/Rear_Light_Red_{side_name}", (rear_light_x, light_y, 0.15), (0.03, 0.10, 0.08), color_light_red)
        create_cube(f"{visual_group_path}/Rear_Light_White_{side_name}", (rear_light_x, light_y + y_dir * 0.06, 0.15), (0.03, 0.04, 0.08), color_light_white)

    # Silver battery box slung under the bench (reference shows it between posts)
    create_cube(f"{visual_group_path}/Battery_Box", (-0.25, 0.0, -0.02), (0.40, 0.50, 0.20), color_battery)
    # Rear fender flares over rear wheels (black, as in photos)
    for side_name, y_dir in [("L", 1), ("R", -1)]:
        fy = y_dir * (rear_track / 2.0)
        create_cube(f"{visual_group_path}/Fender_Top_{side_name}", (-0.55, fy, 0.19), (0.62, 0.30, 0.03), color_chassis)
        # angled skirts via explicit T*R*S (rotate about Y for fore-aft tilt)
        for sfx, sang in [("F", -28.0), ("R", 28.0)]:
            fp = safe_define_prim(f"{visual_group_path}/Fender_Skirt_{side_name}_{sfx}", "Cube")
            UsdGeom.Cube(fp).CreateSizeAttr().Set(1.0)
            fx = UsdGeom.Xformable(fp)
            fx.AddTranslateOp().Set(Gf.Vec3d(-0.55 + (0.28 if sfx == "F" else -0.28), fy, 0.10))
            fx.AddRotateXYZOp().Set(Gf.Vec3d(0, sang, 0))
            fx.AddScaleOp().Set(Gf.Vec3d(0.03, 0.30, 0.22))
            set_color(fp.GetPrim(), color_chassis)
    # Deck side rails
    create_cube(f"{visual_group_path}/Deck_Rail_L", (-0.05, 0.53, -0.06), (1.50, 0.04, 0.04), color_chassis)
    create_cube(f"{visual_group_path}/Deck_Rail_R", (-0.05, -0.53, -0.06), (1.50, 0.04, 0.04), color_chassis)
    # Silver drive motor drum across the front + dark gearbox (reference front)
    create_cylinder(f"{visual_group_path}/Front_Motor", (0.62, 0.0, -0.02), 0.075, 0.55, (0, 90, 0), color_motor)
    create_cube(f"{visual_group_path}/Front_Gearbox", (0.62, 0.0, -0.02), (0.18, 0.22, 0.20), color_chassis)
    # Amber front indicators on bumper ends
    create_cube(f"{visual_group_path}/Amber_L", (0.92, 0.50, -0.02), (0.05, 0.09, 0.06), color_amber)
    create_cube(f"{visual_group_path}/Amber_R", (0.92, -0.50, -0.02), (0.05, 0.09, 0.06), color_amber)
    # Mast removed per user request — no antenna needed for vehicle foundation

    # ============================================================
    # 7. REALISTIC WHEEL VISUALS (Smooth Pneumatic Tires + White Rims + Hubs)
    # ============================================================
    # Realistic utility-vehicle wheels: smooth black pneumatic tire, inset
    # off-white rim, central hub.  No hexagonal/polygonal treads — a clean
    # torus-like tire that sits correctly on the floor (center at 0.25 m,
    # bottom at 0.0) and rolls visibly when the physics wheel rotates.
    # Built as children of the PHYSICS wheel prims at (0,0,0) so they
    # share the wheel centre and follow the joint rotation exactly.
    import math as _math

    def _make_visible(prim):
        v = prim.GetAttribute("visibility")
        if not v or not v.IsValid():
            v = prim.GetPrim().CreateAttribute("visibility", Sdf.ValueTypeNames.Token)
        v.Set("visible")
        return prim

    def _visual_cylinder(path, translation, radius, height, orientation, color):
        prim = create_cylinder(path, translation, radius, height, orientation, color)
        return _make_visible(prim)

    for name in ["FL", "FR", "RL", "RR"]:
        phys_wheel = f"{vehicle_path}/Wheel_{name}"
        # Smooth black pneumatic tire — torus-like, no blocky treads
        _visual_cylinder(f"{phys_wheel}/Tire", (0, 0, 0), wheel_radius, wheel_width, (90, 0, 0), color_tire)
        # Inset off-white rim (pokes 5 mm each side)
        _visual_cylinder(f"{phys_wheel}/Rim", (0, 0, 0), 0.135, wheel_width + 0.010, (90, 0, 0), color_rim)
        # Black hub + centre cap — ensures hub is visible even when tire is dark
        _visual_cylinder(f"{phys_wheel}/Hub", (0, 0, 0), 0.050, wheel_width + 0.022, (90, 0, 0), color_hub)
        _visual_cylinder(f"{phys_wheel}/HubCap", (0, 0, 0), 0.022, wheel_width + 0.026, (90, 0, 0), color_chassis)
        # 5 lug bolts on 85 mm circle — subtle detail, not hexagonal
        for b in range(5):
            bang = _math.radians(b * 72.0)
            _visual_cylinder(f"{phys_wheel}/Lug_{b}",
                             (0.085 * _math.cos(bang), 0.0, 0.085 * _math.sin(bang)),
                             0.011, wheel_width + 0.016, (90, 0, 0), color_bumper)
        # Ensure any stale Tread_* prims from previous runs are removed/hidden
        # (they were 14 cubes causing hexagonal appearance)
        for k in range(14):
            stale = stage.GetPrimAtPath(f"{phys_wheel}/Tread_{k:02d}")
            if stale and stale.IsValid():
                UsdGeom.Imageable(stale).MakeInvisible()

    # ============================================================
    # 8. PHYSICAL RGB-D CAMERA SENSOR & RISER POST
    # ============================================================
    cam_cfg_x = vehicle_cfg.get("camera_mount_x", 0.9)
    cam_cfg_z = vehicle_cfg.get("camera_mount_z", 1.2)
    cam_pitch_deg = vehicle_cfg.get("camera_pitch_deg", -8.5)
    
    cam_local_z = cam_cfg_z - vehicle_root_z  # 1.20 - 0.35 = 0.85m
    
    # Riser Post: spans from floor deck (local z=-0.07) up to camera (local z=cam_local_z)
    # so its foot is embedded in the deck instead of floating 7cm above it.
    post_height = cam_local_z + 0.07
    post_center_z = (cam_local_z - 0.07) / 2.0
    create_cylinder(f"{visual_group_path}/Camera_Riser_Post", (cam_cfg_x, 0.0, post_center_z), 0.025, post_height, (0, 0, 0), color_chassis)
    
    # Camera Body & Dual Lens (visual only - sensor created by rgbd_camera.py)
    sensor_root_path = f"{vehicle_path}/Chassis/Sensors/RGBD_Camera"
    # Sensors Xform and camera already created by rgbd_camera.py - use safe_define_prim
    safe_define_prim(f"{vehicle_path}/Chassis/Sensors", "Xform")
    sensor_xform = safe_define_prim(sensor_root_path, "Xform")
    
    # Add visual camera housing and lenses to existing sensor xform
    create_cube(f"{sensor_root_path}/Camera_Body", (0.0, 0.0, 0.0), (0.08, 0.22, 0.08), color_camera)
    create_cylinder(f"{sensor_root_path}/Lens_RGB", (0.042, 0.06, 0.0), 0.022, 0.01, (0, 90, 0), color_lens)
    create_cylinder(f"{sensor_root_path}/Lens_Depth", (0.042, -0.06, 0.0), 0.022, 0.01, (0, 90, 0), color_lens)
    
    # NOTE: Camera transform (position + pitch) already set by rgbd_camera.py
    # Do NOT apply transform here to avoid conflicts
    
    print(f"[INFO] Visual digital twin created at {visual_group_path}")


if __name__ == "__main__":
    print("High-fidelity vehicle visual module loaded successfully")