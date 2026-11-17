"""
Modular Laboratory Environment Assets
Provides reusable asset functions for creating lab furniture and architectural elements.
All assets have visual geometry and simplified collision geometry.
"""

import numpy as np


def _get_define_prim():
    """Helper to get define_prim with proper import."""
    from isaacsim.core.utils.prims import define_prim
    return define_prim


def create_lab_table(stage, path, translation, scale=(1.5, 0.8, 0.75), color=None):
    """
    Create a lab table with top and 4 legs.
    
    Args:
        stage: USD stage
        path: Prim path
        translation: (x, y, z) position of table center
        scale: (length, width, height) of table top
        color: Optional [r, g, b] color
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.utils.prims import define_prim
    
    length, width, height = scale
    x, y, z = translation
    
    # Table top
    top_path = f"{path}/Top"
    top_prim = define_prim(top_path, "Cube")
    UsdGeom.Cube(top_prim).CreateSizeAttr().Set(1.0)
    xformable = UsdGeom.Xformable(top_prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z + height - 0.04))  # Top at height
    xformable.AddScaleOp().Set(Gf.Vec3d(length, width, 0.08))
    
    # 4 Legs
    leg_radius = 0.04
    leg_height = height - 0.04
    half_l = length / 2.0 - 0.1
    half_w = width / 2.0 - 0.1
    leg_positions = [
        (x + half_l, y + half_w),
        (x + half_l, y - half_w),
        (x - half_l, y + half_w),
        (x - half_l, y - half_w),
    ]
    
    for i, (lx, ly) in enumerate(leg_positions):
        leg_path = f"{path}/Leg_{i}"
        leg_prim = define_prim(f"{path}/Leg_{i}", "Cylinder")
        leg_xform = UsdGeom.Xformable(leg_prim)
        leg_xform.AddTranslateOp().Set(Gf.Vec3d(lx, ly, z + leg_height / 2.0))
        # USD Cylinder default axis is Z (vertical); no rotation needed for vertical table legs
        leg_prim.GetAttribute("radius").Set(leg_radius)
        leg_prim.GetAttribute("height").Set(leg_height)
        
        # Collision on legs
        UsdPhysics.CollisionAPI.Apply(leg_prim)
    
    # Collision on top
    UsdPhysics.CollisionAPI.Apply(top_prim.GetPrim())
    top_rigid = UsdPhysics.RigidBodyAPI.Apply(top_prim.GetPrim())
    top_rigid.CreateKinematicEnabledAttr().Set(True)
    
    # Color
    if color:
        from pxr import Gf as Gf2
        color_gf = Gf2.Vec3f(color[0], color[1], color[2])
        for prim_name in [f"{path}/Top"]:
            prim = stage.GetPrimAtPath(f"{path}/Top")
            if prim.IsValid():
                disp_attr = prim.GetAttribute("primvars:displayColor")
                if not disp_attr:
                    disp_attr = prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
                disp_attr.Set([Gf.Vec3f(color[0], color[1], color[2])])
    
    return define_prim(path, "Xform")


def create_office_chair(stage, path, translation, color=None):
    """
    Create an office chair with seat, backrest, and base.
    
    Args:
        stage: USD stage
        path: Prim path
        translation: (x, y, z) position of chair center
        color: Optional [r, g, b] color
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.utils.prims import define_prim
    import omni.usd
    
    x, y, z = translation
    seat_height = 0.45
    seat_radius = 0.25
    backrest_height = 0.5
    base_radius = 0.35
    base_height = 0.05
    
    # Seat
    seat_path = f"{path}/Seat"
    seat_prim = define_prim(f"{path}/Seat", "Cylinder")
    seat_xform = UsdGeom.Xformable(seat_prim)
    seat_xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z + seat_height))
    seat_prim.GetAttribute("radius").Set(0.25)
    seat_prim.GetAttribute("height").Set(0.08)
    UsdPhysics.CollisionAPI.Apply(seat_prim.GetPrim())
    
    # Backrest
    backrest_path = f"{path}/Backrest"
    backrest_prim = define_prim(f"{path}/Backrest", "Cube")
    backrest_xform = UsdGeom.Xformable(backrest_prim)
    backrest_xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z + seat_height + backrest_height/2.0))
    backrest_xform.AddScaleOp().Set(Gf.Vec3d(0.5, 0.1, 0.5))
    UsdPhysics.CollisionAPI.Apply(backrest_prim.GetPrim())
    
    # Base (5-star base)
    base_path = f"{path}/Base"
    base_prim = define_prim(f"{path}/Base", "Cylinder")
    base_xform = UsdGeom.Xformable(base_prim)
    base_xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z + base_height/2.0))
    base_prim.GetAttribute("radius").Set(0.35)
    base_prim.GetAttribute("height").Set(0.05)
    UsdPhysics.CollisionAPI.Apply(base_prim.GetPrim())
    
    # Central column
    col_prim = define_prim(f"{path}/Column", "Cylinder")
    col_xform = UsdGeom.Xformable(col_prim)
    col_xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z + 0.2))
    col_prim.GetAttribute("radius").Set(0.06)
    col_prim.GetAttribute("height").Set(0.4)
    UsdPhysics.CollisionAPI.Apply(col_prim.GetPrim())
    
    # All kinematic
    for prim_path in [f"{path}/Seat", f"{path}/Base", f"{path}/Column"]:
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            prim.GetAttribute("primvars:displayColor").Set([Gf.Vec3f(0.2, 0.2, 0.2)])
    
    return define_prim(path, "Xform")


def create_slatted_column(stage, path, translation, height=3.0, color=None):
    """
    Create a slatted wooden column.
    
    Args:
        stage: USD stage
        path: Prim path
        translation: (x, y, z) position of column base
        height: Column height
        color: Optional [r, g, b] color
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.utils.prims import define_prim
    import omni.usd
    
    x, y, z = translation
    column_width = 0.6
    column_depth = 0.6
    slat_width = 0.04
    slat_gap = 0.02
    
    # Create root Xform for the column
    column_xform = define_prim(path, "Xform")
    xformable = UsdGeom.Xformable(column_xform)
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    
    # Main column box (collision) as child of the Xform
    col_prim = define_prim(f"{path}/Column_Collision", "Cube")
    col_xform = UsdGeom.Xformable(col_prim)
    col_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height/2.0))
    col_xform.AddScaleOp().Set(Gf.Vec3d(column_width, column_depth, height))
    UsdPhysics.CollisionAPI.Apply(col_prim.GetPrim())
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(col_prim.GetPrim())
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    
    # Wood color
    from pxr import Gf, Sdf
    disp_attr = col_prim.GetPrim().GetAttribute("primvars:displayColor")
    if not disp_attr:
        disp_attr = col_prim.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    disp_attr.Set([Gf.Vec3f(0.5, 0.35, 0.2)])  # Wood color
    
    # Visual slats (8 slats per face, 4 faces = 32 slats)
    slat_width = 0.04
    slat_depth = 0.62  # Slightly larger than column
    slat_height = height / 8.0
    slat_gap = height / 8.0 - slat_height
    
    num_slats = 8
    for face in range(4):  # 4 faces
        for i in range(num_slats):
            slat_z = z + i * (slat_height + slat_gap) + slat_height/2.0
            
            if face == 0:  # +X face
                slat_x = x + column_width/2.0 + 0.01
                slat_y = y
                slat_rot = (0, 0, 90)
            elif face == 1:  # -X face
                slat_x = x - column_width/2.0 - 0.01
                slat_y = y
                slat_rot = (0, 0, 90)
            elif face == 2:  # +Y face
                slat_x = x
                slat_y = y + column_depth/2.0 + 0.01
                slat_rot = (0, 0, 0)
            else:  # -Y face
                slat_x = x
                slat_y = y - column_depth/2.0 - 0.01
                slat_rot = (0, 0, 0)
            
            slat_path = f"{path}/Slat_{face}_{i}"
            slat_prim = define_prim(slat_path, "Cube")
            slat_xform = UsdGeom.Xformable(slat_prim)
            slat_xform.AddTranslateOp().Set(Gf.Vec3d(slat_x, slat_y, z + slat_height/2.0 + i * (slat_height + slat_gap)))
            slat_xform.AddScaleOp().Set(Gf.Vec3d(0.02, column_depth + 0.04, height/8.0))
            
            # Wood color
            from pxr import Gf, Sdf
            disp_attr = col_prim.GetPrim().GetAttribute("primvars:displayColor")
    
    # Make kinematic
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(col_prim.GetPrim())
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    
    return column_xform


def create_glass_partition(stage, path, translation, scale=(0.1, 2.0, 2.5), color=None):
    """
    Create a glass partition wall.
    
    Args:
        stage: USD stage
        path: Prim path
        translation: (x, y, z) position of partition center
        scale: (thickness, width, height)
        color: Optional [r, g, b] color (tint for glass)
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.utils.prims import define_prim
    import omni.usd
    
    thickness, width, height = scale
    x, y, z = translation
    
    partition_prim = define_prim(path, "Cube")
    xformable = UsdGeom.Xformable(partition_prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z + height/2.0))
    xformable.AddScaleOp().Set(Gf.Vec3d(thickness, width, height))
    
    UsdPhysics.CollisionAPI.Apply(partition_prim.GetPrim())
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(partition_prim.GetPrim())
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    
    # Glass color (slightly transparent blue-green)
    from pxr import Gf, Sdf
    glass_color = color if color else [0.7, 0.9, 1.0]
    disp_attr = partition_prim.GetAttribute("primvars:displayColor")
    if not disp_attr:
        disp_attr = partition_prim.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    disp_attr.Set([Gf.Vec3f(glass_color[0], glass_color[1], glass_color[2])])
    
    # Glass material hint (using displayColor alpha)
    # Note: For true transparency, would need UsdShade material
    
    return define_prim(path, "Xform")


def create_wall(stage, path, translation, scale, color=None):
    """
    Create a solid wall.
    
    Args:
        stage: USD stage
        path: Prim path
        translation: (x, y, z) position of wall center
        scale: (thickness, width, height)
        color: Optional [r, g, b] color
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.utils.prims import define_prim
    import omni.usd
    
    thickness, width, height = scale
    x, y, z = translation
    
    wall_prim = define_prim(path, "Cube")
    xformable = UsdGeom.Xformable(wall_prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z + height/2.0))
    xformable.AddScaleOp().Set(Gf.Vec3d(thickness, width, height))
    
    UsdPhysics.CollisionAPI.Apply(wall_prim.GetPrim())
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(wall_prim.GetPrim())
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    
    if color:
        from pxr import Gf, Sdf
        disp_attr = wall_prim.GetPrim().GetAttribute("primvars:displayColor")
        if not disp_attr:
            disp_attr = wall_prim.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
        disp_attr.Set([Gf.Vec3f(color[0], color[1], color[2])])
    else:
        disp_attr = wall_prim.GetPrim().GetAttribute("primvars:displayColor")
        if not disp_attr:
            disp_attr = wall_prim.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
        disp_attr.Set([Gf.Vec3f(0.7, 0.7, 0.7)])  # Light gray
    
    return wall_prim


def create_floor_tiles(stage, path, tile_size=0.3, grid_size=(50, 50), color1=None, color2=None):
    """
    Create a tiled floor using instanced tiles.
    
    Args:
        stage: USD stage
        path: Prim path
        tile_size: Size of each tile in meters
        grid_size: (tiles_x, tiles_y)
        color1: First tile color
        color2: Second tile color (checkerboard)
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.utils.prims import define_prim
    import omni.usd
    
    tiles_x, tiles_y = grid_size
    color1 = color1 or [0.55, 0.55, 0.55]
    color2 = color2 or [0.45, 0.45, 0.45]
    
    floor_xform = define_prim(path, "Xform")
    
    # Create prototype tile
    tile_path = f"{path}/Tile_Proto"
    tile_prim = define_prim(tile_path, "Cube")
    xformable = UsdGeom.Xformable(tile_prim)
    xformable.AddScaleOp().Set(Gf.Vec3d(tile_size, tile_size, 0.02))
    UsdPhysics.CollisionAPI.Apply(tile_prim.GetPrim())
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(tile_prim.GetPrim())
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    
    # For simplicity, create a single large floor with collision
    # Visual tiles would use instancing in production
    floor_prim = define_prim(f"{path}/Floor_Collision", "Cube")
    xformable = UsdGeom.Xformable(floor_prim)
    total_x = tile_size * grid_size[0]
    total_y = tile_size * grid_size[1]
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))
    xformable.AddScaleOp().Set(Gf.Vec3d(total_x, total_y, 0.02))
    UsdPhysics.CollisionAPI.Apply(floor_prim.GetPrim())
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(floor_prim.GetPrim())
    rigid_api.CreateKinematicEnabledAttr().Set(True)
    
    # Visual floor (single large tile with tile color)
    floor_vis = define_prim(f"{path}/Floor_Visual", "Cube")
    xformable = UsdGeom.Xformable(floor_vis)
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xformable.AddScaleOp().Set(Gf.Vec3d(total_x, total_y, 0.02))
    from pxr import Gf, Sdf
    disp_attr = floor_vis.GetPrim().GetAttribute("primvars:displayColor")
    if not disp_attr:
        disp_attr = floor_vis.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    disp_attr.Set([Gf.Vec3f(0.5, 0.5, 0.5)])  # Base gray
    
    # Note: For true checkerboard, would use instancing or texture
    # This is a simplified version
    
    return floor_xform


def create_ceiling_lights(stage, path, grid_spacing=2.0, room_size=(20, 20), light_color=None, intensity=1000):
    """
    Create a grid of ceiling lights (LED panels).
    
    Args:
        stage: USD stage
        path: Prim path
        grid_spacing: Distance between lights in meters
        room_size: (width, depth) of room
        light_color: [r, g, b] color
        intensity: Light intensity
    """
    from pxr import Usd, UsdGeom, UsdLux, Gf
    import omni.usd
    from isaacsim.core.utils.prims import define_prim
    
    light_color = light_color or [1.0, 0.95, 0.9]
    width, depth = room_size
    
    lights_xform = define_prim(path, "Xform")
    
    num_x = int(width / grid_spacing) + 1
    num_y = int(depth / grid_spacing) + 1
    
    for i in range(num_x):
        for j in range(num_y):
            x = -width/2.0 + i * grid_spacing
            y = -depth/2.0 + j * grid_spacing
            z = 2.8  # Ceiling height
            
            light_path = f"{path}/Light_{i}_{j}"
            rect_light = UsdLux.RectLight.Define(stage, light_path)
            rect_light.CreateWidthAttr().Set(0.6)
            rect_light.CreateHeightAttr().Set(0.6)
            rect_light.CreateIntensityAttr().Set(intensity)
            rect_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.9))
            
            light_xform = UsdGeom.Xformable(rect_light.GetPrim())
            light_xform.AddTranslateOp().Set(Gf.Vec3d(-width/2.0 + i * grid_spacing, -depth/2.0 + j * grid_spacing, z))
            light_xform.AddRotateXYZOp().Set(Gf.Vec3d(-90, 0, 0))
    
    return define_prim(path, "Xform")


def create_ceiling_fan(stage, path, translation, blade_radius=0.6, color=None):
    """
    Create a ceiling fan.
    
    Args:
        stage: USD stage
        path: Prim path
        translation: (x, y, z) position of fan center
        blade_radius: Fan blade radius
        color: Optional [r, g, b] color
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
    from isaacsim.core.utils.prims import define_prim
    import omni.usd
    
    x, y, z = translation
    color = color or [0.3, 0.3, 0.3]
    
    fan_xform = define_prim(path, "Xform")
    fan_xform.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, z))
    
    # Central hub
    hub_prim = define_prim(f"{path}/Hub", "Cylinder")
    hub_xform = UsdGeom.Xformable(hub_prim)
    hub_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    hub_xform.AddRotateXYZOp().Set(Gf.Vec3d(90, 0, 0))
    hub_prim.GetAttribute("radius").Set(0.15)
    hub_prim.GetAttribute("height").Set(0.1)
    from pxr import Gf, Sdf
    disp_attr = hub_prim.GetPrim().GetAttribute("primvars:displayColor")
    if not disp_attr:
        disp_attr = hub_prim.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    disp_attr.Set([Gf.Vec3f(0.2, 0.2, 0.2)])
    
    # 4 Blades
    blade_length = 0.6
    blade_width = 0.12
    blade_thickness = 0.02
    
    for i in range(4):
        blade_prim = define_prim(f"{path}/Blade_{i}", "Cube")
        blade_xform = UsdGeom.Xformable(blade_prim)
        blade_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        blade_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, i * 90))
        blade_xform.AddScaleOp().Set(Gf.Vec3d(blade_length, blade_width, blade_thickness))
        disp_attr = blade_prim.GetPrim().GetAttribute("primvars:displayColor")
        if not disp_attr:
            disp_attr = blade_prim.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
        disp_attr.Set([Gf.Vec3f(0.2, 0.2, 0.2)])
    
    return fan_xform


# ============================================================
# Convenience Functions for Common Lab Layouts
# ============================================================

def create_standard_lab_table(stage, path, translation, color=None):
    """Create a standard lab table (1.5x0.8x0.75m)."""
    return create_lab_table(stage, path, translation, (1.5, 0.8, 0.75), color)


def create_meeting_table(stage, path, translation, color=None):
    """Create a larger meeting table (2.4x1.2x0.75m)."""
    return create_lab_table(stage, path, translation, (2.4, 1.2, 0.75), color)


def create_computer_desk(stage, path, translation, color=None):
    """Create a computer desk (1.2x0.7x0.75m)."""
    return create_lab_table(stage, path, translation, (1.2, 0.7, 0.75), color)


if __name__ == "__main__":
    print("Lab assets module loaded successfully")