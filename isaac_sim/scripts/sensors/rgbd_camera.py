"""
RGB-D Camera Sensor Module for SKATEBOARD_IRP
Creates a physical RGB-D camera sensor prim on the vehicle.
Provides both RGB and Depth image streams.
"""

import numpy as np


def create_rgbd_camera(vehicle_path: str, camera_cfg: dict, vehicle_cfg: dict,
                        vehicle_start: tuple = (-4.0, 0.0, 0.35)):
    """
    Create an RGB-D camera sensor mounted on the vehicle.
    
    Args:
        vehicle_path: USD path to vehicle root (e.g., "/World/Vehicle")
        camera_cfg: Camera configuration dictionary
        vehicle_cfg: Vehicle configuration dictionary
    
    Returns:
        Camera prim path
    """
    from pxr import Usd, UsdGeom, Gf, Sdf
    import omni.usd
    from isaacsim.core.utils.prims import define_prim
    
    stage = omni.usd.get_context().get_stage()
    
    # Camera parameters from config
    image_width = camera_cfg["image_width"]
    image_height = camera_cfg["image_height"]
    horizontal_fov = camera_cfg["horizontal_fov"]
    vertical_fov = camera_cfg["vertical_fov"]
    near_clip = camera_cfg["near_clip"]
    far_clip = camera_cfg["far_clip"]
    frame_rate = camera_cfg["frame_rate"]
    
    # Mount position (relative to vehicle base_link / rear axle center)
    mount_x = camera_cfg["position_x"]
    mount_y = camera_cfg["position_y"]
    mount_z = camera_cfg["position_z"]
    
    # Mount orientation
    roll = camera_cfg["roll"]
    pitch = camera_cfg["pitch"]
    yaw = camera_cfg["yaw"]
    
    # Vehicle dimensions for coordinate conversion
    length = vehicle_cfg["length"]
    height = vehicle_cfg["height"]
    ground_clearance = vehicle_cfg["ground_clearance"]
    
    # Camera sensor path — under Chassis (not Vehicle) so the sensor follows
    # the rigid body in dynamic mode instead of staying at the spawn pose.
    camera_path = f"{vehicle_path}/Chassis/Sensors/RGBD_Camera"
    
    # Create Xform for camera mount
    camera_xform = define_prim(camera_path, "Xform")
    xformable = UsdGeom.Xformable(camera_xform)
    
    # Mount position is vehicle-root-relative (chassis center = local origin).
    # This MUST match visual_vehicle.py (Camera_Riser_Post / Camera_Body at
    # local x = camera_mount_x): the sensor sits ON TOP of the riser post,
    # inside its housing. An earlier revision subtracted length/2 here, which
    # stranded the sensor 0.9 m behind the post — the post then photographed
    # as a black pole in the RGB stream.
    vehicle_origin_z = ground_clearance + height / 2.0

    # Camera position relative to vehicle origin
    cam_rel_x = mount_x
    cam_rel_y = mount_y
    cam_rel_z = mount_z - vehicle_origin_z
    
    # Aim the mount with the shared lookAt helper (exact facing + level roll).
    # Hand-rolled eulers put the sensor 85 deg off (staring at the floor).
    # World eye = vehicle root at world (-8,0,0.35) + mount offset, i.e.
    # (-7.1, 0, 1.2): on top of the riser post, inside the camera housing.
    # Target = eye + forward ray from config pitch (yaw=0 = forward, +X).
    from isaac_sim.scripts.utils.aim import aim_prim
    # World eye = spawn + mount offset. Height is mount_z (metres above floor).
    sx, sy = float(vehicle_start[0]), float(vehicle_start[1])
    world_eye = (sx + cam_rel_x, sy + cam_rel_y, mount_z)
    fwd = (np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch))
    world_target = (world_eye[0] + 10.0 * fwd[0],
                    world_eye[1] + 10.0 * fwd[1],
                    world_eye[2] + 10.0 * fwd[2])
    aim_prim(stage, camera_path, world_eye, world_target)
    
    # Create the camera prim as child of the mount — distinct name for dropdown
    camera_prim_path = f"{camera_path}/RGBD_Camera"
    camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
    
    # Camera intrinsic parameters
    camera_prim.CreateFocalLengthAttr().Set(24.0)  # Will be overridden by FOV
    camera_prim.CreateHorizontalApertureAttr().Set(20.955)  # Approximate for 70 deg FOV
    camera_prim.CreateVerticalApertureAttr().Set(15.29)     # Approximate for 45 deg FOV
    camera_prim.CreateClippingRangeAttr().Set(Gf.Vec2f(near_clip, far_clip))
    
    # Set focal length from horizontal FOV
    # focal_length = (sensor_width / 2) / tan(hfov / 2)
    # Using standard sensor width approximation
    sensor_width = 24.0  # mm
    focal_length = (sensor_width / 2.0) / np.tan(horizontal_fov / 2.0)
    camera_prim.CreateFocalLengthAttr().Set(focal_length)
    
    # Set horizontal aperture to match FOV
    horiz_aperture = 2.0 * focal_length * np.tan(horizontal_fov / 2.0)
    camera_prim.CreateHorizontalApertureAttr().Set(horiz_aperture)
    vert_aperture = 2.0 * focal_length * np.tan(vertical_fov / 2.0)
    camera_prim.CreateVerticalApertureAttr().Set(vert_aperture)
    
    # Camera settings for rendering
    camera_prim.CreateFStopAttr().Set(0.0)  # Infinite depth of field
    camera_prim.CreateFocusDistanceAttr().Set(1.0)
    
    # Add camera to render product for RTX rendering
    # This enables the camera to produce RGB and depth data
    
    print(f"[INFO] RGB-D Camera created at {camera_prim_path}")
    print(f"  Resolution: {image_width}x{image_height}")
    print(f"  FOV: H={np.degrees(horizontal_fov):.1f}deg, V={np.degrees(vertical_fov):.1f}deg")
    print(f"  Depth Range: {near_clip}m - {far_clip}m")
    print(f"  Mount Position (relative to base_link): ({mount_x}, {mount_y}, {mount_z})")
    print(f"  Mount Orientation: Roll={np.degrees(roll):.1f}, Pitch={np.degrees(pitch):.1f}, Yaw={np.degrees(yaw):.1f}")
    
    return camera_prim_path


def create_camera_visual(camera_path: str, camera_cfg: dict):
    """
    Visual camera housing — INTENTIONALLY A NO-OP.

    The physical housing + dual lenses are owned exclusively by
    visual_vehicle.py (Camera_Body, Lens_RGB, Lens_Depth under the same
    Sensors/RGBD_Camera Xform). An earlier version of this function created a
    second, 2x-oversized housing here (raw Cube prim without size=1.0),
    which rendered as a detached dark box floating around the camera post.
    Kept as a stub so existing callers don't break.
    """
    print(f"[INFO] Camera visuals owned by visual_vehicle.py (no duplicate housing at {camera_path})")


def apply_display_color(prim, color_rgb):
    """Apply a simple display color to a prim."""
    from pxr import Gf, Sdf
    if not prim or not prim.IsValid():
        return
    display_color_attr = prim.GetPrim().GetAttribute("primvars:displayColor")
    if not display_color_attr:
        display_color_attr = prim.GetPrim().CreateAttribute(
            "primvars:displayColor", Sdf.ValueTypeNames.Color3fArray)
    display_color_attr.Set([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])


def setup_camera_render_product(camera_prim_path: str, camera_cfg: dict):
    """
    Set up render product for the camera to enable RGB and Depth output.
    This connects the camera to the RTX rendering pipeline.
    
    Note: In Isaac Sim 5.1, this is typically done via the Synthetic Data
    extensions or by creating a RenderProduct prim.
    """
    from pxr import Usd, Sdf
    import omni.usd
    
    stage = omni.usd.get_context().get_stage()
    
    # Create a RenderProduct for this camera
    # This is the modern way to get camera data in Isaac Sim
    render_product_path = f"{camera_prim_path}/RenderProduct"
    
    # For now, we just create the camera prim properly configured.
    # The actual ROS 2 bridge will handle creating render products
    # when the camera topics are requested.
    
    print(f"[INFO] Camera render product setup prepared for {camera_prim_path}")
    print(f"  ROS 2 Bridge will create render products for topics:")
    print(f"    {camera_cfg['rgb_topic']}")
    print(f"    {camera_cfg['depth_topic']}")


if __name__ == "__main__":
    print("RGB-D Camera sensor module loaded successfully")