"""
Shared inspection-camera rig (SKATEBOARD_IRP).

Single source of truth for the 24 viewport cameras under
/World/InspectionCameras. Used by EVERY runner (fidelity inspect, Ackermann
demo, verifiers) so all setups offer the same perspectives — no more
"missing cameras" when switching scripts. Viewport-only: no render products,
no ROS output (see bridge/ros2_bridge.py for the namespace contract).
"""

CAMERAS_GROUP_PATH = "/World/InspectionCameras"


def get_camera_configs(veh_x: float = -4.0, veh_y: float = 0.0):
    """(name, position, target, focal_length, focus_distance) for 24 views."""
    return [
        ("ThreeQuarter_Front", (-5.0, -3.0, 2.0), (veh_x, veh_y, 0.5), 24.0, 4.0),
        ("Side_Profile_Left", (veh_x, -4.5, 1.2), (veh_x, veh_y, 0.5), 24.0, 4.5),
        ("Side_Profile_Right", (veh_x, 4.5, 1.2), (veh_x, veh_y, 0.5), 24.0, 4.5),
        ("Rear_View", (veh_x - 4.0, 0.0, 1.5), (veh_x, veh_y, 0.5), 24.0, 4.0),
        ("Driver_Perspective", (veh_x - 0.3, 0.0, 1.15), (veh_x + 3.0, 0.0, 1.15), 20.0, 5.0),
        ("Front_Close_Detail", (veh_x + 1.5, -1.0, 0.8), (veh_x + 1.0, 0.0, 0.3), 18.0, 2.0),
        ("Suspension_CloseUp", (veh_x + 0.5, -1.5, 0.5), (veh_x + 0.55, -1.2, 0.0), 16.0, 1.5),
        ("TopDown_Full_Lab", (0.0, 0.0, 3.4), (0.0, 0.0, 0.0), 28.0, 20.0),
        ("Corner_Back_Left", (-14.0, -11.0, 2.5), (-5.0, 0.0, 1.0), 24.0, 15.0),
        ("Corner_Back_Right", (14.0, -11.0, 2.5), (5.0, 0.0, 1.0), 24.0, 15.0),
        ("Corner_Front_Left", (-14.0, 11.0, 2.5), (-5.0, 0.0, 1.0), 24.0, 15.0),
        ("Corner_Front_Right", (14.0, 11.0, 2.5), (5.0, 0.0, 1.0), 24.0, 15.0),
        ("Front_Wall_Corridor", (14.0, 0.0, 2.5), (-5.0, 0.0, 0.5), 24.0, 18.0),
        ("Back_Wall_View", (-14.0, 0.0, 2.5), (5.0, 0.0, 0.5), 24.0, 18.0),
        ("Left_Corridor", (-5.0, -11.0, 2.0), (-5.0, 0.0, 0.5), 24.0, 15.0),
        ("Right_Corridor", (5.0, 11.0, 2.0), (5.0, 0.0, 0.5), 24.0, 15.0),
        ("Left_Office_View", (-8.0, -10.0, 2.0), (-5.0, -8.0, 1.0), 22.0, 8.0),
        ("Right_Office_View", (8.0, 10.0, 2.0), (5.0, 8.0, 1.0), 22.0, 8.0),
        ("Rear_Office_View", (-12.0, 0.0, 2.0), (-8.0, 0.0, 1.0), 22.0, 8.0),
        ("Meeting_Table_CloseUp", (0.0, 0.0, 2.0), (0.0, 0.0, 0.75), 20.0, 3.0),
        ("Low_Angle_Vehicle", (veh_x, -3.0, 0.3), (veh_x, 0.0, 0.5), 18.0, 3.5),
        ("High_Angle_Vehicle", (veh_x, 0.0, 2.5), (veh_x, 0.0, 0.5), 24.0, 5.0),
        ("Wheel_FL_CloseUp", (veh_x + 0.5, -0.5, 0.3), (veh_x + 0.55, -0.5, 0.0), 16.0, 1.5),
        ("Wheel_RR_CloseUp", (veh_x - 0.5, 0.5, 0.3), (veh_x - 0.55, 0.5, 0.0), 16.0, 1.5),
        ("Ceiling_LED_View", (0.0, 0.0, 3.3), (0.0, 0.0, 3.45), 24.0, 0.5),
    ]


def create_inspection_cameras(stage, veh_x: float = -4.0, veh_y: float = 0.0) -> int:
    """Build all 24 inspection cameras with exact lookAt aim. Returns count."""
    from pxr import UsdGeom
    from isaac_sim.scripts.utils.aim import aim_prim
    print("[INFO] Setting up Comprehensive Inspection Cameras...")
    stage.DefinePrim(CAMERAS_GROUP_PATH, "Xform")
    for name, pos, target, f_len, f_dist in get_camera_configs(veh_x, veh_y):
        cam_prim_path = f"{CAMERAS_GROUP_PATH}/{name}"
        cam_prim = stage.DefinePrim(cam_prim_path, "Camera")
        aim_prim(stage, cam_prim_path, pos, target)
        cam_sensor = UsdGeom.Camera(cam_prim)
        cam_sensor.CreateFocalLengthAttr().Set(f_len)
        cam_sensor.CreateFocusDistanceAttr().Set(f_dist)
    print("[INFO] Inspection cameras created (24 total). Switch via viewport Camera menu.")
    return 24


if __name__ == "__main__":
    print(f"Inspection camera module: {len(get_camera_configs())} views defined")
