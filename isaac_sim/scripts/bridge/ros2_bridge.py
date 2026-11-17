"""
Milestone 6: ROS 2 Bridge for SKATEBOARD_IRP (Isaac Sim 5.1 + ROS 2 Humble).

Builds an OmniGraph ActionGraph (/ROS2_Bridge) that streams:
  - RGB image        -> camera_cfg["rgb_topic"]             (sensor_msgs/Image)
  - Depth image      -> camera_cfg["depth_topic"]           (sensor_msgs/Image)
  - RGB CameraInfo   -> camera_cfg["rgb_camera_info_topic"] (sensor_msgs/CameraInfo)
  - Depth CameraInfo -> camera_cfg["depth_camera_info_topic"]
  - Sim clock        -> sim_cfg["ros2_bridge"]["clock_topic"] (rosgraph_msgs/Clock)
  - TF tree          -> sim_cfg["ros2_bridge"]["tf_topic"]    (tf2_msgs/TFMessage)

Canonical node wiring follows NVIDIA's own bridge tests:
  exts/isaacsim.ros2.bridge/isaacsim/ros2/bridge/tests/
    test_camera.py, test_clock.py, test_pose_tree.py

Camera namespace separation (enforced by construction):
  - REAL SENSE CAMERA (streams ROS topics): the single UsdGeom.Camera at
    /World/Vehicle/Chassis/Sensors/RGBD_Camera/Camera (+ Camera_Body/Lens_RGB/
    Lens_Depth visuals, owned by visual_vehicle.py). The ONLY camera with a
    render product and the ONLY camera the bridge graph touches.
  - INSPECTION CAMERAS (viewport viewpoints only): 24 UsdGeom.Camera prims
    under /World/InspectionCameras/*. They have NO render products, NO graph
    connections, and publish NOTHING — switching the viewport to one of them
    never disturbs the ROS streams.

MODEL DEPLOYMENT CONTRACT (stable — build your stack on exactly this):
  Subscriptions (all sim-time stamped, QoS default):
    /camera/rgb/image_raw          sensor_msgs/Image      rgb8    640x480
      frame_id: camera_rgb_optical_frame
    /camera/depth/image_raw        sensor_msgs/Image      32FC1   640x480, metres
      frame_id: camera_depth_optical_frame
    /camera/rgb/camera_info        sensor_msgs/CameraInfo (plumb_bob, square pixels)
    /camera/depth/camera_info      sensor_msgs/CameraInfo
    /clock                         rosgraph_msgs/Clock    (use with use_sim_time:=true)
    /tf                            tf2_msgs/TFMessage     frames: Vehicle, RGBD_Camera, ...
  The model MUST subscribe only to the topics above. No inspection-camera
  imagery ever appears on ROS topics by design (guard raises ValueError).
"""

GRAPH_PATH = "/ROS2_Bridge"


def setup_ros2_bridge(camera_prim_path: str, camera_cfg: dict, sim_cfg: dict,
                      vehicle_path: str = "/World/Vehicle") -> str:
    """
    Enable isaacsim.ros2.bridge and build the /ROS2_Bridge ActionGraph.

    Args:
        camera_prim_path: USD path of the UsdGeom.Camera prim
            (e.g. "/World/Vehicle/Chassis/Sensors/RGBD_Camera/Camera").
        camera_cfg: camera section of configs/camera.yaml.
        sim_cfg: simulation section of configs/simulation.yaml.
        vehicle_path: vehicle root prim (whole subtree published to TF).

    Returns:
        Graph path ("/ROS2_Bridge").
    """
    import omni.graph.core as og
    import omni.kit.app
    import usdrt.Sdf

    # 0. GUARD: the bridge may ONLY bind the RGBD camera. Refuse any other.
    #    This enforces the future-phase contract: model inputs ONLY from RGBD
    #    (RGB, depth, CameraInfo, clock, TF).  DriverSeat_Camera and all
    #    inspection cameras are viewport-only — they have NO render product
    #    and must never publish.
    if "/InspectionCameras/" in camera_prim_path:
        raise ValueError(
            f"REFUSED: bridge must bind RGBD camera, got inspection camera: {camera_prim_path}"
        )
    if "/DriverSeat_Camera" in camera_prim_path or "/Driver_Seat_Camera" in camera_prim_path:
        raise ValueError(
            f"REFUSED: DriverSeat_Camera is viewport-only (no ROS topics). Use RGBD camera: {camera_prim_path}"
        )
    if not camera_prim_path.startswith("/World/Vehicle/Chassis/Sensors/RGBD_Camera/RGBD_Camera"):
        # Allow legacy "/Camera" suffix for backward compat during transition
        if not camera_prim_path.startswith("/World/Vehicle/Chassis/Sensors/RGBD_Camera/"):
            raise ValueError(
                f"REFUSED: unexpected camera for ROS bridge: {camera_prim_path} (only RGBD_Camera/RGBD_Camera allowed)"
            )

    # 1. Enable the ROS 2 bridge extension (provides all OG nodes below)
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    if not ext_manager.is_extension_enabled("isaacsim.ros2.bridge"):
        ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    print("[INFO] isaacsim.ros2.bridge extension enabled.")

    bridge_cfg = sim_cfg.get("ros2_bridge", {})
    clock_topic = bridge_cfg.get("clock_topic", "/clock")
    tf_topic = bridge_cfg.get("tf_topic", "/tf")

    width = int(camera_cfg.get("image_width", 640))
    height = int(camera_cfg.get("image_height", 480))
    camera_mount_path = camera_prim_path.rsplit("/", 1)[0]  # .../Sensors/RGBD_Camera

    # 2. Build the ActionGraph (if not already existing)
    stage = omni.usd.get_context().get_stage()
    if stage and stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        print(f"[INFO] ROS 2 bridge graph already exists at {GRAPH_PATH}.")
        return GRAPH_PATH

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("RGBInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("DepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("SubAckermann", "isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                ("CreateRenderProduct.inputs:width", width),
                ("CreateRenderProduct.inputs:height", height),

                ("RGB.inputs:topicName", camera_cfg["rgb_topic"]),
                ("RGB.inputs:type", "rgb"),
                ("RGB.inputs:frameId", camera_cfg.get("rgb_optical_frame", "camera_rgb_optical_frame")),

                ("Depth.inputs:topicName", camera_cfg["depth_topic"]),
                ("Depth.inputs:type", "depth"),
                ("Depth.inputs:frameId", camera_cfg.get("depth_optical_frame", "camera_depth_optical_frame")),

                ("RGBInfo.inputs:topicName", camera_cfg["rgb_camera_info_topic"]),
                ("RGBInfo.inputs:frameId", camera_cfg.get("rgb_optical_frame", "camera_rgb_optical_frame")),

                ("DepthInfo.inputs:topicName", camera_cfg["depth_camera_info_topic"]),
                ("DepthInfo.inputs:frameId", camera_cfg.get("depth_optical_frame", "camera_depth_optical_frame")),

                ("Clock.inputs:topicName", clock_topic),

                ("PublishTF.inputs:topicName", tf_topic),
                ("PublishTF.inputs:targetPrims", [
                    usdrt.Sdf.Path(vehicle_path),
                    usdrt.Sdf.Path(camera_mount_path),
                ]),

                ("SubAckermann.inputs:topicName", "/cmd_ackermann"),
            ],
            og.Controller.Keys.CONNECT: [
                # Render product fans out to all image/info publishers + TF
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RGB.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "Depth.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RGBInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishTF.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RGB.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "RGBInfo.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthInfo.inputs:renderProductPath"),
                # Sim-time clock
                ("OnPlaybackTick.outputs:tick", "Clock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubAckermann.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
        },
    )

    print(f"[INFO] ROS 2 bridge graph created at {GRAPH_PATH}:")
    print(f"  RGB         -> {camera_cfg['rgb_topic']}")
    print(f"  Depth       -> {camera_cfg['depth_topic']}")
    print(f"  RGB info    -> {camera_cfg['rgb_camera_info_topic']}")
    print(f"  Depth info  -> {camera_cfg['depth_camera_info_topic']}")
    print(f"  Clock       -> {clock_topic}")
    print(f"  TF tree     -> {tf_topic} (frames under {vehicle_path})")
    print(f"  Ackermann   <- /cmd_ackermann (subscribed via SubAckermann)")
    return GRAPH_PATH


if __name__ == "__main__":
    print("ROS 2 bridge module loaded successfully")
