"""ament_python packaging for rgbd_perception (also runnable directly: python3 <module>.py)."""
from setuptools import setup

package_name = "rgbd_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="SKATEBOARD_IRP team",
    maintainer_email="team@skateboard",
    description="YOLO26n-seg RGB-only detection + mask-median depth fusion + camera-only planner.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_seg_node = rgbd_perception.yolo_seg_node:main",
            "rgbd_fusion_node = rgbd_perception.rgbd_fusion_node:main",
            "camera_planner_node = rgbd_perception.camera_planner_node:main",
        ],
    },
)
