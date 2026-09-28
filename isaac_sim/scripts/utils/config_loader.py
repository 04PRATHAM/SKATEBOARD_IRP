"""
Configuration loader utility for YAML config files.
"""

import yaml
import os


def load_config(config_path: str) -> dict:
    """
    Load a YAML configuration file from the project root.
    
    Args:
        config_path: Path relative to project root (e.g., "configs/vehicle.yaml")
    
    Returns:
        Dictionary with configuration values
    """
    project_root = "/home/ubuntu/SKATEBOARD_IRP"
    full_path = os.path.join(project_root, config_path)
    
    if not os.path.exists(full_path):
        raise FileNotFoundError(f"Config file not found: {full_path}")
    
    with open(full_path, "r") as f:
        config = yaml.safe_load(f)
    
    return config


def get_vehicle_config() -> dict:
    """Get vehicle configuration."""
    return load_config("configs/vehicle.yaml")["vehicle"]


def get_camera_config() -> dict:
    """Get camera configuration."""
    return load_config("configs/camera.yaml")["camera"]


def get_simulation_config() -> dict:
    """Get simulation configuration."""
    return load_config("configs/simulation.yaml")["simulation"]


def get_steering_config() -> dict:
    """Get steering configuration."""
    return load_config("configs/steering.yaml")["steering"]