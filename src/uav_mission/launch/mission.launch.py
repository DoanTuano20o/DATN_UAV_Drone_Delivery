from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = Path(__file__).resolve().parents[1]
    config_path = str(pkg_dir / "config" / "mission.yaml")

    return LaunchDescription([
        Node(
            package="uav_mission",
            executable="mission_manager_node",
            name="mission_manager_node",
            output="screen",
            parameters=[config_path],
        )
    ])