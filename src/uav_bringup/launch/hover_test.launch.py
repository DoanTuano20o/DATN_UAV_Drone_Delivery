from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mission_cfg = os.path.join(
        get_package_share_directory("uav_mission"),
        "config",
        "mission.yaml",
    )

    flight_node = Node(
        package="uav_flight",
        executable="flight_bridge_node",
        name="flight_bridge_node",
        output="screen",
    )

    mission_node = Node(
        package="uav_mission",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
        parameters=[mission_cfg],
    )

    return LaunchDescription([
        flight_node,
        mission_node,
    ])