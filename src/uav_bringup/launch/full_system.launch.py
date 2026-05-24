from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    host_arg = DeclareLaunchArgument("host", default_value="0.0.0.0")
    port_arg = DeclareLaunchArgument("port", default_value="5000")

    camera_cfg = os.path.join(
        get_package_share_directory("uav_camera"),
        "config",
        "camera.yaml",
    )
    aruco_cfg = os.path.join(
        get_package_share_directory("uav_vision"),
        "config",
        "aruco.yaml",
    )
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

    camera_node = Node(
        package="uav_camera",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[camera_cfg],
    )

    vision_node = Node(
        package="uav_vision",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        output="screen",
        parameters=[aruco_cfg],
    )

    servo_node = Node(
        package="uav_servo",
        executable="servo_controller_node",
        name="servo_controller_node",
        output="screen",
        parameters=[
            {
                "servo_cmd_topic": "/servo_cmd",
                "drop_done_topic": "/drop_done",
                "expected_drop_cmd": "DROP",
                "drop_done_payload": "DONE",

                "bus_id": 5,
                "i2c_address": 0x40,
                "servo_channel": 4,
                "pwm_frequency": 50,

                "servo_min_us": 500.0,
                "servo_max_us": 2500.0,
                "servo_close_angle": 10.0,
                "servo_open_angle": 55.0,
                "servo_open_hold_s": 3.0,

                "init_close_on_start": True,
                "release_pwm_on_shutdown": False,
            }
        ],
    )

    mission_node = Node(
        package="uav_mission",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
        parameters=[mission_cfg],
    )

    web_node = Node(
        package="uav_web_bridge",
        executable="web_server_node",
        name="web_server_node",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
            }
        ],
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        flight_node,
        camera_node,
        vision_node,
        servo_node,
        mission_node,
        web_node,
    ])