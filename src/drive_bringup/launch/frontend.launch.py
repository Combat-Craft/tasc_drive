import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_phidgets_hardware = get_package_share_directory('phidgets_hardware')
    config_file = os.path.join(
        pkg_phidgets_hardware,
        'config',
        'ps4_teleop.yaml'
    )

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id": 0,
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
    )

    teleop = Node(
        package="phidgets_hardware",
        executable="ps4_teleop",
        name="ps4_teleop",
        output="screen",
        parameters=[config_file],
    )

    dashboard = Node(
        package="rover_dashboard",
        executable="dashboard",
        name="rover_dashboard",
        output="screen",
    )

    return LaunchDescription([
        joy,
        teleop,
        dashboard,
    ])
