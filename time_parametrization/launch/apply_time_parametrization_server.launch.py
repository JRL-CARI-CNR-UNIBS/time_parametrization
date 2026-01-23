from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def _make_node(context, *args, **kwargs):
    config_pkg = LaunchConfiguration("config_pkg").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)

    pkg_share = get_package_share_directory(config_pkg)
    config_path = os.path.join(pkg_share, config_file)

    if not os.path.isfile(config_path):
        raise RuntimeError(
            f"Config file does not exist: '{config_path}'. "
            f"(config_pkg='{config_pkg}', config_file='{config_file}')"
        )

    return [
        Node(
            package="time_parametrization",
            executable="apply_time_parametrization_server",
            name="apply_time_parametrization_server",  # must match YAML key unless you use '/**'
            output="screen",
            parameters=[
                config_path,  # <-- load as ROS params file
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_pkg",
                default_value="time_parametrization",
                description="Package whose share/ contains the ROS params YAML file",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="config/time_parametrization_server.yaml",
                description="Path to ROS params YAML relative to config_pkg share/",
            ),
            OpaqueFunction(function=_make_node),
        ]
    )
