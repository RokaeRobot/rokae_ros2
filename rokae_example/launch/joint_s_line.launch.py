from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)

    supported_types = [
        "CR7", "CR12", "CR18", "CR20", "CR35",
        "ER3", "ER7", "SR3", "SR4", "SR5",
        "Pro3", "Pro7", "AR5L", "AR5R",
    ]
    if robot_type not in supported_types:
        raise RuntimeError(
            f"Unsupported robot type: {robot_type}. Supported types: {supported_types}"
        )

    description_pkg = get_package_share_directory("rokae_description")
    urdf_path = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF file not found: {urdf_path}")

    moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
    try:
        moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)
    except Exception as exc:
        raise RuntimeError(
            f"MoveIt configuration package not found: {moveit_config_pkg_name}"
        ) from exc

    srdf_file = os.path.join(moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf")
    if not os.path.exists(srdf_file):
        raise RuntimeError(f"SRDF file not found: {srdf_file}")

    kinematics_yaml = os.path.join(moveit_config_pkg_share, "config", "kinematics.yaml")
    if not os.path.exists(kinematics_yaml):
        raise RuntimeError(f"Kinematics YAML file not found: {kinematics_yaml}")

    with open(kinematics_yaml, "r", encoding="utf-8") as f:
        kinematics_params = yaml.safe_load(f)

    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf_path,
            " robot_type:=",
            robot_type,
        ])
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]),
            value_type=str,
        )
    }

    joint_s_line_node = Node(
        package="rokae_example",
        executable="joint_s_line",
        name="joint_s_line",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
        ],
    )

    return [joint_s_line_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type",
            default_value="CR7",
            description="Robot type: CR7/CR12/CR18/CR20/CR35/ER3/ER7/SR3/SR4/SR5/Pro3/Pro7/AR5L/AR5R",
        ),
        OpaqueFunction(function=launch_setup),
    ])
