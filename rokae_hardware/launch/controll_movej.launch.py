# movej.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def launch_setup(context, *args, **kwargs):
    from launch_ros.substitutions import FindPackageShare
    import os
    
    # -------- 新增：选择机型参数 --------
    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="CR7",
        description="Robot type: CR7 or SR4"
    )
    
    # 声明launch参数，提供默认IP（根据你实际情况改）
    robot_type = LaunchConfiguration("robot_type").perform(context)

    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    #moveit_config_pkg = FindPackageShare("rokae_moveit_config").find("rokae_xMateSR4_moveit_config")

    urdf_path = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    #srdf_path = os.path.join(moveit_config_pkg, "config", "xMateSR4.srdf")
    
    # moveit_config 包名和 share
    moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
    moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)

    # SRDF 文件
    srdf_file = os.path.join(
        moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf"
    )
    

        # 从xacro文件中获取的参数
    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_path,
            " robot_type:=", robot_type    #这里把机型传入 xacro
        ])
    }
        # robot_description_semantic
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]),
            value_type=str
        )
    }

    # kinematics
    robot_description_kinematics = os.path.join(
        moveit_config_pkg_share, "config", "kinematics.yaml"
    )

    movej_node = Node(
        package="rokae_hardware",
        executable="movej",
        name="movej",
        parameters=[
            robot_description, 
            robot_description_semantic,
            robot_description_kinematics,
            ],
        # prefix="xterm -e gdb -ex run --args",
        # prefix = "valgrind --leak-check=full --show-leak-kinds=all"
    )

    return [movej_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="CR7", description="Robot type: SR4/ER7/..."),
        OpaqueFunction(function=launch_setup)
    ])
