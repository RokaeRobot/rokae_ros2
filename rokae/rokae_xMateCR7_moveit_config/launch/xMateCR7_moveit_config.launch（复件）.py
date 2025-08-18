from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():
    # 获取路径
    moveit_config_pkg = FindPackageShare('rokae_moveit_config').find('rokae_xMateCR7_moveit_config')
    description_pkg = FindPackageShare("rokae_description").find("rokae_description")

    # 获取参数
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')

    # URDF/xacro 路径
    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")

    # robot_description 参数（xacro 动态传参）
    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_file,
            " robot_ip:=", robot_ip,
            " local_ip:=", local_ip,
            " use_fake_hardware:=true"
        ])
    }

    robot_description_semantic_config = os.path.join(
        moveit_config_pkg, "config", "xMateCR7.srdf"
    )
    with open(robot_description_semantic_config, 'r') as f:
        srdf_content = f.read()

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(srdf_content, value_type=str)
    }

    kinematics_yaml = os.path.join(moveit_config_pkg, "config", "kinemics.yaml")
    joint_limits_yaml = os.path.join(moveit_config_pkg, "config", "joint_lims.yaml")
    ompl_planning_yaml = os.path.join(moveit_config_pkg, "config", "ompl_plnning.yaml")
    controllers_yaml = os.path.join(moveit_config_pkg, "config", "simple_moveit_controllers.yaml")

    # 启动 move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            # kinematics_yaml,
            # joint_limits_yaml,
            # ompl_planning_yaml,
            # controllers_yaml,
            {"planning_pipelines": ["ompl"], "allow_trajectory_execution": True},
        ],
    )

    # RViz可选节点（带 moveit config）
    rviz_config_file = os.path.join(moveit_config_pkg, "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.21.10"),
        DeclareLaunchArgument("local_ip", default_value="192.168.21.131"),

        move_group_node,
        #rviz_node,
    ])
