# movej.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    from launch_ros.substitutions import FindPackageShare
    import os

    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    moveit_config_pkg = FindPackageShare("rokae_moveit_config").find("rokae_xMateCR7_moveit_config")

    urdf_path = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    srdf_path = os.path.join(moveit_config_pkg, "config", "xMateCR7.srdf")

        # 从xacro文件中获取的参数
    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_path,
        ])
    }
    robot_description_semantic = {"robot_description_semantic": open(srdf_path).read()}

    movej_node = Node(
        package="rokae_hardware",
        executable="movej",
        name="movej",
        parameters=[robot_description, robot_description_semantic]
    )

    return LaunchDescription([movej_node])
