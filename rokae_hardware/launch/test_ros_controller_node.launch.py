from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import os

def generate_launch_description():
    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    hardware_pkg = FindPackageShare("rokae_hardware").find("rokae_hardware")

    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    controller_yaml = os.path.join(hardware_pkg, "config", "xMateCR_controllers.yaml")

    # 声明launch参数，提供默认IP（根据你实际情况改）
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.21.10')
    local_ip = LaunchConfiguration('local_ip', default='192.168.21.131')
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    

    # 从xacro文件中获取的参数
    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_file,
            " robot_ip:=", robot_ip,
            " local_ip:=", local_ip,
            " use_fake_hardware:=",use_fake_hardware
        ])
    }

    return LaunchDescription([

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[robot_description, controller_yaml],
            output="screen",
            #condition=IfCondition(use_fake_hardware),
        ),
        
        Node(
        package="rokae_hardware",
        executable="connect",
        parameters=[
            {
                "robot_ip": robot_ip,
                "local_ip": local_ip,
            }
        ],
        output="screen",
        ),
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            parameters=[robot_description, controller_yaml],
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),
    
    ])
