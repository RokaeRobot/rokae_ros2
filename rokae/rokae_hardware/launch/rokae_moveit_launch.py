from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

import os

def generate_launch_description():
    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    hardware_pkg = FindPackageShare("rokae_hardware").find("rokae_hardware")
    moveit_config_pkg = FindPackageShare("rokae_moveit_config").find("rokae_xMateCR7_moveit_config")

    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    srdf_file = os.path.join(moveit_config_pkg, "config", "xMateCR7.srdf")
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
    
    ros2_control_node = Node(
            package="controller_manager",    #功能包：加载并管理控制器
            executable="ros2_control_node",  #节点：用于运行硬件接口 hardware_interface 并与控制器controller_manager通信
            #name="controller_manager",
            parameters=[robot_description, 
                        ParameterFile(controller_yaml, allow_substs=True),
                        #controller_yaml  #给节点ros2_control_node本身传递robot_description和controller_yaml文件中的参数
                        ],   
            # remappings=[("joint_states", "rokae_arm/joint_states")],
            output="both",
        )
        
    connect_node =  Node(
        package="rokae_hardware",
        executable="connect",
        parameters=[
            {
                "robot_ip": robot_ip,
                "local_ip": local_ip,
            }
        ],
        output="screen",
        )
        
    joint_state_broadcaster_node = Node(
            package="controller_manager",
            executable="spawner",
            # parameters=[robot_description, controller_yaml],
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        )
    
    position_joint_trajectory_controller_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "position_joint_trajectory_controller",
                "--controller-manager", "/controller_manager",
            ],
            output="screen"
        )
    
    delayed_position_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[position_joint_trajectory_controller_node]
        )
    )

    
    return LaunchDescription([
        ros2_control_node,
        #connect_node,
        joint_state_broadcaster_node,
        delayed_position_spawner,
        # Joint state publisher
        Node(
            package="joint_state_publisher",      #controller_manager 依赖于 joint_state_publisher 的存在来识别 joint 信息（mmp不启动这个节点后续spawner怎么可能配置成功！！！）
            executable="joint_state_publisher",   #向 /joint_states 发布 joint 名称、顺序、角度等信息
            name="joint_state_publisher",
            parameters=[
                {
                    "source_list": [
                        "rokae_arm/joint_states",
                    ],
                    "rate": 30,
                }
            ],
        ),
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        
        
        # 启动 MoveIt 的 move_group 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_pkg, "launch", "xMateCR7_moveit_config.launch.py")
            ),
        ),
    
    ])

