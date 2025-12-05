from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
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
    # -------- 新增：选择机型参数 --------
    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="CR7",
        description="Robot type: CR7 or SR4"
    )
    
    # 声明launch参数，提供默认IP（根据你实际情况改）
    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.21.10')
    local_ip = LaunchConfiguration('local_ip', default='192.168.21.131')
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    
    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    hardware_pkg = FindPackageShare("rokae_hardware").find("rokae_hardware")
    # moveit_config_pkg = FindPackageShare("rokae_moveit_config")
   # 拼出 rokae_xMateCR7_moveit_config 包名
    moveit_config_pkg_name = PythonExpression([
        "'rokae_xMate' + '", robot_type, "' + '_moveit_config'"
    ])

    # 得到 rokae_xMateCR7_moveit_config 的 share 路径
    moveit_config_pkg_share = FindPackageShare(moveit_config_pkg_name)

    # 拼接到 launch 文件   /path/to/rokae_hardware/launch/xMate_moveit_config.launch.py
    moveit_config_launch_file = PathJoinSubstitution([
        hardware_pkg,   #只启动一个moveit.launch.py
        "launch",
        PythonExpression([
        "'xMate_moveit_config.launch.py'"
        ])
        # moveit_config_pkg_share,    #根据不同机型启动不同launch.py
        # "launch",
        # # "xMateCR7_moveit_config.launch.py"
        # PythonExpression([
        # "'xMate' + '", robot_type, "' + '_moveit_config.launch.py'"
        # ])
    ])


    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    # controller_yaml = os.path.join(hardware_pkg, "config", "xMateCR_controllers.yaml")
    # -------- 根据机型选择控制器配置 --------
    # 控制器配置    hardware  文件夹下
    controller_yaml = PathJoinSubstitution([
        hardware_pkg,
        "config",
        PythonExpression([
            "'xMate' + '", robot_type, "' + '_controllers.yaml'"
        ])
    ])
    
     
    
    # 从xacro文件中获取的参数
    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_file,
            " robot_type:=", robot_type,    #这里把机型传入 xacro
            " robot_ip:=", robot_ip,
            " local_ip:=", local_ip,
            " use_fake_hardware:=",use_fake_hardware,
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
            # prefix="gdb -ex run --args",   #gdb调试打印硬件接口运行情况，定位崩溃位置
            # prefix="xterm -e gdb -ex run --args",
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
    
    # 启动 MoveIt 的 move_group 节点
    # 传给 IncludeLaunchDescription
    # move_group 延迟到 ros2_control_node 启动完成之后再加载
    delayed_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_joint_trajectory_controller_node,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_config_launch_file)
                )
            ],
        )
    )

    
    return LaunchDescription([
        robot_type_arg,  # 加入参数声明
        ros2_control_node,
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
                        "joint_states",
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
        # 传给 IncludeLaunchDescription
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(moveit_config_launch_file)
        # ),
        delayed_move_group,
    ])

