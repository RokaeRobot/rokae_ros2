from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

import os


def generate_launch_description():
    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="CR7",
        description="Robot type for xMate.urdf.xacro and xMate{type}_controllers.yaml (e.g. CR7, CR35, SR4).",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.21.10",
        description="Robot controller IP.",
    )
    local_ip_arg = DeclareLaunchArgument(
        "local_ip",
        default_value="192.168.21.131",
        description="This PC IP on the robot subnet.",
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="If true, ros2_control uses mock hardware (no real robot from driver).",
    )
    controller_manager_timeout_arg = DeclareLaunchArgument(
        "controller_manager_timeout",
        default_value="120",
        description="Timeout waiting for /controller_manager services.",
    )
    service_call_timeout_arg = DeclareLaunchArgument(
        "service_call_timeout",
        default_value="120",
        description="Timeout for controller service calls.",
    )

    warehouse_sqlite_path_arg = DeclareLaunchArgument(
        "warehouse_sqlite_path",
        default_value="",
        description="Optional MoveIt warehouse DB path (passed to nested MoveIt launch).",
    )

    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    local_ip = LaunchConfiguration("local_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout")
    service_call_timeout = LaunchConfiguration("service_call_timeout")

    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    hardware_pkg = FindPackageShare("rokae_hardware").find("rokae_hardware")

    moveit_config_pkg_name = PythonExpression([
        "'rokae_xMate' + '", robot_type, "' + '_moveit_config'"
    ])

    moveit_config_launch_file = PathJoinSubstitution([
        hardware_pkg,
        "launch",
        PythonExpression([
            "'xMate_moveit_config.launch.py'"
        ])
    ])

    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    controller_yaml = PathJoinSubstitution([
        hardware_pkg,
        "config",
        PythonExpression([
            "'xMate' + '", robot_type, "' + '_real_controllers.yaml' "
            "if (('", use_fake_hardware, "' == 'false') and "
            "('", robot_type, "' in ['CR7','CR12','CR18','CR20','ER3','ER7','AR5L','AR5R','Pro3','Pro7','SR3','SR4','SR5'])) "
            "else ('xMate' + '", robot_type, "' + '_controllers.yaml')"
        ])
    ])

    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_file,
            " robot_type:=", robot_type,
            " robot_ip:=", robot_ip,
            " local_ip:=", local_ip,
            " use_fake_hardware:=", use_fake_hardware,
        ])
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(controller_yaml, allow_substs=True),
        ],
        output="both",
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
            "--service-call-timeout", service_call_timeout,
        ],
        output="screen"
    )

    position_joint_trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
            "--service-call-timeout", service_call_timeout,
        ],
        output="screen"
    )

    delayed_position_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[position_joint_trajectory_controller_node]
        )
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": ["joint_states"],
                "rate": 30,
            }
        ],
    )

    delayed_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_joint_trajectory_controller_node,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_config_launch_file),
                    launch_arguments=[
                        ("robot_type", robot_type),
                        ("robot_ip", robot_ip),
                        ("local_ip", local_ip),
                        ("use_fake_hardware", use_fake_hardware),
                        ("warehouse_sqlite_path", warehouse_sqlite_path),
                    ],
                )
            ],
        )
    )

    return LaunchDescription([
        robot_type_arg,
        robot_ip_arg,
        local_ip_arg,
        use_fake_hardware_arg,
        warehouse_sqlite_path_arg,
        controller_manager_timeout_arg,
        service_call_timeout_arg,
        ros2_control_node,
        joint_state_broadcaster_node,
        delayed_position_spawner,
        joint_state_publisher_gui_node,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        delayed_move_group,
    ])
