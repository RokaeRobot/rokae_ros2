# Copyright (c) 2021 PickNik, Inc.

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    robot_sn = LaunchConfiguration("robot_sn")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", robot_type, "joint_limits.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", robot_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", robot_type, "visual_parameters.yaml"]
    )
    # script_filename = PathJoinSubstitution(
    #     [FindPackageShare("rokae_client_library"), "resources", "external_control.urscript"]
    # )
    # input_recipe_filename = PathJoinSubstitution(
    #     [FindPackageShare("rokae_robot_driver"), "resources", "rtde_input_recipe.txt"]
    # )
    # output_recipe_filename = PathJoinSubstitution(
    #     [FindPackageShare("rokae_robot_driver"), "resources", "rtde_output_recipe.txt"]
    # )

    robot_description_content = Command(
        [  
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params_file,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            robot_type,
            " ",
            # "script_filename:=",
            # script_filename,
            # " ",
            # "input_recipe_filename:=",
            # input_recipe_filename,
            # " ",
            # "output_recipe_filename:=",
            # output_recipe_filename,
            # " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "tool_parity:=",
            tool_parity,
            " ",
            "tool_baud_rate:=",
            tool_baud_rate,
            " ",
            "tool_stop_bits:=",
            tool_stop_bits,
            " ",
            "tool_rx_idle_chars:=",
            tool_rx_idle_chars,
            " ",
            "tool_tx_idle_chars:=",
            tool_tx_idle_chars,
            " ",
            "tool_device_name:=",
            tool_device_name,
            " ",
            "tool_tcp_port:=",
            tool_tcp_port,
            " ",
            "tool_voltage:=",
            tool_voltage,
            " ",
            "reverse_ip:=",
            reverse_ip,
            " ",
            "script_command_port:=",
            script_command_port,
            " ",
            "reverse_port:=",
            reverse_port,
            " ",
            "script_sender_port:=",
            script_sender_port,
            " ",
            "trajectory_port:=",
            trajectory_port,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "xMateCR7.rviz"]
    )

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            robot_type.perform(context) + "_update_rate.yaml",
        ]
    )
    
    
    # Robot controllers
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("rokae_ngup"), "config", "xMateCR_controllers.yaml"]
    )
    
    # Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(robot_controllers, allow_substs=True),
            #{"robot_sn": robot_sn},
        ],
        #remappings=[("joint_states", "flexiv_arm/joint_states")],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    # control_node = Node(
    #     package="controller_manager",   #启动控制器节点
    #     executable="ros2_control_node",
    #     parameters=[
    #         robot_description,
    #         update_rate_config_file,
    #         ParameterFile(initial_joint_controllers, allow_substs=True),
    #     ],
    #     output="screen",
    #     condition=IfCondition(use_fake_hardware),
    # )

    # rokae_control_node = Node(
    #     package="rokae_robot_driver",
    #     executable="rokae_ros2_control_node",
    #     parameters=[
    #         robot_description,
    #         update_rate_config_file,
    #         ParameterFile(initial_joint_controllers, allow_substs=True),
    #     ],
    #     output="screen",
    #     condition=UnlessCondition(use_fake_hardware),
    # )

    # dashboard_client_node = Node(
    #     package="rokae_robot_driver",
    #     condition=IfCondition(
    #         AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
    #     ),
    #     executable="dashboard_client",
    #     name="dashboard_client",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[{"robot_ip": robot_ip}],
    # )

    # robot_state_helper_node = Node(
    #     package="rokae_robot_driver",
    #     executable="robot_state_helper",
    #     name="rokae_robot_state_helper",
    #     output="screen",
    #     condition=UnlessCondition(use_fake_hardware),
    #     parameters=[
    #         {"headless_mode": headless_mode},
    #         {"robot_ip": robot_ip},
    #     ],
    # )

    # tool_communication_node = Node(
    #     package="rokae_robot_driver",
    #     condition=IfCondition(use_tool_communication),
    #     executable="tool_communication.py",
    #     name="rokae_tool_comm",
    #     output="screen",
    #     parameters=[
    #         {
    #             "robot_ip": robot_ip,
    #             "tcp_port": tool_tcp_port,
    #             "device_name": tool_device_name,
    #         }
    #     ],
    # )

    connect = Node(
        package="rokae_hardware",
        executable="connect",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    # controller_stopper_node = Node(
    #     package="rokae_robot_driver",
    #     executable="controller_stopper_node",
    #     name="controller_stopper",
    #     output="screen",
    #     emulate_tty=True,
    #     condition=UnlessCondition(use_fake_hardware),
    #     parameters=[
    #         {"headless_mode": headless_mode},
    #         {"joint_controller_active": activate_joint_controller},
    #         {
    #             "consistent_controllers": [
    #                 "io_and_status_controller",
    #                 "force_torque_sensor_broadcaster",
    #                 "joint_state_broadcaster",
    #                 "speed_scaling_state_broadcaster",
    #                 "tcp_pose_broadcaster",
    #                 "rokae_configuration_controller",
    #             ]
    #         },
    #     ],
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [  #节点启动后如下组件自启动
        "joint_state_broadcaster",
        "position_joint_trajectory_controller",
        "velocity_joint_trajectory_controller",
        "torque_joint_trajectory_controller",
        "joint_group_position_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]
    # if activate_joint_controller.perform(context) == "true":
    #     controllers_active.append(initial_joint_controller.perform(context))
    #     controllers_inactive.remove(initial_joint_controller.perform(context))

    # if use_fake_hardware.perform(context) == "true":
    #     controllers_active.remove("tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        #controller_spawner(controllers_inactive, active=False),
    ]

    nodes_to_start = [
        control_node,
        # rokae_control_node,
        # dashboard_client_node,
        # robot_state_helper_node,
        # tool_communication_node,
        # controller_stopper_node,
        connect,
        robot_state_publisher_node,
        rviz_node,
    ] + controller_spawners

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # Rokae specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of used Rokae robot.",
            choices=[
                "xMateCR7",
                "xMateCR5",
                "xCore",
                "xMateSR7",
                "xMateSR5",
                # 可根据实际型号扩充
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="rokae_robot_driver",
            description='Package with the controller\'s configuration in "config" folder. '
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="xMateCR_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="rokae_description",   #功能包名称，不能和robot_description这里重名，否则无法加载.xacro文件
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="xMate.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("robot_type"),
                    "default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch Dashboard Client?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyROKAE",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
