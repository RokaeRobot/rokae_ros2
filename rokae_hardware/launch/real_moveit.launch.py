from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import xacro


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    local_ip = LaunchConfiguration("local_ip").perform(context)
    enable_moveit = LaunchConfiguration("enable_moveit").perform(context).lower() in ("1", "true", "yes")
    enable_driver = LaunchConfiguration("enable_driver").perform(context).lower() in ("1", "true", "yes")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true", "yes")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout").perform(context)
    service_call_timeout = LaunchConfiguration("service_call_timeout").perform(context)

    supported_types = [
        "CR7", "CR12", "CR18", "CR20",
        "ER3", "ER7", "SR3", "SR4", "SR5",
        "Pro3", "Pro7", "AR5L", "AR5R",
    ]
    if robot_type not in supported_types:
        raise RuntimeError(f"Unsupported robot type: {robot_type}")

    description_pkg = get_package_share_directory("rokae_description")

    # Use the unified xMate entrypoint so ros2_control is injected consistently
    # by xMate_macro.xacro (matches rokae_moveit_launch.py behavior).
    urdf_path = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF not found: {urdf_path}")
    robot_desc_xml = xacro.process_file(
        urdf_path,
        mappings={
            "robot_type": robot_type,
            "robot_ip": robot_ip,
            "local_ip": local_ip,
            "use_fake_hardware": "false",
        },
    ).toxml()

    moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
    moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)
    srdf_file = os.path.join(moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf")
    kinematics_yaml = os.path.join(moveit_config_pkg_share, "config", "kinematics.yaml")

    with open(kinematics_yaml, "r", encoding="utf-8") as f:
        kinematics_params = yaml.safe_load(f)

    robot_description = {"robot_description": robot_desc_xml}
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(Command(["cat ", srdf_file]), value_type=str)
    }

    hardware_pkg = get_package_share_directory("rokae_hardware")
    real_controllers_yaml = os.path.join(
        hardware_pkg, "config", f"xMate{robot_type}_real_controllers.yaml"
    )
    default_controllers_yaml = os.path.join(
        hardware_pkg, "config", f"xMate{robot_type}_controllers.yaml"
    )
    # Follow CR7 real-mode behavior: prefer dedicated real controller config.
    controllers_yaml = (
        real_controllers_yaml
        if os.path.exists(real_controllers_yaml)
        else default_controllers_yaml
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_yaml, {"use_sim_time": use_sim_time}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    spawner_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
            "--service-call-timeout", service_call_timeout,
        ],
        output="screen",
    )

    spawner_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
            "--service-call-timeout", service_call_timeout,
        ],
        output="screen",
    )

    delayed_joint_state_spawner = TimerAction(
        period=2.0,
        actions=[spawner_joint_state_broadcaster],
    )

    delayed_arm_spawner = TimerAction(
        period=5.0,
        actions=[spawner_arm_controller],
    )

    actions = [
        ros2_control_node,
        robot_state_publisher,
        delayed_joint_state_spawner,
        delayed_arm_spawner,
    ]

    if enable_driver:
        actions.append(
            Node(
                package="rokae_hardware",
                executable="rokae_driver",
                name="rokae_driver",
                output="screen",
                parameters=[{"robot_ip": robot_ip, "local_ip": local_ip}],
            )
        )

    if enable_moveit:
        actions.append(
            Node(
                package="rokae_hardware",
                executable="movej",
                name="movej",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_params,
                    {"use_sim_time": use_sim_time},
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="CR7"),
            DeclareLaunchArgument("robot_ip", default_value="10.0.2.163"),
            DeclareLaunchArgument("local_ip", default_value="10.0.2.162"),
            DeclareLaunchArgument("enable_moveit", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="120"),
            DeclareLaunchArgument("service_call_timeout", default_value="120"),
            DeclareLaunchArgument(
                "enable_driver",
                default_value="false",
                description="Extra rokae_driver node. Usually OFF when using RokaeHardwareInterface (avoids double SDK connection).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
