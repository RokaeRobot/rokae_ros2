from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def _validate_args(context, *args, **kwargs):
    mode = LaunchConfiguration("resolved_mode").perform(context).strip().lower()
    if mode not in ("sim", "real"):
        raise RuntimeError(
            f"Invalid runtime mode '{mode}'. Expected 'sim' or 'real'."
        )

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )
    if mode == "real" and use_sim_time:
        raise RuntimeError("real mode requires use_sim_time=false.")

    enable_gui = LaunchConfiguration("enable_gui").perform(context).strip().lower() in ("1", "true", "yes")
    enable_movej = LaunchConfiguration("enable_movej").perform(context).strip().lower() in ("1", "true", "yes")
    enable_moveit = LaunchConfiguration("enable_moveit").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )

    if mode == "sim" and enable_gui and enable_movej:
        raise RuntimeError(
            "Conflict in gazebo mode: enable_gui=true and enable_movej=true. "
            "Please keep only one command source."
        )

    if mode == "real" and (enable_gui or enable_movej):
        raise RuntimeError(
            "mode=real does not support enable_gui/enable_movej. "
            "Use enable_moveit to run MoveIt + RViz."
        )

    if mode == "real":
        robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
        local_ip = LaunchConfiguration("local_ip").perform(context).strip()
        if not robot_ip or not local_ip:
            raise RuntimeError("mode=real requires both robot_ip and local_ip.")
    return []


def _resolve_mode(context, *args, **kwargs):
    mode_raw = LaunchConfiguration("mode").perform(context).strip().lower()
    hardware_raw = LaunchConfiguration("hardware_interface").perform(context).strip().lower()

    if mode_raw not in ("", "sim", "real"):
        raise RuntimeError(f"Invalid mode '{mode_raw}'. Expected 'sim' or 'real'.")
    if hardware_raw not in ("", "gazebo", "real"):
        raise RuntimeError(
            f"Invalid hardware_interface '{hardware_raw}'. Expected 'gazebo' or 'real'."
        )

    if hardware_raw:
        resolved_mode = "sim" if hardware_raw == "gazebo" else "real"
    else:
        resolved_mode = mode_raw or "sim"
    return [SetLaunchConfiguration("resolved_mode", resolved_mode)]


def generate_launch_description():
    hardware_pkg = get_package_share_directory("rokae_hardware")
    sim_launch = os.path.join(hardware_pkg, "launch", "controll_movej.launch.py")
    real_launch = os.path.join(hardware_pkg, "launch", "real_moveit.launch.py")
    moveit_launch = os.path.join(hardware_pkg, "launch", "xMate_moveit_config.launch.py")

    hardware_interface = LaunchConfiguration("hardware_interface")
    resolved_mode = LaunchConfiguration("resolved_mode")
    robot_type = LaunchConfiguration("robot_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    enable_gui = LaunchConfiguration("enable_gui")
    enable_movej = LaunchConfiguration("enable_movej")
    enable_moveit = LaunchConfiguration("enable_moveit")
    robot_ip = LaunchConfiguration("robot_ip")
    local_ip = LaunchConfiguration("local_ip")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout")
    service_call_timeout = LaunchConfiguration("service_call_timeout")

    is_gazebo = PythonExpression(["'", resolved_mode, "' == 'sim'"])
    is_real = PythonExpression(["'", resolved_mode, "' == 'real'"])

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        condition=IfCondition(is_gazebo),
        launch_arguments={
            "robot_type": robot_type,
            "gazebo_world_file": world,
            "enable_gui": enable_gui,
            "enable_movej": enable_movej,
            "controller_manager_timeout": controller_manager_timeout,
            "service_call_timeout": service_call_timeout,
        }.items(),
    )

    real_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_launch),
        condition=IfCondition(is_real),
        launch_arguments={
            "robot_type": robot_type,
            "robot_ip": robot_ip,
            "local_ip": local_ip,
            "enable_moveit": "false",
            "use_sim_time": use_sim_time,
            "controller_manager_timeout": controller_manager_timeout,
            "service_call_timeout": service_call_timeout,
        }.items(),
    )

    moveit_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=IfCondition(enable_moveit),
        launch_arguments={
            "robot_type": robot_type,
            "robot_ip": robot_ip,
            "local_ip": local_ip,
            "use_fake_hardware": PythonExpression(
                ["'true' if '", resolved_mode, "' == 'sim' else 'false'"]
            ),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="CR7"),
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="Compatibility mode selector: sim or real",
            ),
            DeclareLaunchArgument(
                "hardware_interface",
                default_value="",
                description="Hardware interface alias: gazebo or real (overrides mode when set)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time for ROS nodes",
            ),
            DeclareLaunchArgument("world", default_value="empty.world"),
            DeclareLaunchArgument("enable_gui", default_value="false"),
            DeclareLaunchArgument(
                "enable_movej",
                default_value="false",
                description="Enable movej demo node in gazebo mode",
            ),
            DeclareLaunchArgument(
                "enable_moveit",
                default_value="false",
                description="Enable MoveIt move_group and RViz in unified launch",
            ),
            DeclareLaunchArgument("robot_ip", default_value=""),
            DeclareLaunchArgument("local_ip", default_value=""),
            DeclareLaunchArgument("controller_manager_timeout", default_value="120"),
            DeclareLaunchArgument("service_call_timeout", default_value="120"),
            OpaqueFunction(function=_resolve_mode),
            OpaqueFunction(function=_validate_args),
            sim_include,
            real_include,
            moveit_include,
        ]
    )
