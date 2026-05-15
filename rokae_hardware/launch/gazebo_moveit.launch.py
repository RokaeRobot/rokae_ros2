"""
统一入口：仿真（Gazebo + gazebo_ros2_control + controll_movej）或真机（real_moveit）。

- mode:=sim / hardware_interface:=gazebo → controll_movej.launch.py（world → gazebo_world_file）
- mode:=real / hardware_interface:=real → real_moveit.launch.py（无 Gazebo）
- enable_moveit:=true（仅允许 mode:=real）→ xMate_moveit_config.launch.py（move_group + RViz）
  仿真下请先只用 enable_gui / enable_movej；仿真 + MoveIt 需与 Gazebo 共用同一 URDF，避免与 xMate.urdf.xacro 冲突。

示例:
  ros2 launch rokae_hardware gazebo_moveit.launch.py \\
    mode:=sim robot_type:=CR35 world:=obstacles.world enable_gui:=true enable_movej:=false

  ros2 launch rokae_hardware gazebo_moveit.launch.py \\
    mode:=real robot_type:=CR35 robot_ip:=192.168.2.160 local_ip:=192.168.2.162 \\
    enable_moveit:=true use_sim_time:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


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


def _validate_args(context, *args, **kwargs):
    mode = LaunchConfiguration("resolved_mode").perform(context).strip().lower()
    if mode not in ("sim", "real"):
        raise RuntimeError(f"Invalid runtime mode '{mode}'. Expected 'sim' or 'real'.")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )
    if mode == "real" and use_sim_time:
        raise RuntimeError("mode=real 需要 use_sim_time=false（真机使用系统时钟）。")

    enable_gui = LaunchConfiguration("enable_gui").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )
    enable_movej = LaunchConfiguration("enable_movej").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )
    enable_moveit = LaunchConfiguration("enable_moveit").perform(context).strip().lower() in (
        "1",
        "true",
        "yes",
    )

    if mode == "sim" and enable_gui and enable_movej:
        raise RuntimeError(
            "mode=sim 下不能同时 enable_gui=true 与 enable_movej=true（多源争夺同一轨迹控制器）。"
        )

    if mode == "real" and (enable_gui or enable_movej):
        raise RuntimeError(
            "mode=real 不支持 enable_gui / enable_movej；规划请用 enable_moveit:=true（MoveIt+RViz）或外部发轨迹。"
        )

    if mode == "sim" and enable_moveit:
        raise RuntimeError(
            "mode=sim 与 enable_moveit 不可同时使用：仿真模型来自 xMate*_Gazebo.urdf.xacro，"
            "MoveIt 侧使用 xMate.urdf.xacro，会重复/不一致。仿真请仅使用 controll_movej 的 "
            "enable_gui 或 enable_movej；若需 MoveIt+Gazebo 联合，请另开 issue 做专用一体化 launch。"
        )

    if mode == "real":
        robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
        local_ip = LaunchConfiguration("local_ip").perform(context).strip()
        if not robot_ip or not local_ip:
            raise RuntimeError("mode=real 必须同时提供 robot_ip 与 local_ip。")
    return []


def _bringup(context, *args, **kwargs):
    hardware_pkg = get_package_share_directory("rokae_hardware")
    mode = LaunchConfiguration("resolved_mode").perform(context).strip().lower()

    sim_launch = os.path.join(hardware_pkg, "launch", "controll_movej.launch.py")
    real_launch = os.path.join(hardware_pkg, "launch", "real_moveit.launch.py")
    moveit_launch = os.path.join(hardware_pkg, "launch", "xMate_moveit_config.launch.py")

    robot_type = LaunchConfiguration("robot_type").perform(context)
    world = LaunchConfiguration("world").perform(context)
    enable_gui = LaunchConfiguration("enable_gui").perform(context)
    enable_movej = LaunchConfiguration("enable_movej").perform(context)
    enable_moveit_raw = LaunchConfiguration("enable_moveit").perform(context)
    enable_moveit_ui = enable_moveit_raw.strip().lower() in ("1", "true", "yes")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    local_ip = LaunchConfiguration("local_ip").perform(context)
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout").perform(context)
    service_call_timeout = LaunchConfiguration("service_call_timeout").perform(context)
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path").perform(context)

    actions = []

    if mode == "sim":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "robot_type": robot_type,
                    "gazebo_world_file": world,
                    "enable_gui": enable_gui,
                    "enable_movej": enable_movej,
                    "controller_manager_timeout": controller_manager_timeout,
                    "service_call_timeout": service_call_timeout,
                }.items(),
            )
        )
    else:
        # 统一入口中 MoveIt+RViz 由 xMate_moveit_config 启动；不在此叠加 real_moveit 的 movej
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_launch),
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
        )

    if enable_moveit_ui:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={
                    "robot_type": robot_type,
                    "robot_ip": robot_ip,
                    "local_ip": local_ip,
                    "use_fake_hardware": "false",
                    "use_sim_time": use_sim_time,
                    "warehouse_sqlite_path": warehouse_sqlite_path,
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="CR7",
                description="机型后缀（CR35、CR7、SR3、Pro3 等），与 controll_movej / real_moveit 一致。",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="sim 或 real；若设置 hardware_interface 则以其为准。",
            ),
            DeclareLaunchArgument(
                "hardware_interface",
                default_value="",
                description="别名：gazebo → sim；real → real。非空时覆盖 mode。",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="仿真 true；真机请 false。",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty.world",
                description="rokae_gazebo/worlds 下文件名；仅 mode=sim 有效，传入 controll_movej 的 gazebo_world_file。",
            ),
            DeclareLaunchArgument("enable_gui", default_value="false"),
            DeclareLaunchArgument(
                "enable_movej",
                default_value="false",
                description="sim 下启用 movej 演示节点；勿与 enable_gui 同时为 true。",
            ),
            DeclareLaunchArgument(
                "enable_moveit",
                default_value="false",
                description="仅 mode=real：启动 MoveIt move_group + RViz（经 xMate_moveit_config.launch.py）。sim 下请勿开启。",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="",
                description="真机控制器 IP（mode=real 必填）。",
            ),
            DeclareLaunchArgument(
                "local_ip",
                default_value="",
                description="本机位于机器人网段的 IP（mode=real 必填）。",
            ),
            DeclareLaunchArgument("warehouse_sqlite_path", default_value=""),
            DeclareLaunchArgument("controller_manager_timeout", default_value="120"),
            DeclareLaunchArgument("service_call_timeout", default_value="120"),
            OpaqueFunction(function=_resolve_mode),
            OpaqueFunction(function=_validate_args),
            OpaqueFunction(function=_bringup),
        ]
    )
