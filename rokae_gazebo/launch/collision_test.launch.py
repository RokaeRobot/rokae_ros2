"""
阶段 2.3：机型无关的碰撞测试 launch（需已启动 test_model 并激活轨迹控制器）。
示例:
  ros2 launch rokae_gazebo collision_test.launch.py robot_type:=CR7
"""
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _extract_joint_names(controllers_cfg):
    for key in (
        "arm_controller",
        "position_joint_trajectory_controller",
        "joint_state_broadcaster",
    ):
        params = controllers_cfg.get(key, {}).get("ros__parameters", {})
        joints = params.get("joints", [])
        if isinstance(joints, list) and joints:
            return joints
    return []


def _select_trajectory_controller_name(controllers_cfg):
    cm_cfg = controllers_cfg.get("controller_manager", {}).get("ros__parameters", {})
    if "arm_controller" in cm_cfg:
        return "arm_controller"
    if "position_joint_trajectory_controller" in cm_cfg:
        return "position_joint_trajectory_controller"
    return "arm_controller"


def _setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    publish_motion = LaunchConfiguration("publish_motion").perform(context).lower() in ("1", "true", "yes")
    contact_topic = LaunchConfiguration("contact_topic").perform(context)
    filter_robot_contacts = LaunchConfiguration("filter_robot_contacts").perform(context).lower() in ("1", "true", "yes")
    robot_collision_substring = LaunchConfiguration("robot_collision_substring").perform(context)
    publish_collision_signal = LaunchConfiguration("publish_collision_signal").perform(context).lower() in (
        "1", "true", "yes"
    )
    collision_signal_topic = LaunchConfiguration("collision_signal_topic").perform(context)
    collision_signal_hz = float(LaunchConfiguration("collision_signal_hz").perform(context))

    controllers_yaml_path = os.path.join(
        get_package_share_directory("rokae_hardware"),
        "config",
        f"xMate{robot_type}_controllers.yaml",
    )
    if not os.path.exists(controllers_yaml_path):
        raise RuntimeError(f"Controller config not found: {controllers_yaml_path}")

    with open(controllers_yaml_path, "r", encoding="utf-8") as f:
        controllers_cfg = yaml.safe_load(f) or {}
    joint_names = _extract_joint_names(controllers_cfg)
    if not joint_names:
        raise RuntimeError(
            f"Could not determine controlled joints from {controllers_yaml_path}"
        )
    controller_name = _select_trajectory_controller_name(controllers_cfg)
    traj_topic = f"/{controller_name}/joint_trajectory"

    return [
        Node(
            package="rokae_gazebo",
            executable="collision_test_node.py",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"contact_topic": contact_topic},
                {"traj_topic": traj_topic},
                {"publish_motion": publish_motion},
                {"joint_names": joint_names},
                {"robot_collision_substring": robot_collision_substring},
                {"filter_robot_contacts": filter_robot_contacts},
                {"publish_collision_signal": publish_collision_signal},
                {"collision_signal_topic": collision_signal_topic},
                {"collision_signal_hz": collision_signal_hz},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="CR7"),
        DeclareLaunchArgument("publish_motion", default_value="false"),
        DeclareLaunchArgument("contact_topic", default_value="/obstacle/bumper_contact"),
        DeclareLaunchArgument("filter_robot_contacts", default_value="true"),
        DeclareLaunchArgument("robot_collision_substring", default_value="xMate"),
        DeclareLaunchArgument("publish_collision_signal", default_value="true"),
        DeclareLaunchArgument(
            "collision_signal_topic",
            default_value="/collision_test/contact_active",
        ),
        DeclareLaunchArgument("collision_signal_hz", default_value="10.0"),
        OpaqueFunction(function=_setup),
    ])
