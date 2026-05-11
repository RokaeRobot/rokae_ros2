from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import re


def launch_setup(context, *args, **kwargs):
	robot_type = LaunchConfiguration("robot_type").perform(context)

	supported_types = [
		"CR7", "CR12", "CR18", "CR20", "CR35",
		"ER3", "ER7", "SR3", "SR4", "SR5",
		"Pro3", "Pro7", "AR5L", "AR5R",
	]
	if robot_type not in supported_types:
		raise RuntimeError(
			f"Unsupported robot type: {robot_type}. Supported types: {supported_types}"
		)

	description_pkg = get_package_share_directory("rokae_description")
	urdf_path = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
	if not os.path.exists(urdf_path):
		raise RuntimeError(f"URDF file not found: {urdf_path}")

	moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
	try:
		moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)
	except Exception as exc:
		raise RuntimeError(
			f"MoveIt configuration package not found: {moveit_config_pkg_name}"
		) from exc

	srdf_file = os.path.join(moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf")
	if not os.path.exists(srdf_file):
		raise RuntimeError(f"SRDF file not found: {srdf_file}")

	kinematics_yaml = os.path.join(moveit_config_pkg_share, "config", "kinematics.yaml")
	if not os.path.exists(kinematics_yaml):
		raise RuntimeError(f"Kinematics YAML file not found: {kinematics_yaml}")

	with open(srdf_file, "r", encoding="utf-8") as f:
		srdf_content = f.read()

	# URDF 根节点固定为 xMate_robot，SRDF 需保持同名避免 MoveIt 语义模型报错。
	semantic_content = re.sub(
		r'<robot\s+name\s*=\s*"[^"]+"',
		r'<robot name="xMate_robot"',
		srdf_content,
		count=1,
	)

	robot_description = {
		"robot_description": Command([
			"xacro ",
			urdf_path,
			" robot_type:=",
			robot_type,
		])
	}

	robot_description_semantic = {
		"robot_description_semantic": semantic_content
	}

	follow_joint_position_node = Node(
		package="rokae_example",
		executable="follow_joint_position",
		name="follow_joint_position",
		output="screen",
		parameters=[
			robot_description,
			robot_description_semantic,
			kinematics_yaml,
			{
				"robot_type": LaunchConfiguration("robot_type"),
				"point_period_s": LaunchConfiguration("point_period_s"),
				"start_blend_s": LaunchConfiguration("start_blend_s"),
				"goal_tolerance_rad": LaunchConfiguration("goal_tolerance_rad"),
				"vel_scale": LaunchConfiguration("vel_scale"),
				"acc_scale": LaunchConfiguration("acc_scale"),
				"cycle_count": LaunchConfiguration("cycle_count"),
			},
		],
	)

	return [follow_joint_position_node]


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			"robot_type",
			default_value="CR7",
			description="Robot type: CR7/CR12/CR18/CR20/CR35/ER3/ER7/SR3/SR4/SR5/Pro3/Pro7/AR5L/AR5R",
		),
		DeclareLaunchArgument("point_period_s", default_value="0.6", description="相邻轨迹点时间间隔（秒）"),
		DeclareLaunchArgument("start_blend_s", default_value="0.6", description="首点预留平滑时间（秒）"),
		DeclareLaunchArgument("goal_tolerance_rad", default_value="0.01", description="关节目标容差（弧度）"),
		DeclareLaunchArgument("vel_scale", default_value="0.2", description="MoveIt 速度缩放(0,1]"),
		DeclareLaunchArgument("acc_scale", default_value="0.2", description="MoveIt 加速度缩放(0,1]"),
		DeclareLaunchArgument("cycle_count", default_value="0", description="往复循环次数，0 表示无限循环"),
		OpaqueFunction(function=launch_setup),
	])
