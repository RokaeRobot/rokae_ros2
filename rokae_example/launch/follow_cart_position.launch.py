from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def launch_setup(context, *args, **kwargs):
	robot_type = LaunchConfiguration("robot_type").perform(context)

	supported_types = [
		"CR7", "CR12", "CR18", "CR20",
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

	with open(kinematics_yaml, "r", encoding="utf-8") as f:
		kinematics_params = yaml.safe_load(f)

	robot_description = {
		"robot_description": Command([
			"xacro ",
			urdf_path,
			" robot_type:=",
			robot_type,
		])
	}

	robot_description_semantic = {
		"robot_description_semantic": ParameterValue(
			Command(["cat ", srdf_file]),
			value_type=str,
		)
	}

	follow_cart_position_node = Node(
		package="rokae_example",
		executable="follow_cart_position",
		name="follow_cart_position",
		output="screen",
		parameters=[
			robot_description,
			robot_description_semantic,
			kinematics_params,
			{
				"amplitude_m": LaunchConfiguration("amplitude_m"),
				"period_s": LaunchConfiguration("period_s"),
				"target_update_hz": LaunchConfiguration("target_update_hz"),
				"control_hz": LaunchConfiguration("control_hz"),
				"kp": LaunchConfiguration("kp"),
				"filter_alpha": LaunchConfiguration("filter_alpha"),
				"duration_s": LaunchConfiguration("duration_s"),
				"vel_scale": LaunchConfiguration("vel_scale"),
				"acc_scale": LaunchConfiguration("acc_scale"),
				"eef_step": LaunchConfiguration("eef_step"),
				"min_fraction": LaunchConfiguration("min_fraction"),
			},
		],
	)

	return [follow_cart_position_node]


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument(
			"robot_type",
			default_value="CR7",
			description="Robot type: CR7/CR12/CR18/CR20/ER3/ER7/SR3/SR4/SR5/Pro3/Pro7/AR5L/AR5R",
		),
		DeclareLaunchArgument("amplitude_m", default_value="0.4", description="Y轴正弦摆动振幅（米）"),
		DeclareLaunchArgument("period_s", default_value="1.33", description="正弦周期（秒）"),
		DeclareLaunchArgument("target_update_hz", default_value="1.0", description="目标点更新频率（Hz）"),
		DeclareLaunchArgument("control_hz", default_value="5.0", description="控制步执行频率（Hz）"),
		DeclareLaunchArgument("kp", default_value="0.6", description="比例跟踪系数"),
		DeclareLaunchArgument("filter_alpha", default_value="0.25", description="一阶滤波系数(0,1]"),
		DeclareLaunchArgument("duration_s", default_value="20.0", description="跟随总时长（秒）"),
		DeclareLaunchArgument("vel_scale", default_value="0.08", description="速度缩放(0,1]"),
		DeclareLaunchArgument("acc_scale", default_value="0.08", description="加速度缩放(0,1]"),
		DeclareLaunchArgument("eef_step", default_value="0.002", description="笛卡尔路径插值步长（米）"),
		DeclareLaunchArgument("min_fraction", default_value="0.95", description="局部路径最小完成率"),
		OpaqueFunction(function=launch_setup),
	])
