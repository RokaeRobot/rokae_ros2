from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument("robot_ip", default_value="192.168.21.10", description="Robot controller IP"),
		DeclareLaunchArgument("local_ip", default_value="192.168.21.131", description="Local NIC IP"),
		DeclareLaunchArgument("movej_speed", default_value="0.2", description="MoveJ speed scale (0,1]"),
		DeclareLaunchArgument("cycle_ms", default_value="20", description="ServoJ command period in ms"),
		DeclareLaunchArgument("duration_s", default_value="30.0", description="Swing duration in seconds"),
		DeclareLaunchArgument("period_s", default_value="4.0", description="Cosine period in seconds"),
		DeclareLaunchArgument("servoj_lookahead_s", default_value="0.06", description="ServoJ lookahead time"),
		DeclareLaunchArgument("servoj_kp", default_value="1.0", description="ServoJ gain"),
		Node(
			package="rokae_hardware",
			executable="servoj_demo",
			name="servoj_demo",
			output="screen",
			parameters=[
				{
					"robot_ip": LaunchConfiguration("robot_ip"),
					"local_ip": LaunchConfiguration("local_ip"),
					"movej_speed": LaunchConfiguration("movej_speed"),
					"cycle_ms": LaunchConfiguration("cycle_ms"),
					"duration_s": LaunchConfiguration("duration_s"),
					"period_s": LaunchConfiguration("period_s"),
					"servoj_lookahead_s": LaunchConfiguration("servoj_lookahead_s"),
					"servoj_kp": LaunchConfiguration("servoj_kp"),
				},
			],
		),
	])
