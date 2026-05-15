import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    """加载 YAML 文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    local_ip = LaunchConfiguration("local_ip").perform(context)
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true", "yes")
    # Mock (GenericSystem) in rokae_moveit_launch: no /clock — MoveIt must use wall time.
    if use_fake_hardware.lower() in ("1", "true", "yes"):
        use_sim_time = False
    # Real Rokae: joint_states on system clock.
    elif robot_type.startswith(("CR", "AR", "ER", "Pro", "SR")) and use_fake_hardware.lower() in ("0", "false", "no"):
        use_sim_time = False

    # description 包
    description_pkg = get_package_share_directory("rokae_description")
    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")

    # moveit_config 包名和 share
    moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
    moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)

    # SRDF 文件
    srdf_file = os.path.join(
        moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf"
    )

    # robot_description
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", urdf_file,
                " robot_type:=", robot_type,
                " robot_ip:=", robot_ip,
                " local_ip:=", local_ip,
                # " use_fake_hardware:=true"
                " use_fake_hardware:=",use_fake_hardware,
            ]),
            value_type=str
        )
    }

    # robot_description_semantic
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]),
            value_type=str
        )
    }

    # kinematics
    robot_description_kinematics = os.path.join(
        moveit_config_pkg_share, "config", "kinematics.yaml"
    )

    # OMPL 配置
    ompl_yaml = load_yaml(moveit_config_pkg_name, "config/ompl_planning.yaml")
    ompl_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/ResolveConstraintFrames "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    if ompl_yaml:
        ompl_pipeline_config["move_group"].update(ompl_yaml)

    # 控制器
    controllers_yaml = load_yaml(moveit_config_pkg_name, "config/simple_moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    joint_limits_yaml = os.path.join(moveit_config_pkg_share, "config", "joint_limits.yaml")
    
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "execution_duration_monitoring": True,
        "trajectory_execution.wait_for_trajectory_completion": True,
        # 添加状态更新配置
        "trajectory_execution.update_state_after_execution": True,
        "trajectory_execution.update_state_before_execution": True,
        "execution_duration_monitoring": False,
        "manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "velocity_scaling_factor": 0.01,    # 默认 1.0，调低到 0.2 就是 20% 速度
        "acceleration_scaling_factor": 0.01
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


    # MoveIt move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            ompl_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz 节点
    rviz_config_file = os.path.join(moveit_config_pkg_share, "rviz", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description, 
            robot_description_semantic,
            robot_description_kinematics,
            ompl_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            ],
        # prefix="gdb -ex run --args",
    )
    
    movej_node = Node(
        package="rokae_hardware",
        executable="movej",
        name="movej",
        parameters=[
            robot_description, 
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            ompl_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            ]
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type",
            default_value="CR7",
            description="Robot type suffix for rokae_xMate{robot_type}_moveit_config and xMate{robot_type}.srdf (e.g. CR7, CR35, SR4).",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.21.10",
            description="Robot controller IP.",
        ),
        DeclareLaunchArgument(
            "local_ip",
            default_value="192.168.21.131",
            description="This PC IP on the robot subnet.",
        ),
        DeclareLaunchArgument("warehouse_sqlite_path", default_value="", description="Warehouse path"),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Forwarded to xacro (mock vs real hardware plugin).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time only when /clock exists (Gazebo or bag).",
        ),
        OpaqueFunction(function=launch_setup)
    ])
