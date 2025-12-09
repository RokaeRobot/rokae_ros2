from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            yaml_content = file.read()

        return yaml.safe_load(yaml_content)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    # 获取路径
    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    # 拼出 rokae_xMateCR7_moveit_config 包名
    moveit_config_pkg_name = PythonExpression([
        "'rokae_xMate' + '", robot_type, "' + '_moveit_config'"
    ])

    # 得到 rokae_xMateCR7_moveit_config 的 share 路径
    moveit_config_pkg_share = FindPackageShare(moveit_config_pkg_name)

    # 获取参数
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # URDF/xacro 路径
    urdf_file = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    srdf_file = PathJoinSubstitution([
        moveit_config_pkg_share,
        "config",
        PythonExpression([
        "'xMate' + '", robot_type, "' + '.srdf'"
        ])
    ])

    # robot_description 参数（xacro 动态传参）
    robot_description = {
        "robot_description": Command([
            " xacro ", urdf_file,
            #" xMate_type:=", PythonExpression(["'xMate_' + '", robot_type, "'"]),
            " robot_type:=", robot_type,       # 这里很关键
            " robot_ip:=", robot_ip,
            " local_ip:=", local_ip,
            " use_fake_hardware:=true"
        ])
    }
    
    # robot_description 参数（xacro 动态传参）
    robot_description_semantic = {
    "robot_description_semantic": ParameterValue(
        Command(["cat " ,  srdf_file]),   #加空格，否则cat本身会被当成字符串读到路径中产生报错
        value_type=str
    )
}
    # robot_description_semantic = {
    #     "robot_description_semantic": open(srdf_file).read()
    # }


    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("rokae_xMateCR7_moveit_config"), "config", "kinematics.yaml"]
    )

    # Planning Configuration
    ompl_planning_pipeline_config = {
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
    ompl_planning_yaml = load_yaml("rokae_xMateCR7_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    moveit_simple_controllers_yaml = load_yaml(
        "rokae_xMateCR7_moveit_config", "config/simple_moveit_controllers.yaml"
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    #moveit_controllers = load_yaml("rokae_xMateCR7_moveit_config", "config/simple_moveit_controllers.yaml")


    trajectory_execution = {
        "moveit_manage_controllers": False,
        # "execution_duration_monitoring": False,
        # "manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "rokae_xMateCR7_moveit_config", "config/joint_limits.yaml"
        )
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # 启动 move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            ompl_planning_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # warehouse_ros_config
        ],
    )
    

    # RViz可选节点（带 moveit config）


    rviz_config_file = PathJoinSubstitution([
        moveit_config_pkg_share,
        "rviz",
        "moveit.rviz"
    ])
    
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
            ompl_planning_yaml,
            ompl_planning_pipeline_config,
        ],
    )
    


    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.21.10"),
        DeclareLaunchArgument("local_ip", default_value="192.168.21.131"),

        move_group_node,
        rviz_node,
    ])
