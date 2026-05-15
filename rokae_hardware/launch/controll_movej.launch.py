from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import os
import re
import yaml
import xacro
import xml.etree.ElementTree as ET
from pathlib import Path


def _extract_joint_names(controllers_cfg):
    """Extract joint names from the most relevant controller block."""
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


def _resolve_model_base_name(robot_type):
    """Map robot_type to actual model filename prefix."""
    ar5_name_map = {
        "AR5L": "AR5-5_07L-W4C4A2",
        "AR5R": "AR5-5_07R-W4C4A2",
    }
    return ar5_name_map.get(robot_type, f"xMate{robot_type}")


def _resolve_urdf_paths(description_pkg, robot_type):
    """Resolve URDF path and whether a Gazebo-specialized file exists."""
    model_base = _resolve_model_base_name(robot_type)
    gazebo_candidates = [
        os.path.join(description_pkg, "urdf", f"{model_base}_Gazebo.urdf.xacro"),
        os.path.join(description_pkg, "urdf", f"{model_base}.urdf.xacro_Gazebo.urdf.xacro"),
    ]
    default_candidate = os.path.join(description_pkg, "urdf", f"{model_base}.urdf.xacro")

    gazebo_urdf_path = next((p for p in gazebo_candidates if os.path.exists(p)), "")
    urdf_path = gazebo_urdf_path if gazebo_urdf_path else default_candidate
    return urdf_path, gazebo_urdf_path


def _resolve_entity_name(robot_type):
    """Use model-specific entity names for clearer Gazebo scene labels."""
    ar5_entity_map = {
        "AR5L": "AR5-5_07L-W4C4A2",
        "AR5R": "AR5-5_07R-W4C4A2",
    }
    return ar5_entity_map.get(robot_type, f"xMate{robot_type}")


def _inject_gazebo_blocks(robot_desc_xml, robot_type, controllers_param_path, joint_names):
    """
    Inject CR7-like Gazebo tags and gazebo_ros2_control blocks for non-CR7 models.
    """
    root = ET.fromstring(robot_desc_xml)

    # Legacy URDFs often include an internal world link + fixed joint.
    # Gazebo spawn_entity already places the model in world, so keep model-local tree only.
    for link in list(root.findall("link")):
        if link.attrib.get("name") == "world":
            root.remove(link)
    for joint in list(root.findall("joint")):
        if joint.attrib.get("name") == "fixed":
            parent = joint.find("parent")
            if parent is not None and parent.attrib.get("link") == "world":
                root.remove(joint)

    for joint_name in joint_names:
        gz_joint = ET.SubElement(root, "gazebo", {"reference": joint_name})
        ET.SubElement(gz_joint, "kp").text = "1000.0"
        ET.SubElement(gz_joint, "kd").text = "100.0"
        ET.SubElement(gz_joint, "mu1").text = "0.1"
        ET.SubElement(gz_joint, "mu2").text = "0.1"
        ET.SubElement(gz_joint, "friction1").text = "0.0"
        ET.SubElement(gz_joint, "friction2").text = "0.0"

    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        if not link_name or link_name == "world":
            continue
        gz_link = ET.SubElement(root, "gazebo", {"reference": link_name})
        ET.SubElement(gz_link, "material").text = "Gazebo/Blue"
        collision = ET.SubElement(gz_link, "collision")
        ET.SubElement(collision, "laser_retro").text = "0.5"

    # Some legacy models contain inconsistent transmission joint names.
    # Rebuild transmissions from controller joint list to keep gazebo_ros2_control stable.
    for old_transmission in list(root.findall("transmission")):
        root.remove(old_transmission)
    for idx, joint_name in enumerate(joint_names, start=1):
        transmission = ET.SubElement(root, "transmission", {"name": f"tran{idx}"})
        ET.SubElement(transmission, "type").text = "transmission_interface/SimpleTransmission"
        transmission_joint = ET.SubElement(transmission, "joint", {"name": joint_name})
        ET.SubElement(transmission_joint, "hardwareInterface").text = "PositionJointInterface"
        actuator = ET.SubElement(transmission, "actuator", {"name": f"motor{idx}"})
        ET.SubElement(actuator, "hardwareInterface").text = "PositionJointInterface"
        ET.SubElement(actuator, "mechanicalReduction").text = "1"

    ros2_control = ET.SubElement(
        root,
        "ros2_control",
        {"name": f"xMate{robot_type}_system", "type": "system"},
    )
    hardware = ET.SubElement(ros2_control, "hardware")
    ET.SubElement(hardware, "plugin").text = "gazebo_ros2_control/GazeboSystem"
    ET.SubElement(hardware, "param", {"name": "use_sim_time"}).text = "true"

    for joint_name in joint_names:
        joint = ET.SubElement(ros2_control, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {"name": "gazebo_ros2_control", "filename": "libgazebo_ros2_control.so"},
    )
    ET.SubElement(plugin, "robot_param").text = "robot_description"
    ET.SubElement(plugin, "robot_param_node").text = "robot_state_publisher"
    ET.SubElement(plugin, "parameters").text = controllers_param_path

    return ET.tostring(root, encoding="unicode")


def _strip_xml_comments_for_gazebo_ros2(robot_desc_xml: str) -> str:
    """Strip <!-- --> so gazebo_ros2_control / rcl param parsing is not tripped."""
    return re.sub(r"<!--(.*?)-->", "", robot_desc_xml, flags=re.DOTALL)


def _resolve_gazebo_controller_params_path(robot_desc_xml, controllers_yaml_path):
    abs_path = os.path.abspath(controllers_yaml_path)
    return re.sub(
        r"(<parameters>)([^<]*controllers\.yaml)(</parameters>)",
        rf"\1{abs_path}\3",
        robot_desc_xml,
        count=1,
    )


def launch_setup(context, *args, **kwargs):
    from launch_ros.substitutions import FindPackageShare

    robot_type = LaunchConfiguration("robot_type").perform(context)
    gazebo_world_file = LaunchConfiguration("gazebo_world_file").perform(context)
    enable_gui = LaunchConfiguration("enable_gui").perform(context).lower() in ("1", "true", "yes")
    enable_movej = LaunchConfiguration("enable_movej").perform(context).lower() in ("1", "true", "yes")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout").perform(context)
    service_call_timeout = LaunchConfiguration("service_call_timeout").perform(context)

    supported_types = [
        "CR7", "CR12", "CR18", "CR20", "CR35",
        "ER3", "ER7", "SR3", "SR4", "SR5",
        "Pro3", "Pro7", "AR5L", "AR5R",
    ]
    if robot_type not in supported_types:
        raise RuntimeError(f"Unsupported robot type: {robot_type}")

    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    urdf_path, gazebo_urdf_path = _resolve_urdf_paths(description_pkg, robot_type)

    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF not found: {urdf_path}")

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
    trajectory_controller_name = _select_trajectory_controller_name(controllers_cfg)
    trajectory_topic = f"/{trajectory_controller_name}/joint_trajectory"
    follow_joint_trajectory_action = (
        f"/{trajectory_controller_name}/follow_joint_trajectory"
    )

    # Gazebo Classic 对 package:// mesh URI 解析经常失败；这里强制用文件路径 URI
    mesh_prefix = (Path(description_pkg) / "meshes").as_uri()
    robot_desc_xml = xacro.process_file(urdf_path, mappings={"mesh_prefix": mesh_prefix}).toxml()
    if not os.path.exists(gazebo_urdf_path):
        robot_desc_xml = _inject_gazebo_blocks(
            robot_desc_xml=robot_desc_xml,
            robot_type=robot_type,
            controllers_param_path=controllers_yaml_path,
            joint_names=joint_names,
        )
    else:
        robot_desc_xml = _resolve_gazebo_controller_params_path(
            robot_desc_xml, controllers_yaml_path
        )
    robot_desc_xml = _strip_xml_comments_for_gazebo_ros2(robot_desc_xml)
    moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
    try:
        moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)
    except Exception:
        raise RuntimeError(f"MoveIt package not found: {moveit_config_pkg_name}")

    srdf_file = os.path.join(moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf")
    kinematics_yaml = os.path.join(moveit_config_pkg_share, "config", "kinematics.yaml")

    with open(kinematics_yaml, "r", encoding="utf-8") as f:
        kinematics_params = yaml.safe_load(f)

    robot_description = {
        "robot_description": robot_desc_xml
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]), value_type=str
        )
    }

    movej_params = [robot_description, robot_description_semantic, kinematics_params]

    gazebo_ros_share_dir = get_package_share_directory("gazebo_ros")
    rokae_gazebo_share_dir = get_package_share_directory("rokae_gazebo")
    world_path = os.path.join(rokae_gazebo_share_dir, "worlds", gazebo_world_file)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "extra_gazebo_args": "--verbose",
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}]
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", _resolve_entity_name(robot_type),
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
        output="screen"
    )

    # 自动加载并激活控制器
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
            "--service-call-timeout", service_call_timeout,
        ],
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            trajectory_controller_name,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
            "--service-call-timeout", service_call_timeout,
        ],
    )

    # 顺序：先 joint_state_broadcaster，再 arm_controller
    delayed_joint_state_spawner = TimerAction(
        period=2.0,
        actions=[spawn_joint_state_broadcaster],
    )
    delayed_arm_spawner = TimerAction(
        period=4.5,
        actions=[spawn_arm_controller],
    )

    actions = [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state_spawner,
        delayed_arm_spawner,
    ]

    if enable_movej:
        movej_node = Node(
            package="rokae_hardware",
            executable="movej",
            name="movej",
            parameters=movej_params,
        )
        # MoveIt demo node starts after controllers are expected to be active.
        delayed_movej = TimerAction(period=7.0, actions=[movej_node])
        actions.append(delayed_movej)

    # ======================== 可选：GUI 控制节点 ========================
    if enable_gui:
        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[
                {"robot_description": robot_desc_xml},
                {"use_sim_time": True},
            ],
            remappings=[("/joint_states", "/joint_states_gui")]
        )

        gui_control_node = Node(
            package="rokae_description",
            executable="gui_to_joint_trajectory.py",
            name="gui_to_joint_trajectory",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"joint_names": joint_names},
                {"trajectory_topic": trajectory_topic},
                {"follow_joint_trajectory_action": follow_joint_trajectory_action},
                {"use_follow_joint_trajectory_action": True},
            ]
        )

        delayed_gui = TimerAction(
            period=6.0,
            actions=[joint_state_publisher_gui, gui_control_node],
        )
        actions.append(delayed_gui)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type",
            default_value="CR7",
            description=(
                "机型后缀；需存在 rokae_description/urdf/xMate{TYPE}_Gazebo.urdf.xacro 或 .urdf.xacro，"
                "及 rokae_xMate{TYPE}_moveit_config、config/xMate{TYPE}_controllers.yaml。例 CR35、SR3。"
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="empty.world",
            description="World file under rokae_gazebo/worlds/",
        ),
        DeclareLaunchArgument(
            "enable_gui",
            default_value="false",
            description="Whether to start GUI joint sliders and auto-publish trajectories.",
        ),
        DeclareLaunchArgument(
            "enable_movej",
            default_value="false",
            description="Whether to start MoveIt demo node 'movej'.",
        ),
        DeclareLaunchArgument("controller_manager_timeout", default_value="120"),
        DeclareLaunchArgument("service_call_timeout", default_value="120"),
        OpaqueFunction(function=launch_setup)
    ])
