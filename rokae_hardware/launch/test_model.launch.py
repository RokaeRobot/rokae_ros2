from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import re
import yaml
import xacro
import xml.etree.ElementTree as ET
from pathlib import Path


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


def _inject_gazebo_blocks(robot_desc_xml, robot_type, controllers_param_path, joint_names):
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
    """gazebo_ros2_control may pass URDF through rcl param rules; ':' + space in
    <!-- comments --> can be misparsed (see gz_ros2_control #503). Strip comments."""
    return re.sub(r"<!--(.*?)-->", "", robot_desc_xml, flags=re.DOTALL)


def _resolve_gazebo_controller_params_path(robot_desc_xml, controllers_yaml_path):
    """Gazebo 插件需绝对路径加载控制器 YAML（与 CR7 一致）。"""
    abs_path = os.path.abspath(controllers_yaml_path)
    return re.sub(
        r"(<parameters>)([^<]*controllers\.yaml)(</parameters>)",
        rf"\1{abs_path}\3",
        robot_desc_xml,
        count=1,
    )


def launch_setup(context, *args, **kwargs):
    logger = get_logger("test_model.launch")
    robot_type = LaunchConfiguration("robot_type").perform(context)
    gazebo_world_file = LaunchConfiguration("gazebo_world_file").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    gui = LaunchConfiguration("gui").perform(context)

    supported_types = [
        "CR7", "CR12", "CR18", "CR20", "CR35",
        "ER3", "ER7", "SR3", "SR4", "SR5",
        "Pro3", "Pro7", "AR5L", "AR5R",
    ]
    if robot_type not in supported_types:
        raise RuntimeError(f"Unsupported robot type: {robot_type}")

    description_pkg = get_package_share_directory("rokae_description")
    custom_gazebo_urdf_names = {
        "AR5L": "AR5-5_07L-W4C4A2.urdf.xacro_Gazebo.urdf.xacro",
        "AR5R": "AR5-5_07R-W4C4A2.urdf.xacro_Gazebo.urdf.xacro",
    }
    gazebo_urdf_filename = custom_gazebo_urdf_names.get(
        robot_type, f"xMate{robot_type}_Gazebo.urdf.xacro"
    )
    gazebo_urdf_path = os.path.join(description_pkg, "urdf", gazebo_urdf_filename)
    use_model_gazebo_xacro = os.path.exists(gazebo_urdf_path)

    if use_model_gazebo_xacro:
        urdf_path = gazebo_urdf_path
    else:
        urdf_path = os.path.join(description_pkg, "urdf", f"xMate{robot_type}.urdf.xacro")
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

    mesh_prefix = (Path(description_pkg) / "meshes").as_uri()
    robot_desc_xml = xacro.process_file(urdf_path, mappings={"mesh_prefix": mesh_prefix}).toxml()
    if not use_model_gazebo_xacro:
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
    logger.info(
        f"robot_type={robot_type}, use_model_gazebo_xacro={use_model_gazebo_xacro}, urdf_path={urdf_path}"
    )

    gazebo_ros_share_dir = get_package_share_directory("gazebo_ros")
    rokae_gazebo_share_dir = get_package_share_directory("rokae_gazebo")
    world_path = os.path.join(rokae_gazebo_share_dir, "worlds", gazebo_world_file)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "gui": gui,
            "extra_gazebo_args": "--verbose",
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc_xml}, {"use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", f"xMate{robot_type}",
            "-x", "0.0", "-y", "0.0", "-z", spawn_z,
        ],
        output="screen",
    )

    # Load controllers required for joint motion validation.
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
        ],
        output="screen",
    )
    spawn_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            trajectory_controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "120",
        ],
        output="screen",
    )

    delayed_joint_state_spawner = TimerAction(
        period=2.0,
        actions=[spawn_joint_state_broadcaster],
    )
    delayed_trajectory_spawner = TimerAction(
        period=4.5,
        actions=[spawn_trajectory_controller],
    )

    actions = [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state_spawner,
        delayed_trajectory_spawner,
    ]

    if gui.lower() in ("true", "1", "yes"):
        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            parameters=[{"robot_description": robot_desc_xml}, {"use_sim_time": True}],
            remappings=[("/joint_states", "/joint_states_gui")],
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
            ],
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
                "机型后缀，对应 urdf/xMate{TYPE}_*.xacro 与 "
                "rokae_hardware/config/xMate{TYPE}_controllers.yaml。例 CR35、CR7、SR3、Pro3。"
            ),
        ),
        DeclareLaunchArgument(
            "gazebo_world_file",
            default_value="empty.world",
            description=(
                "rokae_gazebo/worlds 下文件名（empty.world、obstacles.world）或绝对路径；与机型无关。"
            ),
        ),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
