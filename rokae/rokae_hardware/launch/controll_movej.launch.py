# # movej.launch.py

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import Command
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.actions import DeclareLaunchArgument, OpaqueFunction
# from launch.substitutions import (
#     Command,
#     FindExecutable,
#     LaunchConfiguration,
#     PathJoinSubstitution,
# )

# def launch_setup(context, *args, **kwargs):
#     from launch_ros.substitutions import FindPackageShare
#     import os
    
#     # -------- 新增：选择机型参数 --------
#     robot_type_arg = DeclareLaunchArgument(
#         "robot_type",
#         default_value="CR7",
#         description="Robot type: CR7 or SR4"
#     )
    
#     # 声明launch参数，提供默认IP（根据你实际情况改）
#     robot_type = LaunchConfiguration("robot_type").perform(context)

#     description_pkg = FindPackageShare("rokae_description").find("rokae_description")
#     #moveit_config_pkg = FindPackageShare("rokae_moveit_config").find("rokae_xMateSR4_moveit_config")

#     urdf_path = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
#     #srdf_path = os.path.join(moveit_config_pkg, "config", "xMateSR4.srdf")
    
#     # moveit_config 包名和 share
#     moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
#     moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)

#     # SRDF 文件
#     srdf_file = os.path.join(
#         moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf"
#     )
    

#         # 从xacro文件中获取的参数
#     robot_description = {
#         "robot_description": Command([
#             "xacro ", urdf_path,
#             " robot_type:=", robot_type    #这里把机型传入 xacro
#         ])
#     }
#         # robot_description_semantic
#     robot_description_semantic = {
#         "robot_description_semantic": ParameterValue(
#             Command(["cat ", srdf_file]),
#             value_type=str
#         )
#     }

#     # kinematics
#     robot_description_kinematics = os.path.join(
#         moveit_config_pkg_share, "config", "kinematics.yaml"
#     )

#     movej_node = Node(
#         package="rokae_hardware",
#         executable="movej",
#         name="movej",
#         parameters=[
#             robot_description, 
#             robot_description_semantic,
#             robot_description_kinematics,
#             ],
#         # prefix="xterm -e gdb -ex run --args",
#         # prefix = "valgrind --leak-check=full --show-leak-kinds=all"
#     )

#     return [movej_node]


# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument("robot_type", default_value="CR7", description="Robot type: SR4/ER7/..."),
#         OpaqueFunction(function=launch_setup)
#     ])

# movej.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import os
import yaml

def launch_setup(context, *args, **kwargs):
    from launch_ros.substitutions import FindPackageShare
    
    # 获取机器人类型
    robot_type = LaunchConfiguration("robot_type").perform(context)
    
    # 验证机器人类型（根据你的实际支持类型）
    supported_types = ["CR7", "CR12","CR18", "CR20",
                       "ER3", "ER7", "SR3", "SR4", "SR5",
                       "Pro3", "Pro7","AR5L","AR5R"]  # 添加实际支持的型号
    if robot_type not in supported_types:
        raise RuntimeError(f"Unsupported robot type: {robot_type}. Supported types: {supported_types}")
    
    # 获取描述包路径
    description_pkg = FindPackageShare("rokae_description").find("rokae_description")
    urdf_path = os.path.join(description_pkg, "urdf", "xMate.urdf.xacro")
    
    # 验证URDF文件存在
    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF file not found: {urdf_path}")
    
    # 构建moveit配置包名并验证存在
    moveit_config_pkg_name = f"rokae_xMate{robot_type}_moveit_config"
    try:
        moveit_config_pkg_share = get_package_share_directory(moveit_config_pkg_name)
    except:
        raise RuntimeError(f"MoveIt configuration package not found: {moveit_config_pkg_name}")
    
    # SRDF文件路径
    srdf_file = os.path.join(moveit_config_pkg_share, "config", f"xMate{robot_type}.srdf")
    
    # 验证SRDF文件存在
    if not os.path.exists(srdf_file):
        raise RuntimeError(f"SRDF file not found: {srdf_file}")
    
    # kinematics.yaml文件路径
    kinematics_yaml = os.path.join(moveit_config_pkg_share, "config", "kinematics.yaml")
    
    # 验证kinematics文件存在
    if not os.path.exists(kinematics_yaml):
        raise RuntimeError(f"Kinematics YAML file not found: {kinematics_yaml}")
    
    # 读取kinematics.yaml内容
    with open(kinematics_yaml, 'r') as f:
        kinematics_params = yaml.safe_load(f)
    
    # robot_description参数
    robot_description = {
        "robot_description": Command([
            "xacro ", urdf_path,
            " robot_type:=", robot_type
        ])
    }
    
    # robot_description_semantic参数
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(["cat ", srdf_file]),
            value_type=str
        )
    }
    
    # 合并所有参数
    movej_params = [
        robot_description, 
        robot_description_semantic,
        kinematics_params  # 直接传递加载的YAML字典
    ]
    
    movej_node = Node(
        package="rokae_hardware",
        executable="movej",
        name="movej",
        parameters=movej_params,
        # 调试选项
        # prefix="xterm -e gdb -ex run --args",
        # prefix="valgrind --leak-check=full --show-leak-kinds=all"
    )
    
    return [movej_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type", 
            default_value="CR7", 
            description="Robot type: SR4/ER7/CR7/SR5 etc."
        ),
        OpaqueFunction(function=launch_setup)
    ])