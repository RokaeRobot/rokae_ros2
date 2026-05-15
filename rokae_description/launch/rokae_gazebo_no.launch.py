from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_rokae_description = get_package_share_directory('rokae_description')

    gazebo_model_path = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'export GAZEBO_MODEL_PATH=' + os.path.join(pkg_rokae_description, 'models') +
            ':$GAZEBO_MODEL_PATH'
        ],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            os.path.join('/usr/share/gazebo-11/worlds', 'empty.world'),
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    xacro_file = os.path.join(pkg_rokae_description, 'urdf', 'xMateCR7_Gazebo.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
        ]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'xMateCR7',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    controller_yaml = os.path.join(pkg_rokae_description, 'config', 'controllers.yaml')

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ===================== 【新增：关节拖动 GUI】 =====================
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # 关键：把 GUI 的话题重映射到 /joint_states_gui
        remappings=[
            ('/joint_states', '/joint_states_gui')
        ]
    )

    # 【新增：GUI 转轨迹控制节点】
    gui_control_node = Node(
        package="rokae_description",
        executable="gui_to_joint_trajectory.py",
        name="gui_to_joint_trajectory",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

     # 场景管理服务节点（任务2.2）
    scene_manager_node = Node(
        package="rokae_hardware",
        executable="scene_service",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_publisher_gui,  # <-- 启动GUI
        gui_control_node, 
        scene_manager_node,  # <-- 加在这里

        TimerAction(period=3.0, actions=[joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[arm_controller])
    ])


# shangjialu 03/19