from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_rokae_description = get_package_share_directory('rokae_description')
    pkg_rokae_gazebo = get_package_share_directory('rokae_gazebo')

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
            os.path.join(pkg_rokae_gazebo, 'worlds', 'obstacles.world'),
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so'
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

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/joint_states', '/joint_states_gui')]
    )

    gui_control_node = Node(
        package="rokae_description",
        executable="gui_to_joint_trajectory.py",
        name="gui_to_joint_trajectory",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # ======================== 【这里是我帮你加的：场景服务节点】 ========================
    scene_manager_node = Node(
        package="rokae_hardware",
        executable="scene_service",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # ✅ 延迟启动 GUI
    delayed_gui = TimerAction(
        period=5.0,
        actions=[joint_state_publisher_gui, gui_control_node]
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        TimerAction(period=3.0, actions=[joint_state_broadcaster]),
        TimerAction(period=4.0, actions=[arm_controller]),
        delayed_gui,
        scene_manager_node   # <--- 已加入！
    ])