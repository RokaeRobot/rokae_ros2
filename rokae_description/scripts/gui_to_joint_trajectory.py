#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Time


class GuiToTrajectory(Node):
    def __init__(self):
        super().__init__('gui_to_joint_trajectory')

        self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        )
        self.declare_parameter(
            'trajectory_topic',
            '/position_joint_trajectory_controller/joint_trajectory',
        )
        self.declare_parameter(
            'follow_joint_trajectory_action',
            '/position_joint_trajectory_controller/follow_joint_trajectory',
        )
        self.declare_parameter('use_follow_joint_trajectory_action', True)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self._trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        self._action_name = str(self.get_parameter('follow_joint_trajectory_action').value)
        self._use_action = bool(self.get_parameter('use_follow_joint_trajectory_action').value)

        self.sub = self.create_subscription(
            JointState,
            '/joint_states_gui',
            self.callback,
            10,
        )

        self._pub = None
        self._action_client = None
        if self._use_action:
            self._action_client = ActionClient(
                self, FollowJointTrajectory, self._action_name
            )
            self.get_logger().info(
                f'GUI->轨迹: FollowJointTrajectory ({self._action_name})'
            )
        else:
            self._pub = self.create_publisher(
                JointTrajectory, self._trajectory_topic, 10
            )
            self.get_logger().info(f'GUI->轨迹: 话题 {self._trajectory_topic}')

        self.last_pub_time = self.get_clock().now()
        self.pub_interval = rclpy.duration.Duration(seconds=0.1)
        self._warned_no_server = False

    def _build_trajectory(self, positions):
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        # 0 时间戳表示“立即执行”，避免与 /clock 对齐问题导致整段被判为“已过期”
        traj.header.stamp = Time(sec=0, nanosec=0)
        point = JointTrajectoryPoint()
        point.positions = positions
        n = len(self.joint_names)
        point.velocities = [0.0] * n
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        traj.points = [point]
        return traj

    def callback(self, msg: JointState):
        try:
            now = self.get_clock().now()
            if now - self.last_pub_time < self.pub_interval:
                return
            self.last_pub_time = now

            positions = [0.0] * len(self.joint_names)
            name_to_index = {name: idx for idx, name in enumerate(self.joint_names)}
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    continue
                idx = name_to_index.get(name)
                if idx is not None:
                    positions[idx] = float(msg.position[i])

            traj = self._build_trajectory(positions)

            if self._action_client is not None:
                if not self._action_client.wait_for_server(timeout_sec=0.0):
                    if not self._warned_no_server:
                        self.get_logger().warn(
                            f'FollowJointTrajectory 未就绪: {self._action_name}'
                            '（等待 joint_trajectory_controller 激活）'
                        )
                        self._warned_no_server = True
                    return
                self._warned_no_server = False
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = traj
                send_future = self._action_client.send_goal_async(goal)
                send_future.add_done_callback(self._on_goal_sent)
            elif self._pub is not None:
                self._pub.publish(traj)
        except Exception as e:
            self.get_logger().error(f'gui_to_joint_trajectory: {e}')

    def _on_goal_sent(self, future):
        try:
            goal_handle = future.result()
            if goal_handle is None:
                return
            if not goal_handle.accepted:
                self.get_logger().warning(
                    'FollowJointTrajectory 目标被拒绝（检查控制器是否为 active）'
                )
        except Exception as e:
            self.get_logger().error(f'发送 FollowJointTrajectory 失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GuiToTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
