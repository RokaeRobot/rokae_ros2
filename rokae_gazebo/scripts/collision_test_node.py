#!/usr/bin/env python3
"""订阅 bumper ContactsState；持续接触期间按频率发布碰撞信号，无接触时不发布。"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CollisionTestNode(Node):
    def __init__(self):
        super().__init__('collision_test_node')
        self.declare_parameter('contact_topic', '/obstacle/bumper_contact')
        self.declare_parameter('traj_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        )
        self.declare_parameter(
            'test_positions',
            [0.15, 0.45, -0.35, 0.0, 0.2, 0.0],
        )
        self.declare_parameter('publish_motion', True)
        self.declare_parameter('robot_collision_substring', 'xMateCR7')
        self.declare_parameter('filter_robot_contacts', True)
        self.declare_parameter('log_interval_sec', 1.0)
        self.declare_parameter('print_collision_names', True)
        # 无接触时不再打印“持续”日志；有接触时可按间隔打印
        self.declare_parameter('log_while_contact', True)
        # 是否在持续接触时发布 std_msgs/Bool(data=True)（无接触时不发布任何消息）
        self.declare_parameter('publish_collision_signal', True)
        self.declare_parameter('collision_signal_topic', '/collision_test/contact_active')
        self.declare_parameter('collision_signal_hz', 10.0)

        topic = self.get_parameter('contact_topic').get_parameter_value().string_value
        self._sub = self.create_subscription(ContactsState, topic, self._on_contact, 10)

        traj_topic = self.get_parameter('traj_topic').get_parameter_value().string_value
        self._pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        sig_topic = self.get_parameter('collision_signal_topic').get_parameter_value().string_value
        self._publish_collision_signal = bool(
            self.get_parameter('publish_collision_signal').get_parameter_value().bool_value
        )
        self._sig_pub = self.create_publisher(Bool, sig_topic, 10) if self._publish_collision_signal else None

        self._motion = self.get_parameter('publish_motion').get_parameter_value().bool_value
        self._joint_names = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)
        self._test_positions = list(self.get_parameter('test_positions').get_parameter_value().double_array_value)
        self._log_interval_sec = float(self.get_parameter('log_interval_sec').get_parameter_value().double_value)
        self._print_collision_names = bool(self.get_parameter('print_collision_names').get_parameter_value().bool_value)
        self._log_while_contact = bool(self.get_parameter('log_while_contact').get_parameter_value().bool_value)

        hz = float(self.get_parameter('collision_signal_hz').get_parameter_value().double_value)
        self._collision_publish_period_ns = max(1, int(1e9 / max(0.5, hz)))

        self._hits = 0
        self._last_log_time = self.get_clock().now()
        self._last_signal_time = self.get_clock().now()
        self._last_collision_pair = ("", "")
        self._contact_active = False
        self._timer = self.create_timer(2.0, self._tick_motion)

        self._robot_sub = (
            self.get_parameter('robot_collision_substring')
            .get_parameter_value().string_value
        )
        self._filter_robot = bool(
            self.get_parameter('filter_robot_contacts')
            .get_parameter_value().bool_value
        )

        extra = f'；碰撞信号: {sig_topic} (仅接触中发布 True，频率≈{hz:.1f} Hz)' if self._sig_pub else ''
        self.get_logger().info(
            f'碰撞监听: {topic}；轨迹话题: {traj_topic}{extra}'
        )

    def _pick_robot_contact(self, msg: ContactsState):
        """返回 (是否含机器人相关接触, collision1, collision2)。"""
        if len(msg.states) == 0:
            return False, "", ""
        for st in msg.states:
            c1 = getattr(st, "collision1_name", "") or getattr(st, "collision1", "")
            c2 = getattr(st, "collision2_name", "") or getattr(st, "collision2", "")
            if (not self._filter_robot) or (self._robot_sub in c1 or self._robot_sub in c2):
                return True, c1, c2
        return False, "", ""

    def _on_contact(self, msg: ContactsState):
        self._hits += len(msg.states)
        active, c1, c2 = self._pick_robot_contact(msg)
        now = self.get_clock().now()

        if not active:
            self._contact_active = False
            self._last_collision_pair = ("", "")
            return

        self._contact_active = True
        pair = (c1, c2)
        pair_changed = pair != self._last_collision_pair
        self._last_collision_pair = pair

        extra = ""
        if self._print_collision_names and c1 and c2:
            extra = f"；碰撞体: [{c1}] <-> [{c2}]"

        if pair_changed:
            self.get_logger().warning(
                f'检测到碰撞接触开始 (本帧 states={len(msg.states)}, 累计采样={self._hits}){extra}'
            )
        elif self._log_while_contact:
            if (now - self._last_log_time).nanoseconds >= self._log_interval_sec * 1e9:
                self._last_log_time = now
                self.get_logger().warning(
                    f'碰撞接触持续中 (本帧 states={len(msg.states)}, 累计采样={self._hits}){extra}'
                )

        if self._sig_pub is not None:
            if (now - self._last_signal_time).nanoseconds >= self._collision_publish_period_ns:
                self._last_signal_time = now
                self._sig_pub.publish(Bool(data=True))

    def _tick_motion(self):
        if not self._motion:
            return
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        pt = JointTrajectoryPoint()
        pt.positions = self._test_positions[: len(self._joint_names)]
        if len(pt.positions) < len(self._joint_names):
            pt.positions.extend([0.0] * (len(self._joint_names) - len(pt.positions)))
        pt.time_from_start.sec = 1
        traj.points = [pt]
        traj.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(traj)


def main():
    rclpy.init()
    node = CollisionTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
