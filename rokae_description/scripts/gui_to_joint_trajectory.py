#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Time

class GuiToTrajectory(Node):
    def __init__(self):
        super().__init__("gui_to_joint_trajectory")

        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "trajectory_topic",
            "/arm_controller/joint_trajectory",
        )
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)

        # ✅ 订阅 GUI 专属话题，避免和 joint_state_broadcaster 冲突
        self.sub = self.create_subscription(
            JointState,
            "/joint_states_gui",
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            self.trajectory_topic,
            10
        )

        # ✅ 加发布频率限制，避免刷屏
        self.last_pub_time = self.get_clock().now()
        self.pub_interval = rclpy.duration.Duration(seconds=0.1)  # 10Hz

        self.get_logger().info(
            f"GUI 转轨迹控制器已启动: topic={self.trajectory_topic}, joints={self.joint_names}"
        )

    def callback(self, msg: JointState):
        try:
            # ✅ 频率限制：100ms 发一次，避免控制器过载
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
                    positions[idx] = msg.position[i]

            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            # ✅ 给 header 加上当前时间戳
            traj.header.stamp = self.get_clock().now().to_msg()

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = 1
            point.time_from_start.nanosec = 0
            traj.points = [point]

            self.pub.publish(traj)
            self.get_logger().debug(f"发布轨迹: {positions}")

        except Exception as e:
            self.get_logger().error(f"错误：{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = GuiToTrajectory()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()



# shangjialu 03/19