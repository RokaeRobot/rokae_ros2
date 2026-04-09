// MoveIt 笛卡尔空间直线轨迹示范：沿 Z 轴负方向下降
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

const std::string kControllerName = "position_joint_trajectory_controller";

class CartesianSLineDemo : public rclcpp::Node
{
public:
	CartesianSLineDemo()
	: Node("cartesian_s_line_node")
	{
		controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
			"/controller_manager/switch_controller");
	}

	void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)
	{
		move_group_ = mg;
	}

	bool move_to_pre_position(const std::vector<double>& first_six_target)
	{
		if (!move_group_) {
			RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 未初始化");
			return false;
		}

		auto& arm = *move_group_;
		const auto current = arm.getCurrentJointValues();
		const size_t dof = current.size();

		if (dof < first_six_target.size()) {
			RCLCPP_ERROR(this->get_logger(), "自由度不足，期望至少 %zu，实际 %zu", first_six_target.size(), dof);
			return false;
		}

		auto target = current;
		for (size_t i = 0; i < first_six_target.size(); ++i) {
			target[i] = first_six_target[i];
		}

		arm.setPlanningTime(20.0);
		arm.allowReplanning(true);
		arm.setGoalPositionTolerance(0.01);
		arm.setGoalOrientationTolerance(0.01);
		arm.setMaxAccelerationScalingFactor(0.1);
		arm.setMaxVelocityScalingFactor(0.1);
		arm.setStartStateToCurrentState();
		arm.setJointValueTarget(target);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "预位姿规划失败");
			return false;
		}

		if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "预位姿执行失败");
			return false;
		}

		RCLCPP_INFO(this->get_logger(), "已到达预位姿");
		return true;
	}

	bool execute_cartesian_line_down(double z_down, double ds, double vel_scale, double acc_scale)
	{
		if (!move_group_) {
			RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 未初始化");
			return false;
		}

		if (z_down <= 0.0 || ds <= 0.0) {
			RCLCPP_ERROR(this->get_logger(), "参数非法，要求 z_down > 0 且 ds > 0");
			return false;
		}

		if (vel_scale <= 0.0 || vel_scale > 1.0 || acc_scale <= 0.0 || acc_scale > 1.0) {
			RCLCPP_ERROR(this->get_logger(), "参数非法，要求 0 < vel_scale/acc_scale <= 1");
			return false;
		}

		auto& arm = *move_group_;
		arm.setStartStateToCurrentState();

		const geometry_msgs::msg::Pose pose_1 = arm.getCurrentPose().pose;
		geometry_msgs::msg::Pose pose_2 = pose_1;
		pose_2.position.z -= z_down;

		const double dx = pose_2.position.x - pose_1.position.x;
		const double dy = pose_2.position.y - pose_1.position.y;
		const double dz = pose_2.position.z - pose_1.position.z;
		const double s = std::sqrt(dx * dx + dy * dy + dz * dz);

		if (s < 1e-6) {
			RCLCPP_ERROR(this->get_logger(), "直线长度过小，无法生成轨迹");
			return false;
		}

		const int steps = std::max(1, static_cast<int>(std::ceil(s / ds)));
		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.reserve(static_cast<size_t>(steps) + 1);

		for (int i = 0; i <= steps; ++i) {
			const double delta_s = std::min(i * ds, s);

			geometry_msgs::msg::Pose pose_cur = pose_1;
			pose_cur.position.x = pose_1.position.x + dx * delta_s / s;
			pose_cur.position.y = pose_1.position.y + dy * delta_s / s;
			pose_cur.position.z = pose_1.position.z + dz * delta_s / s;
			waypoints.push_back(pose_cur);
		}

		moveit_msgs::msg::RobotTrajectory traj_msg;
		const double eef_step = std::max(0.001, ds * 0.5);
		const double jump_threshold = 0.0;

		const double fraction = arm.computeCartesianPath(
			waypoints,
			eef_step,
			jump_threshold,
			traj_msg,
			true);

		if (fraction < 0.99) {
			RCLCPP_ERROR(this->get_logger(), "笛卡尔路径规划不完整，完成率: %.3f", fraction);
			return false;
		}

		robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
		rt.setRobotTrajectoryMsg(*arm.getCurrentState(), traj_msg);

		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		const bool timed_ok = iptp.computeTimeStamps(rt, vel_scale, acc_scale);
		if (!timed_ok) {
			RCLCPP_ERROR(this->get_logger(), "轨迹时间参数化失败");
			return false;
		}

		rt.getRobotTrajectoryMsg(traj_msg);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = traj_msg;

		const auto exec_result = arm.execute(plan);
		if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_WARN(this->get_logger(), "笛卡尔轨迹执行失败，尝试重启控制器...");
			arm.stop();
			arm.clearPoseTargets();
			arm.setStartStateToCurrentState();
			reset_controller(kControllerName);
			return false;
		}

		RCLCPP_INFO(this->get_logger(), "笛卡尔直线运动完成：沿 Z 轴负方向下降 %.3f m", z_down);
		return true;
	}

private:
	rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

	bool reset_controller(const std::string& controller_name)
	{
		if (!controller_client_->wait_for_service(std::chrono::seconds(3))) {
			RCLCPP_ERROR(this->get_logger(), "服务 /controller_manager/switch_controller 不可用！");
			return false;
		}

		auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
		request->deactivate_controllers.push_back(controller_name);
		request->activate_controllers.push_back(controller_name);
		request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
		request->timeout = rclcpp::Duration::from_seconds(5.0);

		auto future = controller_client_->async_send_request(request);
		if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
			rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(this->get_logger(), "调用 /switch_controller 服务失败！");
			return false;
		}

		if (!future.get()->ok) {
			RCLCPP_ERROR(this->get_logger(), "控制器切换失败，可能资源被占用！");
			return false;
		}

		auto list_client = this->create_client<controller_manager_msgs::srv::ListControllers>(
			"/controller_manager/list_controllers");
		if (!list_client->wait_for_service(std::chrono::seconds(3))) {
			RCLCPP_ERROR(this->get_logger(), "服务 /controller_manager/list_controllers 不可用！");
			return false;
		}

		const int max_retries = 10;
		for (int i = 0; i < max_retries; ++i) {
			auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
			auto list_future = list_client->async_send_request(list_req);

			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), list_future) !=
				rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_WARN(this->get_logger(), "查询控制器状态失败，重试中...");
				continue;
			}

			auto response = list_future.get();
			for (const auto& ctrl : response->controller) {
				if (ctrl.name == controller_name) {
					if (ctrl.state == "active") {
						RCLCPP_INFO(this->get_logger(), "控制器 [%s] 已经重新激活！", controller_name.c_str());
						return true;
					}
					RCLCPP_INFO(this->get_logger(), "控制器 [%s] 当前状态: %s, 等待中...",
								controller_name.c_str(), ctrl.state.c_str());
				}
			}

			rclcpp::sleep_for(std::chrono::milliseconds(500));
		}

		RCLCPP_ERROR(this->get_logger(), "控制器 [%s] 重启后未进入 active 状态！", controller_name.c_str());
		return false;
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<CartesianSLineDemo>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	std::thread spinner([&executor]() {
		executor.spin();
	});

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "rokae_arm");
	node->set_move_group(move_group);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	const std::vector<double> pre_position = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
	bool ok = node->move_to_pre_position(pre_position);
	if (!ok) {
		RCLCPP_ERROR(node->get_logger(), "移动到预位姿失败");
		rclcpp::shutdown();
		spinner.join();
		return 1;
	}

	ok = node->execute_cartesian_line_down(
		0.10,
		0.005,
		0.08,
		0.08);

	if (!ok) {
		RCLCPP_ERROR(node->get_logger(), "笛卡尔直线运动执行失败");
	}

	rclcpp::shutdown();
	spinner.join();
	return ok ? 0 : 1;
}
