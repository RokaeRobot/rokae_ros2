// MoveIt + FollowJointTrajectory 关节空间往复轨迹示范
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

const std::string kControllerName = "position_joint_trajectory_controller";
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class FollowJointPositionDemo : public rclcpp::Node
{
public:
	FollowJointPositionDemo()
	: Node("follow_joint_position_node")
	{
		controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
			"/controller_manager/switch_controller");

		trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
			this,
			"/position_joint_trajectory_controller/follow_joint_trajectory");

		robot_type_ = this->declare_parameter<std::string>("robot_type", "CR7");
		point_period_s_ = this->declare_parameter<double>("point_period_s", 0.6);
		start_blend_s_ = this->declare_parameter<double>("start_blend_s", 0.6);
		goal_tolerance_rad_ = this->declare_parameter<double>("goal_tolerance_rad", 0.01);
		vel_scale_ = this->declare_parameter<double>("vel_scale", 0.2);
		acc_scale_ = this->declare_parameter<double>("acc_scale", 0.2);
		cycle_count_ = this->declare_parameter<int>("cycle_count", 0);  // 0: 无限循环
	}

	void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
	{
		move_group_ = move_group;
	}

	bool run_reciprocating_motion()
	{
		if (!move_group_) {
			RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 未初始化");
			return false;
		}

		if (point_period_s_ <= 0.0 || start_blend_s_ <= 0.0) {
			RCLCPP_ERROR(this->get_logger(), "参数非法，要求 point_period_s/start_blend_s > 0");
			return false;
		}

		if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
			RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server 不可用");
			return false;
		}

		auto& arm = *move_group_;
		arm.setPlanningTime(20.0);
		arm.allowReplanning(true);
		arm.setGoalPositionTolerance(goal_tolerance_rad_);
		arm.setGoalOrientationTolerance(goal_tolerance_rad_);
		arm.setMaxAccelerationScalingFactor(acc_scale_);
		arm.setMaxVelocityScalingFactor(vel_scale_);

		const auto joint_names = arm.getJointNames();
		const auto current = arm.getCurrentJointValues();
		const size_t dof = current.size();

		if (joint_names.size() != dof || dof == 0) {
			RCLCPP_ERROR(this->get_logger(), "关节名与关节位置维度不一致或为空");
			return false;
		}

		auto preset = build_preset_sequence(robot_type_, dof, current);
		if (preset.empty()) {
			RCLCPP_ERROR(this->get_logger(), "未生成有效预设轨迹");
			return false;
		}

		if (!move_to_start_position(preset.front())) {
			RCLCPP_ERROR(this->get_logger(), "移动到起点失败");
			return false;
		}

		int cycle_index = 0;
		while (rclcpp::ok() && (cycle_count_ <= 0 || cycle_index < cycle_count_)) {
			const auto cycle_points = build_reciprocating_points(preset);
			if (cycle_points.size() < 2) {
				RCLCPP_ERROR(this->get_logger(), "往复轨迹点不足");
				return false;
			}

			trajectory_msgs::msg::JointTrajectory traj;
			traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.2);
			traj.joint_names = joint_names;

			traj.points.reserve(cycle_points.size());
			for (size_t i = 0; i < cycle_points.size(); ++i) {
				trajectory_msgs::msg::JointTrajectoryPoint p;
				p.positions = cycle_points[i];
				p.velocities.assign(dof, 0.0);
				p.accelerations.assign(dof, 0.0);

				const double t = start_blend_s_ + point_period_s_ * static_cast<double>(i);
				p.time_from_start = make_duration_msg(t);
				traj.points.push_back(std::move(p));
			}

			FollowJointTrajectory::Goal goal;
			goal.trajectory = std::move(traj);
			goal.goal_time_tolerance = make_duration_msg(0.3);

			const bool sent_ok = send_goal_and_wait(goal, point_period_s_, cycle_points.size());
			if (!sent_ok) {
				RCLCPP_WARN(this->get_logger(), "轨迹执行失败，尝试重启控制器");
				if (!reset_controller(kControllerName)) {
					RCLCPP_ERROR(this->get_logger(), "控制器恢复失败，终止任务");
					return false;
				}
				continue;
			}

			++cycle_index;
			RCLCPP_INFO(this->get_logger(), "完成第 %d 次往复周期", cycle_index);
		}

		RCLCPP_INFO(this->get_logger(), "关节往复轨迹任务结束");
		return true;
	}

private:
	rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
	rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

	std::string robot_type_;
	double point_period_s_;
	double start_blend_s_;
	double goal_tolerance_rad_;
	double vel_scale_;
	double acc_scale_;
	int cycle_count_;

	static std::string to_upper(std::string value)
	{
		std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
			return static_cast<char>(std::toupper(c));
		});
		return value;
	}

	builtin_interfaces::msg::Duration make_duration_msg(double seconds) const
	{
		builtin_interfaces::msg::Duration d;
		const int32_t sec = static_cast<int32_t>(std::floor(seconds));
		const double frac = seconds - static_cast<double>(sec);
		d.sec = sec;
		d.nanosec = static_cast<uint32_t>(std::round(frac * 1e9));
		if (d.nanosec >= 1000000000U) {
			d.sec += 1;
			d.nanosec -= 1000000000U;
		}
		return d;
	}

	bool is_seven_dof_model(const std::string& robot_type) const
	{
		const std::string t = to_upper(robot_type);
		return t == "PRO7" || t == "PRO3" || t == "SR3";
	}

	std::vector<double> make_target_with_prefix(
		const std::vector<double>& current,
		const std::vector<double>& prefix) const
	{
		auto target = current;
		for (size_t i = 0; i < prefix.size() && i < target.size(); ++i) {
			target[i] = prefix[i];
		}
		return target;
	}

	std::vector<std::vector<double>> build_preset_sequence(
		const std::string& robot_type,
		size_t dof,
		const std::vector<double>& current) const
	{
		const std::string type_upper = to_upper(robot_type);
		const bool want_7d = is_seven_dof_model(type_upper);

		if (want_7d && dof >= 7) {
			RCLCPP_INFO(this->get_logger(), "机型 %s: 载入 7 轴预设轨迹", type_upper.c_str());
			return {
				make_target_with_prefix(current, {0.10, 0.30, 0.10, 0.50, 0.10, 0.30, 0.00}),
				make_target_with_prefix(current, {0.20, 0.50, 0.00, 0.70, 0.20, 0.50, 0.15}),
				make_target_with_prefix(current, {0.40, 0.60, -0.20, 0.90, 0.35, 0.60, 0.25}),
				make_target_with_prefix(current, {0.55, 0.45, -0.35, 0.80, 0.40, 0.45, 0.20}),
				make_target_with_prefix(current, {0.35, 0.25, -0.20, 0.55, 0.25, 0.30, 0.10}),
				make_target_with_prefix(current, {0.10, 0.30, 0.10, 0.50, 0.10, 0.30, 0.00})
			};
		}

		if (dof < 6) {
			RCLCPP_ERROR(this->get_logger(), "当前自由度 %zu 小于 6，无法加载预设轨迹", dof);
			return {};
		}

		RCLCPP_INFO(this->get_logger(), "机型 %s: 载入 6 轴预设轨迹", type_upper.c_str());
		return {
			make_target_with_prefix(current, {-0.018, 0.494, -1.628, 0.102, -1.023, -0.071}),
			make_target_with_prefix(current, {-0.033, 0.466, -1.684, 0.207, -1.007, -0.145}),
			make_target_with_prefix(current, {-0.044, 0.440, -1.739, 0.313, -0.998, -0.219}),
			make_target_with_prefix(current, {-0.051, 0.415, -1.790, 0.419, -0.997, -0.292}),
			make_target_with_prefix(current, {-0.055, 0.392, -1.837, 0.523, -1.005, -0.361}),
			make_target_with_prefix(current, {-0.056, 0.372, -1.879, 0.625, -1.021, -0.426}),
			make_target_with_prefix(current, {-0.055, 0.354, -1.915, 0.724, -1.044, -0.485}),
			make_target_with_prefix(current, {-0.051, 0.338, -1.946, 0.819, -1.074, -0.537}),
			make_target_with_prefix(current, {-0.045, 0.324, -1.972, 0.909, -1.109, -0.583}),
			make_target_with_prefix(current, {-0.037, 0.313, -1.994, 0.996, -1.148, -0.623})
		};
	}

	bool move_to_start_position(const std::vector<double>& start_target)
	{
		auto& arm = *move_group_;
		arm.setStartStateToCurrentState();
		arm.setJointValueTarget(start_target);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "起点规划失败");
			return false;
		}

		if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "起点执行失败");
			return false;
		}

		RCLCPP_INFO(this->get_logger(), "已到达往复轨迹起点");
		return true;
	}

	std::vector<std::vector<double>> build_reciprocating_points(
		const std::vector<std::vector<double>>& forward) const
	{
		std::vector<std::vector<double>> points = forward;
		if (forward.size() <= 1) {
			return points;
		}

		for (size_t i = forward.size() - 2; i > 0; --i) {
			points.push_back(forward[i]);
		}

		points.push_back(forward.front());
		return points;
	}

	bool send_goal_and_wait(
		const FollowJointTrajectory::Goal& goal,
		double point_period_s,
		size_t point_count)
	{
		auto goal_future = trajectory_action_client_->async_send_goal(goal);
		if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
			RCLCPP_ERROR(this->get_logger(), "发送轨迹 goal 超时");
			return false;
		}

		auto goal_handle = goal_future.get();
		if (!goal_handle) {
			RCLCPP_ERROR(this->get_logger(), "轨迹 goal 被拒绝");
			return false;
		}

		auto result_future = trajectory_action_client_->async_get_result(goal_handle);
		const double run_time = start_blend_s_ + point_period_s * static_cast<double>(point_count) + 5.0;
		const auto wait_timeout = std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::duration<double>(run_time));

		if (result_future.wait_for(wait_timeout) != std::future_status::ready) {
			RCLCPP_ERROR(this->get_logger(), "等待轨迹执行结果超时");
			return false;
		}

		const auto wrapped_result = result_future.get();
		if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
			RCLCPP_ERROR(this->get_logger(), "轨迹执行失败，action result code: %d", static_cast<int>(wrapped_result.code));
			return false;
		}

		if (wrapped_result.result && wrapped_result.result->error_code != 0) {
			RCLCPP_ERROR(this->get_logger(), "控制器返回错误码: %d", wrapped_result.result->error_code);
			return false;
		}

		return true;
	}

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
		if (!wait_for_future_ready(future, std::chrono::seconds(6), "switch_controller")) {
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

			if (!wait_for_future_ready(list_future, std::chrono::seconds(3), "list_controllers")) {
				RCLCPP_WARN(this->get_logger(), "查询控制器状态失败，重试中...");
				continue;
			}

			auto response = list_future.get();
			for (const auto& ctrl : response->controller) {
				if (ctrl.name == controller_name && ctrl.state == "active") {
					RCLCPP_INFO(this->get_logger(), "控制器 [%s] 已重新激活", controller_name.c_str());
					return true;
				}
			}

			rclcpp::sleep_for(std::chrono::milliseconds(500));
		}

		RCLCPP_ERROR(this->get_logger(), "控制器 [%s] 未进入 active 状态", controller_name.c_str());
		return false;
	}

	template <typename FutureT>
	bool wait_for_future_ready(
		FutureT& future,
		std::chrono::milliseconds timeout,
		const std::string& tag)
	{
		const auto start = std::chrono::steady_clock::now();
		while (rclcpp::ok()) {
			if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
				return true;
			}

			if (std::chrono::steady_clock::now() - start >= timeout) {
				RCLCPP_ERROR(this->get_logger(), "%s 超时", tag.c_str());
				return false;
			}
		}

		RCLCPP_ERROR(this->get_logger(), "%s 等待被中断", tag.c_str());
		return false;
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<FollowJointPositionDemo>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);

	std::thread spinner([&executor]() {
		executor.spin();
	});

	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "rokae_arm");
	node->set_move_group(move_group);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	const bool ok = node->run_reciprocating_motion();
	if (!ok) {
		RCLCPP_ERROR(node->get_logger(), "follow_joint_position 执行失败");
	}

	rclcpp::shutdown();
	spinner.join();
	return ok ? 0 : 1;
}
