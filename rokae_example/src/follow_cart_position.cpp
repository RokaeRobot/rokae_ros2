// MoveIt 笛卡尔动态跟随示范：沿 Y 轴执行正弦往复运动
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

const std::string kControllerName = "position_joint_trajectory_controller";
constexpr double kPi = 3.14159265358979323846;

class FollowCartPositionDemo : public rclcpp::Node
{
public:
	FollowCartPositionDemo()
	: Node("follow_cart_position_node")
	{
		controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
			"/controller_manager/switch_controller");

		amplitude_m_ = this->declare_parameter<double>("amplitude_m", 0.4);
		period_s_ = this->declare_parameter<double>("period_s", 1.33);
		target_update_hz_ = this->declare_parameter<double>("target_update_hz", 1.0);
		control_hz_ = this->declare_parameter<double>("control_hz", 5.0);
		kp_ = this->declare_parameter<double>("kp", 0.6);
		filter_alpha_ = this->declare_parameter<double>("filter_alpha", 0.25);
		duration_s_ = this->declare_parameter<double>("duration_s", 20.0);
		vel_scale_ = this->declare_parameter<double>("vel_scale", 0.08);
		acc_scale_ = this->declare_parameter<double>("acc_scale", 0.08);
		eef_step_ = this->declare_parameter<double>("eef_step", 0.002);
		min_fraction_ = this->declare_parameter<double>("min_fraction", 0.95);
	}

	void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
	{
		move_group_ = move_group;
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

	bool execute_follow_motion()
	{
		if (!move_group_) {
			RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 未初始化");
			return false;
		}

		if (amplitude_m_ <= 0.0 || period_s_ <= 0.0 || target_update_hz_ <= 0.0 || control_hz_ <= 0.0) {
			RCLCPP_ERROR(this->get_logger(), "参数非法，要求 amplitude/period/update_hz/control_hz > 0");
			return false;
		}

		if (kp_ <= 0.0 || filter_alpha_ <= 0.0 || filter_alpha_ > 1.0) {
			RCLCPP_ERROR(this->get_logger(), "参数非法，要求 kp > 0 且 0 < filter_alpha <= 1");
			return false;
		}

		if (vel_scale_ <= 0.0 || vel_scale_ > 1.0 || acc_scale_ <= 0.0 || acc_scale_ > 1.0) {
			RCLCPP_ERROR(this->get_logger(), "参数非法，要求 0 < vel_scale/acc_scale <= 1");
			return false;
		}

		auto& arm = *move_group_;
		arm.setStartStateToCurrentState();

		const geometry_msgs::msg::Pose base_pose = arm.getCurrentPose().pose;
		const double base_y = base_pose.position.y;
		std::atomic<double> desired_y(base_y);
		double filtered_y = base_y;

		std::atomic<bool> stop_target_thread(false);
		std::mutex io_mutex;

		const auto start_time = std::chrono::steady_clock::now();
		std::thread target_thread([&]() {
			const auto update_dt = std::chrono::duration<double>(1.0 / target_update_hz_);
			while (!stop_target_thread.load()) {
				const auto now = std::chrono::steady_clock::now();
				const double t = std::chrono::duration<double>(now - start_time).count();
				const double omega = 2.0 * kPi / period_s_;
				const double target_offset = amplitude_m_ * std::sin(omega * t);
				desired_y.store(base_y + target_offset);

				{
					std::lock_guard<std::mutex> lk(io_mutex);
					RCLCPP_INFO_THROTTLE(
						this->get_logger(),
						*this->get_clock(),
						1000,
						"目标点更新: t=%.2f s, y_target=%.4f m",
						t,
						desired_y.load());
				}

				std::this_thread::sleep_for(update_dt);
			}
		});

		bool all_ok = true;
		const auto control_dt = std::chrono::duration<double>(1.0 / control_hz_);
		const auto end_time = start_time + std::chrono::duration<double>(duration_s_);

		while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
			arm.setStartStateToCurrentState();
			const geometry_msgs::msg::Pose current_pose = arm.getCurrentPose().pose;

			const double y_des = desired_y.load();
			const double y_error = y_des - current_pose.position.y;
			const double y_prop = current_pose.position.y + kp_ * y_error;
			filtered_y = filter_alpha_ * y_prop + (1.0 - filter_alpha_) * filtered_y;
			filtered_y = std::clamp(filtered_y, base_y - amplitude_m_, base_y + amplitude_m_);

			geometry_msgs::msg::Pose target_pose = current_pose;
			target_pose.position.y = filtered_y;

			const bool ok = execute_cartesian_step(current_pose, target_pose);
			if (!ok) {
				all_ok = false;
				RCLCPP_WARN(this->get_logger(), "当前控制步执行失败，继续下一步跟踪");
			}

			std::this_thread::sleep_for(control_dt);
		}

		stop_target_thread.store(true);
		if (target_thread.joinable()) {
			target_thread.join();
		}

		if (all_ok) {
			RCLCPP_INFO(this->get_logger(), "笛卡尔动态正弦跟随完成");
		} else {
			RCLCPP_WARN(this->get_logger(), "笛卡尔动态正弦跟随结束（含局部失败步）");
		}

		return all_ok;
	}

private:
	rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

	double amplitude_m_;
	double period_s_;
	double target_update_hz_;
	double control_hz_;
	double kp_;
	double filter_alpha_;
	double duration_s_;
	double vel_scale_;
	double acc_scale_;
	double eef_step_;
	double min_fraction_;

	bool execute_cartesian_step(
		const geometry_msgs::msg::Pose& current_pose,
		const geometry_msgs::msg::Pose& target_pose)
	{
		auto& arm = *move_group_;

		std::vector<geometry_msgs::msg::Pose> waypoints;
		waypoints.reserve(2);
		waypoints.push_back(current_pose);
		waypoints.push_back(target_pose);

		moveit_msgs::msg::RobotTrajectory traj_msg;
		const double jump_threshold = 0.0;

		const double fraction = arm.computeCartesianPath(
			waypoints,
			std::max(0.001, eef_step_),
			jump_threshold,
			traj_msg,
			true);

		if (fraction < min_fraction_) {
			RCLCPP_WARN(this->get_logger(), "笛卡尔局部路径不完整，完成率: %.3f", fraction);
			return false;
		}

		robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
		rt.setRobotTrajectoryMsg(*arm.getCurrentState(), traj_msg);

		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		const bool timed_ok = iptp.computeTimeStamps(rt, vel_scale_, acc_scale_);
		if (!timed_ok) {
			RCLCPP_WARN(this->get_logger(), "局部轨迹时间参数化失败");
			return false;
		}

		rt.getRobotTrajectoryMsg(traj_msg);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = traj_msg;

		const auto exec_result = arm.execute(plan);
		if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
			RCLCPP_WARN(this->get_logger(), "局部轨迹执行失败，尝试重启控制器...");
			arm.stop();
			arm.clearPoseTargets();
			arm.setStartStateToCurrentState();
			reset_controller(kControllerName);
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

	auto node = std::make_shared<FollowCartPositionDemo>();

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

	ok = node->execute_follow_motion();
	if (!ok) {
		RCLCPP_ERROR(node->get_logger(), "笛卡尔动态正弦跟随执行结束（存在失败步）");
	}

	rclcpp::shutdown();
	spinner.join();
	return ok ? 0 : 1;
}
