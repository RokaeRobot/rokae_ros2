#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rokae/robot.h>
#include <rokae/data_types.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <vector>

constexpr double kPi = 3.14159265358979323846;

class ServoJDemoNode : public rclcpp::Node
{
public:
	ServoJDemoNode()
	: Node("servoj_demo")
	{
		stop_service_ = this->create_service<std_srvs::srv::Trigger>(
			"/servoj_demo/stop",
			[this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
				   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
				stop_requested_.store(true);
				response->success = true;
				response->message = "stop request accepted";
				RCLCPP_WARN(this->get_logger(), "收到停止请求: /servoj_demo/stop");
			});

		robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.21.10");
		local_ip_ = this->declare_parameter<std::string>("local_ip", "192.168.21.131");

		start_joints_ = this->declare_parameter<std::vector<double>>(
			"start_joints",
			std::vector<double>{0.20, 0.30, 0.10, 0.60, 0.10, 0.30});

		reset_joints_ = this->declare_parameter<std::vector<double>>(
			"reset_joints",
			std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

		amplitudes_ = this->declare_parameter<std::vector<double>>(
			"amplitudes",
			std::vector<double>{0.12, 0.10, 0.08, 0.06, 0.05, 0.04});

		phases_ = this->declare_parameter<std::vector<double>>(
			"phases",
			std::vector<double>{0.0, 1.57, 3.14, 1.57, 0.0, 3.14});

		period_s_ = this->declare_parameter<double>("period_s", 4.0);
		duration_s_ = this->declare_parameter<double>("duration_s", 30.0);
		cycle_ms_ = this->declare_parameter<int>("cycle_ms", 20);

		movej_speed_ = this->declare_parameter<double>("movej_speed", 0.2);
		servoj_lookahead_s_ = this->declare_parameter<double>("servoj_lookahead_s", 0.06);
		servoj_kp_ = this->declare_parameter<double>("servoj_kp", 1.0);
	}

	bool run_demo()
	{
		stop_requested_.store(false);
		run_result_.store(1);

		if (!validate_parameters()) {
			run_result_.store(1);
			return false;
		}

		try {
			std::error_code ec;
			rokae::xMateRobot robot(robot_ip_, local_ip_);

			robot.connectToRobot(ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "连接机器人失败: %s", ec.message().c_str());
				return false;
			}

			robot.setOperateMode(rokae::OperateMode::automatic, ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "设置自动模式失败: %s", ec.message().c_str());
				return false;
			}

			robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "切换实时模式失败: %s", ec.message().c_str());
				return false;
			}

			robot.setPowerState(true, ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "上电失败: %s", ec.message().c_str());
				return false;
			}

			auto rt_con = robot.getRtMotionController().lock();
			if (!rt_con) {
				RCLCPP_ERROR(this->get_logger(), "获取实时控制器失败");
				return false;
			}

			std::array<double, 6> start_target{};
			std::array<double, 6> reset_target{};
			for (size_t i = 0; i < 6; ++i) {
				start_target[i] = start_joints_[i];
				reset_target[i] = reset_joints_[i];
			}

			auto current = robot.jointPos(ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "读取当前关节失败: %s", ec.message().c_str());
				return false;
			}

			RCLCPP_INFO(this->get_logger(), "MoveJ 到起始点...");
			rt_con->MoveJ(movej_speed_, current, start_target);
			if (stop_requested_.load()) {
				RCLCPP_WARN(this->get_logger(), "检测到停止请求，跳过轨迹循环并执行停机");
			}

			const double cycle_s = static_cast<double>(cycle_ms_) / 1000.0;
			rt_con->setServoJoint(cycle_s, servoj_lookahead_s_, servoj_kp_, ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "启用 ServoJ 失败: %s", ec.message().c_str());
				return false;
			}

			rt_con->startMove(rokae::RtControllerMode::jointPosition);
			RCLCPP_INFO(this->get_logger(), "ServoJ 已启动，开始 20ms 周期轨迹下发");

			const double omega = 2.0 * kPi / period_s_;
			const auto loop_start = std::chrono::steady_clock::now();
			auto next_tick = loop_start;

			while (rclcpp::ok() && !stop_requested_.load()) {
				const auto now = std::chrono::steady_clock::now();
				const double t = std::chrono::duration<double>(now - loop_start).count();
				if (t >= duration_s_) {
					break;
				}

				rokae::JointPosition cmd(6);
				for (size_t i = 0; i < 6; ++i) {
					const double offset = amplitudes_[i] * std::cos(omega * t + phases_[i]);
					cmd.joints[i] = start_joints_[i] + offset;
				}

				rt_con->sendCommand(cmd);

				next_tick += std::chrono::milliseconds(cycle_ms_);
				std::this_thread::sleep_until(next_tick);
			}

			RCLCPP_INFO(this->get_logger(), "停止 ServoJ 并复位...");
			rt_con->stopServoJoint();
			rt_con->stopMove();

			current = robot.jointPos(ec);
			if (ec) {
				RCLCPP_WARN(this->get_logger(), "读取当前关节失败，使用起始点作为复位起点");
				current = start_target;
			}

			if (!stop_requested_.load()) {
				rt_con->MoveJ(movej_speed_, current, reset_target);
			}

			robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
			robot.setPowerState(false, ec);

			RCLCPP_INFO(this->get_logger(), "ServoJ 示例执行完成");
			run_result_.store(0);
			return true;
		}
		catch (const std::exception& ex) {
			RCLCPP_ERROR(this->get_logger(), "运行异常: %s", ex.what());
			run_result_.store(1);
			return false;
		}
	}

	int exit_code() const
	{
		return run_result_.load();
	}

private:
	std::string robot_ip_;
	std::string local_ip_;
	std::vector<double> start_joints_;
	std::vector<double> reset_joints_;
	std::vector<double> amplitudes_;
	std::vector<double> phases_;

	double period_s_;
	double duration_s_;
	int cycle_ms_;
	double movej_speed_;
	double servoj_lookahead_s_;
	double servoj_kp_;

	std::atomic<bool> stop_requested_{false};
	std::atomic<int> run_result_{1};
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

	bool validate_parameters()
	{
		if (start_joints_.size() != 6 || reset_joints_.size() != 6 || amplitudes_.size() != 6 || phases_.size() != 6) {
			RCLCPP_ERROR(this->get_logger(), "start_joints/reset_joints/amplitudes/phases 必须都是 6 维");
			return false;
		}

		if (period_s_ <= 0.0 || duration_s_ <= 0.0 || cycle_ms_ <= 0) {
			RCLCPP_ERROR(this->get_logger(), "period_s/duration_s/cycle_ms 必须大于 0");
			return false;
		}

		if (movej_speed_ <= 0.0 || movej_speed_ > 1.0) {
			RCLCPP_ERROR(this->get_logger(), "movej_speed 必须在 (0,1] 区间");
			return false;
		}

		if (servoj_lookahead_s_ <= 0.0 || servoj_kp_ <= 0.0) {
			RCLCPP_ERROR(this->get_logger(), "servoj_lookahead_s/servoj_kp 必须大于 0");
			return false;
		}

		return true;
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ServoJDemoNode>();

	std::thread worker([node]() {
		node->run_demo();
		rclcpp::shutdown();
	});

	rclcpp::spin(node);

	if (worker.joinable()) {
		worker.join();
	}

	return node->exit_code();
}
