#include <rclcpp/rclcpp.hpp>

#include <rokae/robot.h>
#include <rokae/motion_control_rt.h>

#include <chrono>
#include <exception>
#include <string>
#include <thread>

class StopMotionDemoNode : public rclcpp::Node
{
public:
	StopMotionDemoNode()
	: Node("stop_motion_demo")
	{
		robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.2.160");
		local_ip_ = this->declare_parameter<std::string>("local_ip", "192.168.2.100");
		power_on_before_stop_ = this->declare_parameter<bool>("power_on_before_stop", true);
		set_idle_after_stop_ = this->declare_parameter<bool>("set_idle_after_stop", false);
		enable_nrt_reset_ = this->declare_parameter<bool>("enable_nrt_reset", false);
	}

	bool run_demo()
	{
		std::error_code ec;

		try {
			rokae::xMateRobot robot(robot_ip_, local_ip_);

			RCLCPP_INFO(this->get_logger(), "连接机器人: robot_ip=%s local_ip=%s", robot_ip_.c_str(), local_ip_.c_str());
			robot.connectToRobot(ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "连接失败: %s", ec.message().c_str());
				return false;
			}

			auto state = robot.operationState(ec);
			if (ec) {
				RCLCPP_WARN(this->get_logger(), "读取operationState失败: %s", ec.message().c_str());
				ec.clear();
			} else {
				RCLCPP_INFO(this->get_logger(), "当前operationState=%d", static_cast<int>(state));
			}

			if (state == rokae::OperationState::rlProgram) {
				RCLCPP_INFO(this->get_logger(), "检测到RL工程运行中，先尝试pauseProject()");
				robot.pauseProject(ec);
				if (ec) {
					RCLCPP_WARN(this->get_logger(), "pauseProject()返回错误: %s", ec.message().c_str());
					ec.clear();
				}
			}

			if (power_on_before_stop_) {
				robot.setPowerState(true, ec);
				if (ec) {
					RCLCPP_WARN(this->get_logger(), "上电返回错误(继续执行停机): %s", ec.message().c_str());
					ec.clear();
				}
			}

			RCLCPP_INFO(this->get_logger(), "执行实时停止(优先): stopServoJoint() + stopMove()");
			try {
				auto rt_controller = robot.getRtMotionController().lock();
				if (!rt_controller) {
					RCLCPP_WARN(this->get_logger(), "获取实时控制器失败，跳过实时停止");
				} else {
					try {
						rt_controller->stopServoJoint();
					} catch (const std::exception &ex) {
						RCLCPP_WARN(this->get_logger(), "stopServoJoint()异常: %s", ex.what());
					} catch (...) {
						RCLCPP_WARN(this->get_logger(), "stopServoJoint()未知异常");
					}

					try {
						rt_controller->stopMove();
					} catch (const std::exception &ex) {
						RCLCPP_WARN(this->get_logger(), "stopMove()异常: %s", ex.what());
					} catch (...) {
						RCLCPP_WARN(this->get_logger(), "stopMove()未知异常");
					}
				}
			} catch (const std::exception &ex) {
				RCLCPP_WARN(this->get_logger(), "实时停止流程异常: %s", ex.what());
			} catch (...) {
				RCLCPP_WARN(this->get_logger(), "实时停止流程未知异常");
			}

			RCLCPP_INFO(this->get_logger(), "执行非实时停止: stop() 并等待停稳");
			robot.stop(ec);
			if (ec) {
				RCLCPP_WARN(this->get_logger(), "stop()返回错误: %s", ec.message().c_str());
				ec.clear();
			}

			if (!wait_until_motion_stopped(robot, std::chrono::milliseconds(3000), std::chrono::milliseconds(100))) {
				RCLCPP_WARN(this->get_logger(), "等待停稳超时，继续尝试moveReset");
			}

			if (enable_nrt_reset_) {
				std::error_code st_ec;
				auto cur_state = robot.operationState(st_ec);
				if (st_ec) {
					RCLCPP_WARN(this->get_logger(), "读取operationState失败，跳过moveReset: %s", st_ec.message().c_str());
				} else if (cur_state == rokae::OperationState::rtControlling) {
					RCLCPP_WARN(this->get_logger(), "当前仍为rtControlling，跳过moveReset以避免切模崩溃");
				} else {
					RCLCPP_INFO(this->get_logger(), "enable_nrt_reset=true，开始重试moveReset()");
					bool reset_ok = false;
					for (int attempt = 1; attempt <= 20; ++attempt) {
						robot.moveReset(ec);
						if (!ec) {
							reset_ok = true;
							RCLCPP_INFO(this->get_logger(), "moveReset()成功, attempt=%d", attempt);
							break;
						}

						const std::string msg = ec.message();
						RCLCPP_WARN(this->get_logger(), "moveReset()失败, attempt=%d, err=%s", attempt, msg.c_str());
						ec.clear();
						std::this_thread::sleep_for(std::chrono::milliseconds(100));
					}

					if (!reset_ok) {
						RCLCPP_WARN(this->get_logger(), "moveReset()多次重试后仍失败，建议检查控制器当前任务来源(RL/示教器/外部控制)");
					}
				}
			} else {
				RCLCPP_INFO(this->get_logger(), "默认跳过moveReset()；如需执行请设参数 enable_nrt_reset:=true");
			}

			if (set_idle_after_stop_) {
				robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
				if (ec) {
					RCLCPP_WARN(this->get_logger(), "切换Idle失败: %s", ec.message().c_str());
					ec.clear();
				}
			}

			RCLCPP_INFO(this->get_logger(), "停止运动示例执行完成");
			return true;
		}
		catch (const std::exception &ex) {
			RCLCPP_ERROR(this->get_logger(), "运行异常: %s", ex.what());
			return false;
		}
		catch (...) {
			RCLCPP_ERROR(this->get_logger(), "运行异常: 未知异常");
			return false;
		}
	}

private:
	static bool is_motion_active(rokae::OperationState state)
	{
		return state == rokae::OperationState::moving ||
		       state == rokae::OperationState::jogging ||
		       state == rokae::OperationState::rtControlling ||
		       state == rokae::OperationState::rlProgram ||
		       state == rokae::OperationState::demo ||
		       state == rokae::OperationState::dynamicIdentify ||
		       state == rokae::OperationState::frictionIdentify ||
		       state == rokae::OperationState::loadIdentify;
	}

	template <class RobotT>
	bool wait_until_motion_stopped(RobotT &robot,
						   std::chrono::milliseconds timeout,
						   std::chrono::milliseconds poll_interval)
	{
		auto deadline = std::chrono::steady_clock::now() + timeout;
		while (std::chrono::steady_clock::now() < deadline) {
			std::error_code state_ec;
			auto state = robot.operationState(state_ec);
			if (state_ec) {
				RCLCPP_WARN(this->get_logger(), "等待停稳时读取operationState失败: %s", state_ec.message().c_str());
			} else if (!is_motion_active(state)) {
				RCLCPP_INFO(this->get_logger(), "检测到已停稳, operationState=%d", static_cast<int>(state));
				return true;
			}

			std::this_thread::sleep_for(poll_interval);
		}
		return false;
	}

	std::string robot_ip_;
	std::string local_ip_;
	bool power_on_before_stop_;
	bool set_idle_after_stop_;
	bool enable_nrt_reset_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<StopMotionDemoNode>();
	const bool ok = node->run_demo();
	rclcpp::shutdown();
	return ok ? 0 : 1;
}
