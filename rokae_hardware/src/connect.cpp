#include <rclcpp/rclcpp.hpp>
#include <rokae/robot.h>
#include <rokae/data_types.h>
#include "rokae_hardware/rokae_hardware_interface.h"


class RokaeInterfaceNode : public rclcpp::Node
{
public:
  RokaeInterfaceNode()
  : Node("connect")
  {
    declare_parameter("robot_ip", rclcpp::PARAMETER_STRING);
    declare_parameter("local_ip", rclcpp::PARAMETER_STRING);

    // declare_parameter<std::string>("robot_ip", "192.168.21.10");
    // declare_parameter<std::string>("local_ip", "192.168.21.1");

    get_parameter("robot_ip", robot_ip_);
    get_parameter("local_ip", local_ip_);

    RCLCPP_INFO(get_logger(), "Trying to connect to robot at %s from %s", robot_ip_.c_str(), local_ip_.c_str());

    

    try {
      robot_ = std::make_shared<rokae::xMateRobot>(robot_ip_, local_ip_);
      robot_->connectToRobot(ec_);
      if (ec_.value() != 0) {
        RCLCPP_ERROR(get_logger(), "Connection failed with error: %s", ec_.message().c_str());
        return;
      }

      robot_->setOperateMode(rokae::OperateMode::automatic, ec_);
      robot_->setRtNetworkTolerance(20, ec_);
      robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec_);
      robot_->setPowerState(true, ec_);

      if (ec_.value() != 0) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize robot: %s", ec_.message().c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Robot connection and initialization successful.");
      }

      robot_->startReceiveRobotState(std::chrono::milliseconds(1),{rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
                                      rokae::RtSupportedFields::tau_m, rokae::RtSupportedFields::tauExt_inBase,
                                      rokae::RtSupportedFields::tauExt_inStiff});

      // robot_->updateRobotState(std::chrono::milliseconds(1));
      // robot_->getStateData("q_m", pos);
      // robot_->getStateData(rokae::RtSupportedFields::jointVel_m, vel);
      // robot_->getStateData("tau_m", torque);

      for (int i = 0; i < 6; i++) {
      RCLCPP_INFO(get_logger(),
                  "Joint %d: pos=%.6f, vel=%.6f, torque=%.6f",
                  i + 1, pos[i], vel[i], torque[i]);
    }

      // 定时器，100ms周期更新并打印状态
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RokaeInterfaceNode::updateAndPrintState, this));

  //     rci_ = robot_->getRtMotionController().lock();
  //     rci_->startReceiveRobotState({rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
  //                                     rokae::RtSupportedFields::tau_m, rokae::RtSupportedFields::tauExt_inBase,
  //                                     rokae::RtSupportedFields::tauExt_inStiff});
  //     rci_->updateRobotState();
  //     std::array<double, 6> robot_joint_position{};
  //     std::array<double, 6> robot_joint_velocity{};
  //     std::array<double, 6> robot_joint_torque{};

  //     for (int i = 0; i < 6; i++) {
  //     RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"),
  //                 "From robot: pos=%.6f, vel=%.6f, torque=%.6f",
  //                 robot_joint_position[i],
  //                 robot_joint_velocity[i],
  //                 robot_joint_torque[i]);
  // }

  //   // 读取状态数据，getStateData 返回0通常表示成功
  //   int ret_pos = rci_->getStateData("q_m", robot_joint_position);
  //   int ret_vel = rci_->getStateData(rokae::RtSupportedFields::jointVel_m, robot_joint_velocity);
  //   int ret_torque = rci_->getStateData("tau_m", robot_joint_torque);

    } catch (const rokae::NetworkException &e) {
      RCLCPP_FATAL(get_logger(), "NetworkException: %s", e.what());
    } catch (const rokae::ExecutionException &e) {
      RCLCPP_FATAL(get_logger(), "ExecutionException: %s", e.what());
    }
}

private:
  void updateAndPrintState()
  {
    //rci_ = robot_->getRtMotionController().lock();
    if (!robot_) {
      RCLCPP_WARN(get_logger(), "RtMotionController not initialized");
      return;
    }

    // 刷新状态
    robot_->updateRobotState(std::chrono::milliseconds(1));

    int ret_pos = robot_->getStateData("q_m", pos);
    int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, vel);
    int ret_torque = robot_->getStateData("tau_m", torque);

    if (ret_pos != 0)
      RCLCPP_WARN(get_logger(), "Failed to get joint position");
    if (ret_vel != 0)
      RCLCPP_WARN(get_logger(), "Failed to get joint velocity");
    if (ret_torque != 0)
      RCLCPP_WARN(get_logger(), "Failed to get joint torque");

    for (int i = 0; i < 6; i++) {
      RCLCPP_INFO(get_logger(),
                  "Joint %d: pos=%.6f, vel=%.6f, torque=%.6f",
                  i + 1, pos[i], vel[i], torque[i]);
    }

  }
  std::shared_ptr<rokae::xMateRobot> robot_;
  std::string robot_ip_, local_ip_;
  std::error_code ec_;
  using RtType = rokae::RtMotionControl<rokae::WorkType::collaborative, 6>;
  std::shared_ptr<RtType> rci_;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> torque{};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //auto node = std::make_shared<RokaeInterfaceNode>();
  rclcpp::spin(std::make_shared<RokaeInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
