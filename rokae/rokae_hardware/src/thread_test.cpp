#include "rclcpp/rclcpp.hpp"
#include "rokae_hardware/rokae_hardware_interface.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto hw = std::make_shared<rokae_hardware::RokaeHardwareInterface>();
  hardware_interface::HardwareInfo info;
  info.joints.resize(6);
  for (int i = 0; i < 6; i++) {
      info.joints[i].name = "joint" + std::to_string(i+1);
      info.joints[i].command_interfaces.push_back({"position"});
      info.joints[i].state_interfaces.push_back({"position"});
      info.joints[i].state_interfaces.push_back({"velocity"});
  }
  info.hardware_parameters["robot_ip"] = "192.168.21.10";
  info.hardware_parameters["local_ip"] = "192.168.21.131";
  // 这里手动填充 info，比如 joint names、参数等
  hw->on_init(info);
  hw->on_configure(rclcpp_lifecycle::State());
  hw->on_activate(rclcpp_lifecycle::State());
  for (int i = 0; i < 100; ++i) {
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "read() start");
    hw->read(rclcpp::Time(0), rclcpp::Duration(1,0));
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "read() end");
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "write() start");
    hw->write(rclcpp::Time(0), rclcpp::Duration(1,0));
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "write() end");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  hw->on_deactivate(rclcpp_lifecycle::State());
  rclcpp::shutdown();
}
