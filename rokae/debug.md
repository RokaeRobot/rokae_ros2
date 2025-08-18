[INFO] [1733485610.125805]: Started controllers: joint_state_controller, position_joint_trajectory_controller
terminate called after throwing an instance of 'rokae::RealtimeMotionException'
  what():  实时模式异常: 运动错误: 轴空间运动超关节限位或轨迹不连续; 或速度/加速度超过阈值, 请检查客户端规划轨迹. (3-kActualJointVelocityLimitsViolation)
[rokae_hardware-2] process has died [pid 37236, exit code -6, cmd /home/ubuntu/caktin_ws/devel/lib/rokae_hardware/rokae_hardware __name:=rokae_hardware __log:=/home/ubuntu/.ros/log/c5fc120a-b3c7-11ef-88da-4d4342cd8751/rokae_hardware-2.log].
log file: /home/ubuntu/.ros/log/c5fc120a-b3c7-11ef-88da-4d4342cd8751/rokae_hardware-2*.log


home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:289:13: warning: ‘void rokae::MotionControl<rokae::MotionControlMode::RtCommand>::updateRobotState()’ is deprecated: Use BaseRobot::updateRobotState() instead [-Wdeprecated-declarations]
  289 |             rci_->updateRobotState();
      |             ^~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/robot.h:18,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:19,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/motion_control_rt.h:131:9: note: declared here
  131 |    void updateRobotState();
      |         ^~~~~~~~~~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:298:17: warning: ‘int rokae::MotionControl<rokae::MotionControlMode::RtCommand>::getStateData(const string&, R&) [with R = std::array<double, 6>; std::string = std::__cxx11::basic_string<char>]’ is deprecated: Use BaseRobot::getStateData(fieldName, data) instead [-Wdeprecated-declarations]
  298 |             int pos_flag = rci_->getStateData("q_m", robot_joint_postion);
      |                 ^~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/robot.h:18,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:19,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/motion_control_rt.h:145:8: note: declared here
  145 |    int getStateData(const std::string &fieldName, R &data);
      |        ^~~~~~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:300:17: warning: ‘int rokae::MotionControl<rokae::MotionControlMode::RtCommand>::getStateData(const string&, R&) [with R = std::array<double, 6>; std::string = std::__cxx11::basic_string<char>]’ is deprecated: Use BaseRobot::getStateData(fieldName, data) instead [-Wdeprecated-declarations]
  300 |             int vel_flag = rci_->getStateData(rokae::RtSupportedFields::jointVel_m, robot_joint_velocity);
      |                 ^~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/robot.h:18,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:19,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/motion_control_rt.h:145:8: note: declared here
  145 |    int getStateData(const std::string &fieldName, R &data);
      |        ^~~~~~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:301:17: warning: ‘int rokae::MotionControl<rokae::MotionControlMode::RtCommand>::getStateData(const string&, R&) [with R = std::array<double, 6>; std::string = std::__cxx11::basic_string<char>]’ is deprecated: Use BaseRobot::getStateData(fieldName, data) instead [-Wdeprecated-declarations]
  301 |             int tor_flag = rci_->getStateData("tau_m", robot_joint_torque);
      |                 ^~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/robot.h:18,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:19,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/motion_control_rt.h:145:8: note: declared here
  145 |    int getStateData(const std::string &fieldName, R &data);
      |        ^~~~~~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:302:17: warning: ‘int rokae::MotionControl<rokae::MotionControlMode::RtCommand>::getStateData(const string&, R&) [with R = std::array<double, 6>; std::string = std::__cxx11::basic_string<char>]’ is deprecated: Use BaseRobot::getStateData(fieldName, data) instead [-Wdeprecated-declarations]
  302 |             int ext_base = rci_->getStateData(rokae::RtSupportedFields::tauExt_inBase, robot_ext_force_base);
      |                 ^~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/robot.h:18,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:19,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/motion_control_rt.h:145:8: note: declared here
  145 |    int getStateData(const std::string &fieldName, R &data);
      |        ^~~~~~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:303:17: warning: ‘int rokae::MotionControl<rokae::MotionControlMode::RtCommand>::getStateData(const string&, R&) [with R = std::array<double, 6>; std::string = std::__cxx11::basic_string<char>]’ is deprecated: Use BaseRobot::getStateData(fieldName, data) instead [-Wdeprecated-declarations]
  303 |             int ext_stiff = rci_->getStateData(rokae::RtSupportedFields::tauExt_inStiff, robot_ext_force_stiff);
      |                 ^~~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/robot.h:18,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:19,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/motion_control_rt.h:145:8: note: declared here
  145 |    int getStateData(const std::string &fieldName, R &data);
      |        ^~~~~~~~~~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h: In instantiation of ‘bool rokae_hardware::RokaeHardwareInterface<DoF>::initRobot() [with short unsigned int DoF = 6]’:
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:73:18:   required from ‘bool rokae_hardware::RokaeHardwareInterface<DoF>::init(ros::NodeHandle&) [with short unsigned int DoF = 6]’
/home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:38:42:   required from here
/home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:173:17: warning: ‘void rokae::MotionControl<rokae::MotionControlMode::RtCommand>::startReceiveRobotState(const std::vector<std::__cxx11::basic_string<char> >&)’ is deprecated: Use Robot_T::startReceiveRobotState(interval, fields) instead [-Wdeprecated-declarations]
  173 |                 rci_->startReceiveRobotState({rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
      |                 ^~~~
In file included from /home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/robot.h:18,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/include/rokae_hardware/rokae_hardware.h:19,
                 from /home/ubuntu/caktin_ws/src/rokae_hardware/src/rokae_hardware.cpp:8:
/home/ubuntu/caktin_ws/src/rokae_hardware/sdk/include/rokae/motion_control_rt.h:115:9: note: declared here
  115 |    void startReceiveRobotState(const std::vector<std::string>& fields);
  
  
  [ INFO] [1733906954.047409127]: Ready to take commands for planning group motionplanning.
terminate called after throwing an instance of 'std::system_error'
  what():  cancel: 错误的文件描述符
[rokae_hardware-2] process has died [pid 53837, exit code -6, cmd /home/ubuntu/caktin_ws/devel/lib/rokae_hardware/rokae_hardware __name:=rokae_hardware __log:=/home/ubuntu/.ros/log/c8455a48-b79c-11ef-a535-eba93dc3bfef/rokae_hardware-2.log].
log file: /home/ubuntu/.ros/log/c8455a48-b79c-11ef-a535-eba93dc3bfef/rokae_hardware-2*.log


[rokae_hardware-2] process has died [pid 54850, exit code -11, cmd /home/ubuntu/caktin_ws/devel/lib/rokae_hardware/rokae_hardware __name:=rokae_hardware __log:=/home/ubuntu/.ros/log/200f09c2-b79d-11ef-a535-eba93dc3bfef/rokae_hardware-2.log].
log file: /home/ubuntu/.ros/log/200f09c2-b79d-11ef-a535-eba93dc3bfef/rokae_hardware-2*.log  

1.源码方式安装moveit和ompl库（Ubuntu18.04+ROS Melodic版本）
https://www.uudwc.com/A/0wP9/


  
