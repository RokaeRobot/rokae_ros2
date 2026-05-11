#include <vector>
#include <string>
#include <thread>
#include <fstream>  // 添加头文件
#include <iostream>
#include <sstream>
#include <chrono>
#include <cmath>
#include <algorithm>

// Rokae sdk
#include <rokae/robot.h>
#include <rokae/data_types.h>
#include <rokae/motion_control_rt.h>
//#include "rokae_msgs/include/rokae_msgs/rokae_msgs/msg/external_force.h"
#include "rokae_hardware/rokae_hardware_interface.h"
#include "pluginlib/class_list_macros.hpp"



namespace rokae_hardware
{
namespace
{
template <typename T>
std::string vector_to_string(const std::vector<T> &values)
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}
}  // namespace

    //using Rokae6DOFHardware = RokaeHardwareInterface<6>;
    //RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start() called");

template <unsigned short DoF>
RokaeHardwareInterface<DoF>::RokaeHardwareInterface()
//RokaeHardwareInterface::RokaeHardwareInterface()
{
    //RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start() called");

}


template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_init(const hardware_interface::HardwareInfo & info)   //controll manager控制器启动以后，configure参数配置才会被调用
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)   //controll manager控制器启动以后，configure参数配置才会被调用
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "rokae get joint start()");

    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Reading hardware parameters:");
    info_ = info;   //mlgb不保存 info 到成员变量info_读个卵子关节数量
    
    // 1. 直接从info_.joints获取关节名称（不要后续resize!）
    joint_names_.clear();
    for (const auto& joint : info_.joints) {
        joint_names_.push_back(joint.name);
    }
    std::cout << num_joints_ << std::endl;
    std::cout << joint_position_state_.size() << std::endl;
    //num_joints_ = joint_names_.size();

    joint_position_state_.resize(DoF,0.0);
    joint_velocity_state_.resize(DoF,0.0);
    joint_torque_state_.resize(DoF,0.0);

    // External force
    ext_force_in_stiff_.resize(DoF, 0.0);
    ext_force_in_base_.resize(DoF, 0.0);

    // Controller
    // joint_position_controller_running_ = false;
    // joint_velocity_controller_running_ = false;
    // //joint_torque_controller_running_ = false;
    // controllers_initialized_ = false;

    // Commands
    joint_position_command_.resize(DoF,0.0);
    joint_velocity_command_.resize(DoF,0.0);
    joint_torque_command_.resize(DoF,0.0);
    internal_joint_position_command_.resize(DoF,0.0);

    std::cout << DoF << std::endl;
    std::cout << joint_position_state_.size() << std::endl;


   
    return hardware_interface::CallbackReturn::SUCCESS;
}


// ROS2接口注册
//注册状态接口 - 告诉 ROS2 Control 框架硬件能够提供哪些状态数据（从硬件读取的数据）
template <unsigned short DoF>
std::vector<hardware_interface::StateInterface> RokaeHardwareInterface<DoF>::export_state_interfaces()
//std::vector<hardware_interface::StateInterface> RokaeHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start called export_state_interfaces");
    std::cout << DoF << std::endl;
    RCLCPP_INFO(
    rclcpp::get_logger("RokaeHardwareInterface"),
    "关节数量匹配! 期望: %u, 实际: %zu", DoF, joint_names_.size());   //特别小心，原先写成"关节数量匹配! 期望: %ld, 实际: %ld", DoF, joint_names_.size());很可能导致格式化参数和实际类型不匹配，产生内存破坏，进而引起 stack smashing detected。

    for (const auto& name : joint_names_)
    {
        RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Joint name: %s\n", name.c_str());
        std::cout << "Joint name: " << name << std::endl;
    }
    

    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (size_t i = 0; i < info_.joints.size(); ++i) {
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_state_[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_[i]));

    //state_interfaces.emplace_back(hardware_interface::StateInterface(
        //info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_torque_state_[i]));
  //}


    for (size_t i = 0; i < DoF; ++i) {
        RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Export state interface: %s/position", joint_names_[i].c_str());
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "position", &joint_position_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "velocity", &joint_velocity_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], "effort", &joint_torque_state_[i]));
    }
    return state_interfaces;
}

//注册命令接口 - 告诉 ROS2 Control 框架硬件能够接收哪些命令数据（发送给硬件的控制命令）
template <unsigned short DoF>
std::vector<hardware_interface::CommandInterface> RokaeHardwareInterface<DoF>::export_command_interfaces()
//std::vector<hardware_interface::CommandInterface> RokaeHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start called export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < DoF; ++i) {
        RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Export command interface: %s/position", joint_names_[i].c_str());
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], "position", &joint_position_command_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], "velocity", &joint_velocity_command_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], "effort", &joint_torque_command_[i]));
    }

    return command_interfaces;
}


template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_configure(const rclcpp_lifecycle::State & previous_state)    //const 是为了 保证函数内部不能修改 previous_state，它只允许读取对象当前的状态而不允许被修改  
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "rokae get robot_ip start()");

    auto it_robot = info_.hardware_parameters.find("robot_ip");
    if (it_robot != info_.hardware_parameters.end()) {
        robot_ip_ = it_robot->second;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "No 'robot_ip' found in hardware info.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 2. 获取 local_ip
    auto it_local = info_.hardware_parameters.find("local_ip");
    if (it_local != info_.hardware_parameters.end()) {
        local_ip_ = it_local->second;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "No 'local_ip' found in hardware info.");
        return hardware_interface::CallbackReturn::ERROR;
    }

        
    if (!initRobot()) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Robot initialization failed.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3. 打印检查
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("RokaeHardwareInterface"),
        "Using robot_ip: " << robot_ip_ << ", local_ip: " << local_ip_);

    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "on_configure() finished successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;

}


template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_activate(const rclcpp_lifecycle::State & previous_state)   
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)    //连接机器人，必须要通过ROS2中基类接口函数调用
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "on_activate() called");

    try {
        robot_->updateRobotState(std::chrono::milliseconds(1));
    } catch (const std::exception &e) {
        RCLCPP_WARN(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "on_activate: updateRobotState: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "on_activate: updateRobotState threw non-std exception");
    }

    const int ret_pos = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_position_state_);
    const int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocity_state_);
    const int ret_torque = robot_->getStateData("tau_m", joint_torque_state_);
    RCLCPP_INFO(
        rclcpp::get_logger("RokaeHardwareInterface"),
        "on_activate: initial RT getStateData ret pos=%d vel=%d tau=%d | joint_pos=%s | tau=%s",
        ret_pos, ret_vel, ret_torque,
        vector_to_string(joint_position_state_).c_str(),
        vector_to_string(joint_torque_state_).c_str());
    
    // 初始化命令为当前状态
    for (size_t i = 0; i < DoF; ++i) {
        joint_position_command_[i] = joint_position_state_[i];
        joint_velocity_command_[i] = joint_velocity_state_[i];
        joint_torque_command_[i] = joint_torque_state_[i];
    }
    setInitPosition();
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "Controller init successful!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

template <unsigned short DoF>
hardware_interface::CallbackReturn RokaeHardwareInterface<DoF>::on_deactivate(const rclcpp_lifecycle::State & previous_state)   
//保障机器人生命周期的完整
//hardware_interface::CallbackReturn RokaeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "on_deactivate() called");
    //rci_->stopLoop();    // 停掉 MotionControl 内部线程,确保生命周期结束前，SDK loop 已完全退出,防止ROS 2 硬件接口对象销毁后，SDK 内部线程仍然调用回调，导致 this 悬空访问，进而卡斯整个线程
    // 这里可以断开机器人连接，或清理资源
        if (rci_) {
        rci_->stopLoop();  // 停掉内部线程
        rci_.reset();      // 清空智能指针，避免析构后访问
    }

    // if (robot_) {
    //     robot_.reset();    // 确保 robot_ 也析构掉
    // }
    return hardware_interface::CallbackReturn::SUCCESS;
}


template <unsigned short DoF>
bool RokaeHardwareInterface<DoF>::initRobot()
//bool RokaeHardwareInterface::initRobot()
{
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start connect rokae");
    try {
        robot_ = std::make_shared<rokae::xMateRobot>(robot_ip_, local_ip_);   //连六轴机型
        // robot_ = std::make_shared<rokae::xMateErProRobot>(robot_ip_, local_ip_);     //连七轴机型
    } catch (const rokae::NetworkException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Robot instantiation failed: %s", e.what());
        return false;
    }

    try {
        robot_->connectToRobot(ec);
        if (ec.value() != 0)
            throw rokae::ExecutionException("connect failed");
    } catch (const rokae::ExecutionException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Robot connection failed: %s", e.what());
        return false;
    }

    try {
        robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        //robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        robot_->setPowerState(true, ec);
        if (ec.value() != 0)
            throw rokae::ExecutionException("connect failed");
    } catch (const rokae::ExecutionException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Could not enable robot: %s", e.what());
        return false;
    }

    try {
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        auto base_rci = robot_->getRtMotionController().lock();
        rci_ = std::dynamic_pointer_cast<RtType>(base_rci);
        rci_->setFilterLimit(true, 50);
        robot_->startReceiveRobotState(std::chrono::milliseconds(1), {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
                                      rokae::RtSupportedFields::tau_m, rokae::RtSupportedFields::tauExt_inBase,
                                      rokae::RtSupportedFields::tauExt_inStiff});
    } catch (const rokae::ExecutionException &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "Could not set realtime control mode: %s", e.what());
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "机器人连接成功");
    return true;
}

template <unsigned short DoF>
void RokaeHardwareInterface<DoF>::setInitPosition()
//void RokaeHardwareInterface::setInitPosition()
{
    auto robot_joint_postion = robot_->jointPos(ec);
    for (std::size_t i = 0; i < DoF; i++)
        joint_position_state_[i] = robot_joint_postion[i];
    joint_position_command_ = joint_position_state_;
}


// template <unsigned short DoF>
// hardware_interface::return_type RokaeHardwareInterface<DoF>::read(const rclcpp::Time&, const rclcpp::Duration&)
// //hardware_interface::return_type RokaeHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
// {
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "开始从机器人读取数据read() called");
   
//     int update_success = robot_->updateRobotState(std::chrono::milliseconds(1));  ///返回0或268

//     // 读取状态数据，getStateData 返回0通常表示成功

//     int ret_pos = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_position_state_);
//     int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocity_state_);
//     int ret_torque = robot_->getStateData("tau_m", joint_torque_state_);

//     if (ret_pos != 0) {
//         RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint position");
//         return hardware_interface::return_type::ERROR;
//     }
//     if (ret_vel != 0) {
//         RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint velocity");
//     }
//     if (ret_torque != 0) {
//         RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint torque");
//     }

//     internal_joint_position_command_ = joint_position_state_;
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "开始从机器人读取数据read() called");
//     return hardware_interface::return_type::OK;
template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::read(const rclcpp::Time&, const rclcpp::Duration &period)
{
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "开始从机器人读取数据read() called");
    static bool logged_first_successful_read = false;

    // 检查周期时间
    double period_ms = period.seconds() * 1000.0;
    if (period_ms>10) {
        RCLCPP_INFO(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "read周期: %.3fms", period_ms);
    }
   
    try {
        int update_success = robot_->updateRobotState(std::chrono::milliseconds(1));

        int ret_pos = robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_position_state_);
        int ret_vel = robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocity_state_);
        int ret_torque = robot_->getStateData("tau_m", joint_torque_state_);

        // Some robots (e.g. CR35) may leave RtSupportedFields::jointPos_m at ~0 while tau_m updates.
        // That would overwrite good values from setInitPosition()/jointPos(ec). Fall back to NRT jointPos.
        if (ret_pos == 0) {
            const bool rt_pos_all_near_zero = std::all_of(
                joint_position_state_.begin(), joint_position_state_.end(),
                [](double p) { return std::fabs(p) < 1e-8; });
            static auto last_jointpos_nrt_poll = std::chrono::steady_clock::now();
            static bool logged_jointpos_fallback = false;
            const auto now_poll = std::chrono::steady_clock::now();
            if (rt_pos_all_near_zero &&
                (now_poll - last_jointpos_nrt_poll) >= std::chrono::milliseconds(50)) {
                last_jointpos_nrt_poll = now_poll;
                try {
                    const auto jp = robot_->jointPos(ec);
                    for (size_t i = 0; i < DoF; ++i) {
                        joint_position_state_[i] = jp[i];
                    }
                    if (!logged_jointpos_fallback) {
                        logged_jointpos_fallback = true;
                        RCLCPP_WARN(
                            rclcpp::get_logger("RokaeHardwareInterface"),
                            "RT stream jointPos_m was all ~0; using jointPos(ec) for /joint_states positions "
                            "(polled every 50 ms while RT positions stay zero). Verify SDK RT fields for this model.");
                    }
                } catch (const std::exception &e) {
                    static auto last_jointpos_fallback_err = std::chrono::steady_clock::now();
                    const auto nw = std::chrono::steady_clock::now();
                    if (nw - last_jointpos_fallback_err >= std::chrono::seconds(5)) {
                        last_jointpos_fallback_err = nw;
                        RCLCPP_WARN(
                            rclcpp::get_logger("RokaeHardwareInterface"),
                            "jointPos(ec) fallback failed: %s", e.what());
                    }
                }
            }
        }

        if (!logged_first_successful_read && ret_pos == 0) {
            logged_first_successful_read = true;
            RCLCPP_INFO(
                rclcpp::get_logger("RokaeHardwareInterface"),
                "read: first successful getStateData (ret pos=%d vel=%d tau=%d update_ret=%d) "
                "joint_pos=%s joint_vel=%s tau=%s — per-cycle getStateData INFO logs disabled.",
                ret_pos, ret_vel, ret_torque, update_success,
                vector_to_string(joint_position_state_).c_str(),
                vector_to_string(joint_velocity_state_).c_str(),
                vector_to_string(joint_torque_state_).c_str());
        }
        
        if (ret_pos != 0) {
            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint position");
            return hardware_interface::return_type::ERROR;
        }
        if (ret_vel != 0) {
            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint velocity");
        }
        if (ret_torque != 0) {
            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "Failed to get joint torque");
        }

        internal_joint_position_command_ = joint_position_state_;
        RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "成功从机器人读取数据");

        return hardware_interface::return_type::OK;

    } catch (const rokae::RealtimeMotionException& e) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), 
                     "实时运动异常: %s", e.what());  
        static auto last_recover_try = std::chrono::steady_clock::now() - std::chrono::seconds(1);
        const auto now = std::chrono::steady_clock::now();
        if ((now - last_recover_try) < std::chrono::milliseconds(200))
        {
            return hardware_interface::return_type::OK;
        }
        last_recover_try = now;

        try
        {
            if (!robot_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "read恢复失败: robot_为空");
                return hardware_interface::return_type::ERROR;
            }

            robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
            if (ec.value() != 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                             "read恢复失败: setMotionControlMode错误: %s", ec.message().c_str());
                return hardware_interface::return_type::ERROR;
            }

            auto base_rci = robot_->getRtMotionController().lock();
            auto new_rci = std::dynamic_pointer_cast<RtType>(base_rci);
            if (!new_rci)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "read恢复失败: 获取RT控制器失败");
                return hardware_interface::return_type::ERROR;
            }
            rci_ = new_rci;

            setInitPosition();
            internal_joint_position_command_ = joint_position_state_;
            joint_position_command_ = joint_position_state_;
            joint_velocity_command_.assign(DoF, 0.0);
            joint_torque_command_.assign(DoF, 0.0);

            // Must stop any existing motion session before starting a new one.
            // Skipping this causes "已经开始运动, 请勿重复调用" from startMove().
            try { rci_->stopMove(); } catch (...) {}

            if (joint_position_controller_running_)
            {
                rci_->startMove(rokae::RtControllerMode::jointPosition);
            }
            else if (joint_torque_controller_running_)
            {
                rci_->startMove(rokae::RtControllerMode::torque);
            }

            RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "read阶段已自动恢复RT会话");
            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &recover_e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                         "read恢复异常: %s", recover_e.what());
            return hardware_interface::return_type::ERROR;
        }

    } 
}


// template <unsigned short DoF>
// hardware_interface::return_type RokaeHardwareInterface<DoF>::write(const rclcpp::Time&, const rclcpp::Duration &period)
// //hardware_interface::return_type RokaeHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration &period)
// {
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "开始向机器人写入数据write() called");
//     // enforceLimits(period); // 需要自己实现限幅逻辑
//     //robot_->updateRobotState(std::chrono::milliseconds(1));
    
//     RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "开始向机器人写入数据write() called");
//     return hardware_interface::return_type::OK;
// }

template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::write(const rclcpp::Time&, const rclcpp::Duration &period)
{
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "开始向机器人写入数据write() called");
    // 检查周期时间
    double period_ms = period.seconds() * 1000.0;
    if (period_ms>10) {
        RCLCPP_INFO(
            rclcpp::get_logger("RokaeHardwareInterface"),
            "write周期: %.3fms", period_ms);
    }
    
    // 只有位置控制器运行时才发送命令
    if (joint_position_controller_running_ ) {
        try {
            // 正常发送位置命令
            rokae::JointPosition jcmd(DoF);
            for (size_t i = 0; i < DoF; i++) {
                jcmd.joints[i] = joint_position_command_[i];
            }
            
            if (rci_) {
                rci_->sendCommand(jcmd);
                // busy_wait(planPeriod*1000);
            }
            
            RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), 
                        "发送位置命令: %s",
                        vector_to_string(joint_position_command_).c_str());
                        
        } 
        catch (const rokae::RealtimeMotionException& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                        "写入命令实时异常: %s", e.what());

            static auto last_recover_try = std::chrono::steady_clock::now() - std::chrono::seconds(1);
            const auto now = std::chrono::steady_clock::now();
            if ((now - last_recover_try) < std::chrono::milliseconds(200))
            {
                return hardware_interface::return_type::OK;
            }
            last_recover_try = now;

            try
            {
                if (!robot_)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "write恢复失败: robot_为空");
                    return hardware_interface::return_type::ERROR;
                }

                // Stop any existing motion before reconfiguring RT mode.
                // Wrap in try-catch because stopMove() may itself throw if motion
                // was never successfully started (e.g. after a failed read() recovery).
                if (rci_)
                {
                    try { rci_->stopMove(); } catch (...) {}
                }

                robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
                if (ec.value() != 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                                 "write恢复失败: setMotionControlMode错误: %s", ec.message().c_str());
                    return hardware_interface::return_type::ERROR;
                }

                auto base_rci = robot_->getRtMotionController().lock();
                auto new_rci = std::dynamic_pointer_cast<RtType>(base_rci);
                if (!new_rci)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), "write恢复失败: 获取RT控制器失败");
                    return hardware_interface::return_type::ERROR;
                }
                rci_ = new_rci;

                // Sync all command buffers to actual robot position so the next
                // write() does not send a stale mid-trajectory position command.
                setInitPosition();
                internal_joint_position_command_ = joint_position_state_;
                joint_position_command_ = joint_position_state_;
                joint_velocity_command_.assign(DoF, 0.0);
                joint_torque_command_.assign(DoF, 0.0);

                if (joint_position_controller_running_)
                {
                    rci_->startMove(rokae::RtControllerMode::jointPosition);
                }
                else if (joint_torque_controller_running_)
                {
                    rci_->startMove(rokae::RtControllerMode::torque);
                }

                RCLCPP_WARN(rclcpp::get_logger("RokaeHardwareInterface"), "write阶段已自动恢复RT会话");
                return hardware_interface::return_type::OK;
            }
            catch (const std::exception& recover_e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                            "write恢复异常: %s", recover_e.what());
                return hardware_interface::return_type::ERROR;
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"), 
                        "写入命令失败: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }
    
    return hardware_interface::return_type::OK;
}

// 控制器切换接口：ROS2标准
template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::prepare_command_mode_switch
(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
// hardware_interface::return_type RokaeHardwareInterface::prepare_command_mode_switch(
//     const std::vector<std::string> &start_interfaces,
//     const std::vector<std::string> &stop_interfaces)
{
    // 收集 start 的模式
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "prepare_command_mode_switch() called");
    std::vector<std::string> start_modes;
    for (const auto &key : start_interfaces) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const auto joint_full_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const auto joint_full_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
            const auto joint_full_eff = info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;
            if (key == joint_full_pos) start_modes.push_back("position");
            if (key == joint_full_vel) start_modes.push_back("velocity");
            if (key == joint_full_eff) start_modes.push_back("effort");
        }
    }

    // 要么全部不切换，要么每个关节都被赋予新模式
    if (!start_modes.empty() && start_modes.size() != info_.joints.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                     "prepare_command_mode_switch: start_modes size %zu != joints %zu",
                     start_modes.size(), info_.joints.size());
        return hardware_interface::return_type::ERROR;
    }

    // 所有 start_modes 必须相同
    if (!start_modes.empty() &&
        !std::equal(start_modes.begin() + 1, start_modes.end(), start_modes.begin())) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                     "prepare_command_mode_switch: start modes are not uniform");
        return hardware_interface::return_type::ERROR;
    }

    // stop 同理（要求一致）
    std::vector<std::string> stop_modes;
    for (const auto &key : stop_interfaces) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const auto joint_full_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
            const auto joint_full_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
            const auto joint_full_eff = info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;
            if (key == joint_full_pos) stop_modes.push_back("position");
            if (key == joint_full_vel) stop_modes.push_back("velocity");
            if (key == joint_full_eff) stop_modes.push_back("effort");
        }
    }
    if (!stop_modes.empty() && (stop_modes.size() != info_.joints.size() ||
        !std::equal(stop_modes.begin() + 1, stop_modes.end(), stop_modes.begin()))) {
        RCLCPP_ERROR(rclcpp::get_logger("RokaeHardwareInterface"),
                     "prepare_command_mode_switch: stop modes invalid");
        return hardware_interface::return_type::ERROR;
    }

    // 通过
    controllers_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"),
                "prepare_command_mode_switch: OK, start_interfaces=%zu stop_interfaces=%zu",
                start_interfaces.size(), stop_interfaces.size());
    return hardware_interface::return_type::OK;
}



template <unsigned short DoF>
hardware_interface::return_type RokaeHardwareInterface<DoF>::perform_command_mode_switch
(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
// hardware_interface::return_type RokaeHardwareInterface::perform_command_mode_switch(
//     const std::vector<std::string> &start_interfaces,
//     const std::vector<std::string> &stop_interfaces)
{
    // 停止所需接口（只需依据第一个 stop interface 类型执行即可）
    RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "perform_command_mode_switch() called");
    if (!stop_interfaces.empty()) {
        const auto &first = stop_interfaces.front();
        if (first.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
            joint_position_controller_running_ = false;
            if (rci_) rci_->stopMove();
        } else if (first.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
            joint_velocity_controller_running_ = false;
            if (rci_) rci_->stopMove();
        } else if (first.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
            joint_torque_controller_running_ = false;
            if (rci_) rci_->stopMove();
        }
    }

    // 启动新接口（依据第一个 start interface）
    if (!start_interfaces.empty()) {
        const auto &first = start_interfaces.front();
        if (first.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
            joint_position_controller_running_ = true;
            joint_velocity_controller_running_ = false;
            joint_torque_controller_running_ = false;
            if (rci_){
                setInitPosition();
                internal_joint_position_command_ = joint_position_state_;
                //启用servoj接口
                // rci_->setServoJoint(planPeriod,planPeriod*3,1,ec);
                rci_->startMove(rokae::RtControllerMode::jointPosition);
                
            }
        } else if (first.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
            joint_velocity_controller_running_ = true;
            joint_position_controller_running_ = false;
            joint_torque_controller_running_ = false;
            if (rci_) rci_->startMove(rokae::RtControllerMode::jointPosition);
        } else if (first.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
            joint_torque_controller_running_ = true;
            joint_position_controller_running_ = false;
            joint_velocity_controller_running_ = false;
            if (rci_) {
                rci_->stopMove();
                rci_->startMove(rokae::RtControllerMode::torque);
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"),
                "perform_command_mode_switch done. pos=%d vel=%d eff=%d",
                joint_position_controller_running_, joint_velocity_controller_running_,
                joint_torque_controller_running_);

    //启动时被激活[RokaeHardwareInterface]: perform_command_mode_switch done. pos=1 vel=0 eff=0

    if (joint_position_controller_running_) {
        //RCLCPP_DEBUG(rclcpp::get_logger("RokaeHardwareInterface"), "开始向机器人写入数据write() called");
        //std::function<rokae::JointPosition()> callback2 = std::bind(&RokaeHardwareInterface::callback, this);
        //rokae::JointPosition joints_value=callback2();
        
        // std::function<rokae::JointPosition()> callback = [this]()
        // {
        //     // joint_position_command_ 是由ROS2的控制器通过命令接口设置的，所以理论上，只要控制器更新了 joint_position_command_，回调函数中就会使用最新的值
        //     // RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"),"使用回调函数");
        //     rokae::JointPosition jcmd(num_joints_);
        //     for (size_t i = 0; i < num_joints_; i++)
        //     {
        //         jcmd.joints[i] = joint_position_command_[i];
        //     }
        //     this->time_us2 = time_us1;
        //     this->time_us1 = getLocalTimeUs();
        //     //std::this_thread::sleep_for(std::chrono::microseconds(50));
        //     return jcmd;
        // };
        // this->rci_->setControlLoop(callback, 0, true);
        // this->rci_->startLoop(false);
    }
    else if (joint_velocity_controller_running_) {
        //for (std::size_t i = 0; i < num_joints_; i++)
            //internal_joint_position_command_[i] += joint_velocity_command_[i] * period.seconds();
    }
    else if (joint_torque_controller_running_) {
        // rci_->appendCommand(rokae::Torque(joint_torque_command_));
    }

    return hardware_interface::return_type::OK;
}



// 力控限幅（需自己实现逻辑）
template <unsigned short DoF>
void RokaeHardwareInterface<DoF>::enforceLimits(const rclcpp::Duration& period)
//void RokaeHardwareInterface::enforceLimits(const rclcpp::Duration& period)
{
    // TODO: 用joint_limits_interface或自定义限幅逻辑
}

// 发布力控（推荐用rclcpp::Publisher，自己加锁）
template <unsigned short DoF>
void RokaeHardwareInterface<DoF>::publishExternalForce()
//void RokaeHardwareInterface::publishExternalForce()
{
    // TODO: rclcpp::Publisher<rokae_msgs::msg::ExternalForce>::SharedPtr
}

//servoj 接口
template<unsigned short DoF>
void RokaeHardwareInterface<DoF>::busy_wait(int milliseconds) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start).count() < milliseconds) {
    }
}



template class RokaeHardwareInterface<5>;
template class RokaeHardwareInterface<6>;
template class RokaeHardwareInterface<7>;

using RokaeHardwareInterface5 = RokaeHardwareInterface<5>;
using RokaeHardwareInterface6 = RokaeHardwareInterface<6>;
using RokaeHardwareInterface7 = RokaeHardwareInterface<7>;

} // namespace rokae_hardware

PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface5,hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface6,hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface7,hardware_interface::SystemInterface)
//PLUGINLIB_EXPORT_CLASS(rokae_hardware::RokaeHardwareInterface, hardware_interface::SystemInterface)
