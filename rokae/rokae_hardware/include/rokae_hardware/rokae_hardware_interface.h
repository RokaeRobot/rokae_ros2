#include <memory>
#include <string>
#include <vector>

// ROS interface
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

//#include <urdf/model.h>

#include <realtime_tools/realtime_publisher.h>

// ROS control interface
//hardware_interface 接口
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"



// #include "hardware_interface/joint_command_handle.hpp"
// #include "hardware_interface/joint_state_handle.hpp"
//控制器接口


#include <iostream>
// Rokae sdk
#include <rokae/robot.h>
#include <rokae/data_types.h>
//#include "rokae_msgs/rokae_msgs/msg/external_force.h"
#include "stdlib.h"


namespace rokae_hardware    //用override虚函数对基类SystemInterface的成员函数进行复写，否则子类RokaeHardwareInterface中的方法可能无法被ROS2中接口SystemInterface调用（）
{
    template <unsigned short DoF>
    class RokaeHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // RokaeHardwareInterface() = default;
        RCLCPP_SHARED_PTR_DEFINITIONS(RokaeHardwareInterface)
        RokaeHardwareInterface();
        virtual ~RokaeHardwareInterface()
        {
            // RCLCPP_INFO(rclcpp::get_logger("RokaeHardwareInterface"), "start() called");
            robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        }

        

        // ROS 2接口，使用rclcpp::Node::SharedPtr
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        // hardware_interface::return_type start();
        hardware_interface::return_type stop();

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        // 其它功能可以通过自定义函数实现
        bool initParameters(std::shared_ptr<rclcpp::Node> node);
        bool initRobot();
        bool waitForValidState(size_t max_attempts = 50, int delay_ms = 100);
        void setInitPosition();
        bool initROSInterface(std::shared_ptr<rclcpp::Node> node);
        void publishExternalForce();

        void enforceLimits(const rclcpp::Duration & period);
        void busy_wait(int milliseconds);

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        bool checkControllerClaims(const std::set<std::string> & claimed_resources);
        

        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        

    public: 
<<<<<<< Updated upstream
        std::shared_ptr<rokae::xMateRobot> robot_;     //连六轴机型
        // std::shared_ptr<rokae::xMateErProRobot> robot_;    //连七轴机型
=======
        // std::shared_ptr<rokae::xMateRobot> robot_;     //连六轴机型
        std::shared_ptr<rokae::xMateErProRobot> robot_;    //连七轴机型
>>>>>>> Stashed changes
        std::string robot_ip_;
        std::string local_ip_;
        std::error_code ec;
        //const → 值不可变 ；static → 所有对象共享一份
        static const size_t num_joints_ = DoF;   //const 表示这个变量是常量，初始化之后不能再修改，且只能初始化一次；static 表示这是一个 类变量，而不是某个对象独有的。没有 static 的话，就是 实例成员变量，每个对象都会单独保存一份 num_joints_。但轴数其实是由模板参数 DoF 决定的，不需要重复存储，所以用 static
        //size_t num_joints_ = DoF;
        //size_t num_joints_ = 7;
        std::vector<std::string> joint_names_;

        using RtType = rokae::RtMotionControl<rokae::WorkType::collaborative, DoF>;
        std::shared_ptr<RtType> rci_; //

        std::vector<double> joint_position_state_;
        std::vector<double> joint_velocity_state_;
        std::vector<double> joint_torque_state_;

        // External force
        std::vector<double> ext_force_in_stiff_;
        std::vector<double> ext_force_in_base_;

        // Commands
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_torque_command_;
        std::vector<double> internal_joint_position_command_;
         // 自定义失败标志位，需要外部调用者（比如控制器状态回调）设置
        std::atomic<bool> controller_aborted_{false};
        // 当前关节状态和命令缓存
        std::vector<double> state_positions_;
        std::vector<double> cmd_positions_;

        // Controller
        bool joint_position_controller_running_ = false;
        bool joint_velocity_controller_running_ = false;
        bool joint_torque_controller_running_ = false;
        bool controllers_initialized_ = false;

        long times_loop_ = 0;
        double planPeriod = 0.001;  ///servoj 1ms

        // ROS 2 Publisher示例
        // rclcpp::Publisher<rokae_msgs::msg::ExternalForce>::SharedPtr ext_force_in_stiff_pub_;
        // rclcpp::Publisher<rokae_msgs::msg::ExternalForce>::SharedPtr ext_force_in_base_pub_;

        // pvi
        long long time_us1 = 0;
        long long time_us2 = 0;
        unsigned long long getLocalTimeUs()
        {
            auto tp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
            return tp.time_since_epoch().count();
        }
    };
}
