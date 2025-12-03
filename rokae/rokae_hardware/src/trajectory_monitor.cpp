// trajectory_monitor_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>  // 改为 MoveIt 的轨迹消息
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <string>

class TrajectoryMonitor : public rclcpp::Node
{
public:
    TrajectoryMonitor() : Node("trajectory_monitor")
    {
        // 声明参数：关节限位
        this->declare_parameter<std::vector<double>>("joint_limits.lower", std::vector<double>{-3.04, -3.04, -3.04, -3.04, -3.04, -3.04});
        this->declare_parameter<std::vector<double>>("joint_limits.upper", std::vector<double>{3.04, 3.04, 3.04, 3.04, 3.04, 3.04});
        
        // 获取参数
        lower_limits_ = this->get_parameter("joint_limits.lower").as_double_array();
        upper_limits_ = this->get_parameter("joint_limits.upper").as_double_array();
        
        // 订阅 MoveIt 的显示轨迹话题
        display_trajectory_sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 10,  // MoveIt 默认发布的话题
            std::bind(&TrajectoryMonitor::displayTrajectoryCallback, this, std::placeholders::_1));
            
        // 发布安全状态
        safety_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trajectory_safety", 10);
        
        // 发布详细警告
        warning_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trajectory_warning", 10);
        
        RCLCPP_INFO(this->get_logger(), "轨迹监控节点已启动（监听MoveIt规划轨迹）");
        RCLCPP_INFO(this->get_logger(), "关节限位: [%.3f, %.3f] x %zu", 
                   lower_limits_[0], upper_limits_[0], lower_limits_.size());
    }

private:
    void displayTrajectoryCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
    {
        if (msg->trajectory.empty()) {
            RCLCPP_WARN(this->get_logger(), "收到空的轨迹消息");
            return;
        }

        // 取第一条轨迹（通常只有一条）
        const auto& robot_trajectory = msg->trajectory[0];
        const auto& joint_trajectory = robot_trajectory.joint_trajectory;

        // 打印轨迹信息
        // printTrajectoryInfo(robot_trajectory);

        bool is_safe = true;
        bool has_warning = false;
        
        // 检查每个轨迹点
        for (size_t i = 0; i < joint_trajectory.points.size(); ++i) {
            const auto& point = joint_trajectory.points[i];
            
            // 检查每个关节的位置
            for (size_t j = 0; j < point.positions.size() && j < lower_limits_.size(); ++j) {
                double position = point.positions[j];
                
                if (position < lower_limits_[j] || position > upper_limits_[j]) {
                    is_safe = false;
                    RCLCPP_WARN(this->get_logger(), 
                               "轨迹点 %zu, 关节 %zu 位置 %f 超出限位 [%f, %f]", 
                               i, j, position, lower_limits_[j], upper_limits_[j]);
                }
                
                // 检查是否接近限位（警告级别）
                double margin = 0.1; // 0.1弧度作为安全边界
                if (position < lower_limits_[j] + margin || position > upper_limits_[j] - margin) {
                    has_warning = true;
                    if (is_safe) { // 只在安全的情况下记录警告
                        RCLCPP_WARN(this->get_logger(), 
                                   "轨迹点 %zu, 关节 %zu 位置 %f 接近限位边界", 
                                   i, j, position);
                    }
                }
            }
        }
        
        // 发布安全状态
        std_msgs::msg::Bool safety_msg;
        safety_msg.data = is_safe;
        safety_pub_->publish(safety_msg);
        
        // 发布警告状态
        std_msgs::msg::Bool warning_msg;
        warning_msg.data = has_warning && is_safe; // 只有安全但接近边界时才警告
        warning_pub_->publish(warning_msg);
        
        // 记录结果
        if (!is_safe) {
            RCLCPP_ERROR(this->get_logger(), "检测到不安全的轨迹！");
        } else if (has_warning) {
            RCLCPP_WARN(this->get_logger(), "轨迹安全但接近限位边界");
        } else {
            RCLCPP_INFO(this->get_logger(), "轨迹安全");
        }
    }

    // // 打印轨迹详细信息
    // void printTrajectoryInfo(const moveit_msgs::msg::RobotTrajectory& trajectory)
    // {
    //     const auto& joint_trajectory = trajectory.joint_trajectory;
        
    //     RCLCPP_INFO(this->get_logger(), "========== 轨迹信息 ==========");
    //     RCLCPP_INFO(this->get_logger(), "轨迹点数量: %zu", joint_trajectory.points.size());
    //     RCLCPP_INFO(this->get_logger(), "控制关节数: %zu", joint_trajectory.joint_names.size());
        
    //     // 打印关节名称
    //     RCLCPP_INFO(this->get_logger(), "关节名称:");
    //     for (size_t i = 0; i < joint_trajectory.joint_names.size(); i++) {
    //         RCLCPP_INFO(this->get_logger(), "  %zu: %s", i, joint_trajectory.joint_names[i].c_str());
    //     }

    //     // 打印关键轨迹点信息
    //     if (!joint_trajectory.points.empty()) {
    //         // 起始点
    //         const auto& start_point = joint_trajectory.points.front();
    //         RCLCPP_INFO(this->get_logger(), "起始点 (时间: %.3fs):", 
    //                    start_point.time_from_start.sec + start_point.time_from_start.nanosec * 1e-9);
    //         for (size_t i = 0; i < start_point.positions.size(); i++) {
    //             RCLCPP_INFO(this->get_logger(), "  关节%s: %.6f rad", 
    //                        joint_trajectory.joint_names[i].c_str(), start_point.positions[i]);
    //         }

    //         // 中间点（如果有多个点，打印几个关键中间点）
    //         if (joint_trajectory.points.size() > 2) {
    //             size_t mid_index = joint_trajectory.points.size() / 2;
    //             const auto& mid_point = joint_trajectory.points[mid_index];
    //             RCLCPP_INFO(this->get_logger(), "中间点 (时间: %.3fs):", 
    //                        mid_point.time_from_start.sec + mid_point.time_from_start.nanosec * 1e-9);
    //             for (size_t i = 0; i < mid_point.positions.size(); i++) {
    //                 RCLCPP_INFO(this->get_logger(), "  关节%s: %.6f rad", 
    //                            joint_trajectory.joint_names[i].c_str(), mid_point.positions[i]);
    //             }
    //         }

    //         // 结束点
    //         const auto& end_point = joint_trajectory.points.back();
    //         RCLCPP_INFO(this->get_logger(), "结束点 (时间: %.3fs):", 
    //                    end_point.time_from_start.sec + end_point.time_from_start.nanosec * 1e-9);
    //         for (size_t i = 0; i < end_point.positions.size(); i++) {
    //             RCLCPP_INFO(this->get_logger(), "  关节%s: %.6f rad", 
    //                        joint_trajectory.joint_names[i].c_str(), end_point.positions[i]);
    //         }
    //     }
        
        // RCLCPP_INFO(this->get_logger(), "========== 轨迹信息结束 ==========");
    // }
    
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr warning_pub_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}