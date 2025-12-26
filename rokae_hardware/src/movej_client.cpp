// 文件名: simple_movej_client.cpp
#include <rclcpp/rclcpp.hpp>
#include <rokae_msgs/srv/move_j.hpp>
#include <iostream>

int main(int argc, char** argv) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = rclcpp::Node::make_shared("movej_client");
    
    // 创建MoveJ服务客户端
    auto movej_client = node->create_client<rokae_msgs::srv::MoveJ>("/rokae_driver/movej");
    
    // 等待服务可用（最多等待10秒）
    if (!movej_client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "MoveJ服务不可用");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "MoveJ服务已连接");
    
    // 创建请求消息
    auto request = std::make_shared<rokae_msgs::srv::MoveJ::Request>();
    
    // 设置目标关节角度为 [1,1,1,1,1,1] 弧度
    request->joint_positions = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    
    // 设置速度参数（根据你的回调函数，这是必需的）
    request->velocity = 0.2;  // 可以调整速度值，例如 0.1 表示10%的最大速度
    
    RCLCPP_INFO(node->get_logger(), "发送MoveJ请求");
    RCLCPP_INFO(node->get_logger(), "目标关节角度: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] rad",
                request->joint_positions[0], request->joint_positions[1],
                request->joint_positions[2], request->joint_positions[3],
                request->joint_positions[4], request->joint_positions[5]);
    RCLCPP_INFO(node->get_logger(), "速度: %.2f", request->velocity);
    
    // 发送请求并等待响应
    auto future = movej_client->async_send_request(request);
    
    // 等待服务响应（最多等待30秒，因为MoveJ可能需要时间执行）
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(30)) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        try {
            auto response = future.get();
            
            if (response->success) {
                RCLCPP_INFO(node->get_logger(), "MoveJ执行成功: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(node->get_logger(), "MoveJ执行失败: %s", response->message.c_str());
                return 1;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "异常: %s", e.what());
            return 1;
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "服务调用超时");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "程序完成");
    rclcpp::shutdown();
    return 0;
}