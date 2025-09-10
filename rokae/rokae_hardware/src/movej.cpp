//只用moveit作轨迹规划，运动控制调用SDK接口
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <rokae/robot.h>
// #include <rokae/motion_control_rt.h>
// #include <rokae/data_types.h>

// #include <chrono>
// #include <thread>
// #include <vector>
// #include <memory>

// class MoveJOpenLoop : public rclcpp::Node
// {
// public:
//     MoveJOpenLoop()
//     : Node("movej_open_loop_node")
//     {
    
//     }

//     void run()
//     {
//         // ============= 初始化 SDK =============
//         std::error_code ec;
//         robot_ = std::make_shared<rokae::xMateRobot>("192.168.21.10", "192.168.21.131");
//         robot_->setOperateMode(rokae::OperateMode::automatic, ec);
//         robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
//         robot_->setPowerState(true, ec);
//         robot_->startReceiveRobotState(std::chrono::milliseconds(1),
//                                        {rokae::RtSupportedFields::jointPos_m});

//         // 创建 MoveGroupInterface
//         moveit::planning_interface::MoveGroupInterface arm(shared_from_this(), "rokae_arm");

//         arm.setPlanningTime(10.0);
//         arm.setMaxVelocityScalingFactor(0.2);
//         arm.setMaxAccelerationScalingFactor(0.2);

//         // ============= 设置目标点,进行轨迹规划而非真正运动 =============
//         std::vector<double> joint_target = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
//         //std::vector<double> joint_target = {0, 0, 0, 0, 0, 0};
//         arm.setJointValueTarget(joint_target);


//         // auto base_rci = robot_->getRtMotionController().lock();
//         // rci_ = std::dynamic_pointer_cast<RtType>(base_rci);
//         rci_ = robot_->getRtMotionController().lock();

//         // ============= 轨迹规划 =============
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

//         if (!success) {
//             RCLCPP_ERROR(this->get_logger(), "规划失败！");
//             return;
//         }

//         RCLCPP_INFO(this->get_logger(), "规划成功，开始开环执行...");

//         // ============= 手动下发轨迹点 (开环) =============
//         const auto &trajectory = plan.trajectory_.joint_trajectory;
//         // 记录轨迹起始参考时刻（wall-clock，steady）
//         auto start_wall = std::chrono::steady_clock::now();
//         // 将第 i 个点的 ROS Duration 转成 steady_clock 的绝对目标时间： start_wall + point.time_from_start
//         for (const auto &point : trajectory.points) {   //遍历轨迹点
//             // 检查 positions 长度（和 joint 数量）
//             if (point.positions.size() == 0) {
//                 RCLCPP_WARN(this->get_logger(), "轨迹点 positions 为空，跳过该点");
//                 continue;
//             }

//             std::array<double, 6> target_point{};
//             std::copy(point.positions.begin(), point.positions.end(), target_point.begin());

//             // 把 builtin_interfaces::msg::Duration 转成 rclcpp::Duration
//             rclcpp::Duration ros_dur(point.time_from_start);

//             // 用 nanoseconds() 得到 int64_t 纳秒数，转换为 chrono::nanoseconds
//             auto target_time = start_wall + std::chrono::nanoseconds(ros_dur.nanoseconds());

//             // 等待直到目标时间到达（如果目标时间已过，则立即发送）
//             auto now = std::chrono::steady_clock::now();
//             if (target_time > now) {
//                 std::this_thread::sleep_for(target_time - now);
//             }

//             // 下发命令（注意检查 SDK 接口是否是同步或异步、以及是否需要更高精度的发送方法）
//             rci_->MoveJ(0.1, robot_->jointPos(ec), target_point);
//             if (ec) {
//                 RCLCPP_ERROR(this->get_logger(), "下发命令失败: %s", ec.message().c_str());
//                 break;
//             }
//         }

//         RCLCPP_INFO(this->get_logger(), "轨迹下发完成（开环）。");
//     }

// private:
//     std::shared_ptr<rokae::xMateRobot> robot_;
//     using RtType = rokae::RtMotionControl<rokae::WorkType::collaborative, 6>;
//     std::shared_ptr<RtType> rci_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<MoveJOpenLoop>();
//     node->run();

//     rclcpp::shutdown();
//     return 0;
// }



//moveit同时作轨迹规划和轨迹下发执行
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <memory>
#include <cmath>

const std::string kControllerName = "position_joint_trajectory_controller";

class MoveJDemo : public rclcpp::Node
{
public:
    MoveJDemo()
    : Node("movej_demo_node")
    {
        controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
    }

    void set_move_group(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)    //mg 是智能指针类型的变量，用来接收外面传进来的智能指针
    {
        move_group_ = mg;
    }

    void movej()
    {
        auto& arm = *move_group_;   //*move_group_ 解引用 → 得到一个 MoveGroupInterface& 引用,因此arm本质是一个引用

        arm.setPlanningTime(45.0);
        arm.setPoseReferenceFrame("xMatePro3_base");    //xMateCR7_base，xMateSR4_base，base_link，xMatePro7_base,xMatePro3_base
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.2);
        arm.setGoalOrientationTolerance(0.2);
        arm.setMaxAccelerationScalingFactor(0.05);
        arm.setMaxVelocityScalingFactor(0.05);

        //std::vector<double> joint_target = {0, 0, 0, 0, 0, 0};   //六轴测试数据
        //std::vector<double> joint_target = {1, 1, 1, 1, 1, 1};
        std::vector<double> joint_target = {0, 0, 0, 0, 0, 0, 0};   
        //std::vector<double> joint_target = {1, 1, 1, 1, 1, 1, 1};    //七轴测试数据
        arm.setJointValueTarget(joint_target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "规划成功，正在执行...");
            auto result = arm.execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "执行失败，尝试重启控制器...");
            arm.stop();
            arm.clearPoseTargets();
            arm.setStartStateToCurrentState();
            reset_controller(kControllerName);
            rclcpp::sleep_for(std::chrono::seconds(2)); // 等待controller切换
            // if (!reset_controller(kControllerName)) {
            //     RCLCPP_ERROR(this->get_logger(), "控制器恢复失败，后续规划可能继续出错！");
            // }
        }

        } else {
            RCLCPP_ERROR(this->get_logger(), "关节空间规划失败！");
        }
    }

private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    bool reset_controller(const std::string& controller_name)
    {
        // 1. 检查服务是否可用
        if (!controller_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "服务 /controller_manager/switch_controller 不可用！");
            return false;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        // deactivate + activate 代替旧的 stop/start
        request->deactivate_controllers.push_back(controller_name);
        request->activate_controllers.push_back(controller_name);
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT; // 严格模式
        request->timeout = rclcpp::Duration::from_seconds(5.0); // 防止切换长时间卡住

        // 2. 发送切换请求
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

        // 3. 查询控制器状态
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
                    } else {
                        RCLCPP_INFO(this->get_logger(), "控制器 [%s] 当前状态: %s, 等待中...",
                                    controller_name.c_str(), ctrl.state.c_str());
                    }
                }
            }

            rclcpp::sleep_for(std::chrono::milliseconds(500)); // 等待再查
        }

        RCLCPP_ERROR(this->get_logger(), "控制器 [%s] 重启后未进入 active 状态！", controller_name.c_str());
        return false;
    }

};


//ROS2 的通信模型是 异步 的：收到消息不会自动执行用户代码，必须由 rclcpp 的 executor 去调度并执行回调。当调用 arm.plan() / arm.getCurrentState() 等，MoveIt 内部会等待 action/server 的返回或等待订阅到 /joint_states 的回调结果。这些返回/回调的处理依赖 executor 的 spin。
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 1) 创建 node
    auto node = std::make_shared<MoveJDemo>();

    // 2) 创建 executor 并把 node 加进去
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 3) 启动 spin 线程（executor 会不断调度回调）
    std::thread spinner([&executor]() {
        executor.spin();
    });

    // 可选：给 executor 一点时间去发现 action server / topics
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 4) 创建 MoveGroupInterface（或也可以提前创建，只要 spin 在运行即可）
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "rokae_arm");
    //auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "motionplanning");     //ER7 group为motionplanning
    node->set_move_group(move_group);

    // 再等一小会，保证 action/server/params 等被发现
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 5) 执行你的 blocking 流程（plan/execute）
    node->movej();

    // 6) 结束：shutdown 并等待 spinner 退出
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
