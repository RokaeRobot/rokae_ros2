//moveit同时作轨迹规划和轨迹下发执行
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <memory>
#include <cmath>
#include <thread>
#include <vector>

const std::string kControllerName = "joint_s_line";

class MoveJDemo : public rclcpp::Node
{
public:
    MoveJDemo()
    : Node("joint_s_line_node")
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
        // arm.setPoseReferenceFrame("xMateCR12_base");    //***_base 注释可用
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.2);
        arm.setGoalOrientationTolerance(0.2);
        arm.setMaxAccelerationScalingFactor(0.05);
        arm.setMaxVelocityScalingFactor(0.05);

        //std::vector<double> joint_target = {0, 0, 0, 0, 0, 0};   //六轴测试数据
        std::vector<double> joint_target = {1, 1, 1, 1, 1, 1};
        // std::vector<double> joint_target = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};   
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

    bool movej_sequence(const std::vector<std::vector<double>>& joint_targets)
    {
        auto& arm = *move_group_;

        arm.setPlanningTime(45.0);
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.2);
        arm.setGoalOrientationTolerance(0.2);
        arm.setMaxAccelerationScalingFactor(0.05);
        arm.setMaxVelocityScalingFactor(0.05);

        const size_t dof = arm.getCurrentJointValues().size();

        for (size_t i = 0; i < joint_targets.size(); ++i) {
            const auto& target = joint_targets[i];

            if (target.size() != dof) {
                RCLCPP_ERROR(this->get_logger(),
                    "第 %zu 个目标维度错误: 期望 %zu, 实际 %zu",
                    i + 1, dof, target.size());
                return false;
            }

            arm.setStartStateToCurrentState();
            arm.setJointValueTarget(target);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "第 %zu 个目标规划失败", i + 1);
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "第 %zu 个目标规划成功，开始执行", i + 1);
            auto result = arm.execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(this->get_logger(), "第 %zu 个目标执行失败，尝试重启控制器", i + 1);
                arm.stop();
                arm.clearPoseTargets();
                arm.setStartStateToCurrentState();
                reset_controller(kControllerName);
                return false;
            }

            rclcpp::sleep_for(std::chrono::milliseconds(300));
        }

        return true;
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
    node->set_move_group(move_group);

    // 再等一小会，保证 action/server/params 等被发现
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 5) 构造 6 个目标点并依次执行，执行完后程序结束
    const auto current = move_group->getCurrentJointValues();
    if (current.size() < 6) {
        RCLCPP_ERROR(node->get_logger(), "当前关节数不足 6，无法执行六点轨迹（实际: %zu）", current.size());
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    auto make_target = [&current](double j1, double j2, double j3, double j4, double j5, double j6) {
        auto t = current;
        t[0] = j1;
        t[1] = j2;
        t[2] = j3;
        t[3] = j4;
        t[4] = j5;
        t[5] = j6;
        return t;
    };

    std::vector<std::vector<double>> targets = {
        make_target(0.5,  0.5,  0.5,  0.5,  0.5,  0.5),
        make_target(0.7,  0.4,  0.3,  0.6,  0.4,  0.6),
        make_target(0.6,  0.2,  0.0,  0.5,  0.2,  0.4),
        make_target(0.3, -0.1, -0.2,  0.2, -0.1,  0.2),
        make_target(0.1, -0.2, -0.3,  0.0, -0.2,  0.0),
        make_target(0.0,  0.0,  0.0,  0.0,  0.0,  0.0)
    };

    bool ok = node->movej_sequence(targets);
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "顺序运动执行失败");
    }

    // 6) 结束：shutdown 并等待 spinner 退出
    rclcpp::shutdown();
    spinner.join();
    return 0;
}