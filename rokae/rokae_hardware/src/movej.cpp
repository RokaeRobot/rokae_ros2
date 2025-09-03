#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
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
        arm.setPoseReferenceFrame("xMateCR7_base");
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.2);
        arm.setGoalOrientationTolerance(0.2);
        arm.setMaxAccelerationScalingFactor(0.1);
        arm.setMaxVelocityScalingFactor(0.1);

        std::vector<double> joint_target = {1, 1, 1, 1, 1, 1};
        //std::vector<double> joint_target = {1, 1, 1, 1, 1, 1, 1};
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
        if (!controller_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "服务 /controller_manager/switch_controller 不可用！");
            return false;
        }

        // auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        // request->stop_controllers.push_back(controller_name);
        // request->start_controllers.push_back(controller_name);
        // request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->deactivate_controllers.push_back(controller_name);
        request->activate_controllers.push_back(controller_name);
        request->start_controllers.push_back(controller_name);
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;


        auto future = controller_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败！");
            return false;
        }

        if (!future.get()->ok) {
            RCLCPP_WARN(this->get_logger(), "控制器切换失败，可能资源被占用！");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "控制器重启成功！");
        return true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 创建 Node 的 shared_ptr
    auto node = std::make_shared<MoveJDemo>();

    // 只有现在 Node 是 shared_ptr 后，才能安全创建 MoveGroupInterface
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "rokae_arm");

    // 设置 MoveGroup 对象
    node->set_move_group(move_group);

    // 执行运动
    node->movej();

    rclcpp::shutdown();
    return 0;
}
