//moveit同时作轨迹规划和轨迹下发执行
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <memory>
#include <cmath>
#include <thread>
#include <algorithm>

const std::string kControllerName = "joint_position_controller";
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class MoveJDemo : public rclcpp::Node
{
public:
    MoveJDemo()
    : Node("joint_position_control_node")
    {
        controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
        trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/position_joint_trajectory_controller/follow_joint_trajectory");
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

    bool execute_cosine_s_curve_trajectory(
        double t1,
        double t2,
        double t3,
        double dt,
        const std::vector<int>& signs)
    {
        if (!move_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 未初始化");
            return false;
        }

        if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "FollowJointTrajectory action server 不可用");
            return false;
        }

        const auto joint_names = move_group_->getJointNames();
        const auto jnt_pos = move_group_->getCurrentJointValues();
        const size_t dof = jnt_pos.size();

        if (joint_names.size() != dof || dof == 0) {
            RCLCPP_ERROR(this->get_logger(), "关节名与关节位置维度不一致或为空");
            return false;
        }

        if (dof < signs.size()) {
            RCLCPP_ERROR(this->get_logger(), "自由度不足，期望至少 %zu，实际 %zu", signs.size(), dof);
            return false;
        }

        auto make_duration_msg = [](double seconds) {
            builtin_interfaces::msg::Duration d;
            const int32_t sec = static_cast<int32_t>(std::floor(seconds));
            const double frac = seconds - static_cast<double>(sec);
            d.sec = sec;
            d.nanosec = static_cast<uint32_t>(frac * 1e9);
            return d;
        };

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.2);
        traj.joint_names = joint_names;

        if (t1 <= 0.0 || t2 <= 0.0 || t3 <= 0.0 || dt <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "轨迹时间参数非法，要求 t1/t2/t3/dt > 0");
            return false;
        }

        const double amplitude = M_PI / 20.0;
        const double total_time = t1 + t2 + t3;
        const int steps = static_cast<int>(std::ceil(total_time / dt));

        traj.points.reserve(static_cast<size_t>(steps) + 1);

        for (int i = 0; i <= steps; ++i) {
            const double t = std::min(i * dt, total_time);

            double q_start = 0.0;
            double q_end = 0.0;
            double seg_t = 0.0;
            double seg_T = 0.0;

            if (t <= t1) {
                q_start = 0.0;
                q_end = amplitude;
                seg_t = t;
                seg_T = t1;
            } else if (t <= (t1 + t2)) {
                q_start = amplitude;
                q_end = -amplitude;
                seg_t = t - t1;
                seg_T = t2;
            } else {
                q_start = -amplitude;
                q_end = 0.0;
                seg_t = t - t1 - t2;
                seg_T = t3;
            }

            const double tau = std::clamp(seg_t / seg_T, 0.0, 1.0);
            const double s = 0.5 * (1.0 - std::cos(M_PI * tau));
            const double s_dot = 0.5 * M_PI / seg_T * std::sin(M_PI * tau);
            const double s_ddot = 0.5 * (M_PI * M_PI) / (seg_T * seg_T) * std::cos(M_PI * tau);

            const double delta = q_start + (q_end - q_start) * s;
            const double vel = (q_end - q_start) * s_dot;
            const double acc = (q_end - q_start) * s_ddot;

            trajectory_msgs::msg::JointTrajectoryPoint p;
            p.positions = jnt_pos;
            p.velocities.assign(dof, 0.0);
            p.accelerations.assign(dof, 0.0);

            for (size_t j = 0; j < signs.size(); ++j) {
                const double s = static_cast<double>(signs[j]);
                p.positions[j] = jnt_pos[j] + s * delta;
                p.velocities[j] = s * vel;
                p.accelerations[j] = s * acc;
            }

            p.time_from_start = make_duration_msg(t);
            traj.points.push_back(std::move(p));
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory = std::move(traj);
        goal.goal_time_tolerance = make_duration_msg(0.5);

        auto goal_future = trajectory_action_client_->async_send_goal(goal);
        if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "发送轨迹 goal 超时");
            return false;
        }

        auto goal_handle = goal_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "轨迹 goal 被拒绝");
            return false;
        }

        auto result_future = trajectory_action_client_->async_get_result(goal_handle);
        const auto wait_timeout = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(total_time + 10.0));

        if (result_future.wait_for(wait_timeout) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "等待轨迹执行结果超时");
            return false;
        }

        const auto wrapped_result = result_future.get();
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "轨迹执行失败，action result code: %d", static_cast<int>(wrapped_result.code));
            return false;
        }

        if (wrapped_result.result && wrapped_result.result->error_code != 0) {
            RCLCPP_ERROR(this->get_logger(), "控制器返回错误码: %d", wrapped_result.result->error_code);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "三段余弦轨迹执行完成（0 -> +A -> -A -> 0）");
        return true;
    }

    bool move_to_pre_position(const std::vector<double>& first_six_target)
    {
        if (!move_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 未初始化");
            return false;
        }

        auto& arm = *move_group_;
        const auto current = arm.getCurrentJointValues();
        const size_t dof = current.size();

        if (dof < first_six_target.size()) {
            RCLCPP_ERROR(this->get_logger(), "自由度不足，期望至少 %zu，实际 %zu", first_six_target.size(), dof);
            return false;
        }

        auto target = current;
        for (size_t i = 0; i < first_six_target.size(); ++i) {
            target[i] = first_six_target[i];
        }

        arm.setPlanningTime(20.0);
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.01);
        arm.setGoalOrientationTolerance(0.01);
        arm.setMaxAccelerationScalingFactor(0.1);
        arm.setMaxVelocityScalingFactor(0.1);
        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "预位姿规划失败");
            return false;
        }

        if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "预位姿执行失败");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "已到达预位姿 (0.5, 0.5, 0.5, 0.5, 0.5, 0.5)");
        return true;
    }

private:
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
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

    // 5) 先移动到预位姿
    const std::vector<double> pre_position = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    bool ok = node->move_to_pre_position(pre_position);
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "移动到预位姿失败");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    // 6) 一次性下发三段余弦轨迹：0 -> +A -> -A -> 0
    const std::vector<int> signs = {+1, +1, -1, +1, -1, +1};
    ok = node->execute_cosine_s_curve_trajectory(
        2.0,   // t1: 0 -> +A
        2.0,   // t2: +A -> -A
        2.0,   // t3: -A -> 0
        0.02,  // dt
        signs
    );
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "顺序运动执行失败");
    }

    // 6) 结束：shutdown 并等待 spinner 退出
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
