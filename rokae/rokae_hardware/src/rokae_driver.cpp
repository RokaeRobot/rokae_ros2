#include <rclcpp/rclcpp.hpp>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rokae_msgs/srv/move_j.hpp"   //报错去install中找对应的名字
#include "rokae_msgs/srv/move_c.hpp"
#include "rokae_msgs/srv/move_l.hpp"
#include "rokae_msgs/srv/get_robot_info.hpp"
#include "rokae_msgs/srv/calculate_ik.hpp"
#include "rokae_msgs/srv/calculate_fk.hpp"
#include "rokae_msgs/srv/drag_con.hpp"
#include "rokae_msgs/srv/jog_con.hpp"
#include "rokae_msgs/srv/get_do.hpp"
#include "rokae_msgs/srv/set_do.hpp"
#include "rokae_msgs/srv/get_di.hpp"
#include "rokae_msgs/srv/set_di.hpp"
#include "rokae_msgs/srv/read_register.hpp"
#include "rokae_msgs/srv/write_register.hpp"


using namespace rokae;
using sensor_msgs::msg::JointState;


rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_pub;

// 创建机器人对象
rokae::xMateRobot robot; ////六轴
// rokae::xMateErProRobot robot; //// 七轴
std::error_code ec;
const std::string local_ip = "192.168.2.100";
const std::string robot_ip = "192.168.2.160";



//  基础信息查询服务
bool get_robot_info_callback(
    const std::shared_ptr<rokae_msgs::srv::GetRobotInfo::Request> /*request*/,
    std::shared_ptr<rokae_msgs::srv::GetRobotInfo::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "get_robot_info called");
        
        auto robot_info = robot.robotInfo(ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to get robot info: " + ec.message();
            return false;
        }
        
        response->success = true;
        response->message = "Robot info retrieved successfully";
        response->version = robot_info.version;
        response->type = robot_info.type;
        response->sdk_version = robot.sdkVersion();
        
        // // 获取当前关节状态
        // auto joint_pos = robot.jointPos(ec);
        // if (!ec) {
        //     response->joint_positions.assign(joint_pos.begin(), joint_pos.end());
        // }
        
        // // 获取当前笛卡尔位姿
        // auto posture = robot.posture(CoordinateType::flangeInBase, ec);
        // if (!ec) {
        //     response->cartesian_pose.assign(posture.begin(), posture.end());
        // }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

// 开启关闭拖动    需要提前设置超级管理员
bool drag_control_callback(
    const std::shared_ptr<rokae_msgs::srv::DragCon::Request> request,
    std::shared_ptr<rokae_msgs::srv::DragCon::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "drag_control called");
        if (request->command == "on") {  
            // 开启拖拽模式
            
            robot.setOperateMode(rokae::OperateMode::manual, ec);
            robot.setPowerState(false, ec);
            robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
            RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "Drag enabled successfully");
            response->success = true;
            response->message = "Drag enabled successfully";

            
        } else if (request->command == "off") {
            // 关闭拖拽模式
            
            robot.disableDrag(ec);
            RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "Drag disabled successfully");
            response->success = true; 
            response->message = "Drag disabled successfully";
            
        } else {
            // 命令无效
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), 
                       "Invalid command: %s. Expected 'on' or 'off'", 
                       request->command.c_str());
            response->success = false;
            response->message = "Invalid command. Use 'on' or 'off'";
        }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;  // 服务调用完成
}



// 正逆运动学计算服务  弧度
bool calculate_ik_callback(
    const std::shared_ptr<rokae_msgs::srv::CalculateIK::Request> request,
    std::shared_ptr<rokae_msgs::srv::CalculateIK::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "calculate_ik called");
        
        if (request->target_pose.size() != 6) {
            response->success = false;
            response->message = "Target pose must have 6 elements [x, y, z, rx, ry, rz]";
            return false;
        }
        
        std::array<double, 6> target_pose;
        std::copy(request->target_pose.begin(), request->target_pose.end(), target_pose.begin());
        
        auto model = robot.model();
        auto ik_result = model.calcIk(target_pose, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to calculate inverse kinematics: " + ec.message();
            return false;
        }
        
        response->success = true;
        response->message = "IK calculated successfully";
        response->joint_positions.assign(ik_result.begin(), ik_result.end());
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


bool calculate_fk_callback(
    const std::shared_ptr<rokae_msgs::srv::CalculateFK::Request> request,
    std::shared_ptr<rokae_msgs::srv::CalculateFK::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "calculate_fk called");
        
        if (request->joint_positions.size() != 6) {
            response->success = false;
            response->message = "Joint positions must have 6 elements";
            return true;
        }
        
        std::array<double, 6> joint_positions;
        std::copy(request->joint_positions.begin(), request->joint_positions.end(), joint_positions.begin());
        
        auto model = robot.model();
        CartesianPosition fk_result = model.calcFk(joint_positions, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to calculate forward kinematics: " + ec.message();
            return true;
        }
        
        response->success = true;
        response->message = "FK calculated successfully";
        
        // 使用循环添加元素   使用fk_result.pos结果为0
        response->cartesian_pose.clear();
        for (int i = 0; i < 3; i++) {
            response->cartesian_pose.push_back(fk_result.trans[i]);
        }
        for (int i = 0; i < 3; i++){
            response->cartesian_pose.push_back(fk_result.rpy[i]);
        }

    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return true;
    }
    
    return true;
}






// Jog控制服务  参考JogCon.srv
bool jog_control_callback(
    const std::shared_ptr<rokae_msgs::srv::JogCon::Request> request,
    std::shared_ptr<rokae_msgs::srv::JogCon::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "jog_control called");
        
        // 参数验证
        if (request->rate < 0.01 || request->rate > 1.0) {
            response->success = false;
            response->message = "Rate must be between 0.01 and 1.0";
            return true;
        }
        
        if (request->step <= 0) {
            response->success = false;
            response->message = "Step must be greater than 0";
            return true;
        }
        
        // 映射坐标系类型
        JogOpt::Space jog_space;
        
        if (request->space == "world") {
            jog_space = JogOpt::world;
        } else if (request->space == "joint") {
            jog_space = JogOpt::jointSpace;
        } else if (request->space == "tool") {
            jog_space = JogOpt::toolFrame;
        } else if (request->space == "flange") {
            jog_space = JogOpt::flange;
        } else if (request->space == "singularityAvoidMode") {
            jog_space = JogOpt::singularityAvoidMode;
        } else if (request->space == "baseParallelMode") {
            jog_space = JogOpt::baseParallelMode;
        } else if (request->space == "singularityAvoidMode"){
            jog_space = JogOpt::singularityAvoidMode;
        }else if (request->space == "baseFrame"){
            jog_space = JogOpt::baseFrame;
        }else {
            response->success = false;
            response->message = "Invalid space. Use: world, joint, tool, flange, singularityAvoidMode, baseParallelMode";
            return true;
        }
        
        // 切换到手动模式并上电
        robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot.setOperateMode(rokae::OperateMode::manual, ec);
        robot.setPowerState(true, ec);
        
        // RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
        //            "Starting jog: space=%s, rate=%.2f, step=%.2f, index=%d, direction=%s",
        //            request->space.c_str(), request->rate, request->step, 
        //            request->index, request->direction ? "positive" : "negative");
        
        // 启动Jog（单步模式）
        robot.startJog(jog_space, request->rate, request->step, request->index, request->direction, ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to start jog: " + ec.message();
            return true;
        }
        

        float wait_time =10.0; //先随机给10s等待
        // RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
        //            "Waiting %.2f seconds for jog to complete", wait_time);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time * 1000)));
        
        // 停止Jog（必须调用）
        robot.stop(ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to stop jog: " + ec.message();
            return true;
        }
        
        response->success = true;
        response->message = "Jog completed successfully";
        
    } catch (const std::exception& e) {
        // 确保在异常情况下也停止Jog
        robot.stop(ec);
        
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;  // 服务调用完成
}


// /*IO相关*/

//  获取数字输出状态  hmi中通信--系统IO设置
bool get_do_callback(
    const std::shared_ptr<rokae_msgs::srv::GetDO::Request> request,
    std::shared_ptr<rokae_msgs::srv::GetDO::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "get_do called - board: %u, port: %u", 
                   request->board, request->port);
        
        bool state = robot.getDO(request->board, request->port, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to get DO state: " + ec.message();
            return true;
        }
        
        response->state = state;
        response->success = true;
        response->message = state ? "DO is ON" : "DO is OFF";
    
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

//DO信号不存在或为系统输出
bool set_do_callback(
    const std::shared_ptr<rokae_msgs::srv::SetDO::Request> request,
    std::shared_ptr<rokae_msgs::srv::SetDO::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "set_do called - board: %u, port: %u, state: %s", 
                   request->board, request->port, 
                   request->state ? "true" : "false");
        
        robot.setDO(request->board, request->port, request->state, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to set DO: " + ec.message();
            return true;
        }
        
        response->success = true;
        response->message = "DO set to " + std::string(request->state ? "ON" : "OFF");
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


// 设置数字输入信号
bool set_di_callback(
    const std::shared_ptr<rokae_msgs::srv::SetDI::Request> request,
    std::shared_ptr<rokae_msgs::srv::SetDI::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "set_di called - board: %u, port: %u, state: %s", 
                   request->board, request->port, 
                   request->state ? "true" : "false");
        
        robot.setSimulationMode(true, ec);
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to enable simulation mode: " + ec.message();
            return true;
        }
        
        robot.setDI(request->board, request->port, request->state, ec); 
        robot.setSimulationMode(false, ec);

        response->success = true;
        response->message = "DI set to " + std::string(request->state ? "ON" : "OFF");
        
    } catch (const std::exception& e) {
        // 确保在异常情况下也尝试关闭仿真模式
        robot.setSimulationMode(false, ec);
        
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


// 获取数字输入状态
bool get_di_callback(
    const std::shared_ptr<rokae_msgs::srv::GetDI::Request> request,
    std::shared_ptr<rokae_msgs::srv::GetDI::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), 
                   "get_di called - board: %u, port: %u", 
                   request->board, request->port);

        robot.setSimulationMode(true, ec);
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        
        bool state = robot.getDI(request->board, request->port, ec);
        
        if (ec) {
            response->success = false;
            response->message = "Failed to get DI state: " + ec.message();
            return true;
        }
        
        response->state = state;
        response->success = true;
        response->message = state ? "DI is ON" : "DI is OFF";

        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}


bool read_register_callback(
    const std::shared_ptr<rokae_msgs::srv::ReadRegister::Request> request,
    std::shared_ptr<rokae_msgs::srv::ReadRegister::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "read_register called");
        
        // 清除之前的错误码
        ec.clear();
        
        // 根据数据类型分支处理
        if (request->data_type == "float") {
            if (request->read_all) {
                std::vector<float> values;
                robot.readRegister(request->register_name, 0, values, ec);
                if (!ec) {
                    response->float_values = values;
                }
            } else {
                float value;
                robot.readRegister(request->register_name, request->index, value, ec);
                if (!ec) {
                    response->float_values.push_back(value);
                }
            }
        } 
        else if (request->data_type == "bool" || request->data_type == "bit") {
            if (request->read_all) {
                std::vector<bool> values;
                robot.readRegister(request->register_name, 0, values, ec);
                if (!ec) {
                    for (bool v : values) {
                        response->bool_values.push_back(v);
                    }
                }
            } else {
                bool value;
                robot.readRegister(request->register_name, request->index, value, ec);
                if (!ec) {
                    response->bool_values.push_back(value);
                }
            }
        }
        else if (request->data_type == "int16") {
            if (request->read_all) {
                std::vector<int> values;
                robot.readRegister(request->register_name, 0, values, ec);
                if (!ec) {
                    for (int16_t v : values) {
                        response->int16_values.push_back(v);
                    }
                }
            } else {
                int value;
                robot.readRegister(request->register_name, request->index, value, ec);
                if (!ec) {
                    response->int16_values.push_back(value);
                }
            }
        }
        else {
            response->success = false;
            response->message = "Invalid data type.";
            return false;
        }
        
        response->success = true;
        response->message = "Register read successfully";

    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Exception: ") + e.what();
        return false;
    }
    
    return true;
}


bool write_register_callback(
    const std::shared_ptr<rokae_msgs::srv::WriteRegister::Request> request,
    std::shared_ptr<rokae_msgs::srv::WriteRegister::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"), "write_register called");
        
        // 清除之前的错误码
        ec.clear();
        
        // 根据数据类型分支处理
        if (request->data_type == "float") {
            if (request->write_all) {
                robot.writeRegister(request->register_name, 0, request->float_values, ec);
            } else {
                robot.writeRegister(request->register_name, request->index, request->float_value, ec);
            }
        } 
        else if (request->data_type == "bool" || request->data_type == "bit") {
            if (request->write_all) {
                robot.writeRegister(request->register_name, 0, request->bool_values, ec);
            } else {
                robot.writeRegister(request->register_name, request->index, request->bool_value, ec);
            }
        }
        else if (request->data_type == "int16") {
            if (request->write_all) {
                // 逐个写入，避免机器人库的fmt错误
                for (size_t i = 0; i < request->int16_values.size(); i++) {
                    int int_value = static_cast<int>(request->int16_values[i]);
                    robot.writeRegister(request->register_name, i, int_value, ec);
                }
            } else {
                int int_value = static_cast<int>(request->int16_value);
                robot.writeRegister(request->register_name, request->index, int_value, ec);
            }
        }
        else {
            response->success = false;
            response->message = "Invalid data type.";
            return false;
        }
        
        response->success = true;
        response->message = "Register written successfully";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Exception: ") + e.what();
        return false;
    }
    
    return true;
}



bool movej_callback(
    const std::shared_ptr<rokae_msgs::srv::MoveJ::Request> request,
    std::shared_ptr<rokae_msgs::srv::MoveJ::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"movej called");
        // 1. 获取目标关节角度
        std::array<double, 6> target_joints;
        for (int i = 0; i < 6; i++) {
            target_joints[i] = request->joint_positions[i];
        }
        
        // 2. 获取速度参数
        double velocity = static_cast<double>(request->velocity);
        
        // 3. 获取当前关节角度
        std::array<double, 6> current_joints = robot.jointPos(ec);
        if (ec) {
            response->success = false;
            response->message = "Failed to get current joint positions";
            return false;
        }
        
        // 4. 获取实时控制器
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        auto rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            response->success = false;
            response->message = "Failed to get motion controller";
            return false;
        }
        
        // 5. 执行MoveJ
        rtCon->MoveJ(velocity, current_joints, target_joints);
        // rtCon->startMove(RtControllerMode::jointPosition);
        
        // 6. 设置成功响应
        response->success = true;
        response->message = "movej has been executed";
        
        //7.关闭rci
        // rtCon->stopMove();
        
    } catch (const std::exception& e) {
        // 8. 设置错误响应
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

// 接受末端相对的偏移量  笛卡尔可能出现逆解失败
bool movel_callback(
    const std::shared_ptr<rokae_msgs::srv::MoveL::Request> request,
    std::shared_ptr<rokae_msgs::srv::MoveL::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"movel called");
        // 1. 获取速度参数
        double velocity = static_cast<double>(request->velocity);
        
        // 2. 获取当前末端位姿
        CartesianPosition start;
        Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
        if (ec) {
            response->success = false;
            response->message = "Failed to get current flange position";
            return false;
        }
        
        // 3. 分解起始位姿为旋转矩阵和平移向量
        Eigen::Matrix3d rot_start;
        Eigen::Vector3d trans_start;
        Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
        
        // 4. 创建目标平移向量（应用相对偏移）
        Eigen::Vector3d trans_end = trans_start;
        
        // 使用 offset 而不是 target_position
        trans_end[0] += request->offset[0];  // X轴偏移
        trans_end[1] += request->offset[1];  // Y轴偏移
        trans_end[2] += request->offset[2];  // Z轴偏移
        
        // 5. 创建目标位姿（保持起始点的旋转姿态）
        CartesianPosition target;
        Utils::transMatrixToArray(rot_start, trans_end, target.pos);
        
        // 6. 获取实时控制器
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        auto rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            response->success = false;
            response->message = "Failed to get motion controller";
            return false;
        }
        
        // 7. 执行MoveL
        rtCon->MoveL(velocity, start, target);
        // rtCon->startMove(RtControllerMode::cartesianPosition);
        
        // 8. 设置成功响应
        response->success = true;
        response->message = "movel has been executed";

        // 9. 关闭rci
        // rtCon->stopMove();
        
    } catch (const std::exception& e) {
        // 10. 设置错误响应
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}

//x-y平面 z偏移设0
bool movec_callback(
    const std::shared_ptr<rokae_msgs::srv::MoveC::Request> request,
    std::shared_ptr<rokae_msgs::srv::MoveC::Response> response)
{
    try {
        RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"movec called");
        // 1. 获取速度参数
        double velocity = static_cast<double>(request->velocity);
        
        // 2. 获取当前末端位姿作为起点
        CartesianPosition start;
        Utils::postureToTransArray(robot.posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
        if (ec) {
            response->success = false;
            response->message = "Failed to get current flange position";
            return false;
        }
        
        // 3. 分解起始位姿为旋转矩阵和平移向量
        Eigen::Matrix3d rot_start;
        Eigen::Vector3d trans_start, trans_aux, trans_end;
        Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
        
        // 4. 计算辅助点和终点的平移向量（应用相对偏移）
        trans_aux = trans_start;
        trans_end = trans_start;
        
        // 检查偏移量数组大小
        if (request->aux_offset.size() < 3 || request->target_offset.size() < 3) {
            response->success = false;
            response->message = "Offset arrays must have at least 3 elements";
            return false;
        }
        
        // 应用辅助点偏移
        trans_aux[0] += request->aux_offset[0];  // X轴偏移
        trans_aux[1] += request->aux_offset[1];  // Y轴偏移
        trans_aux[2] += request->aux_offset[2];  // Z轴偏移
        
        // 应用终点偏移
        trans_end[0] += request->target_offset[0];  // X轴偏移
        trans_end[1] += request->target_offset[1];  // Y轴偏移
        trans_end[2] += request->target_offset[2];  // Z轴偏移
        
        // 5. 创建辅助点和终点位姿（保持起始点的旋转姿态）
        CartesianPosition aux, target;
        Utils::transMatrixToArray(rot_start, trans_aux, aux.pos);
        Utils::transMatrixToArray(rot_start, trans_end, target.pos);
        
        // 6. 获取实时控制器
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        auto rtCon = robot.getRtMotionController().lock();
        if (!rtCon) {
            response->success = false;
            response->message = "Failed to get motion controller";
            return false;
        }
        
        // 7. 执行MoveC
        rtCon->MoveC(velocity, start, aux, target);
        // rtCon->startMove(RtControllerMode::cartesianPosition);
        
        // 8. 设置成功响应
        response->success = true;
        response->message = "movec has been executed";

        // 9.关闭rci
        // rtCon->stopMove();
        
    } catch (const std::exception& e) {
        // 10. 设置错误响应
        response->success = false;
        response->message = e.what();
        return false;
    }
    
    return true;
}




void joint_position_callback(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_state_pub) {
    try {
        // 1. 从Rokae机器人获取关节位置 (可能包含外部轴)
        auto joint_positions = robot.jointPos(ec);
        auto joint_velocity = robot.jointVel(ec);
        auto joint_jointTorque = robot.jointTorque(ec);
        
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), 
                       "获取关节位置失败: %s", ec.message().c_str());
            return;
        }
        
        // 2. 创建并填充ROS关节状态消息
        JointState joint_state_msg;
        joint_state_msg.header.stamp = rclcpp::Clock().now(); // 设置时间戳
        // joint_state_msg.header.frame_id = "base_link"; // 参考坐标系
        
        // 3. 设置关节名称 (根据你的机器人模型调整)
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            joint_state_msg.name.push_back("joint" + std::to_string(i + 1));
            joint_state_msg.position.push_back(joint_positions[i]);
            joint_state_msg.velocity.push_back(joint_velocity[i]);
            joint_state_msg.effort.push_back(joint_jointTorque[i]);
        }
 
        // 4. 发布消息
        if (joint_state_pub) {
            joint_state_pub->publish(joint_state_msg);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rokae_driver"), 
                    "发布关节状态时出错: %s", e.what());
    }
}


void cartesian_pose_callback(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& cartesian_pose_pub) {
    try {
        // 1. 从Rokae机器人获取法兰相对于基坐标系的位姿
        std::array<double, 6> pose_array = robot.posture(rokae::CoordinateType::flangeInBase, ec);
        
        if (ec) {
            RCLCPP_WARN(rclcpp::get_logger("rokae_driver"), 
                       "获取笛卡尔位姿失败: %s", ec.message().c_str());
            return;
        }
        
        // 2. 创建并填充ROS笛卡尔位姿消息
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "base_link"; // 设置参考坐标系为基坐标系
        
        // 3. 设置位置 (x, y, z) - 单位：米
        pose_msg.pose.position.x = pose_array[0];
        pose_msg.pose.position.y = pose_array[1];
        pose_msg.pose.position.z = pose_array[2];
        
        // 4. 设置姿态 (rx, ry, rz 转换为四元数)
        double rx = pose_array[3]; // roll
        double ry = pose_array[4]; // pitch
        double rz = pose_array[5]; // yaw
        
        // 使用ROS的tf2进行转换
        tf2::Quaternion q;
        q.setRPY(rx, ry, rz);
        
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        // 5. 发布消息
        if (cartesian_pose_pub) {
            cartesian_pose_pub->publish(pose_msg);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rokae_driver"), 
                    "发布笛卡尔位姿时出错: %s", e.what());
    }
}


void state_monitor_worker(rclcpp::Node::SharedPtr node) {
    while (rclcpp::ok()) {
        // std::error_code ec;
        
        // 1. 连接检查与数据获取
        // auto joint_positions = robot.jointPos(ec);
        robot.jointPos(ec);
        
        if (ec) {
            // 连接或读取失败
            RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                                 "连接错误或获取数据失败: %s", ec.message().c_str());
        } else {
            // 2. 连接正常，执行发布回调（目前只有关节状态）
            joint_position_callback(joint_state_pub);
            cartesian_pose_callback(cartesian_pose_pub);
        }
        
        // 3. 控制频率
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc , char** argv){
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("rokae_driver");
    rclcpp::Rate rate(125); 
    // 连接到机器人
    try {
        robot.connectToRobot(robot_ip ,local_ip);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rokae_driver"), "%s", e.what());
        return 0;
    }
    //设置机器人状态
    // robot.setOperateMode(rokae::OperateMode::automatic, ec);
    // robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
    // robot.setPowerState(true, ec);
    // auto rtCon = robot.getRtMotionController().lock();

    RCLCPP_INFO(rclcpp::get_logger("rokae_driver"),"rokae_driver is ready");
    
    auto get_robot_info_service = node->create_service<rokae_msgs::srv::GetRobotInfo>(
        "/rokae_driver/get_robot_info", &get_robot_info_callback);
    auto control_drag_service = node->create_service<rokae_msgs::srv::DragCon>(
        "/rokae_driver/drag_control", &drag_control_callback);
    auto jog_drag_service = node->create_service<rokae_msgs::srv::JogCon>(
        "/rokae_driver/jog_control", &jog_control_callback);

    auto get_do_service = node->create_service<rokae_msgs::srv::GetDO>(
        "/rokae_driver/get_do", &get_do_callback);
    auto set_do_service = node->create_service<rokae_msgs::srv::SetDO>(
        "/rokae_driver/set_do", &set_do_callback);
    auto set_di_service = node->create_service<rokae_msgs::srv::SetDI>(
        "/rokae_driver/set_di", &set_di_callback);    
    auto get_di_service = node->create_service<rokae_msgs::srv::GetDI>(
        "/rokae_driver/get_di", &get_di_callback);
    auto read_register_service = node->create_service<rokae_msgs::srv::ReadRegister>(
        "/rokae_driver/read_register", &read_register_callback);
    auto write_register_service = node->create_service<rokae_msgs::srv::WriteRegister>(
        "/rokae_driver/write_register", &write_register_callback);

    auto calculate_ik_service = node->create_service<rokae_msgs::srv::CalculateIK>(
        "/rokae_driver/calculate_ik", &calculate_ik_callback);
    
    auto calculate_fk_service = node->create_service<rokae_msgs::srv::CalculateFK>(
        "/rokae_driver/calculate_fk", &calculate_fk_callback);

    auto movej_service = node->create_service<rokae_msgs::srv::MoveJ>( //服务消息名称大写开头
    "/rokae_driver/movej",  // 服务名称
    &movej_callback);          // 回调函数

    auto movel_service = node->create_service<rokae_msgs::srv::MoveL>("/rokae_driver/movel",&movel_callback);
        
    auto movec_service = node->create_service<rokae_msgs::srv::MoveC>("/rokae_driver/movec",&movec_callback);

    
    joint_state_pub = node->create_publisher<JointState>("/rokae_driver/joint_states", 10 );
    cartesian_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/rokae_driver/cartesian_pose", 10);

    std::thread monitor_thread(state_monitor_worker, node);
    
    // 保持节点运行
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "正在关闭节点...");
    if (monitor_thread.joinable()) {
        monitor_thread.join();
        // RCLCPP_INFO(node->get_logger(), "状态监控线程已合并。");
    }
    
    // 节点结束时清理
    robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
    robot.setPowerState(false, ec);

    rclcpp::shutdown();
    return 0;
}