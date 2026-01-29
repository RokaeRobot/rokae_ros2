#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <atomic>
#include <iomanip>
#include "rokae/robot.h"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "用法: " << argv[0] << " <机器人IP> <本地IP>" << std::endl;
        return 1;
    }
    
    std::string robot_ip = argv[1];
    std::string local_ip = argv[2];
    
    std::cout << "Rokae 网络通信测试\n";
    std::cout << "机器人IP: " << robot_ip << ", 本地IP: " << local_ip << "\n";
    std::cout << "按 Enter 开始测试..." << std::endl;
    std::cin.ignore();
    
    try {
        // 连接机器人
        rokae::xMateRobot robot(robot_ip, local_ip);
        std::error_code ec;
        robot.connectToRobot(ec);
        if (ec) throw std::runtime_error("连接失败: " + ec.message());
        
        // 设置机器人模式
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        if (ec) throw std::runtime_error("设置模式失败: " + ec.message());
        
        // 移动到零位
        std::array<double, 6> zero_position = {0, 0, 0, 0, 0, 0};
        std::cout << "移动到零位..." << std::endl;
        auto rtCon = robot.getRtMotionController().lock();
        if (rtCon) {
            rtCon->MoveJ(0.3, robot.jointPos(ec), zero_position);
        }
        
        // 启动状态接收
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {
            rokae::RtSupportedFields::jointPos_m
        });
        
        // 测试参数
        const int TEST_DURATION_MS = 10000;  // 10秒
        const int TARGET_FREQ = 1000;        // 1000Hz
        const int CYCLE_US = 1000000 / TARGET_FREQ;  // 1000us = 1ms
        
        // 统计变量
        std::atomic<int> cycle_count{0};
        std::atomic<int> success_count{0};
        std::atomic<int> failed_count{0};
        std::atomic<double> min_delay{10000.0};
        std::atomic<double> max_delay{0.0};
        std::atomic<double> total_delay{0.0};
        
        std::cout << "开始 " << TARGET_FREQ << "Hz 网络测试..." << std::endl;
        
        auto test_start = std::chrono::steady_clock::now();
        auto test_end = test_start + std::chrono::milliseconds(TEST_DURATION_MS);
        
        while (std::chrono::steady_clock::now() < test_end) {
            auto cycle_start = std::chrono::high_resolution_clock::now();
            
            try {
                // 核心测试：读取关节位置
                std::array<double, 6> pos{};
                auto read_start = std::chrono::high_resolution_clock::now();
                int ret_=robot.updateRobotState(std::chrono::milliseconds(1)); 
                int ret = robot.getStateData(rokae::RtSupportedFields::jointPos_m, pos);
                auto read_end = std::chrono::high_resolution_clock::now();
                
                // 计算延迟
                double delay_ms = std::chrono::duration<double, std::milli>(
                    read_end - read_start).count();
                
                bool success = ((ret_ > 0)&&(ret ==0));
                
                cycle_count++;
                if (success) {
                    success_count++;
                    
                    // 更新延迟统计
                    if (delay_ms < min_delay) min_delay = delay_ms;
                    if (delay_ms > max_delay) max_delay = delay_ms;
                    total_delay.store(total_delay.load() + delay_ms);
                } else {
                    failed_count++;
                }
                
                // 每100次循环打印一次
                if (cycle_count % 100 == 0) {
                    double success_rate = (double)success_count / cycle_count;
                    std::cout << "#" << cycle_count 
                              << " 成功率: " << std::fixed << std::setprecision(1) << (success_rate * 100) << "%"
                              << " 延迟: " << std::setprecision(3) << delay_ms << "ms"
                              << std::endl;
                }
                
            } catch (...) {
                failed_count++;
                cycle_count++;
            }
            
            // 精确控制频率
            auto cycle_end = std::chrono::high_resolution_clock::now();
            auto cycle_time = std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_end - cycle_start);
            
            int wait_us = CYCLE_US - cycle_time.count();
            if (wait_us > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(wait_us));
            }
        }
        
        // 测试结束，计算统计
        int total = cycle_count;
        int success = success_count;
        int lost = failed_count;
        
        double avg_delay = (success > 0) ? total_delay / success : 0.0;
        double success_rate = (total > 0) ? (double)success / total : 0.0;
        
        // 输出结果
        std::cout << "\n\n================ 测试结果 ================\n";
        std::cout << "总循环次数: " << total << "\n";
        std::cout << "成功次数: " << success << "\n";
        std::cout << "丢失次数: " << lost << "\n";
        std::cout << "丢包率: " << std::fixed << std::setprecision(2) 
                  << ((double)lost / total * 100) << "%\n";
        std::cout << "成功率: " << std::fixed << std::setprecision(2) 
                  << (success_rate * 100) << "%\n";
        std::cout << "最小延迟: " << std::fixed << std::setprecision(3) 
                  << min_delay << "ms\n";
        std::cout << "平均延迟: " << std::fixed << std::setprecision(3) 
                  << avg_delay << "ms\n";
        std::cout << "最大延迟: " << std::fixed << std::setprecision(3) 
                  << max_delay << "ms\n";
        std::cout << "========================================\n";
        
        // 清理
        robot.setPowerState(false, ec);
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
