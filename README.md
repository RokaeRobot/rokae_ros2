# 运行环境：ubuntu22.04 ROS2 humble  
鱼香ROS2一键安装：<https://blog.csdn.net/pixelprodigy/article/details/147933853>
# 使用步骤：
mkdir -p ~/ros2_ws/src  
cd ~/ros2_ws/src
git clone https://gitlab.i.rokae.com/xcore_sdk/rokae_ros2.git  
cd ..  
colcon build  

# 启动机器人命令    
## 使用封装的硬件接口（以CR7机型为例一般都用这条）  
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=CR7 use_fake_hardware:=false robot_ip:=192.168.21.10 local_ip:=192.168.21.131  
## 使用虚拟仿真硬件接口（测试move_group,rviz等应用配置用，并非连接实体机器人或HMI）
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=CR7 use_fake_hardware:=true robot_ip:=192.168.21.10 local_ip:=192.168.21.131  
现有机型启动，只需将robot_type换成相应的机型（SR4,ER7,Pro3,Pro7）  
rokae_hardware_interface.cpp中：  
robot_ = std::make_shared<rokae::xMateRobot>(robot_ip_, local_ip_); 对应启动六轴机型  
robot_ = std::make_shared<rokae::xMateErProRobot>(robot_ip_, local_ip_);  对应启动七轴机型  
根据机型轴数不同需要对共享指针robot_的定义和初始化进行修改。  
## 启动机器人测试脚本命令  
ros2 launch rokae_hardware controll_movej.launch.py robot_type:=CR7（机型包括CR7,SR4,ER7,Pro3,Pro7）  
movej.cpp测试脚本中：   
arm.setPoseReferenceFrame("xMateCR7_base");    //换成相应的基座(在相应机型srdf下)  
auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "rokae_arm");   //换成相应的group(在相应机型srdf下)  
# 添加新机型步骤：  
## 1.robot_description中导入相应rviz,mesh,xacro文件，文件格式可模仿现有机型  
## 2.robot_description urdf文件中xMate.urdf.xacro,xMate_macro.xacro添加相应机型配置，并编写新机型ros2_controll.xacro文件，具体格式可参照现有文件机型  
## 3.导入相应机型moveit_config文件夹，具体格式可参考现有机型  
## 4.colcon build工作空间，并输入ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=新机型 use_fake_hardware:=false robot_ip:=192.168.21.10 local_ip:=192.168.21.131看是否编译通过，机器人能否正常启动  


# 解决的问题：  
硬件接口运行时会出现崩溃,主要的两个崩溃如下：  
[ERROR] [ros2_control_node-1]: process has died [pid 4417, exit code -11, cmd '/opt/ros/humble/lib/controller_manager/ros2_control_node --ros-args --params-file /tmp/launch_params_jbqbmz5t --params-file /tmp/launch_params_xjmitrq7'].  

[ros2_control_node-1] Stack trace (most recent call last) in thread 4529:  
[ros2_control_node-1] #17   Object "", at 0xffffffffffffffff, in   
[ros2_control_node-1] #16   Source "../sysdeps/unix/sysv/linux/x86_64/clone3.S", line 81, in __clone3 [0x7cc14ff2684f]   
[ros2_control_node-1] #15   Source "./nptl/pthread_create.c", line 442, in start_thread [0x7cc14fe94ac2]  
[ros2_control_node-1] #14   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7cc1502dc252, in   
[ros2_control_node-1] #13   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc14542be6a, in rokae::MotionControl<(rokae::MotionControlMode)3>::Impld::_do_loop()  
[ros2_control_node-1] #12   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc145436d6f, in std::_Function_handler<void (), rokae::MotionControl<(rokae::MotionControlMode)3>::Impld::setCallback<rokae::JointPosition>(std::function<rokae::JointPosition ()> const&, bool)::{lambda()#1}>::_M_invoke(std::_Any_data const&)  
[ros2_control_node-1] #11   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc14533f15f, in std::_Function_handler<rokae::JointPosition (), std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()> >::_M_invoke(std::_Any_data const&)  
[ros2_control_node-1] #10   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc1453419cc, in std::enable_if<is_invocable_r_v<rokae::JointPosition, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()>&>, rokae::JointPosition>::type std::__invoke_r<rokae::JointPosition, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()>&>(std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()>&)  
[ros2_control_node-1] #9    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc145343a54, in rokae::JointPosition std::__invoke_impl<rokae::JointPosition, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()>&>(std::__invoke_other, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()>&)  
[ros2_control_node-1] #8    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc145345950, in rokae::JointPosition std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()>::operator()<, rokae::JointPosition>()  
[ros2_control_node-1] #7    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc1453473e6, in rokae::JointPosition std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*(rokae_hardware::RokaeHardwareInterface*))()>::__call<rokae::JointPosition, , 0ul>(std::tuple<>&&, std::_Index_tuple<0ul>)  
[ros2_control_node-1] #6    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc145348c99, in std::__invoke_result<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*&)(), rokae_hardware::RokaeHardwareInterface*&>::type std::__invoke<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*&)(), rokae_hardware::RokaeHardwareInterface*&>(rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*&)(), rokae_hardware::RokaeHardwareInterface*&)  
[ros2_control_node-1] #5    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc145349ef9, in rokae::JointPosition std::__invoke_impl<rokae::JointPosition, rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*&)(), rokae_hardware::RokaeHardwareInterface*&>(std::__invoke_memfun_deref, rokae::JointPosition (rokae_hardware::RokaeHardwareInterface::*&)(), rokae_hardware::RokaeHardwareInterface*&)  
[ros2_control_node-1] #4    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc145338d49, in rokae_hardware::RokaeHardwareInterface::callback()  
[ros2_control_node-1] #3    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc14542d1f3, in rokae::MotionControl<(rokae::MotionControlMode)3>::updateRobotState()  
[ros2_control_node-1] #2    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc14548c748, in rokae::DataReceiver::updateOnce(std::chrono::duration<long, std::ratio<1l, 1000000000l> >)  
[ros2_control_node-1] #1    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7cc145489143, in void std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >::_M_construct<char*>(char*, char*, std::forward_iterator_tag) [clone .constprop.778]  
[ros2_control_node-1] #0    Source "../sysdeps/x86_64/multiarch/memmove-vec-unaligned-erms.S", line 317, in __memmove_avx512_unaligned_erms [0x7cc14ffa67cd]  
[ros2_control_node-1] Segmentation fault (Address not mapped to object [0x10c])    

解决方法：将回调函数callback写到perform_command_mode_switch中，保证其只在控制器切换的时候调用一次，而不是每次  
在写函数write中循环调用  

# 需要优化的地方  
## 1.rviz端plan时候可视化界面仿真机器人未随着HMI中机器人移动，可优化  
## 2.机器人位置指令下发频率需要和机器人控制周期以及ROS2生命周期匹配，否则会出现：[ros2_control_node-1]   what():  实时模式异常: 运动错误: 指令轴速度超限 (10-kCommandJointVelocityLimitsViolation)报错  
## 3.机器人运动碰到奇异点导致控制器aborted后，无法再进行下一段轨迹规划，需要重新用命令行启动。可以优化使控制器aborted后重新启动，让机器人再继续进行下一段轨迹规划
