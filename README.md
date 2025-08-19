# 使用步骤：
mkdir -p ~/rokae_ws/src  
cd ~/ros2_ws/src
git clone https://gitlab.i.rokae.com/xcore_sdk/rokae_ros2.git  
cd ..
colcon build  

# 启动机器人命令    
## 使用封装的硬件接口（一般都用这条）
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=xMateCR7 use_fake_hardware:=false robot_ip:=192.168.21.10 local_ip:=192.168.21.131 
## 使用虚拟仿真硬件接口（测试move_group,rviz等应用配置用，并非连接实体机器人或HMI）
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=xMateCR7 use_fake_hardware:=true robot_ip:=192.168.21.10 local_ip:=192.168.21.131   

##存在的问题：  
1.只封装了CR7机型，还未添加其它机型  
2.硬件接口运行时会出现崩溃,主要的两个崩溃如下：
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
