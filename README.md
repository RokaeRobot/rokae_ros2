# 使用步骤：
mkdir -p ~/rokae_ws/src    
cd ~/ros2_ws/src  
git clone https://gitlab.i.rokae.com/xcore_sdk/rokae_ros2.git    
cd ..  
colcon build    

# 启动机器人命令      
## 使用封装的硬件接口（一般都用这条）  
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=CR7 use_fake_hardware:=false robot_ip:=192.168.21.10 local_ip:=192.168.21.131   
## 使用虚拟仿真硬件接口（测试move_group,rviz等应用配置用，并非连接实体机器人或HMI）  
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=CR7 use_fake_hardware:=true robot_ip:=192.168.21.10 local_ip:=192.168.21.131     
注意：第二版robot_type可换成不同的机型名称启动相应的机型（CR7,ER7,Pro7等，不需要xMate前缀）；robot_ip与local_ip换成自己的地址即可。  

## 存在的问题：    
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

如果不启动move_group和rviz，目前模版实例化后的硬件接口运行崩溃报错如下
[ros2_control_node-1] terminate called after throwing an instance of 'std::logic_error'
[ros2_control_node-1]   what():  basic_string::_M_construct null not valid
[ros2_control_node-1] Stack trace (most recent call last) in thread 4554:
[ros2_control_node-1] #28   Object "", at 0xffffffffffffffff, in 
[ros2_control_node-1] #27   Source "../sysdeps/unix/sysv/linux/x86_64/clone3.S", line 81, in __clone3 [0x7c12e092684f]
[ros2_control_node-1] #26   Source "./nptl/pthread_create.c", line 442, in start_thread [0x7c12e0894ac2]
[ros2_control_node-1] #25   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7c12e0cdc252, in 
[ros2_control_node-1] #24   Source "./src/ros2_control_node.cpp", line 125, in operator() [0x56d069ba30d7]
[ros2_control_node-1] #23   Source "./src/controller_manager.cpp", line 2216, in write [0x7c12e116d554]
[ros2_control_node-1] #22   Object "/opt/ros/humble/lib/libhardware_interface.so", at 0x7c12e0b1cf8c, in hardware_interface::ResourceManager::write(rclcpp::Time const&, rclcpp::Duration const&)
[ros2_control_node-1] #21   Object "/opt/ros/humble/lib/libhardware_interface.so", at 0x7c12e0b3a57f, in hardware_interface::System::write(rclcpp::Time const&, rclcpp::Duration const&)
[ros2_control_node-1] #20   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da34b5c9, in rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::write(rclcpp::Time const&, rclcpp::Duration const&)
[ros2_control_node-1] #19   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da35edcf, in std::function<rokae::JointPosition ()>::operator()() const
[ros2_control_node-1] #18   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da362e8b, in std::_Function_handler<rokae::JointPosition (), std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()> >::_M_invoke(std::_Any_data const&)
[ros2_control_node-1] #17   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da366210, in std::enable_if<is_invocable_r_v<rokae::JointPosition, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()>&>, rokae::JointPosition>::type std::__invoke_r<rokae::JointPosition, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()>&>(std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()>&)
[ros2_control_node-1] #16   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da36891e, in rokae::JointPosition std::__invoke_impl<rokae::JointPosition, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()>&>(std::__invoke_other, std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()>&)
[ros2_control_node-1] #15   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da36aa32, in rokae::JointPosition std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()>::operator()<, rokae::JointPosition>()
[ros2_control_node-1] #14   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da36c7c2, in rokae::JointPosition std::_Bind<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*(rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*))()>::__call<rokae::JointPosition, , 0ul>(std::tuple<>&&, std::_Index_tuple<0ul>)
[ros2_control_node-1] #13   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da36dda3, in std::__invoke_result<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*&)(), rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*&>::type std::__invoke<rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*&)(), rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*&>(rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*&)(), rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*&)
[ros2_control_node-1] #12   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da36ee1d, in rokae::JointPosition std::__invoke_impl<rokae::JointPosition, rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*&)(), rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*&>(std::__invoke_memfun_deref, rokae::JointPosition (rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::*&)(), rokae_hardware::RokaeHardwareInterface<(unsigned short)6>*&)
[ros2_control_node-1] #11   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da350f7f, in rokae_hardware::RokaeHardwareInterface<(unsigned short)6>::callback()
[ros2_control_node-1] #10   Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da4516d3, in rokae::MotionControl<(rokae::MotionControlMode)3>::updateRobotState()
[ros2_control_node-1] #9    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da4b0c28, in rokae::DataReceiver::updateOnce(std::chrono::duration<long, std::ratio<1l, 1000000000l> >)
[ros2_control_node-1] #8    Object "/home/yyc/rokae2_ws/install/rokae_hardware/lib/rokae_hardware/librokae_hardware_interface.so", at 0x7c12da4ad639, in void std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >::_M_construct<char*>(char*, char*, std::forward_iterator_tag) [clone .constprop.778]
[ros2_control_node-1] #7    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7c12e0ca5343, in std::__throw_logic_error(char const*)
[ros2_control_node-1] #6    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7c12e0cae4d7, in __cxa_throw
[ros2_control_node-1] #5    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7c12e0cae276, in std::terminate()
[ros2_control_node-1] #4    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7c12e0cae20b, in 
[ros2_control_node-1] #3    Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7c12e0ca2b9d, in 
[ros2_control_node-1] #2    Source "./stdlib/abort.c", line 79, in abort [0x7c12e08287f2]
[ros2_control_node-1] #1    Source "../sysdeps/posix/raise.c", line 26, in raise [0x7c12e0842475]
[ros2_control_node-1] #0  | Source "./nptl/pthread_kill.c", line 89, in __pthread_kill_internal
[ros2_control_node-1]     | Source "./nptl/pthread_kill.c", line 78, in __pthread_kill_implementation
[ros2_control_node-1]       Source "./nptl/pthread_kill.c", line 44, in __pthread_kill [0x7c12e08969fc]
[ros2_control_node-1] Aborted (Signal sent by tkill() 4494 1000)

