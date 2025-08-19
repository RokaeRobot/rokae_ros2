# 一.工作空间构建编译，cmakelist构建语法等  
1.colcon build报错解决思路：每个功能包下cmakelist写法都和ROS1中不同，仿照UR5 ROS2和飞锡ROS2相应功能包下cmakelist写法改  
2.硬件接口层迁移：先把hpp和cpp分开，再各自换成ROS2下相应的框架    
3.机器人初始launch文件迁移：先仿照飞锡或UR5搭好主体框架，然后按节点启动顺序依次排查修改。常见节点启动顺序：    
第一步首先调用hardware_interface连上机器人ip，robot_state_publisher（发布机器人的TF变换树）, joint_state_publisher(发布sensor_msgs/JointState消息),ros2_control_node(运行硬件接口 hardware_interface 并与控制器controller_manager通信), controll_manager(机器人控制器,yaml格式要与ROS2适配,urdf文件要配上ros2_control.xacro部分并被正确调用), xml插件文件配置（与硬件接口和ros2_controll.xacro均要同名，ros2插件机制只能调用非模板类），rviz(可视化)    
4.moveit_config.launch文件启动迁移：gpt搭建初始框架，仿照飞夕填充节点，注意yaml文件格式和launch.py中加载方式一定要对应（只加载路径没加载内容则还是不能用）    



# 二.常用命令行：
ros2 run xacro xacro /home/yyc/rokae2_ws/src/rokae/rokae_description/urdf/xMate.urdf.xacro robot_ip:=192.168.21.10 local_ip:=192.168.21.130 > /tmp/rokae_generated.urdf  
作用：验证最原始模型xMate.urdf.xacro参数是否能正常展开  

测试命令行（目前用的最多）:ros2 launch rokae_hardware test_ros_controller_node.launch.py robot_ip:=192.168.21.10 local_ip:=192.168.21.131
  
从UR5移植过来的命令行：ros2 launch rokae_hardware xMate_control.launch.py use_fake_hardware:=false robot_type:=xMateCR7 robot_ip:=192.168.21.10 local_ip:=192.168.21.131  

目前启动最正常的命令行：  
ros2 launch rokae_hardware test_ros_controller_node.launch.py robot_type:=xMateCR7 use_fake_hardware:=true robot_ip:=192.168.21.10 local_ip:=192.168.21.131  


# 三.linux清理磁盘空间命令：  
sudo apt clean：清理apt缓存  
rm -rf ~/.cache/*：清理用户缓存  
sudo find /var -type f -size +100M -exec ls -lh {} \; 2>/dev/null | sort -k5 -hr | head -n 20(/var可换成/usr,/tmp等路径):查找相应目录下的大文件  



后台进入仿真模式命令（ubuntu下调试需要，否则无法调用sdk）：  
sudo su(输入密码)  
cd bin/controller  
pkill -9 MainThread  
./xCore --simu_mode &        

# 四.应用层调用注意事项（以movej.cpp为例）  
ROS2节点为分布式，因此不能将movej单独作为节点运行，否则无效。需要通过rokae_hardware节点调用movej，并加载robot_description下的机器人urdf模型才能使用movej。   

# 五.硬件接口cpp细节：
1.头文件override复写问题(重中之重！！！必须用ROS2自带的SystemInterface接口进行重写，否则rokae_hardware_interface.cpp根本无法进行初始化参数配置)  
2.ROS 2 Humble 的 LifecycleSystemInterface 生命周期顺序：构造函数 → on_init() → on_configure() → on_activate() → on_deactivate() → on_cleanup()  
3.SystemInterface包含#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"中的内容，因此LifecycleNodeInterface接口中函数也可以被调用且必须被封装在机器人硬件接口中  
4.各个接口函数作用：  
on_init()：获取机器人关节等信息。注意一定要把info赋值给新定义的成员变量info_才能成功读取  
on_configure();获取机器人ip参数。参数info_在on_init()中已经从ROS2基类接口中获取，在这里可以用于获取机器人ip。注意成功读取ip不等于成功连接上了机器人  
on_activate():连接机器人，使能机器人等功能用于激活  
on_deactivate():存在即可，保证机器人生命周期顺序完整  
export_command_interfaces():获取机器人关节命令信息  
export_state_interfaces():获取机器人关节状态信息  
read():从SDK接口中获取数据
write():向机器人写数据  
prepare_command_mode_switch：准备命令切换，在控制器启动后执行  
perform_command_mode_switch：执行命令切换  


# 六.博客资料：
Ubuntu22.04 + ROS2 Humble配置Moveit2环境  
<https://blog.csdn.net/qq_44940689/article/details/137968166?spm=1001.2014.3001.5502>    
【Moveit2】使用moveit_setup_assistant配置自己的机械臂功能包  
<https://blog.csdn.net/qq_44940689/article/details/138214260>  


# 第一版终章：  
机器人启动命令：  
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=xMateCR7 use_fake_hardware:=true robot_ip:=192.168.21.10 local_ip:=192.168.21.131(用虚拟机器人)  
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=xMateCR7 use_fake_hardware:=false robot_ip:=192.168.21.10 local_ip:=192.168.21.131（在HMI上调试）  

运行movej命令：ros2 launch rokae_hardware controll_movej.launch.py     

# 各种调试命令：
ros2 node list:查看已启动的节点  
ros2 param list:查看已启动的参数  
ros2 param list /controller_manager：查看controller_manager下启动的参数  
ros2 node info /controller_manager:查看controller_manager参数信息  
ros2 control list_controllers:查看controller_manager下组件的启动配置情况  
ros2 topic echo /joint_states:读取机器人各个关节joint的信息（名称，position,velocity,effort）     


# 单独调试ros2_control_node节点步骤，解决硬件接口线程卡死原因：
gdb --args /opt/ros/humble/lib/controller_manager/ros2_control_node \  
    --ros-args --params-file /home/yyc/rokae2_ws/src/rokae/rokae_hardware/config/xMateCR_controllers.yaml  
    

(gdb)run   