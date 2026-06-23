# Rokae ROS 2

珞石机器人 ROS 2 软件栈，基于 `ros2_control` 与 MoveIt 2，提供 xMate 系列机械臂的仿真与真机控制。

当前栈版本：**0.0.4**

## 兼容性

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04（推荐） |
| ROS 2 | Humble |
| xCore SDK | 与 `rokae_hardware/sdk/VERSION` 一致（当前 **0.7.1**） |
| xCore 控制器 | ≥ v3.2.1 |

## 仓库内容

本仓库包含 ROS 2 源码包、URDF/MoveIt 配置与示例，**不包含** xCore SDK 预编译库。

| 包 | 说明 |
|----|------|
| `rokae_hardware` | 硬件接口、`ros2_control` 插件、驱动与 SDK 头文件 |
| `rokae_description` | URDF / mesh |
| `rokae_msgs` | 自定义消息与服务 |
| `rokae_example` | 运动示例 |
| `rokae_gazebo` | Gazebo 仿真辅助 |
| `rokae_xMate*_moveit_config` | 各机型 MoveIt 配置 |

## 获取与编译

### 1. 克隆仓库

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-github-repo-url> rokae_ros2
```

克隆后目录名只要位于 `src/` 下即可被 `colcon` 发现。

### 2. 下载 xCore SDK 预编译库

1. 查看 `rokae_hardware/sdk/VERSION` 中的 SDK 版本号
2. 打开对应 [xCoreSDK-CPP Release](https://github.com/RokaeRobot/xCoreSDK-CPP/releases) 页面
3. 下载匹配平台的库包（Linux 示例：`xCoreSDK-0.7.1-linux-x86_64.tar.gz`）
4. 按 [rokae_hardware/sdk/lib/README.md](rokae_hardware/sdk/lib/README.md) 解压到 `rokae_hardware/sdk/lib/`

### 3. 安装 ROS 2 依赖并编译

```bash
cd ~/ros2_ws
rosdep install --from-paths src/rokae_ros2 --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 文档

- [使用手册](doc/rokae%20ros2使用手册.md)
- [Demo 启动说明](doc/README_demo.md)

## 发版说明

- 栈版本：更新各包 `package.xml`、`CMakeLists.txt` 中的 `VERSION`，并写 `CHANGELOG.rst` / 根目录 `CHANGELOG.md`
- SDK 升级：同步 `rokae_hardware/sdk/include` 头文件，更新 `rokae_hardware/sdk/VERSION`，在 GitHub Release 说明中注明所需 xCore SDK 版本

## License

Copyright (C) 2026 ROKAE (Beijing) Technology Co., LTD.

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
