# rokae_example Demo 启动说明

本文档对应 `rokae_example/launch` 目录下常用 demo：

- `joint_s_line.launch.py`
- `cartesian_s_line.launch.py`
- `follow_joint_position.launch.py`
- `follow_cart_position.launch.py`
- `servoj_demo.launch.py`

> 说明：`joint_s_line_rt.launch.py` 当前为空文件，暂无可用启动内容。

## 1. 前置条件

- 已安装 ROS 2（与你当前工程匹配的发行版）
- 工作空间已包含并可编译以下包：
  - `rokae_hardware`
  - `rokae_example`
  - `rokae_description`
  - `rokae_xMate<机型>_moveit_config`（例如 `rokae_xMateCR7_moveit_config`、`rokae_xMateCR35_moveit_config`）
- 机器人机型需在支持列表中：
  - `CR7/CR12/CR18/CR20/CR35/ER3/ER7/SR3/SR4/SR5/Pro3/Pro7/AR5L/AR5R`

## 2. 编译与环境加载（bash）

在工作空间根目录（`ros2_ws`）执行：

```bash
colcon build --packages-select rokae_hardware rokae_example rokae_description
```

## 3. 推荐运行顺序（两终端）

下文各 **`ros2 launch rokae_example ...`** 示例，依赖整机已启动后的 **`move_group`**、**`/joint_states`**、**轨迹控制器** 等；请按下面顺序操作。

**终端 1（先启动整机链路，保持运行）**

先启动由 `rokae_hardware` 提供的整机 launch（`ros2_control`、MoveIt、`move_group`、RViz 等会随该 launch 拉起）：

```bash
ros2 launch rokae_hardware rokae_moveit_launch.py robot_type:=<机型> use_fake_hardware:=false robot_ip:=<控制器IP> local_ip:=<本机网卡IP>
```

- 将 `<机型>` 换成你的机型，例如 `CR7`、`CR35`。
- **仅验证软件链与 RViz**：可使用 `use_fake_hardware:=true`（IP 仍可按需填写）。
- **连接真机与 HMI**：使用 `use_fake_hardware:=false`，并把 `robot_ip`、`local_ip` 改为现场控制器与本机地址。

待终端 1 无致命报错、RViz 中 Motion Planning 可用后，再打开**终端 2**（同样 `source install/setup.bash` 或当前环境的等价命令），执行第 4 节中的各 demo 命令。

**终端 2（在整机已稳定运行后再执行）**

- 运行 **`rokae_example`** 下各 demo：`ros2 launch rokae_example <某 launch>.py robot_type:=<机型> ...`。
- 若使用 **`controll_movej.launch.py`**（`rokae_hardware` 包内的 `movej` 节点），同样在终端 1 已启动整机的前提下，于终端 2 执行：

```bash
ros2 launch rokae_hardware controll_movej.launch.py robot_type:=<机型>
```

**不要**在未启动终端 1 整机链路的情况下，单独期望第 4 节中的 MoveIt 类 demo 完成规划与执行。

## 4. 各 Demo 启动方法

以下命令均在**已 source 环境**的 bash 中执行；**请先完成第 3 节终端 1 的整机启动。**

### 4.1 `joint_s_line`（关节空间 S 曲线）

最小启动（以 CR7 为例）：

```bash
ros2 launch rokae_example joint_s_line.launch.py robot_type:=CR7
```

xMate CR35 示例：

```bash
ros2 launch rokae_example joint_s_line.launch.py robot_type:=CR35
```

可选参数：

- `robot_type`：机型，默认 `CR7`

---

### 4.2 `cartesian_s_line`（笛卡尔空间 S 曲线）

最小启动：

```bash
ros2 launch rokae_example cartesian_s_line.launch.py robot_type:=CR7
```

xMate CR35 示例：

```bash
ros2 launch rokae_example cartesian_s_line.launch.py robot_type:=CR35
```

可选参数：

- `robot_type`：机型，默认 `CR7`

---

### 4.3 `follow_joint_position`（关节轨迹点跟随）

最小启动：

```bash
ros2 launch rokae_example follow_joint_position.launch.py robot_type:=CR7
```

常用参数示例：

```bash
ros2 launch rokae_example follow_joint_position.launch.py robot_type:=CR7 point_period_s:=0.6 start_blend_s:=0.6 goal_tolerance_rad:=0.01 vel_scale:=0.2 acc_scale:=0.2 cycle_count:=0
```

参数说明：

- `point_period_s`：相邻轨迹点时间间隔（秒）
- `start_blend_s`：首点预留平滑时间（秒）
- `goal_tolerance_rad`：关节目标容差（弧度）
- `vel_scale`：速度缩放 $(0,1]$
- `acc_scale`：加速度缩放 $(0,1]$
- `cycle_count`：往复循环次数，`0` 表示无限循环

---

### 4.4 `follow_cart_position`（笛卡尔正弦跟随）

最小启动：

```bash
ros2 launch rokae_example follow_cart_position.launch.py robot_type:=CR7
```

常用参数示例：

```bash
ros2 launch rokae_example follow_cart_position.launch.py robot_type:=CR7 amplitude_m:=0.4 period_s:=1.33 target_update_hz:=1.0 control_hz:=5.0 kp:=0.6 filter_alpha:=0.25 duration_s:=20.0 vel_scale:=0.08 acc_scale:=0.08 eef_step:=0.002 min_fraction:=0.95
```

参数说明：

- `amplitude_m`：Y 轴摆动振幅（米）
- `period_s`：正弦周期（秒）
- `target_update_hz`：目标点更新频率（Hz）
- `control_hz`：控制步执行频率（Hz）
- `kp`：比例跟踪系数
- `filter_alpha`：一阶滤波系数 $(0,1]$
- `duration_s`：总时长（秒）
- `vel_scale`：速度缩放 $(0,1]$
- `acc_scale`：加速度缩放 $(0,1]$
- `eef_step`：笛卡尔路径插值步长（米）
- `min_fraction`：局部路径最小完成率

---

### 4.5 `servoj_demo`（SDK ServoJ 示例）

最小启动：

```bash
ros2 launch rokae_example servoj_demo.launch.py
```

指定网络与参数：

```bash
ros2 launch rokae_example servoj_demo.launch.py robot_ip:=192.168.21.10 local_ip:=192.168.21.131 movej_speed:=0.2 cycle_ms:=20 duration_s:=30.0 period_s:=4.0 servoj_lookahead_s:=0.06 servoj_kp:=1.0
```

参数说明：

- `robot_ip`：机器人控制器 IP
- `local_ip`：本机网卡 IP
- `movej_speed`：MoveJ 速度缩放 $(0,1]$
- `cycle_ms`：ServoJ 指令周期（毫秒）
- `duration_s`：摆动总时长（秒）
- `period_s`：余弦周期（秒）
- `servoj_lookahead_s`：ServoJ lookahead 时间
- `servoj_kp`：ServoJ 增益

## 5. 常见问题

1. 提示找不到 `rokae_xMate<机型>_moveit_config`：
   - 检查机型拼写与包是否已编译安装；CR35 对应包名为 `rokae_xMateCR35_moveit_config`。
2. 提示 SRDF 或 `kinematics.yaml` 找不到：
   - 确认对应机型配置包的 `config` 目录完整。
3. 提示命令不存在或包找不到：
   - 重新执行 `colcon build` 并 `source install/setup.bash`（或当前 shell 的等价命令）。
4. `servoj_demo` 无法通信：
   - 核对 `robot_ip`、`local_ip`、网段与防火墙设置。
