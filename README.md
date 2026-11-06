# my_jearm_moveit_config

L_JEARM 的 MoveIt2 配置包（ROS 2 Humble）。

本仓库提供：
- MoveIt2 规划配置（OMPL / Pilz）
- `move_group` + `rviz2` 启动
- `ros2_control` 控制链路（真机）
- Fake 执行模式（不依赖 `ros2_control`）
- 常用清理/重启脚本（推荐 `clean_restart.sh`）

## 二代左臂 LH_MK2（feat/lh-mk2 分支）

二代模型使用独立入口 [launch/lh_mk2.launch.py](launch/lh_mk2.launch.py)。
原有一代单臂、双臂入口和清理脚本保持不变，**不会自动切换为二代**。

构建并加载工作空间后，使用 `ros2 launch my_jearm_moveit_config lh_mk2.launch.py`。
无界面验证可追加 `start_rviz:=false`。请在**没有真机控制器的独立 ROS_DOMAIN_ID**
中运行；模拟器仍使用 `/joint_states` 和 `/jearm_controller/follow_joint_trajectory`，
不能与真机或一代仿真栈同时运行在同一 ROS domain。

配置与资源：
- [config/LH_MK2.urdf](config/LH_MK2.urdf)：来自用户指定的 `LH_MK2` 导出模型；
  保留几何、惯量、关节原点和轴向，仅调整网格 URI 与材质命名。
- [config/meshes/lh_mk2](config/meshes/lh_mk2)：随包安装的八个 STL 网格，
  不再依赖移动硬盘，不覆盖一代网格。
- [config/LH_MK2.srdf](config/LH_MK2.srdf)：`jearm` 规划组，
  `base_link` → `Link7`，关节 `joint1`～`joint7`；`zero` 仅代表 CAD 零姿态。
  只屏蔽实际相邻连杆碰撞，其余自碰撞检查保持启用。
- [config/joint_limits_mk2.yaml](config/joint_limits_mk2.yaml)：专属离线规划限制，
  不复用一代位置限位。仿真速度上限 0.5 rad/s、加速度上限 1.0 rad/s²，
  默认缩放均为 0.1；这些都不是已验证的硬件参数。
- [config/ompl_planning_mk2.yaml](config/ompl_planning_mk2.yaml)：OMPL / RRTConnect，
  复用已有单臂 KDL 求解器和模拟轨迹控制器映射。

**当前仅适配离线规划和模拟执行，不支持真机。** 源模型明确注明所有关节的
±31.4 rad 范围仅供离线兼容测试，因此按原样保留，不猜测二代机械限位，
也不移植一代 `JeZmqSystemHardware`。`use_fake_executor:=false` 会在启动节点前报错。

真机适配前仍需确认：七轴实际位置/速度/力矩限制、零位偏置、方向符号、
驱动关节顺序与通信配置、工具 TCP 相对 `Link7` 的变换，以及二代模型的自碰撞矩阵。
不要为使规划通过而直接屏蔽非相邻碰撞。

静态配置回归检查见 [tests/test_lh_mk2_config.py](tests/test_lh_mk2_config.py)。

2026-11-06 验证记录：
- 7 项静态配置检查通过；源模型关节原点、轴向、惯量、限位逐项一致，八个网格 SHA-256 一致。
- 在隔离域 `217` 中复用已有安装的 C++ 可执行程序，使用源目录资源进行无界面验证；
  未启用任何真机驱动，拒绝真机模式的检查通过。
- 正运动学成功，CAD 零姿态无自碰撞；`Link7` 零位位置约为
  `(0.865340, 0.229998, 1.301002)` m，相对 `base_link`。
- `joint1` 目标 0.1 rad 的 OMPL 规划成功（8 个轨迹点），模拟执行器返回成功。
  此项不等于验证了所有工作空间、笛卡尔路径或真机运动。
- 本机 `move_group` 初始化约需 14 秒；关闭时观察到系统 MoveIt 2.5.9 / rclcpp
  析构阶段段错误（退出码 -11），原因尚未定位，不应将本次测试视为无错误退出。
  还存在 KDL 根连杆惯量警告、未配置 Octomap 三维传感器提示。
- VS Code 的 CMake 构建入口无法完成项目配置，尚未完成本次改动的重新构建、安装及
  RViz 图形验证；现有安装中还没有二代新增资源，需完成正常构建安装后再使用包启动命令。

---

## 1. 目录结构

```text
my_jearm_moveit_config/
├── config/
│   ├── L_JEARM.urdf
│   ├── L_JEARM.srdf(.xacro)
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── planning_pipelines.yaml
│   ├── moveit_controllers.yaml
│   └── ros2_controllers.yaml
├── launch/
│   ├── je_arm_control_stack.launch.py
│   └── moveit_rviz.launch.py
├── rviz/
│   └── moveit.rviz
└── scripts/
    ├── clean_restart.sh
    ├── cleanup_ros.sh
    └── launch_arm.sh
```

---

## 2. 依赖环境

- Ubuntu + ROS 2 Humble
- MoveIt2（`moveit_ros_move_group`, `moveit_planners_ompl`, `moveit_ros_visualization` 等）
- `python3-yaml`
- 可选 IK 插件：`pick_ik`

> 包依赖以 `package.xml` 为准。

---

## 3. 构建

在工作空间根目录（示例：`~/ros2_ws`）：

```bash
cd ~/ros2_ws
colcon build --packages-select my_jearm_moveit_config
source install/setup.bash
```

---

## 4. 推荐启动方式（你当前常用）

推荐脚本：`scripts/clean_restart.sh`

它会执行：
1. 杀掉残留 ROS 相关进程（`move_group`、`ros2_control_node`、`rviz2` 等）
2. 重启 `ros2 daemon`
3. 临时切换 `ROS_DOMAIN_ID=199`，隔离旧 DDS 上下文
4. 启动 `je_arm_control_stack.launch.py`

使用：

```bash
cd ~/ros2_ws
source install/setup.bash
bash src/my_jearm_moveit_config/scripts/clean_restart.sh
```

---

## 5. 直接启动（不走脚本）

### 5.1 总入口（支持 fake / real）

```bash
ros2 launch my_jearm_moveit_config je_arm_control_stack.launch.py
```

可选参数：
- `use_fake_executor:=true|false`
  - `true`：fake 模式（默认）
  - `false`：真机模式（委托到 `moveit_rviz.launch.py` + `ros2_control`）
- `start_rviz:=true|false`

示例：

```bash
# Fake + RViz
ros2 launch my_jearm_moveit_config je_arm_control_stack.launch.py use_fake_executor:=true start_rviz:=true

# Real + RViz
ros2 launch my_jearm_moveit_config je_arm_control_stack.launch.py use_fake_executor:=false start_rviz:=true
```

### 5.2 真机链路启动

```bash
ros2 launch my_jearm_moveit_config moveit_rviz.launch.py start_rviz:=true
```

---

## 6. 关节限制说明（重要）

本仓库当前采用“双层一致”策略：

1. **URDF 硬限制（根源）**：`config/L_JEARM.urdf`
   - 每个关节 `<limit lower upper velocity effort>`

2. **MoveIt 规划限制**：`config/joint_limits.yaml`
   - `has_position_limits / min_position / max_position`
   - `has_velocity_limits / max_velocity`
   - `has_acceleration_limits / max_acceleration`

建议始终保持两处一致，避免“模型能到但规划不放行”或“规划能出但硬件越界”。

---

## 7. 常用脚本

- `scripts/clean_restart.sh`：彻底清理并用临时 Domain 重启（推荐）
- `scripts/cleanup_ros.sh`：只做清理，不启动
- `scripts/launch_arm.sh`：启动包装器，退出时自动清理子进程

如需执行权限：

```bash
chmod +x src/my_jearm_moveit_config/scripts/*.sh
```

---

## 8. 快速排查

### 8.1 看节点是否起来

```bash
ros2 node list
```

常见应包含：
- `/move_group`
- `/controller_manager`
- `/joint_state_broadcaster`
- `/jearm_controller`

### 8.2 验证 MoveIt 运行时关节限制

```bash
ros2 param list /move_group | grep robot_description_planning.joint_limits
ros2 param get /move_group robot_description_planning.joint_limits.joint1.min_position
ros2 param get /move_group robot_description_planning.joint_limits.joint1.max_position
```

### 8.3 常见问题

- **旧节点残留 / 参数读错**：先执行 `clean_restart.sh`
- **看不到实时姿态**：确认 `joint_state_broadcaster` 正常运行
- **规划能出但执行超时**：检查 `ros2_controllers.yaml` 的约束和硬件反馈

---

## 9. 维护建议

- 修改 URDF 关节上下限后，同步更新 `joint_limits.yaml`
- 修改控制器行为时，优先检查：
  - `config/ros2_controllers.yaml`
  - `config/moveit_controllers.yaml`
- 优先使用 `je_arm_control_stack.launch.py` 作为统一入口
