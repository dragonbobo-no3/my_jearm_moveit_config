# my_jearm_moveit_config

L_JEARM 的 MoveIt2 配置包（ROS 2 Humble）。

本仓库提供：
- MoveIt2 规划配置（OMPL / Pilz）
- `move_group` + `rviz2` 启动
- `ros2_control` 控制链路（真机）
- Fake 执行模式（不依赖 `ros2_control`）
- 常用清理/重启脚本（推荐 `clean_restart.sh`）

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
