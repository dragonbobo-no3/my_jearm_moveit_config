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
**2026-11-09 起，不带参数启动默认接收控制器 SDK 反馈：实体臂或控制器 SDK 仿真使用同一入口、同一协议，不使能或发送运动。**
控制器启动时成功检测到零从站会选择隔离 SDK 仿真；预期七轴则进入原有真实启动保护流程。
检测失败或从站布局异常会拒绝启动，不把故障当作仿真。上位机无需区分后端，也不会因缺反馈切换 fake。
**方向修正（2026-11-09）：用户确认示教器与实体臂方向一致，因此 SDK 反馈模型保留原 URDF
的全部关节轴向和原点，不再强制改成 +Z / MDH 链。J4、J6 恢复原文件的近似 −Z 轴；
SDK 角度不取反、不增加偏置。之前“与 MDH 一致”的测试不能证明实体方向正确。**
无界面验证可追加 `start_rviz:=false`。
离线 CAD 模拟必须显式指定 `use_fake_executor:=true`，并在
**没有真机反馈发布者的独立 ROS_DOMAIN_ID** 中运行；模拟器仍使用 `/joint_states`
和 `/jearm_controller/follow_joint_trajectory`，不能与真机或一代仿真栈同时运行在同一域。

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

**默认使用控制器只读反馈及独立的 SDK 坐标模型；本机 CAD fake 模拟需显式选择。** CAD 源模型的 ±31.4 rad
仅供离线兼容测试，原样保留；实体模式按用户确认的控制器参数生成位置限制和关节正向，
不使用该离线范围。不要为使规划通过而直接屏蔽非相邻碰撞。

### 统一 SDK 反馈入口（实体臂 / 控制器 SDK 仿真）

启动文件：[launch/lh_mk2_hardware.launch.py](launch/lh_mk2_hardware.launch.py)。
二代总入口默认 `use_fake_executor:=false`，实体臂和控制器 SDK 仿真都进入此入口。
**旧的一代总入口和清理脚本仍不会自动切换二代模型。**

构建安装后可使用以下启动方式：
- 只读反馈：`ros2 launch my_jearm_moveit_config lh_mk2_hardware.launch.py`。
  默认 `hardware_mode:=feedback`，只订阅控制器 SDK 状态、发布 `/joint_states`，
  不启动硬件写驱动或 fake 执行器，MoveIt 的轨迹执行被关闭。
- 无界面只读反馈可追加 `start_rviz:=false`。
- 控制入口目前暂停：`hardware_mode:=control` 在任何节点启动前被拒绝，
  即使 `confirm_enable:=true` 也不能绕过。恢复原 URDF 后，必须重新核验 SDK 命令、
  示教器显示与实体臂坐标映射，之后才可解除该保护。
  现有命令能力预检仍保留；仅有七轴反馈不代表具备命令链路。
  后续命令桥接必须显式声明支持命令，且另行现场验收。
  **这不是只读连接：既有驱动激活会发送 `Switch=true`，随后持续发送当前位置保持命令，
  退出时还可能发送去使能。** 必须由现场确认安全、可靠支撑及急停可用后才启动。
  此次没有运行该写驱动，也没有发送真实轨迹；该入口尚未完成实体联调验收。

参数：`robot_ip`（默认 `192.168.0.99`）、`state_port`（默认 `8000`）、
`command_port`（默认本机绑定 `8001`）、`state_timeout`（默认 5 秒）、
`startup_timeout`（等待第一帧默认 15 秒）。只读入口先仅启动接收器，收到完整有效首帧后
才启动模型、MoveIt 和 RViz；首帧超时不启动这些进程，避免边加载边退出的闪退链。
只读模式断流后停止发布并退出整个本机启动链路，绝不补零或把旧值加新时间戳重复发布。
控制模式在启动驱动前，必须通过纯订阅预检：完整、有限、在控制器限位内的七轴反馈；
随后等待控制器管理器、状态广播器和轨迹控制器依次就绪，不使用固定延时冒充硬件就绪。

本机配置与坐标约定：
- [config/hardware_mk2.yaml](config/hardware_mk2.yaml)：从当前控制器 DeviceConfig
  只读提取的参数快照，附源路径与 SHA-256。用户确认该文件是已调好的二代配置。
  若控制器参数后续变更，应重新核对快照，而非自动覆盖控制器参数。
- [launch/lh_mk2_support.py](launch/lh_mk2_support.py)：保留原 URDF 的七轴 `axis`、`origin`、
  连接关系、网格、碰撞和惯量；只叠加硬件限位、接口及独立固定法兰。
  不再从 MDH 改写关节方向或原点，J4/J6 的近似 −Z 轴与用户确认的示教器模型一致。
- 源 CAD 的 `Link7` 位于距 J6 95 mm 处，控制器 MDH 末节为 122.1 mm，故新增
  `sdk_flange`（沿 `Link7` 局部 Z 延长 27.1 mm）作为实体模式规划末端。
  该固定扩展予以保留，但新模型与 SDK 法兰/TCP 的完整坐标对应仍需重新核验；
  不能用旧模型的 MDH 一致性测试替代这项核验，外部工具仍需单独配置。
- `joint1..joint7` 直接对应 `Robot0.joint[0..6]` 的 SDK 弧度坐标；
  编码器 `initial_offset`、`direction`、`encoder_ruler` 已由控制器处理，ROS 不二次补偿。
- 实体位置限位依次为 J1 ±5.14、J2 [-1.8231, 1.7302]、J3 ±2.96、
  J4 [-1.4828, 2.3798]、J5 ±2.96、J6 [-1.4778, 1.4697]、J7 ±2.96 rad。
  规划速度/加速度额外限制为不超过 0.3 rad/s、0.5 rad/s²，默认缩放均为 0.1。
- [config/ros2_controllers_mk2.yaml](config/ros2_controllers_mk2.yaml)：七轴闭环位置控制，
  不接受缺轴目标；单臂输出固定为 `JE_ARM_CONTROLLED_ARM=left`，不向 Robot1 输出。
- [scripts/mk2_state_bridge.py](scripts/mk2_state_bridge.py)：纯 ZMQ SUB 反馈桥接，
  也提供 `--probe-only`，不绑定任何命令端口，依赖系统 Python 的 `pyzmq`/ROS `rclpy`。

**2026-11-06 真实反馈已打通：** 原先二代独立配置的插件列表为空，状态端口
`8000` 未监听。用户随后确认七轴退使能且机械可靠支撑，明确授权备份后修改桥接配置
及受控重启。现已加载新编写的
[只读状态插件](controller_bridge/mk2_state_publisher.c)，真实反馈约 100 Hz。
仅修改二代配置的插件列表；DeviceConfig、BusConfig 与备份逐字节一致，未重新取零，
未加载旧夹爪逻辑，未启动写驱动，未使能或发送运动指令。

**当前边界是“只有反馈、没有命令接收器”，不是完整运动控制验收。**
当前插件明确发布 `commands_enabled: false`；控制入口预检会拒绝它及任何未显式声明
命令能力的桥接，避免只因看到关节状态就启动会自动使能的旧驱动。
能力声明本身也不等于验证命令链路或安全保护。后续若增加命令桥接，需另行审查
命令目标 IP、使能/停止机制、状态超时处理及现场低速运动测试。
2026-11-09 已新增独立的关节控制桥接和假 SDK 离线测试，**尚未部署，现场仍仅有反馈**。
新桥接不自动使能/下使能，不清错或恢复强制停止；Switch=true 只等待首帧，
保持目标验证通过后才接管 SDK 伺服，错误/失联停止后锁存。
保护、测试证据、已知边界及部署前置条件见
[controller_bridge/CONTROL_BRIDGE.md](controller_bridge/CONTROL_BRIDGE.md)。
不要直接恢复会自动初始化旧夹爪等设备的旧 JEServer。
部署路径、备份、日志、哈希和验收结果见
[controller_bridge/COMMISSIONING.md](controller_bridge/COMMISSIONING.md)。
2026-11-09 SDK 仿真同协议发布、控制器无参数自动后端选择及已安装默认入口验收见
[controller_bridge/UNIFIED_BACKEND.md](controller_bridge/UNIFIED_BACKEND.md)。

旧硬件插件在运行中尚无完整的状态超时安全停机策略；本机预检、进程退出和 ROS
轨迹容差不是驱动安全功能，也不能替代硬件急停和控制器自身的保护。

按用户要求，开发阶段新增的本机测试脚本和假 SDK 夹具已清理；下方测试数量与结果为
清理前的历史验收记录，不表示仓库当前仍提供这些测试。正式运行代码、安全检查及
控制器上的部署备份不受此次清理影响。

2026-11-06 首次离线验证记录（下列“拒绝真机”描述为当时行为，现已由上述显式确认替代）：
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

本次实体入口验证：19 项配置/协议检查通过，其中 40 组关节姿态验证生成模型
与当前控制器 MDH 的法兰正运动学一致；本机回环 ZMQ → ROS 反馈与断流退出验证通过，
未确认使能、无状态反馈及只读插件三种情况下写驱动均被阻止。
真实数据验收在隔离域 `218` 中以源目录资源和已有 C++ 程序完成：601 帧约 100 Hz、
序号无丢失；ROS 只有一个真实关节状态发布者，当前姿态无自碰撞；MoveIt 法兰 FK
与当前控制器 MDH 一致，TF 位置误差约 0.035 mm（采样时刻不同）。
本机测试进程已退出，控制器只读发布仍运行，七轴复核均退使能、无故障。
MoveIt 关闭阶段再次观察到上述已有析构段错误，不能将启动器退出码 0 视为无错误退出。
VS Code CMake 入口仍无法配置项目，新增 ROS 文件尚未完成重新构建安装；
本次源目录验收不等于已安装包验收，尚未进行写驱动联调或运动验证。

2026-11-09 “总入口收不到反馈”排查：当天日志显示旧默认值启动了 `OFFLINE ONLY`
和 fake 执行器；控制器纯 SUB 探测可收到真实七轴状态，位置均在控制器限位内。
同时发现本机旧 symlink-install 只有二代总入口，缺少新增硬件入口、支持模块与反馈脚本。
因此将二代默认值改为实体只读反馈，并补充默认不使能的回归测试。
20 项配置/协议检查通过；启动描述干运行验证默认转入实体入口、显式模拟仍保留，
并确认旧安装缺资源时在任何节点启动前给出明确的重新编译安装提示。
**必须重新编译安装本包后再启动**：已有总入口软链接更新不代表新增文件已安装；
本轮编译由用户执行，不将纯订阅探测误记为完整安装入口验证。

上述缺安装资源记录为当时状态：2026-11-09 后续确认新增资源均已安装为源文件软链接。
本轮没有替用户重新编译；已直接验收安装入口的统一 SDK 仿真反馈、RViz/MoveIt 启动与 FK。
31 项配置/协议/后端选择/首帧就绪测试通过。正常退出时系统 MoveIt/RViz 析构崩溃仍存在，
不将启动阶段修复表述为整个退出过程无错误。

后续方向修复验收：**35 项测试通过**，新增七轴轴向/原点保留、J4/J6 正角运动、
网格/碰撞/惯量不变及 40 组按真实 `axis` 计算的原模型运动一致性回归。
隔离域 `223` 实际读取已安装入口的 `robot_description`，确认七轴轴向和原点逐项一致，
收到 1180 帧 SDK 关节反馈，MoveIt FK 服务就绪。确认控制请求在任何节点启动前被拒绝。
这里只验证模型修复与只读链路，没有运动命令或实体方向复验；需要用户重新启动本机
MK2 launch 使已有界面加载新模型。已安装文件为源文件软链接，本次未重新编译或重启控制器。

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
