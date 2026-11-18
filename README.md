# my_jearm_moveit_config

L_JEARM 的 MoveIt2 配置包（ROS 2 Humble）。

本仓库提供：
- MoveIt2 规划配置（OMPL / Pilz）
- `move_group` + `rviz2` 启动
- `ros2_control` 控制链路（真机）
- Fake 执行模式（不依赖 `ros2_control`）
- 常用清理/重启脚本（推荐 `clean_restart.sh`）

## 二代左臂 LH_MK2（feat/lh-mk2 分支）

以下为分阶段记录，当前状态以首节为准；历史测试数量不代表当前总数。

### 最新：真实 ROS action 接口隔离联测通过（2026-11-13 15:37）

[实验 action 执行端](src/je2_ros_action_executor.hpp)已把实际 `FollowJointTrajectory` 接单、
位置反馈、取消应答及一次性结果发布接到目标生命周期，不再只有内部结果枚举。
取消必须等关闭回执；取消期间故障仍返回 ABORTED，普通结束仍检查终点容差。
旧目标 UUID 的取消请求不能影响新会话目标，局部退出不会冒充有回执的 CANCELED。
**仅测试端接通，不是生产驱动/MoveIt 启动接线，也不是完整自动交接已完成。**

构建和七个 CTest 目标全部通过：新增真实 action 测试 14 项，加原 61 项，共 **75 个 gtest**；
另有三个 C 回归目标通过。测试启动了本机 ROS 节点，显式域 217、独立命名空间，
DDS 配置及实际 participant QoS 均验证为仅回环 UDP/本机共享内存；不是实机网络或认证边界。
控制器仍为内存 IO/替身 writer，时钟为模拟时钟；没有运行真实 SDK、连接或安装实体控制器。
没有默认运行入口、自动 claim、故障恢复、轨迹插值或重复 Servo 能力。
调用合同与完整限制见[协议记录](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 阶段记录：实验目标生命周期与结果判定通过（2026-11-13 15:02）

新增[目标执行组件](src/je2_ros_goal_execution.hpp)，连接定时发送、最终 ACK、显式收尾和一次性结果。
正常结束要求已确认关闭的同会话反馈仍新鲜，且最终位置满足显式七轴容差；仅 ACK/关闭不算目标成功。
取消立即进入收尾，待回执后才产生取消结果；失权、状态故障、超时或发送失败均中止，不冒充取消成功。
接单要求轨迹已包含最终保持区间，不自动补点、重定时或修正移动终点。
**此阶段仅有内部结果；后续真实 ROS action 测试接线见首节，生产执行器仍未接通。**
构建和六个 CTest 目标全通过：隔离联测 42 项、适配器 12 项、共享时钟 7 项，共 61 个 gtest；
三个 C 回归目标也通过。本轮没有 SDK、控制器连接、实体操作或启动默认值修改。
见[目标生命周期合同与验收范围](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 最新：实验会话显式收尾与退出确认通过（2026-11-13）

[客户端](controller_bridge/je2_session_client.h)和[ROS 适配器](src/je2_ros_goal_adapter.hpp)新增显式收尾：
冻结新目标，等待已有帧确认，再生成一次带原会话身份的 Release；不跳序、不重发。
Owner 回执关联实际接收成功的释放序号，只有原引擎完成 writer 退出及退出后的静止屏障，才可确认收尾。
End 返回、状态空闲、消费序号前移均不能单独确认；失权只退役，异常仍锁存。
旧服务器缺少回执字段时在生成 Release 前拒绝。`Confirmed` 是该次引擎屏障的历史回执，
**不是到位、当前物理停止、ROS goal 成功或允许再次 Servo 的证明**。
构建和六个 CTest 目标通过：隔离联测 32 项、适配器 12 项、共享时钟 7 项，共 51 个 gtest；
C 协议普通/ASan/UBSan及旧 Owner 兼容测试也通过。新增测试仍用内存运动回调和显式替身 writer 证据。
没有改变引擎停止/恢复策略、默认启动或实机安装，未运行真实 SDK；原生停止来源及完整自动再接管仍未解决。
收尾合同、期限和取消调用区别见[协议记录](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 最新：单步定时发送与过期停发通过（2026-11-13）

[ROS 适配器](src/je2_ros_goal_adapter.hpp)新增 `bind_scheduled` / `dispatch`：在接单时固定迟到预算，
每次调用最多生成一帧；未到时不发送，背压不跳点，超过该点期限就锁存停发，不追赶或重定时。
迟发后也不压缩实际生成帧间隔；定时目标不能绕过调度使用底层 `prepare`。
`Drained` 仅表示所有目标帧已获得匹配的健康确认，**不是到位、停止或 ROS 执行成功**。
取消仍是本地退役，不冒充物理停止确认；不自动续发保持点，后端 watchdog 继续有效。
构建和六个 CTest 目标通过：隔离联测 22 项、适配器 12 项、共享时钟 7 项，共 41 个 gtest；
另有旧 Owner/C 协议普通/C 协议 ASan/UBSan 三个 C 目标通过。
**仍为测试构建内的调度接口，不是完整 action server/JTC 执行器。** 本轮未接线生产启动、连接控制器或操作实机；
调度合同及剩余完成/取消边界见[协议记录](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 最新：隔离会话握手与 ROS 目标联测通过（2026-11-13）

[进程内联测](test/je2_ros_session_loopback_test.cpp)已连接实际会话解析器、控制引擎、
Owner 序列化和 ROS 目标适配器；claim → Switch → 首次保持 → ACTIVE/ACK 均由协议交互产生，
不再手工设置客户端确认状态。底层运动回调、反馈、时钟及 writer 退出证据仍为测试替身。
11 项联测覆盖连续 25 点、背压、失权后的旧帧、丢包/迟到 ACK、回调失败、服务端跟随误差、
状态 7 锁存、重启身份隔离、multipart 和本地取消边界，均通过。
编辑器构建成功，六个 CTest 目标全通过：30 个 gtest，加旧 Owner 固定样本兼容、C 协议普通及 ASan/UBSan 三个 C 测试目标。
仅使用 `inproc://` / ZMQ_PAIR；没有 TCP、ROS 节点、SDK、真实 writer 或控制器连接。
**不是 action server/JTC 接线、原生示教器操作、第二次 Servo 或完整自动交接验收。**
实验插件的 Owner 格式化已提取为[共享函数](controller_bridge/jeserver_v2_owner.inc)供联测复用；
本轮未重编/运行完整插件、未修改启动默认值或部署。详见[联测证据与限制](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 最新：ROS 目标绑定与运动约束检查通过（2026-11-13）

新增[ROS 消息适配边界](src/je2_ros_goal_adapter.hpp)及[gtest 用例](test/je2_ros_goal_adapter_test.cpp)，
在接收目标时绑定完整会话身份并保存不可变目标副本；旧回调不能在执行时重新获取当前身份。
顺序点提交、背压、退役、旧取消和发送失败均通过目标票据隔离。
现要求显式七轴限位配置，绑定前预检全部点的位置、声明速度/加速度与命令离散差分；
基准来自实际首次封装的保持目标而非后续反馈，只有成功生成封套才推进发送历史。
不会限幅、插值、重定时或追赶迟到点；测试限位仅为构造值，不是实体臂参数。
**仅 BUILD_TESTING 下的构建/测试边界，已通过实际 CTest 执行**，不是已接通的 action server/JTC/硬件驱动。
编辑器独立构建通过；`je2_ros_goal_adapter_test` 12/12、`mk2_shared_clock_cycle_test` 7/7，零失败；
独立 C 客户端的普通及 ASan/UBSan 协议回归也已接入 CTest 并通过，合计四个测试目标。
[编辑器设置](.vscode/settings.json)使用独立构建目录和完整的 ROS 环境；
`AMENT_PREFIX_PATH` 用于资源索引，`PYTHONPATH` 必须同时传给配置和测试启动器，而非仅配置阶段。
原 colcon 构建/安装目录不变；验证细节见[协议记录](controller_bridge/PC_SESSION_PROTOCOL.md)。
默认插件、launch 和实机安装未改动，完整自动交接仍未完成。

### 最新：实验客户端支持有界连续目标（2026-11-13）

新增完整会话 lease 校验和显式目标封装：最多一帧等待确认，不积压、不补发；
即使反馈持续正常，目标确认超时也会锁存停发。旧回调在检查数据前隔离，不能污染新会话。
普通/ASan/UBSan、实际私有 ZMQ 下连续 25 个替身目标、丢包超时及四类服务端运动保护测试通过。
**未接入 ROS、未部署；不是完整轨迹执行或原生自动再接管验收。**
见[接口时序、调用责任和验证边界](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 最新：独立会话客户端完成私有联测（2026-11-13）

新增[客户端绑定组件](controller_bridge/je2_session_client.h)：显式 claim → 确认 → Switch →
从确认后反馈生成首次保持。Owner 单条快照同时包含启动/会话身份、阶段、错误和关节反馈，
不拼接两条独立消息的“最新值”。失权/重启永久退役，异常/超时锁存；不会发送旧目标或旧停止。
普通与 ASan/UBSan、实际 ZMQ/完整插件/替身 SDK 联测和默认插件回归通过。
**本节记录首次保持版本；后续目标接口见上节。组件仍未接入 ROS 驱动；编辑器构建/测试现已恢复，未安装或操作实机。**
完整原生自动交接和再接管仍未完成；见[客户端范围与验证](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 最新：实验新会话通信入口已接通（2026-11-13）

显式会话实验新增每次加载随机 boot 标识、claim 确认和 client/代次/序号封套。
真实 ZMQ/完整插件/内核线程及替身 SDK 验证旧目标、停止、释放、multipart 和重启旧帧均被隔离；
状态 7 及后续健康反馈仍不能重新 claim。普通与 ASan/UBSan、默认插件及原交接回归通过。
**本节记录控制器侧实验通信；后续新增的独立客户端见上节。ROS 驱动未接线、第二次 Servo 未验收、未部署。**
默认协议和生产拒绝不变；见[会话协议、ACK 条件和验证范围](controller_bridge/PC_SESSION_PROTOCOL.md)。

### 最新：修复交接请求入口漏锁存瞬态异常（2026-11-13）

实验引擎现在在有效点动/再接管/PC 请求入口读到异常时立即锁存，不再等下一次 tick；
避免异常只出现于两次轮询之间、后续健康反馈掩盖它。旧代次隔离和初始退使能未就绪语义保留。
新增 105 组回归先复现旧问题、修复后普通与 ASan/UBSan 通过，完整替身插件沙箱回归也通过。
SDK 只读审计另确认两种停止共用 `order=1`，`get_robot_order1` 还会改写状态，不能作为只读原因接口。
**仅实验源码修复，未部署；自动交接仍未完成。** 详见[入口锁存修复与停止接口审计](controller_bridge/tests/SDK_HANDOVER_REVIEW.md)。

### 最新：停止来源对照确认现有反馈不足（2026-11-13）

真实 SDK 私有虚拟测试中，原生 teach STOP、外部 `RobotStop`、自身 STOP 后追加 `RobotStop`
均可得到 `Power=1 / MoveState=7 / 两类 Error=0`。新增两组测试均通过且没有再次点动或恢复；
因此不能仅凭这些字段或“自己发过 STOP”自动续动。不是硬件急停验收，也未证明外部停止后 SDK 会拒绝点动。
34 类替身控制流及普通/ASan/UBSan 回归通过，实体进程、配置、已安装插件未变。
**自动交接仍未完成**，需要可信停止原因/事件顺序；见[停止来源对照记录](controller_bridge/tests/SDK_HANDOVER_REVIEW.md)。

### 最新：真实 SDK 原生停止/再次点动语义已验证（2026-11-13）

在五私有 namespace 的虚拟后端执行两轮原生点动/STOP：第一次停止后状态 7，
第二次原生点动返回成功、进入状态 1 并产生新的虚拟位移，再停止后仍为 7。
与 SDK 点动接口内部恢复的静态证据一致；未加载控制插件、未启动 Servo、未显式调用恢复，
实体进程/配置/启动器快照未变。**这不证明停止来源已可区分，也不是自动交接或 ROS 再接管通过。**
专用探针、保护回归及实际结果见[原生停止语义验证](controller_bridge/tests/SDK_HANDOVER_REVIEW.md)。

### 最新：原生首次故障诊断已停机安装（2026-11-12）

经现场确认退使能、支撑和停机并授权后，诊断版物理插件已安装，独立指纹与停机核查通过。
只更新插件加载路径，保留原库及回滚备份；标定、限位、控制策略、SDK 和启动保护不变。
新插件增加停止前首次故障快照和伺服状态观察，**不放宽 `state_error`，不自动清错或重新接管**。
现场随后启动控制器，已只读验证 100 帧递增反馈和新诊断字段；保持退使能、命令禁用、无故障，
未发出保持/运动命令。在线反馈通过不等于控制接管、故障复现或自动交接通过。
具体指纹、备份及下一阶段条件见 [实体诊断部署记录](controller_bridge/PHYSICAL_CONTROL.md)。

### 最新：区分正常交接与故障，先实现控制丢失后的只读降级

用户现已明确要求恢复一代的切换体验。**整套自动点动接管/ROS 再接管尚未完成**；
普通 control 的单向降级现已改为：控制进程退出后保留反馈/模型和 RViz 窗口，
只等待旧 `move_group` 实际退出，再启动只规划服务，不恢复驱动、不重放轨迹、不把故障当作示教器获权。
修复了禁用 Execute 接口导致 Humble RViz 客户端等待的问题：保留接口但服务端禁止执行。
51 项 Python 回归通过；另用本机真实 MoveIt 验证 Execute 返回 CONTROL_FAILED 且不转发控制器。
尚未做实际 RViz 窗口/实机点动切换验收；外部 MoveIt 退出崩溃仍存在。
新增启动模块首次使用需要安装本配置包，已有源码符号链接的安装会随修改更新；本轮未部署或重启实体控制器。
行为、安装要求及尚缺的原生交接证据见 [控制丢失降级说明](controller_bridge/CONTROL_LOSS_FALLBACK.md)。

### 当前主线：沿用一代方式，适配二代臂（2026-11-12）

此前范围收敛时，**暂不以示教器优先、自动让位或完整仲裁为基础适配目标**；后续切换需求见上节。
保留原有 MoveIt → ros2_control → ZMQ → 控制器 SDK 结构，只使用二代左臂对应的
模型、七轴映射、限位和已标定控制器配置；不恢复一代的自动清错、上下电或外设初始化。

- ROS 继续使用 [二代统一入口](launch/lh_mk2.launch.py)，实际硬件链路在
  [二代硬件入口](launch/lh_mk2_hardware.launch.py)。不改一代单臂/双臂入口。
- 关节映射固定为 `Robot0[0..6] → joint1..joint7`，SDK 弧度值不再重复叠加编码器零偏或方向。
  [二代标定快照](config/hardware_mk2.yaml) 与 [模型适配](launch/lh_mk2_support.py) 保持不变。
- 控制器沿用既有受保护的 RobotMain_v2 / 二代配置启动方式；不为本轮主线替换正在运行的插件，
  也不退回原始一代 JEServer。真实使能和运动仍须按现有条件单独验收。
- `hardware_mode:=control` 是基本控制路径；`shared`、JEServer_v2、RobotMain_v2_pro
  保留为可选实验，不作为二代基础适配的依赖。下方接管研究结果是历史记录，不代表基本路径被禁用。
- **以当前源码为准：顶层默认 `mode:=sim`、`hardware_mode:=control`、`confirm_enable:=true`**，
  会在后端/反馈预检通过后申请 Servo 并发送保持点，并非只读。只观察时须显式选择
  `hardware_mode:=feedback`；实体后端须显式选择 `mode:=real`，不会因反馈缺失切换到 fake。

本次范围收敛不改变运行代码、启动默认值、标定或保护，也不启动、停止或部署实体控制器。

**接管适配器进展（2026-11-12）：** 已把内核写线程退出观察接入显式仿真插件，
真实 PluginMain / TCP / ZMQ、真实内核线程及替身 SDK 的正常接管、取消和失败场景通过。
真实 SDK/实际插件随后已验证旧 Servo 线程退出、退出后约 108ms 等待、点动虚拟位移和旧 PC 隔离；
但 **STOP 后 SDK 返回状态 7，整体健康空闲验收失败**。没有放行强制停止或自动恢复，
默认路径和生产启动未放开；见[实际证据与当前阻塞项](controller_bridge/tests/SDK_HANDOVER_REVIEW.md)。

**真实 SDK 隔离验证已通过（只读）：** 修复临时副本 ENI 路径后，pro 已在控制器的私有沙箱中实际
初始化真实 SDK、启动新版插件、接收 State/Owner 并成功卸载；`power=0`、`state=0`、命令始终禁用。
原进程/安装配置/启动器快照未变，64 项普通及 ASan/UBSan 保护测试也通过。
这不是自动接管或生产启动验收。详见 [真实 SDK 验收证据与限制](controller_bridge/tests/SDK_LIFECYCLE_RESULT.md)。

**JEServer_v2 最新进展（2026-11-12，未部署）：** 已编译出带 `PluginMain` 的独立仿真共享库，
接入单线程 ZMQ / loopback 文本按钮 / State 与 Owner 发布；实际共享库宿主拒绝、反复加载卸载、
TCP 取消/旧帧隔离和 ASan/UBSan 均通过；真实私有 namespace 中的完整工作线程/卸载假 SDK 测试也已通过。
`RobotMain_v2_pro --build-sim-plugin` 可构建新版，
`--test-plugin` 可运行假 SDK 插件联测，两者均需显式 `JE2_SDK_INCLUDE`。实际 `--sim/--real` 启动仍拒绝，
**生产自动让位尚未可用**：独占 Start / writer 证据的生产接线、完整示教协议和受保护启动尚未验收。
没有部署、重启或实体操作。见 [JEServer_v2 当前范围与验证](controller_bridge/JESERVER_V2.md)。

**历史扩展需求（现已暂停）：示教器优先，现有点动键直接触发 ROS 自动退让，不另按接管按钮。**
该研究曾取代“先到先执行、忙时拒绝”，也不是下面的手动 shared 交接；不再作为本轮基础适配前提。
根据用户的一代参考，现已在控制器旧工程中找到 JEServer 源码及与安装库哈希相同的构建产物。
SDK 本身已有伺服执行状态检查；旧桥不检查 ServoJoint 返回值，二代桥则有故障锁存与停止，
不能再仅凭“头文件没前置回调”断言只能等厂商接口。迁移应区分 SDK 正常失权与真实故障，
而不是恢复旧版自动清错/上下电或忽略所有错误。见[一代查证与二代迁移边界](controller_bridge/LEGACY_TAKEOVER_REVIEW.md)。
继续查证发现：SDK 拒绝新的伺服目标不等于后台伺服线程退出。源码已补首次非伺服状态与
SDK 调用结果诊断，替身回归通过；没有放宽保护，尚未部署，也未实现自动接管。
详见[示教器优先规则与既有手动交接边界](controller_bridge/OWNERSHIP_HANDOVER.md)。

**新增：示教器 / MoveIt 显式交接（源码阶段，未部署）。**
可选 `hardware_mode:=shared`，初始只读，核对硬件/JTC inactive 后由示教器持有入口；
通过 `/mk2/control_owner/set_moveit` 显式申请/交还，不能同时点动和执行轨迹。
服务成功只表示接单，必须等状态话题确认 `MOVEIT` / `PENDANT`。
旧插件缺交接能力时拒绝接管，实体正常释放默认仍关闭；本轮没有实机操作。
21 项 Python 离线测试通过。14:38 复查确认本包控制节点已由用户更新（实际 ELF 14:32），
但配套 `je_software` 仍为 11 月 10 日旧驱动，控制器实体插件也未更新，不能直接使用共享交接。
编辑器 C++ 构建/CTest 的配置问题仍未解决，尚非实机可用验收。
默认 control 不变；详见[交接接口、保护及部署前提](controller_bridge/OWNERSHIP_HANDOVER.md)。

二代模型使用独立入口 [launch/lh_mk2.launch.py](launch/lh_mk2.launch.py)。
原有一代单臂、双臂入口和清理脚本保持不变，**不会自动切换为二代**。

构建并加载工作空间后，使用 `ros2 launch my_jearm_moveit_config lh_mk2.launch.py`。
**当前不带参数启动默认连接 SDK 仿真并申请基本控制/保持点，不是只读。**
实体臂和 SDK 仿真使用相同反馈协议；只观察请选择 `hardware_mode:=feedback`，实体后端须显式选择 `mode:=real`。
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
- **SDK 仿真控制**：选择 `mode:=sim`（默认），使用 `hardware_mode:=control` 和
  `confirm_enable:=true`，并先由用户在已连接的仿真示教器使能模拟轴。
  纯 SUB 预检必须确认 `control_guarded`、`sdk_simulation`、严格布尔值的
  `simulation_only=true` / `simulation_isolated=true`、`ready`、`fault=none`
  及命令能力；该模式拒绝实体、未知后端或未使能/故障状态。
  **这不是只读连接：驱动激活发送 `Switch=true`，随后持续发送当前位置保持命令。**
  原生桥不自动使能/退使能/清错/恢复停止。禁止示教器并发点动及夹爪指令。
  2026-11-09 已完成 MoveIt → JTC → ZMQ → 原生 SDK 仿真七轴小幅往返及断流验收，
  但未进行实体臂运动验收；完整坐标/TCP、实体方向与安全停机仍需现场核验。
  注意：默认只读模式下，MoveIt 可能对 Plan & Execute 输出成功，实际上只做了规划；
  应以执行开关、JTC 结果及原生反馈确认运动，不能仅看 RViz 的 executed 字样。

**共享采样时钟修正（已由用户构建安装，完整联调仍待验收）**：MK2 控制入口使用本包的
[mk2_shared_clock_control_node](src/mk2_shared_clock_control_node.cpp)，每周期只取一次
ROS 时间，供 read/JTC update/write 共用，避免用不同阶段的取时差估算同一轨迹样本。
没有放宽驱动 0.24 / 0.4 或原生 0.3 / 0.5 保护；只读及一代入口不变。
**本次须同时构建安装本包和配套的 `je_software`，不能只编译硬件驱动**；
否则符号链接安装的 launch 可能已经更新，但新增可执行程序尚不存在，启动会失败。
成功启用应看到 `MK2 shared sample clock enabled`；旧版本的往返验收不代表当前
源码已验收，详见[最新调查与验证边界](controller_bridge/SIMULATION_CONTROL.md)。

**实体控制（2026-11-10，两端已更新，运动待验收）**：后端只用一个开关：
`mode:=sim` 为隔离 SDK 仿真，`mode:=real` 为显式实体伺服接管确认，默认仍为 sim。
总入口原有 `hardware_mode:=control`、`confirm_enable:=true` 默认值不变，
因此原启动命令只需追加 `mode:=real`；不再要求 `control_backend` 或
`confirm_physical_control` 两个启动参数。只读仍可指定 `hardware_mode:=feedback`。
实体模式要求真实 guarded 桥、精确配置指纹、已人工使能且空闲的轴和递增反馈；
不会自动切换控制器后端、上电、清错或提交运动目标。模式不匹配仍拒绝启动。
控制器实体插件已备份安装，本机驱动已由用户更新（16:56:51，实体校验标记存在）；
用户日志确认实体桥 ready/none，但实体接管、运动及停止仍待现场验收。
见[实体控制保护、部署前提与验证边界](controller_bridge/PHYSICAL_CONTROL.md)。

**正常退出后重连（仅隔离仿真，两端已安装，用户已确认可运行）**：新增隔离仿真的能力协商式干净释放，正常退出
用 SDK `RobotStopRun`，核对停稳后结束伺服并回到 `ready`；异常仍强停锁存。
本机配套 `je_software` 已安装；本轮已备份并更新控制器端**隔离仿真插件**，
实体配置与仿真配置哈希均不变，部署时未启动控制器或自动使能/恢复/运动。
2026-11-10 用户确认“可以跑了”。本次仅清理本机离线测试产物，未改动运行代码；
这条确认不等同于各项故障、断流与退出重连场景的逐项验收记录。
见[释放协议、备份与部署记录](controller_bridge/SIMULATION_CONTROL.md)。

参数：`robot_ip`（默认 `192.168.0.99`）、`state_port`（默认 `8000`）、
`command_port`（默认本机绑定 `8001`）、`state_timeout`（默认 5 秒）、
`startup_timeout`（等待第一帧默认 15 秒）。只读入口先仅启动接收器，收到完整有效首帧后
才启动模型、MoveIt 和 RViz；首帧超时不启动这些进程，避免边加载边退出的闪退链。
只读模式断流后停止发布并退出整个本机启动链路，绝不补零或把旧值加新时间戳重复发布。
控制模式在启动驱动前，必须通过纯订阅预检：完整、有限、在控制器限位内的七轴反馈，
且满足所选 sim/real 后端的对应控制条件；
随后等待控制器管理器、状态广播器和轨迹控制器依次就绪，不使用固定延时冒充硬件就绪。

**实时反馈不再依赖控制驱动存活**：控制模式预检成功后也启动独立纯 SUB 接收器，
它单独发布全局 `/joint_states`，使控制故障锁存后仍能显示有效的 SDK 实际关节。
控制器广播器改用 `/joint_state_broadcaster/joint_states` 等私有诊断话题，
避免驱动停收后继续用新时间戳发布旧位置。真实状态断流/无有效关节时接收器仍超时退出，
不补零、不回放缓存，并结束启动链路。收到位置不代表控制可用，也不会清除故障。
若由示教器控制、这里只看跟随，应使用 `mode:=real hardware_mode:=feedback`；
不要在本机已经接管保持/轨迹控制时同时示教器点动。当前运行进程不会被源码修改自动替换。

**2026-11-11 11:15 退出记录**：用户确认本机 control 接管期间做过示教器点动。
原生先锁存 state_error；新硬件故障保护约 3 ms 后停止控制，launch 随即关闭启动链路，
不再让失效 JTC 接收目标。其后 MoveIt 2.5.9 / RViz 的 `-11` 是退出过程中的独立段错误，
不能当作正常退出，也不是最初的原生故障；当前尚未修复该外部库退出问题。
本机新保护已编译安装（跟随安装链接的实际 ELF 为 11:06:48，且包含保护标记），
无需为本次日志提示重复编译。若仅看示教器运动，使用上述反馈模式，不占用伺服；
控制器已锁存的故障不会因此清除，也不妨碍有效关节的只读显示。

**2026-11-11 14:38 再次退出**：实际仍是 `mode=real hardware_mode=control`，
用户确认故障前又做过示教器点动/拖动；并未运行新共享交接流程。
原生先报 state_error，约 3 ms 后本机保护退出；MoveIt/RViz 随后退出段错误仍是独立问题。
不能从合并错误反推具体 SDK 状态位；现给原生源码增加停止前的首发采样诊断，
12 个独立 C 场景及实际 JSON 序列化回归通过，SDK 头文件下三种分支语法检查通过。
诊断未部署，不会追溯补出本次已丢失的首发状态。详见[交接现状与诊断说明](controller_bridge/OWNERSHIP_HANDOVER.md)。

**故障后继续接收目标修正（2026-11-11，已由用户构建安装，运动待验收）**：
11 月 10 日 17:24:36 原生 state_error 后，控制驱动已停收，但 JTC 仍接受后续目标。
独立反馈中 joint2 从约 -1.57 变到 0，JTC 却使用旧值，产生约 1.57 rad 的瞬时容差失败；
这不是小目标需要放宽容差。共享时钟节点现检查每个循环阶段的硬件 ACTIVE 状态及
控制器 update 返回结果，失效即退出控制进程，由原有退出处理终止启动链路，
避免在失效硬件上继续收目标。并非自动恢复，也不允许同时示教器与 PC 控制。
该修改涉及 C++，用户随后已完成本机构建安装，且最新日志验证故障退出分支生效。
编辑器构建和 CTest 仍因“无法配置项目”受阻，新增回归测试尚无通过记录，实体轨迹未验收。

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
  规划速度/加速度额外限制为不超过 0.18 rad/s、0.25 rad/s²，默认缩放均为 0.1。
- [config/ros2_controllers_mk2.yaml](config/ros2_controllers_mk2.yaml)：七轴闭环位置+速度命令接口，
  不接受缺轴目标；单臂输出固定为 `JE_ARM_CONTROLLED_ARM=left`，不向 Robot1 输出。
- [scripts/mk2_state_bridge.py](scripts/mk2_state_bridge.py)：纯 ZMQ SUB 反馈桥接，
  也提供 `--probe-only`，不绑定任何命令端口，依赖系统 Python 的 `pyzmq`/ROS `rclpy`。

**2026-11-06 真实反馈已打通：** 原先二代独立配置的插件列表为空，状态端口
`8000` 未监听。用户随后确认七轴退使能且机械可靠支撑，明确授权备份后修改桥接配置
及受控重启。现已加载新编写的
[只读状态插件](controller_bridge/mk2_state_publisher.c)，真实反馈约 100 Hz。
仅修改二代配置的插件列表；DeviceConfig、BusConfig 与备份逐字节一致，未重新取零，
未加载旧夹爪逻辑，未启动写驱动，未使能或发送运动指令。

**实体后端仍然只有反馈，没有放开命令；隔离 SDK 仿真已部署受保护控制桥。**
实体只读插件的 `commands_enabled: false` 仍会被控制入口拒绝；能力声明本身
不是命令链路或安全证书。仿真命令桥使用相同 SDK 弧度反馈，不改变统一反馈入口。
2026-11-09 仿真验收收到 589 帧原生运动反馈，七轴指定方向与幅值均跟踪并回零；
MoveIt/JTC 成功，约 0.203 秒观察到断流故障，恢复发布后仍锁存拒绝运动。
断流验收结束时 ROS 栈退出，模拟器保留 `watchdog` 锁存停止；恢复须按原受保护流程
重启隔离仿真，再重连示教器、手动使能，网络命令不能自动恢复。
后续用户已重启仿真，并确认切换至域 224 的显式 control/RViz 界面，实际核验执行开关
为 true、JTC active、原生桥 active 且无故障；此次界面切换未重放任何运动目标。
新桥接不自动使能/下使能，不清错或恢复强制停止；Switch=true 只等待首帧，
保持目标验证通过后才接管 SDK 伺服，错误/失联停止后锁存。
当前仿真部署、备份、启动条件、实测结果与回滚见
[controller_bridge/SIMULATION_CONTROL.md](controller_bridge/SIMULATION_CONTROL.md)。
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
收到 1180 帧 SDK 关节反馈，MoveIt FK 服务就绪。当时控制请求在任何节点启动前被拒绝；
该历史行为现已由上文严格的隔离 SDK 仿真控制预检替代，实体控制仍拒绝。
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
