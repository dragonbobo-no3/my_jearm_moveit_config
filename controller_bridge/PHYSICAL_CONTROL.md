# LH_MK2 实体控制准备（2026-11-10）

## 最新 2026-11-13 15:46：首个保持调用 SDK 返回 7，控制退出后只规划

用户在 RViz 点击 Plan & Execute，看到完成/“executed”但实体未动。
只读核对本机本次启动及控制节点日志，顺序已明确：

- 启动目录 `2026-11-13-15-46-54-502014-q670-29559`，实际参数为 `mode=real hardware_mode=control`，
  不是用户从一开始选了 feedback。后端预检进程退出码 0，控制节点 PID 29574 启动。
- 15:46:55.302：本机驱动报告 hardware activated；这不等于原生已接受首次保持。
- 15:46:55.416：原生 Bridge 为 `fault=sdk_error / sdk_error=7 / accepted_joints=0`；
  `servo_observation.last_sdk_call` 为 `ServoJoint`，`result=7`、`command_time=0.01`、
  `phase=waiting_first_joint`，`seen_servo_state=false`。首次保持尚未获成功确认，没有进入轨迹执行。
- 命令验证采样为 `PowerState=1 / MoveState=0 / DriverError=MoveError=0`。
  此采样由核心在首次 ServoStart/ServoJoint 两调用之前保存，**不是 ServoStart 返回后的新采样**。
  因此不能把本例直接写成“验证时 MoveState=7”，也不能据此断言 ServoJoint 入口当时仍是状态 0。
  现有历史 SDK 静态记录显示 ServoJoint 存在按当前状态提前返回的路径；本次返回 7 的具体路径尚未独立核实。
- 15:46:55.420：硬件变为 unconfigured，共享时钟循环拒绝继续使用旧状态并退出；进程退出码 1。
- 15:46:55.534：launch 记录 `CONTROL_UNAVAILABLE`、`FEEDBACK_ONLY_LATCHED`，
  启动新的只规划 MoveGroup（PID 29719）并保留 RViz。后续的 Execute 按钮不会恢复控制权。

MoveGroup 随后明确警告 `not allowed to execute trajectories ... Only a motion plan will be computed`。
该禁执行模式仍可能打印 “Solution was found and executed” 和客户端 “Plan and Execute complete”；
**这些是此次规划请求完成，不是硬件目标被发送/执行的证据**。新实验 action 端点并未接入本次生产启动。

同时出现的 camera_04/camera_05 与 base_link 的 TF 树断开，以及 RViz 更新时间源比较异常，
是另外需要处理的问题；它们不能解释更早已经发生的首个 ServoJoint 失败，也不能靠修复 TF 来恢复控制。
相机变换须依据真实安装/标定关系，不能为消除告警随意发布零位姿变换。

本轮仅读取本机日志及源码，没有连接控制器、启动/重启节点、发送命令、恢复故障或修改执行保护。
后续应核对原生此次 ServoStart/ServoJoint 的状态变化及 SDK 返回路径，不是强制打开
`allow_trajectory_execution`、忽略返回码或自动重试运动。以下在线正常反馈记录均为更早的历史状态。

### 同日后续：只读核对原生 SDK，返回 7 的含义已确认，STOP 来源仍未确定

通过新建的临时 SSH 复用连接只读查询控制器，并经用户在终端输入密码读取 root 保护的原生日志。
未运行 SDK 函数、调试附加、状态恢复、使能或运动；没有修改远端文件、权限及配置。
原生 RobotMain PID 2792、EtherCAT 主站 PID 2436 在查询前后保持同一启动实例。

当前磁盘 SDK 与历史审计指纹相同：
- libRobotControl：`40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1`。
- libHYYRobotTechnology：`cb854d244ca3835cce6e992792d229f414bb821edd7f01a4342b21826c451227`。
- 实体诊断插件仍为 `c834b04898a5f4578e09d85386a25f8e2cb5a9b82e5dc75bb171ff7c6bbd9afb`；
  GlobalConfig 与实体策略指纹也与部署记录一致。

**已核实的返回路径：**
- ServoJoint 在 `0x1230cc` 调用 get_robot_state，`0x1230d1` 比较 13。
  非 13 时保留该值直接返回（`0x1230d6–0x1230e4`）；正常写入路径返回 0，设备缺失路径返回 -103。
  因而针对这份 SDK，本次 result=7 是入口读取到状态 7 的拒绝，不是“第七轴”、轨迹参数错误号或 MoveIt 错误码。
  启动前命令验证样本为 0 与此不冲突：它们属于不同时间点。
- ServoStart 创建 `_servo` 线程后调用 WaitThreadStart，随后返回 0。
  `_servo` 先调用 ThreadFunctionStart（`0x122d8e`），再调用 servo_execution（`0x122d95`）；
  WaitThreadStart 只检查线程函数状态不再为 -1，不确认机器人状态仍为 13，也不提供故障来源/持续存活证明。
  所以不能据 ServoStart 返回 0 认定后续 ServoJoint 必然可用，单纯添加延时同样没有这种保证。
- servo_execution 的启动入口要求状态 0，随后置 13；循环读取 order，order=1 时从
  `0x12209a` 转到 `0x122c95`，通过 `0x122c85` 写回状态 7 并退出。
  这是可导致“启动前健康、首帧已经状态 7”的具体路径，但本轮没有动态观察到其被执行。
- RobotStop 与原生 robot_teach_stop 最终均能写入同一个 order=1；setter 不同时改写 MoveState。
  因此 MoveState=0/电源及错误字段正常本身不能排除待消费的 STOP 订单。
  get_robot_order1 读取后会调用 set_robot_order，**不是只读探针，本轮没有调用它**。

**原生日志与尚缺证据：**
- 当前 v2_jHNJsB7W/controller.log 与 PC 日志一致：ready → waiting_first_joint → sdk_error=7，
  同一单调时刻 5658.201132555、同一首次 ServoJoint 输入，无额外的 STOP 来源记录。
- SDK 日志 20261113010938log.txt 在控制器本地 02:41:37–02:41:45 记录了 goto/teach_stop，
  02:49 以后又有示教动作。控制器时区为 -05:00，PC 日志时区不同；不能把未统一的钟表时间直接当并发证据。
  没有据此认定用户在此次 ROS 接管瞬间点动，也没有证明具体哪一条 STOP 留存到首帧。
- 本机驱动激活源码只在就绪及 Switch 确认后开始保持；其故障清理 Switch=false 出现在失败之后。
  本轮未发现它正常激活路径先主动发送 Switch=false 的代码依据；这不排除其他进程或历史订单。

**当前结论：首帧失败已追到 SDK 入口状态 7；谁触发状态变化仍缺因果证据。**
下一步应在与实体完全隔离的 SDK 仿真中对照“无待处理 STOP / 待处理 STOP”的首次启动行为，
并验证合法的接管准备合同；不能在实体上消耗 STOP 订单来试探，也不能把来源未知的停止自动清掉。
本轮没有修复或重新验收实体控制，更没有据此放开自动接管。

## 最新 2026-11-12：首次故障诊断已安装，退使能在线反馈验证通过

现场随后按原流程启动控制器。只读采样 100 帧、序号 8523→8622，约 1.009 秒，
七轴反馈全部有限且序号严格递增；始终 `PowerState=0`、`MoveState=0`、两类 Error=0。
桥为 `ready/fault=none/sdk_error=0`，`commands_enabled=false`，接收/拒绝命令计数均为 0。
新 `servo_observation` 字段已出现，`seen_servo_state=false`、`ownership_evidence=false`；
没有故障时不出现 `first_state_error` 是预期行为，不代表诊断缺失。
原生 PID 本次为 75269，实体 profile、新库及 GlobalConfig 指纹仍匹配；PC 8001 无监听、
无 ROS 控制进程。以上证明诊断反馈已在线，不证明故障触发分支、伺服接管或交接已实体验收。
本次助手仅订阅状态和只读检查，没有启动、使能或发送保持/运动。

现场明确确认机械臂停止、可靠支撑、退使能，ROS 控制栈、RobotMain 和 EtherCAT 主站
已停止，并授权仅安装诊断插件。独立检查进程、端口及原两把锁后，安装事务成功完成。
安装后新 SSH 会话再次核对指纹、权限及停机状态：全部通过。没有启动控制器、使能、清错、
恢复停止、发送保持或运动。**这不是自动交接修复，也不表示 ROS 执行能力已恢复。**

- 新插件目录：`control_guarded_physical_diag_20261112_hsVAKfCn`（实体 profile 下）。
- 新库 SHA-256：`c834b04898a5f4578e09d85386a25f8e2cb5a9b82e5dc75bb171ff7c6bbd9afb`。
- 新 GlobalConfig SHA-256：`6dbf8e7dca6d80b9b0e738332638d6131ce7df87caab85ab9bfa0d1cbc78bd74`。
  只替换 `plugin_list` 的单个库路径；反向替换与原配置逐字节比较通过。
- 备份：`/home/robot/Work/system/robot_config_new_left/backup_before_diagnostics.v4CgQp3k`，
  root:root / 0700；含原配置、原库、策略、经指纹验证的源码/产物、安装脚本及前后清单。
- 新库 root:root / 0755，GlobalConfig root:root / 0644；旧库保留原位、指纹不变。
- 策略、位置/速度/加速度限值、DeviceConfig、BusConfig、仿真配置、原启动器、ENI 和 SDK 均未改变。
- 保持 `MK2_PHYSICAL_HANDOVER=0`，不导入 StopRun、Recover、上下电、清错或点动接口。

诊断内容是停止处理前保存的 `Bridge.first_state_error`，以及 `Bridge.servo_observation`。
前者记录首次失败检查和当时 SDK 输入；后者记录同一会话观察到的首次非伺服状态及最近
Servo 调用输入/返回值。只增加观察，不改变故障判定，不把状态变化或 SDK 返回 0 当成交接证据。
`Robot0` 仍发布当前反馈，不用故障快照替换实时关节。

[独立诊断回归](tests/run_mk2_diagnostics_offline.sh) 的 12 项快照、11 项伺服观察及实际
JSON 发布测试均通过普通和 ASan/UBSan 检查，最大测试反馈 2642/4096 字节。
[仅编译脚本](build_physical_diagnostics.sh) 已使用控制器原 SDK 严格编译链接；
[固定指纹安装事务](install_physical_diagnostics_20261112.sh) 已实际执行成功。
库编译与安装通过不等于在线加载、实机运动或自动交接验收通过。

退使能启动与只读字段核验已完成。不要为了采样故意同时进行示教器点动和 ROS 伺服控制。
之后的控制接管和运动测试须另行确认条件，不能靠重启或忽略 `state_error` 自动恢复。

回滚须再次确认安全停机并持原 startup/session 两把锁，只将备份的原 GlobalConfig
原子恢复（原 SHA `171ec5f235cf8ee66a1cea1da331abe5f133f8f816cc5507dd4d1fef2003f815`），
验证其指向仍保留的旧库和全部未变清单；策略无需删除或改写，不自动启动进程。
下方各节是历史记录，涉及“诊断未部署”和旧 GlobalConfig 指纹的描述以本节为准。

## 最新 2026-11-11 11:15：故障保护已生效；并发点动场景及退出段错误

用户确认这次本机 control 接管后，做过示教器点动。不能用此操作回答进一步推断
具体哪个 SDK 状态量异常；但本机保持和示教器运动不应并发，应改为
`mode:=real hardware_mode:=feedback` 观察示教器运动，不创建硬件写驱动。
11:15:40.372 原生 state_error，40.375 本机识别硬件 unconfigured 并退出控制，
launch 随后发 SIGINT；MoveIt/RViz 的段错误发生于这之后，是独立的析构/退出问题。
现场栈指向 MoveIt 2.5.9 回调组析构及动态库对象卸载；不能据此断定具体补丁版本的修复。
保留保护退出，不忽略原生故障、不自动恢复或重启，不强杀进程掩盖退出崩溃。

安装路径实际符号链接到构建目录。跟随链接核对 ELF 为 2026-11-11 11:06:48，
存在 `refusing trajectories on stale hardware state`，最新运行日志也证明已启用。
之前仅 stat 安装链接得到的 Nov9 时间不是实际二进制时间，不能再据此要求重复构建。
只读 SUB 99 帧确认当前实体桥仍 state_error/commands_enabled=false；SDK 当前
Power=1、Move=0、DriverError=MoveError=0，不能反推故障首次触发瞬态。
没有使能/清错/恢复/重启控制器，也未启动本机控制或发送运动。

已安装 MoveGroup 2.5.9 Feb19、rclcpp 16.0.15；本地 apt 元数据有更新候选，但未升级
系统 ROS 依赖，也未声称更新一定解决段错误。本次仅补充启动和故障退出提示，
没有修复外部库段错误；这类日志提示变更不需重新编译或部署控制器插件。

## 2026-11-11：小目标瞬间中止实际是故障后使用旧控制状态

已核对 11 月 10 日 17:24:21 那次完整控制启动日志，不是根据单条容差错误猜测：

- 17:24:36.367 原生桥 state_error，驱动状态接收线程退出。
- 独立反馈日志中的 joint2 随后从约 -1.570 变到 0.000，显示跟随修复已起效。
- JTC 在故障约 33 秒后仍接收第一个目标，约 72 秒后接收用户贴出的目标；
  四次目标均立即出现约 1.57 rad 的路径容差失败。
- 已安装 JTC 容差代码打印的是从零开始的 joint_idx；本配置下 index 1 是 joint2，
  不是 joint1。原生/URDF/驱动的七轴映射没有发现错序证据。
- RViz 不同时间源异常发生在执行中止后的更新起点回调，不是首次 state_error
  或这次路径容差失败的起点；本次没有盲目替换时间源或修改 MoveIt 库。

前次反馈解耦只修复显示，没有解决失效硬件下仍留存 JTC action 的问题。
本次共享时钟节点派生自 Humble ControllerManager，封装其 protected 资源管理器，
在 read 前后、update 后、write 后验证 lh_mk2_system 必须 ACTIVE，
并检查 update 的错误返回。失效抛出异常，停止控制循环并关闭进程，现有 launch
退出处理随后终止启动链路。驱动已有超时/原生故障锁存继续有效，未放宽容差，
未清错、恢复停止、上电或重发轨迹。最初 state_error 的瞬态原因仍未确定。

新增回归测试覆盖缺失/非 ACTIVE 硬件、read/update/write 阶段失效、update 返回错误，
以及 8000 个周期的相同采样时间/周期传递。编辑器构建和 CTest 均报“无法配置项目”，
无 CMake 诊断，因此这些 C++ 测试尚未运行，不能视为构建或实体验收通过。
日志回放核验通过；源码 API 依据已安装 Humble 头文件复核。
该次调查当时尚未验证新二进制；随后 11:15 日志及跟随链接检查已证明新保护生效，见顶部。
本次核查时本机控制启动已由用户退出，没有启动控制器/本机控制节点或试动实体。

## 17:09 故障后反馈冻结调查与本机修正

当前用户启动的是 `mode:=real`、默认 `hardware_mode:=control`，不是只读示教跟随。
17:09:13 本机日志记录原生桥 `fault=state_error`、commands_enabled=false；随后
驱动状态接收线程退出，8000/8001 连接关闭，控制器管理器和 RViz 仍运行。
只读四秒采样收到原生状态 403 帧、序号 110146→110548，关节值有变化；同一 ROS 域
224 的 `/joint_states` 收到 401 帧，时间戳递增但位置数组完全不变，发布者为广播器。
这证明本次显示冻结在 ROS 控制驱动/广播器链路，不是原生状态 PUB 断流。
当前原生 PowerState=1、MoveState=0、DriverError=MoveError=0 不能解释最初瞬态，
不能据此认定是示教器操作触发；现有日志不足以确定 state_error 的首发条件。

本机启动配置现为：预检成功后另起独立纯 SUB 接收器发布全局 `/joint_states`；
控制器广播器使用私有话题，保留诊断但不再作为全局实测反馈源。
原生故障只撤销控制能力，不阻断有效关节显示；状态断流仍超时终止本机链路。
不修改故障锁存、限幅、配置指纹或自动恢复策略，不改控制器插件。
只做示教器运动观察时应使用 `mode:=real hardware_mode:=feedback`，不接管伺服。
当前正在运行的进程没有重启或修改；配置改动下次启动生效。

验证：实际 sim/real × feedback/control 启动配置及预检成功/失败回调检查通过，
广播器支持只读参数 use_local_topics，100 Hz、JTC 容差及接口不变。
独立 ROS 域 225 中短时运行实际纯 SUB 桥，在原生 state_error 始终锁存、
commands_enabled=false 的情况下收到 394 帧原生状态、389 帧 ROS 关节反馈；
关节值与原生最新帧对应，唯一状态发布者为该桥。测试接收器已正常退出，
未影响用户域 224、控制器或故障状态。完整控制栈的新配置尚未在线重启验收，
没有执行示教器点动/实体运动试验，不据微小关节变化认定运动验收通过。

## 最新状态：实体插件和本机驱动已更新；入口统一 mode:=sim / mode:=real

按用户要求，后端选择与实体确认合并为单一启动参数：`mode:=sim`（默认）或
`mode:=real`。显式选择 real 即确认请求实体伺服接管，不再公开
`control_backend`、`confirm_physical_control` 启动参数。驱动内部仍收到精确后端、
配置 SHA 和实体确认，底层预检/故障/限幅/锁存保护没有取消。
总入口保持原 control/confirm_enable=true 默认值；只读用 hardware_mode=feedback。
sim 指隔离 SDK 仿真，不是离线 CAD 假执行器；real 与 use_fake_executor=true 冲突会拒绝。

只读检查确认本机驱动已于 16:56:51 更新，包含新增实体确认校验标记。用户提供的
16:57 启动日志显示实体桥 physical/ready/none、PowerState=1、MoveState=0；
当时默认 sim 预检因后端不匹配拒绝，未接管伺服。当前不再要求重复构建驱动。
本次只修改启动 Python 和说明，已安装 launch 均符号链接到源码，无需重装控制器插件。
已在内存构造实际 sim/real × feedback/control 启动动作，核对预检类型和驱动参数；
没有执行 LaunchService、启动节点、绑定命令端口或发送机器人命令。

## 安装完成时记录：实体插件已安装，当时本机驱动待构建、控制器停止

sudo 安装事务已成功完成。持原 startup/session 两把共享锁，核对停机、端口、
配置与构建指纹后，已安装实体插件和受限命令策略，并原子替换实体 GlobalConfig
中的插件路径。库为 root:root / 0755，GlobalConfig 和策略为 root:root / 0644。

原 GlobalConfig、StateOnly 库、源码快照及安装前后指纹备份在：
`/home/robot/Work/system/robot_config_new_left/backup_before_physical_wJhPzqqI`。
原 StateOnly 库仍留在原路径，没有覆盖或删除；回滚需停机持原两把锁，恢复备份的
GlobalConfig 并撤去本次新增策略，重新校验原件，不自动启动进程。

安装后独立核查确认新库、GlobalConfig、策略与下方指纹一致；实体 DeviceConfig、
BusConfig，仿真三份配置、原启动器、ENI 和 SDK 均未改变。RobotMain、主站及
相关端口仍为空闲，没有使能、清错、恢复停止、上下电、保持或运动命令。

**尚不能直接执行实体控制**：本机已安装驱动仍为 2026-11-09 16:59:27 版，
没有新增实体确认校验标记，必须先构建安装配套 `je_software` 和配置包。
本轮编辑器完整构建仍报“无法配置项目”，且无 CMake 诊断。新增插件尚未加载，
真实 SDK 接管/停止、实体运动及退出重连仍未在线验收。

### 安装准备与实际构建指纹（历史过程）

用户明确反馈“已退使能，robomain也停了”。本轮独立核查 RobotMain、EtherCAT
主站均不存在，8000/6666/6667/6668 监听及本机 8001 命令端口均空闲；未停止进程。
实体与仿真六份配置指纹仍为部署前基线。新插件已在实际 SDK 下严格编译链接，
固定实体 profile，只导入 RobotStop 和 ServoStart/ServoJoint/ServoEnd。

- 暂存目录：`/home/robot/mk2-physical-build.m5N0JeOa`。
- 新实体插件 SHA-256：
  `1aae19d554714aedc9faa59ff2ed0a94569b1aae268acc54d031f1db9cf6e65c`。
- 已安装目录：`robot_config_new_left/control_guarded_physical_20261110_m5N0JeOa`。
- 新 GlobalConfig 仅更换插件路径，逐字节比较通过，SHA-256：
  `171ec5f235cf8ee66a1cea1da331abe5f133f8f816cc5507dd4d1fef2003f815`。
- 新策略与已审查实体示例一致，仅明确设置 `allow_commands=true`，不改限值；
  SHA-256：`7c343019190b31169eb9d2a81539c828ab2dee22bce88d199abf627758b12323`。
- 安装脚本持原两把共享锁，要求停机、空闲端口及全部配置/启动器/SDK/ENI 指纹
  匹配；备份原 GlobalConfig 和 StateOnly 库，先装策略、最后原子替换加载项，
  失败恢复配置并撤去新策略和本次插件。原只读库保持原位，所有标定/总线/仿真
  配置不改。此段是安装准备，不等同安装成功。
- 本机完整 ROS 构建入口本轮仍报“无法配置项目”，无 CMake 诊断；已安装驱动
  尚无新增实体校验标记。控制器插件安装后也必须先补齐本机配套构建，不能直接执行。

**此部署不启动原生控制器或主站，不使能、清错、恢复停止或发送保持/运动。**

## 源码阶段记录（安装前；非当前部署状态）

用户明确要求控制实体臂。只读核查发现当前原生进程使用
`robot_config_new_left`，运行真实 EtherCAT 主站，但只加载 StateOnly 插件；
反馈为 `feedback_only`、`commands_enabled=false`、`PowerState=1`。
因此原控制启动被正确拒绝，不应绕过预检。此次不停止、重启、使能或移动实体臂。

### 显式选择，不自动切换（当前单一 mode 接口）

- 现有 `hardware_mode`、`confirm_enable` 默认值不变；后端用 `mode:=sim`（默认）
  或 `mode:=real`。real 的显式选择即为实体接管确认，内部实体确认校验仍保留。
- 总入口默认 control/confirm_enable=true，所以实体只需选择 `mode:=real`。
  **mode 不是使能或部署命令；旧 StateOnly 插件及错误后端仍无法通过预检。**
- 纯订阅预检要求明确 physical/control_guarded、非仿真、精确 profile/DeviceConfig
  指纹、配置校验标记、SDK 无错误、已人工使能且空闲、ready 状态以及递增反馈序号。
  驱动在激活前及运行中重复校验预期后端，沿用反馈超时与序号倒退锁存。
- 控制启动会申请伺服并持续发送当前位置保持；不是只读操作。没有自动规划目标，
  也没有自动上电、下电、清错、停止恢复、归零或夹爪操作。

### 原生插件的独立实体构建

- 实体构建必须显式定义 `MK2_PHYSICAL_CONTROL=1`、`MK2_SIMULATION_ONLY=0`，
  固定 `PROFILE_ROOT=/home/robot/Work/system/robot_config_new_left`。
  两种后端宏不能同时启用；均未启用时拒绝插件启动，不默认开放实体写入。
- 保留原 `RobotMain_v2` 受保护实体启动器的七轴身份/禁用状态、ENI、WKC、
  共享内存和 SDK 指纹检查。插件额外要求继承原 fd 9 会话锁、真实 EtherCAT
  字符设备、`simulation: false` 以及已确认的 DeviceConfig/BusConfig SHA-256。
  实体后端每轮重查这些条件；失配停止自己拥有的伺服，不尝试自动恢复。
- DeviceConfig 指纹固定为
  `a542dc92c51e2822e78dcfc385a90f925a909ebd70072fab0b5ab75d4e0c2bab`，
  BusConfig 指纹固定为
  `425db50795d4e2e9fc44e4005e7dcf797e677c7d4fa87a135236a449a5f6ab84`。
  ROS 侧使用已确认的硬件快照，不再次处理编码器方向或零偏。
- 原示例策略默认 `allow_commands=false`；实体部署必须单独审核策略与命令源。
  复用首帧实际位置保持、时间/速度/加速度/位置/跟随误差/看门狗检查，不提高上限。
  这些是调试软件限值，不是已认证的实体负载、安全速度或急停系统。
- 发布 SDK DriverError/MoveError、profile 和已校验指纹；不伪造仿真隔离标记。
  当前 ZMQ 协议没有命令源认证/会话令牌，必须隔离控制网并确保唯一上位机命令源，
  禁止示教器并发点动、其他伺服程序或第二套控制栈。
- **实体不提供 clean Release**，不导入 RobotStopRun/上下电接口。实体构建仅在
  首帧保持、`ServoStart` 之前，调用**一次** `RobotStopRecover(0)` 以清除强停
  锁存（state 7），对标 gen-1 激活流程，但不做上下电、不清错、不复位驱动/运动
  故障——真实驱动/运动/越界故障仍在首帧前被拒绝。进入 ACTIVE 后再出现的强停仍
  会故障锁存、不自动恢复。正常退出仍使用现有 RobotStop + ServoEnd 锁存停止；再次
  启动可能需要操作员按厂商支持流程处理。仿真的退出重连结论不能直接推广到实体。

## 部署前必须满足

1. 操作员先停止运动，使用示教器退使能并可靠支撑机械臂；确认急停可用、工作区清空。
   此后按原流程停止原生控制器和主站。当前 PowerState=1 时不在线替换/重启。
2. 重新核查进程、端口、锁及配置基线；持原 startup/session 两把锁备份实体
   GlobalConfig 和只读插件。新控制插件必须**替换** StateOnly 加载项，不能同时占用
   8000，也不能启用自动归零或传统自动上电服务器。DeviceConfig、BusConfig、ENI、
   SDK、模型坐标及启动保护不得改写。保留完整可回滚记录。
3. 构建并安装配套 ROS 驱动和配置包；只更新 launch 符号链接不足以更新驱动二进制。
   按原受保护流程启动后先确认禁用状态及真实反馈；操作员再手动使能，不自动接管。
4. 先做只读预检，再单独授权当前位置保持接管；之后才进行低速单轴小幅运动、取消、
   跟随误差、断流、退出锁存与重新连接的分项验收。禁止自动发送测试轨迹。

## 本轮验证

- Python 语法、实体/仿真正反例、单独实体确认、两套生成模型轴向/原点保持检查通过。
- 预检主循环用内存 socket 运行 8 项：两帧递增、单帧/重复帧超时、序号倒退、
  重复后递增、错误指纹、错误后端和 SDK 错误；全部通过，无网络命令。
- 实际 C++ 反馈校验辅助代码独立编译运行通过；实体错误/缺失指纹/仿真串入/
  不支持的实体 Release 均被拒绝。
- 实际 C 核心使用内存 SDK 回调，验证默认锁定、未使能/错误/忙碌拒绝、首帧保持、
  无 Release 回调、看门狗仅停止一次及停止/故障不可自动清除，全部通过。
- 控制器实际 SDK 下实体和仿真插件均通过严格 C11 编译/链接及未定义符号检查；
  实体只导入 RobotStop/ServoStart/ServoJoint/ServoEnd，无上下电、清错或 Recover。
  仅编译，不加载插件；临时编译产物自动删除。
- ROS 工程完整构建仍被编辑器构建入口“无法配置项目”阻塞，未产生 CMake 诊断；
  未宣称驱动二进制已更新。部署、实体保持接管及实际运动均未执行。

上述软件检查不是功能安全认证，也不证明实体停车距离、负载适配或完整 TCP/坐标验收。