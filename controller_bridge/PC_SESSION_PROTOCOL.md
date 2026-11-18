# JEServer_v2 实验 PC 会话协议（2026-11-13）

## 范围

已接通控制器侧通信入口，而不是只调用引擎内的 `je2_claim_pc`。
仅 `JE2_SIM_SESSION_EXPERIMENT=1` 的显式私有仿真插件使用此协议；默认值为 0。
开发入口 `RobotMain_v2_pro --test-session-sandbox` 构建实际插件并在五私有 namespace 内
运行真实 ZMQ / 工作线程与**替身 SDK**，不连接控制器、不安装插件、不加载真实 SDK。

**ROS 驱动尚未接线，生产自动接管未完成。** 原生停止来源合同、完整示教协议/旁路阻断、
可重复 Servo writer 生命周期都未因此验收。现有实验 writer 仍每次插件加载只允许一次 Start；
第二次 claim/Switch 成功不是第二次 Servo 或执行轨迹成功。

**最新（15:37）：测试专用真实 ROS action 端点已接通接单、位置反馈、取消及结果发布，
见下述“真实 action 接口隔离联测”。其替身控制器不连接实体网络，不等于生产驱动接线。**

## 启动身份与兼容性

- 每次工作线程启动使用 `getrandom` 取得 128 位启动标识，编码为 32 位小写十六进制。
  熵读取失败就拒绝启动，不用时钟、PID、环境变量或固定默认值替代。
- 会话实验从第一条消息起就拒绝旧的无封套 Switch/Joint/Release，不提供失败后回退。
  否则旧消息可在重启后利用重新从 1 计数的代次进入新进程。
- 默认插件仍使用原有初始 legacy 会话；任何交接后封闭 legacy 的既有行为不变。
  两种模式不能在同一运行实例中自动切换。
- 这些标识是公开的消息关联值，**不是密码、认证令牌或停止来源证明**。同一私有网络中的
  恶意进程仍可能伪造；不得把本协议直接暴露到实体网络而宣称获得授权/身份验证。

## 单帧格式

命令通过既有私有 ZMQ 命令入口发送，每条消息必须是一个完整单帧，总长不超过 4096 字节。
元数据使用严格定长/定界 ASCII，不使用 JSON 字段合并或浮点数解析：

- 申请：`JE2C1 <boot> <observed_generation> <client>`
- 命令：`JE2F1 <boot> <claimed_generation> <client> <sequence> <core_frame>`

boot/client 均为恰好 32 位小写十六进制；客户端为每次新申请生成新的 client 标识。
代次和序号为 1 到 UINT64_MAX 的规范十进制，不接受 0、前导零、正负号、小数、指数或溢出。
各元数据只由一个 ASCII 空格分隔，申请末尾不接受空格/换行；禁止嵌入 NUL。
core_frame 是已有严格 Switch、Joint 或 Release 帧，原限位、时钟、首保持点与错误保护继续适用。

## 申请及命令确认

1. 读取同一条新 Owner 消息中的 boot、当前 generation、owner 和实验能力信息。
2. 显式发送申请，携带观察到的 generation 与新的 client 标识。
   已存在故障、状态 7、旧 writer 未退出、仍在点动或静止时间不足时不会获权。
   底层仍要求新鲜静止反馈至少 100ms；单次失败不代表可以盲目恢复或重试动作。
3. 等待新的 Owner 同时满足 boot 相同、client 相同、generation 等于申请代次加 1、
   session_generation 等于当前 generation、owner 为 AVAILABLE。代次/标识必须一起核对。
   PUB 消息可能丢失或迟到，发送成功本身不是 ACK；没有 ACK 就不能发送运动目标。
4. 命令 sequence 从 1 开始严格连续。先 Switch，再依据最新反馈重新生成首保持点。
   不拷贝旧会话目标、不延续旧轨迹缓存；claim 和 Switch 都不调用 ServoStart。

Owner 新增字段：

| 字段 | 含义 |
|---|---|
| session_transport_enabled | 当前实例是否启用实验封套；不是生产可用标志 |
| session_boot | 本次插件加载的启动标识 |
| session_client | 最近一次成功申请的客户端标识；未申请时为空 |
| session_generation | 该次成功申请的代次；初始为 0 |
| session_last_sequence | 该会话已送入引擎的最后命令序号；不是执行成功确认 |
| session_release_sequence | 该会话从 ACTIVE 成功接收规范 Release 的序号；0 为无请求，失败/仅消费不产生回执 |
| session_release_phase | 0 无有效进度、1 已接收且仍在释放/退出屏障、2 该次引擎退出和静止屏障已完成；不是生产能力或目标成功 |
| snapshot_version | 会话完整快照格式，目前为 1 |
| snapshot_sequence | 本次发布序号，同次 State 的发布序号；客户端不依赖拼接 State |
| snapshot_phase / snapshot_fault / snapshot_sdk_error | 同次发布的 core 阶段、锁存故障和 SDK 错误 |
| snapshot_feedback | valid、PowerState、MoveState、DriverError、MoveError、七轴 Joint；非有限关节为 null |

快照扩展只在会话实验启用，默认 Owner 路径不增加这些快照字段。
身份与反馈来自同次执行器发布，但 SDK 采样本身是顺序读取，**不是厂商原子安全快照或停止来源证明**。

当前 generation 可能已因点动/释放再次变化；历史 session_client/session_generation 不代表仍持有控制。
消费序号即使对应 core 拒绝的命令也会前移。客户端还必须核对 owner、控制阶段、fault 及关节接收结果，
不能把 session_last_sequence 当作运动完成 ACK。代次必须保留完整 uint64 精度，不能经浮点数转换。

`automatic_priority_ready`、`writer_exit_verified`、`pc_reclaim_supported` 仍全部为 false。
会话实验不通过原 State 的 `commands_enabled` 向旧驱动宣告可执行；新旧客户端不能混用。

## 隔离和失败处理

- boot、代次、client 不匹配，无会话、重复/倒序消息及不完整元数据，在入口读取 SDK 前丢弃。
  不让旧会话的坏帧、停止或释放调用新所有者的清理。周期反馈/tick 仍独立运行。
- 当前有效会话出现序号缺口立即锁存 BAD_FRAME，不跨缺口继续播放；故障不自动清除。
- multipart 每轮仅丢弃一个部分，最后一段（MORE=false）清除丢弃状态。
  尾段即使看起来是有效新命令也不会执行，后面的完整单帧可以正常处理。
  不用无界阻塞 drain，不把旧 multipart 当作全插件卸载理由；不能据此宣称抗恶意洪泛。
- 状态 7 或任何已锁存故障仍拒绝 claim；下一帧正常不能重建会话或清错。
- 没有自动 claim、目标重放、透明重连或掉线后自动恢复；活跃会话仍受原 watchdog 约束。

## 独立客户端组件（尚未接入生产 ROS 驱动）

[接口](je2_session_client.h)及[实现](je2_session_client.c)不含套接字、SDK 调用或轨迹队列。
实现显式申请、绑定、Switch、首次保持及其确认，以及有界的显式后续目标封装。
后续新增的显式收尾只在带合法释放回执字段的会话端点使用，见下述收尾合同；旧端点不盲发 Release。
ACTIVE 表示该协议阶段得到确认，不是轨迹完成、生产获权认证或可重复 Servo 生命周期验收。
C 组件本身不生成轨迹、不处理 ROS action 或自动再接管；后续实验 action 包装见下节，服务端原 watchdog 仍有效。

- `init` 仅生成 client 标识；boot 必须来自服务端。退役/故障不会自行重置或重新绑定。
- 一次显式 claim 操作可以由调用者轮询，间隔至少 20ms，重发完全相同的申请以满足
  服务端 100ms 静止确认及 PUB 建链；500ms 总期限固定，不因重发延长。
  这不是运动重试或失败后自动恢复。Switch 和首次保持不能重复输出。
- claim 确认必须匹配 boot/client、新旧代次、AVAILABLE/READY 和初始消费序号 0。
  Switch 确认还要求 PC/WAIT_FIRST、序号 1、无错误；首次保持只取该确认之后的关节反馈，
  不接收调用者旧目标。首次保持确认要求 PC/ACTIVE、序号 2、无错误；不能只看消费序号。
- 数字字段既检查原始规范拼写/uint64 溢出，又检查层级和 JSON 类型；已知字段重复、转义、
  非有限关节、错误/状态 7 等均拒绝。失权/启动变化为 RETIRED，故障为 FAULT，均不再输出。
- 新观察接收间隔上限 150ms；重复/倒序快照不更新接收时间或覆盖保持位姿。
  时钟逆行、超时、发送失败均锁存。接收时间是本地时钟，不能证明任意网络排队消息的绝对采样年龄，
  也不提供认证或抗恶意重放保证；服务端自身采样保护和代次隔离仍必须保留。
- 将来 ROS 适配器必须串行化观察、tick、输出及发送结果，清空失效目标队列并保留 RViz。
  未更改驱动、旧 `commands_enabled` 校验或安装库。编辑器构建和下述 ROS 消息单元测试现已通过，
  但这仍不能替代实际 ROS 执行链路的集成验收。

## 有界连续目标接口

### ROS 消息边界（已编译并通过单元测试，不是运行链路）

[RosGoalAdapter](../src/je2_ros_goal_adapter.hpp)直接使用已安装 Humble 的
`control_msgs::action::FollowJointTrajectory::Goal`，由
[单元测试](../test/je2_ros_goal_adapter_test.cpp)及[进程内会话联测](../test/je2_ros_session_loopback_test.cpp)引用。
在本包 `BUILD_TESTING` 下，CMake 将独立 C 客户端链接到 C++/ROS 消息测试；没有 install、
插件注册或 launch 切换。适配器自身无套接字或 action server；本节联测单独使用进程内 ZMQ，
后续真实 ROS 节点/action 包装测试见“真实 action 接口隔离联测”。

- `bind` 仅在首次保持已确认、尚未发送后续目标时接收一个目标；保存最多 512 点的不可变副本，
  并在此时捕获完整会话 lease。回调保留同一 Ticket，不允许在回调执行时重新绑定当前身份。
- 只接收明确按期采样的七轴位置/速度点、严格一致的关节顺序和零 header 时间戳。
  非空 effort、多自由度目标、容差等未实现的字段拒绝；不声称支持通用 FollowJointTrajectory 语义。
  点时间相对目标接收时刻，平移一次到会话累计时间；不插值、不修改采样间隔、不追赶迟到点。
- 构造适配器必须显式提供七轴 `Limits`（位置下/上界、速度及加速度上界），由调用方配置而非 goal 提供。
  有限且严格有序的位置界限、正的有限速度/加速度上界缺一不可；无无限制回退。
  不自动读取/修改实体限位；用例中的 ±1、0.2、0.4 仅为测试构造值。
- `bind` 预检整个不可变目标，包括首点进入过程：所有点的位置、声明的速度/可选加速度，
  以及 `v=(q-prev_q)/dt`、`a=(v-prev_v)/dt` 必须合规。首段从首次成功封装的保持点
  `first_target`、其累计时间和零命令速度开始；该点不会被后续关节反馈覆盖。
  检查的是命令离散差分，不是实体瞬时速度/加速度、碰撞、跟随误差或动力学证明。
- `prepare` 只允许顺序提交当前票据的点，复核命令历史并再次检查该步；背压不消费索引或历史，
  只有正返回（成功生成封套）才推进。旁路发送造成序号/累计时间与历史不一致时锁存本地 FAULT，
  不发停止/恢复帧。不得绕过串行调用约定或直接改写客户端内部字段。
  不自动播放整条队列；未来执行器仍须驱动下述单步调度、接入轨迹/反馈保护及完成/取消处理，
  本边界尚不能作为运动执行器使用。最后一帧格式化也不等于 ROS goal 成功。
- 失权、重启或故障清除当前绑定。当前 Ticket 的取消会退役本地会话；旧 Ticket 的 prepare、
  cancel、send_failed 都不能影响新会话。取消仅表示本地禁止输出，不是物理停止或 action 取消完成证明。
- 新测试源码覆盖绑定前拒绝、不可变目标、顺序/背压、旧回调/取消/发送失败、新会话隔离、
  超时及 13 类不支持/无效消息，另增七轴违规/非法限位矩阵、保持基准、背压发送历史、
  旁路写入、迟到点及入段校验。**12 个 gtest 均已由实际 CTest 执行通过**；
  其中客户端初始确认状态是测试构造值，不是 ROS 节点或真实 SDK 握手验收。
- 用户手动构建日志 `build_2026-11-13_10-48-11` 只选择 `je_software`，执行器返回 0，
  并含若干 Python bytecode 安装 FileExistsError 输出；不等于新客户端或此适配器编译通过。
  该轮未覆盖当前包的新增目标；后续编译结果见下段。
  不要把独立 C 用例已通过当作 C++/ROS 适配器已通过。

后续编译核验：最新 `build_2026-11-13_11-07-45` 同时选择了当前配置包和 `je_software`。
日志明确记录 `je2_session_client_test_support` 的 C 编译/静态库链接及
`je2_ros_goal_adapter_test` 的 C++ 编译/可执行文件链接成功；构建目录中存在新二进制和对应 CTest 注册。
当时编辑器测试发现为空、执行入口无法配置；用户构建只证明编译通过。后续重载和环境修复后的
实际执行结果如下，不再处于“待测试”状态。

#### 2026-11-13 13:38 本地执行证据与编辑器修复

- 在 GCC 11.4 / Unix Makefiles / legacy 通信模式及独立目录
  `/home/test/ros2_ws/build/my_jearm_moveit_config_vscode` 下，编辑器构建返回 0。
  没有覆盖 colcon 构建目录，没有执行 install。
- 首次真正配置时报 `No 'rosidl_typesupport_c' found`：对应 ament 索引文件实际存在，
  但编辑器没有 `AMENT_PREFIX_PATH`。补齐索引路径后构建成功。
- 最初 CTest 启动器报 `ModuleNotFoundError: ament_cmake_test`，尚未进入 gtest。
  将 ROS 环境放入 `cmake.environment`，而非仅 `cmake.configureEnvironment`，
  使配置、构建和测试共享所需路径后，两个测试目标均返回 0。
- `je2_ros_goal_adapter_test.gtest.xml`：6 tests、0 failures、0 errors、0 disabled。
  `mk2_shared_clock_cycle_test.gtest.xml`：7 tests、0 failures、0 errors、0 disabled。
  两份实际报告时间均为 2026-11-13 13:38:42，位于上述独立构建目录的
  `test_results/my_jearm_moveit_config/`；合计 13 个用例。
- 当前设置保留 `cmake.buildBeforeRun=false`；修改测试或源码后必须先显式构建，
  不能把旧二进制的测试结果算作新修改的验证。
- 本轮只修改编辑器环境和验证记录，未修改控制源码、未启动 ROS 节点或 SDK、未连接或操作实机。
  全链路真实 SDK FAIL/79、重复 Servo 生命周期及生产自动再接管仍未解决。

#### 2026-11-13 13:48 运动约束增量验证

- 修改客户端保持目标存档、适配器及测试后，显式编辑器构建返回 0；随后四个 CTest 目标均通过。
- `je2_ros_goal_adapter_test.gtest.xml` 时间 13:48:08，12 tests / 0 failures / 0 errors / 0 disabled；
  原共享时钟 7 个 gtest 同样通过。协议普通与独立 ASan/UBSan 目标也通过（不是另外两个 gtest）。
- C 协议回归新增非零首保持之后更新反馈的断言；保留全部既有协议用例。
  CMake 的普通/消毒器目标均强制保留 assert，避免 Release 的 NDEBUG 静默取消检查；
  消毒器目标实际编译 C 客户端及测试源，不是只在未插桩静态库外加链接标志。
- 无套接字/SDK/ROS 节点或物理运动；默认驱动、控制器插件及安装不变。
  本轮不构成完整 FollowJointTrajectory 执行器、重复 Servo 生命周期或自动原生交接验收。

#### 2026-11-13 14:23 隔离握手与失权联测

[测试服务器](../test/je2_loopback_server.c)直接复用实际会话解析器、控制引擎及
[Owner 格式化函数](jeserver_v2_owner.inc)，不是另写协议替身。
实验插件发布函数也改为调用该共享格式化函数；非会话分支的固定样本由
[字节兼容测试](../test/je2_owner_legacy_test.c)核对，能力标志仍为 false，小输出缓冲区拒绝。
本轮未重新构建或运行完整 PluginMain，不能将格式化单测等同于插件生命周期回归。

- 路径：实际 ROS Goal → 适配器/C 客户端 → 进程内 ZMQ → 实际解析器/引擎 → 内存运动回调 →
  实际 Owner 序列化 → 进程内 ZMQ → 客户端。claim 的静止资格轮询、Switch 确认、确认后的
  非零首保持及 ACTIVE/目标 ACK 全部经此路径产生，不直接写客户端 phase/ACK。
- 正常链路连续 25 个原样 ROS 目标，只有一次 Start；最后 ACK 不标记 ROS goal 成功。
  暂扣 ACK 时不跳点/推进历史；丢目标但反馈健康仍按固定期限锁存；实际成功消费的 ACK
  迟到也不能复活客户端。回调失败即使消费序号已前进，也不算成功。
- 点动意图编码注入引擎后退役客户端；已经序列化的旧目标在新代次被拒绝，且不调用反馈读取或运动回调。
  End 返回和空闲状态不足以点动，须显式注入替身 writer EXITED 证据并等待新的静止反馈。
  这是引擎边界测试，不是实际原生示教器传输、真实线程退出或第二次 Servo 验收。
- 状态 7 锁存为 FAULT，后续健康反馈不清错。合法本地目标仍可能被服务器跟随误差检查拒绝，
  且拒绝发生在追加运动回调之前；客户端校验不取代服务端保护。
- 重建测试服务器并显式新建客户端后重新握手，旧票据/旧 boot 帧不能借同一个数值代次复用；
  这不证明同一真实插件生命周期可以再次 Start。multipart 尾部不重新解释为新命令。
  本地 cancel 仅停发；服务器随后 watchdog 故障，不将取消返回视为物理停止确认。
- 仅 ZMQ_PAIR、`inproc://`、IO_THREADS=0，套接字收发超时 200ms；模拟时间由测试串行推进。
  不覆盖生产 PUB/SUB、TCP、并发/实时调度、网络延迟预算、真实 SDK 跟随或完整插件加载卸载。
  fixture 限位和理想化反馈不是实体配置。未连接控制器，未安装、启动节点或更改生产启动默认值。

验证：修改后显式编辑器构建返回 0，六个 CTest 目标全部通过。
`je2_ros_session_loopback_test.gtest.xml` 的磁盘报告为 **2026-11-13T14:23:52，11 tests，
0 failures / 0 errors / 0 disabled**，其中包括本轮新增的迟到 ACK 和跟随误差两个用例。
原目标适配器 12 个、共享时钟 7 个 gtest 同样通过，合计 30 个 gtest；另有非会话 Owner、
C 协议普通、C 协议 ASan/UBSan 三个 C 测试目标（消毒器覆盖 C 客户端，不是整个回环测试）。
报告位于独立编辑器构建目录的 `test_results/my_jearm_moveit_config/`。
完整自动原生交接、ROS 执行器及真实 SDK FAIL/79 仍未解决。

### 单步定时调度接口（2026-11-13 14:33）

`bind_scheduled(goal, now, max_lateness)` 使用相同的完整目标预检及不可变会话票据，
另将调度起点和迟到预算固定在接单时。预算必须是有限的 0–0.002 秒；2ms 是当前实验上限，
小于最小 5ms 采样间隔的一半，不是已验收的实体臂/网络实时预算。零预算不允许迟到。
旧 `bind` / `prepare` 保留用于显式点提交；定时目标只能使用 `dispatch`，不能用 `prepare` 绕过等待。

- `dispatch(ticket, now, out, size)` 单次最多生成一帧，无线程、睡眠、自循环、重试、插值或补发。
  到期时间始终是接单时刻加原始 `time_from_start`，发送迟到不改变累计命令时间。
- 返回 `DispatchResult`：`FrameReady` 时 `size>0`；其余状态长度为零、输出缓冲区清空。
  `WaitingDue` 表示尚未到时，`WaitingInterval` 表示距离上次生成帧不足 5ms（仅有 1ns 数值裕量），
  `WaitingAck` 表示目标等待确认。等待不消费点或修改命令历史。
- 期限检查在背压/间隔等待之前：超过当前点的预算即返回 `Expired`，将本地客户端锁存为 FAULT
  并清除目标，不追赶下一点。故障可早于协议的 100ms ACK 超时；不产生停止/释放帧。
  其它拒绝为 `Rejected`，原因须结合当前客户端状态区分，不能将旧票据拒绝当作新会话故障。
- 当前票据的每次等待仍先检查客户端时钟、反馈新鲜度、ACK watchdog 和命令历史一致性。
  旧票据先隔离，不能借 NaN 时钟、取消或发送失败回调破坏新会话。独立 `observe` / `tick` 仍必需。
- 发送责任仍在调用方：同一串行执行上下文立即尝试发送 `FrameReady`，发送失败调用 `send_failed`，
  不缓存待重试帧。最小间隔是**生成帧时刻**的检查，不证明真实网络投递/SDK执行间隔或实时能力。
- 最后一帧生成后只报告 `WaitingAck`，必须有身份、阶段、健康和序号匹配的最终确认才报告 `Drained`。
  `Drained` 不是到位、目标完成、释放控制权或 ROS success。它不会清除绑定、自动接收下一目标、
  续发保持或停止帧。继续驱动 watchdog；未另行实现受保护的收尾协议时，服务端仍会因断流故障。
- `cancel` 仍仅退役本地客户端、失效定时回调；不是物理停止或 ROS cancel 完成证明。
  该接口没有加入 ROS action server、完成判据、受保护释放或自动再接管。

新增 11 项实际握手回环测试覆盖：定时 25 点及最终确认、预算内背压不重定时、过期停发、
最终帧未确认不能 Drained、首点唤醒过晚、迟发后最小生成间隔、预算/绕过拒绝、等待时坏时钟、
新会话的旧定时回调隔离、取消和发送失败。原 11 项联测保留，共 22 项。
最初一项测试用 `.01*i` 计算期望时间，比消息 `nanosec*1e-9` 略早一个浮点单位，正确返回 WaitingDue；
测试改为使用消息时间，并新增目标时刻前一个浮点值仍不能发送的断言，没有放宽调度实现。

修改后显式构建及六个 CTest 目标全通过。磁盘 XML 时间 **2026-11-13T14:33:54**：
联测 22、适配器 12、共享时钟 7，共 41 个 gtest，全部 0 failures / 0 errors / 0 disabled；
旧 Owner、C 协议普通和 C 协议 ASan/UBSan 同样通过。仍仅进程内 ZMQ、模拟时钟/运动回调；
无真实 SDK、生产执行器接线或实体操作，完整自动原生交接与真实 SDK FAIL/79 未解决。

### 显式会话收尾合同（2026-11-13 14:49）

新增 C 接口 `je2_client_close(client, captured_lease, now, out, size)`，ROS 适配器对应
`request_close(ticket, now, out, size)` / `close_state(ticket)`。这是结束流或取消意图的
**会话收尾原语**，不是 ROS action result，也没有把新恢复接口或状态 7 放行加入引擎。
只调用既有、受私有实验构建隔离约束的 Release 路径；回环用内存回调，不调用真实 RobotStopRun。
不能据此把该回调的实体停止语义或未知停止来源认定为安全。

#### 调用和期限

- 只接受当前 ACTIVE 会话的原 ticket/lease。先核验完整身份，再访问时钟；旧收尾回调不能操作新会话。
  必须已有新鲜 Owner 携带两个类型/层级/规范数字都合法的释放字段。旧服务器缺字段仍可观察，
  但收尾请求在生成 Release 前本地 FAULT，输出为空；不猜测或回退为无确认释放。
- 第一次请求进入 DRAINING，立即禁止后续目标并清除适配器的活动调度绑定。
  已格式化且未确认的目标不能跳过；保留原目标 ACK 截止期，等待它到达或故障，不延长期限。
- 等待完成后仅生成一次 `JE2F1 ... Release {"Release":true}`，进入 CLOSING。
  正返回是唯一一帧，0 是等待/无新输出，负值为拒绝。之后显式轮询同一 ticket 不重发。
  格式化和立即发送仍须串行，失败必须调用 `send_failed`；不能积压、重发或在失权后发送停止。
- 从第一次请求计固定 500ms 总收尾期限；Release 自身未确认期限为 100ms；反馈间隔仍为 150ms。
  持续健康但 writer 未退出的反馈不会延长总期限。超时锁存为 FAULT；停发不证明已物理停止。
- `close_state` 区分 None、Pending、Confirmed、Retired、Fault。
  旧 `cancel(ticket)` **仍仅本地退役**，并未改成发 Release。需要显式收尾时应直接请求
  `request_close`，不能先 cancel 退役后又试图借旧身份释放。请求期间原 `dispatch/prepare` 不再输出。

#### 回执与退出屏障

- 服务端仅在规范 Release 从 ACTIVE 且拥有 servo 的状态实际处理成功、代次加一并进入 YIELDING/RELEASING
  后记录 `session_release_sequence`。消费计数已前移但回调失败不记录成功请求；原生点动引起的 yield
  也不伪装为该 Release 的回执。新 claim 清除此关联。
- 阶段 1 要求当前代次仍等于被释放会话代次加一，且处在既有 YIELDING 或 EXIT_BARRIER。
  阶段 2 还要求核心 READY、已调用 End、无故障/清理错误、`had_writer=false`、无待点动或活动点动，
  原引擎已完成特定 writer EXITED 观察和退出后不少于 100ms 的新鲜静止确认。
  序列化函数只读取这些引擎条件，不增加 SDK 读取，也不自行把 End/state0 推断为 writer 退出。
- 客户端同时核对 boot/client、被释放的 session_generation、当前 generation=原代次+1、
  Release 消费序号和释放序号相等、阶段/Owner/健康反馈。阶段 2 必须对应 PENDANT/READY/MoveState0。
  重复/倒序快照不能把 Pending 推成 Confirmed；错误类型、错误层级、重复键或整数溢出拒绝。
- 点动抢先使原会话失效，或新 claim 在最终回执送达前已经发生时，只退役旧客户端，不清理新所有者。
  任一状态 7/故障仍为 FAULT，后续健康反馈不能变成成功收尾。
- 确认后 C 客户端进入终态 CLOSED，不能自动 bind/claim/再次 Servo。保留的 Confirmed 仅是
  **收到过该次引擎屏障回执的历史事实**；不会继续用它宣称当前位置、停止来源或当前控制权。
  它没有检查 ROS 最终目标容差，不能作为 ROS success/canceled 或实体停止的独立证明。
  所有公开标识和回执仍不是认证；真实线程观察器、SDK 停止来源及重复生命周期须另行验收。

#### 验证

- 新增 10 项完整握手回环用例：Drained 后显式收尾、取消等待已有目标、原生意图竞态、释放丢包、
  未确认目标丢失、writer 一直未退出、收尾中状态 7、发送失败/旧回调、新 Release 回调失败、
  新 claim 先于关闭回执。新 claim 测试没有进行第二次 ServoStart。
- C 普通及 ASan/UBSan 回归新增完整/重复回执、旧 lease、14 类非法回执、重复键/溢出/指数写法，
  并验证旧服务器缺回执字段时不生成 Release。
- 实际编辑器构建和六个 CTest 目标全部通过；磁盘 XML 时间 **2026-11-13T14:49:46**：
  联测 32、适配器 12、共享时钟 7，共 51 个 gtest，0 failures / 0 errors / 0 disabled。
  旧 Owner 固定样本、C 协议普通和 C 协议 ASan/UBSan 三个 C 目标通过。
- 修改实验会话解析/Owner、C 客户端和适配器，未改变控制引擎算法、默认非会话字段或启动保护。
  没有本轮完整 PluginMain/真实 SDK/实体运行，没有生产 action server/JTC 接线。
  本轮 writer EXITED、时钟及运动均为替身，真实 SDK FAIL/79 和完整自动再接管仍未解决。

### 实验目标生命周期（2026-11-13 15:02）

[RosGoalExecution](../src/je2_ros_goal_execution.hpp)在现有适配器上连接以下流程：
`start → Streaming → 最终 ACK → Closing → 终点检查 → 一次性结果`。
取消使用 `request_cancel → Closing`，不先调用仅本地退役的 `adapter.cancel`。
该类无套接字、线程、ROS 节点或 action server，仅在测试目标引用，不能当作生产执行器启动。

#### 接单与输出

- 显式 Policy 按值保存：七轴正且有限的位置容差、0–2ms 调度迟到预算、正且不超过 150ms 的结果证据年龄。
  用例容差 0.0001、证据年龄 50ms 仅为测试合同，不是实体臂验收参数，也不能由 goal 放宽。
- 除底层完整轨迹预检外，要求至少两点、最后两点位置完全一致、最后一点声明速度与可选加速度为零。
  不插入保持点，不修改运动终点。该命令终点条件不等于实体瞬时速度/加速度证明。
  服务端未声明合法释放回执字段时不接新目标，避免发送后才发现没有收尾协议。
- 拒绝抢占正在处理的目标，也不覆盖尚未领取的终态结果；旧回调始终携带原 Ticket。
  接单后保存不可变轨迹和最终位置；外部修改 goal 或 Policy 不改变已接受目标的判据。
- `advance` 是单步接口，返回 Waiting、Target、Release、Terminal 或 Stale；只有 Target/Release 有帧。
  Streaming 沿用严格定时调度，Drained 后该组件显式调用收尾接口，最多生成一次 Release。
  这是此组件主动连接收尾流程，不改变底层 `dispatch` 本身不自动收尾的合同。
  调用方必须串行、立即尝试发送，失败调用 `send_failed`，禁止重试；独立 observe/tick 仍须驱动。

#### 结果与取消

- 普通结束只有在该票据 CloseState=Confirmed，并取得同会话、同关闭回执保存的位置/序号/本地接收时间后，
  才检查证据时间不在未来、年龄未超限、最终七轴位置误差均在策略容差内，产生实验结果 Succeeded。
  不满足终点容差则 Aborted/GoalTolerance；陈旧或无法取得证据则 Aborted/StaleEvidence。
  不等待任意未来健康反馈清除已确定失败，也不在已经关闭的会话上追加 Stop。
- `request_cancel` 立即禁止该目标的进一步调度，已有未确认帧仍按原期限等待，随后走同一次收尾。
  只有有效、新鲜关闭回执才产生 Canceled；取消不要求到达已放弃的原目标位置。
  关闭中可接受取消，但关闭回执已经到达或结果已终态时拒绝，不能事后改写结果。
- 取消期间的状态 7、其他控制故障、失权、时间/ACK 错误或发送失败仍产生 Aborted，不是 Canceled。
  失权不生成旧身份的 Release；调度过期等故障也不尝试用新释放帧掩盖故障。
- 终态结果不可再修改。`take_result(ticket)` 对正确票据只返回一次；错误票据/重复领取不消费新目标的结果。
  结果未领取不能接下一个目标；领取后仍须满足新显式会话条件，不能重用 CLOSED 客户端。
  此阶段尚未接线 ROS action handle，不能把此处枚举误称为已发出的 ROS 结果；后续接线验证见下节。
- 成功/取消枚举只满足上述实验模型合同；不是安全停止证书、厂商原子反馈、停止来源证据、当前控制权，
  也不证明真实 SDK 可第二次 Servo。生产和实体模式仍保持原有拒绝/验收边界。

新增 10 项实际握手回环测试覆盖：终点及回执后成功/一次性结果、闭合但目标误差超限、取消等待已有目标、
正常关闭中取消遇故障、失权、调度过期、陈旧关闭证据、非法策略/移动终点/不抢占、旧回调与新目标隔离、发送失败。
修改后实际编辑器构建及六个 CTest 目标全通过；磁盘 XML 时间 **2026-11-13T15:02:34**：
联测 42、适配器 12、共享时钟 7，共 61 个 gtest，0 failures / 0 errors / 0 disabled；
旧 Owner、C 协议普通及 C 协议 ASan/UBSan 三个 C 目标也通过。消毒器覆盖 C 客户端，不是新 C++ 生命周期。
仅内存运动/模拟时钟/替身 writer 与进程内 ZMQ，无真实 SDK、ROS action 节点或实体执行。

### 真实 action 接口隔离联测（2026-11-13 15:37）

[RosActionExecutor](../src/je2_ros_action_executor.hpp)创建真实 Humble `rclcpp_action` 的
`FollowJointTrajectory` 服务端，以实际 goal UUID/handle 绑定上述不可变 Ticket；
[测试](../test/je2_ros_action_executor_test.cpp)使用真实客户端、取消服务和结果/反馈消息，
不是直接调用伪造的 action 回调或只检查内部枚举。

#### 调用与执行合同

- 构造时显式传入 node、action 名称、适配器、执行策略、单调时钟及非阻塞单次发送函数。
  类本身没有定时器、网络传输、自动 claim、硬件激活、进程入口或安装规则。
  仅 `BUILD_TESTING` 中有引用，不占用现有生产控制器名称，不改 MoveIt/ros2_control/launch 默认值。
- 构造、ROS 回调、所有 client/adapter 操作、`observe`、`poll`、`shutdown`、销毁必须在同一个线程串行执行；
  外部线程调用在接触客户端前抛出异常。采用单线程 executor，不使用 detached 线程或嵌套 spin；
  注入时钟/发送函数必须非阻塞、不重入。不是多线程/实时调度认证。
- goal 回调先预检并绑定，拒绝未就绪、不支持的目标及抢占；accepted 回调关联同 UUID 的真实 handle，
  `ACCEPT_AND_EXECUTE` 后不再重复调用 `execute()`。accepted 前不发目标。
  调度起点仍是接单时间，不因 DDS 应答或第一次 `poll` 延迟而重置。
- `poll` 独立推进一次调度或收尾，生成帧即单次发送；返回 false 或发送异常按 SendFailed 中止，不重发。
  `observe` 验证原会话 Owner；不把 transport enqueue 当作目标 ACK。拒绝点集的原有全部条件保留：
  七轴严格名称/顺序、5–50ms 已采样点、显式限位及最终保持、无 goal 自选容差/多自由度等。
  不自动把普通稀疏 MoveIt 轨迹插值成此格式，也不处理 `/clock` 跳变来恢复动作。
- 取消回调只决定是否接受并发起一次受保护收尾；实际框架在回调返回后才转为 CANCELING。
  回调中不调用 `canceled()` 或 `abort()`，即使 Release 发送失败也等后续 `poll` 发布结果。
  同线程约束和 `is_canceling()` 校验共同避免“取消已受理但框架尚未转状态”的发布竞态。
- 反馈只包含已观察的位置、最近发送目标的位置及位置差，不虚构实际速度/加速度；
  `desired.time_from_start` 为最近发送点的原时间，未提供实际采样相对时间（保留默认零）。
  header 是发布时 ROS 时间，不是厂商采样时间。只发布匹配会话且年龄在策略界内的新序号，重复轮询不刷新证据。

#### ROS 结果映射

| 生命周期结论 | ROS action 状态 | FJT error_code |
|---|---|---|
| 关闭回执新鲜且终点容差满足 | SUCCEEDED | SUCCESSFUL |
| 已受理取消且关闭回执新鲜 | CANCELED | SUCCESSFUL（必须同时读 action 状态，不是到达原目标） |
| 关闭但终点误差超限 | ABORTED | GOAL_TOLERANCE_VIOLATED |
| 失权、故障、超时、发送失败、陈旧证据等 | ABORTED | PATH_TOLERANCE_VIOLATED，并在 error_string 写明实际原因 |

FJT 没有 `CONTROL_FAILED` 常量；以上是实验端明确的错误映射，不伪造新常量或将故障映射为成功。
真实 handle 只发布一次终态，旧 UUID 的取消请求不能作用于新目标。
`shutdown` 对未提交结果的目标发布 ABORTED，不发停止/释放帧，不声称物理停止；
析构前优先显式调用 shutdown，避免框架 goal-handle 析构默认 CANCELED 被误作有效收尾。
发布异常锁住新接单/输出，不自动重试；DDS 故障/进程崩溃下不承诺客户端必定收到结果。

#### 隔离与实际证据

- 私有 ROS Context 显式 domain 217，PID/实例独立命名空间，忽略全局 ROS 参数；
  `ROS_LOCALHOST_ONLY=1`、固定 `rmw_fastrtps_cpp`，不启动参数服务或 rosout。
  [DDS 配置](../test/je2_action_loopback_dds.xml)关闭 builtin transport，仅声明 127.0.0.1 UDP。
  创建节点前断言 XML 解析成功及 profile QoS 合法；创建后再检查实际 participant 的每个 transport。
  Humble 的 localhost 模式还追加回环 UDP 和本机 SHM：允许这两类，拒绝无白名单/外部接口/未知 transport，
  并确认无 multicast locator。回环/共享内存不是认证，也不是网络 namespace 隔离或纯 inproc 传输。
- 控制器侧使用显式假发送队列，驱动真实会话 parser、引擎、Owner 序列化和内存 IO。
  claim/Switch/首次保持真实走协议，无手设 ACTIVE/ACK；writer EXITED 和控制时钟仍为替身。
  原有另一个 CTest 继续覆盖进程内 ZMQ；本轮不覆盖生产 PUB/SUB、SDK 实时延迟、硬件或完整 PluginMain。
- 14 项 action 用例验证正常成功/反馈/一次性结果、未就绪/非法目标/忙碌拒绝、取消等待已有 ACK 和 writer 屏障、
  取消后 state7 中止、失权不发旧 Release、终点误差、首次 poll 过期、目标和取消 Release 发送失败、
  回执后的迟到取消、取消期间局部退出、异线程拒绝、旧取消 UUID 与新会话隔离、重复 poll 不重发反馈/目标。
  新会话用例重新初始化**新的替身控制器生命周期**，不是同一真实 Servo writer 再次启动。
- 最终实际编辑器构建与七个 CTest 目标全部通过；磁盘 XML **2026-11-13T15:37:04–15:37:05**：
  action 14 + 会话联测 42 + 适配器 12 + 共享时钟 7 = **75 个 gtest**，0 failures / errors / disabled。
  旧 Owner、普通 C 协议和 C 协议 ASan/UBSan 三个 C 目标亦通过；消毒器不覆盖 C++ action/DDS。
- 本轮修复两个验收陷阱：新 ROS 节点测试需要 `APPEND_LIBRARY_DIRS` 指向已配置 rclcpp 库目录；
  空 `metatrafficMulticastLocatorList` 被此 Fast DDS 版本拒绝，早期测试虽绿但 XML 配置被忽略，不能作为隔离通过证据。
  最终已移除该空标签，加入加载/实际 QoS 双检查，并确认测试日志无 XMLPARSER 错误。

**实际 ROS action 接线现已在替身环境通过；生产驱动/JTC/MoveIt 运行入口、独立传输/时钟驱动循环，
原生停止来源、可重复 Servo 和完整自动再接管仍未完成。未安装、连接实体控制器或运行真实 SDK。**

### C 客户端目标接口合同

- 接受新 goal 时，调用 `je2_client_get_lease` 获取 boot/client/generation 的**值拷贝**，
  在该 goal 的回调中一直携带同一份 lease。不能等旧回调执行时才读取当前 lease，
  否则会把旧目标错误标成新会话。lease 是关联值，不是认证或同一会话内的 goal ID。
- `je2_client_target` 只允许 ACTIVE 当前会话输出。旧 lease 在检查时钟、目标之前就拒绝，
  不因旧回调的 NaN/坏时钟破坏新会话。周期 tick 仍必须独立运行，不能依赖旧回调触发 watchdog。
- `data_time` 是从首次保持延续的**累计命令时间**，不是每帧 dt，也不是墙上时钟。
  当前实验客户端要求增量 5–50ms、相对首次发送的累计时钟偏差不超过 50ms，所有关节为有限数。
  不自动修改时间、插值、补发或重新定时。此处仅是协议/时序检查，调用方仍需执行已有轨迹限位验证，
  服务端的位置、跟随误差、速度、加速度和采样保护继续保留。
- 最多一个后续目标等待 ACK；返回正数为一帧，0 为有命令未确认且**新目标没有入队**，负数为拒绝。
  返回 0 的参数不会被缓存、之后也不会自动发出；调用者不得积压或追赶过期轨迹。
  格式化须与立即发送串行；发送失败须报告，不能重试旧帧。
- 首次保持后的目标使用严格连续序号。确认必须同时核对当前身份、PC/ACTIVE、无故障及消费序号。
  消费计数越过已发送序号或在新快照中倒退均锁存。固定 ACK 等待期限 100ms，
  正常但消费序号不前进的快照不会延长它，迟到 ACK 不能复活故障客户端。
- 序号耗尽不会回绕；不合法当前目标、超时或发送失败后停发。**停发不是已停止运动的证明**，
  客户端不自动发送可能干扰新所有者的停止/释放；服务端 watchdog/故障路径仍负责其自身控制责任。
- 单未确认窗口是保守实验设计，不承诺 ROS 控制频率、长时间吞吐或网络延迟预算。
  即便 ACK 在 100ms 内到达，下一目标若已超出 50ms 时间增量也必须拒绝，不能放慢时间轴继续执行。

## 最新连续目标验证

- 普通与 ASan/UBSan：`/tmp/je2-session-client.yKFwm6qI`。
  连续确认、单帧背压且不缓存、旧 boot/client/generation 回调隔离、11 类坏输入/溢出、
  正常反馈但 ACK 停滞、迟到/未来/倒退 ACK、发送失败及失权停发通过；前一轮测试仍通过。
- 完整私有 ZMQ/插件/内核 writer/替身 SDK：`/tmp/jeserver-v2-sandbox.plcy19if`。
  首保持后连续发送并确认 25 个按累计时间生成的微小测试目标，单次 ServoStart；
  点动让位后本地不再输出，已序列化的旧帧再次注入仍被服务端隔离。
  四个独立替身实例分别确认位置、跟随误差、速度、加速度故障在新增 SDK 目标调用前拒绝。
  故意丢掉一个已格式化目标时，服务端仍发布真实健康快照，客户端仍在固定 ACK 期限后锁存且不重发。
- 默认完整插件回归：`/tmp/jeserver-v2-sandbox.pvl79iEH`。本轮服务端生产源码、ROS 驱动、
  实机安装均未改动；无远端命令或真实 SDK 运行。当时的 CMake 阻碍已由上述后续环境修复解除。
- 以上不是实体示教、完整轨迹执行或第二次 Servo 生命周期验收，真实 SDK FAIL/79 保持未解决。

## 前一轮客户端及快照验证

- 独立普通与 ASan/UBSan：`/tmp/je2-session-client.wAxTYpQ1`。
  覆盖完整阶段、固定申请期限、重复消息不续期、全部 13 个数字字段错层、独立溢出/类型错误、
  截断输出、发送/时钟/健康错误；首次保持测试确实改变确认后位姿并注入旧快照。
  申请中/绑定/活跃三个阶段重启及外来 client 确认均不能重入。
- 完整私有插件及实际客户端：`/tmp/jeserver-v2-sandbox.GBLPYdYf`。
  五私有 namespace、真实 ZMQ/PluginMain/内核 writer，**替身 SDK**：客户端实际申请并确认、
  首次保持进入 ACTIVE，原点动意图触发让位后客户端 RETIRED，不能输出旧保持或清理。
  原服务端 DSO 重载/旧帧/状态 7 等用例也通过；不是实体原生示教面板验收。
- 插件入口普通/ASan/UBSan、原仿真及实体对象编译：`/tmp/jeserver-v2-plugin-tests.6Y0atiIr`。
  默认完整插件回归：`/tmp/jeserver-v2-sandbox.SPagDFJQ`。
- 没有安装、远端暂存、真实 SDK 运行或实体操作；真实 SDK 全链路 FAIL/79 仍未解决。

## 前一轮服务端验证

- 辅助入口正常与 ASan/UBSan、原插件/TCP/对象编译回归：
  `/tmp/jeserver-v2-plugin-tests.wrFeIq6z`。
- 完整新会话插件：`/tmp/jeserver-v2-sandbox.WvCoRDrh`。
  真实 ZMQ → 首次 PC Servo（替身）→ 内核 writer 退出 → 点动 → 替身 STOP 返回 0 状态 →
  新 claim/Switch → 旧目标/stop/release/multipart 隔离；multipart 后新 Release 正常送达。
  第二次 ServoStart **没有尝试**。另验证实际 DSO 重载改变 boot、重复目标不执行、缺序故障、
  STOP 后状态 7 及随后正常反馈仍不能 claim。
- 默认完整插件 `/tmp/jeserver-v2-sandbox.uJjqmmAJ`、原 writer 实验
  `/tmp/jeserver-v2-sandbox.yaoetqKB` 均通过；原 105 组入口异常锁存、12 项首次故障、11 项伺服观察回归通过。
- 生产启动拒绝测试 `/tmp/robotmain-v2-pro-test.5wLd4xoX` 通过。

以上会话验证全部使用替身 SDK，不能替换[真实 SDK 全链路仍 FAIL/79 的记录](tests/SDK_HANDOVER_REVIEW.md)。
本轮未暂存到控制器、未部署、未重启或操作实体。后续仍需 ROS 客户端适配及明确的生产验收，
不能通过反复加载插件来规避一次性 writer 或故障锁存限制。