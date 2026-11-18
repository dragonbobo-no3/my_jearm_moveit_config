# 显式虚拟接管插件实验（2026-11-12）

## 当前结论

**真实 SDK 已走通 PC Servo → 内核实例退出 → 点动获权/实际虚拟位移/旧 PC 隔离；STOP 后健康空闲验收失败，总体仍未通过。**
修复就绪等待竞态后的实际产物为 `/tmp/jeserver-v2-sdk.5ZngX8NA`，退出 79。
失败不是密码或初始化问题，而是原生点动 STOP 后返回 `MoveState=7`，现有保护正确锁存 FAULT。
此前真实 SDK 的三轮写线程实例退出已通过，但它没有加载插件或点动，不能替代本次接管验收。
生产启动仍拒绝，现有示教器完整协议仍未接入。

## 后续会话通信实验（仅替身 SDK）

显式会话编译已接通 ZMQ claim/带代次帧、随机启动标识、客户端标识和连续序号隔离。
完整私有插件测试验证 PC → writer 退出 → 点动/替身 STOP 状态 0 → 新 claim/Switch，
以及旧帧、multipart、实际 DSO 重载和状态 7 拒绝；普通及 ASan/UBSan 回归通过。
**没有第二次 ServoStart、ROS 驱动接线或真实 SDK 新验收**，不改变本页真实全链路 FAIL/79 结论。
默认插件与生产入口不启用新协议，未部署；细节见[会话通信实验](../PC_SESSION_PROTOCOL.md)。

## 2026-11-13 后续：停止接口只读审计与入口异常锁存修复

### 新的静态证据：order 也不是停止来源

只读核查控制器实际厂商头文件和固定 SDK 导出/反汇编，前后 SHA256 均为
`40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1`，没有加载 SDK。

- `RobotStop` 的合法 robot 分支在 0x12f38c–0x12f393 调用 `set_robot_order(1, robot)`；
  `set_robot_move_stop` 在 0x133c60–0x133c67 也转到同一调用。
  与先前 teach STOP 调用链合并，说明两种 API 最终写入相同的停止 order。
- `set_robot_order`（0x69b30）在信号量保护下于 0x69b66 写入每个 robot 的单个整数，
  随后 msync/释放信号量；该函数没有记录调用来源或事件序号。
  `get_robot_order`（0x69ae0）读取这个整数，不能从重复写入的 1 重建停止历史。
- **`get_robot_order1`（0x69ba0）不是只读 getter**：先读取旧值、释放读锁，再于
  0x69be4 调用 `set_robot_order` 写入传入值，最后返回旧值。禁止把它作为诊断接口导入；
  函数名或存在导出符号都不是无副作用/公开支持的证据。本轮未调用它或任何 order 接口。
- 公开 `get_robot_run_state`（0x165f70）直接转到 `get_robot_move_state`，不是另一条原因通道。
  `CheckRobotState` 的扫描在 0x119800 / 0x119805 把状态 7 和 0 汇入同一计数分支，
  是汇总状态而非源事件序列。
- 本次头文件候选中的 InputEvent 是 Linux input/joystick 外设读取；Modbus/TCP 的回调是
  自建通信服务处理函数，不是原生示教 STOP 的前置仲裁或厂商安全事件回调。
  在本次检查范围内没有找到可直接接线的公开停止原因/事件序列合同；不据此断言厂商另无接口。

### 已修复：请求读到的异常不能被下一帧正常反馈抹掉

审查[实验引擎](../jeserver_v2.c)发现：`je2_pendant_jog` 和 `je2_claim_pc` 的部分拒绝路径
没有锁存当前异常；忙状态还可能先于反馈检查返回。交接后的 `AVAILABLE` 在 tick 中也直接跳过。
因此异常若仅被某个请求入口读到、下一次 tick 已正常，旧实现可能继续接受后续请求。

新增 `observe_healthy` 统一处理符合当前代次/所有权的可信反馈读取：

- 已进入控制生命周期后，点动、claim、PC 帧及 tick 任一有效入口读到异常，立即锁存 FAULT；
  不再靠下一次 tick 重新看到同一异常。忙状态拒绝不能丢弃已经读到的异常。
- 保留旧 PC/错误代次在时钟和反馈检查之前的隔离，避免旧请求触发新所有者的清理。
- 初始未参与控制的退使能状态仍是未就绪拒绝，不锁死启动；交接后即使回到 `AVAILABLE`
  也继续保护。后续正常反馈不清锁、不生成新代次、不再次点动，不重复 Stop/End 清理。

[新增回归](jeserver_v2_test.c)覆盖 105 组组合：5 个生命周期位置、符合入口所有权的
点动/claim/PC/tick，以及状态 7、负状态、驱动错误、运动错误、掉使能、无效样本、NaN 关节。
另保留初始退使能与旧代次不触发清理的用例。
先在旧引擎上复现断言失败（`/tmp/jeserver-v2-offline.YQlcG7A1`），修复后普通和
ASan/UBSan 均通过（`/tmp/jeserver-v2-offline.eZjaemPF`），含原 12 项首次故障及 11 项伺服观察回归。

插件/真实 TCP 辅助路径及 sanitizer 回归通过：`/tmp/jeserver-v2-plugin-tests.hv94XvLh`。
完整真实工作线程/内核任务、五私有 namespace、替身 SDK 联测也通过：
写线程观察实验 `/tmp/jeserver-v2-sandbox.psy8udPg`，默认路径 `/tmp/jeserver-v2-sandbox.nbdhHMnp`。
以上均为**替身 SDK 测试**，新增修复未在真实 SDK 下重跑，未暂存/安装到控制器。
原生产 sim/physical 编译分支通过，安装插件与运行策略未改。

### 仍不能用本地记账替代的合同

此修复只记住**已经被读取到**的异常，不保证观察到全部外部停止，也不证明停止来源。
即使增加事件回调，若下一次 `robot_teach_joint` 内部恢复与外部停止没有统一仲裁，
“读事件后、开始运动前”的竞态仍存在。下一阶段需要厂商支持并验证：

1. 停止来源及单调事件序列的定义、丢失/重连/进程重启边界，覆盖外部与内部停止路径。
2. 外部停止锁存优先于正常松键；正常会话停止不会覆盖已经存在或后来到达的外部停止。
3. 续动与最新停止事件检查在 SDK 控制边界具有明确的原子/串行语义，不能靠用户态轮询后盲调点动。
4. 原生按钮、PC、其他 SDK 写入路径的独占/旁路阻断，以及旧 Servo writer 退出和 ROS 新代次接管。

这不是要求新增一个替代点动键的“接管按钮”。原生按键体验目标保持不变，
但不能用未实现的回调、可改写的 getter 或 `own_stop` 标志伪造验收。全链路 FAIL/79 与生产启动拒绝不变。

## 2026-11-13：真实 SDK 停止来源对照已完成

新增两个专用入口，各在全新的五 namespace / SDK 实例内执行一次虚拟点动：

- `--test-teach-external-stop-virtual`：点动后仅调用 `RobotStop(0)`。
- `--test-teach-overlap-stop-virtual`：点动后调用自身 teach STOP，紧接着调用 `RobotStop(0)`。

两者均不再点动、不显式恢复、不调用 StopRun；不加载插件或启动 Servo。
复用锁定策略、固定库指纹、私有设备/共享内存和时间/位移限制，未改变生产引擎。
这是测试主动注入的软件停止及固定顺序叠加，**不是硬件急停或所有并发时序的测试**。

### 结果与可复核证据

| 来源 | 实际产物（控制器） | STOP 后 Power / State / DriverError / MoveError | 最终 J1 / rad |
|---|---|---|---:|
| 原生 teach STOP（上一轮基线） | `/tmp/jeserver-v2-sdk.zOow6pqB` | 1 / 7 / 0 / 0 | 第一轮 0.000035596707639673065 |
| 外部 `RobotStop` | `/tmp/jeserver-v2-sdk.Rc1LOoti` | 1 / 7 / 0 / 0 | 0.000014950617208662687 |
| 自身 teach STOP 后追加 `RobotStop` | `/tmp/jeserver-v2-sdk.Y1vU5u7p` | 1 / 7 / 0 / 0 | 0.000035596707639673065 |

新增两次实际测试均退出 0、仅一次点动且有实际虚拟位移，停止 API 均返回 0，
停后状态 7 / 关节稳定至少 100ms。两次使用相同新版探针 SHA256：
`49efc086bb4720fa4c3ef3ccc0da76714caff2987ec0e37fb9056e548871df49`。
上轮原生基线使用旧探针，指纹和两轮数据保留在下一节，未将它冒称为新版运行结果。
隔离校验用但未加载的插件 SHA256 仍为
`5523034fa0edd4f253ba389f9196a19cc61f16b1c5494eb98783c81beec96bb8`。

- 新增 `RobotStop` 的实际厂商头文件 ABI 检查通过；不加载 SDK 的预检产物为
  `/tmp/jeserver-v2-sdk.irE6YUvg`。
- 34 类 native/external/overlap 替身控制流测试、原 65 项普通及 65 项 sanitizer 探针回归、
  状态 7 拒绝及 CLI/实时限制/环境注入拒绝通过；离线产物
  `/tmp/jeserver-v2-sdk-offline.FML78sLe`。替身测试不作为真实 SDK 行为证据。
- 独立普通 SSH 重读两份实际日志和二进制指纹，分别比较前后配置及进程快照一致，
  无私有测试残留；实体 RobotMain 仍为 PID 76071，PC 无控制进程或 8001 监听。
  这不等于已核验实体退使能。
- 实体 SDK / GlobalConfig / 已安装诊断库指纹仍为 `40c38d1a...` / `6dbf8e7d...` /
  `c834b048...`。只更新普通用户暂存源码，未安装、重启或向实体发送命令。

### 对交接实现的约束

**仅凭上述四个反馈字段不能区分这些停止来源；记住“我刚发过 STOP”也不足以排除随后外部停止。**
这排除了“自身 STOP + 状态 7 + 零错误码就自动恢复”的实现依据。
没有测试外部停止后的下一次点动，因此不宣称 SDK 会拒绝续动；探针明确不尝试续动。
也不能据此断言厂商所有接口都无法提供停止原因，只能说当前读取的字段不够。

下一阶段需要可验证的停止原因/事件顺序和厂商支持的续动条件，并覆盖：
正常松键、外部停止、两者叠加、迟到事件，以及检查后/下一次运动前到达的新停止。
本地停止标志和轮询健康状态不能单独充当这一合同。
完整原生点动键入口、旁路阻断、生产独占 writer 退出及 ROS 新会话代次仍需接通并验收。
原全链路 FAIL/79 结论不变，不放行状态 7，不自动清错/续动；自动交接与再接管仍未完成。

## 2026-11-13：独立原生示教语义验证入口

为验证“下一次原生点动隐含恢复”而不放宽交接引擎，新增
[专用探针](jeserver_v2_sdk_teach_probe.inc) 和 `--check-teach-sdk` / `--test-teach-virtual`。
专用编译宏 `JE2_SDK_TEACH_PROBE=1` 与 handover 宏互斥，普通二进制拒绝 teach 模式，
teach 二进制拒绝普通加载/Servo/交接模式。原生控制器与部署插件不变。

- 复用固定 SDK/technology 指纹、五私有 namespace、只读根、私有共享内存、仅 loopback、
  隐藏实体设备/配置、私有只读 ENI、零 capabilities、30 秒外层期限及实时 CPU 上限。
- 保持桥策略 `allow_commands=false`，不加载插件、不启动 Servo、不建立 PC/按钮通信。
  **探针会直接使能虚拟机器人并调用两次原生点动，因此不是只读 SDK 测试。**
- 每次点动观测窗口小于 200ms，累计位移必须小于 0.02rad，非目标轴漂移不超过 1e-5rad。
  STOP 后只把连续至少 100ms 的静止状态 7 作为待验证现象；第二次原生点动仅用于
  观察 SDK 内部恢复与实际虚拟位移，绝不把状态 7 当生产空闲或交接 ACK。
- 不直接调用恢复、StopRun、清错或私有状态写入。SDK 的点动接口自身可能恢复；
  失败不重试点动，必要时只请求 teach STOP，然后退出整个私有 PID namespace。
- 使用[只编译 ABI 检查](jeserver_v2_sdk_teach_abi.c)与实际厂商头文件核对全部函数签名。

已通过十类替身控制流测试、专用模式拒绝、原有 65 项普通/65 项 sanitizer 探针回归，
以及原交接状态 7 拒绝测试；产物 `/tmp/jeserver-v2-sdk-offline.5TARMcBk`。
实际控制器 `--check-teach-sdk` 通过，产物 `/tmp/jeserver-v2-sdk.5kB7hiJy`，
ABI、五 namespace 和锁定策略全部通过，前后实体进程/配置/启动器快照一致，**未加载 SDK**。
预检时实体 RobotMain PID 为 76071，助手未启动或停止它。

### 实际真实 SDK 结果：两轮原生点动/停止通过

实际执行 `--test-teach-virtual`，产物 `/tmp/jeserver-v2-sdk.zOow6pqB`，退出 0。
ABI、隔离、SDK 初始化与虚拟使能均成功；未加载插件、未启动 Servo、未显式调用恢复。

| 阶段 | MoveState | J1 实际反馈 / rad | 返回值 |
|---|---:|---:|---:|
| 第一次点动前 | 0 | 0 | — |
| 第一次原生点动后，新位移 | 1 | 0.0000029663923033060887 | 0 |
| 第一次 STOP 后，连续静止状态 7 | 7 | 0.000035596707639673065 | 0 |
| 第二次原生点动前 | 7 | 0.000035596707639673065 | — |
| 第二次原生点动后，新位移 | 1 | 0.000036783264560995503 | 0 |
| 第二次 STOP 后，连续静止状态 7 | 7 | 0.000050547324848335754 | 0 |

采样的 PowerState 始终为 1、两类 Error 为 0；停后静止验证各至少 100ms。
最终状态仍为 7，**不是状态 0 的交接验收**。探针正常返回后退出私有进程，未留下测试进程。
独立普通 SSH 重读日志、比较 before/after 快照一致；实体 RobotMain 仍为 PID 76071，
PC 无控制进程/8001 监听。实体 SDK、GlobalConfig 与已安装诊断库指纹仍分别为
`40c38d1a...`、`6dbf8e7d...`、`c834b048...`，没有部署或操作实体机械臂。

- 本次实际探针 SHA256：`491fe7eaca5a8ee79c4221d4ecafb9e47f6d59e6512d083cd1753e837b3c1457`。
- 隔离检查用、**未加载**的插件 SHA256：`5523034fa0edd4f253ba389f9196a19cc61f16b1c5494eb98783c81beec96bb8`。
- 探针源码 SHA256：`3315a762288237afb9dd995f2464d4437163a1b6748ab06d5685207a98b0de49`；
  teach include：`567e3d7504f9193106d16f4ce393a832ef59236b1050aab663fededaf1618d4a`。

**结论：** 真实 SDK 的“STOP 后状态 7 → 下一次原生点动成功且产生新位移”行为已动态验证，
与前述点动内部恢复的静态调用链一致。没有对恢复函数做动态拦截，也未观测瞬时状态 0，
不能把此实验描述成逐条指令的动态调用跟踪。它没有测试并发外部停止，
不证明停止来源可区分、生产自动交接可用或 ROS 新会话接管通过。

## 2026-11-12 后续只读核查：下一次点动也隐含恢复

现场确认停控、可靠支撑、退使能后，只读核验 PC 控制进程及 8001 监听均已退出，
原生 RobotMain 和 8000 状态监听也不存在。此时没有在线反馈，不能把现场退使能确认
写成软件采样验证。本轮未启动控制器、加载 SDK、使能、点动或修改部署。

再次读取 SDK，前后 SHA256 均为
`40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1`。
固定二进制只读反汇编得到一个此前未计入的调用边界：

- `robot_teach_joint`（0x167dc0）先检查 `get_robot_busy_state`；非 busy 分支在
  **0x167def 调用 `recover_robot_move_state`**，随后在 0x167e13 跳转 `axis_movej`。
  因此“没有直接调用恢复接口”不等于点动调用链没有恢复；下一次按键本身就可能恢复状态 7。
- `recover_robot_move_state` 在状态 7/0 时写回 0，并清停止 order；非上述状态也会清 order。
  不能用零错误码或一次健康采样证明该调用不会覆盖并发的停止请求。
- `robot_ok` 的 0x12fc22 分支检查 order=1；状态为 0/9 时走到 0x12fc92/0x12fc97，
  写入状态 7 并返回非正常运行。此为 SDK 内部停止分支的静态证据，
  **不是先前虚拟点动事件确实走过该指令的动态调用栈**。
- 公开头文件把 `RobotStopRun` 定义为停止后允许后续命令，把 `RobotStop` 定义为
  停止后不允许后续命令；`robot_teach_stop` 仅承诺停止示教移动及返回码，未承诺返回状态 0。
  厂商示例在 teach STOP 后继续启动项目，但不提供停止来源的原子确认或旧 writer 退出合同。

**修正结论：** 不能把 STOP 后状态 7 一概解释成 SDK 自身发生了运动错误；
它与已观察到的原生停止语义相容。当前桥仍应锁存 FAULT，因为它不能从相同的
`PowerState=1/MoveState=7/两类 Error=0` 区分自身正常 STOP、外部停止或并发安全事件。
下一次原生点动隐含恢复能解释“再次按下可继续”的一种机制，但不证明一代整条链路的实际原因，
也不授权在新桥中自动重试点动、替换 StopRun 或放行状态 7。

### 本轮离线回归

[引擎测试](jeserver_v2_test.c) 增加主动 STOP/未请求 STOP 两种来源，
以替身 jog 回调模拟“若调用就会隐式恢复”的副作用，确认：

- state=7/errors=0 时，即使按钮先于 tick 到达也不再进入 jog 回调；
- tick 锁存 `MK2_STATE_ERROR`，随后出现新鲜 state=0 反馈也不能清锁；
- 第二次点动、旧/当前代次 PC Switch、PC claim 均不能恢复或生成新代次；
- 重复 STOP/卸载不重复清理，也不额外调用 StopRun。

独立 C 普通及 ASan/UBSan 测试通过，另有 12 项首次故障和 11 项伺服观察回归通过；
最终复跑产物 `/tmp/jeserver-v2-offline.XaodMVv4`，包含非零旧代次拒绝断言。
这是**替身 SDK 回归**，不是新的真实 SDK 动作验收。
生产运行时代码及已安装插件未改变。

后续需要建立可验证的停止来源/事件顺序和厂商支持的续动合同，再决定如何处理
“本会话主动停止”和“未知/故障停止”。仅延长等待、见到 state=0 或先检查 Error=0
都不能替代该合同。未具备之前，真实自动交接与 ROS 再接管仍未完成。

## 实现

- [写线程适配器](../jeserver_v2_writer_adapter.inc) 仅由 `JE2_SIM_WRITER_EXPERIMENT=1`
  的私有仿真插件包含；默认构建仍是 UNKNOWN writer / NULL StopRun，拒绝 PC 持有时点动。
- 先用公开 `CreateSaveData1("servotest",1,100,14)` 初始化 SDK 原日志队列，再在独占 Start 区间
  捕获唯一新 `_servo_robot0` 内核任务，固定 proc 目录句柄。一次插件生命周期只允许一次 Start 尝试。
- 点动意图先隔离旧 PC 代次，再 StopRun、静止、End；End 返回不代表退出。
  必须独立确认捕获的实例消失，再累计退出后的新静止反馈至少 100ms，最后才点动。
- Owner 新增 `writer_observer_enabled`、`writer_instance_exited`、`writer_exit_time` 诊断。
  `automatic_priority_ready`、`writer_exit_verified`、`pc_reclaim_supported` 继续为 false；
  State 的旧 `ownership_handover_supported` 也保持 false，不能把实验 StopRun 冒充已验收交接合同。

## 已执行验证

### 最新真实 SDK 结果（非替身）

- 真实 SDK 初始化、虚拟使能、新版 PluginMain 正向就绪确认全部通过。
- ZMQ Switch/保持点实际接单：`owner=1`、`accepted=1`、SDK state=13。
- 捕获旧写线程 TID 17；固定 proc 实例确认退出后，再等待 **0.108026349 秒**，
  `owner=4`、`generation=2`，随后 SDK `robot_teach_joint` 实际执行。
- 虚拟关节发生真实反馈变化；三条旧 PC stop/release/坏帧全部被隔离。STOP 后失败反馈中的
  J1 为 `0.00017988202927248122 rad`，其他轴保持初始值。
- `tv33!` 调用 `robot_teach_stop` 后，反馈 `PowerState=1`、两类 Error=0、`MoveState=7`。
  引擎进入 `state_error`/FAULT，不把 7 当成空闲，不恢复、不重新点动。插件正常卸载，总体 FAIL。
- 独立普通 SSH 再读测试自己的日志并比较前后快照，结果相同；实体 RobotMain 仍为 PID 66190，
  固定 SDK/technology 和实体 Global/Device/Bus 指纹与既有记录一致。
- 本次实际插件 SHA256：`cec929d9c0c93b9c61124561a127dfed5b9fc0d6c6b354d57366e82acc07754d`；
  探针 SHA256：`b3775d9ec42f7914c5193ee16cf5c737a05096383b442bc12f56ff18478ddf0c`。
  后续源码只增加诊断/保护回归与澄清注释，不能把新编译产物冒称为上述实际执行指纹。

### STOP 的固定 SDK 实现证据

只读公开头文件及同指纹库反汇编确认：

- `robot_teach_stop`（0x167960）→ `robot_move_stop`（0x12f0e0）→
  `set_robot_move_stop`（0x133c60）→ `set_robot_order(1, robot)`。
- `RobotStopRun`（0x130160）→ `robot_stop_run_i`（0x130150）：先调用
  `robot_move_stop_run`（0x130010）等待停止并清 order，再由 `robot_stop_run_i`
  **调用 `recover_robot_move_state`**（0x133ac0），不是前一个等待函数自身调用恢复。
  后者在状态 7/0 时写回状态 0，并清停止 order。
- 因此不能把 teach STOP 随意换成 StopRun，并宣称“没有恢复”；也不能对状态 7 加白名单。
  现有显式 Servo 释放实验本来就使用公开 StopRun，其厂商内部恢复语义必须计入适用边界；
  健康采样不能使它与外部安全事件形成原子操作，**不能直接用于生产安全承诺**。
- 尚未确认一个公开、可验证且无需恢复的“连续点动停止后可续动”方案。本轮不调用私有状态写入、
  不新增自动恢复、不修改状态 7 的拒绝条件。正常停止可续动仍是明确阻塞项。

### 回归

| 范围 | 结果 / 保留产物 |
|---|---|
| 真五命名空间、真实 PluginMain/工作线程、真实命名 pthread，替身 SDK | PASS，`/tmp/jeserver-v2-sandbox.1lqkE1Mb` |
| 正常 PC→点动、STOP 取消、断连取消 | PASS，实际 TCP/ZMQ 与内核线程；旧 PC stop/release/坏帧不打断新点动 |
| End=0 但线程不退出、Start 部分失败、线程缺失、重复线程 | PASS，拒绝点动、故障清理一次；夹具仅卸载后释放残留替身线程 |
| 默认插件/运行时及原 sim/physical 编译，含 ASan/UBSan | PASS，`/tmp/jeserver-v2-plugin-tests.MMk98Wyu` |
| 默认完整插件沙箱，PC 持有时仍拒绝点动 | PASS，`/tmp/jeserver-v2-sandbox.y97V2HiM` |
| 核心/仲裁及真实内核线程普通与 sanitizer 回归 | PASS，`/tmp/jeserver-v2-offline.XfCwiolG`、`/tmp/jeserver-v2-task-watch.aNGlFXB9` |
| SDK 探针 65 项普通 + 65 项 ASan/UBSan，以及专用实验策略/CLI 测试 | PASS，`/tmp/jeserver-v2-sdk-offline.0gTnsDXS` |
| 生产启动拒绝 / 新模式实时上限和注入拒绝 | PASS，`/tmp/robotmain-v2-pro-test.T29K8O2l` |
| 实际控制器专用预检：完整五 namespace、SDK/插件指纹、RO、私有 ENI、独立命令策略 | PASS，`/tmp/jeserver-v2-sdk.DIGySyax`；**未加载 SDK**，前后进程/安装配置/启动器快照相同 |
| 最新真实 SDK 动作/插件链路 | 前半程通过；STOP 后状态 7 故障锁存，总体 FAIL/79，`/tmp/jeserver-v2-sdk.5ZngX8NA` |
| 新增第八类完整插件场景：STOP→7 后禁止第二次点动、额外 StopRun 和重复清理 | PASS，`/tmp/jeserver-v2-sandbox.4r1hwVdH`，替身 SDK/真实内核线程 |
| 最新 65 项普通+65 项 ASan/UBSan、专用策略与 STOP 状态解析器拒绝测试 | PASS，`/tmp/jeserver-v2-sdk-offline.mlpr3lFa`；构造 JSON 仅解析器夹具，不作实际 SDK 证据 |
| 最新默认插件/原 sim/physical 编译与 sanitizer，生产启动拒绝 | PASS，`/tmp/jeserver-v2-plugin-tests.C0KsmoQo`、`/tmp/robotmain-v2-pro-test.ufTOuxFn` |

实际预检前另行观察实体 RobotMain PID 66190，原入口与实体启动器指纹仍与之前一致。
没有安装、停止、重启或修改实体配置。源码只复制到既有普通用户暂存目录。

首次实际动作探针产物 `/tmp/jeserver-v2-sdk.A1iUXFkJ`，退出 79：真实 SDK 初始化和虚拟使能
成功，但探针把插件启动时的 `status=-1`（初始化中也使用此值）当作终止失败；日志随后确认
工作线程就绪并正常卸载。尚未发送 Switch/保持点/按钮，前后快照相同。
已将探针改为有界等待 `status=1` 加真实 State/Owner 确认，不把负状态当即时终止；等待条件不放宽。

## 显式入口与限制

[开发入口](../RobotMain_v2_pro)：

- `--test-handover-sandbox`：上面的替身 SDK / 实际内核任务完整插件测试。
- `--check-handover-sdk`：控制器专用预检，不加载 SDK，实时上限为 0 也可执行。
- `--test-handover-virtual`：需要实时上限至少 38，显式虚拟使能，加载真实 SDK 与实验插件；
  使用真实 ZMQ Switch/保持点，再通过 loopback 文本按钮触发接管，检查实例退出、退出后等待、
  虚拟关节实际变化、旧 PC 隔离、STOP 后静止与插件卸载。

[真实 SDK 专用探针](jeserver_v2_sdk_handover_probe.inc) 仅编入独立实验二进制，
该二进制拒绝 `--load` 等旧模式，不能冒充只读测试。独立策略只在专用临时副本独占创建，
保留所有原限位/保护字段；原 `--load` 和 writer-only 模式继续强制锁定策略。
插件本身不使能/上下电/清错，也不直接调用恢复接口；公开 StopRun 和 teach joint 的内部恢复语义见上文，
不能据此声称整个 SDK 调用链从不恢复。动作仅在已验证的私有仿真后端进行。
500ms / 0.02rad 点动硬界限、全局 30 秒超时、实时 CPU 上限均保留。
SDK 不在内部线程仍运行时 dlclose，最终退出整个私有 PID namespace。

**尚不能宣称现有示教器自动优先可用。** 当前按钮是 loopback 原型，不是完整原生 UI 协议；
原生端口重路由和启动前旁路阻断、PC 重连/代次绑定、受保护生产 supervisor 及实体验收仍未完成。
本实验不需要两台 PC 直连，也不要求 UI PC 提供源码。