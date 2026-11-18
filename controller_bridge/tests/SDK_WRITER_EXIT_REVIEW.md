# SDK Servo 线程实例退出研究

## 最新结果：真实 SDK 隔离虚拟实验三轮通过

2026-11-12，公开日志 API 提前初始化后，实际固定 SDK 实验退出 0。
产物：`/tmp/jeserver-v2-sdk.N14ydOBo`。本次**没有加载通信插件，也没有点动交权**。

| epoch | 实际捕获的沙箱 TID | 释放请求 | 独立内核退出确认 |
|---|---:|---|---|
| 1 | 12 | ServoEnd | 通过 |
| 2 | 13 | RobotStopRun + ServoEnd | 通过 |
| 3 | 14 | RobotStopRun + ServoEnd | 通过 |

每轮先通过固定任务目录句柄观测 RUNNING，holding 期间持续核查该实例、SDK 健康与
位置边界；End 返回单独记录为 `NOT_EXIT_PROOF=1`，再由 proc 内核证据确认 EXITED。
随后 12 次、每次间隔 10ms 的新反馈校验 state=0、power=1、无驱动/运动错误、
全部关节相对起始保持位置偏差不超过 `1e-5 rad`，三轮均通过。
同一个数字 FD 49 被逐轮正常关闭/重新分配，不代表复用同一任务实例。

最终输出为 `result=PASS epochs=3 KERNEL_INSTANCE_EXIT=1 NO_HANDOVER=1 PRODUCTION_READY=0`。
普通 SSH 独立重读日志并比较前后进程、安装配置和启动器快照，均相同；原实体
RobotMain PID 66190 未重启。原受保护启动器 SHA 仍为
`afe7df171c839744eab59a7ff889a65798525cafa66f720b83487a7a810f041b`。

探针二进制 SHA-256：
`1d05955b55813c2b711dacb76f3680ffdd0ffd119931af88893af42a1cc155d2`。
编译出的仿真插件 SHA-256（本实验没有加载它）：
`495aa38e60b14e18af27d35aeeef63a436bd21a87b49884b93b6971cacd4f2ed`。

这证明**固定 SDK、无外部客户端、独占 Start、日志预初始化的隔离虚拟条件下**可以独立
观测本次写线程实例退出，不是生产协议兼容、物理安全认证或自动接管验收。
没有测试真实 SDK 的异常 Start、End 超时、故障或 TID 强制回收；相应离线拒绝规则不变。
SDK 的缺字段/无网卡授权警告仍存在，未绕过授权或据此接受实体配置。

## 已核实的固定 SDK 实现

2026-11-12 通过只读 SSH 核对控制器 SDK，SHA-256 为
`40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1`。
没有在实体进程调用线程或运动函数。

- `_servo`（0x122d10）构造 `_servo_robot%d`，调用 `setCurrentThreadName`，
  再调用 `ThreadFunctionStart → servo_execution → ThreadFunctionEnd → FreeThreadData`。
- `setCurrentThreadName`（0x5d3e0）实际调用 `prctl(15, name)`，即 Linux `PR_SET_NAME`。
  因此机器人 0 的 `_servo_robot0` 是实际 Linux comm 候选，不仅是内部登记名。
  但 SDK 没有检查这次改名的返回值，观测不到仍必须拒绝。
- `ServoStart`（0x123029）把 detached_flag=1 传给 `CreateThread`。
  `CreateThread` 对非零标记设置 `PTHREAD_CREATE_DETACHED`。
- 公开头文件的 `ThreadWait(const char *)` 只承诺对 JOINABLE 线程有效。
  实际 `ThreadWait → WaitThread`（0x6bfb0），仅 detached_flag=0 分支调用
  `pthread_join`；detached 分支轮询 `getThreadFunctionState`，找不到记录也返回 0。
  不使用它确认 Servo 写线程退出，更不调用 `ThreadDataFree` 或 `CancelThread`。
- `ThreadFunctionEnd` 写状态发生在 `_servo` 真正返回之前。
  `ServoEnd` 成功、状态 0、登记消失或 Linux 名称消失都不是已捕获实例退出证明。

### 实测发现的同名辅助线程及初始化要求

诊断复测 `/tmp/jeserver-v2-sdk.OIRs3TeA` 在 Start 后发现 TID 11 和 12 都叫
`_servo_robot0`，观察器以 `duplicate_candidate` 拒绝，退出 79，原快照不变。
没有选择其中一个“看起来像写线程”的 TID。

随后只读查到：`servo_execution` 的 0x122ab6 调用
`RSaveDataFast1("servotest", 1, 100, 14, data)`；首次调用经
`CreateSaveData1 → SaveDataListInsert2 → _init_save_data` 在 0x54f5a 创建
`_saveDataToFile` detached 线程，继承调用线程 comm。因此 Linux 名称本身确实不唯一。

新实验在 **任何 Servo 之前**，由非 Servo 主线程调用公开 Tool/saveData.h 的
`CreateSaveData1("servotest", 1, 100, 14)`，使用完全相同的队列参数，提前初始化
原本会惰性创建的日志资源。失败就拒绝实验，不改 SDK、不注入调用、不改线程名、
不删队列、不忽略重复候选，也不更改观察器的唯一性检查。日志保留到私有进程退出。
该初始化顺序是新实验的明确前提；未经这个前提验证的原始惰性启动流程仍应拒绝。

## 新增的独立 Linux 观察组件

[任务观察器](../jeserver_v2_task_watch.c) 仅负责特定内核任务实例：

1. 在一次 Start 尝试之前记录全部任务 TID 基线，并要求候选名称不存在。
2. Start 后必须观测到唯一的新 TID 候选；旧任务改名、未捕获即退出、重复候选、
   扫描错误、超出 1024 个任务均锁存 UNKNOWN。
3. 固定真实 procfs 中该任务的目录 FD，并确认其 stat 当时可读。
4. 后续只相对这个 FD 读取 stat，不根据名称或数字路径重新查找。
   任务改名仍是 RUNNING；该实例消失后内核返回 ENOENT/ESRCH 才确认 EXITED。
   EBADF、权限和其它错误都不当退出；FD 保留到本次观察结束。
5. 不接受自定义 proc 根、不自动复位 UNKNOWN、不同 epoch 不共享捕获句柄。

**限制：**名称只能识别候选，组件不是通用安全认证或 SDK 所有权确认。
要把候选与该次 ServoStart 对应，仍须固定 SDK 和由唯一执行器独占线程创建区间，
没有其它未受控 Start/同名线程来源。当前只在独立、无插件、无外部客户端的隔离
实验中建立这个前提。生产原生端口绕过尚未封堵，所以**没有接入生产 writer 回调**。
组件也不宣称只要这个实例退出就证明没有其它控制写者。

## 离线验证

[真实内核任务测试](jeserver_v2_task_watch_test.c) 包含 detached 线程（无 join）、
未观测、提前退出、同名多线程、改名后仍活跃、旧任务改名冒充、同名新实例、
坏 FD、非法 epoch/name，以及 64 个独立捕获/退出周期。
普通严格 C 编译和 ASan/UBSan 已通过；未加载 SDK、不联网。
同名新实例测试不等于已强制制造内核数字 TID 回收。

测试还实际发现：`pthread_join` 的返回可能先于 proc 任务销毁。
夹具现在有界等待内核清理，观察器没有为此放宽规则。
原 JEServer_v2 旧会话屏蔽、退出后新反馈屏障、取消/故障保护回归亦已通过。

## 显式真实 SDK 虚拟实验入口

[pro 入口](../RobotMain_v2_pro) 新增两个独立测试动作：

- `--test-writer-exit-offline`：上述 SDK 无关真实任务测试。
- `--test-writer-exit-virtual`：固定真实 SDK，仅在完整五命名空间隔离及原有哈希、
  私有 ENI、只读配置、零能力、硬件隐藏、loopback 检查成功后进行虚拟使能与
  holding Servo。不加载通信插件，不建立按钮/PC 客户端，不修改 locked 策略。

[实验实现](jeserver_v2_sdk_writer_probe.inc) 连续三轮验证：先捕获 live 实例，
发送不改变位置的 holding 目标，执行 End（后两轮先 StopRun），独立等待该实例
内核退出，再验证退出后健康/静止反馈。End 返回与内核证据分别记录；未知、超时、
位置漂移、任何 SDK 错误均失败并终止整个私有进程，不向点动交权。
原 `--test-plugin-sdk` 仍执行原有无使能/无运动生命周期路径。

**历史：新虚拟 Servo 实验前两次第一轮捕获失败，退出 79。**
真实 SDK 初始化、虚拟使能和 `ServoStart` 均返回 0；两次均未取得 live 实例捕获证据，
没有进入退出确认或点动交权。失败产物为 `/tmp/jeserver-v2-sdk.GoNkPuBi`，
实体进程/安装配置/启动器前后快照相同。已增加阶段、拒绝原因、errno 和只读任务列表
诊断，仍保留 UNKNOWN 锁存。随后用公开日志 API 提前初始化并复测成功，见顶部结果；
未把历史失败改判成功，也未放宽重复候选检查。
已有的只读加载 PASS 仍只对应 [原生命周期记录](SDK_LIFECYCLE_RESULT.md)。
新探针已在控制器完成严格编译与实际五命名空间预检（未加载 SDK），产物
`/tmp/jeserver-v2-sdk.qRZxgO5D`；锁定策略、私有 ENI 和前后实体进程/配置快照检查通过。
新增虚拟模式的主机拒绝测试后，策略/隔离回归共 65 项，普通和 ASan/UBSan 均通过。
所有生产启动动作继续拒绝，不更换实体插件、配置、进程或网络规则。