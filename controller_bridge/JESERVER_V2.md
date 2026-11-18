# JEServer_v2：沿用一代入口的控制器侧改造

> **范围更新（2026-11-12）：** 用户先要求优先完成二代基本适配，随后明确要求继续正常交接与再接管。
> 本文及 RobotMain_v2_pro 的仲裁/接管研究已继续，但仍不作为二代基本控制的前置条件。
> 基本路径继续使用既有二代配置和受保护启动，不因本实验未验收而要求替换插件或等待完整原生 UI 接入。
> 此范围调整不授权去掉保护、自动恢复或实体部署；详见 [当前适配主线](../README.md)。

## 最新：显式启动代次/客户端会话通信实验

`JE2_SIM_SESSION_EXPERIMENT=1` 把原来只在引擎内可用的 claim/带代次命令接入实际 ZMQ 入口。
每次加载随机 boot 标识，显式新 client 申请后才接受连续序号的命令；会话实验没有旧协议回退。
`--test-session-sandbox` 已通过完整私有插件、真实通信和内核线程/替身 SDK 联测，
包括实际 DSO 重载旧帧、multipart 后续完整消息、旧目标/停止隔离及状态 7 禁止 claim。
生产能力标志仍 false，默认编译不启用；标识是公开关联值而非认证。
ROS 驱动未接线，一次性 Start 限制未放宽，第二次 claim/Switch 不是第二次 Servo 验收。
详见[新会话协议与实际覆盖边界](PC_SESSION_PROTOCOL.md)。

## 请求入口即刻锁存已观察异常

2026-11-13，修复有效点动/claim/PC 请求读到异常却只拒绝当前请求的缺口：
`observe_healthy` 在忙状态拒绝前保留异常；交接后的 `AVAILABLE` 也不跳过保护。
初始退使能未就绪和旧代次在反馈检查前的隔离仍保留。105 组瞬态反馈/入口/所有权组合
先复现旧引擎失败，再通过普通与 ASan/UBSan；默认与 writer 实验完整替身插件联测通过。
没有导入新的 SDK API、自动恢复或安装修改；本地锁存不等于原生停止事件来源可信。
只读审计确认 `get_robot_order1` 有写入副作用，也不提供源序列，不能用于诊断或交接授权。
见[审计证据、修复与剩余厂商合同](tests/SDK_HANDOVER_REVIEW.md)。

## 真实 SDK 外部/叠加停止反馈与正常停止相同

2026-11-13，在两个全新私有虚拟 SDK 实例内，分别执行一次点动后 `RobotStop`、
一次点动后自身 teach STOP 再追加 `RobotStop`，两者均通过，最终四个健康字段均为
`Power=1 / State=7 / DriverError=0 / MoveError=0`，与上轮原生停止基线相同。
外部停止后没有再点动或恢复，不宣称 SDK 会拒绝下一次点动；未测试硬件急停。
因此本地“主动 STOP”标志不能排除叠加停止，现有反馈不能作为自动续动授权。
新增 34 类替身控制流及 ASan/UBSan 回归通过；生产引擎/部署未改，仍需可信停止事件合同。
实际产物、指纹、剩余条件见[停止来源对照](tests/SDK_HANDOVER_REVIEW.md)。

## 原生下一次点动内置恢复：静态发现与动态验证

**2026-11-13 动态验证补充：** 独立真实 SDK 私有虚拟探针已通过两轮点动/STOP。
第一轮 STOP 后状态 7，第二次原生点动返回 0、进入状态 1 并出现新的虚拟位移，
再次 STOP 后仍为 7；未加载插件、未调用显式恢复，实体进程/配置快照不变。
这是原生 API 语义验证，不是自动交接通过，详见下方研究记录。

固定 SDK 的 `robot_teach_joint()` 在非 busy 分支先调用 `recover_robot_move_state()`，
再进入关节点动。故“STOP 后状态 7、再次按下继续”可能依赖下一次调用内的恢复，
并非 STOP 必须自己回到 0；状态 7 也不能单独证明 SDK 运动错误。
但桥尚不能区别正常停止与并发外部停止，仍锁存故障，不自动调用下一次点动。
新增主动/未请求 STOP 两类隐式恢复隔离回归，普通及 ASan/UBSan 通过；未加载真实 SDK，未部署。
见[调用地址、证据边界与测试记录](tests/SDK_HANDOVER_REVIEW.md)。

## 最新：真实 SDK 接管前半程通过，STOP 健康空闲验收未通过

修复探针就绪等待竞态后，真实 SDK/实际插件已完成 PC 保持点、旧写线程实例退出、退出后
108ms 等待、点动获权与实际虚拟位移，三条旧 PC 消息也被隔离。
但原生 `tv33!` STOP 返回 SDK state=7，现有保护锁存 FAULT，总体仍为 FAIL/79。
没有放行状态 7、自动恢复或修改实体配置。反汇编还确认公开 StopRun 内部含恢复步骤，
不能简单换接口来掩盖停止问题；正常停止可续动仍未解决。
已补第八类完整插件和普通/ASan/UBSan 解析器拒绝回归，生产启动仍关闭。
见[实际证据、STOP 语义及限制](tests/SDK_HANDOVER_REVIEW.md)。

## 线程退出观察已接入显式仿真插件

新增 `--test-handover-sandbox`，已通过真实 PluginMain/工作线程、五私有 namespace、
真实 Linux 写线程和替身 SDK 的七类接管/取消/失败场景；默认插件仍拒绝未知 writer 的交接。
专用真实 SDK 接管探针已编译并通过普通/ASan/UBSan 策略保护测试，控制器
`--check-handover-sdk` 也已通过（不加载 SDK，原进程/安装配置/启动器快照不变）。
后续 `--test-handover-virtual` 已实际运行，前半程通过但 STOP 验收失败，见顶部最新结论。
所有生产启动继续拒绝。详见[虚拟接管插件验证范围](tests/SDK_HANDOVER_REVIEW.md)。

## 最新：新增写线程实例证据与独立虚拟实验入口

已通过固定 SDK 只读反汇编确认 `_servo_robot0` 是实际 Linux comm，
但 Servo 线程是 detached，公开 `ThreadWait` 在此路径不执行 `pthread_join`。
新增[任务实例观察组件](jeserver_v2_task_watch.c)：启动前基线、唯一新任务捕获、
固定 proc 目录句柄、内核退出确认；缺失、歧义或错误一律 UNKNOWN。
真实 Linux 线程测试和 ASan/UBSan 已通过，包括 detached 与 64 次捕获/退出。

pro 新增 `--test-writer-exit-offline` 与 `--test-writer-exit-virtual`。
后者是**显式虚拟使能/holding Servo 实验**，不是原只读加载测试；保留完整隔离，
不加载通信插件、不启用策略、不点动、不向示教器交权。真实 SDK 新实验已三轮通过：
每轮捕获实际 live 写线程，分别执行 End 或 StopRun+End 后，独立确认该内核实例退出，
再通过退出后的新反馈健康/静止检查。最初因日志辅助线程继承同名而拒绝，现用公开
日志 API 在 Servo 前初始化原队列，保留原有唯一候选检查。产物
`/tmp/jeserver-v2-sdk.N14ydOBo`，退出 0，原实体进程/安装配置/启动器快照相同。
生产 writer 回调仍为 UNKNOWN，所有生产启动仍拒绝。
实现依据、适用前提和验证范围见[线程退出研究记录](tests/SDK_WRITER_EXIT_REVIEW.md)。

## 最新：真实 SDK 只读加载与卸载已通过（2026-11-12）

修复私有 ENI 路径后，通过 pro 在控制器的强隔离沙箱中实际运行：

- 真实 SDK 初始化 `result=0`，`simulation=1`、七轴、`power=0`、`state=0`。
- 新版 `PluginMain` 工作线程实际就绪，State/Owner 校验通过，序号到 18，命令始终禁用。
- 新版插件实际卸载成功，关节/电源/运动状态不变，错误为零；SDK 保留到整个私有进程退出。
- 测试退出码 0；原进程、安装配置和启动器快照相同。没有使能、轨迹、点动或实体操作。

完整范围、产物指纹及剩余限制见 [真实 SDK 生命周期验收记录](tests/SDK_LIFECYCLE_RESULT.md)。
这证明真实 SDK 下**禁命令插件生命周期**可行，不证明 Servo writer 退出、完整 UI 或自动接管。
`--sim/--real/--auto/--start` 继续拒绝，生产保护不变。

## 最新：修复隔离副本的 ENI 绝对路径（2026-11-12）

操作者在同一 SSH 会话中完成授权后，真实 SDK 已实际加载并执行初始化，
但返回 `-148`，**尚未加载新版插件**。失败产物为 `/tmp/jeserver-v2-sdk.FiwFVX9W`。
原因已确认：复制的 BusConfig 仍引用安装目录下的 ENI，而安装仿真目录在沙箱中正确地被隐藏；
SDK 因找不到 ENI 而无法初始化。原进程/安装配置/启动器快照保持不变。

修复只作用于独立暂存副本：

- 探针新增 `--prepare-bus`，验证源 BusConfig 中唯一、完整且已知的 ENI 引用，
   仅将其改成 `/tmp/mk2-pro/profile/eni7_sim.xml`，其余字节保持不变。
- 输出只允许规范化的专用临时目录，采用独占创建，拒绝覆盖、符号链接、歧义或未知引用；
   不修改安装目录，不把被隐藏的原目录重新暴露给沙箱。
- 加载 SDK 前另外确认 BusConfig 只引用私有 ENI，ENI 本身也须只读挂载。
- 已新增 33 项回归，共 64 项普通及 64 项 ASan/UBSan 测试通过。
   修复后的控制器只读预检通过，产物 `/tmp/jeserver-v2-sdk.HCK0YZ0I`，前后快照仍不变。

本段记录修复后的预检；后续真实 SDK 初始化/插件加载已经通过，见顶部独立验收记录。
sudo 授权必须与实际测试位于同一 SSH 会话；单独运行 sudo 验证不能保证新会话继承授权。

## 最新：真实 SDK 生命周期探针与控制器预检（2026-11-12）

新增 [真实 SDK 探针](tests/jeserver_v2_sdk_probe.c) 和
[隔离运行脚本](tests/run_jeserver_v2_sdk_probe.sh)，已接入 pro：

- `--check-plugin-sdk`：控制器上的编译、独立复制配置、锁定策略和实际 namespace 预检，
   **不加载 SDK**。控制器已实际通过一次，实时优先级上限为 0 也可执行此只读预检。
- `--test-plugin-sdk`：仅在上述隔离检查通过后加载固定指纹的真实 SDK，使用
   `simulation=true` / `plugin_list=[]` 初始化，然后单独加载新版插件，检查 `PluginMain`
   就绪及真实 State/Owner。策略强制 `allow_commands=false`，不建立 PC 命令发布者、
   不连接按钮端口；没有使能、Servo、点动、停止、恢复或上下电调用。
- `--test-plugin-sdk-offline`：64 项锁定策略、私有 ENI 路径与加载前拒绝测试，普通及 ASan/UBSan 均通过；
   另回归参数、实时上限、namespace 标记与动态加载环境的负向预检。

探针在 `dlopen` 前再次验证实际五类私有 namespace、只读根、私有共享内存、零 capabilities、
仅 loopback、隐藏实体设备和实体配置，以及 SDK/technology/新插件指纹。
配置与插件独立只读挂载；只有专用复制目录中的 app/data 可写。原进程、两套安装配置与原启动器
均做前后快照，变化只报错，不擅自停止或恢复。插件卸载后保留 SDK，最终退出整个私有 PID namespace，
不对尚有内部线程的 SDK 调用 `dlclose`。外层限时 30 秒，另限制单次连续实时 CPU 占用。

源码仅暂存于普通用户目录 `/home/robot/mk2-pro-sdk-source.HEV3iVIS`，未安装到系统启动位置。
控制器缺少完整 json-c 开发头文件，因此仅在该暂存目录复制已有完整声明，使用现有运行库，
没有安装软件包。控制器预检产物为 `/tmp/jeserver-v2-sdk.ahBreB5z`，结果为
`JE2_ISOLATION_OK`、`policy_locked=1`、原进程和配置/启动器快照不变。

最初为普通用户测试子进程设置实时上限的 `sudo -n` 被拒绝，提示需要密码；
随后改为同会话直接授权，已进入真实 SDK 初始化，新的失败与修复见顶部记录。
密码只能在终端输入，不能交给聊天。探针和 pro 不自行提权，也不降低实时检查。
`--sim/--real/--auto/--start` 继续拒绝，自动交接与完整生产协议仍未验收。

## 最新：可装载仿真插件已实现（2026-11-12，未部署）

已从纯引擎推进到实际共享库：

- [jeserver_v2_plugin.c](jeserver_v2_plugin.c) 导出 `PluginMain`、`JEServerV2Status`、
   `JEServerV2ButtonPort`，复用原插件的策略加载、配置哈希、SDK 采样和 State 序列化。
   原插件默认编译分支保留；新版只有显式选择这个编译单元才启用。
- [jeserver_v2_runtime.inc](jeserver_v2_runtime.inc) 在**唯一工作线程**中处理 ZMQ、按钮事件、
   仲裁核心及所有 SDK 变更。State 保留原格式；新增 Owner 话题公开 owner/generation、旧帧计数、
   `automatic_priority_ready=false`、`writer_exit_verified=false` 和 `pc_reclaim_supported=false`。
   已封锁的 PC 会话不再被 State 误报为 commands_enabled。
- [jeserver_v2_transport.c](jeserver_v2_transport.c) 是 `127.0.0.1:16666` 的文本按钮原型，
   支持有界拆包/粘包、STOP 优先、断连/畸形包取消、单客户端及连接代次。
   它**不是原 6666/6667/6668 全协议替代品**，没有接到另一台电脑的生产示教 UI。
- 按钮先处理、核心后 tick；事件预算耗尽则取消，防止积压中的 STOP 尚未处理就执行点动。
   点动以 1% 速度运行，并限制最多 500ms、任意轴相对启动位置位移小于 0.02rad；
   超限锁存停止，不会靠长连接或重复按键续期。它不是连续长按功能。
- 新版固定专用 profile，只接受五类私有 namespace、只读宿主根、私有共享内存、仅 loopback、
   无有效 capabilities、隐藏物理设备/实体配置以及 SDK simulation=1、1 机器人/7 轴的环境。
   宿主拒绝发生在 SDK getter/变更之前。没有实体编译覆盖或旧插件回退开关。

**尚无独立 Servo writer-exit 证据，所以实际适配器刻意不提供 stop_run 能力。**
PC 正在持有 Servo 时，示教请求在任何 SDK 变更之前被拒绝，不会假称自动让位已完成；
`ServoEnd()==0`、状态变化及内部 owns_servo 均不当作退出证明。初始 PC 会话与无 PC writer 的
空闲点动可以由这个运行时处理；让位后的无代次旧帧永久封锁，尚无重新申请传输协议。
没有使能、清错、恢复、上下电或夹爪初始化调用。

### pro 入口现在实际能做什么

- `--build-sim-plugin` 调用 [build_jeserver_v2_sim.sh](build_jeserver_v2_sim.sh)，使用显式
   `JE2_SDK_INCLUDE` 厂商头目录，在临时目录产生真正的仿真共享库、符号/依赖检查及 SHA256。
   **只编译，不加载 SDK、不安装、不启动控制器**；本机已实际执行通过。
- `--test-plugin` 调用 [tests/run_jeserver_v2_plugin_tests.sh](tests/run_jeserver_v2_plugin_tests.sh)：
   使用真实 SDK 声明和假 SDK 定义，对该共享库实际进行 `dlopen/dlsym/PluginMain/dlclose`。
   错 profile 与正确 profile 但未隔离的宿主各 32 次加载/重复入口/卸载，确认零 SDK 调用。
   它另行测试运行时 helper、真实本机 TCP/消息序列化及 ASan/UBSan，**不是启动成功验收**。
   测试拒绝预加载/动态库搜索覆盖；ROS 终端的库搜索变量只能在测试子进程中清除。
- `--test-plugin-sandbox` 运行 [完整隔离生命周期测试](tests/run_jeserver_v2_sandbox_test.sh)，
   在真实 bwrap 五类私有 namespace 中加载**相同生产源码编译的共享库**，实际通过原保护并启动
   `PluginMain` 工作线程；仍然只链接假 SDK，未篡改 OS 检查或生产保护。
   七种场景已通过：等待/执行中卸载、TCP STOP/断连，以及实际 ZMQ Switch/Joint 与 State 确认。
   验证工作线程退出、共享库卸载、清理仅一次、卸载后 SDK 调用计数不再变化及监听端口关闭。
   测试计数使用原子变量，避免测试线程与实际工作线程之间的数据竞争。
- `--test` 仍是原有完全无 SDK、无网络的引擎/保护回归。
- `--sim` / `--real` / `--auto` / `--start` **仍返回 78**：真实 SDK 只读生命周期已经通过，
   但尚未完成 guarded supervisor、物理后台端口防绕行、完整 UI 协议和 writer-exit 适配。

上述仿真插件实现轮次只在开发机编译和测试，没有 SDK 初始化或控制器操作；
更新的远端源码暂存与只读预检见顶部记录。
本节历史验证只有假 SDK；最新顶部已追加真实 SDK 禁命令运行成功记录，
仍不能表述为“实机自动接管可用”。

## RobotMain_v2_pro 独立入口（2026-11-12）

新增 [RobotMain_v2_pro](RobotMain_v2_pro)，不改原 RobotMain、RobotMain_v2、实体配置和插件。
当前是**开发入口，不是已完成的生产 supervisor**：

- `--test` 实际运行本目录 JEServer_v2 与 guarded core 的假 SDK 联测、ASan/UBSan 和原有保护回归，
   只允许普通用户；不加载 SDK、不连接网络或机械臂。
- 默认及 `--check` 输出缺失组件和当前配置的只读报告，返回 78，表示未就绪，不是通过预检。
- `--auto` / `--real` / `--sim` / `--start` 均明确拒绝；不执行旧入口，不替换插件，不停止现有进程，
   也没有可以临时跳过的 `--force` / 环境变量开关。
- [入口测试](tests/robotmain_v2_pro_test.sh) 覆盖帮助、默认、检查、各启动模式、无效参数及异目录调用。

上一轮只读查询发现实体 RobotMain 正在运行，加载的是原 `libMK2PhysicalControl.so`；没有停止它。
原 supervisor 指纹为 `afe7df171c839744eab59a7ff889a65798525cafa66f720b83487a7a810f041b`，
底层受保护启动脚本指纹为 `d61221184ea2e739b4dfb421e889605ac3e8080b1b5b4dad40186949b994e8de`。
这些是当时的观察基线，不是本轮状态复核或未来可跳过检查的永久授权。

后续真正接线时必须继承原启动锁、原安全会话退出、全部七轴退使能检查、配置/标定指纹、
总线 OP/WKC/共享内存检查、控制器优先清理及启动失败不回退语义。还须在任何替换/停止前
验证新版插件、后台示教端口不可绕行以及插件初始化就绪；不能把当前开发入口返回 78 当作
系统已通过检查，也不能仅给旧入口换名便声称使用了 JEServer_v2。

源码已普通用户暂存于控制器 `/home/robot/mk2-pro-source.NH0BMFL2`，没有放到系统启动位置。
本机入口拒绝测试、通过 pro 入口的完整离线联测均通过；控制器入口拒绝测试也通过。
控制器缺少 json-c 的 pkg-config 开发包，第一次离线编译因此失败；没有为此安装系统包。
随后显式使用此前已有的 JSON 头文件和运行库，仅通过测试专用的
`JE2_TEST_JSON_INCLUDE` / `JE2_TEST_JSON_LIBRARY` 编译覆盖参数，完整离线测试通过，
日志产物目录 `/tmp/jeserver-v2-offline.RAPvQiqU`。这些参数不影响任何启动准入，也不能解除
`NOT_READY`。再次只读复核时原启动器两份指纹及实体 RobotMain PID 均未变。

## 为什么可以改成独立插件

两套配置最终使用同一份 RobotMain 和运动 SDK，主要差异在通信插件。
因此可以做 `JEServer_v2`，在唯一现有 RobotMain 进程中替换通信插件，
不是再运行第二个硬件控制进程，也不要求示教电脑连接本机 ROS 电脑。

目标保留现有点动键、示教电脑的控制器地址和外部端口；收到原按键请求时由控制器侧让位。
不照搬一代的自动清错、恢复、上下电、夹爪初始化或忽略 `ServoJoint` 返回值。

## 本次已经写入的源码

- [jeserver_v2.h](jeserver_v2.h)：唯一串行执行域的事件接口和会话状态。
- [jeserver_v2.c](jeserver_v2.c)：实际复用现有 guarded core 的仲裁引擎，不再只是纯策略模型。
- [tests/jeserver_v2_test.c](tests/jeserver_v2_test.c)：与真实 guarded core 联测，SDK 均为假回调。
- [tests/run_jeserver_v2_offline.sh](tests/run_jeserver_v2_offline.sh)：独立离线检查，非 ROS/CMake 构建目标。

引擎之外，现已补充可编译的仿真 `PluginMain` 共享库，详见顶部最新进展。没有更换实体插件、修改控制器端口、
调整网络规则、自动打开实体交接开关或执行实体命令。

### 已实现顺序

1. 初始 PC 会话可继续使用原 `Switch` / `Joint` 主体；所有原有时间、首帧保持、限位、
   跟随误差、速度/加速度、健康和看门狗保护仍由现有核心执行。
2. 验证到现有原生关节点动请求后，先关闭旧 PC 会话准入；之后的旧 Joint、Switch false、
   Release、畸形帧和旧 claim 在进入核心或触发 SDK 清理前被拒绝。
3. 内部申请清洁 Release，调用受控 StopRun，利用严格前进的反馈采集时间验证停稳，随后 End。
   没有 handover 能力时，在任何 SDK 变更前拒绝自动让位。
4. 等待该次会话独立的 writer-exit 证据。`ServoEnd()==0`、状态不等于 13、核心
   `owns_servo=false` 都不是此证据；未知时失败锁存，仍运行时继续有界等待。
5. 退出确认后必须重新取得采集时间位于确认之后的反馈，并再次持续停稳，才将尚未松开且
   不超过 0.5 秒的点动意图执行一次。重复旧反馈不能累计静止时间。
6. 松开/断连取消尚未执行的意图；失败、过期不重放。运行中的示教停止保留责任，直到
   新鲜反馈证实停稳；故障与停止失败保留首个错误及清理错误，不盲目重复 SDK 清理。
7. 再次接受 ROS 必须重新获取新代次，随后仍需要新 Switch 和新鲜首帧保持；不恢复旧轨迹。
   硬 Switch false 保持锁存停止语义，不能借新 claim 自动清除它。

### “兼容一代”的明确边界

旧消息本身没有可靠的会话身份。因此只允许**首次会话**的无代次消息；首次让位或 Release 后
永久封锁这些旧消息，防止积压帧和迟到清理重夺控制。后续 PC 会话必须完成服务器签发/绑定
的 generation 握手，并逐事件携带正确代次。

这保留了初始命令主体，但**不是 PC 驱动完全不改就可反复自动获取的逐字节替代品**。
真实 transport 还需绑定客户端身份、发布代次与状态并同步适配 PC 驱动；generation 不是认证密钥。

文本按钮解码器只接受完整的标准关节按键包和停止包；不猜测别名、不透明放行未知操作。
文本原型现已有有界拆包/粘包、STOP 与断连接线；完整功能分类和二进制协议仍未接入。
0.5 秒待执行意图期限与最新适配器的 500ms 运行上限是两道不同保护，不能混为一谈。

## 尚不能作为生产插件启动的具体原因

1. **入口路由尚未接线。** SDK 在插件加载前已经创建原示教服务器，不能靠插件抢原端口。
   可以迁移原服务器后台端口，但 SDK 仍监听通配地址；必须在启动前封住后台外网绕行。
   仅增加高端口、不封绕行，不构成受控仲裁。原启动锁和校验不能绕过。
2. **独立 writer 退出适配尚未实测。** 当前 SDK 的 End 等待耗尽也可能返回 0；新引擎明确
   要求独立证据，不给它假 ACK。线程身份/退出的候选方案仍需隔离 SDK 验证，或使用厂商正式接口。
3. **生产接线尚未完成。** 仿真插件已将文本示教、ZMQ、状态发布和核心接到一个执行器，
   真实 SDK 禁命令启动/退出验收已通过，但仍缺完整 UI 协议、PC 新会话握手及受保护启动器。
   不能与旧插件各持一个 core 并发操作 SDK。真实故障/急停不能被旧 PC 会话隔离机制吞掉。

## 当前验证

严格 C11 编译和 fake-SDK 联测、ASan/UBSan 通过，覆盖自动 Release、退出等待、未知/超时拒绝、
退出后反馈屏障、旧帧/旧 claim、松开取消、过期不重放、新会话、硬停止锁存、SDK 失败、
重复/倒退反馈、限位、看门狗、禁用策略与文本按钮校验。
同时回归原有 12 个首发故障快照场景和 11 个伺服观察场景。

离线测试二进制不加载机器人 SDK；顶部新增真实 SDK 禁命令插件加载/卸载验收，
仍不构成完整示教 UI 兼容、运动中制动或实机接管验收。既有三组直接 API 和四组原生协议处理器实验见
[一代参考与 SDK 实验记录](LEGACY_TAKEOVER_REVIEW.md)。