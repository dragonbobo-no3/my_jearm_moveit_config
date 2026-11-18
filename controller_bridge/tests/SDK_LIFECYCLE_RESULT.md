# JEServer_v2 真实 SDK 只读生命周期验证

日期：2026-11-12。结论：**PASS，仅限强隔离 SDK 仿真、禁命令、无使能/运动的加载与卸载。**

## 被验证路径

[RobotMain_v2_pro](../RobotMain_v2_pro) 的 `--test-plugin-sdk` →
[私有沙箱脚本](run_jeserver_v2_sdk_probe.sh) →
[真实 SDK 探针](jeserver_v2_sdk_probe.c) → 实际 SDK 初始化 → 实际新版 `PluginMain`。

不是假 SDK，也不是仅编译或仅宿主拒绝测试。原安装插件、系统启动入口和原 RobotMain 未被替换或停止。

## 结果

| 检查项 | 实际结果 |
| --- | --- |
| OS 隔离 | 五类私有 namespace、只读根、私有共享内存、capabilities=0、仅 loopback |
| 原配置/设备 | 原实体与仿真 profile 隐藏，物理设备不可见 |
| 策略 | `allow_commands=false`，不建立 PC 命令发布者，不连接按钮端口 |
| SDK 初始化 | `result=0` |
| SDK 模型 | `simulation=1`、七轴 |
| 初始电源/运动状态 | `power=0`、`state=0` |
| 插件就绪 | `verified=1`，State、Owner 均收到，State 序号到 18 |
| 控制准入 | `commands_enabled=0`，零 accepted joints |
| 插件卸载 | `plugin_unloaded=1`，不再映射该共享库 |
| SDK 清理边界 | 不卸载仍有内部线程的 SDK；退出整个私有 PID namespace |
| 末尾反馈检查 | 电源/运动状态不变、错误为零、关节变化不超过 `1e-8 rad` |
| 外层结果 | `result=PASS`，退出码 0 |
| 宿主快照 | 原进程列表、两套安装配置和启动器指纹前后逐字节相同 |

旧版结果行中的 `state=1 owner=1` 是**两个话题已接收**的布尔值，不是 SDK MoveState 或仲裁所有者。
验证器要求 Owner 内容中的所有者为 AVAILABLE；SDK MoveState 为 0。

## 证据位置与指纹

控制器保留产物目录：`/tmp/jeserver-v2-sdk.Nb3UmHWD`。
其中含 probe.log、SHA256SUMS、前后进程与配置指纹快照；之后已用普通用户只读复核两组快照完全一致。

| 产物 | SHA256 |
| --- | --- |
| 仿真插件共享库 | `495aa38e60b14e18af27d35aeeef63a436bd21a87b49884b93b6971cacd4f2ed` |
| SDK 探针可执行文件 | `8ee9a470db790bdda3ab9574f0ad010d0e5a778bc16d4a6f95b7771af115a942` |
| 实际 RobotControl SDK | `40c38d1a762523414df0a984542bf1b83f7ea2dd2a4110dbf449d469c00a2bc1` |
| 实际 Technology 库 | `cb854d244ca3835cce6e992792d229f414bb821edd7f01a4342b21826c451227` |

复核时原实体 RobotMain PID 为 66190；这是该次观察，不是未来运行状态的保证。
原受保护入口 SHA256 仍为 `afe7df171c839744eab59a7ff889a65798525cafa66f720b83487a7a810f041b`，
原 profile 启动脚本仍为 `d61221184ea2e739b4dfb421e889605ac3e8080b1b5b4dad40186949b994e8de`。

## 已修复的前置失败

首次实际初始化返回 `-148`，原因是复制的 BusConfig 引用被隐藏的安装目录 ENI。
修复只在临时副本内独占创建 BusConfig，精确重定位唯一 ENI 引用，保留其余字节。
原目录仍隐藏，配置与 ENI 仍只读挂载；64 项普通及 64 项 ASan/UBSan 回归通过。

## 未通过此测试证明的内容

- 没有启动 Servo writer，所以**不证明 ServoEnd 或 writer 退出屏障可靠**。
- 没有点动、ROS 轨迹、运动中制动或示教器自动接管测试。
- 没有验证完整生产 UI 协议、原后台端口防绕行、PC 新会话握手或受保护生产 supervisor。
- `automatic_priority_ready=false`、`writer_exit_verified=false` 保持不变；实际启动开关仍拒绝。
- SDK 仍报告部分配置字段缺失及无网卡环境下的 30 分钟未授权提示。未绕过授权或忽略真实故障，
  本次短时只读成功不能作为生产配置或许可证验收。