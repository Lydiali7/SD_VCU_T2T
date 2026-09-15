# 重载虚拟编组 T2T communication & simulation description

latest update：2026.9.15

## 1. 项目目标与当前定位

本项目面向重载列车虚拟编组（virtual coupling）中的车车通信（T2T）,跟驰控制和通信安全监督研究。项目以五列车物理仿真为基础，逐步把原先直接使用仿真变量的实现，迁移为可连接实物 VCU、未来 ATP/TCMS/MVB/CAN 数据源和对端 OBU 的工程化边界。

当前成果是一个**研究与 HIL 原型**，不是通过认证的 ATP、TCMS 或安全通信产品。所有阈值、动作策略和通信帧仍须经过真实 ICD 映射、危害分析、参数论证、边界工况验证及相应的安全认证，才能用于实车安全功能。

## 2. 当前系统结构

```text
SIM/HIL 物理世界
        |
        v
world_server
  五列车动力学、线路位置场景、WorldFeedbackPacket
        |
        v
vcu_node（当前 DUT: Train 3）
  控制、旧有 NetworkHealth、AtpSnapshot、T2T 收发
        |
        +-------------------------------+
        |                               |
        v                               v
本车 AtpSnapshot                  对端 T2TAtpSnapshotFrame
        |                               |
        v                               v
T2TFrameHeader + CRC              CommHealthState
                                        |
                                        v
                           CommSafetySupervisor（shadow）
                                        |
                                        v
                           CommSafetyAction（shadow）
                                        |
                                        v
                            当前不写入 cur_f、EB 或 ATP trip

PC / mock_peer（模拟 Train 2 / 第二台 OBU）
  接收 Train 3 snapshot，发送 Train 2 snapshot，回发 PeerHealthPacket
```

在双机 HIL 中，`world_server` 可在上位机运行，`vcu_node` 可在实物机箱运行；`mock_peer` 在 PC 上充当第二台 OBU。当前主链路使用 UDP，`PeerHealthPacket` 是对端观测的辅助交叉检查，而不是本车安全决策的第一依据。

## 3. 已完成能力

### 3.1 基础仿真与单节点 HIL

- `world_server <-> vcu_node` UDP 闭环已稳定跑通。
- 实物样机可运行当前 VCU 节点。
- SIM 可按线路位置图触发隧道/盲跑、NLOS 曲线、坡道、曲线、解编和新编队等场景。
- 已验证基础故障注入、blind run 和 fail-safe 的状态触发。
- 本车状态已统一经 `AtpSnapshot` 边界输出。

### 3.2 ATP/TCMS/MVB/CAN 工程化数据边界

- 已建立 `ATPAdapter` 和 `AtpSnapshot` 归一化边界。
- SIM/HIL fallback 到 `AtpSnapshot` 的路径已实现并有单元测试。
- 定点数编码、validity mask、snapshot CRC 和 frame CRC 已验证。
- CAN、MVB 和 ATP gateway 的 HAL/接入位置已预留。

尚未完成的是依据真实 ATP、TCMS、MVB、CAN ICD 对每一个字段、有效位、时间基准、故障码和安全等级进行正式映射。当前 fallback 映射不能被当作真实车载数据接口。

### 3.3 T2T snapshot 工程化通信

- 已实现 `T2TFrameHeader + AtpSnapshot + T2TAtpSnapshotFrame + CRC`。
- 已验证 `vcu_node -> PC mock_peer` 的接收、解析、CRC、序列号和 AoI 统计。
- 已升级为双向对称 peer snapshot HIL：VCU 发送本车 Train 3 状态，同时接收 mock peer 发送的 Train 2 状态。
- 接收端可检查帧长度、magic、版本、CRC、发送者、目标、可选源 IP、duplicate、replay、序列缺口和 timestamp age。

### 3.4 mock_peer 对端仿真与 PeerHealth

- `mock_peer` 可模拟第二台 OBU：接收 Train 3 snapshot，并周期发送 Train 2 snapshot。
- 可统计 AoI、sequence gap、missing snapshot、当前/最大 burst loss 和 invalid packet。
- 可周期回发 `PeerHealthPacket`。
- 已在 50% snapshot loss 注入下验证 gap/missing 检测：丢失表现为序列不连续，而不是误判为 CRC 坏包。
- `mock_peer -> vcu_node` 的 `PeerHealthPacket` 反向链路已打通；VCU 可观测 PeerHealth RxAoI、Seq、Gap、Invalid、WrongSource 等字段。

`PeerHealthPacket` 的正确定位是**secondary cross-check / diagnostics**。本车对本地接收到的、通过全部校验的 peer snapshot 的观测，才是通信安全判断的 primary evidence。

### 3.5 CommHealthState：Phase 2A

`CommHealthState` 已统一聚合以下接收事实：

- 最新有效 peer snapshot 的 freshness/AoI；
- sequence、累计 missing、当前 burst 和最大 burst；
- CRC、错误发送者、错误源 IP、错误目标、duplicate、replay、陈旧 timestamp 等验证统计；
- peer health 的辅助交叉检查信息。

关键原则是：只有**通过全部验证的 peer snapshot**才刷新 `last_valid_rx`。即使 socket 持续收到坏包、错误发送者包或 replay 包，安全层看到的 AoI 仍会恶化，不会把“网络很忙”错误解释为“通信可用”。

### 3.6 CommSafetySupervisor：Phase 2B

已实现下列 shadow 通信安全状态机：

```text
NORMAL
  -> DEGRADED
  -> BLIND_RUN
  -> FAIL_SAFE_LOCK
```

- 入口由本地有效 snapshot AoI 阈值驱动。
- 恢复采用 AoI 回落加连续 good snapshot 的滞回，避免状态在边界反复跳变。
- `FAIL_SAFE_LOCK` 已锁存；通信恢复后不会自动退回 `DEGRADED` 或 `NORMAL`。
- boundary、recovery 和 latch 单元测试均已通过。

当前研究/shadow 默认阈值为：`DEGRADED=300 ms`、`BLIND_RUN=1000 ms`、`FAIL_SAFE_LOCK=5000 ms`。这些不是认证参数；它们必须由列车动力学、制动曲线、定位不确定度、无线链路预算和危害分析共同论证。

### 3.7 CommSafetyAction：Phase 2C-1

已实现 shadow action matrix：

| CommSafetyState | Shadow 动作 |
|---|---|
| `NORMAL` | 正常牵引、允许协同跟驰 |
| `DEGRADED` | 限制牵引比例、增加不确定性 |
| `BLIND_RUN` | 禁止正牵引、退出实时协同、请求盲跑处理 |
| `FAIL_SAFE_LOCK` | 禁止牵引、请求 fail-safe brake |

当前该矩阵仅打印并写入 telemetry。它**不会**修改 `cur_f`、实际 EB、ATP trip、旧有 `NetworkHealth`、不确定性缓冲或现有 fail-safe 控制路径。

## 4. 已完成的 HIL Shadow 验证

| 场景 | 已观测结果 | 判定 |
|---|---|---|
| Clean baseline | `CommSafety=NORMAL`，`CommAction=NORMAL` | 通过 |
| 50% snapshot loss | 正确出现 seq gap/missing，未把丢失伪装成 invalid CRC 包 | 通过 |
| 3 s application pause | `NORMAL -> DEGRADED -> BLIND_RUN -> DEGRADED -> NORMAL` | 通过 |
| 6 s application pause | `NORMAL -> DEGRADED -> BLIND_RUN -> FAIL_SAFE_LOCK` | 通过 |
| 6 s 后恢复通信 | 新 snapshot 再次有效、AoI 回落，但状态保持 `FAIL_SAFE_LATCHED` | 通过 |

3 s 和 6 s 试验采用对同一 `mock_peer` 进程执行 `SIGSTOP` / `SIGCONT` 的方式，以保持 sequence 连续性。它暂停 snapshot 和 PeerHealth 一并发送，验证的是对端应用暂停/链路中断等价场景；它不是严格的“仅 snapshot 单链路 blackout”试验。

在 6 s 试验中，现场日志记录到 AoI 约 `5004.5 ms` 时提出 `FAIL_SAFE_LOCK`，通信恢复且有效 snapshot 的 RxAoI 回落后，仍保持 `FAIL_SAFE_LATCHED`。这验证了状态机锁存语义。

注意：`[TELEMETRY] State:<n>` 仍可能显示旧的 `NetworkHealth` / `WorldFeedbackPacket` 状态；它不是新的 `CommSafetyState`。应以 `[COMM_SAFETY]` 和 `[COMM_ACTION]` 日志或对应 CSV 字段判读本阶段 shadow 结果。

## 5. 进度估计

下表是当前原型的粗略工程成熟度，不代表安全完整性等级或认证成熟度：

| 工作项 | 当前进度 |
|---|---:|
| 基础仿真与单节点 HIL | 100% |
| 工程化 AtpSnapshot 边界 | 约 90% |
| T2T 双向通信框架 | 约 90% |
| 通信健康观测 | 100% |
| Shadow 通信安全状态机 | 100% |
| Shadow 动作策略 | 100% |
| 实物 Shadow HIL 验证 | 约 80-90% |
| 实际控制接入 | 约 0-20% |
| 真实 ATP/MVB/CAN 数据接入 | 约 20-30% |

## 6. 当前限制与不能误解的结论

- 活跃 `vcu_node` 目前仍以 Train 3 DUT 为中心，不是完整的两台可自主切换角色的 OBU 控制器。
- 对端重启后 sequence 从零开始时，当前 receiver 会按 replay/out-of-order 规则拒绝数据；尚未设计 boot/session epoch 和安全重建握手。
- 尚未完成可配置的“仅 peer snapshot blackout、PeerHealth 仍发送”的确定性故障注入。
- `CommSafetyAction` 仍是 shadow，不具有制动执行权限。
- 不存在真实 ATP/TCMS/MVB/CAN ICD 映射、设备诊断或认证安全通信栈。
- UDP + CRC 对研究/HIL 很有用，但不等同于符合 IEC 62280 等要求的端到端安全通信。

因此，当前阶段可以准确表述为：**完成了相对完整的 T2T 通信工程化边界、通信健康监督及 Shadow 安全策略验证。**

## 7. Future work

### Phase 2C-2：最小化控制接入

先只接入风险较低的动作：把 `traction_permitted` 和 `traction_limit_ratio` 接入实际 force command 路径，限制或禁止**正牵引**。应保持现有制动、EB 和 ATP trip 路径独立，并增加回归测试和明确的 enable 开关。不要一次把 blind-run、额外不确定性和 fail-safe brake 全部接入。

### 通信故障试验完善

在 `mock_peer` 加入可选、默认关闭的确定性故障注入：仅停止 snapshot 一段时间、保持 PeerHealth；以及延迟、jitter、随机 loss、burst loss。随后补充 clean、3 s、6 s、50% loss、delay+jitter 的可重复证据和 CSV 分析。

### 会话与对端重启处理

引入 session/boot epoch、明确的重启检测、重建条件和审计日志。在安全协议设计审查后，允许对端重启后的受控重新加入，而不是让 sequence reset 永久被视为 replay。

### 真实数据源接入

获得 ATP/TCMS/MVB/CAN ICD 后，再实现 `build_snapshot_from_atp_gateway(...)` 及相应 CAN/MVB 映射。应逐字段确认量纲、符号、有效性、时间戳来源、失效语义、更新周期和安全责任边界。

## 8. 关键实现位置

| 文件 | 作用 |
|---|---|
| `include/network_proto.hpp` | T2T frame、AtpSnapshot、CRC 和帧验证 |
| `include/comm_health.hpp` | 通信健康数据契约 |
| `include/comm_safety_supervisor.hpp` | 通信安全状态机接口 |
| `include/comm_safety_action.hpp` | shadow 动作接口 |
| `src/comm_safety_supervisor.cpp` | AoI 阈值、恢复滞回、锁存实现 |
| `src/comm_safety_action.cpp` | 状态-动作矩阵实现 |
| `src/vcu_node.cpp` | VCU 收发、健康聚合、shadow 日志 |
| `tools/mock_peer.cpp` | 对端 OBU 模拟、状态回发、故障注入 |
| `tests/test_comm_safety_supervisor.cpp` | 状态机单元测试 |
| `tests/test_comm_safety_action.cpp` | 动作矩阵单元测试 |
