# 共享台账摘要 · 10-11 13:35 | 42 条 | 有效 23 | 否证 7
> 只读本文件即可开工；细节用主题名 grep `handoff/LEDGER.jsonl`。全量详版：`render.py --full`


| topic | 结论 | 状态 |
|---|---|---|
| `cmd11-cmd65` | cmd11 与 cmd65 的 6 个 pulse 逐字节同源（含… | 实测 |
| `context-reading` | 开工改用短摘要与指纹去重，不再每轮整读工程记忆和历史。 | 实测 |
| `division-of-work` | 用户确认Claw负责常规采集，Codex负责代码设计、Bug审查和… | 实测 |
| `emergency-stop` | 物理急停效果尚未单独验证，不能按已通过处理。 | 待验 |
| `gui-limit-not-protection` | GUI范围不能证明主动可达或避障；真实硬限位仍未读出。 | 待验 |
| `gui-range-constants` | 反编译常量与用户范围数值相符，但不能认证硬限位或无碰撞。 | 待验 |
| `host-joint-limits` | 六轴用户范围已接入主机检查，未写入舵机内部限位。 | 实测 |
| `id4-target-comparison` | ID4历史对照：2940/3090实体移动并回位，3234那次未动。 | 实测 |
| `id4-target-feedback` | 第1点ID4回包3084已获用户认可，3413是GUI可编辑目标。 | 实测 |
| `k230-link` | K230 为 I2C 从机地址 0x5F，挂 Wire1(SDA2… | 待验 |
| `limit-readback` | 现有查询没有获得可信的舵机内部限位读回值。 | 实测 |
| `old-points-review` | 旧八点ID4的1/2/3/4/5/8点超过软件上限3413。 | 实测 |
| `path-validation` | 新路线点间避障与到达精度待验，未取得路径通过结论。 | 待验 |
| `points-export` | 带电示教点位开始随采随写入共享目录 handoff/points_… | 实测 |
| `powered-teaching` | 第5点 t0_above_grip 已采集：刻度 [2425,16… | 实测 |
| `project-state-archive` | 历史归档到根目录文件，主状态保持3000字符以内。 | 实测 |
| `serial-port` | 机械臂串口已由 COM9 变为 COM11（CH340, VID:… | 实测 |
| `servo-addressing` | 原始舵机写帧的 ID 必须用舵机号；ID=0xFF 广播对 CMD… | 实测 |
| `shared-layer-adoption` | Codex开工入口已接入共享摘要与工程约定，增量读取不改完整摘要。 | 实测 |
| `six-axis-validation` | 六轴逐轴运动及回位均获用户目视确认，无异常。 | 实测 |
| `smooth-replay` | 连续平滑、多轴协调与低速带载完整搬运仍待验证。 | 待验 |
| `tooling-capture` | 点位采集工具为 八点推进_20261008/capture_joi… | 实测 |
| `write-path` | 主机可用原始舵机写帧驱动机械臂：ID=舵机号(1~6), CMD=… | 实测 |

**已否证（勿重犯）**：`amplitude-threshold`、`eight-points-id4`、`gui-limits-source`、`id4-limit`、`reachability-model`、`torque-key1`、`torque-off-gravity`

证据与备注查阅：`grep <topic> handoff/LEDGER.jsonl`
