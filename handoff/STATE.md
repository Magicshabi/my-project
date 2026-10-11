# 共享台账摘要 · 10-11 20:50 | 75 条 | 有效 39 | 否证 7
> 只读本文件即可开工；细节用主题名 grep `handoff/LEDGER.jsonl`。全量详版：`render.py --full`


| topic | 结论 | 状态 |
|---|---|---|
| `battery-model-geometry` | STEP离线实测为有效实体，外形70×80×80毫米，横梁中部宽2… | 实测 |
| `cmd11-cmd65` | cmd11 与 cmd65 的 6 个 pulse 逐字节同源（含… | 实测 |
| `context-reading` | 开工改用短摘要与指纹去重，不再每轮整读工程记忆和历史。 | 实测 |
| `division-of-work` | 用户确认Claw负责常规采集，Codex负责代码设计、Bug审查和… | 实测 |
| `emergency-stop` | 物理急停效果尚未单独验证，不能按已通过处理。 | 待验 |
| `empty-cycle` | 用户确认底座回转后接续至P1，ID3反馈2564对目标2586相差… | 实测 |
| `empty-trial-tolerance` | 空载单段试验临时采用六轴20刻度到位标准，原10刻度超时日志保留；… | 实测 |
| `gui-limit-not-protection` | GUI范围不能证明主动可达或避障；真实硬限位仍未读出。 | 待验 |
| `gui-range-constants` | 反编译常量与用户范围数值相符，但不能认证硬限位或无碰撞。 | 待验 |
| `handoff-claude` | Claude接替Codex角色：协议/代码设计、Bug审查、路径方… | 实测 |
| `host-joint-limits` | 六轴用户范围已接入主机检查，未写入舵机内部限位。 | 实测 |
| `id2-feedback` | 18:52用户断电重启后六轴均有保持力；18:55回读ID2=22… | 实测 |
| `id4-target-comparison` | ID4历史对照：2940/3090实体移动并回位，3234那次未动。 | 实测 |
| `id4-target-feedback` | 第1点ID4回包3084已获用户认可，3413是GUI可编辑目标。 | 实测 |
| `intentional-tilt` | 用户确认第4点夹持稳定，倾斜是为跨越邻篮主动选择，保留原姿态。 | 实测 |
| `k230-link` | K230 为 I2C 从机地址 0x5F，挂 Wire1(SDA2… | 待验 |
| `limit-readback` | 现有查询没有获得可信的舵机内部限位读回值。 | 实测 |
| `old-points-review` | 旧八点ID4的1/2/3/4/5/8点超过软件上限3413。 | 实测 |
| `path-validation` | 新路线点间避障与到达精度待验，未取得路径通过结论。 | 待验 |
| `points-export` | 带电示教点位开始随采随写入共享目录 handoff/points_… | 实测 |
| `powered-route-review` | 新八点离线审查通过：八份导出一致，64组采样最大波动1刻度，目标均… | 实测 |
| `powered-teaching` | ★ 新路线八点带电示教采集全部完成（8/8，complete=tr… | 实测 |
| `project-state-archive` | 历史归档到根目录文件，主状态保持3000字符以内。 | 实测 |
| `recover-to-p2` | restore_torque已发CMD61开力矩且各轴读数变化≤4… | 实测 |
| `replay-entry` | 回放入口改为明确端口、SHA和单个正向相邻段，增加限定试验计划及设… | 实测 |
| `replay-observe-wait` | 已修正：reposition_empty与新路点工具先关运动闸门和… | 实测 |
| `replay-p6-interrupt` | 用户说明15:12关闭了Codex，与拒绝访问时间吻合；机械臂其后… | 实测 |
| `replay-setup` | 本轮P5→6→7→8空载分段运行完成，三段均获用户实体正确、停稳无… | 实测 |
| `replay-start-mismatch` | 首次新路线1→2空载试验因实时反馈仍为第8点而终止，运动命令0条，… | 实测 |
| `return-p5` | 用户调整姿态后，Codex完成空载回第5点；用户确认实际到位、停稳… | 实测 |
| `scene-obstacles` | 用户确认四篮相邻，目标旁三个篮体为障碍，间距估计约10毫米。 | 实测 |
| `serial-port` | 机械臂串口已由 COM9 变为 COM11（CH340, VID:… | 实测 |
| `servo-addressing` | 原始舵机写帧的 ID 必须用舵机号；ID=0xFF 广播对 CMD… | 实测 |
| `shared-layer-adoption` | Codex开工入口已接入共享摘要与工程约定，增量读取不改完整摘要。 | 实测 |
| `six-axis-validation` | 六轴逐轴运动及回位均获用户目视确认，无异常。 | 实测 |
| `smooth-replay` | smooth_route新增start_mode=home：每圈先… | 实测 |
| `technical-plan` | 技术方案从4968精简到2170字符，原文逐字节归档并保留SHA；… | 实测 |
| `tooling-capture` | 点位采集工具为 八点推进_20261008/capture_joi… | 实测 |
| `write-path` | 主机可用原始舵机写帧驱动机械臂：ID=舵机号(1~6), CMD=… | 实测 |

**已否证（勿重犯）**：`amplitude-threshold`、`eight-points-id4`、`gui-limits-source`、`id4-limit`、`reachability-model`、`torque-key1`、`torque-off-gravity`

证据与备注查阅：`grep <topic> handoff/LEDGER.jsonl`
