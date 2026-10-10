# 共享台账摘要 · 10-11 00:48 | 14 条 | 有效 12 | 否证 2
> 只读本文件即可开工；细节用主题名 grep `handoff/LEDGER.jsonl`。全量详版：`render.py --full`


| topic | 结论 | 状态 |
|---|---|---|
| `amplitude-threshold` | 验证幅度铁律：任何“是否动了”的实验，幅度须 >= +-150 t… | 实测 |
| `cmd11-cmd65` | cmd11 与 cmd65 的 6 个 pulse 逐字节同源（含… | 实测 |
| `eight-points-id4` | 旧八点执行目标的 ID4 依次为 3448/3447/3448/3… | 实测 |
| `gui-limit-not-protection` | ID4 的 GUI 上限 3413 高于实测可达上限（约 3090… | 实测 |
| `gui-limits-source` | 用户提供的 6 组关节“限位”（ID1 341~3755 / ID… | 实测 |
| `id4-limit` | ID4 带电可达上限落在 3090~3234 之间：目标 2940… | 实测 |
| `k230-link` | K230 为 I2C 从机地址 0x5F，挂 Wire1(SDA2… | 待验 |
| `reachability-model` | 手摆录点=几何可达（力矩关，手能推到机械止挡）；带电执行=力学可达… | 实测 |
| `serial-port` | 机械臂串口已由 COM9 变为 COM11（CH340, VID:… | 实测 |
| `servo-addressing` | 原始舵机写帧的 ID 必须用舵机号；ID=0xFF 广播对 CMD… | 实测 |
| `tooling-capture` | 点位采集工具为 八点推进_20261008/capture_joi… | 实测 |
| `write-path` | 主机可用原始舵机写帧驱动机械臂：ID=舵机号(1~6), CMD=… | 实测 |

**已否证（勿重犯）**：`torque-key1`、`torque-off-gravity`

证据与备注查阅：`grep <topic> handoff/LEDGER.jsonl`
