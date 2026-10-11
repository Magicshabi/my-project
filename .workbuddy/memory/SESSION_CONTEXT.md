# Claw 会话上下文（压缩版 · 2026-10-11 13:46）

> 用途：新会话开工前 **60 秒读完**，替代翻整段对话历史。
> 开工入口（2026-10-11 起）：先跑 `python handoff/brief.py`；需要背景时再来本文件 / 按 topic 查台账。

---

## 一、项目与分工

- **项目**：NEXarm 六轴机械臂（ESP32 + AT32 + 6×HX-30HM 舵机），场地 `A:\NEXarm\Claude`
- **分工（2026-10-11 用户更新）**：**Claw**（本线：现场配合 + 只读采集 + 落盘；任务卡 `handoff/CLAW_CAPTURE_TASK.md`）｜**Codex**（协议/代码设计、Bug 审查与验收）；开工入口改为 `python handoff/brief.py`
- **主线目标**：固定红篮从上方夹提手 → 搬至固定 T0；**先保证放置正确，视觉后置**
- **共享层 `handoff/`** 已建成且被 Codex 接入（`AGENTS.md` 是双方入口规则）

## 二、本线已完成（可直接复用，勿重做）

| 成果 | 位置 |
|---|---|
| 运动逻辑规格（触发条件 / 状态更新 / 每步流程 / 边界与终止） | `舵机基础运行逻辑/MOTION_LOGIC_SPEC.md` |
| 机器可读规格（帧规则 / 16 条命令载荷 / 实测值 / 反模式） | `舵机基础运行逻辑/motion_profile.json` |
| 主机库 + 34 项离线自检 | `舵机基础运行逻辑/nexarm/`、`tests/test_codec.py` |
| **★ 已验证写路径（黄金样本）** | `八点推进_20261008/golden_20261008_first_motion.md` |
| exe 逆向全套（837 模块解析） | `nexarm_exe_re_20261007/` |
| 共享台账 / 摘要 / 设备锁 | `handoff/{LEDGER.jsonl, STATE.md, lock.py}`；开工摘要 `brief.py` |
| 点位共享导出（每点只增） | `handoff/points_powered/`（第1/2点；schema `nexarm.shared-point/1`） |

## 三、最关键的三条技术事实

1. **已验证写路径**
   `FF FF | ID=舵机号(1~6) | LEN=10 | CMD=3 | [41, acc=100, pos_i16LE, 0, 0, speed_i16LE] | XOR`
   `pos` 是**舵机原始刻度**（1 tick=0.08789°），不是角度也不是 XYZ；**`ID=0xFF` 无效**；本路径**无 time 参数**
2. **判定纪律**：`cmd11` 与 `cmd65` **同源**（逐字节相同），**不能证明舵机真的动了**；判据必须是**物理观测**
3. **端口**：机械臂现为 **COM11**（CH340，`1A86:7523`）；**写死 COM9 会失败**

## 四、我犯过的错（勿重犯）

- ❌「所烧为诊断版固件、力矩关闭、须按 KEY1」→ 仅从**源码快照**推断；设备当日已被刷回出厂程序
- ❌「ID4 偏差是力矩未开 / 重力掉落」→ 被实测否证（力矩开着且能自保持）
- ❌ 极小幅度（≤50 刻度）的「没动」结论不可靠（低于肉眼阈值；且与速度等参数有关——见台账 `amplitude-threshold` 修正）

> **教训**：能一次小步实测解决的问题，不要用推理链下结论；**源码快照 ≠ 在跑的二进制**。
> 全部已否证条目见 `handoff/STATE.md` 的「已否证（勿重犯）」一行。

## 五、当前进度与下一步（以 `brief.py` 输出与台账为准）

- 新路线**带电示教 8/8 已采完（complete=true，13:45）**：全部刻度见 `handoff/points_powered/point_01~08.json` 与数据文件（**以 points 数组为准**）；汇总条目见台账 topic `powered-teaching`。
- 数据：`八点推进_20261008/data/eight_joints_powered_20261009.json`（**以 points 数组为准**）
- ✅ **共享导出启用（2026-10-11）**：`handoff/points_powered/point_NN.json`（每点只增，schema `nexarm.shared-point/1`）；**第 1~8 点全部导出**；台账 `powered-teaching` 回执+汇总已提交（当前 49 条，含 Codex 条目）。
- **采集阶段完成（2026-10-11 13:45）**：8 点汇总已按任务卡提交台账（供 Codex 代码/路径审查）。Codex 侧接下来做协议适配/代码/路径方案；现场验证阶梯（空载逐段→急停→低速带载，不跳级）在其后。
- 待命事项：若某点需重采（用户重摆位），流程同前（`--point N`；点3/7 有 ≤12 刻度硬约束）；采集前先探测串口空闲、取锁、备份、采后导出+回执+render+commit。
- **Codex 下午进展（13:54–15:12，未提交）**：离线审查通过；空载回位+分段试验推进至 5→6；最后 P5→P6 于 ~15:12 因串口拒绝访问中断；**成果未 git 提交、未完整入台账**（Claw 未触碰）。
- **用户拟以 Claude 接替 Codex**；交接提示词已产出（对话，2026-10-11 17:25）；遗留收尾方式待用户决定。
- **尚未验证**：整段搬运、物理急停、连续平滑 → 都是 `hypothesis`，不要标成功

## 六、高频命令

```bash
python handoff/brief.py                   # ★开工入口（2026-10-11 起，替代双文件必读）
python handoff/brief.py --refresh         # 接续/压缩恢复时强制展示
python handoff/render.py                  # 刷共享摘要
python handoff/render.py --check          # 校验台账
python handoff/render.py --since <ISO>    # 只看增量
python handoff/lock.py status COM11       # 设备占用
rg "<topic>" handoff/LEDGER.jsonl         # 按主题查细节（勿整读）
```

## 七、边界

- `.workbuddy/memory/`、`舵机基础运行逻辑/` 是 **Claw 私有**；`PROJECT_STATE.md` 是 **Codex 私有**
- 要改对方的东西，**先在 `handoff/LEDGER.jsonl` 追加一条说明**
- 台账格式：`{"ts","author","topic","claim","status","evidence","supersedes","note"}`
- `handoff/STATE.md` 是生成物，**任何人不手改**
