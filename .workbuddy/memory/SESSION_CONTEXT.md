# Claw 会话上下文（压缩版 · 2026-10-11 13:41）

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

- 新路线**带电示教 6/8 点已采**：点1「提手上方张开」/ 点2「下降到提手两侧，未夹紧」/ 点3「夹紧提手」（[1616,1545,2625,3087,2678,1993]）/ 点4「提起」（[1615,1542,2612,1981,2673,1999]）/ 点5「转运至 T0 上方」（[2425,1680,2431,3083,2485,2020]）/ 点6「下降落地」（[2407,1607,2430,3087,2454,2020]）
- 数据：`八点推进_20261008/data/eight_joints_powered_20261009.json`（**以 points 数组为准**）
- ✅ **共享导出启用（2026-10-11）**：`handoff/points_powered/point_NN.json`（每点只增，schema `nexarm.shared-point/1`）；第 1~6 点已导出；台账 `powered-teaching` 回执随点追加（43 条）。
- **下一步**：等用户摆好**第 7 点（松爪：落稳后张开夹爪，仅 ID6 动作）**→ 同一套流程（换 `--point 7`）：
  ```
  python handoff/lock.py acquire COM11 --minutes 30 --purpose "第7点带电采集"
  python 八点推进_20261008/capture_joint_points.py capture --port COM11 --point 7 --powered-sliders-confirmed --observed "<现场确认>"
  python handoff/lock.py release COM11
  # 采完：导出 point_07.json → 台账回执（≤400字）→ render.py → commit
  ```
- 采集注意：**点7 与点3 一样有硬约束——ID1~5 变化须 ≤12 刻度**，只动 ID6；张开前确认篮子已落稳；串口未被占用即可采集。
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
