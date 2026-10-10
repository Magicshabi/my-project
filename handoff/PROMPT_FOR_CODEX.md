# 给 Codex 的接入提示词（直接复制粘贴）

> 用法：把下面横线之间的内容整段发给 Codex。

---

【项目协作更新 2026-10-11】项目根目录 `A:\NEXarm\Claude` 新增共享层 `handoff/`，请按以下方式接入。

**1. 开工前先读（约 30 秒）**

- `handoff/STATE.md` —— 共享台账的人读视图（**由脚本生成，请勿手改**）
- `handoff/RULES.md` —— 写入规则
- 特别看 `STATE.md` 第二节「**已否证的结论**」，避免重犯

**2. 有新结论请追加到 `handoff/LEDGER.jsonl`**

**只追加，不修改、不删除**已有行。每行一个 JSON：

```json
{"ts":"2026-10-11T09:00+08:00","author":"codex","topic":"id4-limit",
 "claim":"一句话结论","status":"verified","evidence":"证据路径",
 "supersedes":null,"note":"补充说明"}
```

- `status` 三档：`verified`（有实测证据）/ `hypothesis`（推断待验）/ `refuted`（已否证，**保留不删**）
- 被推翻的旧结论：**新追加一条 `refuted` 并指回旧条目**，不要删
- 追加后跑 `python handoff/render.py` 刷新视图；校验用 `python handoff/render.py --check`

**3. `PROJECT_STATE.md` 不要再堆叠新块**

它正在从「状态文件」退化成「日志」（已有 3 个块、539+ 行）。
它继续作为**你的私有记忆**与「当前状态」权威；**历史结论请迁移到 LEDGER**。

**4. 开串口前先登记设备锁**

```
python handoff/lock.py status  COM11
python handoff/lock.py acquire COM11 --minutes 30 --purpose "<用途>"
python handoff/lock.py release COM11
```

若锁被 `claw` 持有且未过期：**停手并告知用户，不要抢串口。** TTL 过期视为自动释放。

**5. 两条与当前现场直接相关的提醒**

- 机械臂串口**已由 COM9 变为 COM11**（CH340，VID:PID `1A86:7523`）。
  `capture_joint_points.py` 的 `--port` 默认值**仍是 COM9**，须显式传 `--port COM11`，否则连不上。
- ID4 的 GUI 上限 3413 **高于**实测可达上限（约 3090~3234）。
  **GUI 的数值范围不能当保护用**，按它规划点位会撞墙。

**6. 回填请求**

请把你这两天已确认的关键结论（六轴逐轴验证、带电示教进展、限位采集、路径检查结论、
急停验证状态等）按上述格式补进 LEDGER，并跑一次 `render.py`。
我在下一次协作时会读到。

---

## 附：为什么要这样做（背景）

两条工作线此前**互不可见**：`eight_point_mvp_20261007/` 与 `PROJECT_STATE.md` 对
Claw 侧交付物（`舵机基础运行逻辑` / `motion_profile` / `MOTION_LOGIC_SPEC`）的引用数为 **0**，
两端各自独立反编译过一遍 `NexArm.exe`。

共享层的目的就是消除这类重复劳动与结论矛盾，同时**不引入写冲突**：
私有区各自独占写、共享区只追加、人读视图由脚本生成（无人手改）。
