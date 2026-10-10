> **历史通知，已被2026-10-11最新分工与精简读取协议替代。** 当前入口为AGENTS.md和`python handoff/brief.py`；不要再次执行下文整读记忆、归档或回填的一次性步骤。原文保留查证。

# 给 Codex 的指令（合并版 · 2026-10-11）—— 直接复制粘贴

> 用途：把横线之间的内容整段发给 Codex。

---

【Codex 指令 2026-10-11】共享层接入 + `PROJECT_STATE.md` 归档（两件事）

**一、接入共享层 `handoff/`（此后每次开工照做，约 30 秒）**

1. 开工先读这两个文件，**够了**：
   - `handoff/STATE.md` —— 共享台账摘要（**约 1k 字**，脚本生成，勿手改）
   - `.workbuddy/memory/MEMORY.md` —— 工程约定与红线
   - 尤其看 `STATE.md` 里的「**已否证（勿重犯）**」一行
2. 新结论**追加**到 `handoff/LEDGER.jsonl`（**只追加，不修改不删除**），每行一个 JSON：
   ```json
   {"ts":"2026-10-11T09:00+08:00","author":"codex","topic":"<小写短横线>",
    "claim":"一句话结论","status":"verified","evidence":"证据路径",
    "supersedes":null,"note":"可空"}
   ```
   - `status`：`verified`（有实测证据）/ `hypothesis`（推断待验）/ `refuted`（已否证，**保留不删**）
   - 推翻旧结论时：新追加一条 `refuted` 指回旧条目（`supersedes` 字段），**不要删旧条目**
   - 追加后跑 `python handoff/render.py` 刷新摘要；校验 `python handoff/render.py --check`
3. **省 token 读取纪律**（写入 `handoff/RULES.md` 第五节）：
   - ❌ **不要整读** `PROJECT_STATE.md`、`LEDGER.jsonl`、往期日志
   - ✅ 需要细节时按主题 `grep <topic> handoff/LEDGER.jsonl`
   - 增量读取：`python handoff/render.py --since <ISO时间>`
4. 规则全文见 `handoff/RULES.md`（四条铁律：私有区独占写 / 共享区只追加 / 视图勿手改 / **物理实测 > 源码推断**）

**二、本次一次性清理：归档 `PROJECT_STATE.md`（36,158 字 → 目标 3,000 字以内）**

这是当前最大的 token 消耗源（约 2.5 万 token/次），请执行：

1. **先 git commit 一次**，留下还原点。
2. 新建 `PROJECT_STATE_ARCHIVE.md`，把下列内容**整段移入**（**移动，不删除**）：
   - 所有**已过时**的历史状态段落（10-07 及更早的描述、已被后续结论取代的部分）
   - 我加的四个通知块 `CLAW-XREV` / `CLAW-DIAG` / `CLAW-HANDOFF`（一次性通知，已完成使命）
3. `PROJECT_STATE.md` 只保留：
   - **当前状态**（一节，最新交接块 `NEXARM-HANDOFF` 的最新一条）
   - **待办 / 未决问题**
   - **指针**：指向 `handoff/STATE.md`、`handoff/LEDGER.jsonl`、`PROJECT_STATE_ARCHIVE.md`
4. 移入归档的段落中，**仍然有效的结论**请条目化进 `handoff/LEDGER.jsonl`
   （这样平时不用读归档文件）。
5. 完成后再报一次字数，确认 ≤ 3,000 字。

**三、开串口前登记设备锁（防两端抢 COM11）**

```
python handoff/lock.py status  COM11
python handoff/lock.py acquire COM11 --minutes 30 --purpose "<用途>"
python handoff/lock.py release COM11
```

若锁被 `claw` 持有且未过期：**停手并告知用户，不要抢串口**。TTL 过期视为自动释放。

**四、两条现场提醒**

- 机械臂串口**已由 COM9 变为 COM11**（CH340，`1A86:7523`）。
  `capture_joint_points.py` 的 `--port` 默认仍是 COM9，须显式传 `--port COM11`，否则连不上。
- ID4 的 GUI 上限 3413 **高于**实测可达上限（约 3090~3234）。
  **GUI 数值范围不能当保护用**，按它规划点位会撞墙。

**五、回填请求**

把你已确认的关键结论（六轴逐轴验证、带电示教进展、限位采集、路径检查、急停验证状态等）
按上面的格式补进 `LEDGER.jsonl`，并跑一次 `render.py`。

**六、边界（我不会越界，请你也一样）**

- `PROJECT_STATE.md` 是你的私有记忆，**我只追加带 `CLAW-` 前缀的块，从不改你的原文**。
- `.workbuddy/memory/`、`舵机基础运行逻辑/`（含我写的 `MOTION_LOGIC_SPEC.md`/`motion_profile.json`）是 Claw 侧，
  改动前请先在 LEDGER 追加一条说明。
- 背景：此前两端**互不可见** —— `eight_point_mvp_20261007/` 与 `PROJECT_STATE.md` 对 Claw 侧交付物的引用数为 **0**，
  且两端各自独立反编译过一遍 `NexArm.exe`。共享层就是为了消除这类重复劳动。
