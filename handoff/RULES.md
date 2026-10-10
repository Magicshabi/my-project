# 共享层写入规则（双端必读）

本目录是 Claw 与 Codex 两条工作线**唯一的共享层**。
目的：让两端的结论互相可见，同时**不产生写冲突**。

---

## 一、四条铁律

### 1. 私有区独占写，互不编辑对方文本

| 归属 | 私有记忆位置 | 谁写 |
|---|---|---|
| Claw | `.workbuddy/memory/` | 只有 Claw |
| Codex | `PROJECT_STATE.md` | 只有 Codex |

**对方的文本只读。** 要反驳、要补充，**追加自己的块**（带 `作者-日期` 标记），
**绝不修改、删除对方的字**。现有示例：`PROJECT_STATE.md` 里的 `CLAW-XREV` / `CLAW-DIAG` 两个块。

### 2. 共享区只追加

- 所有双方都要知道的结论，**追加到 `LEDGER.jsonl`**，一行一条。
- **不修改、不删除**已有行。结论被推翻时，**新追加一条 `status: "refuted"` 指回旧条目**。
- 追加式是冲突概率最低的写入方式；每人各占一行，物理上不会互相覆盖。

### 3. `STATE.md` 由脚本生成，任何人不得手改

`handoff/STATE.md` 是 `render.py` 从 LEDGER 渲染出来的**只读视图**。
手改它在下次渲染时会被覆盖 —— 所以**别改，去改 LEDGER**。

```
python handoff/render.py
```

### 4. 权威顺序：物理实测 > 源码推断

同一主题出现冲突结论时，**以带实测证据的那条为准**。
只从源码推断出来的结论，必须标 `status: "hypothesis"`，不得标 `verified`。

> 这条不是流程洁癖。2026-10-09 就发生过：Claw 从源码推断"力矩关闭、需按 KEY1"，
> 而设备当时已被刷回出厂程序 —— 该推断后来被 Codex 的实测否证。

---

## 二、LEDGER 记录格式

每行一个 JSON 对象：

```json
{"ts":"2026-10-11T00:40+08:00","author":"claw","topic":"id4-limit",
 "claim":"ID4 带电可达上限落在 3090~3234 之间","status":"verified",
 "evidence":"八点推进_20261008/id4_target_comparison_20261009.md",
 "supersedes":null,"note":"3090/2940 可动；3234 被拒绝，反馈冻结"}
```

| 字段 | 必填 | 说明 |
|---|---|---|
| `ts` | ✅ | ISO8601 带时区，如 `2026-10-11T00:40+08:00` |
| `author` | ✅ | `claw` 或 `codex` |
| `topic` | ✅ | 短横线小写标识，同一主题用**同一个** topic，便于分组 |
| `claim` | ✅ | 一句话结论，写清楚，不要"可能也许" |
| `status` | ✅ | `verified`（有实测证据）/ `hypothesis`（推断待验）/ `refuted`（已否证，保留） |
| `evidence` | ✅ | 证据路径（文件/日志/命令），便于复核 |
| `supersedes` | ⭕ | 指回被本条推翻的条目，建议格式 `author:topic:ts` |
| `note` | ⭕ | 补充说明 |

**校验**：`python handoff/render.py --check` 会检查必填字段与 JSON 合法性。

---

## 三、设备占用锁

物理设备（串口）**同一时刻只能被一方占用**。开串口前先登记：

```
python handoff/lock.py status COM11          # 查当前占用
python handoff/lock.py acquire COM11 --minutes 30 --purpose "带电可达性审查"
python handoff/lock.py release COM11
```

- 锁文件在 `handoff/locks/<设备>.lock`，含持有者、开始时间、TTL、用途。
- **TTL 过期视为自动释放**（避免一方崩溃后另一方永久阻塞）。
- 若锁被对方持有且未过期：**停手，告诉用户**，不要抢串口。

---

## 四、开工前的两个动作

1. **读 `handoff/STATE.md`** —— 30 秒知道对方最新结论，尤其是 `refuted` 部分（别重犯）
2. **`git status` + `ls` 目标目录** —— 确认对方没有正在改你准备动的文件
