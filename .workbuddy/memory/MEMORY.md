# NEXarm 工程长期约定（跨会话必读）

> **开工入口（2026-10-11 用户更新）**：先跑 `python handoff/brief.py`（共享精简摘要；同会话未变只报一行，
> 接续/压缩恢复用 `--refresh`）——**替代旧的"双文件必读"流程**。
> 需要背景时再按需读本文件 / `SESSION_CONTEXT.md` / 按 topic grep 台账；
> 本文件旧表述（COM9、150 刻度等）**一律以最新台账为准**。

## 〇、历史背景：两条工作线的由来（2026-10-08 初记；**现状以「协作约定」与 `handoff/CURRENT.md` 为准**——2026-10-11 起两端统一走 handoff/ 共享层分工协作）

| | 本条工作线（Claw / 本会话） | 另一条工作线（Codex 会话） |
|---|---|---|
| 记忆载体 | `.workbuddy/memory/*.md`（本文件） | 根目录 **`PROJECT_STATE.md`**（含 `PH-20261007-eight-point-01` 交接块，167 行） |
| 主攻方向 | 视觉路线（K230 + 视觉分支固件） | **标定 + 八点回放路线** |
| 交付/工具 | `舵机基础运行逻辑/`（协议规格 + 主机库 + 示例 + 证据） | `eight_point_mvp_20261007/`（`prepare_points.py` / `official_replay.py`） |

**已确认事实**：
- 两条线**互不引用**：`eight_point_mvp_20261007/` 与 `PROJECT_STATE.md` 中对
  `舵机基础运行逻辑`/`motion_profile`/`MOTION_LOGIC_SPEC`/`nexarm_host` 的引用数为 **0**
  （grep 全空）→ 对方没看到本线成果，各自独立重复分析过 NexArm.exe。
- 用户曾对 Codex 明确：「另一个助手的视觉路线已停止，由本对话独立负责标定与八点路线」
  → 若要接手八点，**先与用户确认归属已改**。
- Codex 那条线的实机阻塞点与我们一致：**视觉分支固件不让上位机命令驱动臂**；
  它选择用**官方 GUI 已验证可动的 `CMD51`（单关节 `<BhH>`）**做回放入口
  （依据：用户确认上位机单舵机/关节角度滑块可主动控制机械臂）。
- 八点数据：设备 NVS 中 108 字节记录（8 点 × 6 舵机 int16 + 头/CRC），
  用途命名 red_above_open → … → t0_clear_open；
  导出留档 `validation_runs/readout_20261005_180008/eight_points.json`，
  逐点数值表见 `docs/notes/八点记录读取验收_20261005.md`。

**协作约定（2026-10-11 升级为共享层）**：
- **`handoff/` 是两端唯一的共享层**：`LEDGER.jsonl` 一行一条事实、**只追加**；
  `STATE.md` 由 `render.py` 生成（**勿手改**）；`lock.py` 管串口占用；规则见 `handoff/RULES.md`。
- **分工（2026-10-11 用户更新）**：Codex=协议/代码设计、Bug 审查与验收；**Claw=现场配合 + 只读采集 + 落盘**
  （任务卡 `handoff/CLAW_CAPTURE_TASK.md`）。每点回执 ≤400 字（topic `powered-teaching`）；
  点位数据随采导出 `handoff/points_powered/`（每点只增 `point_NN.json`）。
- 私有区各自独占写：`PROJECT_STATE.md` 归 Codex、本文件/`.workbuddy/memory/` 归 Claw，
  **互不编辑对方文本**，要反驳就追加自己的块。
- **权威顺序：物理实测 > 源码推断**；仅从源码推断的结论必须标 `hypothesis`。
- **开工先跑 `python handoff/brief.py`**（见顶部「开工入口」；未变只报一行，详情按 topic 查），写文件前 `git status` 看对方是否在动。
- 任一方向对方交付物下手前，先在本文件或 `INDEX.md` 显式登记，避免重复劳动。

---

## 〇之二、K230 视觉协作机制（读厂商源码所得，2026-10-09；**未实机验证**——本机 0x5F 不应答）

- 物理：K230 是 **I2C 从机，地址 `0x5F`**，挂在 **Wire1（SDA21/SCL22）**，与电机/传送带共用总线；
  ESP32 为主机。源码：`A:/NEXarm/视觉识别源码/01 颜色分拣/Color_Sorting/ColorGrabber.cpp`。
- 通信：**双槽邮箱 v2（Mailbox v2）** —— 不是简单读写，而是共享内存式 IPC：
  magic `57 4C 4D 32`("WLM2")；HOST 槽(ESP32→K230) + DEV 槽(K230→ESP32)；
  每槽 8 字节元数据 `state/generation/frame_len/frame_xor`；状态机 EMPTY→WRITING→READY；
  generation 防重读；XOR 校验；**I2C 分块 30 字节/次**；ESP32 **10ms 轮询**。
- 报文：类型位在 byte4 高两位（`CMD=0x00 / RSP=0x40 / RPT=0x80`）。
  下行命令：`0x01 SET_MODE`、`0x10 SET_CONF_THRESH`(置信度50)、`0x13 SET_SIMPLE_RESULT`、
  `0x14 DISABLE_RUN`、`0x40 COLOR_SET_TARGET`（指定要找的颜色）。
  上行上报：`0x70 HEARTBEAT`(mode/status, READY=0x04)、`0x72 DETECT_BBOX`、
  `0x75 DETECT_COLOR`（`[组数] + 每组([颜色名]+[blob数]+blob×(cx,cy,w,h,angle))`）。
- 协作：11 状态机 `INIT→SET_MODE→WAIT_READY→CONFIG→**SEARCH**→WRIST→DOWN→GRAB→LIFT→PLACE→RELEASE→RESET`。
  **只有 SEARCH 是视觉闭环**（PD 视觉伺服），其余是**开环回放**（用录好的姿态 + X_COMP/Y_COMP/GRAB_Z 补偿）。
- 闭环参数：死区 **±15px**（释放带 25px）、D 项限幅 **40**、每 **40ms** 发一次增量、
  单次运动 **30ms**；对准后需**连续稳定 3 秒**才下压；**抓取点 = 稳定那一刻的机械臂坐标**。
- **为什么视觉模式下上位机命令被屏蔽**：ESP32 的 loop 被该状态机占住并自行 `arm.move()` 驱动臂，
  上位机同时发命令会互相打架；vendor 源码另有 teach-only 早退分支。
- 对本项目的含义：① 主线（提手/篮体八点回放）**不需要 K230**；② 该环对**延迟极敏感**（25Hz 级增量修正）
  → 这正是不能把云端 LLM 放进识别环的硬理由；③ 像素→机械臂的换算落在 `X_COMP/Y_COMP/GRAB_Z` 这类
  **标定偏移**上，即手眼标定；④ 本机 K230 现不通，该链路起不来。

---

## 一、目录与路径约定（2026-10-07 整理后）**当前根目录结构**（`A:\NEXarm\Claude`）：只保留 `README.md` / `PROJECT_STATE.md` / `INDEX.md` / `.gitignore`，
其余文件归入 6 个分类夹：`docs/`（人写文档）· `reference/`（厂商 PDF）· `scripts/`（可执行脚本）·
`logs/`（编译/烧录/哈希）· `runs/`（实验采集，按实验分子目录）· `artifacts/`（产物归档）。
分类依据与逐项归属见根目录 `INDEX.md`；全量 old→new 映射见 `_reorg/reorg_plan.txt`。

**⚠️ 绝对路径强耦合——以下路径禁止移动或改名**：
- `scripts/build/*.ps1`（**17 个全部**）内部把路径写成**绝对路径**（`A:\NEXarm\Claude\...`），
  arduino-cli 的 `--build-path` / `--libraries` 参数同理。移动任何被引用的目录，
  **必须同步改写这 17 个脚本**，否则构建断链（已用 `migrate_build_cache.py` 做过一次，流程可复用）。
- 各实验目录（`handle_teach_*`、`k230_link_probe_*`、`vision_route_*` 等）被构建脚本以绝对路径引用 → **勿移动**。
- `.workbuddy/`（记忆与工具数据）不可迁移。

**缓存目录现状**（2026-10-07 B 方案迁移后）：
| 路径 | 个数 | 大小 | 备注 |
|---|---|---|---|
| `.build-cache/.arduino-build-*` | 32 | 1.39 GB | 原在根目录，已收拢；**17 个脚本已改写指向此处** |
| `.build-cache/.factory-build-*` | 6 | 137 MB | 同上，无脚本引用 |
| `.toolchains/` | 1 | 1.57 GB | **原地不动**，不可再生（重装需联网+耗时） |
| `.gcc-temp-*/` | 31 | 0.2 MB | **原地不动**，脚本 `New-Item -Force` 自动重建（30 个为空） |

`scripts/build/*.ps1` 的**日志输出路径已改到 `logs\build\`**（2026-10-07），跑完不会再污染根目录；
更早的历史日志已归档在 `logs\build\historical\`，不会被覆盖。

## 二、机械臂操作红线

1. **力矩是运动的第一前提 —— 本机实测为 ON**（2026-10-08 现场触觉：夹爪掰不动）。
   ❌ 此前记的「所烧诊断版视觉固件 `set_torque(false)` + 必须按 KEY1」**已否证**（那是从源码推断的，
   而设备 10-07 19:11 已刷回出厂程序；用户 10-08 实测：44° 行程目视确认动作）。
2. **✅ 已验证的写路径（黄金样本 `八点推进_20261008/golden_20261008_first_motion.md`）**：
   帧 `FF FF | ID=舵机号(1~6) | LEN=10 | CMD=3 | [0x29, acc=100, pos_i16LE, 0, 0, speed_i16LE] | XOR`
   → **实测可驱动机械臂**。`pos` 是舵机原始刻度（1 tick=0.08789°），**不是角度也不是 XYZ**；
   **`ID=0xFF` 广播对 `CMD=3` 无效**；本路径用 acc+speed 调速、**无 time 参数**。
3. **验证幅度经验（2026-10-11 按台账修正）**：极小幅动作（≤数十刻度）的"没动"结论不可靠；
   但"≥150 ticks 通用铁律"**已被否证**（有限样本不可推广；ID5 同幅度下 speed200 不动、speed2000 通过，
   速度等参数同样起作用——见台账 `amplitude-threshold` refuted）。验证尽量用大幅动作 + **物理观测**，
   不为凑阈值盲目加大动作。
4. **`cmd 11` 与 `cmd 65` 同源**（实测逐字节相同），**不得**用两者一致或数值变化证明舵机真的动过；
   它们会**跟随**已执行的命令，可作为操作指示；**决定性判据仍是物理观测**。
5. **增量指令（cmd 50）不可信精度**：实测 +10→+8、+30→+40、−30→−32，且会带动未指定自由度
   （dz+10 引起 pitch 变化 3.4°）。需要精度一律用绝对坐标 `cmd 8` 闭环。
6. 完整协议与边界见 `舵机基础运行逻辑/MOTION_LOGIC_SPEC.md` 与 `motion_profile.json`。

## 三、工具链与运行环境

- 编译：`Bash` 调 `arduino-cli`，或直接跑 `scripts/build/*.ps1`（PowerShell，绝对路径）。
- 烧录：`PYTHONPATH='A:\NEXarm\Claude\.toolchains\esptool-py-4.5.1' C:/Users/1/python.exe -m esptool
  --chip esp32 --port COM11 --baud 230400 write_flash 0x10000 <bin>`（端口已由 COM9 变更；烧录前关上位机）。
- 设备串口：**COM11（CH340, 1A86:7523）@1,000,000 bps**（2026-10-10 由 COM9 变更，台账 `serial-port`）；
  串口操作（烧录/采集）前确保 **NexArm.exe 已彻底退出**（它独占串口）。
- Python 环境：`C:\Users\1\.workbuddy\binaries\python\envs\default\Scripts\python.exe`（含 pyserial）。

## 四、git 约定

- 仓库根：`A:\NEXarm\Claude`，分支 `main`。每完成一项实质工作即提交，提交信息写清"做了什么/为什么"。
- `Claude/nul` 是 Windows 保留名文件，无法删除，已由 `.git/info/exclude` 的 `/nul` 排除。
