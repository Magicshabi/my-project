# NEXarm 工程长期约定（跨会话必读）

## 一、目录与路径约定（2026-10-07 整理后）

**当前根目录结构**（`A:\NEXarm\Claude`）：只保留 `README.md` / `PROJECT_STATE.md` / `INDEX.md` / `.gitignore`，
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

`scripts/build/*.ps1` 运行时会把日志写回**根目录**（脚本内是绝对路径），跑完请把日志移入 `logs/build/`。

## 二、机械臂操作红线

1. **力矩是运动的第一前提**：当前所烧的**自建诊断版**视觉固件在 `setup()` 末尾执行
   `arm.set_torque(false)` + `enter_pose_calibration("startup requires manual confirmation")`，
   **必须扶臂摆位后短按 KEY1** 才写目标并开力矩；未解锁前任何主机命令只改 AT32 内部记账。
2. **`cmd 11` 与 `cmd 65` 同源**（实测逐字节相同），**不得**用两者一致来证明舵机真的动过；
   判据必须来自物理观测。
3. **增量指令（cmd 50）不可信精度**：实测 +10→+8、+30→+40、−30→−32，且会带动未指定自由度
   （dz+10 引起 pitch 变化 3.4°）。需要精度一律用绝对坐标 `cmd 8` 闭环。
4. 完整协议与边界见 `舵机基础运行逻辑/MOTION_LOGIC_SPEC.md` 与 `motion_profile.json`。

## 三、工具链与运行环境

- 编译：`Bash` 调 `arduino-cli`，或直接跑 `scripts/build/*.ps1`（PowerShell，绝对路径）。
- 烧录：`PYTHONPATH='A:\NEXarm\Claude\.toolchains\esptool-py-4.5.1' C:/Users/1/python.exe -m esptool
  --chip esp32 --port COM9 --baud 230400 write_flash 0x10000 <bin>`。
- 设备串口：**COM9（CH340, 1A86:7523）@1,000,000 bps**；烧录前必须**关闭 NexArm.exe**（它独占串口）。
- Python 环境：`C:\Users\1\.workbuddy\binaries\python\envs\default\Scripts\python.exe`（含 pyserial）。

## 四、git 约定

- 仓库根：`A:\NEXarm\Claude`，分支 `main`。每完成一项实质工作即提交，提交信息写清"做了什么/为什么"。
- `Claude/nul` 是 Windows 保留名文件，无法删除，已由 `.git/info/exclude` 的 `/nul` 排除。
