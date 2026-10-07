# 目录说明（分类依据与索引）

本文件说明 `A:\NEXarm\Claude` 的目录划分理由、每类用途与包含文件，
供人和 agent 快速定位，避免全目录扫描。

整理日期：2026-10-07　整理方式：纯重命名（git 全部识别为 rename，可完整还原）
全量映射表：`_reorg/reorg_plan.txt`（276 条 old→new）　机器可读：`_reorg/reorg_manifest.json`

---

## 1. 划分依据（三条硬原则）

1. **按"谁会产生它"分，不按文件后缀分。**
   编译产生日志、运行产生采集、人写产生文档、外部带来参考资料 —— 四类来源的行为完全不同：
   日志和采集是**只增不改的存档**，脚本是**会再执行**的，文档是**供人读**的。
2. **放进目录的前提是该目录能被安全移动。**
   实测 `scripts/build/*.ps1` 内部全是**绝对路径**（`A:\NEXarm\Claude\...`），
   因此 **所有既有目录一律原地不动** —— 一旦把 `handle_teach_20260920/`、
   `.toolchains/`、`.arduino-build-*/` 挪位置，所有构建脚本立刻断链。
   所以本次只整理**根目录散落的文件**。
3. **缓存目录不参与整理。**
   `.toolchains/`(1.6G)、`.arduino-build-*/`(~1.5G)、`.factory-build-*/`、`.gcc-temp-*/`
   是可由脚本重建的缓存，被 `.gitignore` 排除，且被绝对路径引用。搬它们只有成本没有收益。

---

## 2. 分类总表

| 目录 | 划分依据（匹配规则） | 文件数 | 说明 |
|---|---|---|---|
| `docs/` | `*.md`、`*_review*.json` | 16 | 人写的文档：总结、方案、复盘、代码审查 |
| `docs/notes/` | `*.md` 且非 `r*.md` | 12 | 中文复盘/方案/手册 |
| `docs/reviews/` | `^r[0-9]*.md`、`*_review*.json` | 4 | 代码审查与独立复核记录 |
| `reference/` | `*.pdf` | 7 | **外部**参考资料（厂商教程），单列以免与自产文档混扫 |
| `scripts/build/` | `^build_*.ps1` | 17 | Arduino 构建脚本（内部全绝对路径，移动后仍可运行） |
| `scripts/capture/` | `^capture_*.py` | 13 | 串口采集脚本（输出写在脚本同级目录） |
| `scripts/test/` | `^test_*.py` | 7 | 离线自检脚本 |
| `scripts/tools/` | 其余 `*.py`、`*lock*.ps1`、`capture_factory_*.ps1` | 9 | 一次性工具与排障脚本 |
| `logs/build/` | `^build_*.log` | 32 | 编译日志 |
| `logs/flash/` | `*_flash.log` | 16 | 烧录日志 |
| `logs/manifest/` | `*_hashes.json` | 14 | 烧录哈希清单（校验产物） |
| `runs/` | `*.bin`、`*.json`、`*.log`、`*.csv` | 135 | 每次实验的采集三元组，按实验分 15 个子目录 |
| `artifacts/` | `factory_*.bin/csv`、`*.diff`、`*.o`、`^_r7_check*.txt` | 10 | 固件产物、差异文件、临时检查文本 |
| `_reorg/` | 本次操作自身记录 | 2 | 映射表与清单，**还原依据** |

### runs/ 的 15 个实验子目录

| 子目录 | 文件数 | 来源实验 |
|---|---|---|
| `readonly/` | 33 | 只读前后对照（含 20260922~20261006 多轮） |
| `wrapped_move/` | 30 | 包裹式移动探针（boot+run 多轮） |
| `id6_torque/` | 13 | 6 号舵机力矩探针 |
| `single_joint/` | 13 | 单关节执行验证 |
| `teach_record_r6/` | 9 | 示教记录 r6 |
| `execution_preflight/` | 6 | 执行前检查 |
| `read_diagnostic/` | 6 | 只读诊断 |
| `match_safe/` | 5 | 匹配安全版 r2/r3 |
| `handle_teach_r4/` | 4 | 提手示教 r4 |
| `teach_record_r7/`、`teach_record_r7k2/` | 各 3 | 示教记录 r7 系列 |
| `readonly_goal/`、`readonly_oled/`、`readonly_r1/` | 各 3 | 只读探针分轮 |
| `misc/` | 1 | 未归入具体实验的采集 |

---

## 3. 未参与搬迁的项目（及原因）

| 项目 | 类型 | 保持原地的原因 |
|---|---|---|
| `README.md`、`PROJECT_STATE.md`、`.gitignore` | 根文件 | 项目入口，agent 第一眼应看到 |
| `.toolchains/`(1.6G)、`.arduino-build-*/`(~1.5G)、`.factory-build-*/`、`.gcc-temp-*/`、`.test-output-r7/` | 缓存 | 可重建、被绝对路径引用、体积大 |
| 33 个实验目录（`handle_teach_*`、`k230_link_probe_20261007` 等） | 源码/实验 | 内部源码已被 `.ps1` 的绝对路径引用 |
| `_pdf_pages/` | 图片 | 被 `舵机基础运行逻辑/DECODE_REPORT.md` 引用 |
| `舵机基础运行逻辑/` | 交付物 | 已自成体系（含自己的 README 与规格） |
| `.workbuddy/` | 工具数据 | 记忆与配置，**不可搬迁** |
| `nul` | 特殊 | Windows 保留名，无法删除，已由 `.git/info/exclude` 排除 |

---

## 4. 给 agent 的读取建议（省 token 的关键）

| 你的任务 | 直接读 | 不要扫 |
|---|---|---|
| 让机械臂动 / 复现运动逻辑 | `舵机基础运行逻辑/MOTION_LOGIC_SPEC.md`、`motion_profile.json` | 不要翻 `runs/`（135 个历史采集） |
| 查某次实验当时到底发生了什么 | `runs/<实验名>/`（三元组 `.log` 可直读文字） | 不要翻 `logs/build/` |
| 找编译/烧录问题 | `logs/build/`、`logs/flash/` | — |
| 找厂商教程原文 | `reference/*.pdf` | — |
| 看当前进度与待办 | `PROJECT_STATE.md` | — |
| 追溯某文件被搬到哪 | `_reorg/reorg_plan.txt` | — |

**约定**：`runs/` 与 `logs/` 是**只增不改的存档**，不要在里面生成新文件；
新实验请建 `runs/<新实验名>/` 或在对应实验目录内新建。
