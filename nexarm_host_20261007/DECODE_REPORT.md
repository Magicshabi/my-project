# NexArm.exe 解码报告

对象：`A:\NEXarm\NexArm.exe`（66,840,654 字节，赛题上位机）
目的：搞清楚"上位机到底怎么把臂驱动起来的"，以便写出等价的、可复用的运动控制逻辑。
性质：对**本人设备**自带软件做互操作性分析。只读取、只分析，未修改、未重打包、未绕过任何授权。

---

## 1. 结论摘要

1. `NexArm.exe` 是 **PyInstaller one-file 打包的 Python 3.8 + PyQt5 应用**，不是原生程序。
   它的全部 Python 字节码（**837 个模块**）都在包内，可以完整还原出结构与常量。
2. 上位机与机械臂只有**一条**控制通路：**USB 串口，1,000,000 bps，帧格式 `FF FF ID LEN CMD ARGS XOR`**。
3. 真正让臂动起来的是 `CMD_COORDINATE_SET(8)` 与 `CMD_ARM_MOVE_INC(50)` 两条命令，
   载荷 `'<hhhhhhH'`，**IK 不在上位机做**，由下位机 AT32 完成。
4. 由此推出可复用结构（见 `nexarm/`）：帧层 → 命令层 → 传输层 → 编排层 → 安全层。

---

## 2. 方法链路（可复现）

| 步骤 | 输出 | 工具 |
|---|---|---|
| 1. 判形态 | PE x64 / GUI 子系统；命中 `PyInstaller` `_MEIPASS` `PyQt5` `serial.serialutil` `python38.dll` | `pyi_extract.py` |
| 2. 解 CArchive | cookie `MEI\x0c\x0b\x0a\x0b\x0e` @ `0x3FBE7F6`，archive 起点 `0x49800`，`pyvers=308`，条目 **293** 个 | `pyi_extract.py` |
| 3. 取 PYZ | `PYZ.pyz` = 4,078,866 B，头 `PYZ\0` + py3.8 magic `55 0d 0d 0a` + TOC@4,041,644 | `pyz_extract.py` |
| 4. 解 TOC | TOC 是 marshal 列表 `[(名字, (是否包, 偏移, 长度)), …]`，**837 个模块** | `marshal38.py` |
| 5. 还原模块 | 逐个 zlib 解压 + 手写 3.8 marshal 读码对象 → 模块名 / 文件路径 / 每函数 `co_consts` `co_names` | `dump_pyz.py` |
| 6. 萃取协议 | 按模块统计引用的 `CMD_*`；抽取全部 struct 格式串及其所在函数 | `cmd_usage.py` `fmt_usage.py` |

```
python pyi_extract.py  "A:\NEXarm\NexArm.exe" <out> --only-code
python pyz_extract.py  "A:\NEXarm\NexArm.exe" <tmp>      # 模块清单
python dump_pyz.py     "A:\NEXarm\NexArm.exe" report.txt # 837 模块常量/名字表
python cmd_usage.py    "A:\NEXarm\NexArm.exe"            # 命令号使用映射
python fmt_usage.py    "A:\NEXarm\NexArm.exe" nexarm_qt.ui.
```

关键难点与解法：本机只有 Python 3.13，**无法加载 3.8 的 marshal 字节流**（code object 布局在
3.11 变了）。因此手写了 `marshal38.py`——3.8 的 wire format 是稳定的，且 3.8 字节码
除 `EXTENDED_ARG` 外没有变长指令，线性遍历即可精确还原码对象。**837/837 模块解析成功，0 失败。**

---

## 3. 上位机结构（解码所得）

```
nexarm_qt
├─ constants            全部命令号、舵机寄存器号
├─ comm_manager         串口/WiFi 传输、收包状态机、回包分发、连接后读偏差
├─ styles / translations
└─ ui
   ├─ main_window        主窗口、设备连接
   ├─ coord_tab          坐标控制（步进/速度、方向轮、3D 视图、全部回中）
   ├─ dpad_widget        底盘方向盘（4dir 履带 / 6dir 麦轮）
   ├─ servo_tab          舵机与动作组（力矩、动作组下载/运行）
   ├─ servo_advanced_tab 偏差 / PID / 过载 / 波特率 / 最大力矩 / 角度限位 / ID / 模式
   ├─ peripheral_tab     电机 / 传送带 / 步进 / ESP-NOW
   ├─ teach_tab          示教编辑（本机 + 同步器）
   ├─ system_tab         版本 / 电压 / 限位 / 全局加速度 / 运动学 / 底盘 / 恢复出厂
   ├─ ai_tab             视觉玩法（颜色/人脸/手势/AprilTag/垃圾分类/大模型）
   ├─ ai_calibration_widget  视觉偏移标定
   ├─ arm_3d_widget      3D 姿态与直接下发坐标
   └─ urdf_parser        STL/URDF 模型加载（`<I` 二进制读）
```

## 4. 从 exe 里读出来的关键实现

**传输与收包**（`comm_manager.CommManager`）
- `serial.Serial(port, 1000000, timeout=0.02, write_timeout=0.2)`；WiFi 走 `socket` + 8080 端口。
- `send_packet(id, cmd, args)`：`length = 2 + len(args)`，校验 `(~sum(bytes(id..args))) & 0xFF`。
- `process_buffer()`：`FF FF` → ID → LEN → CMD → DATA → 校验，LEN 语义 = 数据长 + 2。
- `handle_packet()` 按 `cmd` 分发到 20 多个 Qt 信号；接受 `id ∈ {0xFF, 0x5A}`（0x5A 是 AT32）。
- 连上后立刻 `_read_all_offsets()`：对舵机 1~6 逐个发 `CMD_GET_POS_OFFSET(58)`，间隔 50 ms。

**坐标运动**（`ui.coord_tab.CoordTab`）
- `send_coordinate_move` / `adjust_coord` / `arm_3d_widget._send_coordinate` 都用 **`'<hhhhhhH'`**：
  `(pitch×10, x, y, z, roll, claw, time_ms)`，14 字节。
- `home_all` 下发 `(x=200, y=0, z=200, pitch=0, roll=0, claw=0, time=1000)` —— 这就是"全部回中"。
- 方向轮 `DirectionWheel` 输出 `X+ / X- / Y+ / Y-`，配"步进(mm)"与"速度(mm/s)"两个输入框。
- 坐标页本身**只发** `CMD_COORDINATE_SET(8)`、`CMD_ARM_MOVE_INC(50)`、`CMD_GET_CUR_COORDS(11)`；
  力矩开关在舵机页，即**上位机不会自动帮你上电**。

**配置类载荷**（其余页面的 struct 串，逐一在 `commands.py` 落成函数）
`SET_POS_OFFSET '<Bh'`、`SET_PID_PARAM '<BBBBH'`、`SET_COORD_LIMITS '<hhhhhhhhhhhh'`、
`SET_CHASSIS_CONFIG '<f'×`、`SET_KINEMATICS_PARAM '<f'×9`、`BUZZER_SET '<IIHH'`、`STEPPER_RUN '<i'`、
`APRILTAG_SET_OFFSET '<fff'`（颜色抓取偏移复用它）。

## 5. 三方交叉验证

| 项目 | NexArm.exe | ESP32 固件 (`Global.h`/`CommProtocol.cpp`) | 厂商 SDK (`ros_robot_controller_sdk.py`) | 判定 |
|---|---|---|---|---|
| 帧头/长度/校验 | `FF FF`，`2+len`，`~sum&0xFF` | 同 | 同 | ✅ 一致 |
| 命令号 1~94 / 120~135 | 全套 | 除 89 外全套 | 常用子集 | ✅ 一致 |
| 坐标载荷 | `'<hhhhhhH'` | AT32 期待同序，ESP32 仅转发 | `'<hhhhhhh'` | ✅ 字段序一致（时间字段宽度 H vs h，<32768 时字节相同） |
| 增量载荷 | `'<hhhhhhH'` | 转发 | `(dx,dy,dz,dpitch,droll,dclaw,time)` | ✅ 一致 |
| `GET_CUR_COORDS` 回包 | `<hhhhhh` + 6 舵机 | AT32 侧同样解析 24B | 同 | ✅ 一致 |
| 回包来源 ID | `0x5A` | AT32_SYS_ID | 判定 `0xFF 或 0x5A` | ✅ 一致 |

**差异项（已记录，未擅自处理）**
- `CMD_SET_INTERP_MODE = 89`：exe 与 SDK 有，出厂固件 `Global.h` 无、`grep INTERP` 无实现。
- exe 里有一批**别名/新增命名**（`CMD_MOTOR_STOP` `CMD_MECANUM_RUN` `CMD_TANK_RUN`
  `CMD_ESPNOW_*` `CMD_STEPPER_SET_DIV` `CMD_PS3_SET_MAC` `CMD_SYNC_MOVE_SERVOS`
  `CMD_READ_*`/`CMD_SET_*`），对应固件里同号但不同名的命令。
- 出厂固件比 exe 多出 `CMD_ARM_HOME(67)`、`CMD_LEROBOT_MODE(68)` 等（未在 UI 中暴露）。

## 6. 实机只读验证（2026-10-07 20:10，COM9）

`examples/01_query_status.py COM9` 全程只发读命令（1/2/11/65/66/80/64/58），未开闸门、未发力矩。
原始输出留档 `logs/status_probe_20261007_2010.txt`：

```
固件版本: esp=(1,0,2)  at32=(1,0,42)
电池    : 12.33 V
当前位姿: Pose(x=168, y=17, z=169, pitch=-102.6, roll=0, claw=-60,
               servos=[1984, 1991, 1947, 3058, 2048, 1364])
TCP 位姿: x=168 y=17 z=169 yaw=5.6 pitch=-102.6 roll=0.0 claw=-60.1
真实关节: J1=-5.6°(1984) J2=84.9°(1991) J3=-98.8°(1947) J4=-88.7°(3058) J5=0.0°(2048) J6=-60.1°(1364)
坐标限位: [100,400, -450,450, 20,500, -98.0,12.0, -90,90, -100,83]
运动学  : 110.45 225.00 36.97 145.00 0.00 130.23 0.00 50.00 70.50
舵机偏差: {1:46, 2:66, 3:-625, 4:-669, 5:-11, 6:51}
链路    : rx=15 帧 / 733 B，tx=13 帧，checksum_errors=0，resyncs=0
```

**由此从"纸面推导"升级为"实机确认"的项**：帧格式与校验、回包 ID=0x5A、位姿 24 B 布局、
真实关节 24 B 布局、TCP 14 B 布局、电池 `<H` mV、运动学 9×float、舵机偏差 `<Bh`。

**本次新发现（已改进库）**
- **限位的 pitch 字段是 0.1° 单位**：回包原始 `-980 / 120`，上位机 UI 显示 `-98.0 / 12.0`
  （证据：`SystemTab.set_coord_limits` 对下标 6/7 乘 10，`update_coord_limits` 再除 10，
  输入框范围 Pitch ±90°、Roll ±90°、Claw -60~30、X 50~450、Y ±350、Z 0~500）。
  库中 `pack_coord_limits` 已改为"入参用度、内部 ×10"，`decode_coord_limits` 输出度。
- 当前姿态 pitch=-102.6° **越过了 pitch 下限 -98.0°**，说明坐标限位只约束**新指令**、
  不约束既有状态——所以"回中/移动到合法点"不会被自身限位挡住，但按当前姿态做增量前要先算清落点。

**值得复盘的疑点**：AT32 自报 **1.0.42**，而 P0 阶段的结论是"1.0.42 = Bootloader 版本、
已强制刷入 1.0.2"。现在 AT32 能返回完整位姿/关节/TCP 数据，说明应答的固件是**有应用逻辑**的。
两种可能：(a) 1.0.42 其实就是官方应用版本，当初"停在 Bootloader"的判断有误；
(b) 版本寄存器未被那次写入更新。无论哪种，结论一致：**臂当前功能正常**，但版本号不能再当状态判据。



## 7. 局限（没做到的，不编）

1. **没有做完整源码级反编译**。本项目还原到"模块名 / 函数名 / 常量 / 名字表 / struct 格式串"这一层，
   足以定性协议与流程；没有逐函数还原成可读 Python 源码（缺 3.8 解释器，只能手写 marshal 读取器）。
2. 动作组 `.rob` 文件的**帧内部结构**未解码（只解到"分片下载 + 首片建文件"的语义）。
3. **读路径已实机验证，写路径还没有**：坐标/增量/加速度三条写入命令尚未在硬件上跑过实动作；
   首次使用按 README 建议小步试探（先 ±10mm）。
