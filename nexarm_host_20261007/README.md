# nexarm — NexArm 六轴机械臂主机侧运动控制库

从 `NexArm.exe`（PyInstaller + PyQt5 上位机）与 ESP32 出厂固件双向校对出来的协议，
封装成一套**能直接复用的 Python 运动控制层**。只依赖 `pyserial`，纯 Python 3.8+。

> 本目录是**新增**的独立实现，没有改动 `出厂程序/`、`视觉识别源码/`、`busservo_control/` 里的任何既有文件。

---

## 1. 它解决什么问题

一句话：**把"我发一条指令，臂就该动"这件事，变成一段可测试、可干跑、不会乱动的代码。**

机械臂的真实链路是三层，任何一层没对齐，臂就不动：

```
PC/脚本 ──USB 1,000,000bps──> ESP32 ──串口──> AT32(运动控制) ──> 6×HX-30HM 舵机
   └─ 本库这一层 ─┘             └ 转发/透传 ┘   └─ 真正做 IK 与 10ms 插补 ┘
```

所以"生成的代码驱动不了臂"通常不是语法问题，而是：**帧对不对、命令号对不对、
载荷字段顺序对不对、舵机力矩开没开、AT32 有没有收到**——本库把这些全部固化下来。

---

## 2. 目录结构（可复用结构）

```
nexarm_host_20261007/
├─ nexarm/
│  ├─ frames.py       帧格式 + 流式解析器（跨 read 边界、坏帧重同步）
│  ├─ commands.py     Cmd 命令表 + 全部 pack_*/decode_* 载荷编解码
│  ├─ transport.py    串口/TCP 传输层：后台收包线程、命令订阅、单写者锁
│  ├─ safety.py       安全层：工作空间盒、运动闸门、增量限幅、dry-run 计划
│  └─ arm.py          ArmController 门面：home/move_to/jog/move_path/…
├─ examples/
│  ├─ 01_query_status.py     只读体检（零风险，先跑这个）
│  ├─ 02_jog_and_home.py     干跑预览 → 加 --go 才真动
│  └─ 03_vision_pick_loop.py 视觉抓取骨架（标定接口，未标定直接拒绝执行）
├─ tests/test_codec.py       32 项离线自检（不接硬件）
└─ README.md
```

分层理由：`frames` 不知道"臂"是什么，`commands` 不知道"串口"是什么，
`transport` 不知道"运动"是什么，`arm` 只做编排，`safety` 谁都能调。
换掉任一层（比如把串口换成 WiFi/TCP）不影响其它层。

---

## 3. 快速开始

```bash
pip install pyserial
python tests/test_codec.py                 # 32 项自检，全绿再往下
python examples/01_query_status.py COM9    # 只读体检
python examples/02_jog_and_home.py COM9    # 干跑，打印动作计划
python examples/02_jog_and_home.py COM9 --go
```

```python
from nexarm import ArmController

arm = ArmController()
arm.connect_serial("COM9")     # 1000000 bps；务必先关掉 NexArm.exe，它独占串口
arm.enable_motion(True)        # 不显式打开，任何 move_* 都会 SafetyError
arm.set_torque(True)           # 力矩关了坐标指令不会让臂动
arm.set_acc(30)                # 起步加速度 0~254，0 最快

arm.home()                                                  # 回中 (200,0,200)
arm.move_to(x=250, y=0, z=150, pitch=0, roll=0, claw=-60,
            time_ms=1200)                                   # 绝对坐标，等到位
arm.jog(dz=20, time_ms=500)                                 # 增量
arm.move_path([(250, 0, 120), (250, 60, 120)], claw=-60)
print(arm.get_pose(), arm.get_battery_mv())
arm.close()
```

---

## 4. 接口一览

| 分类 | 方法 | 说明 |
|---|---|---|
| 连接 | `connect_serial(port, baud, reset_acc=False)` / `connect_socket(host, port)` / `close()` | 串口默认 1000000 8N1；默认不发任何指令 |
| 闸门 | `enable_motion(bool)` | 默认**关**；关着时所有运动 API 抛 `SafetyError` |
| 状态 | `get_pose()` | `Pose(x, y, z, pitch, roll, claw, servos)` |
| | `get_tcp_pose()` / `get_real_joints()` | 真实位姿 / 6 关节角度（cmd 66 / 65） |
| | `get_battery_mv()` / `get_version()` | 电压、ESP32+AT32 版本 |
| | `get_pose_offsets()` | 逐舵机偏差（上位机连上后做的同一件事） |
| 配置 | `get/set_limits()`, `push_limits()` | 工作空间盒，默认值取自固件 |
| | `get/set_kinematics()` | 9 项运动学参数（36 字节 float） |
| | `set_torque`, `set_acc`, `set_global_acc` | 力矩 / 起步加速度 / 全局加速度 |
| 运动 | `home()`, `reset_all(ms)` | 回中（坐标法，同 GUI）；全关节复位（cmd 54） |
| | `move_to(x,y,z,pitch,roll,claw,time_ms)` | 绝对坐标，IK 在 AT32 |
| | `jog(dx,dy,dz,dpitch,droll,dclaw,time_ms)` | 增量 |
| | `set_claw(angle)`, `move_servo(id, angle)` | 只动夹爪 / 单舵机直驱（调试） |
| | `move_path([(x,y,z), …])` | 路点序列 |
| 等待 | `wait_for(x,y,z,tol,timeout)` | 轮询 cmd 11 到位判定 |
| 外设 | `action_group_run(id)`, `buzzer()`, `oled()`, `conveyor()`, `stepper_run()` | |
| 调试 | `arm.plan` + `dry_run=True` | 干跑只做越界检查与序列记录 |

命令号、载荷格式、字段顺序、回包解析全部在 `nexarm/commands.py` 内联注释标明来源。

---

## 5. 协议速查

```
帧： FF FF | ID | LENGTH=2+len(ARGS) | CMD | ARGS | CHECKSUM
     下发 ID=0xFF(广播)；AT32 回包 ID=0x5A。
     CHECKSUM = (~sum(ID..ARGS)) & 0xFF        ← 不含两个 0xFF 帧头
```

| 命令 | 号 | 载荷 | 回包 |
|---|---|---|---|
| COORDINATE_SET | 8 | `<hhhhhhH` = (pitch×10, x, y, z, roll, claw, time) | 无 |
| ARM_MOVE_INC | 50 | `<hhhhhhH` = (**dx, dy, dz**, dpitch×10, droll, dclaw, time) | 无 |
| GET_CUR_COORDS | 11 | 空 | 24B：x,y,z,pitch×10,roll,claw + 6 舵机 |
| GET_REAL_JOINT_ANGLES | 65 | 空 | 24B：6×(pulse, angle×10) |
| GET_REAL_TCP_POSE | 66 | 空 | 14B：x,y,z,yaw×10,pitch×10,roll×10,claw×10 |
| ARM_RESET | 54 | `<H` 时长 ms | 无 |
| SET_TORQUE | 61 | `<B` 0/1 | 无 |
| SET_MOVE_ACC | 56 | `<B` 0~254 | 原样回执 |
| SET_COORD_LIMITS | 79 | `<hhhhhhhhhhhh`（x/y/z=mm，**pitch=0.1°**，roll/claw=度） | 无 |
| SET/GET_KINEMATICS_PARAM | 63/64 | 9×float32 | 36B |
| SET_POS_OFFSET | 57 | `<Bh` (id, offset) | 3B |
| SET_PID_PARAM | 59 | `<BBBBH` | 5B |
| BUZZER_SET | 9 | `<IIHH` | 无 |
| STEPPER_RUN | 19 | `<i` 步数 | 无 |
| ACTION_GROUP_RUN | 3 | `<B` 组号 | 无 |
| ACTION_GROUP_DOWNLOAD | 5 | `[组号, 总帧, 帧序号, 数据…]` | 无 |

**三个最常踩的坑**（库已内置，写别的代码时务必注意）：
1. `COORDINATE_SET` 是 `(pitch, x, y, z, …)`，**不是** `(x, y, z, pitch, …)`；而
   `ARM_MOVE_INC` 又是 `(dx, dy, dz, dpitch, …)`——两条命令字段顺序不一样。
2. **限位里的 pitch 是 0.1° 单位**（其余是 mm / 度）：出厂固件回包 `[-980, 120]` 对应
   UI 上的 `-98.0 / 12.0`。`pack_coord_limits` 入参用度、内部 ×10；`decode_coord_limits`
   出来就是度。差 10 倍会让限位形同虚设或把臂锁死。
3. 帧之间**绝不能夹 0xFF 填充**：`FF FF` 会被当成下一帧的 ID/LENGTH，吞掉一整帧
   （`tests/test_codec.py` 里有专门用例）。

---

## 6. 安全红线

- 运动和状态是两套 API，运动默认锁死，`enable_motion(True)` 才放行。
- 目标点先过工作空间盒，越界**报错而不是截断**（截断会让"动了但位置是错的"）。
- 单条增量默认上限 80 mm，防手滑。
- 标定参数（舵机偏差、运动学、坐标限位）动之前先 `get_*` 读回真机值，
  再用 `set_*` 写；本库内置的默认值来自固件源码，**不等于你机器上的标定值**。

## 7. 已知限制 / 尚未硬件验证

| 项 | 状态 |
|---|---|
| 帧格式、校验、命令号 | ✅ 三处独立来源一致 + 离线用例逐字节比对 |
| **读路径**（版本/电压/位姿/关节/TCP/限位/偏差/运动学） | ✅ **已实机验证**（`logs/status_probe_20261007_2010.txt`，rx 15 帧 0 校验错） |
| **写路径**（坐标/增量/加速度/力矩） | ⚠️ 尚未跑实动作；首次用 ±10mm 小步试探 |
| `ARM_MOVE_INC` 字段顺序 | ⚠️ 依厂商 SDK 与 exe 一致推导 |
| `SET_MOVE_ACC` 语义（0 最快） | ⚠️ 仅来自厂商 SDK 注释 |
| 动作组 `.rob` 帧内部结构 | ⚠️ 未解码，`action_group_run/erase` 可用，下载分片需自行拼 |
| `CMD_SET_INTERP_MODE=89` | ⚠️ 上位机有，出厂固件未见实现 |
| 与当前所烧固件的关系 | 视觉分支（Color_Sorting）也能收这些命令，但它同时跑视觉状态机；**做纯运动控制建议先烧回出厂程序 `Nex_Arm`** |

## 8. 与"机械臂动不了"的直接对应关系

| 现象 | 根因 | 本库怎么防 |
|---|---|---|
| 发了坐标，日志说动了，实际没动 | ESP32 只是转发者，AT32 才是执行者；帧/载荷错就静默失败 | 载荷顺序固化为常量 + 离线逐字节用例 |
| 换了套代码就"报幻象轨迹" | 只更新了 ESP32 侧变量，没真正写舵机 | 只走 cmd 8/50 这一条被验证过的下发路径 |
| 无动作、无报错 | 舵机力矩没开 | `set_torque()` 显式化，README 明说 |
| 一动就冲、或根本不动 | 起步加速度/插补参数没设 | `set_acc()` 显式设置（0 最快、254 最平滑） |
| 串口打不开 | `NexArm.exe` 独占 COM 口 | `TransportError` 直接说明原因 |
