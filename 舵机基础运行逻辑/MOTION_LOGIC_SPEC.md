# NexArm 运动逻辑规格（实现级 · 供 codex 复现）

版本：2026-10-07 · 依据：实机实测 + 固件源码 + NexArm.exe 反编译三方交叉验证
配套代码：`nexarm/`（Python 主机库）、`tests/test_codec.py`（34 项离线自检）
证据留档：`logs/status_probe_20261007_2010.txt`、`logs/motion_staged_20261007_2018.txt`

> **给实现者的一句话**：这套运动不是"主机算出关节角再发下去"，而是
> **主机只表达意图（笛卡尔位姿或增量），IK 与插补全部由 AT32 内部完成**。
> 复现时不要自己实现 IK，也不要自己按帧发关节角流。

---

## 0. 术语与实体定义（先统一，后面全部复用）

**"角色"= 机械臂本体**，由 1 个末端执行器（TCP/夹爪）+ 6 个关节舵机构成。本文所有"运动"指该实体的位姿变化。

四层实体，**职责不可混淆**：

| 代号 | 实体 | 职责 | 不负责 |
|---|---|---|---|
| `HOST` | PC 上位机 / 本库 | 表达意图（位姿/增量）、限位与安全、到位判定 | 逆解、插补、舵机时序 |
| `MCU` | ESP32（中位机） | 帧路由、状态机（示教/视觉/分拣）、外设、上报 | **不产生运动**，只转发给 AT32 |
| `CTRL` | AT32（运动控制器） | **逆解 IK、10ms 插补、轨迹生成、限位执行**、状态记账 | — |
| `ACT` | 6×HX-30HM 总线舵机 | 角度执行与位置反馈 | — |

链路：`HOST ──USB 1000000bps──> MCU ──串口──> CTRL ──总线──> ACT`

### 术语表（全文一致）

| 术语 | 含义 | 单位 / 取值 |
|---|---|---|
| `pose` | 末端位姿 `(x, y, z, pitch, roll, claw)` | x/y/z: **mm**；pitch/roll/claw: **度** |
| `target` | CTRL 内部记录的**目标值** | 同上 |
| `pulse` | 舵机位置计数（tick） | int16，0~4095 |
| `tick` | 舵机角度分辨率单位 | **1 tick = 360/4096 = 0.08789°** |
| `frame` | 协议帧 | 见 §2.1 |
| `cmd` | 命令号 | uint8，见 §2.2 |
| `内部态` | CTRL 记账的 target，**不等于**舵机实际位置 | — |
| `插补` | CTRL 把一段运动拆成 10ms 步进 | — |
| `增量` | `cmd 50`，以当前位姿为基准加一个偏移 | — |
| `残差` | 增量指令的实走距离与指令距离之差 | mm |
| `串扰` | 增量指令带动未指定的自由度 | 度 |

---

## 1. 模块划分（复现时的目录结构即按此分）

```
M0 Transport   传输：串口 1000000 8N1 / TCP 8080；后台收包线程；单写者锁
M1 Codec       帧编解码：build_frame / FrameParser（流式、可重同步）
M2 Command     命令层：命令号表 + 载荷 pack_*/decode_*
M3 Safety      安全：工作空间盒、运动闸门、增量限幅、dry-run
M4 Orchestrator 编排：ArmController.move_to / jog / set_claw / wait_for / move_path
M5 CtrlModel   CTRL 行为模型（**只读观测**，不参与下发）：内部态、残差、串扰
M6 Actuator     执行器通道：舵机寄存器直读（本固件不可用，见 §4.4）
```

**依赖方向**：`M4 → M3 → M2 → M1 → M0`；`M5/M6` 只被观测，不被下发路径依赖。
**禁止**：在 M4 里做 IK；在 M1 里做限位；在 M0 里解析业务命令。

---

## 2. 协议层（M1 / M2）

### 2.1 帧格式（三处来源逐字节一致）

```
+------+------+-----+--------+-----+---------------+----------+
| 0xFF | 0xFF | ID  | LENGTH | CMD | ARGS (N 字节) | CHECKSUM |
+------+------+-----+--------+-----+---------------+----------+
```

- `ID`：`HOST` 下发固定 `0xFF`；`CTRL` 回包为 `0x5A`；`ACT` 透传回包的 ID = 舵机号。
- `LENGTH = 2 + len(ARGS)`，范围 `2..252`（ARGS ≤ 250）。
- `CHECKSUM = (~sum(ID, LENGTH, CMD, ARGS)) & 0xFF`，**不含两个 0xFF 帧头**。
- 校验错误 → 丢帧并计数，不回复。

**硬约束（违反即静默失败）**：
1. 帧与帧之间**禁止插入 0xFF 填充**：`FF FF` 会被当成下一帧的 ID/LENGTH，吞掉整帧。
2. 单帧 ARGS 超 250 字节必须分片；分片语义由各命令自己定义。

### 2.2 运动相关命令与载荷（字段顺序是**命令级**的，不可套用）

| cmd | 名称 | 载荷（struct） | 字段顺序 | 备注 |
|---|---|---|---|---|
| 8 | `COORDINATE_SET` | `'<hhhhhhH'` 14B | **(pitch×10, x, y, z, roll, claw, time_ms)** | 绝对值 |
| 50 | `ARM_MOVE_INC` | `'<hhhhhhH'` 14B | **(dx, dy, dz, dpitch×10, droll, dclaw, time_ms)** | 增量 |
| 11 | `GET_CUR_COORDS` | 空 | 回包 24B：x,y,z,pitch×10,roll,claw + 6×pulse | 见 §4.4 |
| 65 | `GET_REAL_JOINT_ANGLES` | 空 | 回包 24B：6×(pulse, angle×10) | **与 cmd 11 同源** |
| 66 | `GET_REAL_TCP_POSE` | 空 | 回包 14B：x,y,z,yaw×10,pitch×10,roll×10,claw×10 | |
| 54 | `ARM_RESET` | `'<H'` | duration_ms (uint16) | 全关节复位 |
| 61 | `SET_TORQUE` | `'<B'` | 0/1 | **力矩闸门**，见 §3 |
| 56 | `SET_MOVE_ACC` | `'<B'` | 0~254，0=最快、254=最平滑 | 插补起步加速度 |
| 31 | `SET_GLOBAL_ACC` | `'<B'` | 0~100 | |
| 79 | `SET_COORD_LIMITS` | `'<hhhhhhhhhhhh'` 24B | 12×int16，**pitch 字段是 0.1°** | 见 §7.1 |
| 80 | `GET_COORD_LIMITS` | 空 | 同上格式回包 | |
| 63/64 | `SET/GET_KINEMATICS_PARAM` | `'<9f'` 36B | 9 个 float32 | 更换连杆后才需要动 |

> ⚠️ **两条最致命的字段顺序差异**：`cmd 8` 以 **pitch 打头**，`cmd 50` 以 **dx 打头**。
> 按 `(x,y,z,pitch,...)` 去打包 `cmd 8` 会得到一条"看似成功、实际乱走"的指令。

---

## 3. 触发条件：运动能够发生，以下条件**全部**为真

| 编号 | 条件 | 判据 / 验证方法 | 不满足时的表现 |
|---|---|---|---|
| T1 | 供电与总线正常 | 能读到 `cmd 1/2/11` 回包；`checksum_errors=0` | 无回包 / 乱码 |
| T2 | **力矩 ON** | 上电后必须显式开启；也可听声音/推末端感知 | **所有数值照变，臂一动不动**（最易误判） |
| T3 | `CTRL` 应用在运行 | `cmd 11` 返回合理位姿与 6 个 pulse | 版本号会自报但无有效数据 |
| T4 | 帧合法 | 长度/校验符合 §2.1 | 静默丢弃，无任何提示 |
| T5 | 目标在限位内 | 先 `cmd 80` 读真机限位 | 被 CTRL 拒绝或截断 |
| T6 | `MCU` 状态机不在独占态 | 视觉/示教任务会抢占臂 | 命令被状态机覆盖 |
| T7 | （主机侧）安全闸门已开 | `enable_motion(True)` | 本库直接抛 `SafetyError`，不发帧 |

### 3.1 T2 的两种达成方式（本次实机的坑）

当前实机所烧固件是**自建诊断版**，`setup()` 末尾为：

```cpp
arm.set_torque(false);
enter_pose_calibration("startup requires manual confirmation");   // 等待 KEY1
```

即**上电不自动回零、力矩强制关闭**。解锁方式二选一：

- **A（现场）**：扶住机械臂 → 摆到期望姿态 → 保持不动 → **短按 KEY1** → 固件写目标 + 开力矩 + 校验 2s（串口打印 `[PoseCal] Saved+locked:`）。
- **B（刷固件）**：烧回厂商原版（`perform_arm_poweron_reset()` → `arm.reset_all(2500)`，上电自动回零并开力矩）。

> 厂商原版路径不要求 KEY1；这条差异是我们自己引入的，复现时若目标是"上电即可动"，应采用 B。

---

## 4. 状态更新模型

### 4.1 `CTRL` 内部目标态（唯一驱动源）

收到 `cmd 8` / `cmd 50` 后，`CTRL` 更新内部 `target`：

- `cmd 8`：`target.pose ← (x, y, z, pitch, roll, claw)`，`target.pulse ← IK(target.pose)`
- `cmd 50`：`target.pose ← f(target.pose ⊕ (dx,dy,dz,dpitch,droll,dclaw))`
  **注意：`f` 是关节空间再解算，不是严格笛卡尔加法**（见 §5.2 串扰实测）

随后 `CTRL` 以 10ms 为周期做插补，逐步把 `target.pulse` 写成舵机目标寄存器。

### 4.2 舵机实际位置

真值只能由 `ACT` 的 `PRESENT_POSITION_L(56)` 寄存器提供。
**本机固件未把舵机透传数据回推到 USB**：`cmd=2, reg=56` 直读无回包。

### 4.3 主机侧影子状态

`HOST` 维护 `last_pose`（由 `cmd 11` 回包更新）。它属于**缓存**，不是真值。

### 4.4 ⚠️ 不可推断规则（复现时务必遵守）

实测：`cmd 11` 的 6 个 pulse 与 `cmd 65` 的 6 个 pulse **逐字节完全相同**，包括 1-tick 抖动，
且在运动过程中也每次相等。

**结论**：二者同源（`CTRL` 用同一份内部数组填两个回包），**都不构成"舵机真的动了"的证据**。

**因此**：
- ❌ 不得用 "cmd 11 与 cmd 65 一致" 证明执行器已动作。
- ❌ 不得把 `pose` 当作舵机实际位置做闭环标定。
- ✅ 判定"是否真的动"必须用**物理观测**（视觉/听觉/触觉）或 `ACT` 寄存器真值。

---

## 5. 坐标与位移计算

### 5.1 绝对位姿（`cmd 8`）

`HOST` 只发目标笛卡尔位姿；`CTRL` 内部执行 IK：

```
target.pulse = IK(x, y, z, pitch, roll, claw | 机构参数)
```

机构参数由 `cmd 63` 的 9 个 float 描述（本机实测值）：
`[110.45, 225.00, 36.97, 145.00, 0.00, 130.23, 0.00, 50.00, 70.50]`
→ 复现时**直接读取**，不要臆造。

### 5.2 增量（`cmd 50`）——两条必须建模的偏差

| 指令 | 实走 | 偏差 |
|---|---|---|
| `dz = +10` | Δz = +8 | −2 |
| `dx = +30` | Δx = +40 | **+10** |
| `dx = −30` | Δx = −32 | −2 |

- **残差**：与历史记录的"AT32 增量回退残留（+9~10 恒定残差）"一致，量级 **±10mm**，方向相关。
- **串扰**：仅命令 `dz=+10` 时，`pitch` 从 −102.6° 变为 −99.2°（**3.4°**），即未指定自由度被带动。

**复现策略（强制）**：
- 需要精度时用 `cmd 8` 绝对坐标闭环，**不要**用一串增量去逼近目标。
- 必须用增量时，把它当"粗动"，随后用 `cmd 8` 或视觉闭环收敛。

### 5.3 单位换算（贯穿全链）

```
pitch_on_wire = round(pitch_deg × 10)      # cmd 8 / 50 / 11 回包 / 65 回包 都是 ×10
angle_deg     = angle_x10 / 10             # cmd 65
tick          = pulse                      # 0..4095
angle_deg     = pulse × 360 / 4096         # = pulse × 0.087890625
```
**例外**：`cmd 79/80` 的限位数组里，**只有下标 6、7（pmin/pmax）是 ×10**，其余（x/y/z 是 mm，roll/claw 是度）原样。

### 5.4 视觉 → 坐标的衔接（P2 用，尚未实机标定）

```
(u, v) 像素 ──H(3×3 单应)──> (x, y) 机械臂 mm
x = (H0·u + H1·v + H2) / (H6·u + H7·v + 1)
y = (H3·u + H4·v + H5) / (H6·u + H7·v + 1)
```
`H` 由 ≥4 组 `(像素, 机械臂坐标)` 对应点解出（高斯消元，`examples/03`）。
**Z 不来自视觉**：抓取高度是标定常量（approach_z / grab_z / lift_z）。

---

## 6. 每步执行流程（一次 `move_to` 的完整时序）

```
Step 1  前置检查（M3）
        gate.enabled? 否则 SafetyError
        limits.check(x,y,z,pitch,roll,claw) 越界 → SafetyError（不截断）
        gate.pace() 限制指令密度（默认 50ms）
Step 2  打包（M2）
        args = pack_coord(x, y, z, pitch, roll, claw, time_ms)   # 注意字段顺序
Step 3  组帧（M1）
        frame = FF FF FF (2+14) 08 args (~sum&0xFF)
Step 4  下发（M0）
        tx_lock 内 write(serial) / sendall(socket)
Step 5  CTRL 侧（不属本库，但必须理解）
        CTRL 更新 target.pose → IK → target.pulse
        以 10ms 周期插补，把 pulse 写入 ACT 目标寄存器
        time_ms 决定本次运动的总时长（uint16，0~65535）
Step 6  回包（M0 → M1 → M5）
        CTRL 用 ID=0x5A 回 cmd 11 的 24 字节位姿；HOST 更新 last_pose
Step 7  到位判定（M4）
        poll cmd 11，满足 |x-x*|≤tol 且 |y-y*|≤tol 且 |z-z*|≤tol → 到位
        默认 tol = 8mm，超时默认 8s
Step 8  超时处理
        超时 → 记录并返回 False；**不重发、不累计误差**
Step 9  结束
        清理订阅；连接可复用（无需重连）
```

**伪代码（可直接照抄）**

```python
def move_to(x, y, z, pitch, roll, claw, time_ms, tol=8.0, timeout=8.0):
    gate.require()                       # T7
    limits.check(x, y, z, pitch, roll, claw)   # T5，越界 raise
    gate.pace()
    args = struct.pack("<hhhhhhH", int(pitch*10), int(x), int(y), int(z),
                       int(roll), int(claw), time_ms & 0xFFFF)
    send(build_frame(8, args))           # Step 3+4
    deadline = now() + timeout
    while now() < deadline:              # Step 7
        p = request(11, timeout=0.3)
        if p and abs(p.x-x) <= tol and abs(p.y-y) <= tol and abs(p.z-z) <= tol:
            return True
        sleep(0.1)
    return False                         # Step 8
```

---

## 7. 边界与终止条件

### 7.1 坐标限位（本机实测值）

```
[100, 400,  -450, 450,  20, 500,  -98.0, 12.0,  -90, 90,  -100, 83]
 xmin xmax   ymin ymax   zmin zmax  pmin°  pmax°  rmin rmax  cmin cmax
```
- **pitch 越界不阻止"移动到合法点"**：实测当前姿态 `pitch = −102.6°` 已低于下限 −98.0°，
  说明限位**只约束新指令，不约束既有状态**。
- 复现时必须：**读真机限位**（`cmd 80`）后同步到 M3，不要用文档默认值当本机值。

### 7.2 数值与协议边界

| 项 | 边界 | 违背后果 |
|---|---|---|
| `time_ms` | uint16，0~65535 | 溢出即错时机 |
| `pitch` 编码 | ×10 后须落在 int16 | 溢出致反向 |
| ARGS 长度 | ≤250 | 必须分片 |
| 帧间填充 | **禁止 0xFF** | 吞掉整帧（有专门用例） |
| `acc` | 0~254 | 越界被钳位 |

### 7.3 终止条件（必须实现的四类）

| 类型 | 触发 | 动作 |
|---|---|---|
| 正常终止 | 到位（§6 Step 7） | 返回 True |
| 超时终止 | 超过 `timeout` | 返回 False，**停止等待但不重发** |
| 安全终止 | 越界 / 闸门关闭 / 单次增量 > 80mm | 抛 `SafetyError`，**不发送任何帧** |
| 链路终止 | 校验连续错误 / 串口异常 | 关闭接收线程，标记断连，后续 `send` 抛 `TransportError` |

### 7.4 明确的"不要做"

1. 不要用 `cmd 11`/`cmd 65` 的一致性证明舵机动作（§4.4）。
2. 不要用增量指令的累加结果当作已知位置（§5.2 残差 +10mm 量级）。
3. 不要在多轴联动的目标上用 `tol` 之外的假设（到位判定只看 x/y/z，不看角度）。
4. 不要在 `time_ms` 很短（<300ms）时下发大位移，插补会变成阶跃。

---

## 8. 复现检查清单

按序执行，每步都有可观测的通过判据：

1. 打开串口（1000000 8N1，先关掉 `NexArm.exe`，它独占 COM 口）。
2. 读 `cmd 1` → 得到 `esp=1.0.2 / at32=1.0.42` 形态的版本号。
3. 读 `cmd 2` → 得到电压 mV。
4. 读 `cmd 11` → 得到 24 字节位姿 + 6 pulse；连读两次应稳定。
5. 读 `cmd 80` → 同步限位到 M3。
6. 读 `cmd 64` → 同步 9 项机构参数。
7. **确保力矩 ON**（§3.1：KEY1 或刷厂商固件）。
8. 发 `cmd 50` `dz=+10` → **物理观测**末端是否上抬约 8~10mm。**这一步是唯一可信的写路径验收**。
9. 通过后再发 `cmd 8` 做绝对移动，用 `cmd 11` 轮询到 `tol=8mm`。
10. 全流程 `checksum_errors == 0`、`resyncs == 0`。

---

## 9. 已知未知项（**不允许推断，必须标注**）

| 项 | 状态 | 复现时的处理 |
|---|---|---|
| `ACT` 实际位置真值 | ❌ 本固件不透传（`cmd 2/reg 56` 无回包） | 需物理观测替代；或改用支持透传的固件 |
| `cmd 8` 的实际重复定位精度 | ⚠️ 未标定 | 先在真机上做点位往返测量再写进参数 |
| `SET_MOVE_ACC` / `SET_GLOBAL_ACC` 的精确语义 | ⚠️ 仅厂商 SDK 注释（0 最快） | 小步试探后再用于产线 |
| 动作组 `.rob` 帧内部结构 | ❌ 未解码 | 只用 `cmd 3` 运行已存在的组，不要自己构造 |
| `cmd 89 SET_INTERP_MODE` | ⚠️ 上位机有、出厂固件无实现 | 不要发 |
| 相机 → 机械臂的 `H` 与 Z 常量 | ⚠️ 未标定 | 必须先做 4 点标定 |

---

## 10. 与"看似能动"的假象对照表（排错用）

| 症状 | 真因 | 定位方法 |
|---|---|---|
| 数值全变、臂不动 | **力矩 OFF**（T2） | §3.1；推末端感知刚性；KEY1 解锁 |
| 发了命令毫无反应，日志也无 | 帧不合法（T4） | 校验和/长度/帧间 0xFF 填充 |
| 臂动了但方向不对 | 载荷字段顺序错（§2.2） | 对照 `cmd 8` vs `cmd 50` 的顺序 |
| 走到一半停下 | 限位截断 / 超时 | 读 `cmd 80`；加大 `timeout` |
| 连续几次增量后位置越走越偏 | 增量残差（§5.2） | 改用 `cmd 8` 闭环 |
| 只有某个自由度动不了 | 该关节被状态机占用 / 该自由度已被串扰带偏 | 读 `cmd 65` 观察该关节 pulse |
