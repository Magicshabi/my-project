# AT32 OTA 固件升级方案

## 概述

ESP32 通过 USART1（Serial1, 1Mbps）对 AT32F421 进行 OTA 固件升级。升级流程在 ESP32 上电时自动执行，比较嵌入的固件版本与 AT32 当前版本，仅在有新版本时升级。

## 架构

```
ESP32 (Serial1 GPIO16/17)  <--USART1 1Mbps-->  AT32F421 (PA9/PA10)
     |                                              |
     |  1. query_version (CMD 94)                    |  App: 回复版本号
     |  2. CMD 90 → AT32 App 写 magic + 复位         |  Bootloader: 检测 magic
     |  3. CMD 94 → Bootloader 回复就绪              |  Bootloader: 等待命令
     |  4. CMD 91 → 擦除 App 区                      |  Bootloader: flash_erase_app
     |  5. CMD 92 × N → 分包写入                     |  Bootloader: flash_write_data
     |  6. CMD 93 → 完成重启                         |  Bootloader: NVIC_SystemReset
     |  7. query_version → 验证新版本                 |  App: 回复新版本号
```

## 涉及文件

### ESP32 端
| 文件 | 说明 |
|------|------|
| `Nex_Arm.ino` | 入口，`SET_LOOP_TASK_STACK_SIZE(16384)` 防止栈溢出 |
| `AT32_OTA.h` | OTA 类声明 |
| `AT32_OTA.cpp` | OTA 核心逻辑：版本查询、握手、擦除、写入、验证 |
| `at32_firmware.h` | 由 `bin2c.py` 生成，包含固件二进制数据和版本号 |
| `system_task_handle.cpp` | 在 `register_system_task()` 中调用 `AT32_OTA::check_and_update()` |

### AT32 端
| 文件 | 说明 |
|------|------|
| `bootloader_proj/project/src/main.c` | Bootloader：帧解析、Flash 擦写、跳转 App |
| `AT32F421F8P7_nexarm_demo/project/src/main.c` | App：`CMD_FW_QUERY` 回复版本号，`CMD_FIRMWARE_UPDATE` 写 magic 并复位 |
| `AT32F421F8P7_nexarm_demo/hiwonder/Global.h` | App 版本号定义 `AT32_FW_VERSION_MAJOR/MINOR/PATCH` |

## Flash 布局

```
0x08000000 ┌──────────────────┐
           │   Bootloader     │  8KB (0x2000)
0x08002000 ├──────────────────┤
           │   App            │  56KB (0xE000)
0x08010000 └──────────────────┘
```

## 协议帧格式

```
0xFF 0xFF [ID] [LEN] [CMD] [DATA...] [CHECKSUM]

ID:       0xFF = 广播/ESP32→AT32, 0x5A = AT32→ESP32
LEN:      CMD(1) + DATA(n) 的字节数 + 1(checksum 不算在内... 实际 LEN = 2 + data_len)
CHECKSUM: ~(ID + LEN + CMD + DATA...) & 0xFF
```

## 命令定义

| CMD | 名称 | 方向 | 说明 |
|-----|------|------|------|
| 90 | `CMD_FIRMWARE_UPDATE` | ESP32→App | App 写 magic word 到 `0x20003FF0` 并复位 |
| 91 | `CMD_FW_START` | ESP32→Bootloader | 擦除 App 区（56KB），回复 ACK |
| 92 | `CMD_FW_DATA` | ESP32→Bootloader | 写入 128 字节数据包，回复 ACK + 校验结果 |
| 93 | `CMD_FW_END` | ESP32→Bootloader | 升级完成，Bootloader 复位 |
| 94 | `CMD_FW_QUERY` | ESP32→App/Bootloader | 查询版本。App 回复 `[major, minor, patch]`，Bootloader 回复 `[1]` |

## 升级流程详细时序

```
ESP32                                    AT32 (App 1.0.1)
  |                                         |
  |--- CMD_FW_QUERY (94) ------------------>|
  |<-- reply: [1, 0, 1] -------------------|  (版本 1.0.1)
  |                                         |
  |  比较: 1.0.1 < 1.0.2, 需要升级          |
  |                                         |
  |--- CMD_FIRMWARE_UPDATE (90) ----------->|
  |                                         |  写 magic, NVIC_SystemReset()
  |         delay(3000)                     |
  |                                         |  Bootloader 启动, 检测 magic
  |                                         |  主动发 CMD_FW_QUERY reply: [1]
  |                                         |
  |--- CMD_FW_QUERY (94) ------------------>|
  |<-- reply: [1] -------------------------|  (Bootloader 就绪)
  |                                         |
  |--- CMD_FW_START (91) ------------------>|
  |                                         |  擦除 56KB Flash (~500ms)
  |<-- reply: [0] -------------------------|  (擦除完成)
  |                                         |
  |--- CMD_FW_DATA (92) pkt 0 ------------>|
  |         delay(15)                       |  写入 128 字节 + 读回校验
  |--- CMD_FW_DATA (92) pkt 1 ------------>|
  |         delay(15)                       |
  |  ... 共 185 包 ...                      |
  |--- CMD_FW_DATA (92) pkt 184 ---------->|
  |                                         |
  |--- CMD_FW_END (93) ------------------->|
  |                                         |  NVIC_SystemReset()
  |         delay(3000)                     |
  |                                         |  Bootloader → jump_to_app()
  |                                         |  App 1.0.2 启动
  |--- CMD_FW_QUERY (94) ------------------>|
  |<-- reply: [1, 0, 2] -------------------|  (验证成功)
```

## 关键设计决策

### 1. AT32 复位时不操作 OLED
AT32 复位时 USART1 TX 线上可能产生毛刺，如果此时 ESP32 正在操作 I2C（OLED），可能导致 ESP32 崩溃。因此在发送 CMD 90 和 CMD 93 前后不调用 OLED，等 `delay(3000)` AT32 复位完成后再操作。

### 2. CMD 90 只发一次
之前反复发 CMD 90 导致 AT32 在 App 和 Bootloader 之间反复复位，ESP32 崩溃。改为只发一次 + `delay(3000)` 等待。

### 3. 栈大小 16KB
`SET_LOOP_TASK_STACK_SIZE(16384)` — OTA 函数 + `register_system_task` 初始化的局部变量总量超过默认 8KB 栈。

### 4. 数据包 pkt 用 static
`static uint8_t pkt[134]` 避免每次循环在栈上分配 134 字节。

### 5. Bootloader 写后读回校验
`flash_write_data` 写入后逐字节读回比较，累计 `verify_errors`。

## 如何更新固件

1. 用 Keil 编译新的 AT32 App，生成 `app.bin`
2. 更新 `hiwonder/Global.h` 中的版本号（递增）
3. 运行 `bin2c.py` 生成新的 `at32_firmware.h`：
   ```
   python bin2c.py <path_to_app.bin> <major> <minor> <patch>
   ```
4. 编译烧录 ESP32
5. 上电后自动检测并升级

## OLED 显示

升级过程中 OLED 显示：
```
行1: Now:  1.0.1        (当前版本)
行2: New:  1.0.2        (目标版本)
行3: Writing flash...   (当前步骤)
行4: 85%                (进度百分比)
```

## 串口日志

升级过程中 ESP32 USB Serial 输出：
```
[OTA] Start: embedded=1.0.2 size=23608
[OTA] query: got=1 ver=1.0.1
[OTA] Upgrading...
[OTA] Step1: CMD 90
[OTA] Waiting for bootloader...
[OTA] Bootloader ready: 1
[OTA] Step2: Erase
[OTA] Erase done: 1
[OTA] Step3: Write
[OTA] 1/185
[OTA] 51/185
[OTA] 101/185
[OTA] 151/185
[OTA] 185/185
[OTA] Step4: FW_END
[OTA] Verify: ok=1 ver=1.0.2
[OTA] Done.
```
