# 更新 r7k2 前的 r7 全闪存备份

`flash_full_4MB.bin`：4194304 字节，设备摘要验证通过，包含 r7 程序、分区、NVS。

SHA256：`03d6f0687b1358942dedfc24f35135f5d1fbbe8bbe5097bd22368a374d66998a`。

设备 ESP32-D0WD rev1.1，MAC 68:09:47:7b:9f:64，备份时 COM9。详情 `backup_manifest.json`、`read_flash.log`、`verify_backup.log`。原 r6 备份仍在相邻的 before_r7_20261004_193516 目录。

恢复会把之后的数据回退到此备份时刻；本次没有执行恢复。仅在明确需要回滚时，托稳机械臂、断开外接舵机电源、只接 USB，并确认当前端口和设备，再执行：

```powershell
Set-Location -LiteralPath 'A:\NEXarm\Claude'
$env:PYTHONPATH='A:\NEXarm\Claude\.toolchains\esptool-py-4.5.1'
python -m esptool --chip esp32 --port COM9 --baud 460800 --before default_reset --after hard_reset write_flash --flash_mode keep --flash_freq keep --flash_size keep --verify 0x0 'backups\before_r7k2_20261004_1958\flash_full_4MB.bin'
```

恢复前另存最新状态。此备份不含 AT32 闪存、舵机内部设置或 eFuse。
