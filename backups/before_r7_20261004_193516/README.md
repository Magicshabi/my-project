# r7 写入前设备备份

2026-10-04，从 COM9 的 ESP32-D0WD rev1.1（MAC 68:09:47:7b:9f:64）读取。

- `flash_full_4MB.bin`：完整 4194304 字节，包含旧 r6 程序、分区、NVS 和其他闪存内容。
- SHA256：`120b018ce4e101efd130e50cff688780e294aa1c6cf2f8ed07ab99b5c5d4ddcc`。
- `verify_backup.log`：写入前设备端摘要匹配；`backup_manifest.json`：大小、摘要、原 r6 程序和分区一致性。
- `nvs_before.bin`：0x9000 起 0x5000 字节。`verify_nvs_after.log` 证明安装并启动 r7 后 NVS 与此备份一致。
- `r7_target_hashes.json`：此次目标源码及镜像的校验清单。

仅 ESP32 闪存备份，不含 AT32 内部闪存、舵机内部配置或 eFuse。未执行回滚。

如以后明确需要恢复此次 r6 状态，先托稳机械臂、断开外接舵机电源、只接 USB，确认是同一台设备及当前端口，然后使用本机 esptool 4.5.1：

```powershell
Set-Location -LiteralPath 'A:\NEXarm\Claude'
$env:PYTHONPATH='A:\NEXarm\Claude\.toolchains\esptool-py-4.5.1'
python -m esptool --chip esp32 --port COM9 --baud 460800 --before default_reset --after hard_reset write_flash --flash_mode keep --flash_freq keep --flash_size keep --verify 0x0 'backups\before_r7_20261004_193516\flash_full_4MB.bin'
```

此命令会覆盖 ESP32 全部闪存，并把后续示教数据回退到备份时刻；实际恢复前先另存最新闪存。这里仅提供恢复说明，没有执行此命令。
