# 只读故障诊断 r2

当前已写入ESP32并完成两轮查询。程序不含位置、力矩、模式、ID、标定或OTA写入，NVS只读。按KEY1/KEY2均不会执行动作；主机输入丢弃。

结果见项目根目录`故障排查运行记录_20261006.md`和`read_diagnostic_20261006_131902_findings.json`。动作成功与故障根因均尚未确认。

- 构建：`build_read_diagnostic_r2.ps1`，ESP32 core2.0.17，独立目录`.arduino-build-read-diagnostic-r2`。
- 启动采集：`capture_read_diagnostic.py --port COM9`，主动复位ESP32并只读采集30秒。
- 离线重分析：`capture_read_diagnostic.py --log read_diagnostic_20261006_131902.log`，读取同名`.bin`，不打开串口。
- 运行前使用项目内串口依赖，`PYTHONPATH=A:\NEXarm\Claude\.toolchains\esptool-py-4.5.1`。
- 原r1b产物及记录副本：`backups/before_read_diag_r2_20261006`；设备全闪存原备份仍在`backups/before_execution_preflight_20261005_182226`。

`at32_static_analysis`中的镜像和反汇编来自本地出厂程序嵌入的1.0.2镜像。没有读出或改写设备AT32闪存；设备仅返回版本号1.0.2。静态结果不能单独证明设备上的每一字节一致。
