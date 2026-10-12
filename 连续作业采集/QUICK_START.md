# 快速开始：批量采集

## 环境变量（每次新会话设置一次）

```powershell
Set-Location A:\NEXarm\Claude
$py = 'C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$task = 'A:\NEXarm\Claude\连续作业采集'
$env:PYTHONPATH = 'A:\NEXarm\Claude\.runtime_deps\serial_capture'
$env:PYTHONIOENCODING = 'utf-8'
```

## 检查进度

```powershell
& $py "$task\progress_tracker.py"
```

## 批量采集命令

### P0 初始位

```powershell
& $py "$task\capture_collection.py" batch --home --port COM11 --holder claw --grip-target 1800
```

### 红篮路线

```powershell
& $py "$task\capture_collection.py" batch --color 红 --target P1 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 红 --target P2 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 红 --target P3 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
```

### 黄篮路线

```powershell
& $py "$task\capture_collection.py" batch --color 黄 --target P1 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 黄 --target P2 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 黄 --target P3 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
```

### 蓝篮路线

```powershell
& $py "$task\capture_collection.py" batch --color 蓝 --target P1 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 蓝 --target P2 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 蓝 --target P3 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
```

### 绿篮路线

```powershell
& $py "$task\capture_collection.py" batch --color 绿 --target P1 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 绿 --target P2 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
& $py "$task\capture_collection.py" batch --color 绿 --target P3 --port COM11 --holder claw --grip-open 1800 --grip-close 2000
```

## 操作流程

1. **启动批量命令**（例如红→P1）
2. **每个点的操作**：
   - 用上位机摆好位置
   - 退出上位机（释放串口）
   - 终端按回车开始采集
   - 查看采集结果
   - 选择：回车继续 / r重采 / s跳过 / q退出
3. **完成后**：自动释放锁，篮子归位，继续下一条路线

## 验证

```powershell
# 验证某条路线
& $py "$task\progress_tracker.py" --verify 红 P1

# 全部完成后验证
& $py "$task\progress_tracker.py" --verify-all
```

## 注意事项

- `--grip-open` 和 `--grip-close` 的值根据实际篮子调整
- 如果四色篮提手大小相同，可以统一用 1800/2000
- 如果某色需要不同力度，单独调整那条路线的参数
- Ctrl+C 随时退出，自动释放锁
- 中断后重启命令会从下一个点继续
