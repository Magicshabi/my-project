# 设备占用锁

物理设备（串口）同一时刻只能被一方占用。开串口前先登记：

```
python handoff/lock.py status  COM11
python handoff/lock.py acquire COM11 --minutes 30 --purpose "带电可达性审查"
python handoff/lock.py release COM11
```

- 锁文件：`handoff/locks/<设备>.lock`（JSON：holder / since / ttl_min / purpose）
- **TTL 过期视为自动释放**，避免一方崩溃后另一方永久阻塞
- 若锁被对方持有且未过期：**停手并告诉用户**，不要抢串口
