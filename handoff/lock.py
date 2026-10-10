# -*- coding: utf-8 -*-
"""设备占用锁：防止 Claw 与 Codex 同时抢同一个串口。

    python handoff/lock.py status  COM11
    python handoff/lock.py acquire COM11 --minutes 30 --purpose "带电可达性审查"
    python handoff/lock.py release COM11

锁文件：handoff/locks/<设备>.lock（JSON：holder / since / ttl_min / purpose）
TTL 过期视为自动释放，避免一方崩溃后另一方永久阻塞。
"""
import argparse
import json
import os
import sys
from datetime import datetime, timedelta, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
LOCKDIR = os.path.join(HERE, "locks")
TZ = timezone(timedelta(hours=8))


def lock_path(device):
    return os.path.join(LOCKDIR, "%s.lock" % device.upper())


def read_lock(device):
    path = lock_path(device)
    if not os.path.isfile(path):
        return None
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    try:
        since = datetime.fromisoformat(data["since"])
    except (KeyError, ValueError):
        return None
    data["_since_dt"] = since
    data["_expired"] = datetime.now(TZ) > since + timedelta(minutes=data.get("ttl_min", 30))
    return data


def cmd_status(args):
    data = read_lock(args.device)
    if data is None:
        print("%s: 空闲（无锁记录）" % args.device.upper())
        return 0
    state = "已过期（视为空闲）" if data["_expired"] else "被占用"
    print("%s: %s" % (args.device.upper(), state))
    print("  持有者: %s" % data.get("holder"))
    print("  开始于: %s" % data.get("since"))
    print("  TTL: %s 分钟" % data.get("ttl_min"))
    print("  用途: %s" % data.get("purpose"))
    return 0 if data["_expired"] else 1


def cmd_acquire(args):
    data = read_lock(args.device)
    if data is not None and not data["_expired"]:
        print("拒绝：%s 已被 %s 持有（开始于 %s，用途 %s）"
              % (args.device.upper(), data.get("holder"), data.get("since"), data.get("purpose")))
        print("→ 不要抢串口。等对方 release 或 TTL 过期，或先与用户确认。")
        return 2
    os.makedirs(LOCKDIR, exist_ok=True)
    if data is not None:
        print("提示：存在已过期的锁（持有者 %s），已接管。" % data.get("holder"))
    payload = {"holder": args.holder, "device": args.device.upper(),
               "since": datetime.now(TZ).isoformat(timespec="seconds"),
               "ttl_min": args.minutes, "purpose": args.purpose}
    with open(lock_path(args.device), "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    print("已获取 %s：%s，TTL %d 分钟" % (args.device.upper(), args.holder, args.minutes))
    return 0


def cmd_release(args):
    path = lock_path(args.device)
    data = read_lock(args.device)
    if data is None:
        print("%s: 无锁记录，无需释放" % args.device.upper())
        return 0
    if data.get("holder") != args.holder and not args.force:
        print("拒绝：锁由 %s 持有，你不是持有者（如需强制，加 --force）" % data.get("holder"))
        return 2
    os.remove(path)
    print("已释放 %s" % args.device.upper())
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("status", "acquire", "release"))
    ap.add_argument("device")
    ap.add_argument("--holder", default="claw", choices=("claw", "codex"))
    ap.add_argument("--minutes", type=int, default=30)
    ap.add_argument("--purpose", default="")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    return {"status": cmd_status, "acquire": cmd_acquire, "release": cmd_release}[args.action](args)


if __name__ == "__main__":
    raise SystemExit(main())
