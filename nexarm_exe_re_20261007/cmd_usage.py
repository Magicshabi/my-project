# -*- coding: utf-8 -*-
"""Which firmware command identifiers does each host module actually reference?"""
import os
import re
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import marshal38  # noqa: E402
from pyz_extract import find_pyz_entry  # noqa: E402

CMD_RE = re.compile(r"^(CMD_|SERVO_|AT32_|ESP32)[A-Z0-9_]+$")


def walk(co, acc):
    for n in co.names or ():
        if isinstance(n, str) and CMD_RE.match(n):
            acc.add(n)
    for n in co.varnames or ():
        if isinstance(n, str) and CMD_RE.match(n):
            acc.add(n)
    for c in co.consts or ():
        if isinstance(c, marshal38.Code):
            walk(c, acc)


def main():
    _, blob, _ = find_pyz_entry(sys.argv[1] if len(sys.argv) > 1
                                else r"A:\NEXarm\NexArm.exe")
    toc = marshal38.loads(blob[struct.unpack_from("!i", blob, 8)[0]:])
    rows = []
    for name, v in toc:
        off, ln = int(v[1]), int(v[2])
        try:
            co = marshal38.loads(zlib.decompress(blob[off: off + ln]))
        except Exception:
            continue
        acc = set()
        walk(co, acc)
        if acc:
            rows.append((name, sorted(acc)))
    rows.sort()
    print("modules referencing firmware commands: %d" % len(rows))
    for name, cmds in rows:
        if not name.startswith("nexarm_qt"):
            continue
        print("\n[%s]" % name)
        print("   " + ", ".join(cmds))
    print("\n\n===== 全部命中命令号统计 =====")
    from collections import Counter
    cnt = Counter(c for _, cs in rows for c in cs)
    for c, n in sorted(cnt.items()):
        print("  %-32s %d" % (c, n))


if __name__ == "__main__":
    main()
