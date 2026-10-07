# -*- coding: utf-8 -*-
"""List every struct format string (and nearby literals) used by each host module."""
import os
import re
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import marshal38  # noqa: E402
from pyz_extract import find_pyz_entry  # noqa: E402

FMT = re.compile(r"^[<>!=@][A-Za-z0-9x\?]{1,24}$")


def walk(co, prefix, out):
    name = prefix + co.name
    fmts = [c for c in (co.consts or ()) if isinstance(c, str) and FMT.match(c)]
    if fmts:
        out.append((name, co.firstlineno, sorted(set(fmts))))
    for c in co.consts or ():
        if isinstance(c, marshal38.Code):
            walk(c, name + ".", out)


def main():
    want = sys.argv[2:] or ["nexarm_qt.ui"]
    _, blob, _ = find_pyz_entry(sys.argv[1] if len(sys.argv) > 1
                                else r"A:\NEXarm\NexArm.exe")
    toc = marshal38.loads(blob[struct.unpack_from("!i", blob, 8)[0]:])
    rows = []
    for name, v in toc:
        if not any(name.startswith(w) for w in want):
            continue
        off, ln = int(v[1]), int(v[2])
        try:
            co = marshal38.loads(zlib.decompress(blob[off: off + ln]))
        except Exception:
            continue
        out = []
        walk(co, "", out)
        if out:
            rows.append((name, out))
    for name, out in sorted(rows):
        print("\n### %s" % name)
        for fn, line, fmts in out:
            print("   %-58s L%-5s %s" % (fn, line, ", ".join(fmts)))


if __name__ == "__main__":
    main()
