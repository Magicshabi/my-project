# -*- coding: utf-8 -*-
"""Dump a PyInstaller PYZ archive: module list + per-function constant/name tables."""
import os
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import marshal38  # noqa: E402
from pyz_extract import find_pyz_entry  # noqa: E402


def iter_codes(co, depth=0):
    yield co, depth
    for c in co.consts or ():
        if isinstance(c, marshal38.Code):
            yield from iter_codes(c, depth + 1)


def fmt(v, limit=90):
    if isinstance(v, str):
        s = v if len(v) <= limit else v[:limit] + "..."
        return repr(s)
    if isinstance(v, (bytes, bytearray)):
        return "bytes(%d)" % len(v)
    if isinstance(v, marshal38.Code):
        return "<code %s>" % v.name
    return repr(v)


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else r"A:\NEXarm\NexArm.exe"
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.environ.get("TEMP", "."), "nexarm_pyz_report.txt")
    _, blob, pyvers = find_pyz_entry(exe)
    import struct
    toc_off = struct.unpack_from("!i", blob, 8)[0]
    print("pyvers=%s blob=%d toc@%d" % (pyvers, len(blob), toc_off))
    toc = marshal38.loads(blob[toc_off:])
    if isinstance(toc, list):
        norm = {}
        for item in toc:
            v = item[1]
            if len(v) == 2:
                norm[item[0]] = (int(v[0]), int(v[1]))
            else:
                norm[item[0]] = (int(v[1]), int(v[2]))
        toc = norm
    print("modules in PYZ:", len(toc))
    def entry(n):
        v = toc[n]
        return int(v[0]), int(v[1])

    names = sorted(toc)
    lines = []
    lines.append("PYZ module list (%d):" % len(names))
    for n in names:
        off, ln = entry(n)
        lines.append("  %-60s off=%-9d len=%d" % (n, off, ln))
    lines.append("")
    lines.append("=" * 78)
    for n in names:
        off, ln = entry(n)
        raw = blob[off: off + ln]
        try:
            plain = zlib.decompress(raw)
        except Exception as exc:
            lines.append("!! %s decompress failed: %s" % (n, exc))
            continue
        try:
            co = marshal38.loads(plain)
        except Exception as exc:
            lines.append("!! %s unmarshal failed: %s" % (n, exc))
            continue
        lines.append("### %s   (file=%s, code bytes=%d)"
                     % (n, co.filename, len(co.code or b"")))
        for c, depth in iter_codes(co):
            pad = "  " * (depth + 1)
            lines.append("%sdef %s(argc=%d, line=%d, varnames=%s)"
                         % (pad, c.name, c.argcount, c.firstlineno,
                            list(c.varnames or ())))
            consts = [fmt(v) for v in (c.consts or ()) if not isinstance(v, (marshal38.Code,))]
            if consts:
                lines.append("%s  consts : %s" % (pad, ", ".join(consts)))
            if c.names:
                lines.append("%s  names  : %s" % (pad, ", ".join(map(str, c.names))))
        lines.append("")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    print("report:", out, "lines:", len(lines))
    print("---- 模块名一览 ----")
    for n in names:
        print(" ", n)


if __name__ == "__main__":
    main()
