# -*- coding: utf-8 -*-
"""Extract PyInstaller PYZ archive (pure-python modules) out of a one-file exe.

PYZ layout:  b'PYZ\\x00' + <pyvers '!i'> + <toc_offset '!i'> ... [data] [toc]
toc entry:    <entry_len '!i'> <name_len '!i'> name <offset '!i'> <length '!i'>
"""
import os
import re
import sys
import zlib
import struct

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pyi_extract import parse_toc  # noqa: E402

NAME_RE = re.compile(rb"^[A-Za-z0-9_\.\-/\\]{1,120}$")


def find_pyz_entry(exe):
    data = open(exe, "rb").read()
    info = parse_toc(data)
    arch = info["arch_start"]
    for e in info["entries"]:
        if e["name"].lower().endswith(".pyz"):
            raw = data[arch + e["offset"]: arch + e["offset"] + e["clen"]]
            if e["compress"]:
                raw = zlib.decompress(raw)
            return e["name"], raw, info["pyvers"]
    raise SystemExit("no .pyz entry found")


def parse_pyz(blob):
    if blob[:4] != b"PYZ\x00":
        raise SystemExit("bad PYZ magic: %r" % blob[:8])
    candidates = []
    for pos in (8, 4, 12, 16, 20):
        if pos + 4 > len(blob):
            continue
        toc_off = struct.unpack_from("!i", blob, pos)[0]
        if not (0 < toc_off < len(blob)):
            continue
        toc = blob[toc_off:]
        entries = []
        i = 0
        ok = True
        while i + 8 <= len(toc):
            e_len, n_len = struct.unpack_from("!ii", toc, i)
            if e_len < 12 or n_len <= 0 or i + 8 + n_len + 8 > len(toc):
                ok = False
                break
            name = toc[i + 8: i + 8 + n_len]
            off, length = struct.unpack_from("!ii", toc, i + 8 + n_len)
            if not NAME_RE.match(name):
                ok = False
                break
            entries.append((name.decode("utf-8", "replace"), off, length))
            i += e_len
            if i >= len(toc):
                break
        if ok and entries:
            candidates.append((len(entries), pos, toc_off, entries))
    if not candidates:
        raise SystemExit("could not parse PYZ toc")
    candidates.sort(reverse=True)
    return candidates[0]


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else r"A:\NEXarm\NexArm.exe"
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.environ.get("TEMP", "."), "nexarm_pyz")
    name, blob, pyvers = find_pyz_entry(exe)
    n, pos, toc_off, entries = parse_pyz(blob)
    print("PYZ entry=%s  size=%d  pyvers=%s  header_field@%d toc@%d modules=%d"
          % (name, len(blob), pyvers, pos, toc_off, n))
    os.makedirs(outdir, exist_ok=True)
    listing = []
    for mname, off, length in entries:
        raw = blob[off: off + length]
        try:
            plain = zlib.decompress(raw)
            status = "zlib"
        except Exception:
            plain = raw
            status = "raw"
        flat = mname.replace(".", "_").replace("/", "_").replace("\\", "_") + ".pyc38"
        with open(os.path.join(outdir, flat), "wb") as fh:
            fh.write(plain)
        listing.append("%-8s %-8d %s" % (status, len(plain), mname))
    with open(os.path.join(outdir, "_modules.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(sorted(listing)))
    print("wrote %d modules to %s" % (len(listing), outdir))
    for line in sorted(listing):
        print(" ", line)


if __name__ == "__main__":
    main()
