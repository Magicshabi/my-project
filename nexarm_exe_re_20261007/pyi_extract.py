# -*- coding: utf-8 -*-
"""PyInstaller one-file archive extractor (no external deps).

Layout of a PyInstaller CArchive:
    [ entry data ... ][ TOC ][ cookie ]
cookie (88 bytes):
    magic[8]  package_len(u32)  toc_offset(u32)  toc_len(u32)  pyvers(u32)  pylibname[64]
TOC entry:
    entry_len(u32)  offset(u32)  comp_len(u32)  raw_len(u32)  compress(u8)  typecode(1B char)  name
offset is relative to the archive start (cookie_pos + 88 - package_len).
"""
import os
import sys
import zlib
import struct

MAGIC = b"MEI\014\013\012\013\016"


def find_cookie(data):
    pos = data.rfind(MAGIC)
    if pos < 0:
        raise SystemExit("PyInstaller cookie not found")
    return pos


def parse_toc(data):
    cookie = find_cookie(data)
    magic, pkg_len, toc_off, toc_len, pyvers, pylib = struct.unpack_from(
        "!8sIIII64s", data, cookie)
    pylib = pylib.split(b"\x00")[0].decode("latin1")
    arch_start = cookie + 88 - pkg_len
    if arch_start < 0 or arch_start > cookie:
        raise SystemExit(
            "bad cookie: cookie=0x%X pkg_len=%d -> arch_start=%d (filesize=%d)"
            % (cookie, pkg_len, arch_start, len(data)))
    toc = data[arch_start + toc_off: arch_start + toc_off + toc_len]
    entries = []
    i = 0
    while i + 18 <= len(toc):
        e_len, off, c_len, r_len = struct.unpack_from("!IIII", toc, i)
        compress, typecode = struct.unpack_from("!Bc", toc, i + 16)
        if e_len <= 0 or i + e_len > len(toc):
            break
        name = toc[i + 18: i + e_len].decode("utf-8", "replace").replace("\x00", "")
        entries.append({
            "name": name, "offset": off, "clen": c_len, "rlen": r_len,
            "compress": compress, "type": typecode.decode("latin1"),
        })
        i += e_len
    return {
        "arch_start": arch_start, "pyvers": pyvers, "pylib": pylib,
        "pkg_len": pkg_len, "toc_len": toc_len, "entries": entries,
        "cookie": cookie,
    }


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else r"A:\NEXarm\NexArm.exe"
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "_extracted")
    data = open(exe, "rb").read()
    info = parse_toc(data)
    arch_start = info["arch_start"]
    print("cookie@0x%X  archive@0x%X  pyvers=%d  pylib=%s  entries=%d" % (
        info["cookie"], arch_start, info["pyvers"], info["pylib"],
        len(info["entries"])))
    os.makedirs(outdir, exist_ok=True)
    only_code = "--only-code" in sys.argv
    keep_ext = (".pyc", ".py", ".json", ".ui", ".qss", ".ini", ".cfg", ".txt")
    manifest = []
    for e in info["entries"]:
        m = e.copy()
        raw = data[arch_start + e["offset"]: arch_start + e["offset"] + e["clen"]]
        if e["compress"]:
            try:
                raw = zlib.decompress(raw)
            except Exception as exc:
                m["error"] = str(exc)
        m["ok"] = len(raw) == e["rlen"] or not e["compress"]
        if only_code and not e["name"].lower().endswith(keep_ext):
            manifest.append(m)
            continue
        flat = e["name"].replace("/", "__").replace("\\", "__").strip() or "_noname"
        flat = flat.replace(":", "_")
        if len(flat) > 180:
            flat = flat[:140] + "_%08X" % (e["offset"])
        target = os.path.join(outdir, flat)
        try:
            with open(target, "wb") as fh:
                fh.write(raw)
        except OSError as exc:
            m["error"] = "write:" + str(exc)
        manifest.append(m)
    with open(os.path.join(outdir, "_toc.txt"), "w", encoding="utf-8") as fh:
        for m in manifest:
            fh.write("%-8s %-9d %-9d %s\n" % (m["type"], m["rlen"], m["clen"], m["name"]))
    print("extracted to", outdir)


if __name__ == "__main__":
    main()
