# -*- coding: utf-8 -*-
"""生成 A:\\NEXarm\\Claude 的结构快照，便于人工核对目录是否已按类别整理。

    python scripts/tools/snapshot_structure.py
输出：_reorg/AFTER_STRUCTURE.txt
"""
import datetime
import os
import sys


def resolve_root():
    cur = os.path.dirname(os.path.abspath(__file__))
    for _ in range(5):
        if os.path.isfile(os.path.join(cur, "PROJECT_STATE.md")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return os.getcwd()


ROOT = resolve_root()
CATEGORIES = ["docs", "reference", "scripts", "logs", "runs", "artifacts", "_reorg"]


def main():
    os.chdir(ROOT)
    lines = ["A:\\NEXarm\\Claude 结构快照",
             "生成时间: %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
             ""]
    loose = [f for f in sorted(os.listdir("."))
             if os.path.isfile(f) and f != "nul"]
    lines.append("根目录散落文件: %d 个（整理前为 278 个）" % len(loose))
    for f in loose:
        lines.append("   %-16s %d B" % (f, os.path.getsize(f)))
    lines.append("")
    lines.append("分类夹文件数:")
    total = 0
    for c in CATEGORIES:
        if not os.path.isdir(c):
            continue
        n = sum(len(fs) for _, _, fs in os.walk(c))
        total += n
        lines.append("   %-12s %3d" % (c + "/", n))
    lines.append("   %-12s %3d" % ("合计", total))
    lines.append("")
    lines.append("runs/ 子目录:")
    if os.path.isdir("runs"):
        for d in sorted(os.listdir("runs")):
            p = os.path.join("runs", d)
            if os.path.isdir(p):
                lines.append("   %-24s %3d" % (d, len(os.listdir(p))))
    lines.append("")
    lines.append("原地保留的目录（被构建脚本绝对路径引用，勿移动）:")
    keep = [d for d in sorted(os.listdir("."))
            if os.path.isdir(d) and not d.startswith(".")
            and d not in CATEGORIES and d not in ("__pycache__",)]
    lines.append("   " + "、".join(keep))

    text = "\n".join(lines)
    out = os.path.join("_reorg", "AFTER_STRUCTURE.txt")
    os.makedirs("_reorg", exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(text)
    print(text)
    print("\n已写入:", out)


if __name__ == "__main__":
    sys.exit(main())
