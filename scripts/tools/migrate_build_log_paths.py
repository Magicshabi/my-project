# -*- coding: utf-8 -*-
"""把构建脚本的日志输出路径从根目录改到 logs\\build\\。

    python scripts/tools/migrate_build_log_paths.py            # 干跑
    python scripts/tools/migrate_build_log_paths.py --apply    # 执行

两个动作：
  1. 改写 scripts/build/*.ps1 里的日志绝对路径
     A:\\NEXarm\\Claude\\build_x.log -> A:\\NEXarm\\Claude\\logs\\build\\build_x.log
     （每个脚本 2 处：`*>` 写入 + `Get-Content` 回读）
  2. 把与脚本同名的**已有历史日志**移入 logs\\build\\historical\\，避免下一次跑
     构建时把它们覆盖掉（历史只增不删）。
"""
import glob
import json
import os
import re
import shutil
import sys

LOG_RE = re.compile(r"(?<!logs\\build\\)A:\\NEXarm\\Claude\\(build_[A-Za-z0-9_\-\.]+\.log)")
HIST_DIR = os.path.join("logs", "build", "historical")


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


def main():
    apply_ = "--apply" in sys.argv
    os.chdir(ROOT)

    scripts = sorted(glob.glob(os.path.join("scripts", "build", "*.ps1")))
    edits = {}
    names = set()
    for path in scripts:
        text = open(path, encoding="utf-8", errors="replace").read()
        hits = LOG_RE.findall(text)
        if hits:
            edits[path] = len(hits)
            names.update(hits)

    print("=== 改写计划 ===")
    print("脚本: %d 个, 共 %d 处日志路径" % (len(edits), sum(edits.values())))
    for p, n in sorted(edits.items()):
        print("   %-52s %d 处" % (os.path.basename(p), n))

    collide = [n for n in sorted(names)
               if os.path.isfile(os.path.join("logs", "build", n))]
    print("\n同名历史日志（将移入 %s/）: %d 个" % (HIST_DIR, len(collide)))
    for n in collide:
        print("   %s" % n)

    if not apply_:
        print("\n（干跑结束，未改动任何文件。加 --apply 执行）")
        return 0

    # 1) 保护历史日志
    os.makedirs(HIST_DIR, exist_ok=True)
    moved = []
    for n in collide:
        src = os.path.join("logs", "build", n)
        dst = os.path.join(HIST_DIR, n)
        if os.path.exists(dst):
            print("   !! historical 中已存在，跳过: %s" % n)
            continue
        shutil.move(src, dst)
        moved.append(n)
    print("\n已归档历史日志 %d 个" % len(moved))

    # 2) 改写脚本
    changed = 0
    for path in edits:
        text = open(path, encoding="utf-8", errors="replace").read()
        new = LOG_RE.sub(r"A:\\NEXarm\\Claude\\logs\\build\\\1", text)
        if new != text:
            with open(path, "w", encoding="utf-8", newline="") as fh:
                fh.write(new)
            changed += 1
    print("已改写 %d 个脚本" % changed)

    manifest = {
        "rewritten_scripts": sorted(os.path.basename(p) for p in edits),
        "archived_logs": moved,
        "historical_dir": HIST_DIR.replace(os.sep, "\\"),
        "note": "还原：把脚本里的 A:\\NEXarm\\Claude\\logs\\build\\ 前缀去掉，"
                "并把 historical/ 下的日志移回 logs\\build\\",
    }
    with open(os.path.join("_reorg", "build_log_path_migration.json"), "w",
              encoding="utf-8") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=2)
    print("manifest -> _reorg/build_log_path_migration.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
