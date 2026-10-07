# -*- coding: utf-8 -*-
"""把构建缓存目录收进 .build-cache/，并同步改写构建脚本里的绝对路径。

    python scripts/tools/migrate_build_cache.py            # 干跑，只报告
    python scripts/tools/migrate_build_cache.py --apply    # 执行

为什么需要改写脚本：`scripts/build/*.ps1` 内部把 `--build-path` 写成绝对路径
（A:\\NEXarm\\Claude\\.arduino-build-xxx），目录一移动就必须同步改写，否则构建断链。

安全性：
  * 只移动 `.arduino-build*` 与 `.factory-build*`（可重建的编译缓存）。
  * 只改写**可执行文件**（scripts/ 下的 .ps1/.py）里的路径字符串；
    历史日志/文档/hashes 里的旧路径**一律保留**（那是"当时发生了什么"的记录，改了就是伪造）。
  * 产出 manifest，且迁移前的工作区已提交，可 git 还原。
"""
import json
import os
import re
import shutil
import sys

CACHE_DIR = ".build-cache"
MOVED_PREFIXES = (".arduino-build", ".factory-build")
REWRITABLE_EXT = (".ps1", ".py", ".cmd", ".bat", ".sh")

# 绝对路径改写：A:\NEXarm\Claude\.arduino-build-xxx -> A:\NEXarm\Claude\.build-cache\.arduino-build-xxx
PATH_RE = re.compile(
    r"(?<!\.build-cache\\)A:\\NEXarm\\Claude\\(\.(?:arduino-build|factory-build)[A-Za-z0-9_\-]*)")


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


def find_cache_dirs():
    out = []
    for name in sorted(os.listdir(ROOT)):
        if not os.path.isdir(os.path.join(ROOT, name)):
            continue
        if name.startswith(MOVED_PREFIXES):
            out.append(name)
    return out


def find_code_files():
    out = []
    me = os.path.basename(os.path.abspath(__file__))
    for base in ("scripts",):
        root = os.path.join(ROOT, base)
        for cur, dirs, files in os.walk(root):
            dirs[:] = [d for d in dirs if d != "__pycache__"]
            for f in files:
                if f.endswith(REWRITABLE_EXT) and f != me:
                    out.append(os.path.join(cur, f))
    return out


def main():
    apply_ = "--apply" in sys.argv
    os.chdir(ROOT)

    dirs = find_cache_dirs()
    total_mb = 0.0
    for d in dirs:
        for cur, _, files in os.walk(d):
            for f in files:
                try:
                    total_mb += os.path.getsize(os.path.join(cur, f))
                except OSError:
                    pass
    total_mb /= 1048576.0

    print("=== 迁移计划 ===")
    print("缓存目录: %d 个, 合计 %.1f MB -> 目标 %s/" % (len(dirs), total_mb, CACHE_DIR))
    for d in dirs:
        print("   %s" % d)

    edits = {}
    for path in find_code_files():
        try:
            text = open(path, encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        hits = PATH_RE.findall(text)
        if hits:
            edits[os.path.relpath(path, ROOT)] = len(hits)

    print("\n需改写的文件: %d 个, 共 %d 处路径"
          % (len(edits), sum(edits.values())))
    for p, n in sorted(edits.items()):
        print("   %-52s %d 处" % (p, n))

    if not apply_:
        print("\n（干跑结束，未改动任何文件。加 --apply 执行）")
        return 0

    # 1) 建目录并移动
    os.makedirs(CACHE_DIR, exist_ok=True)
    moved = []
    for d in dirs:
        src = os.path.join(ROOT, d)
        dst = os.path.join(ROOT, CACHE_DIR, d)
        if os.path.exists(dst):
            print("   !! 目标已存在，跳过: %s" % dst)
            continue
        shutil.move(src, dst)
        moved.append(d)
    print("\n已移动 %d 个缓存目录" % len(moved))

    # 2) 改写脚本
    changed = 0
    for rel in edits:
        path = os.path.join(ROOT, rel)
        text = open(path, encoding="utf-8", errors="replace").read()
        new = PATH_RE.sub(r"A:\\NEXarm\\Claude\\.build-cache\\\1", text)
        if new != text:
            with open(path, "w", encoding="utf-8", newline="") as fh:
                fh.write(new)
            changed += 1
    print("已改写 %d 个脚本" % changed)

    # 3) 让 git 忽略新位置
    gi = os.path.join(ROOT, ".gitignore")
    gtext = open(gi, encoding="utf-8", errors="replace").read()
    if CACHE_DIR + "/" not in gtext:
        if not gtext.endswith("\n"):
            gtext += "\n"
        gtext += "\n# 构建缓存（2026-10-07 从根目录收拢至此）\n%s/\n" % CACHE_DIR
        with open(gi, "w", encoding="utf-8", newline="") as fh:
            fh.write(gtext)
        print("已更新 .gitignore")

    manifest = {
        "moved_dirs": moved,
        "rewritten_files": sorted(edits),
        "cache_dir": CACHE_DIR,
        "note": "还原：把 .build-cache/* 移回根目录，并把脚本里的 "
                "A:\\NEXarm\\Claude\\.build-cache\\ 前缀去掉",
    }
    with open(os.path.join(ROOT, "_reorg", "build_cache_migration.json"), "w",
              encoding="utf-8") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=2)
    print("manifest -> _reorg/build_cache_migration.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
