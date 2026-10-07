# -*- coding: utf-8 -*-
"""Claude 目录整理：把根目录 278 个散落文件按类别归入 5 个顶层分类夹。

    python reorg_claude.py            # 干跑，输出完整映射表（不动文件）
    python reorg_claude.py --apply    # 执行（纯重命名，git 可完整还原）

设计原则
--------
1. **只搬根目录的散落文件**，所有目录原地不动 —— 因为 .ps1 构建脚本与 arduino-cli
   参数里全是绝对路径（A:\\NEXarm\\Claude\\...），移动目录会直接断链。
2. 纯重命名，不删除、不覆盖、不改内容；同名冲突会报错而不是覆盖。
3. 产出 manifest.json 记录 old->new 全量映射，可据此一键还原。
"""
import json
import os
import re
import shutil
import sys

def _resolve_root():
    """脚本可能被归类到 scripts/tools/，向上找到真正的工程根（含 README+PROJECT_STATE）。"""
    cur = os.path.dirname(os.path.abspath(__file__))
    for _ in range(5):
        if (os.path.isfile(os.path.join(cur, "PROJECT_STATE.md"))
                and os.path.isdir(os.path.join(cur, "舵机基础运行逻辑"))):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))


ROOT = _resolve_root()
if not os.path.isdir(os.path.join(ROOT, "scripts")):
    raise SystemExit("无法定位工程根目录，ROOT=%s" % ROOT)

# 必须留在根目录的入口文件
KEEP_AT_ROOT = {"README.md", "PROJECT_STATE.md", "INDEX.md", ".gitignore"}

# 顶层分类：(目录名, 用途说明)
CATEGORIES = {
    "docs": "文档与资料：总结/方案/复盘 md、厂商 PDF、代码审查记录（人读）",
    "scripts": "可执行脚本：Python 采集/测试工具、PowerShell 构建脚本",
    "logs": "运行日志：编译日志、烧录日志、烧录哈希清单",
    "runs": "实验采集：每次运行产出的 .bin/.json/.log 三元组，按实验分子目录",
    "artifacts": "产物归档：固件 bin/csv、diff、临时检查文本、o 文件",
    "reference": "外部参考资料：厂商 PDF 教程（单列以免与自产文档混扫）",
}

# 顺序敏感：先匹配到的规则胜出
RULES = [
    (r"^build_.*\.ps1$", "scripts", "build"),
    (r"^build_.*\.log$", "logs", "build"),
    (r"_flash\.log$", "logs", "flash"),
    (r"_hashes\.json$", "logs", "manifest"),
    (r"^r[0-9].*\.md$", "docs", "reviews"),
    (r"_review.*\.json$", "docs", "reviews"),
    (r".*\.md$", "docs", "notes"),
    (r".*\.pdf$", "reference", ""),
    (r"^capture_.*\.py$", "scripts", "capture"),
    (r"^test_.*\.py$", "scripts", "test"),
    (r".*\.py$", "scripts", "tools"),
    (r"lock.*\.ps1$", "scripts", "tools"),
    (r"^capture_factory_.*\.ps1$", "scripts", "tools"),
    (r"^factory_.*\.(bin|csv)$", "artifacts", "firmware"),
    (r"_backup_path\.txt$", "artifacts", "checks"),
    (r"^_r7_check\d*\.txt$", "artifacts", "checks"),
    (r".*\.diff$", "artifacts", "diff"),
    (r".*\.o$", "artifacts", "misc"),
    (r"\.(bin|json|log|csv)$", "runs", None),      # 归入 runs/<实验名>/
]

# 实验名推断（前缀 → 子目录），按长度降序匹配，避免短前缀吃掉长的
RUN_TASKS = [
    ("execution_preflight_boot", "execution_preflight"),
    ("single_joint_probe_r1b", "single_joint_r1b"),
    ("single_joint_probe_r1", "single_joint_r1"),
    ("single_joint_boot", "single_joint"),
    ("single_joint_trial", "single_joint"),
    ("teach_record_r7k2", "teach_record_r7k2"),
    ("teach_record_r7", "teach_record_r7"),
    ("teach_record_r6k1", "teach_record_r6k1"),
    ("teach_record_r6", "teach_record_r6"),
    ("wrapped_move_probe", "wrapped_move"),
    ("wrapped_move_boot", "wrapped_move"),
    ("wrapped_move_run", "wrapped_move"),
    ("id6_torque_probe", "id6_torque"),
    ("id6_torque_boot", "id6_torque"),
    ("id6_torque_run", "id6_torque"),
    ("read_diagnostic", "read_diagnostic"),
    ("handle_teach_r4", "handle_teach_r4"),
    ("readonly_goal_r2", "readonly_goal"),
    ("readonly_oled_r3", "readonly_oled"),
    ("readonly_r1", "readonly_r1"),
    ("readonly_before", "readonly"),
    ("readonly_after", "readonly"),
    ("match_safe_r2", "match_safe"),
    ("match_safe_r3", "match_safe"),
    ("match_readonly_r1", "match_readonly"),
    ("match_readonly_goal", "match_readonly"),
    ("match_readonly_oled", "match_readonly"),
]


def pick_rule(name):
    for pat, cat, sub in RULES:
        if re.search(pat, name):
            return cat, sub
    return None, None


def pick_task(name):
    for prefix, task in sorted(RUN_TASKS, key=lambda x: -len(x[0])):
        if name.startswith(prefix):
            return task
    return "misc"


def plan():
    files = [f for f in sorted(os.listdir(ROOT))
             if os.path.isfile(os.path.join(ROOT, f))]
    moves, skipped = [], []
    for name in files:
        if name in KEEP_AT_ROOT:
            skipped.append((name, "根目录入口文件，保留"))
            continue
        cat, sub = pick_rule(name)
        if cat is None:
            skipped.append((name, "无匹配规则，保留待人工判定"))
            continue
        if cat == "runs":
            target = os.path.join("runs", pick_task(name))
        else:
            target = os.path.join(cat, sub) if sub else cat
        moves.append((name, os.path.join(target, name)))
    return moves, skipped


def main():
    apply_ = "--apply" in sys.argv
    moves, skipped = plan()

    per_cat = {}
    for old, new in moves:
        head = new.split(os.sep)[0] if os.sep in new else new
        per_cat.setdefault("/".join(new.split(os.sep)[:-1]), 0)
        per_cat["/".join(new.split(os.sep)[:-1])] += 1

    print("=== 归类计划 ===")
    print("待移动: %d 个   保留: %d 个" % (len(moves), len(skipped)))
    print("\n-- 各分类文件数 --")
    for k in sorted(per_cat):
        print("  %-34s %3d" % (k or "(根)", per_cat[k]))
    print("\n-- 保留在根目录 --")
    for name, why in skipped:
        print("  %-40s %s" % (name, why))

    plan_path = os.path.join(ROOT, "_reorg", "reorg_plan_latest.txt")
    os.makedirs(os.path.dirname(plan_path), exist_ok=True)
    with open(plan_path, "w", encoding="utf-8") as fh:
        fh.write("old -> new 全量映射（%d 条）\n" % len(moves))
        for old, new in moves:
            fh.write("%s\t->\t%s\n" % (old, new))
    print("\n完整映射表已写入: %s" % plan_path)

    if not apply_:
        print("\n（干跑结束，未移动任何文件。加 --apply 执行）")
        return 0

    errors = []
    for old, new in moves:
        src = os.path.join(ROOT, old)
        dst = os.path.join(ROOT, new)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if os.path.exists(dst):
            errors.append((old, "目标已存在，跳过"))
            continue
        try:
            shutil.move(src, dst)
        except Exception as exc:
            errors.append((old, str(exc)))

    manifest = {"moved": [{"from": o, "to": n} for o, n in moves],
                "kept": [{"name": n, "reason": w} for n, w in skipped],
                "errors": errors}
    with open(os.path.join(ROOT, "_reorg", "reorg_manifest.json"), "w",
              encoding="utf-8") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=2)
    print("\n已移动 %d 个；错误 %d 个" % (len(moves) - len(errors), len(errors)))
    for o, e in errors:
        print("  !! %s: %s" % (o, e))
    print("还原依据: _reorg_manifest.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
