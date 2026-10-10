# -*- coding: utf-8 -*-
"""把共享台账 LEDGER.jsonl 渲染成**极简**人读视图 STATE.md（默认省 token 模式）。

    python handoff/render.py                 # 默认：紧凑摘要表（目标 < 1KB）
    python handoff/render.py --since 2026-10-10T00:00+08:00   # 只看新增条目
    python handoff/render.py --full          # 全量详版（费 token，仅在需要时）
    python handoff/render.py --check         # 只校验台账格式

设计：写入用「追加」（低冲突），读取用「摘要」（省 token）—— 两者分离。
STATE.md 是生成物，任何人不得手改。
"""
import json
import os
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
LEDGER = os.path.join(HERE, "LEDGER.jsonl")
STATE = os.path.join(HERE, "STATE.md")
REQUIRED = ("ts", "author", "topic", "claim", "status", "evidence")
VALID_STATUS = ("verified", "hypothesis", "refuted")
MAX_CLAIM = 34          # 紧凑模式每条结论的截断长度


def load():
    rows, errors = [], []
    with open(LEDGER, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            raw = raw.strip()
            if not raw or raw.startswith("#"):
                continue
            try:
                obj = json.loads(raw)
            except json.JSONDecodeError as exc:
                errors.append("第 %d 行 JSON 非法: %s" % (lineno, exc))
                continue
            for field in REQUIRED:
                if not obj.get(field):
                    errors.append("第 %d 行缺字段 %s" % (lineno, field))
            if obj.get("status") not in VALID_STATUS:
                errors.append("第 %d 行 status 非法: %r" % (lineno, obj.get("status")))
            rows.append(obj)
    return rows, errors


def shorten(text, limit=MAX_CLAIM):
    text = text.strip()
    return text if len(text) <= limit else text[:limit - 1] + "…"


def pick_latest(rows):
    """每个 topic 取最新一条；最新为 refuted 的归入否证组。"""
    by_topic = {}
    for r in rows:
        by_topic.setdefault(r["topic"], []).append(r)
    live, dead = [], []
    for topic, items in by_topic.items():
        items.sort(key=lambda x: x["ts"])
        (dead if items[-1]["status"] == "refuted" else live).append((topic, items))
    return sorted(live), sorted(dead)


def header(rows, live, dead, tag):
    return ("# 共享台账摘要 · %s | %d 条 | 有效 %d | 否证 %d\n"
            "> 只读本文件即可开工；细节用主题名 grep `handoff/LEDGER.jsonl`。"
            "全量详版：`render.py --full`\n\n" %
            (datetime.now().strftime("%m-%d %H:%M"), len(rows), len(live), len(dead)))


def render_compact(rows):
    live, dead = pick_latest(rows)
    out = [header(rows, live, dead, "compact")]
    out.append("| topic | 结论 | 状态 |")
    out.append("|---|---|---|")
    for topic, items in live:
        cur = items[-1]
        flag = "实测" if cur["status"] == "verified" else "待验"
        out.append("| `%s` | %s | %s |" % (topic, shorten(cur["claim"]), flag))
    out.append("")
    if dead:
        out.append("**已否证（勿重犯）**：%s" % "、".join("`%s`" % t for t, _ in dead))
    out.append("")
    out.append("证据与备注查阅：`grep <topic> handoff/LEDGER.jsonl`")
    return "\n".join(out) + "\n"


def render_full(rows):
    live, dead = pick_latest(rows)
    out = [header(rows, live, dead, "full"), "## 一、当前有效结论\n"]
    for topic, items in live:
        cur = items[-1]
        out.append("### `%s` ［%s］" % (topic, "实测" if cur["status"] == "verified" else "推断待验"))
        out.append("- 结论：%s" % cur["claim"])
        out.append("- 作者/时间：%s %s" % (cur["author"], cur["ts"]))
        out.append("- 证据：`%s`" % cur["evidence"])
        if cur.get("note"):
            out.append("- 备注：%s" % cur["note"])
        out.append("")
    if dead:
        out.append("## 二、已否证的结论（保留，勿重犯）\n")
        for topic, items in dead:
            cur = items[-1]
            out.append("### `%s`" % topic)
            out.append("- 被否证的说法：%s" % cur["claim"])
            out.append("- 否证依据：`%s`" % cur["evidence"])
            if cur.get("note"):
                out.append("- 教训：%s" % cur["note"])
            out.append("")
    return "\n".join(out) + "\n"


def main():
    argv = sys.argv
    rows, errors = load()
    if errors:
        print("台账校验失败：")
        for e in errors:
            print("  -", e)
        return 1
    print("台账校验通过：%d 条" % len(rows))
    if "--check" in argv:
        return 0

    if "--since" in argv:
        since = argv[argv.index("--since") + 1]
        rows = [r for r in rows if r["ts"] > since]
        print("过滤后：%d 条（ts > %s）" % (len(rows), since))
        text = render_compact(rows) if rows else "# 无新增条目\n"
    elif "--full" in argv:
        text = render_full(rows)
    else:
        text = render_compact(rows)

    with open(STATE, "w", encoding="utf-8") as fh:
        fh.write(text)
    print("已渲染 -> STATE.md（%d 字）" % len(text))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
