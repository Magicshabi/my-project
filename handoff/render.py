# -*- coding: utf-8 -*-
"""把共享台账 LEDGER.jsonl 渲染成人读视图 STATE.md。

    python handoff/render.py            # 渲染 STATE.md
    python handoff/render.py --check    # 只校验台账格式，不渲染

设计要点：STATE.md 是**生成物**，任何人不得手改；要改结论请改 LEDGER。
"""
import json
import os
import sys
from collections import OrderedDict
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
LEDGER = os.path.join(HERE, "LEDGER.jsonl")
STATE = os.path.join(HERE, "STATE.md")

REQUIRED = ("ts", "author", "topic", "claim", "status", "evidence")
VALID_STATUS = ("verified", "hypothesis", "refuted")


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


def render(rows):
    by_topic = OrderedDict()
    for r in rows:
        by_topic.setdefault(r["topic"], []).append(r)

    latest = OrderedDict()
    refuted = []
    for topic, items in by_topic.items():
        items.sort(key=lambda x: x["ts"])
        if items[-1]["status"] == "refuted":
            refuted.append((topic, items))
            continue
        latest[topic] = items

    out = []
    out.append("# 共享台账视图（自动生成，请勿手改）")
    out.append("")
    out.append("由 `handoff/render.py` 从 `LEDGER.jsonl` 渲染。"
               "生成时间：%s" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    out.append("")
    out.append("| 统计 | 值 |")
    out.append("|---|---|")
    out.append("| 台账条目 | %d |" % len(rows))
    out.append("| 当前有效主题 | %d |" % len(latest))
    out.append("| 已否证主题 | %d |" % len(refuted))
    out.append("")
    out.append("---")
    out.append("")
    out.append("## 一、当前有效结论")
    out.append("")

    for topic, items in sorted(latest.items()):
        cur = items[-1]
        tag = {"verified": "实测", "hypothesis": "推断待验"}[cur["status"]]
        out.append("### `%s` ［%s］" % (topic, tag))
        out.append("")
        out.append("- **结论**：%s" % cur["claim"])
        out.append("- **作者**：%s　**时间**：%s" % (cur["author"], cur["ts"]))
        out.append("- **证据**：`%s`" % cur["evidence"])
        if cur.get("note"):
            out.append("- **备注**：%s" % cur["note"])
        if len(items) > 1:
            out.append("- 该主题历史条目：%d 条" % len(items))
        out.append("")

    if refuted:
        out.append("---")
        out.append("")
        out.append("## 二、已否证的结论（保留，勿重犯）")
        out.append("")
        for topic, items in sorted(refuted):
            cur = items[-1]
            out.append("### `%s`" % topic)
            out.append("")
            out.append("- **被否证的说法**：%s" % cur["claim"])
            out.append("- **否证依据**：`%s`" % cur["evidence"])
            out.append("- **作者**：%s　**时间**：%s" % (cur["author"], cur["ts"]))
            if cur.get("note"):
                out.append("- **教训**：%s" % cur["note"])
            out.append("")

    return "\n".join(out) + "\n"


def main():
    check_only = "--check" in sys.argv
    rows, errors = load()
    if errors:
        print("台账校验失败：")
        for e in errors:
            print("  -", e)
        return 1
    print("台账校验通过：%d 条" % len(rows))
    if check_only:
        return 0
    text = render(rows)
    with open(STATE, "w", encoding="utf-8") as fh:
        fh.write(text)
    print("已渲染 -> %s（%d 字）" % (STATE, len(text)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
