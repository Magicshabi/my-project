"""当前工作摘要；同一会话未变只报一行，不读历史或串口。"""
import argparse
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ROUTE = "八点推进_20261008/data/eight_joints_powered_20261009.json"
FOCUS = ("division-of-work", "powered-teaching", "path-validation",
         "emergency-stop", "limit-readback", "gui-limit-not-protection")
LIMIT = 1600


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def snapshot(root):
    paths = ("handoff/CURRENT.md", ROUTE, "handoff/LEDGER.jsonl")
    blobs = {name: (root / name).read_bytes() for name in paths}
    hashes = {name: hashlib.sha256(blob).hexdigest() for name, blob in blobs.items()}
    route = json.loads(blobs[ROUTE])
    latest = {}
    for number, line in enumerate(blobs[paths[2]].decode("utf-8-sig").splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        row = json.loads(line)
        if not all(row.get(k) for k in ("topic", "ts", "claim", "status")):
            raise ValueError(f"台账第{number}行缺字段")
        if row["status"] not in ("verified", "hypothesis", "refuted"):
            raise ValueError(f"台账第{number}行状态无效")
        prior = latest.get(row["topic"])
        if prior is None or row["ts"] >= prior["ts"]:
            latest[row["topic"]] = row
    if not isinstance(route["points"], list):
        raise ValueError("points不是数组")
    state = {"hashes": hashes, "topics": {t: digest(row) for t, row in latest.items()}}
    return blobs[paths[0]].decode("utf-8-sig"), route, latest, state


def brief_text(current, route, latest, previous):
    lines = [current.strip(), f"实时文件进度：{len(route['points'])}/8；"
             f"complete={route.get('complete')}；execution_verified={route.get('execution_verified')}。"]
    for topic in FOCUS:
        row = latest.get(topic)
        if row:
            claim = " ".join(row["claim"].split())
            lines.append(f"{topic}[{row['status']}]：{claim}")
    dead = sorted(t for t, row in latest.items() if row["status"] == "refuted")
    lines.append("已否证（勿重犯）：" + "、".join(dead))
    if previous:
        changed = [t for t, row in latest.items() if previous.get("topics", {}).get(t) != digest(row)]
        if changed:
            lines.append("台账变化topic（含未展开项）：" + "、".join(sorted(changed)))
    lines.append("细节按topic查台账；现场状态未读取。本摘要只供接续，不是实机执行许可。")
    text = "\n".join(lines) + "\n"
    if len(text) > LIMIT:
        raise ValueError(f"摘要{len(text)}字符超过{LIMIT}上限；缩短CURRENT或按topic查询，不自动截断")
    return text


def main(argv=None, root=ROOT):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--refresh", action="store_true", help="压缩/新接续时强制展示短摘要")
    ap.add_argument("--session", default=os.environ.get("CODEX_THREAD_ID"), help="无会话ID则不缓存")
    args = ap.parse_args(argv)
    try:
        current, route, latest, state = snapshot(root)
        cache = None
        previous = None
        if args.session:
            key = hashlib.sha256(args.session.encode()).hexdigest()[:32]
            cache = root / "handoff/.brief-cache" / (key + ".json")
            if cache.exists():
                try:
                    previous = json.loads(cache.read_text(encoding="utf-8"))
                    if not isinstance(previous, dict) or not isinstance(previous.get("topics"), dict):
                        previous = None
                except (OSError, ValueError):
                    pass
        if not args.refresh and previous == state:
            print("上下文无变化：沿用当前进度与分工；无需重读背景文件。")
            return 0
        text = brief_text(current, route, latest, previous)
        print(text, end="")
        if cache:
            cache.parent.mkdir(parents=True, exist_ok=True)
            cache.write_text(json.dumps(state, ensure_ascii=False), encoding="utf-8")
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"摘要不可用：{exc}。只按当前问题查证，不沿用缓存判断已同步。")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
