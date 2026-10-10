import contextlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import unittest
import uuid

spec = importlib.util.spec_from_file_location("brief", Path(__file__).with_name("brief.py"))
brief = importlib.util.module_from_spec(spec)
spec.loader.exec_module(brief)


class BriefTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / (".test-brief-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup)
        (self.root / "handoff").mkdir()
        (self.root / "handoff/CURRENT.md").write_text("Codex审查，Claw采集。", encoding="utf-8")
        route = self.root / brief.ROUTE
        route.parent.mkdir(parents=True)
        route.write_text(json.dumps(dict(points=[1, 2], complete=False, execution_verified=False)), encoding="utf-8")
        self.ledger = self.root / "handoff/LEDGER.jsonl"
        self.ledger.write_text("", encoding="utf-8")
        self.add_row("path-validation", "尚未验证路径", "hypothesis")

    def cleanup(self):
        target = self.root.resolve()
        assert target.parent == Path(__file__).resolve().parent and target.name.startswith(".test-brief-")
        shutil.rmtree(target)

    def add_row(self, topic, claim, status="verified"):
        row = dict(topic=topic, claim=claim, status=status, ts="2026-10-11T01:00:00+08:00")
        with self.ledger.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    def run_brief(self, *args):
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            code = brief.main(["--session", "fixture", *args], root=self.root)
        return code, stream.getvalue()

    def test_unchanged_short_and_refresh_restores(self):
        code, full = self.run_brief()
        self.assertEqual(code, 0)
        self.assertIn("2/8", full)
        self.assertIn("尚未验证路径", full)
        code, short = self.run_brief()
        self.assertEqual(code, 0)
        self.assertIn("无变化", short)
        self.assertLess(len(short), 100)
        self.assertEqual(self.run_brief("--refresh")[1], full)

    def test_route_change_invalidates_cache(self):
        self.run_brief()
        (self.root / brief.ROUTE).write_text(json.dumps(dict(points=[1, 2, 3], complete=False)), encoding="utf-8")
        self.assertIn("3/8", self.run_brief()[1])

    def test_unfocused_new_topic_is_not_silently_hidden(self):
        self.run_brief()
        self.add_row("new-blocker", "need inspection")
        self.assertIn("new-blocker", self.run_brief()[1])

    def test_refuted_topic_is_visible(self):
        self.add_row("unsafe-rule", "old wrong claim", "refuted")
        self.assertIn("已否证（勿重犯）：unsafe-rule", self.run_brief()[1])

    def test_corrupt_input_cannot_report_unchanged(self):
        self.run_brief()
        self.ledger.write_text("bad json", encoding="utf-8")
        code, text = self.run_brief()
        self.assertEqual(code, 1)
        self.assertNotIn("上下文无变化", text)

    def test_oversized_context_fails_without_partial_safety_claims(self):
        (self.root / "handoff/CURRENT.md").write_text("长" * 2000, encoding="utf-8")
        code, text = self.run_brief()
        self.assertEqual(code, 1)
        self.assertIn("不自动截断", text)


if __name__ == "__main__":
    unittest.main()
