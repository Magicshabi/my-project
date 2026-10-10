"""离线验证读取命令不会覆盖共享视图；不使用真实台账。"""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import unittest
import uuid
from unittest.mock import patch


spec = importlib.util.spec_from_file_location("handoff_render", Path(__file__).with_name("render.py"))
render = importlib.util.module_from_spec(spec)
spec.loader.exec_module(render)


class ReadOnlyQueryTests(unittest.TestCase):
    def setUp(self):
        # 当前 Windows 沙箱的系统临时目录不允许访问新建的 0700 目录。
        self.root = Path(__file__).resolve().parent / (".test-render-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.clean_fixture)
        self.ledger = self.root / "LEDGER.jsonl"
        self.state = self.root / "STATE.md"
        self.rows = [
            dict(ts="2026-10-10T10:00:00+08:00", author="codex", topic="older",
                 claim="older evidence", status="verified", evidence="old.json"),
            dict(ts="2026-10-11T10:00:00+08:00", author="codex", topic="newer",
                 claim="newer evidence", status="verified", evidence="new.json"),
        ]
        self.ledger.write_text("".join(json.dumps(row) + "\n" for row in self.rows), encoding="utf-8")
        self.state.write_bytes(b"full view must survive\r\n")

    def clean_fixture(self):
        target = self.root.resolve()
        assert target.parent == Path(__file__).resolve().parent
        assert target.name.startswith(".test-render-")
        shutil.rmtree(target)

    def call(self, *args):
        output = io.StringIO()
        with patch.object(render, "LEDGER", str(self.ledger)), \
             patch.object(render, "STATE", str(self.state)), \
             patch.object(render.sys, "argv", ["render.py", *args]), \
             contextlib.redirect_stdout(output):
            code = render.main()
        return code, output.getvalue()

    def test_since_is_read_only_with_matches(self):
        before = self.state.read_bytes()
        code, output = self.call("--since", "2026-10-11T00:00:00+08:00")
        self.assertEqual(code, 0)
        self.assertIn("`newer`", output)
        self.assertNotIn("`older`", output)
        self.assertEqual(self.state.read_bytes(), before)

    def test_since_is_read_only_without_matches(self):
        before = self.state.read_bytes()
        code, output = self.call("--since", "2030-01-01T00:00:00+08:00")
        self.assertEqual(code, 0)
        self.assertIn("无新增条目", output)
        self.assertEqual(self.state.read_bytes(), before)

    def test_check_does_not_write_view(self):
        before = self.state.read_bytes()
        self.assertEqual(self.call("--check")[0], 0)
        self.assertEqual(self.state.read_bytes(), before)

    def test_default_render_keeps_all_topics(self):
        self.assertEqual(self.call()[0], 0)
        text = self.state.read_text(encoding="utf-8")
        self.assertIn("`older`", text)
        self.assertIn("`newer`", text)


if __name__ == "__main__":
    unittest.main()
