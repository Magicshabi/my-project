# NEXarm 协作入口

- 用户意图达到至少80%明确再执行；重要缺项先问，不能猜测现场状态。
- 每次开工先读 `handoff/STATE.md` 与 `.workbuddy/memory/MEMORY.md`，尤其摘要的“已否证（勿重犯）”；查看 `git status --short`，防止覆盖并行修改。
- 不整读 `PROJECT_STATE.md`、`handoff/LEDGER.jsonl` 或历史日志。当前阶段只读 `PROJECT_STATE.md` 的“当前状态”节；细节按主题用 `rg`，增量用 `python handoff/render.py --since <ISO时间>`。
- 共享结论只追加到 `handoff/LEDGER.jsonl`，格式、证据与状态遵循 `handoff/RULES.md`。推翻旧结论追加 `refuted` 并填写 `supersedes`；随后运行 `render.py` 和 `render.py --check`。不得手改生成的 `handoff/STATE.md`。
- `PROJECT_STATE.md` 保持3000字符以内，仅当前状态、待办/未决及指针；历史放 `PROJECT_STATE_ARCHIVE.md`，不再追加旧状态到主文件。
- `.workbuddy/memory/` 和 `舵机基础运行逻辑/` 属于 Claw；如确需改动，先在共享台账说明。默认只读，不把旧文档当作新增授权。
- 开串口前检查并登记 `handoff/lock.py` 设备锁，不抢占；当前记录为 COM11，采集脚本须显式 `--port COM11`。物理观察优先于源码推断；软件范围不是硬限位或避障证明。
