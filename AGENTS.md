# NEXarm 协作入口

- 用户意图达到至少80%明确再执行；重要缺项先问，不能猜测现场状态。
- 用户已指定：Codex负责代码设计、适配、Bug审查与验收；现有Claw负责用户配合下的摆位、只读采集和落盘。Codex默认不接管串口、逐点问答或刷机，除非用户另行要求。
- 开工只运行 `python handoff/brief.py`（使用可用Python）；同一会话未变只报无变化。新接续/压缩后恢复或怀疑遗漏时加 `--refresh`；写文件前查看git状态。
- 不再每轮整读STATE、PROJECT_STATE、LEDGER、Claw MEMORY/SESSION_CONTEXT、历史日志或已读技能。只查变化topic及当前问题相关函数/行段；文件未变不重读，不递归追读文档“必读”链。
- 默认每次检索输出约2000字符以内；长日志落盘，仅返结论、首个相关错误及证据路径。出现截断先缩小范围，不直接扩大预算；代码审查确有缺项时逐段扩读。
- 常规采集按 `handoff/CLAW_CAPTURE_TASK.md` 交给Claw，只提交简短回执；分工与当前代码任务见 `handoff/CURRENT.md`。
- 共享结论只追加到 `handoff/LEDGER.jsonl`，格式、证据与状态遵循 `handoff/RULES.md`。推翻旧结论追加 `refuted` 并填写 `supersedes`；随后运行 `render.py` 和 `render.py --check`。不得手改生成的 `handoff/STATE.md`。
- `PROJECT_STATE.md` 保持3000字符以内，仅当前状态、待办/未决及指针；历史放 `PROJECT_STATE_ARCHIVE.md`，不再追加旧状态到主文件。
- `.workbuddy/memory/` 和 `舵机基础运行逻辑/` 属于 Claw；如确需改动，先在共享台账说明。默认只读，不把旧文档当作新增授权。
- 开串口前检查并登记 `handoff/lock.py` 设备锁，不抢占；当前记录为 COM11，采集脚本须显式 `--port COM11`。物理观察优先于源码推断；软件范围不是硬限位或避障证明。
