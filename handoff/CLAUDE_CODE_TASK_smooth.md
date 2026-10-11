# Claude Code 任务：平滑整程空载试跑（2026-10-11）

仓库：`A:\NEXarm\Claude`。用户赶时间：除每次运行后问一句“顺不顺”外，不追加确认问题。

## 背景（只需知道这些）
- 八点路线已由 Codex 分段跑通（台账 topic `replay-setup`、`empty-cycle`）。问题是动作顿、冲。
- 新程序 `八点推进_20261008/smooth_route.py`：每段每轴只写一次 CMD3/reg41，按行程分配速度/加速度做同步梯形，六轴同起同停，到位即接下一段；夹爪事件只动ID6；越界/超时则停后续段并写原地保持。离线测试见 `test_smooth_route.py`。
- 计划文件（绑定路线/限位/程序SHA）：`review_20261011/smooth_cycle_{p8,p1}_{gentle,normal}.json`。p8=从第8点起（先高位转底座→P1→…→P8），p1=从第1点起。程序会核对起点，偏差>24刻度拒绝运行。
- 加速度寄存器单位按100刻度/s²推算，未实机验证 → 先 gentle，再 normal。

## 规则
- 用采集时的 Python：`C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`，pyserial 在 `.runtime_deps/serial_capture`（加入 PYTHONPATH）。NexArm.exe 不能占着 COM11。
- 串口前取锁、结束释放：`python handoff/lock.py acquire|release COM11 --holder claude`。锁被别人有效持有就停手告诉用户。
- 结论只追加到 `handoff/LEDGER.jsonl`（author `claude`，topic `smooth-replay`），然后 `python handoff/render.py` 和 `python handoff/render.py --check`。不手改 STATE.md，不动 `.workbuddy/memory/`、`舵机基础运行逻辑/`。
- 不带载；程序报 STOP/error 时不重跑同一计划、不换脚本补动作，先把首个错误和 RUN_DIR 告诉用户。

## 步骤
1. `cd 八点推进_20261008`；`python -m unittest test_smooth_route test_move_reviewed_legs test_run_empty_route test_replay_joint_points` 全部通过。
2. `python smooth_route.py --dry-run p8 --profile gentle`（看一眼时间表）。
3. 取锁，运行（机械臂在P1就把 p8 换 p1）：
   `python smooth_route.py --port COM11 --holder claude --plan review_20261011/smooth_cycle_p8_gentle.json --go --empty-load`
   运行完出现 OBSERVE 时问用户“顺不顺、到位对不对”，用户说好输入 `yes`，否则原样输入用户的话。
4. 若 gentle 被认可：同样方式跑 `smooth_cycle_p8_normal.json`（起点须回到P8；gentle 跑完正好停在P8）。
5. 释放锁；台账追加一条结果（两次 RESULT 的 total_s、用户评价、RUN_DIR），render + --check。
6. 若用户指出某段仍顿/冲：读该次 `logs/smooth_*/events.jsonl` 中该段的 step_begin/sample，调整 `smooth_route.py` 的 PROFILES / CAREFUL / ARRIVE_TOL 并补测试；改程序后计划文件里的 runner SHA 需重算（`pins`），否则程序会拒绝运行。
7. 可选收尾：`git status`；今天未提交的成果分两次提交——先提交下午Codex的运行日志与评审文件，再提交 Claude 的工具（read_pose、move_reviewed_legs、restore_torque、watch_pose、smooth_route 及测试、lock.py/replay/reposition 修改、review_20261011 下的计划JSON）。提交信息写清做了什么/为什么。
