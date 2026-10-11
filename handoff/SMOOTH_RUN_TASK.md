# 带篮平滑整程任务卡（smooth_route.py，2026-10-11）

仓库 `A:\NEXarm\Claude`。任务：用 `smooth_route.py` 跑带篮整程（gentle）。每圈跑完只问用户一句“这圈怎样”，其余不追加确认。

## 谁能执行
- `--holder` 目前只接受 `claude`、`codex`。Codex/Claude 可直接执行。
- Claw 可读本卡了解流程与现场配合，但**暂不能执行**：`claw` 不在允许列表，且现行分工中 Claw 负责采集。需用户决定后改代码与 AGENTS.md 分工。

## 环境
- Python：`C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`
- `PYTHONPATH=A:\NEXarm\Claude\.runtime_deps\serial_capture`；工作目录 `八点推进_20261008`；NexArm.exe 不能占 COM11。

## 开跑前
1. `python handoff/brief.py`；离线测试 `python -m unittest test_smooth_route` 必须全过。
2. 取锁：`python handoff/lock.py acquire COM11 --holder <你> --minutes 60 --purpose "带篮整程"`。锁被别人有效持有就停手告诉用户。
3. 机械臂须在初始位置（`data/home_pose_20261011.json`）或 P8；程序自动判断，两处都不在会拒绝，不要用别的脚本去追。
4. 请用户把红篮放到抓取位、站到断电开关旁，用户回“好了”再跑。

## 每圈
```
python smooth_route.py --port COM11 --holder <你> --plan review_20261011/smooth_cycle_home_gentle_loaded.json --go --loaded
```
- 路线：[P8→初始位置]→初始位置转底座到P1方位→P1→P2→P3合爪(夹紧+200)→P4→P5→P6→P7张爪→P8。
- 出现 OBSERVE 时问用户“这圈怎样”：用户确认没问题才输入 `yes`，否则原样输入用户的话（注意不要带 BOM）。
- 下一圈前请用户把篮子放回抓取位。
- 程序报 STOP/error：不重跑同一计划、不换脚本补动作，先把首个错误和 RUN_DIR 告诉用户。

## 禁止（需要先问用户）
- 改 `smooth_route.py` 及参数（GRIP_SQUEEZE、PROFILES、容差等）、重算计划 pins、改初始位置文件、跑 normal 档带载、git 提交。
- 不动 `.workbuddy/memory/`、`舵机基础运行逻辑/`；不手改 `handoff/STATE.md`。

## 收尾
- 释放锁：`python handoff/lock.py release COM11 --holder <你>`
- 结果追加到 `handoff/LEDGER.jsonl`（author=自己，topic=`smooth-replay`，写清各圈 total_s、用户评价、RUN_DIR），再运行 `python handoff/render.py` 和 `python handoff/render.py --check`。

## 已知限制
避障和急停均未验证，出问题只能断电；夹紧量200由用户指定，ID6每圈持续顶住约7秒；初始位置↔P1/P8为关节直线，无避障证明。
