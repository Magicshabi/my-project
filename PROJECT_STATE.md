# NEXarm 当前状态

<!-- NEXARM-HANDOFF-BEGIN PH-20261011-eight-point-03 -->
## 当前状态

最新接续：整程空载验证暂停于P1附近：反馈[1618,1649,2564,3086,2704,1457]，ID3目标2586、误差22超过20刻度，未开始P1→2；串口和锁已释放。等待用户判断是否接受实物起点。底座容差传递已修复，新运行入口45项测试通过，原始失败日志保留。实时查empty-cycle主题，整程尚未通过。

更新：2026-10-11。第一目标：固定红篮从上方夹提手并搬到固定T0，先保证正确放置，再优化连贯性和速度，视觉后置。

Claw已完成本轮采集交付；Codex负责代码适配、Bug检查、轨迹审查与验收。新路线离线审查通过，执行标记由`python handoff/brief.py`读取[数据JSON](八点推进_20261008/data/eight_joints_powered_20261009.json)及台账；旧八点不混入本轮。用户另行授权Codex执行空载回位及单段试验，实时进展见[本轮记录](八点推进_20261008/review_20261011/README.md)。

六轴单独动作/回位已实机验证；整段路径、带载搬运、连续平滑及物理急停仍待验。端口COM11，现场状态以负责测试方当次确认与设备锁为准。

[技术方案](八点推进_20261008/固定场地搬运_示教与轨迹规划_20261010.md)已精简，原文完整归档。电池STEP已核验为有效实体，外形70×80×80毫米；详见[测量记录](artifacts/model_battery_20261011/README.md)。用户说明四篮相邻、邻篮间距约10毫米；这是估计，非实测避障余量。

## 待办 / 未决问题

1. 本轮审查已接收Claw全部数据；后续补采仍按[采集任务卡](handoff/CLAW_CAPTURE_TASK.md)交Claw。Codex进行阶段、路径与执行代码验收。
2. 第4点倾斜为用户确认的越障策略，保留原点；结合夹爪与带篮包络验证中间净空，采集成功不代表整段路径通过。
3. 复用[已验证逻辑](舵机基础运行逻辑/MOTION_LOGIC_SPEC.md)及`A:\NEXarm\运动控制`案例。连续插值、控制器同步性和停机效果先验证，再逐步开展空载/带载测试。
4. GUI目标不等于反馈，软件范围不证明真实限位或避障；不写固定偏差补偿，不把历史单次拒动推为真实上限。保留[回放诊断锁](八点推进_20261008/replay_diagnostic_hold.json)，不裁剪旧数据后运行。

## 指针

- 开工：[AGENTS.md](AGENTS.md)、[brief.py](handoff/brief.py)；压缩后加`--refresh`，未变不重读。分工见[CURRENT](handoff/CURRENT.md)。
- 共享：[STATE](handoff/STATE.md)、[LEDGER](handoff/LEDGER.jsonl)、[RULES](handoff/RULES.md)；台账按topic查，只追加，STATE由render生成并校验。
- 证据：[六轴验证](八点推进_20261008/six_axis_verified_20261009.md)、[ID4对照](八点推进_20261008/id4_target_comparison_20261009.md)、[软件范围](八点推进_20261008/data/joint_limits_20261010.json)。
- 历史：[PROJECT_STATE_ARCHIVE](PROJECT_STATE_ARCHIVE.md)、[共享接入记录](handoff/adoption_20261011.md)、[原技术方案](archives/technical_plan/固定场地搬运_示教与轨迹规划_20261010_before_20261011.md)。仅按需定位，不整读。
<!-- NEXARM-HANDOFF-END PH-20261011-eight-point-03 -->
