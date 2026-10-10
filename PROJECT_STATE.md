# NEXarm 当前状态

<!-- NEXARM-HANDOFF-BEGIN PH-20261011-eight-point-02 -->
## 当前状态

更新：2026-10-11；材料已核对，无新接手对话。已完成共享接入与上下文精简，未连接设备。用户新分工：Claw负责常规采集，Codex负责代码适配、Bug审查和路径方案；Claw接手待回执。

第一目标：八点主动示教与低速验证，完成固定红篮从上方夹提手、搬运至固定T0；先保证放置正确，再优化速度，视觉后置。

**新路线2/8，未回放。** 用户用NexArm.exe保持力矩开启、关节滑块摆位，再退出上位机，只读采集。当前六轴原始刻度：

| 点 | 姿态 | ID1～6 |
|---|---|---|
| 1 | 提手上方张开 | 1607,1653,2586,3084,2707,1461 |
| 2 | 下降到提手两侧，未夹紧 | 1612,1557,2622,3088,2675,1071 |

两点各8组稳定反馈；用户确认第2点ID6是主动张大。数据见[新路线](八点推进_20261008/data/eight_joints_powered_20261009.json)，以points数组为准（method_status仍有旧零点描述）。complete和execution_verified均为false；旧8/8不是当前进度。

最后记录串口COM11、1Mbps，读取后已关闭；ESP1.0.2/AT32 1.0.42为10月10日报告。现场供电、姿态与占用未在本轮重查。六轴单独动作/回位已验证，整段搬运未验证。

## 待办 / 未决问题

1. 由Claw按[采集任务卡](handoff/CLAW_CAPTURE_TASK.md)接续，Codex不逐点占用串口。等用户明确新第3点摆好：ID1～5保持，仅ID6慢慢夹稳，篮子留桌面；退出上位机、保持供电且无异常后只读采集。目前没有完成确认。
2. 使用[capture_joint_points.py](八点推进_20261008/capture_joint_points.py)，明确`--port COM11 --point 3 --powered-sliders-confirmed`；开串口先查占用锁。默认COM9不可沿用。
3. 复用[舵机基础逻辑](舵机基础运行逻辑/MOTION_LOGIC_SPEC.md)与`A:\NEXarm\运动控制`案例。完成8点后按空载分段、低速带载顺序验证，逐点到位不等于平滑避障。
4. ID4的3413是GUI目标，反馈3084已获用户认可；真实内部限位未读出，不写329固定补偿。“3090～3234真实上限”超出历史对照证据范围，已在台账更正。
5. 软件范围仅作拒绝检查，不证明硬限位或路径安全。保持[回放诊断锁](八点推进_20261008/replay_diagnostic_hold.json)；旧数据不裁剪后运行，物理急停和连续平滑效果仍待验。

## 指针

- 开工规则：[AGENTS.md](AGENTS.md)；只运行[brief.py](handoff/brief.py)，压缩后加`--refresh`。未变不重读背景；[当前分工](handoff/CURRENT.md)与[共享摘要](handoff/STATE.md)按需查。
- [共享台账](handoff/LEDGER.jsonl)：按topic用rg查询；[写入规则](handoff/RULES.md)。仅追加，生成摘要后运行`render.py --check`；`--since`只输出增量。
- [完整历史归档](PROJECT_STATE_ARCHIVE.md)、[归档核验与回填记录](handoff/adoption_20261011.md)。还原点`e459b5e`。不得整读历史作为开工步骤。
- [六轴实测](八点推进_20261008/six_axis_verified_20261009.md)、[ID4对照](八点推进_20261008/id4_target_comparison_20261009.md)、[软件限位](八点推进_20261008/data/joint_limits_20261010.json)。

<!-- NEXARM-HANDOFF-END PH-20261011-eight-point-02 -->
