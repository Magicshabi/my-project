# r7k2：修正现场 KEY2 电平判断

本板现场确认：KEY1/GPIO0 按下低电平；KEY2/GPIO2 松开低、按下高。旧 r7 把两个按键均解释为低电平按下，导致 KEY2 松开显示 DOWN，KEY1 被两键重叠保护拦截。

修正：KEY2 使用 INPUT_PULLDOWN，高电平产生按下位；KEY1 保持 INPUT_PULLUP、低有效。日志 gpio0/gpio2 输出实际采样电平，OLED 显示归一化的 UP/DOWN。松开两键应显示 K1:UP K2:UP，程序等待 35 ms 稳定后 armed=true。短按松开生成事件；长按与真正的两键重叠仍不执行。

保存、NVS 格式、八点顺序及 CMD65-only 限制不变。无力矩/目标/回位/回放命令。只记录固件不能主动解除力矩，操作仍需托稳。

现场流程沿用 ../handle_teach_r7_20261004/README.md，但固件名称为 nexarm-teach-record-r7k2，OLED 首行为 R7K2 NO LOCK；teach_validation_r7.py 已要求此修正版，旧版本六轴报告不能用作本版本现场验证。

本次构建：../build_match_teach_record_r7k2.ps1；安装及现场结果见 ../KEY2极性修正_20261004.md。原 r7 源码与镜像保留，未覆盖。
