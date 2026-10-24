# LAP11 自动发车版

## 操作

把车放到已标定姿态的赛道上，复位或重新上电。程序在核心板内完成：

1. 等待连续至少5帧的可靠双边赛道和宽度匹配，图像新鲜度小于150ms。
2. 舵机持续回到现有中位命令92.5，电机输出保持0，非阻塞等待100个10ms周期。
3. 1秒内持续满足识别条件后自动启动电机，转入原中线循迹控制。

不需要电脑发送RUN，不需要电脑持续发送K。串口STOP仍可停车；BENCH调试仍保留保活租约和2秒上限。

等待期间丢线、宽度不匹配、图像变旧或执行SNAP，会取消倒计时，重新稳定识别后重新等待1秒。故障停车后不会自动再次启动，需复位或重新上电。

“回中”指发送现有舵机中位PWM命令，无前轮角度传感器，因此不是机械角度闭环确认。若机械中位不直，需要单独校准中位参数。

## 显示

- AUTO WAIT：等待可靠赛道。
- CENTER 1s：前轮中位，1秒倒计时。
- AUTO RUN：自动循迹中。
- STOP RESET：停车锁定，需复位。

无线新增AUTO11：phase=0/1/2/3对应上述状态，wait_ms为剩余等待时间。运行遥测版本标识为C11LAP11。

## 保留的控制边界

继续使用本次直道标定宽度：第90行142像素。赛道宽度或摄像头安装姿态改变仍需重新标定。保持原转向系数、速度目标、单边补线及0.5秒单边上限、图像失效停车、编码器异常和堵转停车、30%驱动输出上限与单次120秒运行上限。

无线诊断输出在RTS忙时跳过，不等待模块流控；停车SNAP抓图仍使用原完整发送方式，并暂停自动发车倒计时。

## 工程与验证

- 工程：code/demo/mdk/rt1064.uvprojx。
- 启动状态机：code/demo/user/src/auto_start.c、inc/auto_start.h。
- 硬件控制集成：code/demo/user/src/main.c的trial_tick。
- 状态机测试：code/verification/test-auto-start.c；覆盖1秒边界、丢线重新计时、SNAP抑制、停车锁定和32位计时回绕。
- 实际trial_tick主机集成测试：code/verification/check-auto-integration.py及auto-integration.log；覆盖自动启动、无线未就绪与无保活、丢线、STOP锁定、SNAP、过期图像与BENCH租约。硬件I/O为主机桩，不能代替真实车轮与赛道验证。
- 最终构建：code/verification/build-auto-final.log。

code目录保留了来源工程的历史verification文件；旧trial.ps1是LAP10电脑手动发车工具，不用于本自动发车版。历史HANDOFF与报告不作为LAP11实测结论。本文件及本次生成的AUTO测试、构建和烧录记录描述LAP11状态。
