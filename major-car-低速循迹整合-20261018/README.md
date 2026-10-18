# 低速视觉循迹整合候选 T｜2026-10-18

本目录是从昨晚实测N版建立的独立修改副本。源码版本标识为 `20261018T`。本轮仅修改、测试、编译和保存；没有烧录、没有连接串口或驱动车辆。本候选的实车循迹尚未验收。

## 文件入口

- [逐步修改记录](修改记录.md)：STEP-01～STEP-06，每步目的、修改、检查和边界。
- [完整代码](code/README.md)：包含逐飞V3.5.5/NXP库、工程、测试及工具。
- Keil工程：`code/competition/firmware/camera_car/demo/mdk/rt1064.uvprojx`。
- 主要参数：`code/competition/firmware/camera_car/demo/user/inc/car_config.h`。
- [现场验收步骤](实车验收清单.md)：先机械/图像/静态w，后限时循迹。
- `firmware/`：本次构建产物、SHA-256和构建说明；均不是已实测固件。
- `records/`：测试失败与通过记录、最终构建、独立复核、文件差异与指纹。
- 当前合成图像报告：`code/competition/firmware/camera_car/tests/out/vision/report.html`。

## 已实现的默认行为

`a` 是一次受监护的循迹试验：至少3帧新鲜有效图像；立即按视觉误差转向；先保持电机零输出200ms；再最多行驶3秒。每100ms发送p查询，车端500ms没有收到续期即停车。停止必须重新明确启动，p不能重启车辆。

目标20计数/10ms。沿用昨晚落地控制的前馈1500、Kp4/Ki20作为初值；最高30%用于有界起转，非恒定行驶输出；每轮连续3次达到5计数/10ms确认起转，200ms内未起转停车。起转后进入速度PI。物理速度、低速稳定性和控制增益尚待实车验证。

`w` 保留为静态视觉转向：两电机零输出，最多30秒；丢线/图像过期/前台异常会结束。`s/S`仍在RX中断执行停车。其余独立电机/行驶测试命令保留，**也会产生真实运动**。

## 验证与使用

以下只测试、编译，不烧录、不打开串口：

```powershell
cd "D:\桌面\智能车\major-car-低速循迹整合-20261018\code"
.\repair\run-tests.cmd
python .\repair\build_arm.py --mode camera
```

测试入口按本机MSVC路径配置；构建入口使用现有 `D:\keil5\ARM\ARMCLANG\bin` 的ArmClang6.7。换机时调整路径。Keil工程原元数据6.21/Pack19保留，编译器版本不同不宣称逐字节重现。

完成现场准备与w静态验收后，可使用新配套工具。下面第一条只发送d/p查询；第二条明确发送a，车辆会运动。本轮没有执行这两条：

```powershell
python .\repair\tracking_trial.py --port COM6
python .\repair\tracking_trial.py --port COM6 --run
```

COM6只是昨晚本机端口，换机重查。工具使用pyserial，未随本轮安装；脚本运行前需已有该依赖。--run会检查T版标识、有效图像、待机及零PWM，每100ms续期，保存串口原字节、JSON和CSV，并在退出时尝试发s；车端租约独立生效。

## 来源与边界

基线：`sssss11111-kt/major-car` 的 `codex/flash-tests-20261017`，提交 `53d6dba300a88824b0fbc201af3e8d8aa9fce5d7`。本目录没有覆盖旧工程或修改远端分支。

昨晚已验证的是N版独立行驶和模块动作。本候选主机测试与ARM构建通过，不代表标准赛道、真实DMA时序、机械中位、负载速度和停车距离已通过。环岛、十字和提速不在本次范围。
