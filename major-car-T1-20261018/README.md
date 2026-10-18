# RT1064智能车 T1修复候选｜2026-10-18

本包修复T版采集缓冲交接及试验工具判定，版本标识为 `20261018T1`。完整源码、固件和验证记录均在本目录。**尚未烧录、尚未实车验证；不能据离线通过宣称赛道循迹通过。** 原T固件和N版历史归档保留在原仓库目录及Git记录，没有覆盖。

## 本次改动

1. CSI回调把刚完成的缓冲回交到对应完成寄存器，不再把另一块交回，避免正常交替采集中“发布块与下一帧目标相同”。保留前台复制前后的完成序号校验：复制期间换帧则丢弃，并等待后续新帧。
2. 固件输出 `ready`、`good_frames`、`accepted_age`；ready与真正启动共用判定，至少3个新鲜有效已处理帧才就绪。
3. 每次成功启动递增 `trial_id`；只有正常到期置 `trial_done=1`，并记录包括200ms准备期的 `trial_ms`。显式停车、故障及新启动清除结束标志。停车后的p不能重新启动车辆。
4. 工具仅接受T1。等待就绪后才允许一次显式a，不自动重发；观察到新试验ID、双轮闭环阶段的正反馈与正输出、同ID正常到期及归零回中后才记为 `completed`。提前停止、未起转、身份不符、故障和状态陈旧记为 `error`，退出尝试发s。
5. 每次工具会话保存 `result.json`、原串口字节、JSON和CSV。`completed`仅表示软件结束条件和反馈被观察到，仍需人工核对实际轨迹与停车。

接线、速度目标20计数/10ms、控制增益、PWM上限30%、200ms准备、最长3秒行驶和500ms通信租约保持原T配置。主板左DIR=C11 HIGH、右DIR=C9 LOW。没有升级逐飞V3.5.5或替换SDK。

## 文件入口

- `code/`：完整工程、逐飞/NXP依赖及原许可证。
- Keil工程：`code/competition/firmware/camera_car/demo/mdk/rt1064.uvprojx`。
- 参数：`code/competition/firmware/camera_car/demo/user/inc/car_config.h`。
- `firmware/20261018T1-camera.hex`：真实摄像头修复候选；AXF为相应调试产物。
- `firmware/20261018T1-mock.hex`、`20261018T1-diag.hex`：模拟/诊断，禁用电机；不能代替真实摄像头验收。
- `firmware/SHA256SUMS.txt`：预编译文件指纹；构建详情见 `records/build-summary.json`。
- `records/`：本次主机回归、三模式构建、真实C遥测、原代码失败及修复通过记录、源码清单和修改差异。源码新增文件已包含在code与manifest中，tracked-changes.patch仅覆盖Git已跟踪文件。
- [修复与验证记录](修改记录.md)、[现场验收清单](实车验收清单.md)。

代码目录README/HANDOFF中的旧N/T内容是历史背景；本包T1的最新状态以本README及本包records为准。

## 离线验证

在 `code` 目录执行（不烧录、不打开串口）：

```powershell
.\repair\run-tests.cmd
python .\repair\build_arm.py --mode camera
python .\repair\build_arm.py --mode mock
python .\repair\build_arm.py --mode diag
```

主机入口自动寻找现有MSVC环境；需要已有Python及Visual Studio C++工具。ARM脚本使用现有 `D:\keil5\ARM\ARMCLANG\bin`，换电脑需调整本地安装路径。本包实际构建日志显示MDK Professional5.39、Arm Compiler6.21。历史T/N交付使用6.7，不能宣称T1与历史文件逐字节相同。下载工具/私人原Flash备份/探针标识未随包分发。

本轮额外用已有Zig跑通第二套回归。可设置 `ZIG` 为已有zig.exe后执行 `python competition/firmware/camera_car/tests/run_tests.py`；不需要把旧D盘工程源码覆盖进本工程。

## 以后现场使用

先完成机械回中、真实图像和w静态联动验收，再安排短时低速循迹。未来使用本包固件时，d应输出 `TRACK-CFG v=20261018T1`；仍看到 `DRIVE-CFG v=20261018N` 是独立行驶子系统标签。旧T固件与T1脚本不能混用。

默认工具仅预检；显式 `--run` 才会发送运动命令。本轮未执行以下现场操作：

```powershell
python .\repair\tracking_trial.py --port COM6
# 下条会驱动车辆，必须先完成现场准备和静态验收：
python .\repair\tracking_trial.py --port COM6 --run
```

COM6只为历史端口示例，使用时重新确认。工具访问真实串口需已有pyserial；离线测试不需要pyserial。保护输出归零不是主动制动，实际停车距离必须实测。

## 证据边界

MSVC及Zig主机回归通过，三模式ARM构建成功且零告警。CSI测试提取实际SDK/设备回调/前台采集代码，验证256次交替完成的地址关系、复制期间换帧拒收及恢复；硬件寄存器和DMA时刻仍是主机替身，不认证实板极端中断延迟、相机消隐、负载供电或实际停车。真实赛道图像、控制效果及长时间运行仍待现场验证。
