# BDS300 舵机初始化候选包 — 2026-10-15

状态：软件测试与构建通过，未完成实车校准和赛道验收。

- 分支：`codex/competition-servo-init`，基于 `camera` 的 `818248b1ae579369a1ebc229b4fe191a255ab3f4`。
- 源码目录：[competition/firmware/camera_car](../firmware/camera_car/README.md)。
- 初始化说明：[servo-init.md](../docs/servo-init.md)。
- 压缩包：[camera-car-servo-init-20261015.zip](camera-car-servo-init-20261015.zip)。
- 包大小：3491815 字节。
- SHA-256：`a2c7302ecab3cbe0e280809b41f5c7d07e678b9ba6cb95822f806376a17cc8ce`。
- 清单：[逐文件校验](servo-init-20261015-manifest.json)。包内有同一份 RELEASE-MANIFEST.json。
- 软件验证：[测试日志](../evidence/servo-init-20261015/host-tests.log)，[真实摄像头构建](../evidence/servo-init-20261015/build-camera.log)，[模拟构建](../evidence/servo-init-20261015/build-mock.log)。

原有上电回中代码已整理为 car_servo.c/.h：C30、50 Hz、中位1500 us，允许范围1350～1650 us。前轮实际中位与限位需要实测。单独调用 car_servo_init() 不初始化摄像头；完整程序仍按真实摄像头模式启动。

解压后打开 camera_car/demo/mdk/rt1064.uvprojx。源码编译器 ArmClang V6.21、器件包 NXP.MIMXRT1064_DFP.19.0.0。预编译文件在 prebuilt/camera/rt1064.hex；prebuilt/mock/rt1064_mock.hex 是禁止电机的模拟测试版本，不能混淆。选择下载器及烧录算法的步骤见包内 README。

包内不含工具缓存、个人调试器设置和无关资料。源代码与逐飞/NXP 许可证一并保留。本版本不修改 main 或 camera 分支，后续在当前分支完成实机验收。
