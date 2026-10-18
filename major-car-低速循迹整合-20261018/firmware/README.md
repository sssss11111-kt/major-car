# 本轮构建产物｜20261018T

- `20261018T-camera.hex`：真实摄像头、受监护的低速循迹候选。
- `20261018T-camera.axf`：对应ARM调试/链接产物。
- `20261018T-mock.hex`：模拟图像，禁用电机，不能验证真实相机或运动。
- `20261018T-diag.hex`：通信诊断，禁用电机。
- `SHA256SUMS.txt`：以上文件的SHA-256。

构建工具为本机已有ArmClang6.7；沿用原工程Flash起始0x70000000、4MiB及DCD/SDRAM布局。校验每条Intel HEX记录的长度、校验和、地址范围和EOF；数据字节数与SHA在../records/FINAL-build-summary.json。

本轮仅生成和校验文件，**未烧录、未在实车运行**。软件测试和构建证据不等同硬件/赛道验收。现场顺序见../实车验收清单.md。mock/diag不能替代camera候选。
