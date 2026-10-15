# 来源与版本

- 官方库：https://gitee.com/seekfree/RT1064_Library
- 本工程固定使用本届《镜头组例程.zip》中附带的逐飞 RT1064 库，`libraries/doc/version.txt` 顶部版本 V3.5.5；未假定其为当前官方最新版本。
- 输入压缩包 SHA-256：`8c7ab9fbe357bdaa0435dc8677b92fe2afb68ffc8737be5aa251a6fd32a6dbdc`。
- 保留 `libraries` 下逐飞 GPL 和 NXP/SDK 原版权与许可证文件。
- 应用新增 `car_core`、`car_app`、`car_config`、主机测试和工具；修改主入口、用户中断、编译器版本与工程源文件列表。校赛原有图像/PID 文件不参与构建。
- 不混用旧 V1.1.1 API。摄像头入口为 `mt9v03x_init()`，库内部使用 CSI 双缓冲。
- GitHub 交付不包含原始校赛文档、截图、个人 DAP/J-Link 序列号、工具缓存或编译产物。

- 2026-10-15：新增 car_servo 独立模块和单元测试；两套 Keil 工程均包含该模块；交付到 competition 独立分支，附带软件验证候选包。
