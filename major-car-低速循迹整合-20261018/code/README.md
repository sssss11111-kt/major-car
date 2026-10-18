# RT1064 低速视觉循迹候选 T

完整说明见[交付入口](../README.md)、[逐步修改记录](../修改记录.md)及[实车验收清单](../实车验收清单.md)。

本代码基于N版 `53d6dba`，版本标识20261018T；本轮没有烧录、没有实车验收。旧N版和硬件历史保留在[HANDOFF.md](HANDOFF.md)，不能将其中已测结论视为T版通过。

- 工程：`competition/firmware/camera_car/demo/mdk/rt1064.uvprojx`。
- 参数：`competition/firmware/camera_car/demo/user/inc/car_config.h`。
- 主机完整回归：`repair/run-tests.cmd`。
- ARM真实摄像头构建：`python repair/build_arm.py --mode camera`。
- 循迹配套脚本：`repair/tracking_trial.py`，默认查询，显式--run才发送运动命令。

默认a行为：3帧有效图像、200ms转向准备、最多3秒行驶、500ms接收续期超时；目标20计数/10ms、最高30%有界起转。w为静态视觉转向。s/S停车。

保留原N版接线、逐飞V3.5.5、NXP库和许可证。完整旧固件23版/照片/串口归档没有复制进本软件副本，历史引用可在[基线GitHub分支](https://github.com/sssss11111-kt/major-car/tree/codex/flash-tests-20261017)查看。
