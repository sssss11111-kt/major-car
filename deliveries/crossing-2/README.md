# crossing-2

从 crossing-1 派生。弯道分支在远端用逐列连通白区配对内外边界，取中点作为二维路径目标，用原有曲线采样器检查转角和连接线白区。二维候选通过后供控制预瞄使用，否则沿用 crossing-1 的逐行路径。离线播放器为 crossing-2.html，中间列显示 crossing-1。

在工作区运行 build-replay.ps1 编译主机 DLL，再运行 replay.py 复算播放器。RT1064 工程入口为 deliveries/lap30-ring-20261107/project/demo/mdk/rt1064.uvprojx。
实车测试前需用 Keil 重新生成 RT1064 固件。随附 HTML 可离线打开。

