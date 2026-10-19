# G1构建、回归及现场工具

这是当前G1维护入口。历史版本脚本在versions目录保存；旧脚本中的COM6等现场值不能作为当前指令。

## 不访问车辆的检查

```powershell
# 从本次归档根目录运行；MSVC由vswhere查找
.\code\repair\run-tests.cmd
# 本机已有ArmClang6.7，默认D:\keil5\ARM\ARMCLANG\bin；换机先改build_arm.py
python .\code\repair\build_arm.py --mode camera
# 检查发布文件与数据，无需串口
python .\tools\verify_archive.py
```

完整SDK在code/competition/firmware/camera_car；原Keil项目入口demo/mdk/rt1064.uvprojx。当前测试中只用主机I/O替身和合成图像，不能代替地面验证。

## 维护依赖与只读查询

局部依赖未上传；需要时安装到本归档的tools目录，不修改全局Python：

```powershell
python -m pip install --target .\code\repair\tools -r .\code\repair\requirements-maintenance.txt
python .\code\repair\tracking_trial.py --port COM9
```

不带--run只查询，不发a。当前无赛道，可能得到“没有三帧新鲜有效结果”的拒绝，这是不能启动的证据。test_low.ps1默认也只预检L3低速子系统（其查询标识仍为L3，G1整体身份须查TRACK-CFG）。

## 会真实运动的入口

`tracking_trial.py --run`会发a；`test_low.ps1 -Run`会发J/N/B；`test_hold.ps1 -Run`会发U/V；`test_calibration.ps1 -Run`会发L/R。它们不是离线验收，必须按HANDOFF重新确认物理状态。其余旧单轮/舵机/独立行驶脚本仍可能直接发运动命令，且部分保留旧COM号，不作为本阶段默认入口。

## 下载模板

下载须先确认电池关闭、DAP连接正确且核心板灯亮。环境变量SMARTCAR_PROBE_ID由现场私有探针信息提供，SMARTCAR_BASELINE_HEX必须指向已核验的当前车上HEX。`flash_with_backup.py`每次先备份4MiB、核对基线，再写入本地ARM构建产物并逐字节回读；默认保持暂停，不运行目标。

本归档提供原官方Pack19算法code/repair/flash-algorithm/MIMXRT1064_QSPI_4KB_SEC.FLM。仅公开维护脚本的算法路径从临时build/diag改为此归档路径，不影响固件或控制源码；provenance记录了文本变化。私人Flash备份与下载器唯一ID没有上传。不要拿别人的备份恢复到本板，也不要在缺乏当前基线核验时自动运行写入。
