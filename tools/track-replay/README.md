# 赛道图像回放工具

**下载、解压，用浏览器打开即可使用，无需安装 Python 或逐飞助手。**

- [下载便携版 ZIP](https://github.com/sssss11111-kt/major-car/raw/refs/heads/main/tools/track-replay/track-replay-portable.zip)
- [单独下载 HTML](https://github.com/sssss11111-kt/major-car/raw/refs/heads/main/tools/track-replay/track-replay.html)（保存到电脑后打开）
- [下载必要源码](https://github.com/sssss11111-kt/major-car/raw/refs/heads/main/tools/track-replay/track-replay-source.zip)

## 三步开始

1. 下载 ZIP 并解压，双击 `track-replay.html`，选择新版 Chrome 或 Edge 打开。
2. 点“选择采集文件夹”，选择一组包含 `frames.jsonl` 和 `frames/*.pgm` 的记录；也可导入 `.trackreplay` 数据包或单独PGM原图。
3. 查看同步的灰度原图、巡线二值图、边界中线图；支持播放、逐帧、跳转、问题备注与导出。

工具不附带本车录制数据。请加载自己的188×120、8位P5 PGM原图；没有元数据也能分析，但不能核对原始帧号和时间。普通视频、JPEG和94×60缩略图不属于当前输入格式。

## 分享记录

“保存问题对照HTML”把已标记帧（没有标记时为当前帧）的原图、算法、保存结果和备注写入独立HTML。
“保存便携数据”按选定范围导出无损 `.trackreplay`，可与空白工具HTML一起转移。
仅复制空白工具不会自动带走图像；网页备注需导出后保留。大记录建议每包约300张。

## 算法与验证边界

- HTML内置同一份SF2视觉C代码编译的WebAssembly，数据在浏览器本地处理，页面不联网。
- 120张真实记录在连续、单帧两种模式下的13项指标、360个线坐标、120行阈值与本机C程序一致。验证摘要见 `validation.json`。
- 二值图用实际分行阈值生成；叠加线包含补线和历史估计。
- 环岛引导固定为0，没有重演完整入环、绕行、出环和车辆控制器。离线结果不能直接视为车端当时决策。
- 带结果数据先保留保存时的结果；只有点击“重新分析”才重算。挑出的子集无法重建缺失历史。
- 已在Windows Chrome本地文件与离线模式测试；其他系统和手机未做实际设备验收。手机/聊天软件的HTML预览可能无法运行脚本。
- 隔离自动化浏览器的下载到磁盘环节未通过（普通文本下载也失败）。已验证实际生成的HTML内容与独立打开，原Windows桌面工具直接导出也已验证；不把自动化下载称为已验收。

## 版本

发布：2026-11-04。文件SHA256见 `SHA256.json`；C源码与WASM身份见 `algorithm.json`。
必要源码包用于重建浏览器版：安装uv后，在源码根目录运行 `uv run --with ziglang==0.13.0 python -X utf8 portable/build_portable.py`。日常打开HTML不需要编译器。
本工具不自动连接串口、DAP或控制车辆。
