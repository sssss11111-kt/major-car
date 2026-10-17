# N：10秒直行和通信租约

- 固件：[firmware.hex](firmware.hex)
- SHA-256：`01b7c108ca777934c7fe1a3893ac8beff9f47f7b41cbf38311fb40b3e2dd7af5`
- 烧录：写入回读校验通过；功能结论：**多次实测；最终测量前进360cm、左偏5cm，机械中位/重复性尚待排查。**
- 源码覆盖：当前完整源码见 competition/firmware/camera_car；源文件清单见 current-source-manifest.json。
- [回读/写入记录](flash.json)；[版本元数据](version.json)。构建记录中的pending是当时状态，最终结论以本页和会话总结为准。

历史版本用于追溯，不应当作当前接线下的默认下载文件。诊断版与摄像头版、历史C7与当前C11不可混用。
