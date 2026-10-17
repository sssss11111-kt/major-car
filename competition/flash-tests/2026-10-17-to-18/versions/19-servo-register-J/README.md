# J：舵机PWM寄存器只读查询

- 固件：[firmware.hex](firmware.hex)
- SHA-256：`91ac6265b52a6b9405aab798c6395147be32fb7e184abe0df217aede0ec564b0`
- 烧录：写入回读校验通过；功能结论：**50Hz、约1500us；后续实测1650左转、1350右转，均回中。**
- 源码覆盖：应用/核心代码根据对话变更重建，与保留库对象重链接后HEX逐字节SHA完全匹配；见 sources/reconstructed-J-K.zip。重建过程未烧录。
- [回读/写入记录](flash.json)；[版本元数据](version.json)。构建记录中的pending是当时状态，最终结论以本页和会话总结为准。

历史版本用于追溯，不应当作当前接线下的默认下载文件。诊断版与摄像头版、历史C7与当前C11不可混用。
