# 远端UART8自动波特率修复/真实摄像头版

状态：**远端交付验证记录记载已烧录、65152字节读回一致及电池冷启动；本轮未重新实测。**

远端验证记载双向查询、无零字节、帧号增长、用户确认画面。该交付的motor_commands_sent=false，不能把本聊天旧镜像版的点动日志套用到这个新版。

源码：[独立UART8修复完整源码](https://github.com/sssss11111-kt/major-car/tree/af2877ab0bb9c070cdbc28ee34dcaf22161b2b57/competition/firmware/camera_car)；固定提交 `af2877ab0bb9c070cdbc28ee34dcaf22161b2b57`。

目录内 firmware 为本次归档原件；evidence 为保留的历史日志，主机路径已脱敏。构建成功、Flash校验成功和实机验收分开记录。不得直接落地运行。
