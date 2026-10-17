# 模拟图像/舵机初始化版

状态：**历史日志显示写入校验成功；预编译HEX来自旧交付包，未保留当时Flash读回指纹。**

CAR_SIMULATED=1，合成图像，禁止电机。用户后来验证电池单独供电时舵机回到稳定位置；机械中位和限位未完整验收。日志有Application running之后的RDDI错误，不能把该文字当作启动成功。

源码：[完整历史源码快照](../sources/01-servo-camera-before-uart8-source.zip)

目录内 firmware 为本次归档原件；evidence 为保留的历史日志，主机路径已脱敏。构建成功、Flash校验成功和实机验收分开记录。不得直接落地运行。
