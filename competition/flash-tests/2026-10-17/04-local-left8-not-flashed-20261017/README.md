# 本机左8%起转诊断候选版

状态：**仅编译与主机测试通过；两次烧录失败，未确认写入核心板。**

左路8%、右路5%，均1秒自动停止。新增自动化检查先失败后通过；完整主机回归48986检查0失败，两种工程0错误0警告。两次下载均SWD/JTAG Communication Failure及RDDI-DAP Error，未见Verify OK。实机8%点动从未执行。本候选不包含远端新UART8自动波特率修复。

源码：[完整左8%源码快照](../sources/03-local-left8-candidate-source.zip)

目录内 firmware 为本次归档原件；evidence 为保留的历史日志，主机路径已脱敏。构建成功、Flash校验成功和实机验收分开记录。不得直接落地运行。
