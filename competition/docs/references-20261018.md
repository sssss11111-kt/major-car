# 资料、案例和开源来源

本表区分“实际工程依赖”“已查阅依据”和“前交接提及的参考”。没有把未完成阅读或未移植的仓库写成技术成果。

| 来源 | 链接/位置 | 本项目实际用途与边界 |
|---|---|---|
| 本项目GitHub | [major-car](https://github.com/sssss11111-kt/major-car) | 本轮源码与归档，指定分支codex/flash-tests-20261017；main不等于当前实车 |
| 逐飞官方RT1064库 | [Gitee：seekfree/RT1064_Library](https://gitee.com/seekfree/RT1064_Library) | 正式上游。当前固定学校《镜头组例程.zip》内V3.5.5，不宣称与上游最新一致 |
| 逐飞RT1064 Lite资料 | [官方核心板入口](https://www.seekfree.com.cn/产品资料/核心板/恩智浦系列/rt1064-lite核心板/) | 前交接提供的硬件入口，实际手册/版本需匹配实物 |
| 逐飞无线转串口 | [官方无线资料入口](https://www.seekfree.com.cn/产品资料/传感器/无线通讯/无线转串口/) | UART8/RTS、V2.x自动波特率握手依据，当前完整A5 FF FF已实机验证 |
| 逐飞库内驱动 | `libraries/zf_device/zf_device_wireless_uart.*`、`libraries/zf_driver/zf_driver_pwm.*`、`zf_driver_encoder.*` | 实际调用/核查；使用本仓库同版本，避免旧API混用 |
| NXP MCUXpresso SDK | 工程`libraries/sdk`内MIMXRT1064.h、fsl_pwm.c、fsl_iomuxc.h等 | 实际寄存器、PWM时钟/分频/引脚定义依据；保留NXP许可证 |
| NXP MIMXRT1064 DFP | 本机Pack19的MIMXRT1064_QSPI_4KB_SEC.FLM | 实际下载算法；第三方工具/Pack不随此次归档复制，换机需自行配置 |
| pyOCD | [pyocd/pyOCD](https://github.com/pyocd/pyOCD) | 本轮下载器连接、编程、回读；使用本地隔离安装，不提交安装缓存 |
| IntelHex | [python-intelhex/intelhex](https://github.com/python-intelhex/intelhex) | HEX读取与Flash比对；不是用新构建替代旧实测文件 |
| 学校硬件资料 | 《RT1064主板V3.0.1.2原理图》《镜头组例程》、摄像头组宣讲 | 前交接/本地资料，主板照片为V3.1；图纸版本不同必须对照，不擅自发布学校原压缩包 |
| 参考案例A | [hmayzszjcts/smartcar-Open-source](https://github.com/hmayzszjcts/smartcar-Open-source) | 前交接用户提供，本次GitHub接口核验仓库可读取、默认分支master；未保留完整硬件匹配/移植评审，不宣称直接可用 |
| 参考案例B | [ypwhs/smartCar](https://github.com/ypwhs/smartCar) | 前交接用户提供，本次核验仓库可读取、默认分支master；不是当前代码依赖，无已完成移植结论 |
| 无线说明书转载 | [Scribd历史转载](https://www.scribd.com/document/1066663455/逐飞科技-无线转usb-串口说明书-v2-3) | 仅前交接记录的转载入口；优先官方与对应硬件修订，不用灯色/插针旧图直接判当前模块 |

库输入压缩包SHA-256为`8c7ab9fbe357bdaa0435dc8677b92fe2afb68ffc8737be5aa251a6fd32a6dbdc`，见工程PROVENANCE。GitHub参考仓库元数据可读不代表其算法可靠或适配本车。Gitee/厂家入口来自原交接和库内来源，本次未宣称逐页重新审计其全部在线内容。

案例借鉴应先对照芯片/SDK、摄像头、编码器、引脚、单位和控制周期；先移植纯算法并做离线测试，再架空，再短时落地。不要直接复制别车的PID、舵机中位/限位或前进DIR电平。
