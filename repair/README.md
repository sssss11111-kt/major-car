# 本轮构建、维护与测试脚本

这些是2026-10-17～18的本地调试脚本；完整结果见[固件/数据归档](../competition/flash-tests/2026-10-17-to-18/README.md)。当前N版支持10秒直行，禁止把旧脚本中的历史限制当现有行为。

- `run-tests.cmd` / `run_tests.py`：现有MSVC主机回归；VS路径按实际安装调整，输出使用唯一目录避免文件锁。不会驱动车辆。
- `build_arm.py --mode camera|diag|mock`：现有ArmClang6.7按原Keil源表构建；默认编译器路径需配置。不会烧录。
- `test_radio.ps1 -PortName COM6 -BaudRate 115200 -ServoRegisters`：p/z只读查询。串口和设备身份按本机检查。
- `test_drive.ps1 -Action straight|left|right|straight10`：**真实运动**；只在用户确认上电、场地和人员状态后运行；前3种1秒、straight10为10秒，故障/到时停车回中。
- `test_drive.ps1 -PreflightOnly`：只查询版本与待机状态，finally可能发s确认停止，不发运动命令。
- `test_servo.ps1`、`test_one_motor.ps1`、`test_speed.ps1`及包装器：分别会驱动舵机/单轮/双轮，必须按HANDOFF确认架空或场地，不要为“检查上传”执行。
- `flash_with_backup.py`：本机维护下载流程模板，需要局部pyOCD、IntelHex、匹配NXP FLM和原车私人备份。公开版将探针ID改成必填环境变量`SMARTCAR_PROBE_ID`；没有下载器连接就不执行。

公开仓库不含`tools/`环境、`build/`对象、私人4MiB Flash读回；`--reuse-backup`只适用于有本板完整校验备份的原维护环境。换机不要照抄绝对路径；先配置依赖和备份，不直接运行写入命令。

本次发布只运行主机回归和归档完整性检查，没有发送运动/烧录命令。最新物理动作、测量、未完成项见[HANDOFF](../HANDOFF.md)。
