# C11LAP21 连续弯道转向与提速候选

已编译、相关主机检查通过，尚未烧录、尚未实车验证。当前车载仍为LAP20，用户确认可走完全程，但过弯转向抽搐、幅度偏大。

| 项目 | LAP20 | LAP21候选 |
| --- | ---: | ---: |
| 直道目标 counts/10ms | 45 | 67.5 |
| 满打角急弯目标 counts/10ms | 24 | 36 |
| 单边识别目标上限 counts/10ms | 24 | 36 |
| 增益及打角映射 | 门槛切换 | 随弯曲程度连续变化 |
| 指令稳定 | 图像近误差限变 | 另加中值/时间滤波、转向变化率限制、速度渐进恢复 |
| 舵机中位/左限/右限 | 92.5/124/61 | 92.5/124/61 |
| PWM可用范围 | 0～100% | 0～100% |

详细依据、算法、验证边界见CHANGES.md。目标提高50%不等于实速已提高50%。缓急分级是像素域估计，没有相机/轮角标定，不能按指定厘米半径闭环转弯。

- 工程：project/demo/mdk/rt1064.uvprojx
- 固件：project/demo/mdk/Objects/rt1064.hex
- HEX SHA256：af24ed5eea725ab58029444579274017af1cd6b0e6171d64cc7e6298658614e9
- 有效程序：69,056字节；Keil ARM Compiler 6.7零错误零警告。
- 状态：project/verification/LAP21-status.json
- 构建：tools/build-keil.ps1 -Keil D:\Keil5\UV4\UV4.exe
- 主机检查：tools/run-host-checks-msvc.cmd；其他电脑用VS_VCVARS64指定vcvars64.bat。也支持ZIG工具链。
- flash-lap21.py仅为下次明确烧录时准备，需--prepared且校验本版精确HEX；本轮未执行。receive-lap21.py只被动接收。

保留可靠赛道→回中1秒→自动起步，以及停车保护、故障锁定至复位。LAP20完整源码、实际烧录HEX和现场数据在同分支保留，可回退。本交付保留源码、工程、HEX、验证材料；本机临时AXF/对象文件不列入交付。
