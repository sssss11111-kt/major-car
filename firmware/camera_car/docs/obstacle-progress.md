# 红砖开发记录

更新日期为 2026-11-07，Asia/Shanghai。

## 当前阶段

工作目录为 `major-car-obstacle`。根据用户后续要求，已拉取 GitHub 并以默认分支 `main@4d0e421` 创建 `codex/20261107-red-brick`。任务书的历史分支和 HTML 的 SF2 名称仅作为来源线索。初始 LAP28 接入实验已保存到本地 stash，历史分支与交付目录保留。

用户指定的回放页第 100–200 张已无损提取。图像分析、标注、四种路线比较、参数来源和真实回归见 [数据说明](../../../data/obstacle/README.md)。任务书原文来自 Downloads，HTML 原件来自用户提供的微信资料路径。图像使用本地数据完成分析。

阶段验收采用三个边界。数据和检测先做离线真实回归，C 模块再验证内存、队列和时序，最后才允许讨论控制接管。17 个正面框由开发时目视标注，仍需独立复核。当前真实检测在 main 巡线输出上命中 8/17、漏检 9/17，其中 2 次错定位。独立场景负样本、距离/车身标定和最新完整环岛整车代码缺失，因此控制接管和实车绕行验收待完成。

## 模块与接口

- `obstacle_detect.h/.c` 的 `obstacle_detect_frame()` 提供只读灰度检测、二值诊断、固定内存 8 邻域连通域、几何筛选和连续目标确认。
- `obstacle_vision.h` 把已有 `vision_result_t` 的当前双边行转换为检测输入，沿用当前结构定义。
- `car_app.c` 的停车观测在原 `control_frame()` 之后执行。编译选项 `OBSTACLE_SHADOW_ENABLED` 默认 0。启用后仍受停车状态、图像有效性和图传限频约束。
- `obstacle_bypass.h/.c` 的 `obstacle_bypass_update()` 是独立的距离规划模块，输出毫米偏移或停车请求。调用方需提供实际标定及整条候选走廊的保守物理空间。

原 `car_core.c`、`car_config.h`、`main.c` 和中断代码与 GitHub 起点一致。绕行规划没有接入控制输出。

构建证据见 `data/obstacle/results/`。本轮同时修正既有主机测试在中文工作区路径下的文件打开问题，C 场景测试改传相对输出路径，报告进程明确使用 UTF-8。测试脚本可通过 `ZIG` 环境变量指定编译器。

## 已提交证据

阶段代码依次提交为 `d2d16c2` 数据提取、`96a21b6` 像素分析、`feef9f7` C 检测、`922b012` 停车观测、`7756ac1` 标定式距离规划及测试。最终证据另作独立提交。

`results/image-analysis.zip` 包含全部 101 帧的原图/二值图/ROI/候选 mask/框对照及完整统计 JSON，`results/c-replay.json` 保存三条真实 C 回归路径的逐帧输出。`results/tests.log`、`detector.log`、`shadow-test.log` 保存主机证据。`arm-default.log` 和 `arm-shadow.log` 保存 Arm 工具链构建摘要。两个候选 HEX、完整 map 和源码/产物 SHA-256 见 `results/verification.json`。

Git 作者配置缺失，本轮提交以命令级 `Codex <codex@local>` 身份署名。切换到 main 后，原目录中此前已存在的历史构建中间文件显示为未跟踪的 `deliveries/`。这些文件未纳入本轮提交。

## 绕行规划与合成验证

标定表的每个节点记录 `y_max` 对应的到障碍物近表面的距离和横向毫米/像素。表按行递增、距离递减，采用有界插值。缺少有效标定且已有确认目标，或已在规划中而标定失效时，锁存停车请求。没有确认目标且尚未进入规划时保持空闲。没有从本次图像推算厘米。

按车身半宽、转弯扫掠、边界/延迟余量和障碍物不确定性膨胀障碍物及赛道边界，再检查两侧安全偏移。候选中选绝对偏移较小者，等代价选择左侧。三次平滑曲线约束最大斜率及二阶导数所需的最小转弯半径。输入的 `corridor_left_mm/right_mm` 必须是整段接近、通过、回正走廊的最小值，单行像素间隙不足以提供这个输入。

状态依次为 `IDLE → APPROACH → PASS → RETURN → COOLDOWN → IDLE`。累计前进计数控制接近距离、砖体深度加车尾余量、回正和冷却。计数回绕可处理，倒退和过大单步增量拒绝。确认不足时保持空闲，接近时丢失目标或量测不一致、无安全路径、多目标、里程异常、通道收窄、特殊赛道中断或紧急停车均产生停车请求。停车锁存，由上层显式复位。

测试参数仅为合成几何。测试覆盖左/右/双侧空间、两侧不足、边缘砖、中央砖、距离不足、车尾清空、连续第二目标、丢失目标、里程回绕/倒退/跳变、未标定、失效标定、环岛/十字屏蔽和紧急停车。

```powershell
$zig = "<zig.exe 路径>"
& $zig cc -std=c99 -Wall -Wextra -Werror -O2 -UNDEBUG -I firmware/camera_car/demo/user/inc firmware/camera_car/tests/test_obstacle_bypass.c firmware/camera_car/demo/user/src/obstacle_bypass.c -o "$env:TEMP/brick-bypass-test.exe"
& "$env:TEMP/brick-bypass-test.exe"
```

## 后续实车验证

先补采曝光变化、左右贴边砖、正常标记、弯道、环岛和反光的独立记录，改进并复测检测。确认完整整车工程后迁移观测接口，核验所有特殊赛道屏蔽信号和原有停车优先级。再测量固定相机下的多距离标定、编码器 counts/m、车宽、车尾长度、转向扫掠范围和稳定控制时延。

实车按架空输出核对、静态检测、低速直道左中右单砖、绕行回正、弯道与连续砖逐项执行。每次只调一个参数并保留原始图像、检测日志和停车原因。本轮已完成主机和编译验证，板上测试待执行。
