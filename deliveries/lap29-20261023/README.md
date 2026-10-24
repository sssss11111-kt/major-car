# LAP29：多证据环岛入口识别与拐点约束入环目标路径规划

基于 `codex/20261022-lap21-stripes` 分支基线提交 `875db50`（`deliveries/lap28-20261022`）。**已编译 0 错误 0 警告、全部主机检查与离线回放通过；未烧录、未实车发车。** 车内仍保持 LAP28，仅封装交付目录与压缩包，不修改 `main` 分支。

## 诊断与改进要点

1. **保留黑岛拓扑 + 新增三类几何辅助证据（`img_process.h/.c`）**：
   - 保留完整封闭黑岛（`IP_RING_EV_FULL_ISLAND`）与侧向裁切黑岛（`IP_RING_EV_SIDE_ISLAND`）8 邻域连通拓扑检测；
   - 新增岛底分叉下拐点（`IP_RING_EV_CORNER_LOW`）、岛侧内凸弧顶拐点（`IP_RING_EV_CORNER_APEX`）、外侧引导边界连续单调性（`IP_RING_EV_OUTER_CONT`）及咽喉展宽/支路收缩宽度异常（`IP_RING_EV_WIDTH_ANOMALY`）；
   - 辅助证据绝不单独触发环岛，6 类开放凹湾、27 类连续约 30° 波浪弯、单边丢线急弯、S 弯、上下开口连通域与侧边矩形反例共 496 帧误检率为 0%。

2. **候选确认与入环路径生成分离（`ip_side_island` & `rb_plan_entry_error`）**：
   - 未引导候选确认阶段严格要求黑岛上下端留白且两端收窄；
   - 已进入 `RB_ENTER`/`RB_ORBIT`（`s_ring_guide != 0`）后，即使黑岛随车前进触及近场底边 `y = 105`，`guided_seg` 仍持续分割当前实测白色绕行支路，解决 LAP28 黑岛触底（`maxy > 102`）即丢失分支掩码的问题。

3. **实测白色通道 + 拐点约束生成入环目标中线（消除底部断线与双单边切换跳变）**：
   - 预扫描支路最近双边可见行的实测半宽偏差 `branch_hw_delta`，消除外侧边出画转为单边补线时的跳变（真实帧 `ring-entry-gray.bin` 在 `y = 58 -> 59` 的跳变从 `8 px` 降至 `1 px`）；
   - 在岛底分叉口下方的咽喉区 `y \in (s_ring_maxy, 105]`，沿岛底拐点与弧顶拐点生成平滑入环目标中线并逐行校验当前白像素可通行性（真实帧 `ring-entry-gray.bin` 底部 `y = 94..105` 的缺失行数从 `12 行` 降至 `0 行`）。

4. **疑似态提前减速 + 消除确认帧与滤波器的入环响应延迟（`roundabout_control.h` / `steering_control.h` / `main.c`）**：
   - 新增 `rb_suspect()`：发现 1～2 帧环岛候选即提前将目标速度上限收敛至 `80`（确认前不切换路径）；
   - 当第 3 帧确认进入 `RB_ENTER`（`just_entered == 1`）时，`main.c` 同帧立即激活 `applied_ring_guide = -RB_TURN_SIGN` 生成分支目标路径，并通过 `steering_prime_entry()` 预热三帧中值滤波历史，将确认到舵机首次响应的帧延迟从 **2 帧（40 ms）降至 0 帧（0 ms）**；
   - 完整保留一阶低通平滑、外向 `110 点/s` / 回正 `180 点/s` 变化率限制、`69..116` 软限幅、平面黑条优先及失线/超时/编码器/堵转停车保护，禁止固定盲打角。

## 封装清单

- 完整工程：`project/demo/mdk/rt1064.uvprojx`
- 核心源码：`project/demo/user/inc/{img_process.h,roundabout_control.h,steering_control.h,main.h,pid.h,auto_start.h}`、`project/demo/user/src/{img_process.c,steering_control.c,main.c,pid.c,auto_start.c,isr.c}`
- 编译固件：`project/demo/mdk/Objects/rt1064.hex`
- 验证与构建状态：`project/verification/LAP29-status.json`、`project/verification/host-checks-LAP29.log`、`project/verification/build-reproduced.log`
- 烧录脚本：`tools/flash-lap29.py`（锁定本版 HEX SHA256，**未执行**）
- 哈希清单：`SHA256SUMS.txt`
