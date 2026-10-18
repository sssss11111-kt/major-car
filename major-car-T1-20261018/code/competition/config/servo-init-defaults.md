# 舵机候选默认配置

基准：2026-10-15，BDS300，C车模；未进行机械标定。

唯一生效配置为 ../firmware/camera_car/demo/user/inc/car_config.h。

- PWM4_MODULE2_CHA_C30，50 Hz。
- 中位1500 us；起始范围1350～1650 us；占空比750/10000。
- 真实方向、机械中位和限位待确认。
- 启动只回中，不自动左右扫舵；c/l/j 为待机手动测试命令。
