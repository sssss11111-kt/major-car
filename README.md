# major-car

智能车摄像头组 A：图像采集、算法学习与比赛交付。

> 我能打上海major  
> 全场欢呼 danking danking

## 学习记录

当前阶段为第 1 周，目标是跑通 RT1064 摄像头例程，稳定显示灰度图，并保存可复现实验记录。本周先完成采集与稳定性，不提前进入二值化和搜线。

- [ ] 记录开发环境和库版本：[`docs/week1/environment.md`](docs/week1/environment.md)
- [ ] 记录硬件接线和摄像头安装：[`docs/week1/camera-setup.md`](docs/week1/camera-setup.md)
- [ ] 保存正常、较亮、较暗环境各 10 张原始灰度图
- [ ] 记录曝光、角度实验及 30 分钟连续运行结果：[`docs/week1/test-record.md`](docs/week1/test-record.md)

## 比赛交付

比赛使用的稳定版本与最终交付材料集中放在 [`competition/`](competition/)；学习过程记录继续放在 `docs/week1/` 和 `data/week1/`。比赛交付目录的结构和验收清单见 [`competition/README.md`](competition/README.md)。

## 目录约定

- `docs/week1/`：第一周学习、环境、接线和实验记录
- `data/week1/`：第一周采集的原始图像素材
- `competition/firmware/`：经过验证的比赛固件工程和源码
- `competition/config/`：比赛参数与标定记录
- `competition/docs/`：算法、硬件和接口交付文档
- `competition/evidence/`：实测日志、视频和验收证据

请勿提交密钥、个人隐私或无关的大型编译产物。测试结果必须来自实际测试；模板不代表已经完成验收。
