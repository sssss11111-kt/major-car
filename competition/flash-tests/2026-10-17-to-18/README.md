# 本对话固件、烧录与实测归档｜2026-10-17～18

[项目现状](../../../README.md) · [HANDOFF](../../../HANDOFF.md) · [调试全过程](../../docs/debug-session-20261017-18.md) · [参考来源](../../docs/references-20261018.md)

## 归档覆盖与证据边界

- 本地成功回读记录对应的**23版精确HEX**全部找回；包含由旧公开验证摘要确认的UART8摄像头整合版。每版SHA与当时记录一致。
- J/K曾被覆盖：按会话代码变更恢复，使用保留库对象重链接，HEX的SHA与当时一致。恢复只在主机进行，**没有操作硬件**。
- 118项原始串口二进制在[raw-serial.zip](raw-serial.zip)，[逐项大小和SHA](raw-serial-index.json)；不是私人4MiB芯片读回。
- 258份初始文本记录（后续以manifest实数为准），脱敏公开保留记录语义，保留[原/公开SHA对照](text-record-provenance.json)。原始build里的pending及历史文档“当前”没有被倒改；本页结论优先。
- 9张[现场照片](photos/index.json)；[用户观察汇总](physical-observations.json)与遥测分开。
- [原有早期归档](../2026-10-17/README.md)保留，含15/16日历史程序和未成功烧录候选，未把候选算成本轮成功。
- [源码覆盖](SOURCE-COVERAGE.md)：当前N完整源码；历史精确HEX不等于每一版都有完整源码快照。

## 版本总表

全部条目写入/回读成功，功能结果各不相同。文件名包含“both15”表示两侧各有配置，**每次仍只输出一侧**，不是双轮同时实测。

| 版本入口 | 用途 | 实物/数据结论 | SHA前12位 |
|---|---|---|---|
| [01-uart8-diag-v1](versions/01-uart8-diag-v1/README.md) | 诊断初版：UART8接入，无自动握手 | 已烧录回读；电脑仍收到全00，通信未通过。 | `dddf6663f280` |
| [02-uart8-diag-b](versions/02-uart8-diag-b/README.md) | 诊断B：加入自动波特率握手 | AB1，ASCII双向通信通过；摄像头跳过，运动禁用。 | `e81d302c2564` |
| [03-uart8-camera](versions/03-uart8-camera/README.md) | UART8摄像头整合版 | 摄像头恢复、帧增长、无线双向查询通过；当时5%点动不转。 | `329836b3bda1` |
| [04-left10](versions/04-left10/README.md) | 左10%、2秒 | 未转动；后续确认输出约0.75V且待机/结束归零。 | `8ce8e3b1a4ff` |
| [05-left15](versions/05-left15/README.md) | 左15%、2秒 | 未转动；输出平均约1.3V，不等于驱动没有PWM。 | `946455dd507f` |
| [06-both15-single-command](versions/06-both15-single-command/README.md) | 两侧各15%配置，逐侧测试 | 每次只驱动一侧；右轮有声音/抖动但未起转。 | `c0f9a1f5e352` |
| [07-right30-high](versions/07-right30-high/README.md) | 右30%、2秒，原方向 | 能起转；用户先误判前进，复测确认后退、编码器为负。 | `233d16d01ee9` |
| [08-right30-low](versions/08-right30-low/README.md) | 右DIR LOW修正 | 右轮前进、计数为正、自动停止，左轮不动。 | `bd2096dbb1ca` |
| [09-left30-c7-high](versions/09-left30-c7-high/README.md) | 左30%、C7 HIGH | 左轮后退、计数为负，右轮不动。 | `4261d0cbf100` |
| [10-left30-c7-low](versions/10-left30-c7-low/README.md) | 左C7 LOW对照 | 仍后退；不能仅凭改宏断言实物方向已变。 | `74ff71ccb9d3` |
| [11-direction-registers](versions/11-direction-registers/README.md) | 方向GPIO配置/读值诊断 | C7配置LOW，左轮仍后退；后续发现GND/LDIR曾接反。 | `83671f38fe6b` |
| [12-left-high-corrected-wiring](versions/12-left-high-corrected-wiring/README.md) | 纠正接线后的C7 HIGH对照 | 方向问题未取得有效解决证据；转入静态引脚对照。 | `e1fa3b22229a` |
| [13-c7-static](versions/13-c7-static/README.md) | C7静态诊断，禁用电机/跳过摄像头 | 寄存器与pad读回存在差异，未证明C7损坏。 | `5053b9fe9148` |
| [14-c11-static](versions/14-c11-static/README.md) | C11静态HIGH/LOW对照 | 万用表HIGH=3.3V、LOW=0V；HIGH时pad仍读0。 | `237a4dfe2293` |
| [15-left-c11-F](versions/15-left-c11-F/README.md) | F：左C11摄像头整合 | 左轮前进正计数，连续两次实测通过。 | `9c44808f9ded` |
| [16-right-speed-G](versions/16-right-speed-G/README.md) | G：右轮独立速度闭环 | 80计数/10ms，4秒，用户确认前进平稳停车、左轮不动。 | `ff398c18d848` |
| [17-left-speed-H](versions/17-left-speed-H/README.md) | H：左轮独立速度闭环 | 80计数/10ms，4秒，用户确认前进平稳停车、右轮不动。 | `f545be9f0ec1` |
| [18-dual-speed-I](versions/18-dual-speed-I/README.md) | I：双轮速度闭环 | 双轮前进平稳，4秒后均停；仅短时架空验证。 | `1879b08cd4ce` |
| [19-servo-register-J](versions/19-servo-register-J/README.md) | J：舵机PWM寄存器只读查询 | 50Hz、约1500us；后续实测1650左转、1350右转，均回中。 | `91ac6265b52a` |
| [20-static-steering-K](versions/20-static-steering-K/README.md) | K：静态视觉转向模式 | 已烧录；赛道识别时有效时无效，未启动w完成实体视觉联动验收。 | `e7cc8db8fb12` |
| [21-ground-L](versions/21-ground-L/README.md) | L：不依赖视觉的1秒落地行驶 | 直行通过；左转已发生，但内轮未达20计数门槛而提前停车。 | `0b61e4cd606c` |
| [22-ground-M](versions/22-ground-M/README.md) | M：落地启动判断修正 | 连续3次>=5计数确认起转；1秒左转、右转实物通过。 | `c9f1022154d9` |
| [23-straight10-N](versions/23-straight10-N/README.md) | N：10秒直行和通信租约 | 多次实测；最终测量前进360cm、左偏5cm，机械中位/重复性尚待排查。 | `01b7c108ca77` |

完整机器清单：[versions.json](versions.json)。当前建议下载目标为N版，历史C7配置不适用于当前C11接线；不要根据目录顺序盲目烧录。

## 数据目录索引

以下是采样会话，不保证每个目录都真的发送运动命令；预检失败、没有看清、重复测试也保留。按result中的commands/failure/active_samples及用户观察共同判断。无线分阶段bin可从ZIP取出按阶段顺序拼接，避免把采样窗口拆断的文本当损坏。

- [both-speed-20261017-232323](records/both-speed-20261017-232323/)：result.json、CSV及已保存观察/统计。
- [c11-20261017-223050](records/c11-20261017-223050/)：result.json、CSV及已保存观察/统计。
- [c11-20261017-223103](records/c11-20261017-223103/)：result.json、CSV及已保存观察/统计。
- [c11-20261017-223305](records/c11-20261017-223305/)：result.json、CSV及已保存观察/统计。
- [drive-left-20261018-002346](records/drive-left-20261018-002346/)：result.json、CSV及已保存观察/统计。
- [drive-left-20261018-002802](records/drive-left-20261018-002802/)：result.json、CSV及已保存观察/统计。
- [drive-right-20261018-002842](records/drive-right-20261018-002842/)：result.json、CSV及已保存观察/统计。
- [drive-straight-20261018-002321](records/drive-straight-20261018-002321/)：result.json、CSV及已保存观察/统计。
- [drive-straight10-20261018-003426](records/drive-straight10-20261018-003426/)：result.json、CSV及已保存观察/统计。
- [drive-straight10-20261018-003951](records/drive-straight10-20261018-003951/)：result.json、CSV及已保存观察/统计。
- [drive-straight10-20261018-004104](records/drive-straight10-20261018-004104/)：result.json、CSV及已保存观察/统计。
- [drive-straight10-20261018-004327](records/drive-straight10-20261018-004327/)：result.json、CSV及已保存观察/统计。
- [left-speed-20261017-230843](records/left-speed-20261017-230843/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-173837](records/motor-20261017-173837/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-174352](records/motor-20261017-174352/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-174452](records/motor-20261017-174452/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-192427](records/motor-20261017-192427/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-192537](records/motor-20261017-192537/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-193256](records/motor-20261017-193256/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-194144](records/motor-20261017-194144/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-195038](records/motor-20261017-195038/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-195428](records/motor-20261017-195428/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-195816](records/motor-20261017-195816/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-195842](records/motor-20261017-195842/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-201226](records/motor-20261017-201226/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-201324](records/motor-20261017-201324/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-201356](records/motor-20261017-201356/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-203328](records/motor-20261017-203328/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-203817](records/motor-20261017-203817/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-203958](records/motor-20261017-203958/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-204426](records/motor-20261017-204426/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-204457](records/motor-20261017-204457/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-204810](records/motor-20261017-204810/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-205343](records/motor-20261017-205343/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-205934](records/motor-20261017-205934/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-210832](records/motor-20261017-210832/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-210922](records/motor-20261017-210922/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-211602](records/motor-20261017-211602/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-211653](records/motor-20261017-211653/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-224014](records/motor-20261017-224014/)：result.json、CSV及已保存观察/统计。
- [motor-20261017-224214](records/motor-20261017-224214/)：result.json、CSV及已保存观察/统计。
- [right-speed-20261017-225928](records/right-speed-20261017-225928/)：result.json、CSV及已保存观察/统计。
- [right-speed-20261017-230007](records/right-speed-20261017-230007/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-232627](records/servo-20261017-232627/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-232722](records/servo-20261017-232722/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-232835](records/servo-20261017-232835/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-233334](records/servo-20261017-233334/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-233440](records/servo-20261017-233440/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-234740](records/servo-20261017-234740/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-234803](records/servo-20261017-234803/)：result.json、CSV及已保存观察/统计。
- [servo-20261017-234919](records/servo-20261017-234919/)：result.json、CSV及已保存观察/统计。
- [servo-20261018-000920](records/servo-20261018-000920/)：result.json、CSV及已保存观察/统计。
- [servo-20261018-000931](records/servo-20261018-000931/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-171213](records/wireless-20261017-171213/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-172004](records/wireless-20261017-172004/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-172346](records/wireless-20261017-172346/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-192445](records/wireless-20261017-192445/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-204435](records/wireless-20261017-204435/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-211611](records/wireless-20261017-211611/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-211838](records/wireless-20261017-211838/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-221811](records/wireless-20261017-221811/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-221840](records/wireless-20261017-221840/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-222202](records/wireless-20261017-222202/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-234252](records/wireless-20261017-234252/)：result.json、CSV及已保存观察/统计。
- [wireless-20261017-235906](records/wireless-20261017-235906/)：result.json、CSV及已保存观察/统计。
- [wireless-20261018-001456](records/wireless-20261018-001456/)：result.json、CSV及已保存观察/统计。

## 重点证据

- UART8：records/VALIDATION.md，wireless-20261017-172004、172346。
- 左C11：motor-20261017-224014、224214；C11静态电压观察另见c11相关记录。
- 架空闭环：right-speed-20261017-230007、left-speed-20261017-230843、both-speed-20261017-232323。
- 舵机：servo-20261017-234803、234919；接口/纸赛道照片08/09；PWM查询wireless-20261017-234252。
- 静态识别：wireless-20261017-235906、wireless-20261018-001456；K版静态视觉联动未实物验收。
- L启动保护误杀：drive-left-20261018-002346；M左/右成功：002802、002842。
- N直行：drive-straight10-20261018-003426、003951、004104、004327；最后一份对应用户360cm/左偏5cm。

## 如何校验

SHA256SUMS.txt是本目录全部交付文件的相对路径指纹，排除自身和manifest.json；manifest.json列大小与SHA。使用Python标准库即可校验，无需连接车。TEXT记录经过路径/标识脱敏，不能用它们的公开SHA去假装原文件完全未改；HEX和原串口字节没有改写。

没有提交：私人全Flash读回、工具环境、安装报告缓存、个人探针ID、无关资料。保留版权/许可证；公开维护脚本的探针ID改为环境配置，详见根repair/README.md。
