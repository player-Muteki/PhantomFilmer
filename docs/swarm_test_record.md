# PhantomFilmer Swarm 测试记录

## 2026-07-13 本地开发记录

- 已拉取 `https://github.com/player-Muteki/PhantomFilmer` 到本地任务目录。
- 已创建分支 `feature/chen-swarm-real`。
- 系统 `python3` 基线测试失败原因：缺少 `numpy`，不是 swarm 逻辑错误。
- 已补充真机 IP 节点工厂、Fake 运行中失联模拟、短时 RC 后自动清零、`swarm-hover-test` 和 `swarm-rc-test` 入口。
- 未执行真实四机联网、状态读取、起飞、悬停或移动测试。

## 真机记录模板

| 日期 | 阶段 | 节点 | IP | 电量 | 高度 | 结果 | 问题 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 待填写 | 阶段0 | drone_1 | 待确认 | 待确认 | 待确认 | 待确认 | 待确认 |
| 待填写 | 阶段0 | drone_2 | 待确认 | 待确认 | 待确认 | 待确认 | 待确认 |
| 待填写 | 阶段0 | drone_3 | 待确认 | 待确认 | 待确认 | 待确认 | 待确认 |
| 待填写 | 阶段0 | drone_4 | 待确认 | 待确认 | 待确认 | 待确认 | 待确认 |

## 已知未完成项

- 真实四台 TT 的 STA/AP 联网方式和 IP 尚未现场确认。
- `djitellopy` 多实例端口冲突尚未现场验证。
- 四机起降和低速同步移动需要按测试计划逐阶段录制演示视频。
- 第一版 `formation_correction = 0`，未接入外部定位或 ArUco 纠偏。
