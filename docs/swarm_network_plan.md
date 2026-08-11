# PhantomFilmer 四机联网方案

## 当前结论

当前 `swarm/` 已建立 `SwarmDroneNode`、`SwarmManager`、`SwarmSafetyManager`、Fake Swarm 和真机节点工厂。现阶段真机推进顺序仍然是网络与状态读取优先，不起飞、不转桨；确认四机 IP 和端口后，再进入单机、两机、四机飞行测试。

真机编队非零指令默认启用反馈门禁：外部位置跟踪器必须通过 `SwarmManager.update_formation_feedback()` 提供覆盖所有节点、且未超过 `formation_feedback_timeout_s` 的修正值。反馈缺失或过期时只允许零 RC、降落和急停操作。

## 四台 TT 联网目标

四台 RoboMaster TT 需要进入同一个局域网，并且每台都有唯一身份：

- `drone_1`：leader，只允许它开启视频流。
- `drone_2`、`drone_3`、`drone_4`：follower，不开启视频流，只接收状态读取和 RC 控制。

建议先使用 TT 扩展模块或 SDK 支持的 STA 模式，把四台无人机接入同一个路由器热点。不要在未验证 IP 和端口前写死真机地址。

## IP 与端口确认

真机测试前需要逐台记录：

- 无人机编号：例如贴纸标记 `drone_1` 到 `drone_4`。
- 实际 IP：由路由器后台、扫码配置工具或 SDK 查询结果确认。
- 命令端口：Tello SDK 常用 UDP `8889`。
- 状态端口：常用 UDP `8890`。
- 视频端口：常用 UDP `11111`，第一阶段只允许 leader 打开。

验收标准是：四台均有唯一 `drone_id` 和 IP，状态输出能区分四台，任意一台掉线有明确错误，不影响其他节点返回结果。

## djitellopy 多实例风险

当前 `drone/tello_adapter.py` 支持按 `host` 创建 `TelloDroneAdapter`，四机配置由 `config.yaml` 的 `swarm.drones` 提供。四机控制仍可能遇到：

- 多个实例监听相同本地状态端口，导致状态包来源混乱。
- 视频流端口冲突或带宽不足。
- 命令发送间隔过短导致 UDP 拥塞。
- 某一台失联后普通命令继续发送，造成盲飞风险。

阶段0必须先验证当前 `djitellopy` 版本是否支持对不同 IP 的多实例控制和独立状态读取。未验证前不得进入两机或四机起飞。如果运行时报出“不支持指定 host”，需要升级 `djitellopy` 或切换到原始 UDP 方案。

## 原始 UDP 备选方案

如果 `djitellopy` 多实例端口冲突无法解决，备选方案是新增独立的多机 UDP 控制层：

- 每台无人机维护独立 `drone_id -> ip` 映射。
- 命令统一发往目标 IP 的 SDK 命令端口。
- 状态接收按来源 IP 区分，并记录超时。
- 视频只接 leader，其他三台不打开视频端口。
- 急停命令绕过普通队列，优先向所有已知 IP 发送 `rc 0 0 0 0` 或 emergency/land 类安全命令。

这个备选方案应封装在新的 swarm 层中，不改现有单机 `TelloDroneAdapter`。

## 阶段0安全规则

- 不起飞。
- 不转桨。
- 不安装伞具。
- 不同时开启四路视频流。
- 只做联网、SDK 模式、状态读取和端口验证。
- 任意异常先停止测试，记录到 `docs/swarm_test_record.md`。
