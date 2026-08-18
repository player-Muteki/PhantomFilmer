# PhantomFilmer WebUI 真机适配修改计划书

> 稳定基线：`e45ec66e047d82579f166fb03a217ac998a92887`
> 基线提交：`feat: add ReID scene labels and obstacle overlays`
> 仓库：`player-Muteki/PhantomFilmer`
> 编写日期：2026-08-18
> 文档状态：待审核，未开始实施

## 1. 目标与结论

本计划以 `e45ec66` 作为唯一稳定基线，不引入后续 `8f70f46` 的红外主导/视觉辅助融合避障改动。

当前 WebUI 的 React 页面和 REST/WebSocket 适配层使用了 `e45ec66` 中已存在的主要接口，因此基础结构可以迁移；但当前实现不能直接视为真机稳定版，必须先解决以下问题：

1. Web 启动链路仍可选择 `FakeDroneAdapter`；
2. 当前连接状态只是一次性布尔标记，没有持续健康检查；
3. `command -> ok` 成功但 `battery?` 失败时，底层适配器仍可保持已连接；
4. 遥测 WebSocket 每 200ms 直接读取多个 SDK 字段，容易占用 Tello 命令通道；
5. 每个 MJPEG 请求都会新建 `CameraStream` 并调用 `stream_on()`，多页面/多队员访问时可重启视频解码器；
6. Web 预览流和 `FollowSession` 都会抢占摄像头生命周期；
7. 前端的“周界感知雷达”、虚构障碍点和部分就绪状态没有真实传感器依据。

结论：**前端可以适配 `e45ec66`，但需要一次真机连接、视频所有权和遥测架构改造，不是只删除几个 UI 元素。**

## 2. 稳定基线核对

### 2.1 可直接复用的接口

| WebUI 需求 | `e45ec66` 真实接口 | 兼容性 |
|---|---|---|
| 系统装配 | `app.builder.build_system(use_fake, obstacle_enabled)` | 兼容 |
| 真机适配器 | `TelloDroneAdapter` | 兼容 |
| 连接 | `ConsoleTools.connect()` | 兼容，但需 Web 层二次验证 |
| 状态 | `ConsoleTools.get_status()` | 兼容 |
| 起飞前检查 | `ConsoleTools.can_start_task()` | 兼容 |
| 启动跟随 | `ConsoleTools.start_follow_task()` | 兼容 |
| 停止/急停 | `stop_task()` / `emergency_stop()` | 兼容 |
| 任务状态 | `is_task_active()` | 兼容 |
| 视频帧 | `CameraStream.start/read_frame/stop` | 接口兼容，生命周期不兼容 |
| 电量/高度/航向 | `get_battery/get_height/get_yaw` | 兼容 |
| 前向 ToF | `get_front_distance_cm()` | 可选，必须容错 |

### 2.2 不能直接复用的当前行为

| 当前 WebUI 行为 | 问题 | 处置 |
|---|---|---|
| `create_app(use_fake=True/False)` | 生产 Web 路径可进入 Fake | 删除 `use_fake` 参数 |
| `python main.py --mode web --fake` | 可在页面中产生模拟画面 | 对 Web 模式显式拒绝 |
| WebSocket 5Hz 直读 SDK | 命令通道负载过高 | 改为后台低频采样+高频缓存推送 |
| 每个 `/video/stream` 新建摄像头 | 多客户端冲突 | 改为单例 `VideoHub` |
| 预览和任务各自 `stream_on()` | 解码器被重启 | 增加视频所有权交接 |
| 环形周界感知 | 无对应传感器 | 删除 |

### 2.3 基线范围约束

- 以 `e45ec66` 创建新工作分支，不在当前 `8f70f46` 工作树上继续叠加。
- 不带入 `8f70f46` 新增的 `control/obstacle_fusion.py` 和 `vision/visual_obstacle.py`。
- 保留 `e45ec66` 的 ReID 角色标签与可选视觉物体提示，但不将视觉物体标签当作飞控避障输入。
- `vision.visual_object_detection_enabled` 保持默认 `false`。
- WebUI 不修改 RC 输出限制、降落保护和 `MotionArbiter` 优先级。

## 3. 真机专用启动路径

### 3.1 生产代码中移除 Fake 路径

修改目标：

1. `run_web()` 删除 `use_fake` 参数；
2. `web.server.create_app()` 删除 `use_fake` 参数；
3. Web 装配固定执行 `build_system(use_fake=False, ...)`；
4. `main.py` 检测到 `--mode web --fake` 时立即返回错误，不能静默忽略；
5. 保留 `FakeDroneAdapter` 给原有离线单元测试和其他 CLI 模式，但生产 Web 入口不可达；
6. WebAPI 测试通过依赖注入的测试双体完成，不把 Fake 适配器暴露给 Web 启动参数。

### 3.2 唯一正式启动命令

```powershell
python main.py --mode web
```

启动 Web 服务不等于连接无人机。页面初始状态必须为：

```text
服务已就绪 / 真机未连接 / 视频未启动 / 任务不可启动
```

## 4. 连接状态机与健康检查

### 4.1 状态定义

Web 层新增与 `ConsoleTools.connected` 区分的真机连接状态：

```text
DISCONNECTED  未连接
CONNECTING    正在连接
VERIFIED      已验证真机
DEGRADED      连接失效/遥测连续失败
CLOSING       正在释放资源
```

### 4.2 首次连接验证

页面点击“连接真机”后，必须顺序完成：

1. 后端状态进入 `CONNECTING`；
2. `ConsoleTools.connect()` 调用 `TelloDroneAdapter.connect()`；
3. Tello SDK 的 `command` 必须返回精确 `ok`；
4. Web 层紧接着调用 `ConsoleTools.get_status()`；
5. `battery?` 必须成功返回 `0–100`；
6. 高度读取成功；
7. 记录 `verified_at` 和最新电量；
8. 全部通过后才进入 `VERIFIED`。

任一步失败：

- 不返回已连接；
- 调用 `ConsoleTools.close()` 释放部分初始化的 SDK 资源；
- 状态回到 `DISCONNECTED`；
- 向前端返回可理解的错误原因。

### 4.3 持续健康检查

- 不在 5Hz WebSocket 循环中直接查询 SDK。
- 新增后台 `TelemetryService`，单线程串行访问 Tello 命令通道。
- 电量/基础健康检查默认每 2 秒一次。
- 高度与航向默认每 1 秒一次，可根据真机压力测试降频。
- WebSocket 可以继续每 200ms 推送，但只推送最新缓存，不触发 SDK 查询。
- 连续 3 次健康检查失败后进入 `DEGRADED`。
- `DEGRADED` 状态禁用新任务启动，但不擅自中断已在运行的任务；已运行任务继续依靠原有 `FollowSession` 和安全层处理。
- 连接恢复必须重新完成首次验证，不只把布尔值改回 `true`。

## 5. 任务启动硬门禁

### 5.1 必须条件

`POST /api/task/start` 不直接信任前端按钮状态，后端必须重新检查：

1. 连接状态为 `VERIFIED`；
2. 最后一次健康检查未过期（建议不超过 5 秒）；
3. 立即重新执行 `ConsoleTools.can_start_task()`；
4. 本次电量读取成功；
5. 电量达到 `SafetyManager` 起飞阈值；
6. 当前没有其他飞行任务；
7. 用户完成二次飞行安全确认。

任一条件失败，后端返回 `409` 或 `503`，不创建 `FollowSession`。

### 5.2 前向 ToF 不作为 WebUI 通用启动门禁

- WebUI 不要求前向 ToF 必须存在。
- 前向 ToF 缺失时显示“设备未提供”，不影响连接验证和 WebUI 自身的启动按钮。
- 如果项目配置显式启用 `obstacle.enabled=true`，则保留 `e45ec66` 核心安全层对 ToF 的 fail-closed 行为；WebUI 不能绕过该安全规则。
- 无前向 ToF 硬件的部署默认使用 `obstacle.enabled=false`。
- Web 模式不弹出误导性的“是否开启避障”提示；是否开启由经过真机审核的配置决定。

## 6. 视频流单例化与所有权交接

### 6.1 当前风险

`e45ec66` 中：

- `TelloDroneAdapter.stream_on()` 会先释放旧视频接收器，再调用 `streamon()`；
- `FollowSession` 会自己创建 `CameraStream`，并在任务开始/结束时控制视频流；
- 当前 Web 的每个 MJPEG 客户端也会创建 `CameraStream`。

这会导致队员同时打开页面、Web 预览与飞行任务切换时反复 `stream_on/stream_off`。

### 6.2 `VideoHub` 设计

新增单例 `VideoHub`：

- 一个后台视频生产者；
- 保存最新一帧 JPEG/调试帧；
- 任意数量浏览器只消费缓存帧；
- 浏览器连接/断开不直接调用 `stream_on/stream_off`；
- 真机未进入 `VERIFIED` 时，`/video/stream` 返回 `503`，不启动摄像头。

### 6.3 所有权状态机

```text
NO_OWNER
   ↓ 真机连接已验证
WEB_PREVIEW_OWNER
   ↓ 用户确认启动任务
HANDOFF_TO_TASK
   ↓ Web 释放预览相机
FOLLOW_SESSION_OWNER
   ↓ 任务结束
HANDOFF_TO_WEB
   ↓ 仍保持真机连接
WEB_PREVIEW_OWNER
```

交接要求：

1. 启动任务前，Web 预览必须完整停止并释放解码器；
2. `FollowSession` 成为唯一视频读取者；
3. 给 `FollowSession` 增加可选、默认为空的 `frame_sink`/帧观察器；
4. `frame_sink` 只向 `VideoHub` 发布已读取帧，不发送飞控指令；
5. 任务结束后，如果连接仍是 `VERIFIED`，Web 预览才可重新接管；
6. 所有权交接超时时不启动任务，不并发抢占视频。

## 7. 前端界面修改

### 7.1 删除项

完全删除：

- “周界感知”环形雷达；
- 2.4m RANGE 文字；
- 三个虚构的障碍点；
- SAFE/BLOCKED 的全向感知表达；
- 无真实数据来源的目标框、准星、假 OSD 和假录像状态；
- “避障传感器通过”的通用起飞就绪项；
- 任何 Fake Camera/Fake Target 文案和视觉元素。

### 7.2 保留与替换项

保留：

- 真机视频区域；
- 电量；
- 底部 ToF 离地高度；
- 航向（真机不支持时显示“不可用”）；
- 前向 ToF（可选）；
- 任务控制、停止、长按急停；
- 真实任务日志。

未连接时视频区域显示：

```text
真机未连接
请先连接 RoboMaster TT / Tello Wi-Fi，然后点击“连接真机”
```

连接验证成功但视频失败时显示：

```text
真机已连接，视频流未就绪
重试视频不会触发起飞
```

### 7.3 起飞就绪区域

只显示有真实后端依据的项目：

| 项目 | 数据来源 | 是否阻塞 |
|---|---|---|
| 真机连接已验证 | `ConnectionService.state == VERIFIED` | 是 |
| 健康检查未过期 | `last_success_at` | 是 |
| 电量达到起飞阈值 | `can_start_task()` | 是 |
| 视频流可用 | `VideoHub` 实际收到帧 | 由 FollowSession 起飞前流程最终判定 |
| 前向 ToF | 可选遥测 | 否（除非核心避障显式开启） |

## 8. API 与 WebSocket 数据修改

### 8.1 连接 API

`POST /api/connect`

成功返回：

```json
{
  "ok": true,
  "connection_state": "verified",
  "battery": 82,
  "height": 0,
  "verified_at": "2026-08-18T12:00:00+08:00"
}
```

新增 `GET /api/connection`：

```json
{
  "state": "verified",
  "last_success_age_ms": 630,
  "consecutive_failures": 0,
  "message": "真机 SDK 和电量验证通过"
}
```

### 8.2 起飞检查 API

`GET /api/task/can-start` 返回可审计的检查列表：

```json
{
  "allowed": true,
  "message": "真机连接和电量检查通过",
  "checks": {
    "connection_verified": true,
    "heartbeat_fresh": true,
    "battery_read_ok": true,
    "battery_safe": true,
    "task_inactive": true
  }
}
```

### 8.3 遥测 WebSocket

遥测数据来自后台缓存：

```json
{
  "connection_state": "verified",
  "telemetry_fresh": true,
  "battery": 82,
  "height": 105,
  "yaw": -20,
  "front_distance": null,
  "front_tof_supported": false,
  "mode": "待机",
  "airborne": false,
  "streaming": true,
  "task_active": false
}
```

`front_distance=null` 和 `front_tof_supported=false` 是合法状态，前端不告警、不禁用通用任务按钮。

## 9. 拟修改文件

### 9.1 稳定基线原文件

| 文件 | 修改 |
|---|---|
| `main.py` | 增加 Web 模式；Web 模式拒绝 `--fake` |
| `app/modes.py` | `run_web()` 固定真机路径 |
| `app/config.py` | 读取 Web 配置；不把 Web 模式强制纳入交互式避障提示 |
| `control/follow_session.py` | 增加可选帧发布观察器，默认行为不变 |
| `console/tools.py` | 为 Web 装配注入可选帧发布与任务生命周期回调，默认行为不变 |
| `config.yaml` | 新增 Web 心跳/缓存/端口配置 |
| `requirements.txt` | FastAPI/uvicorn 依赖 |

### 9.2 新增 Web 目录

```text
web/
├── server.py
├── state.py
├── connection.py       # 真机连接状态机与二次验证
├── telemetry_service.py# 串行低频 SDK 采样与缓存
├── video_hub.py         # 单生产者、多消费者视频分发
├── api/
│   ├── drone.py
│   ├── telemetry.py
│   └── video.py
└── frontend/
    └── ...
```

## 10. 开发阶段

### 阶段 0：稳定基线隔离（0.5 天）

- 从 `e45ec66` 创建新分支/独立工作树；
- 不复制 `8f70f46` 的融合避障文件；
- 移植当前 `web/` 和最小 CLI/配置改动；
- 先运行基线测试，记录未改动的起点。

### 阶段 1：移除 Web Fake 路径（0.5 天）

- 删除 Web 入口的 `use_fake`；
- 拒绝 `--mode web --fake`；
- 将单元测试改为依赖注入双体；
- 验证生产代码不存在 Fake Web 可达路径。

### 阶段 2：真机连接状态机（1 天）

- 实现 `ConnectionService`；
- 实现 `command ok + battery + height` 二次验证；
- 实现连续失败降级；
- 实现任务启动时强制再验证。

### 阶段 3：遥测缓存与命令限流（1 天）

- 实现 `TelemetryService`；
- 去除 WebSocket 循环内的直接 SDK 查询；
- 为前向 ToF 实现“支持/不支持/暂时失败”区分；
- 评估真机命令频率和超时。

### 阶段 4：视频所有权改造（1–1.5 天）

- 实现 `VideoHub`；
- 实现单摄像头生产者；
- 实现 Web 预览 ↔ FollowSession 所有权交接；
- 多浏览器客户端不得重复 `stream_on()`；
- 任务中通过帧观察器向 Web 发布真实调试帧。

### 阶段 5：前端真实性收敛（0.5–1 天）

- 删除周界雷达和虚构 OSD；
- 连接、健康度、电量和视频状态全部由后端真实数据驱动；
- 前向 ToF 显示为可选能力；
- 重写起飞就绪检查区域。

### 阶段 6：测试与真机分级验证（1–2 天）

- 离线单元测试；
- 不起飞的 Wi-Fi/SDK 连接测试；
- 不起飞的视频流测试；
- 多浏览器观看测试；
- 断网恢复测试；
- `follow-dry-run` 检查；
- 最后才进行低风险真机起飞验证。

估计工期：**4.5–6.5 天**，不包括真机现场排期和硬件故障排查时间。

## 11. 测试计划

### 11.1 离线自动化测试

1. Web 模式拒绝 `--fake`；
2. `command` 超时时不进入 `VERIFIED`；
3. `command=ok` 但 `battery?` 失败时不进入 `VERIFIED`；
4. 电量超出 `0–100` 时连接失败；
5. 连续 3 次心跳失败进入 `DEGRADED`；
6. 心跳过期时 `/api/task/start` 拒绝执行；
7. 电量低于阈值时任务拒绝执行；
8. 前向 ToF 不支持时仍可通过通用 Web 连接检查；
9. 两个 MJPEG 客户端只产生一次 `stream_on()`；
10. Web 预览交接给 FollowSession 前确实释放解码器；
11. 任务结束后只恢复一个 Web 视频生产者；
12. WebSocket 推送不直接调用 SDK 读取方法。

### 11.2 真机不起飞验证

1. 未连接 Tello Wi-Fi：连接失败，无视频，无任务按钮；
2. 连接 Tello Wi-Fi：`command` 和 `battery?` 均成功；
3. 视频预览只有一个解码器；
4. 两台队员电脑同时访问，画面不重启；
5. 断开 Wi-Fi，连接状态在规定失败次数后变为 `DEGRADED`；
6. 断网后“开始跟随”必须禁用；
7. 重连后必须重新验证，不沿用旧状态。

### 11.3 真机起飞前门禁

只有以下证据同时满足时才进入真机起飞：

- 真机连接与视频不起飞测试通过；
- 断网恢复测试通过；
- 所有离线自动化测试通过；
- `follow-dry-run` 无异常；
- 现场安全员确认防护罩、空域、电量和急停路径。

## 12. 验收标准

### 12.1 必须通过

- Web 生产启动路径不能构建 `FakeDroneAdapter`；
- 打开网页不会自动连接真机；
- 未经验证的真机不会启动视频流；
- 未经验证的真机不会创建飞行任务；
- 连接验证包含 `command=ok` 和有效电量回复；
- 任务启动前重新检查连接新鲜度和电量；
- 前向 ToF 缺失不会被 WebUI 误判为通用启动失败；
- 页面不存在周界雷达、虚构障碍点和虚构传感器状态；
- 多个观看者共享同一视频生产者；
- FollowSession 任务期间只有一个摄像头所有者；
- 原 `e45ec66` 测试集与新增 Web 测试全部通过。

### 12.2 未验收前禁止声称

在未完成真机不起飞和分级起飞验证前，不得声称：

- 真机视频已稳定；
- 断网自动恢复已验证；
- 多队员同时观看已验证；
- 前向 ToF 已在当前硬件上可用；
- WebUI 真机飞行链路已完成。

## 13. 风险与回滚

| 风险 | 影响 | 缓解 |
|---|---|---|
| Tello 命令通道被遥测占用 | 超时/任务抖动 | 串行采样、缓存推送、降频 |
| 视频解码器被重复启动 | 黑屏/线程残留 | 单例 VideoHub+所有权交接 |
| 心跳误报断线 | 无法开始新任务 | 连续失败阈值+明确重连 |
| 前向 ToF 不存在 | 页面显示缺失 | 标记为可选，不作 Web 通用门禁 |
| 帧观察器影响 FollowSession | 控制循环变慢 | 非阻塞最新帧覆盖队列，满时丢帧 |

回滚点：新分支始终以 `e45ec66` 为父基线，各阶段独立提交。任一阶段验证失败时，回退当阶段 Web 适配提交，不回退或改写稳定基线。

## 14. 实施前需要用户确认的最终范围

本计划默认采用以下决策：

1. WebUI 仅真机，但不删除项目其他模式的 Fake 测试能力；
2. 删除前端周界雷达，保留可选前向 ToF 数值；
3. 无前向 ToF 的当前硬件配置 `obstacle.enabled=false`；
4. 页面可在真机已验证但未起飞时显示真实摄像头预览；
5. 任务期间继续显示 FollowSession 发布的真实帧；
6. 多队员页面只读视频/遥测的权限分离，不在本阶段默认允许所有队员控制真机。

第 6 点是团队分享与真机安全的必要要求：实施时建议进一步增加“主控端/只读观众端”权限分离。
