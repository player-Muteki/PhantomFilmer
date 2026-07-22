# PhantomFilmer

基于 RoboMaster TT / Tello Talent 的无人机自动跟拍系统 Python 原型。

## 依赖

- `djitellopy` — 无人机通信控制
- `opencv-python` — 摄像头图像处理与目标检测
- `numpy` — 数值计算
- `pyyaml` — 配置文件读取
- `matplotlib` — 四机编队仿真可视化

## 适配器架构

`DroneAdapter` 是统一抽象基类，定义 `connect/takeoff/land/stop/move_rc/get_battery/get_height/stream_on/stream_off/get_frame` 接口。其他模块只依赖此接口，不直接导入硬件 SDK。

- **TelloDroneAdapter** — RoboMaster TT / Tello Talent 真机实现，唯一直接导入 `djitellopy` 的模块。
- **FakeDroneAdapter** — 模拟实现，生成动态 OpenCV 测试画面（红色目标左右/上下移动、大小周期变化、间歇丢失），无真机时使用 `--fake` 启用。

如需接入其他型号无人机，新增 `DroneAdapter` 子类即可。

## 运行

```bash
pip install -r requirements.txt
python3 main.py --mode <mode> [--fake]
```

### 模式列表

| 模式 | 说明 | 支持 --fake |
|------|------|-------------|
| `demo`（默认） | 显示系统描述 | 是 |
| `status` | 连接无人机，读取电量与高度 | 是 |
| `camera` | 开启视频流与目标检测画面（红色或 ArUco，取决于 `detector_type`） | 是 |
| `camera-debug` | 同时显示 BGR 原图、BGR/RGB 互换、红色掩膜三个窗口，不发送 move_rc | 是 |
| `follow` | 起飞 → 目标跟随（低速、yaw 优先），窗口按键 `q` 停止降落、`e` 急停 | 是 |
| `follow-dry-run` | 只计算理论控制量并显示在画面上，不起飞、不发送 move_rc | 是 |
| `follow-test` | 单元测试 FollowController 方向逻辑，不需无人机和摄像头 | 否 |
| `fixed-demo` | 固定航线（左移 3s → 前进 2s → 右移 3s）→ 自动进入目标跟随 | 是 |
| `console` | 自然语言控制台 REPL，支持本地规则+LLM 回退 | 是 |
| `basic-flight-test` | 连接 → 读取电量 → 用户确认 → 起飞 → 悬停 5s → 降落 | 否 |
| `safety-test` | 单元测试 SafetyManager 电量/限速/高度/目标丢失逻辑 | 否 |
| `swarm-sim` | 虚拟结构法编队仿真：四机分布在目标中心矩形四角，输出坐标并保存二维示意图。`drone_i=target+(±d,±d,h)` | 否 |
| `swarm-status` | 多机状态读取，不起飞、不开视频 | 是 |
| `swarm-connect-test` | 多机连接 + 零 RC + 急停 | 是 |
| `swarm-basic-test` | 多机连接 → 顺序起飞 → 清零 → 顺序降落 → 急停 | 是 |
| `swarm-hover-test` | 多机顺序起飞 → 同步悬停 → 顺序降落 | 是 |
| `swarm-rc-test` | 多机顺序起飞 → 低速短时 RC 移动 → 清零 → 降落 | 是 |

### follow 窗口按键

- `p` — 暂停/继续（仅 console 模式支持）
- `q` — 停止跟随 + 降落
- `e` — 急停：立即清零控制量 + 降落

## 检测器

`config.yaml` 中 `vision.detector_type` 决定检测器类型：

- **`red`**（默认）— 红色目标检测，使用 HSV 颜色阈值 + RGB 主色过滤
- **`aruco`** — ArUco 二维码检测（默认 DICT_4X4_50, ID 23），支持坐标/面积平滑和临时丢失容忍

```bash
# 打印 ArUco 标记
python3 vision/generate_marker.py
```

两种检测器的输出接口相同（`found`、`center`、`area`、`bbox`），下游控制器无需改动。

## 控制逻辑

`FollowController` 将目标检测结果转换为安全 RC 指令：

1. **水平**：优先 yaw 转向，不使用左右平移。误差在死区内不输出，死区外按比例计算偏航速度并施加最小速度约束。
2. **前后**：根据目标面积比例控制前进/后退。面积小于阈值则前进，大于阈值则后退，在中间范围悬停。
3. **上下**：目标在画面中偏上则上升，偏下则下降。
4. **对准减速**：存在偏航或上下修正时，前后速度固定为较低值，避免姿态未稳定时冲撞。
5. **目标锁定**：目标在画面中心且面积适中稳定持续 15 帧后进入 LOCKED 状态，四轴全部归零停止移动；目标移出退出范围后解除锁定。
6. **目标丢失**：丢失后先悬停等待，超过 `target_lost_hover_seconds` 继续悬停，超过 `target_lost_land_seconds` 自动降落。

所有 RC 指令经过 `SafetyManager.limit_rc_command` 限速（`max_rc_speed`）。

## 安全

`SafetyManager` 负责单机安全检查：
- 起飞前电量低于 `min_battery_takeoff` 禁止起飞
- 飞行中电量低于 `low_battery_land` 建议降落
- `limit_rc_command` 每通道钳位到 `[-max_rc_speed, max_rc_speed]`
- `check_height` 确保高度在 `[min_height_cm, max_height_cm]`

## 控制台

```bash
python3 main.py --mode console      # 真机
python3 main.py --mode console --fake  # 模拟
```

自然语言控制台采用两层命令解析：

1. **本地规则**：优先匹配固定命令和自然语言关键词。
2. **在线 LLM 回退**：本地无法确定且 `llm_enabled: true` 且有 `LLM_API_KEY` 时，调用 OpenAI 兼容 API 做动作分类。

所有解析结果映射为白名单动作：`GET_STATUS`、`START_FOLLOW`、`STOP_TASK`、`EMERGENCY_STOP`、`EXIT`、`UNKNOWN`。

控制台只能通过 `ConsoleTools` 调用任务级工具，`ConsoleTools` 内部经过 `SafetyManager` 安全检查。大模型不直接操控飞控。

启用 LLM 分类需设置：
```yaml
llm_enabled: true
llm_base_url: "https://api.deepseek.com/chat/completions"
llm_model: "deepseek-chat"
```
并设置环境变量 `LLM_API_KEY`。未设置密钥时控制台自动退回到仅使用本地规则。

支持的自然语言输入：

- `状态` / `还有多少电` — 显示电量、高度、当前模式
- `开始任务` / `帮我开始跟随目标` — 检查电量 → 用户确认 → 起飞 → 跟随
- `停止任务` / `先停一下` — 停止跟随并降落
- `急停` / `紧急停止` — 清零控制输出
- `退出` / `退出系统` — 安全结束并退出

含糊、询问式表达（含"吗"、"？"等）返回 `UNKNOWN`，不执行任何动作。

## Fake 模式

无真机时通过 `--fake` 启用 `FakeDroneAdapter`：

- 返回模拟电量（80%）和高度
- 生成包含动态红色目标的 OpenCV 测试画面
- 目标会左右/上下移动、大小周期变化、间歇丢失
- 画面仍然经过 `TargetDetector.detect`、`FollowController`、`SafetyManager` 和 `FakeDroneAdapter.move_rc`，不是直接伪造识别结果

Fake 模式无法验证 ArUco 识别（Fake 画面只生成红色目标）。使用 ArUco 时需使用真实摄像头。

## 仪表盘

`ui/dashboard.py` — 占位骨架，供未来遥测显示使用。

## 测试

```bash
python3 -m pytest tests/
```
