# DroneUmbrella

基于 RoboMaster TT / Tello Talent 的无人机移动打伞系统 Python 原型项目。

## 项目目标

本项目面向中国大学生物联网竞赛，目标是完成一个“低空移动遮雨无人机系统”的缩比验证原型。初赛阶段先使用 RoboMaster TT / Tello Talent 小型无人机进行功能验证，重点实现安全起降、视觉目标检测、跟随控制、基础状态展示和后续多机协同仿真接口。

## 硬件平台

- RoboMaster TT / Tello Talent 小型无人机
- 电脑或开发板作为地面控制端
- 无人机自带摄像头
- 后续可扩展：轻量化伞面结构、定位标签、外部传感器、地面基站

## 软件平台

- Python 3.9+
- Tello SDK / RoboMaster TT SDK
- `djitellopy` 用于无人机通信控制
- OpenCV 用于摄像头图像处理与目标检测
- NumPy 用于数值计算
- PyYAML 用于读取配置文件
- Matplotlib 用于仿真与结果可视化

## 适配器架构

项目通过 `DroneAdapter` 统一抽象无人机连接、起飞、降落、视频流、状态读取和 RC 控制接口。当前 `TelloDroneAdapter` 是 RoboMaster TT / Tello Talent 的具体实现，`FakeDroneAdapter` 用于无真机仿真验证。

后续如果更换为其他支持编程接口的大型无人机，应新增对应的适配器实现 `DroneAdapter`，而不是让视觉、控制、Agent 或安全模块直接依赖新的硬件 SDK。当前项目还没有实现真实大型无人机适配器，也不声称已经支持大型无人机真机控制。

## 运行方式

1. 进入项目目录：

   ```bash
   cd ~/Desktop/物联网竞赛/DroneUmbrella
   ```

2. 创建并启用虚拟环境，安装依赖：

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

3. 运行主程序骨架：

   ```bash
   python3 main.py
   ```

4. 查看无人机状态：

   ```bash
   python3 main.py --mode status
   ```

5. 打开摄像头识别调试画面：

   ```bash
   python3 main.py --mode camera
   ```

6. 运行低速目标跟随模式：

   ```bash
   python3 main.py --mode follow
   ```

   follow 模式会连接 RoboMaster TT / Tello、开启视频流，并在用户输入 `YES` 确认后才进入起飞流程。起飞后会根据红色目标块位置进行低速跟随：水平偏差优先使用 `yaw` 原地转向，距离偏差使用目标面积比例控制前进或后退。按 `q` 退出并降落，按 `e` 立即发送零速度并降落。

   普通 `follow` 模式和规则版 `agent` 的跟随任务已经复用同一套 `FollowSession` 核心流程。两者外层入口不同，但每帧的画面读取、红色目标识别、跟随控制、安全限速、目标丢失处理、RC 输出和窗口清理逻辑保持一致。

   跟随调试画面会显示 `FPS`、`CTRL_HZ`、水平误差、面积比例和当前 RC 指令。当前单机人物跟随模式不主动使用左右平移，也不加入高度跟随或 yaw 以外的姿态控制扩展。

7. 运行导入测试：

   ```bash
   PYTHONPATH=. python3 tests/test_imports.py
   ```

## 规则版 Agent

当前 Agent 已支持两层命令解析：

1. **本地规则解析**：优先识别固定命令和一组常见自然语言表达。
2. **在线大模型解析**：当本地规则无法确定意图，且 `llm_enabled: true` 时，再调用 OpenAI 兼容接口做动作分类。

无论本地规则还是在线模型，最终都只能映射为白名单动作：`GET_STATUS`、`START_FOLLOW`、`STOP_TASK`、`EMERGENCY_STOP`、`EXIT`、`UNKNOWN`。Agent 只负责理解高层任务和调度任务，不直接接触底层飞控 SDK。

Agent 只能调用 `AgentTools` 提供的安全工具。起飞前会经过 `SafetyManager` 电量检查，跟随控制量会经过 `SafetyManager.limit_rc_command` 限速。大模型也只能复用这些任务级工具，不能直接调用 `djitellopy`、`tello.send_rc_control` 或绕过安全层。

启动真机规则版 Agent：

```bash
python3 main.py --mode agent
```

无真机时可以运行：

```bash
python3 main.py --mode agent --fake
```

如需启用在线大模型分类，在 `config.yaml` 中设置：

```yaml
llm_enabled: true
llm_base_url: "https://api.openai.com/v1/chat/completions"
llm_model: "gpt-4o-mini"
llm_timeout_seconds: 8
```

并在运行前设置环境变量：

```bash
export LLM_API_KEY="你的接口密钥"
```

如果 `llm_enabled: true` 但没有设置 `LLM_API_KEY`，Agent 启动时会给出提示，并自动退回到仅使用本地规则解析。

支持的输入包括固定命令和常见自然语言表达：

- `状态` / `看看现在无人机状态` / `还有多少电`：显示电量、高度和当前模式。
- `开始任务` / `帮我开始跟随目标` / `启动无人机跟随`：检查电量，等待用户确认，然后起飞并打开 `DroneUmbrella Agent Follow` 窗口进行低速跟随；跟随运行时请使用窗口按键 `p`、`q`、`e` 控制暂停、停止和急停。
- `停止任务` / `先停一下` / `停止当前任务`：在未进入跟随窗口时停止当前任务并降落。
- `急停` / `立即急停` / `紧急停止`：在未进入跟随窗口时清零当前控制输出；跟随窗口运行中请直接按 `e` 急停。
- `退出` / `退出系统`：安全结束当前任务并退出 Agent。

含糊、询问式或不确定的动作表达会返回 `UNKNOWN`，不会触发起飞、停止、急停或退出。

如果本地规则和在线模型都无法安全确定意图，系统会返回 `UNKNOWN`，不会执行危险动作。

Agent 跟随窗口显示实时画面、目标框、目标中心、画面中心、误差线、Agent 状态、REAL/FAKE 模式、电量、高度和当前 RC 控制量。窗口按键：

- `p`：暂停或继续跟随。暂停时继续显示画面和识别结果，但 RC 输出固定为 `0,0,0,0`。
- `q`：停止跟随、清零控制量、降落、关闭视频流和窗口，然后返回 `Agent>` 命令界面。
- `e`：急停，立即清零控制量并降落，Agent 状态切换为 `EMERGENCY_STOP`。

后续接入大模型 API 时，大模型只用于把自然语言转换为上述任务命令和编排任务，底层实时飞控仍由确定性的控制器和安全工具负责。

## 无真机验证方法

当前没有 RoboMaster TT / Tello 真机时，可以使用 `--fake` 进入模拟无人机模式。带 `--fake` 时程序不会连接真机，而是使用 `FakeDroneAdapter` 返回模拟电量、高度和动态 OpenCV 测试画面。不带 `--fake` 时仍然连接 RoboMaster TT / Tello 真机，真机控制逻辑保持不变。

Fake 画面会生成真实 NumPy/OpenCV 图像：红色目标会左右、上下移动，大小会周期变化，并会短时间模拟目标丢失。画面仍然经过 `TargetDetector.detect(frame)`、`FollowController`、`SafetyManager` 和 `FakeDroneAdapter.move_rc()`，不是直接伪造识别结果。

示例命令：

```bash
python3 main.py --mode status --fake
python3 main.py --mode camera --fake
python3 main.py --mode follow --fake
python3 main.py --mode agent --fake
```

模拟模式适合验证状态读取、红色目标识别和跟随控制理论逻辑；真实飞行前仍必须重新做安全检查。

Fake Agent 手动验收建议：

1. 运行 `python3 main.py --mode agent --fake`。
2. 输入 `状态`，确认显示模拟电量和高度。
3. 输入 `开始任务`，再输入 `yes`。
4. 确认出现 `DroneUmbrella Agent Follow` 窗口。
5. 确认红色目标会移动、大小变化，目标框和中心点正常。
6. 确认 RC 数据随目标位置和面积变化。
7. 按 `p` 暂停，确认画面继续刷新但 RC 为 0。
8. 再按 `p` 恢复。
9. 按 `q` 停止并返回 Agent 命令界面。
10. 再次输入 `开始任务`，确认不会出现重复窗口。
11. 按 `e` 测试急停状态。

真机 Agent 安全验收建议：

1. 连接 RoboMaster TT / Tello Wi-Fi。
2. 运行 `python3 main.py --mode agent`。
3. 输入 `状态`，确认电量满足起飞阈值。
4. 输入 `开始任务`，按提示完成用户确认。
5. 起飞后确认电脑显示实时摄像头画面。
6. 红色目标出现时确认检测框、误差线和 RC 数据正常。
7. 按 `p` 暂停，确认无人机悬停。
8. 按 `q` 停止并降落。
9. 确认视频流和窗口关闭。

## 四机协同打伞仿真

初赛阶段的实物验证以单台 RoboMaster TT / Tello Talent 缩比原型为主，优先验证视觉识别、低速跟随和安全保护。四机协同打伞目前只做算法仿真，不控制真实四架无人机。

仿真采用“虚拟结构法”：把行人目标中心记为 `target=(x, y, z)`，设 `d` 为四架无人机相对伞面中心的水平偏移，`h` 为相对目标中心的飞行高度。四架无人机目标位置为：

```text
drone_1 = target + (-d, +d, h)
drone_2 = target + (+d, +d, h)
drone_3 = target + (-d, -d, h)
drone_4 = target + (+d, -d, h)
```

运行命令：

```bash
python3 main.py --mode swarm-sim
```

程序会在终端输出四架无人机的目标坐标，并用 matplotlib 生成二维示意图，显示行人目标、伞面中心、四架无人机位置和伞面矩形。真实多机控制、通信同步和避障逻辑留到后续阶段扩展。

## 开发路线

- 第 1 阶段：完成项目骨架、配置文件、模块边界和导入测试。
- 第 2 阶段：接入 Tello / RoboMaster TT，完成连接、起飞、降落、电量读取和基础安全检查。
- 第 3 阶段：接入摄像头视频流，完成颜色目标或标记目标检测。
- 第 4 阶段：实现低速跟随控制，让无人机保持在目标上方或附近的安全位置。
- 第 5 阶段：增加异常处理，例如低电量降落、目标丢失悬停、目标长时间丢失自动降落。
- 第 6 阶段：开发简单仪表盘，显示电量、高度、目标状态和控制状态。
- 第 7 阶段：开展多机编队与遮雨覆盖范围仿真，为后续实物扩展做准备。

## 安全注意事项

- 首次调试必须拆除桨叶或使用保护罩，确认控制逻辑无误后再带桨测试。
- 室内飞行要选择空旷区域，远离人群、玻璃、灯具和易损物品。
- 起飞前检查电量，默认低于 30% 不允许起飞。
- 飞行中电量低于 20% 时应立即降落。
- 控制速度和高度必须受配置文件限制，避免无人机快速冲撞。
- follow 模式只允许低速跟随，所有 RC 输出都必须经过 `SafetyManager.limit_rc_command` 限速。
- follow 模式的水平跟随优先使用偏航旋转，不再主要依靠左右平移追踪目标；目标水平偏差很大时会抑制前进速度，避免未对准时前冲。
- follow 模式起飞前必须由用户手动输入 `YES` 确认，程序不能自动起飞。
- 调试 follow 模式时先拆除桨叶或使用保护罩，确认红色目标识别和控制方向正确后再短时间带桨测试。
- 跟随测试时目标移动要缓慢，禁止让无人机靠近人脸、头顶、玻璃、灯具和易损物品。
- 目标丢失后先悬停观察，长时间丢失后自动降落。
- follow 画面中按 `q` 退出并降落，按 `e` 立即发送零速度并降落。
- 本项目初赛阶段只做缩比验证，不直接承载真实雨伞或靠近人体飞行。
