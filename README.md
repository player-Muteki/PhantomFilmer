# PhantomFilmer

基于 RoboMaster TT / Tello Talent 的无人机自动跟拍系统 Python 原型。

## 依赖

- `djitellopy` - 无人机通信控制
- `opencv-contrib-python` - 摄像头图像处理、目标检测与 ArUco 支持
- `numpy` - 数值计算
- `pyyaml` - 配置文件读取
- `matplotlib` - 四机编队仿真可视化

## 适配器架构

`DroneAdapter` 是统一抽象基类，定义 `connect/takeoff/land/stop/move_rc/get_battery/get_height/stream_on/stream_off/get_frame` 接口。其他模块只依赖此接口，不直接导入硬件 SDK。

- **TelloDroneAdapter** - RoboMaster TT / Tello Talent 真机实现，唯一直接导入 `djitellopy` 的模块。
- **FakeDroneAdapter** - 模拟实现，按检测器配置生成动态红色目标或 ArUco 标记，支持移动、大小变化和间歇丢失；无真机时使用 `--fake` 启用。

如需接入其他型号无人机，新增 `DroneAdapter` 子类即可。

## 运行

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python main.py --mode <mode> [--fake]
```

开发和测试环境使用 `requirements-dev.txt`。OpenCV 核心包与 contrib 包固定为同一版本，并让 contrib 最后安装，以兼容 `djitellopy` 的传递依赖和 ArUco 模块。

### 模式列表

| 模式 | 说明 | 支持 --fake |
|------|------|-------------|
| `demo`（默认） | 显示系统描述 | 是 |
| `status` | 连接无人机，读取电量与高度 | 是 |
| `camera` | 开启视频流与目标检测画面 | 是 |
| `camera-debug` | 显示 BGR 原图、通道互换和红色掩膜，不发送 RC | 是 |
| `follow` | 起飞并目标跟随，支持按次选择是否开启避障 | 是 |
| `follow-dry-run` | 只计算理论控制量和避障结果，不起飞、不发送 RC | 是 |
| `follow-test` | 测试 FollowController 方向逻辑 | 否 |
| `fixed-demo` | 固定航线（左移 3s -> 前进 2s -> 右移 3s）后自动跟随 | 是 |
| `console` | 自然语言控制台，支持本地规则和 LLM 回退 | 是 |
| `basic-flight-test` | 用户确认后起飞、悬停 5s、降落 | 否 |
| `safety-test` | 测试电量、限速、高度和目标丢失逻辑 | 否 |
| `swarm-sim` | 四机虚拟结构编队仿真 | 否 |
| `swarm-status` | 多机状态读取，不起飞、不开视频 | 是 |
| `swarm-connect-test` | 多机连接、零 RC、急停 | 是 |
| `swarm-basic-test` | 多机连接、顺序起飞、清零、顺序降落 | 是 |
| `swarm-hover-test` | 多机顺序起飞、同步悬停、顺序降落 | 是 |
| `swarm-rc-test` | 多机低速短时移动、清零、降落 | 是 |

### follow 窗口按键

- `p` - 暂停/继续（仅 console 模式支持）
- `q` - 停止跟随并降落
- `e` - 急停，立即清零控制量并降落

## 检测器

`config.yaml` 中 `vision.detector_type` 决定检测器类型：

- **`red`** - 红色目标检测，使用 HSV 颜色阈值和 RGB 主色过滤
- **`aruco`** - ArUco 标记检测，支持坐标/面积平滑和临时丢失容忍
- **`person_reid`** - 实验性人物 ReID，使用 YOLO 行人检测和 Torchreid/OSNet 外观特征匹配

当前默认选择 `aruco`；检测器工厂在未配置类型时使用 `red`。

```bash
python3 tools/generate_aruco_marker.py
```

各检测器保持相同的基础输出接口（`found`、`center`、`area`、`bbox`），下游控制器无需按检测器类型分支。

## 人物 ReID

人物 ReID 是实验模式，只替换视觉检测器；现有跟随控制、安全限速、目标丢失悬停和自动降落逻辑保持不变。模型会在飞行会话起飞前预加载，加载失败时不会起飞。

建议创建独立环境并运行仓库提供的分步安装脚本：

```bash
bash scripts/setup_reid_env.sh python3
```

模型权重、人物照片和数据集不会提交到 Git。所需材料和校验方式见 `docs/reid_materials.md`，配置示例见 `config.reid.offline-snippet.yaml`。典型本地文件包括：

```text
weights/yolov8n.pt
weights/osnet_x0_25_msmt17.pth
data/reid_target/front.jpg
data/reid_target/side.jpg
```

先使用不发送飞控指令的模式验证：

```bash
.venv-reid/bin/python main.py --mode follow-dry-run
MPLCONFIGDIR=.matplotlib YOLO_CONFIG_DIR=.ultralytics \
  .venv-reid/bin/python tools/reid_offline_eval.py
```

已有离线结果见 `docs/reid_test/`。ReID 只进行外观匹配，不识别真实姓名；俯视、遮挡、换衣、逆光和低分辨率都会降低可靠性。完成真实目标视频测试以前，不得启用真机 ReID 起飞跟随，ArUco 模式必须保留为安全降级方案。

## 控制逻辑

`FollowController` 将目标检测结果转换为安全 RC 指令：

1. **水平**：优先 yaw 转向，不使用左右平移。误差在死区内不输出，死区外按比例计算偏航速度并施加最小速度约束。
2. **前后**：根据目标面积比例控制前进/后退。面积小于阈值则前进，大于阈值则后退，在中间范围悬停。
3. **上下**：目标在画面中偏上则上升，偏下则下降。
4. **对准减速**：存在偏航或上下修正时降低前后速度，避免姿态未稳定时冲撞。
5. **目标锁定**：目标在画面中心且面积适中稳定持续指定帧数后进入 LOCKED 状态；目标移出退出范围后解除锁定。
6. **目标丢失**：丢失后先悬停，超过 `target_lost_land_seconds` 自动降落。

所有 RC 指令经过 `SafetyManager.limit_rc_command` 限速。

## 障碍避让

`follow`、`follow-dry-run` 和 `console` 启动时会询问本次是否开启视觉避障，默认值来自 `config.yaml` 的 `obstacle.enabled`。避障检测器会排除目标区域，并在风险区域出现障碍时限制前进或输出绕行动作；最终命令仍经过 `SafetyManager`。

该功能基于单目画面的轮廓和面积启发式判断，不提供真实深度测量。真机使用前必须先在 `follow-dry-run` 中标定，并保留人工急停和净空检查。

## 安全

`SafetyManager` 负责单机安全检查：

- 起飞前电量低于 `min_battery_takeoff` 禁止起飞
- 飞行中电量低于 `low_battery_land` 建议降落
- 飞行高度超出 `[min_height_cm, max_height_cm]` 时立即清零并降落
- `limit_rc_command` 将每个通道限制在 `[-max_rc_speed, max_rc_speed]`
- 目标丢失、身份歧义和外部停止请求都会优先产生零控制输出

## 控制台

```bash
.venv/bin/python main.py --mode console
.venv/bin/python main.py --mode console --fake
```

自然语言控制台先使用本地规则，无法确定时可调用 OpenAI 兼容接口做白名单动作分类。所有结果只会映射为 `GET_STATUS`、`START_FOLLOW`、`STOP_TASK`、`EMERGENCY_STOP`、`EXIT` 或 `UNKNOWN`。

控制台只能通过 `ConsoleTools` 调用任务级工具，底层输出必须经过 `SafetyManager`。跟随任务在后台线程运行，停止、急停和退出命令会清零输出并等待降落清理完成。

启用 LLM 分类需在配置中设置 `llm_enabled: true`，并提供环境变量 `LLM_API_KEY`；未设置密钥时自动退回本地规则。

## Fake 模式

无真机时通过 `--fake` 启用 `FakeDroneAdapter`：

- 返回模拟电量和高度
- 按 `vision.detector_type` 生成动态红色目标或 ArUco 标记
- 目标会移动、缩放并间歇丢失
- 画面仍经过检测器、控制器、安全层和模拟 RC 输出链路

Fake ArUco 可验证软件链路，但无法复现光照、运动模糊和无线视频延迟。`person_reid` 仍需要真实参考图、模型权重和可检测的人物画面。

## 编队反馈保护

真机 `send_rc_all` 默认要求外部位置跟踪器为所有节点提供新鲜的编队修正。反馈过期或缺少任意节点时，非零编队指令会被清零并拒绝。Fake Swarm 为软件测试可跳过该门禁。

## 测试

```bash
.venv/bin/python -m pytest tests/
```
