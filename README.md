# PhantomFilmer

基于 RoboMaster TT / Tello Talent 的无人机自动跟拍系统原型。Python CLI 提供开发、
模拟和诊断模式；Electron GUI 已接入同一自动任务与安全内核，面向真机操作员提供从人物
建档、地面确认、起飞、跟随、手动接管到降落和诊断的完整工作流。

## 依赖

- `djitellopy` - 无人机通信控制
- `opencv-python` - 摄像头图像处理与调试画面
- `numpy` - 数值计算
- `pyyaml` - 配置文件读取

## 适配器架构

`DroneAdapter` 是统一抽象基类，定义 `connect/takeoff/land/stop/move_rc/get_battery/get_height/stream_on/stream_off/get_frame` 接口。其他模块只依赖此接口，不直接导入硬件 SDK。

- **TelloDroneAdapter** - RoboMaster TT / Tello Talent 真机实现，唯一直接导入 `djitellopy` 的模块。
- **FakeDroneAdapter** - 模拟实现，生成动态人物轮廓，支持移动、大小变化和间歇丢失；无真机时使用 `--fake` 启用。

如需接入其他型号无人机，新增 `DroneAdapter` 子类即可。

### 模拟与真机行为差异

`--fake` 模式用于无硬件验证，但模拟器与真机存在以下行为差异，`follow-dry-run` 等模拟验证通过不代表真机行为一致：

- **起飞确认**：真机 `takeoff()` 始终要求用户输入 `YES`（或经外层工作流 `authorize_next_takeoff()` 预授权）；模拟器直接置高 70 cm，不要求确认。
- **急停与停止**：真机 `stop()` 会发送零 RC 并关闭视频流；模拟器 `stop()` 只打印"模拟急停"，不修改内部状态。
- **电量与高度**：模拟器电量固定 80%、高度由起飞/降落直接设定；真机从飞控读取实时值，存在读取失败（降级为 `None` 并重试）的可能。
- **网络与延迟**：模拟器不模拟 Wi-Fi 延迟、丢包和视频流卡顿；真机控制循环的帧率与指令送达率会显著波动。
- **目标丢失**：模拟目标按固定周期消失/重现；真机目标丢失受光照、遮挡、运动模糊等影响，不可控。

因此，任何真机飞行前都应先跑 `follow-dry-run` 验证控制量与避障决策，再按安全提示执行真机测试。

## 运行

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python main.py --mode <mode> [--fake]
```

开发和测试环境使用 `requirements-dev.txt`。人物 ReID 的模型依赖较重，建议使用下文的独立环境安装脚本。

### 模式列表

| 模式 | 说明 | 支持 --fake |
|------|------|-------------|
| `demo`（默认） | 显示系统描述 | 是 |
| `status` | 连接无人机，读取电量与高度 | 是 |
| `connection-test` | 只验证 `command -> ok` 和 `battery?`，不飞行、不打开摄像头 | 是 |
| `camera` | 开启视频流与目标检测画面 | 是 |
| `follow` | 起飞并目标跟随，支持按次选择是否开启避障 | 是 |
| `reid-enroll` | 从参考照片创建本地具名人物特征档案，不连接无人机 | 否 |
| `reid-demo` | 现场录入人物，人工授权起飞后识别并跟随 | 是（不代表真实识别与动力学） |
| `follow-dry-run` | 只计算理论控制量和避障结果，不起飞、不发送 RC | 是 |
| `follow-test` | 测试 FollowController 方向逻辑 | 否 |
| `console` | 自然语言控制台，支持本地规则和 LLM 回退 | 是 |
| `basic-flight-test` | 用户确认后起飞、悬停 5s、降落 | 否 |
| `safety-test` | 测试电量、限速、高度和目标丢失逻辑 | 否 |

连接 `RMTT-XXXXXX` 后，可先独立验证 SDK UDP 通信：

```bash
.venv/bin/python main.py --mode connection-test
```

真机固定使用 `192.168.10.1:8889`。该模式只发送 SDK 握手和电量查询，
不会起飞、开启视频流或启动跟随控制。

### follow 窗口按键

- 到达 150 cm 后会一直悬停等待选择：`m` 手动、`a` 普通自动、`s` 侧向、`f` 前向；没有默认选择或超时
- 自动飞行中：`1/2/3` 安全切换普通/侧向/前向模式；切换期间悬停并重新确认人物
- 手动模式：`w/s` 前后，`a/d` 左右横移，`r/f` 升降，`j/l` 左右偏航，空格立即悬停
- 手动模式按 `m` 切回自动；程序先悬停并要求连续 5 帧新鲜 ReID，确认完成后的下一帧才允许自动运动
- `p` - 暂停/继续（仅 console 模式支持）
- `q` - 停止跟随并降落
- `e` - 急停，立即清零控制量并降落

手动方向键采用 250 ms 看门狗；按键没有持续刷新就自动清零。手动前进时，
前向 ToF 有效距离 `<= 60 cm` 或读数无效/过期只会禁止前进，不会启动自动右移；
后退、横移、升降和偏航仍按各自的高度安全限制执行。ToF 超量程表示前方远距离，
允许前进。启用这项保护后，顶部 ToF 是起飞前的必检硬件，即使本次关闭自动避障也一样。

手动选择依赖 OpenCV 摄像头窗口；若 `display_console_camera: false`，系统没有键盘
输入源，会安全停止并降落。

## 识别链路

系统只保留人物 ReID：YOLO26n 检测画面中的行人，Torchreid/OSNet 提取外观特征，
再与本地参考照片或具名档案进行余弦相似度匹配。`create_detector()` 始终创建
`PersonReIDDetector`，不存在运行时识别策略开关。

检测结果使用稳定的基础接口（`found`、`center`、`area`、`bbox`、`ambiguous`），
供跟随、搜索、避障和安全模块消费。

## 人物 ReID

人物 ReID 是实验模式，复用现有跟随控制和安全限速，并增加有界的目标丢失搜索。模型会在飞行会话起飞前预加载，加载失败时不会起飞。

建议创建独立环境并运行仓库提供的分步安装脚本：

```bash
bash scripts/setup_reid_env.sh python3
```

模型权重、人物照片和数据集不会提交到 Git。所需材料和校验方式见 [docs/06-配置测试与源码索引.md 第 4 节](docs/06-配置测试与源码索引.md#4-依赖与本地数据)，ReID 模型加载与档案校验见 [docs/03-视觉感知与目标跟随.md](docs/03-视觉感知与目标跟随.md)。配置示例见 `config.reid.offline-snippet.yaml`。典型本地文件包括：

```text
weights/yolo26n.pt
weights/osnet_x0_25_msmt17.pth
data/reid_target/front.jpg
data/reid_target/side.jpg
```

项目当前使用 COCO 预训练 YOLO26n。程序在飞行现场设置 `YOLO_OFFLINE=1`，
因此首次运行前需在可联网环境下载官方权重：

```bash
mkdir -p weights
curl -fL https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt \
  -o weights/yolo26n.pt
```

桌面构建脚本会自动下载并校验该权重。更换检测模型后，已有 ReID 档案会因权重哈希
变化而被拒绝加载，这是预期行为；请重新执行 `reid-enroll`。

人物框默认显示目标角色标签。若要在 ReID 画面中额外显示车辆、椅子、背包等视觉障碍候选，先完成 CPU 性能测试，再将 `config.yaml` 中的 `vision.visual_object_detection_enabled` 改为 `true`；这些框只用于屏幕提示，不参与飞控避障。

先使用不发送飞控指令的模式验证：

```bash
.venv-reid/bin/python main.py --mode follow-dry-run
MPLCONFIGDIR=.matplotlib YOLO_CONFIG_DIR=.ultralytics \
  .venv-reid/bin/python tools/reid_offline_eval.py
```

### JointBDOE 人体朝向角验证

可选 JointBDOE 后端会在 ReID 接受目标身份后，匹配同一人物框并附加连续的
`0～360°`人体朝向角。普通自动跟随仍只显示该角度；在 150 cm 悬停选择界面
按 `s` 后，侧向跟随会用它选择更近的 `90°` 或 `270°` 并横移环绕到该侧面。

```bash
bash scripts/setup_jointbdoe.sh
.venv-reid/bin/python tools/jointbdoe_angle_eval.py \
  data/reid_target/现场注册/一组照片 --output-dir /tmp/jointbdoe-results
```

官方角度约定在当前实拍中表现为：背面约 `0°`、正面约 `180°`、两个侧面约
`90°/270°`。当前 `vision.jointbdoe_enabled` 已为 `true`；真机前仍须先验证视频连续性
和 CPU 速度。模型只提供人体检测置信度，没有独立的角度置信度。

ReID 只进行外观匹配，不识别真实姓名；俯视、遮挡、换衣、逆光和低分辨率都会降低可靠性。完成真实目标视频测试以前，不得启用真机 ReID 起飞跟随。

### 现场 ReID 演示

`reid-demo` 不会修改 `config.yaml`，只在本次运行内选择参考照片或本地档案。
推荐先将一组照片注册成只保存在本地的具名档案：

```bash
.venv-reid/bin/python main.py --mode reid-enroll \
  --profile person-a-current-outfit \
  --reference-dir data/reid_target/现场注册/一组照片
```

注册会验证每张照片恰好包含一个人物，分别保留每张照片的归一化 OSNet 模板，再写入
已被 Git 忽略的 `data/reid_profiles/<profile>/`。运行时综合最佳视角模板和模板中心
相似度；已锁定目标若要跳到画面中不连续的人物，还必须连续确认。档案同时记录
YOLO、OSNet 权重哈希和特征格式；权重或预处理版本变化时会拒绝加载并要求重新注册。
后续直接加载档案：

```bash
.venv-reid/bin/python main.py --mode reid-demo \
  --profile person-a-current-outfit
```

也可以不保存档案，直接为单次运行选择照片：

```bash
.venv-reid/bin/python main.py --mode reid-demo \
  --reference-image /path/to/front.jpg \
  --reference-image /path/to/side.jpg
```

也可以用电脑摄像头现场连拍：

```bash
.venv-reid/bin/python main.py --mode reid-demo \
  --capture-reference --reference-count 3
```

未传任何照片参数时，程序会询问输入照片路径或选择 `CAMERA`。拍摄时请让目标
人物全身入镜，按空格键拍摄不同角度。照片保存到已被 Git 忽略的
`data/reid_target/现场注册/`。

程序随后按以下顺序执行：

1. 连接无人机前加载 YOLO、OSNet 和已校验的人物档案（或现场参考照片）。
2. 连接并检查电量后，不要求地面画面先识别到目标；操作者在命令行输入 `y` 直接授权起飞。
3. 无人机先使用底部离地 TOF 闭环到达 150 cm 基础悬停高度；这一阶段完全忽略顶部前向 ToF 避障，不会因前方测距而横移或转向。超过 `base_hover_timeout_seconds`（当前 30 秒）仍未稳定则清零 RC 并安全降落。
4. 到达 150 cm 后持续悬停，可在窗口按 `m` 手动、`a` 原自动跟随、`s` 侧向跟随或 `f` 前向跟随；等待期间不运行 ReID、搜索或避障。侧向模式自动选择最近的 `90°/270°`，前向模式固定以 `180°` 为目标。
   进入自动飞行后使用 `1/2/3` 在普通、侧向、前向模式间安全切换；切换时先悬停并重新确认人物。完整键位见 [`按键.md`](按键.md)。
5. 选择手动后暂停 ReID 推理，使用上述键盘控制与 250 ms 看门狗；按 `m` 退出时悬停，连续 5 帧新鲜身份确认完成后才恢复自动运动。
6. 选择自动后，若从未发现目标，直接从当前高度开始固定三层扫描，首次搜索同样不受顶部前向 ToF 避障影响；完整搜索一轮仍未找到就降落。人物通过连续 5 帧身份确认并首次进入正常跟随后，才启用现有顶部 ToF 避障。此后跟随中途丢失，继续沿用原来的避障、过近后退、遮挡恢复和最后方向搜索逻辑。
7. 目标丢失时先悬停 1 秒；疑似过近时以 `-35` 后退 1.5 秒、停顿 0.5 秒，最多两次。之后只按最后水平方向以 `±25` 偏航，再按“当前高度 → 上方 20 cm → 下方 20 cm”的固定顺序分层搜索。每层以 `±30` 偏航，并根据飞控 yaw 累计完整旋转 360°；相邻层反向旋转。三层完整搜索一轮后返回起始高度，仍未找回则清零并降落，不再使用总时间限制。
   侧向和前向模式在进入上述搜索前额外要求持续丢失 `0.5` 秒；短暂漏检只悬停并保持当前模式显示，人物恢复后立即取消丢失计时。

真机运行会把每次实际调用 Tello `takeoff`/`land` SDK 的时间、PID、线程、
调用栈和当前跟随状态写入 `logs/flight_events/`。如果飞机已落地但对应文件中没有
`land_command` 的 `requested` 记录，说明本程序没有发送降落命令，应优先排查
飞控自动降落或另一个控制进程。

侧向跟随会先悬停采集 5 帧稳定朝向并锁定更近的一侧。第一次锁定后，人物只要偏离
画面中心就优先同方向横移且不转机头；重新居中后，等待朝向连续 5 帧稳定，再比较
`90°/270°` 哪个更近，并只在确有角度偏差时重新绕人。角度仍在变化时不会抢先绕行。
它按设计不运行障碍检测或绕障。人物持续丢失 0.5 秒后直接进入普通 ReID 跟随的
三层有界搜索；任何阶段重新确认人物都会停止搜索，重新采集朝向并恢复跟随。
绕行方向采用现场标定关系：顺时针增大 JointBDOE 角度，逆时针减小；
目标角较大时顺时针，目标角较小时逆时针。首次真机测试必须低速、宽阔、带保护罩。
跑道平移方向错误时调整 `tracking_lateral_direction_sign`；若机体横移坐标与当前
标定相反，可调整 `clockwise_lateral_direction` 的 `left/right`。
当前绕行连续 2 帧超出角度退出阈值后启动，角度比例为 `0.35`，横移指令限制在
`10～25`；跑道平移比例提高到 `70`，速度范围为 `8～25`。绕人时偏航以前馈
`0.80 × 绕行横移` 与人物中心误差反馈相加，偏航总指令限制为 `30`；跑道平移
阶段仍不偏航。
绕人过程没有时间上限，只要目标持续可靠可见且角度未恢复就会继续；需由操作者
使用 `m` 接管或 `q` 降落，目标丢失、电量、高度和视频异常保护仍会生效。

比赛前详细演练清单与流程见 [docs/01-功能与运行模式.md 第 4.5 节（reid-demo）](docs/01-功能与运行模式.md#45-reid-demo现场-reid-跟随)、[docs/03-视觉感知与目标跟随.md 第 5–6 节](docs/03-视觉感知与目标跟随.md#5-档案完整性与隐私) 与 [docs/05-安全机制与硬件边界.md 第 7 节](docs/05-安全机制与硬件边界.md#7-真机检查清单非穷举)。

人物档案包含可关联个人外观的特征数据。不要使用真实姓名作为档案名，不要提交或
上传 `data/reid_profiles/`，比赛结束后按本人意愿删除。换人或明显换衣后应建立新档案。

## 控制逻辑

`FollowController` 将目标检测结果转换为安全 RC 指令：

1. **水平**：优先 yaw 转向，不使用左右平移。误差在死区内不输出，死区外按比例计算偏航速度并施加最小速度约束。
2. **前后**：根据目标面积比例控制前进/后退。面积小于阈值则前进，大于阈值则后退，在中间范围悬停。
3. **上下**：目标在画面中偏上则上升，偏下则下降。
4. **对准减速**：存在偏航或上下修正时降低前后速度，避免姿态未稳定时冲撞。
5. **目标锁定**：目标在画面中心且面积适中稳定持续指定帧数后进入 LOCKED 状态；目标移出退出范围后解除锁定。
6. **目标丢失**：普通跟随按当前会话配置处理；启用搜索的 ReID 演示使用 `target_search` 有界搜索，期间只执行受限后退、偏航和分层升降。

所有 RC 指令经过 `SafetyManager.limit_rc_command` 限速。

## 障碍避让

当前完整中文流程、优先级、参数和异常保护见 [docs/04-搜索避障与运动仲裁.md](docs/04-搜索避障与运动仲裁.md)。

`follow`、`follow-dry-run`、`console` 和 `reid-demo` 启动时会询问本次是否开启避障，默认值来自 `config.yaml` 的 `obstacle.enabled`。避障只读取 RoboMaster TT 顶部扩展前向 ToF：有效距离小于等于 `front_tof_blocked_distance_cm`（当前 60 cm）时输出 `BLOCKED`，大于阈值或超量程时不增加风险。摄像头只用于目标识别与跟随，不参与障碍判断。ReID 首次确认目标后，跟随和后续丢失恢复才经过 `MotionArbiter`；起飞爬升与首次搜索明确绕过前向 ToF 避障。所有最终命令仍经过 `SafetyManager`。

手动模式不经过 `MotionArbiter`：它直接读取同一个后台 ToF 快照，只把危险的正向
速度压成 0，绝不会因此生成右移 1 m 或偏航绕障命令。

在线避障决策完全不依赖远程大模型，只使用确定性算法，避免网络延迟、超时和不可复现指令。每次观测和决策会异步写入 `logs/avoidance/*.jsonl`，格式可离线交给 LLM 或分析工具阅读，但不会被用于实时飞控。

侧向和前向跟随会异步写入 `logs/side_follow/*.jsonl`。日志逐控制帧记录模式、人物中心与朝向、目标角、转身稳定判定、位置优先、丢失搜索阶段、横移/偏航分量、电量、高度、机体 yaw 和最终 RC 指令；队列满或写盘失败时只丢弃日志，不影响飞控。

侧向或前向目标丢失后直接复用普通 ReID 跟随的 `target_search`：先短暂悬停，按需执行
有限后退和最后方向偏航，再依次在当前、上方 20 cm、下方 20 cm 三层各旋转
360°，最后返回起始高度并降落。重新识别必须连续确认 5 帧；确认后清空旧侧面，
重新采集 JointBDOE 角度并恢复原朝向模式。朝向搜索仅保留一个例外：不进入避障仲裁。

所有避障状态都禁止前进。普通 BLOCKED 和目标丢失遮挡路线都固定以速度 20 向右平移约 1 m（当前按约 5 秒航位推算），然后以速度 12 左转最多 90 度寻找目标。左转过程中只要 ReID 重新确认目标，就立即停止转向并交回跟随；若始终未识别到目标，转满后再交回跟随/搜索仲裁。Tello 没有横向里程计，因此 1 m 是速度—时间估算，会受电量、风和地效影响。顶部 ToF 不能保护后方、侧方、上方或下方；真机启用前必须人工确认这些方向净空，并随时准备按 `e` 急停。

若目标丢失前连续 3 个可靠 ReID 框都没有触碰画面左右 5% 边缘，则判定为非横向离场，进入受顶部 ToF 保护的直行恢复：以速度 25 向前，直到 ToF 检测到 120 cm 内任意实体，立即停止前进并执行右移约 1 m、左转 90 度的避障。ToF 只能判断实体距离，不能确认实体是不是人；读数超时、错误或过期时必须悬停而不是继续直行。

目标丢失时只有“视觉历史明确判断人物过近”的有限后退高于 ToF 避障：面积比例不低于过近阈值并满足面积增长、触边或此前已后退等条件时，先按配置后退并停顿；完成后重新读取最新 ToF，若仍有实体再执行右移约 1 m、左转 90 度。除该过近恢复外，普通搜索、偏航和分层动作都仍低于避障。

顶部模块通过后台线程读取 `EXT tof?`，控制循环只使用带新鲜度检查的缓存，不会被 SDK 查询阻塞。模块在起飞前无响应会禁止起飞；飞行中读数失效或过期会清零悬停。真机使用前必须先在 `follow-dry-run` 中验证。

## 安全

`SafetyManager` 负责单机安全检查：

- 起飞前电量低于 `min_battery_takeoff`（当前 20%）禁止起飞
- 飞行中电量达到或低于 `low_battery_land`（当前 5%）强制降落
- 起飞后的独立离地高度确认默认关闭；稳定完成后直接进入 150 cm 基础高度控制
- 飞行高度超过 `max_height_cm` 时立即清零并降落；低离地高度本身不再触发迫降
- 自动跟随会话稳定后直接使用底部 TOF 闭环到达 `base_hover_height_cm`（当前为 150 cm）；升高过程继续显示 ReID，但升高许可不依赖识别结果，并且不执行顶部前向 ToF 避障。爬升超过 `base_hover_timeout_seconds`（当前 30 秒），或底部测高、视频、识别连续异常、越过安全高度时会清零并降落
- 飞行控制高度统一采用底部 TOF 到正下方表面的离地距离，并使用最近 5 次有效读数的中位数；飞控原始 `h` 只用于状态诊断，不参与升降控制
- `limit_rc_command` 将每个通道限制在 `[-max_rc_speed, max_rc_speed]`
- ReID 身份歧义或刚丢失时先产生零控制输出；随后只允许配置限定的低速后退、原地偏航和上下分层搜索。外部停止请求始终优先清零

## 控制台

```bash
.venv/bin/python main.py --mode console
.venv/bin/python main.py --mode console --fake
```

自然语言控制台先使用本地规则，无法确定时可调用 OpenAI 兼容接口做白名单动作分类。所有结果只会映射为 `GET_STATUS`、`START_FOLLOW`、`STOP_TASK`、`EMERGENCY_STOP`、`EXIT` 或 `UNKNOWN`。

控制台只能通过 `ConsoleTools` 调用任务级工具，连接、状态、任务启动/停止和急停统一进入 `MissionManager` 强类型命令总线；底层输出仍必须经过 `SafetyManager`。普通跟随、ReID 和 Console 任务统一由 `MissionFactory` 构造 `FollowSession`。跟随任务在后台线程运行，停止、急停和退出命令会清零输出并等待降落清理完成。

启用 LLM 分类需在配置中设置 `llm_enabled: true`，并提供环境变量 `LLM_API_KEY`；未设置密钥时自动退回本地规则。

## Fake 模式

无真机时通过 `--fake` 启用 `FakeDroneAdapter`：

- 返回模拟电量和高度
- 生成会移动、缩放并间歇丢失的中性人物轮廓
- 目标会移动、缩放并间歇丢失
- 画面仍经过 ReID、控制器、安全层和模拟 RC 输出链路

模拟人物轮廓不保证能被 YOLO/ReID 接受，因此 Fake 模式主要验证目标丢失保护、
控制生命周期和模拟 RC 输出。完整识别验证仍需要真实参考图、模型权重和真实人物画面。

## 真机桌面端

仓库包含 Electron 桌面飞控台。单屏自适应布局（1040×760 起无滚动条）分为左栏与飞行区：
左栏为“人物档案 / 起飞准备 / 运行事件”三个 tab，起飞按钮常驻底部；飞行区从上到下依次是
态势条（飞行状态中文标签、暂停/遥测陈旧/安全告警）、模式行（普通/侧向/前向/手动/暂停）、
铺满的视频区（按原始比例放大、上下轻微裁切）、遥测与停止/急停。手动接管后控制板以
半透明 HUD 叠加在视频底部，可在 HUD 内关闭键盘控制。自动任务直接运行
`FollowSession/KernelSession`、目标搜索与前向 ToF 避障（安全保护状态在“起飞准备”
tab 确认）；视频流显示人物框和状态，两步建档（选照片→确认）后即可起飞。

GUI 与 CLI 共用 `MissionManager` 命令/事件模型和 `MissionFactory` 装配路径，但仍是两个
独立进程，不共享正在运行的会话或控制状态，也不能同时控制同一架无人机。Electron main
通过 `/api/v1` 强类型命令调用 sidecar；手动 RC 使用 1 秒独占租约、递增序号和时间戳，
松键或失焦会释放租约并悬停。sidecar 从 `config.yaml` 读取与 CLI 相同的电量、高度、
RC 和手动控制阈值；独立遥测线程在手动飞行中持续监视低电、超高和读取失效。端口和会话令牌由 Electron 主进程
持有，renderer 只能调用类型化 preload 接口。

任务入口严格依据 `/api/v1/capabilities` 返回的真实能力和本机模型资产启用，按钮使能跟随
运行时快照的 `allowedActions`。选择档案、连接真机并通过预检后，点击“起飞”并再次确认；
无人机上升至 150 cm 后，再选择普通、侧向、前向或手动接管。前端按钮和键盘（含 `P` 暂停、
`Q`/`E` 双击确认的停止/急停）通过相同的语义命令通道控制任务；顶栏提供日志目录与断开
真机入口。Electron 主进程以 5 秒心跳探测 sidecar，连续 3 次无响应即判定挂起并提示恢复。
地面预览接口保留给诊断用途，但不是自动任务的起飞前置条件。

优先交付 Linux x64 AppImage 与 macOS x64/arm64 DMG，Windows x64 NSIS 随后构建。
首轮安装包未签名；从 CI 或 Release 下载对应产物并核对同目录 `SHA256SUMS` 后安装。
运行前连接 `RMTT-XXXXXX` 无人机 Wi-Fi，不需要浏览器或外部网络服务。

从源码构建完整桌面包：

```bash
bash scripts/build_desktop_app.sh
```

脚本会创建隔离 Python 环境、固定安装 Node 依赖、下载并校验模型、构建 sidecar、生成
原生安装包和 `SHA256SUMS`。完整的前置工具、平台边界、模型规则与故障排查见
[`docs/桌面端开发构建指南.md`](docs/桌面端开发构建指南.md)。

架构、能力差异与已知安全边界见
[`docs/07-桌面端架构与安全边界.md`](docs/07-桌面端架构与安全边界.md)；安装、权限、
安全退出和真机验收流程见
[`docs/桌面端真机操作指导书.md`](docs/桌面端真机操作指导书.md)。

## 测试

```bash
.venv/bin/python -m pytest tests/
```
