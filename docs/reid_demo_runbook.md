# ReID 现场演示手册

## 演示前准备

1. 连接 RoboMaster TT / Tello Wi-Fi，确认场地净空、安装桨叶保护罩。
2. 确认 `.venv-reid/` 环境可用。
3. 确认两个权重存在：
   - `weights/yolov8n.pt`
   - `weights/osnet_x0_25_msmt17.pth`
4. 先用比赛场地的灯光、人员服装和背景运行一次 `follow-dry-run`。
5. 保留 ArUco 方案作为 ReID 现场效果不稳定时的降级演示。

## 推荐现场流程

1. 征得目标人员同意，拍摄 3–5 张当前服装的清晰全身照。
2. 比赛前将照片注册为本地人物档案（此命令不连接无人机）：

   ```bash
   .venv-reid/bin/python main.py --mode reid-enroll \
     --profile person-a-current-outfit \
     --reference-dir /path/to/reference-photo-directory
   ```

3. 现场通过档案启动：

   ```bash
   .venv-reid/bin/python main.py --mode reid-demo \
     --profile person-a-current-outfit \
     --lock-frames 15
   ```

   也可以跳过持久档案，直接使用照片文件启动：

   ```bash
   .venv-reid/bin/python main.py --mode reid-demo \
     --reference-image /path/to/front.jpg \
     --reference-image /path/to/left.jpg \
     --reference-image /path/to/right.jpg
   ```

   或用电脑摄像头启动：

   ```bash
   .venv-reid/bin/python main.py --mode reid-demo \
     --capture-reference --reference-count 3
   ```

4. 根据提示选择本次是否开启避障。
5. 让目标人员站在无人机镜头正前方，保持全身可见，等待 `GROUND LOCK 10/10`。
6. 核对黄色/绿色框确实是目标人员，在同一个视频窗口按 `Y` 确认起飞；按 `Q` 取消。
7. 起飞高度由连续多次遥测确认；升到 150 cm 期间 ReID 会持续显示，但即使暂时未识别到目标也会继续定高，且没有总时间限制。
8. 跟随中保持急停人员在电脑旁：`q` 停止并降落，`e` 急停并降落。

飞行控制使用无人机底部 TOF 到正下方表面的离地距离，并对最近 5 次有效读数取中位数。飞控原始 `h` 仅在 `status` 中作为诊断对照；飞过桌面、台阶等物体时，TOF 的参照表面会随之改变。

## 起飞门禁

以下任一情况不会起飞：

- ReID 依赖、YOLO 或 OSNet 权重缺失。
- 人物档案不存在、损坏、与当前模型不兼容，或现场参考照片无效。
- 无人机电量低于安全阈值。
- 视频流连续读取失败。
- 目标未在配置时间内连续锁定，或画面中身份模糊。
- 现场人员没有在视频窗口按 `Y` 确认，按了 `Q`，或确认时目标已经离开/身份变得模糊。

## 现场调参

- 默认起飞前连续锁定帧数为 10，可用 `--lock-frames 15` 临时提高，不会修改配置文件。
- 锁定超时由 `config.yaml` 的 `reid_lock_timeout_seconds` 控制。
- 锁定后的人工确认超时由 `reid_confirmation_timeout_seconds` 控制；等待期间窗口和 ReID 检测会持续刷新。
- 调试画面会画出全部 YOLO 人物候选框，并显示每人的 ReID 相似度、最高分和当前阈值：`YOLO people=0` 表示行人检测失败；候选数大于 0 但显示 `BELOW THRESHOLD` 表示检测到了人但外观匹配不足；`AMBIGUOUS` 表示多人分数过于接近。
- ReID 阈值由 `vision.reid_similarity_threshold` 控制；不应为了强行锁定而在现场大幅降低。
- 目标换衣后必须重新录入。此 ReID 匹配的是外观，不是人的法定身份。

人物档案位于 `data/reid_profiles/`，照片位于 `data/reid_target/现场注册/`；两者均
只应保存在本地且不得上传。演示结束后按现场隐私要求删除。
