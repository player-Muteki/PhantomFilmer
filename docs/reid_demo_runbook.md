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
     --profile person-a-current-outfit
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
5. 检查无人机周围、后方和上下方净空，在命令行输入 `y` 授权起飞；此时不要求摄像头已经识别到目标。
6. 起飞高度由连续多次 TOF 遥测确认；无人机先闭环升到 150 cm，再进入跟随程序。
7. 进入跟随后如果从未稳定识别到目标，只以 `yaw=+20` 沿固定方向偏航一整周；连续可信识别 10 帧后才进入跟随。一周结束仍未锁定或达到 35 秒总安全上限则降落，不执行横移、前后或分层升降。
8. 跟随中途丢失时，先悬停确认遮挡。全局 `BLOCKED`，或面积至少 2% 且与目标预测框连续关联的 `CAUTION`，在侧方净空评分合格时只做一次横移脉冲：使用 `lr=±25` 横移约 4.0 秒（名义约 1 m），随后保持航向识别；仍未找到则以 `yaw=±20` 朝横移反方向局部偏航 30°，停留识别后回正。找到候选人后连续 5 帧确认身份才恢复跟随。若左右侧都不安全，则悬停 3 秒等待后降落，不执行全局旋转。
9. 跟随中保持急停人员在电脑旁：`q` 停止并降落，`e` 急停并降落。

飞行控制使用无人机底部 TOF 到正下方表面的离地距离，并对最近 5 次有效读数取中位数。飞控原始 `h` 仅在 `status` 中作为诊断对照；飞过桌面、台阶等物体时，TOF 的参照表面会随之改变。

## 起飞门禁

以下任一情况不会起飞：

- ReID 依赖、YOLO 或 OSNet 权重缺失。
- 人物档案不存在、损坏、与当前模型不兼容，或现场参考照片无效。
- 无人机电量低于安全阈值。
- 操作者没有在命令行输入 `y` 确认起飞。

起飞后，视频流、TOF 或检测器连续异常以及高度超过上限，仍会触发清零和安全降落；低高度不触发自动迫降，是否已经看到目标也不再作为起飞门禁。

## 现场调参

- `--lock-frames`、`reid_lock_stable_frames`、`reid_lock_timeout_seconds` 和 `reid_confirmation_timeout_seconds` 仅为旧版地面锁定流程保留；当前 `reid-demo` 起飞门禁不再使用它们。
- 调试画面会画出全部 YOLO 人物候选框，并显示每人的 ReID 相似度、最高分和当前阈值：`YOLO people=0` 表示行人检测失败；候选数大于 0 但显示 `BELOW THRESHOLD` 表示检测到了人但外观匹配不足；`AMBIGUOUS` 表示多人分数过于接近。
- ReID 阈值由 `vision.reid_similarity_threshold` 控制；不应为了强行锁定而在现场大幅降低。
- 目标换衣后必须重新录入。此 ReID 匹配的是外观，不是人的法定身份。

人物档案位于 `data/reid_profiles/`，照片位于 `data/reid_target/现场注册/`；两者均
只应保存在本地且不得上传。演示结束后按现场隐私要求删除。
