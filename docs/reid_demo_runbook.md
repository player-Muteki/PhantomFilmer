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
6. 核对黄色/绿色框确实是目标人员，再在终端输入大写 `YES`。
7. 跟随中保持急停人员在电脑旁：`q` 停止并降落，`e` 急停并降落。

## 起飞门禁

以下任一情况不会起飞：

- ReID 依赖、YOLO 或 OSNet 权重缺失。
- 人物档案不存在、损坏、与当前模型不兼容，或现场参考照片无效。
- 无人机电量低于安全阈值。
- 视频流连续读取失败。
- 目标未在配置时间内连续锁定，或画面中身份模糊。
- 现场人员没有输入精确的大写 `YES`。

## 现场调参

- 默认起飞前连续锁定帧数为 10，可用 `--lock-frames 15` 临时提高，不会修改配置文件。
- 锁定超时由 `config.yaml` 的 `reid_lock_timeout_seconds` 控制。
- ReID 阈值由 `vision.reid_similarity_threshold` 控制；不应为了强行锁定而在现场大幅降低。
- 目标换衣后必须重新录入。此 ReID 匹配的是外观，不是人的法定身份。

人物档案位于 `data/reid_profiles/`，照片位于 `data/reid_target/现场注册/`；两者均
只应保存在本地且不得上传。演示结束后按现场隐私要求删除。
