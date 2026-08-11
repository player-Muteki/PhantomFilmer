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
2. 使用照片文件启动：

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

3. 根据提示选择本次是否开启避障。
4. 让目标人员站在无人机镜头正前方，保持全身可见，等待 `GROUND LOCK 10/10`。
5. 核对黄色/绿色框确实是目标人员，再在终端输入大写 `YES`。
6. 跟随中保持急停人员在电脑旁：`q` 停止并降落，`e` 急停并降落。

## 起飞门禁

以下任一情况不会起飞：

- ReID 依赖、YOLO 或 OSNet 权重缺失。
- 参考照片不存在、无法读取，或不是恰好检测到一个完整人物。
- 无人机电量低于安全阈值。
- 视频流连续读取失败。
- 目标未在配置时间内连续锁定，或画面中身份模糊。
- 现场人员没有输入精确的大写 `YES`。

## 现场调参

- 默认起飞前连续锁定帧数为 10，可用 `--lock-frames 15` 临时提高，不会修改配置文件。
- 锁定超时由 `config.yaml` 的 `reid_lock_timeout_seconds` 控制。
- ReID 阈值由 `vision.reid_similarity_threshold` 控制；不应为了强行锁定而在现场大幅降低。
- 目标换衣后必须重新录入。此 ReID 匹配的是外观，不是人的法定身份。

演示结束后，按现场隐私要求删除 `data/reid_target/现场注册/` 中的照片。
