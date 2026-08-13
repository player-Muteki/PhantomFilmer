# ReID 遮挡恢复真机专项测试

## 运行代码

已有本地人物档案时，在项目根目录执行：

```bash
.venv-reid/bin/python main.py --mode reid-recovery-test \
  --profile person-a-current-outfit
```

直接使用照片时执行：

```bash
.venv-reid/bin/python main.py --mode reid-recovery-test \
  --reference-image data/reid_target/front.jpg \
  --reference-image data/reid_target/side.jpg
```

程序首先要求输入精确口令 `RECOVERY-TEST`，完成模型和真机检查后还会要求输入 `y` 才起飞。专项模式强制开启避障、每帧重新检测障碍并写入 `logs/avoidance/*.jsonl`；它不接受 `--fake`。

## 测试顺序和期望输出

1. 目标人物先不要入镜。无人机到达基础高度后，应始终输出 `lr=0, fb=0, ud=0, yaw=+20`，沿同一方向完成 360°。不得左右来回扫描；一周仍未发现目标应清零降落。
2. 第二次起飞时让人物在一周扫描过程中入镜。无人机应立即悬停，以 `INITIAL_LOCK_VERIFY` 连续验证 10 帧，然后才进入 `FOLLOWING`。
3. 完成锁定后，用轻质泡沫板覆盖人物最后位置，左右至少一侧保持数米净空。系统应先进入 `OCCLUSION_CHECK` 悬停；`BLOCKED` 障碍，或面积至少 2% 的 `CAUTION` 障碍，与预测目标框连续关联 3 帧后，才进入 `OCCLUSION_BYPASS`。现场日志表明泡沫板通常会被判为 `CAUTION`，这是允许的预期表现。若左右两侧都不安全，系统进入 `OCCLUSION_NO_SAFE_ROUTE` 悬停，3 秒内空间恢复就横移，否则降落，不再误入 360° 搜索。
4. 绕行横移只能出现 `lr=±25, fb=0, ud=0, yaw=0`，持续约 4.0 秒（名义约 1 m），且每次遮挡事件最多执行一次，随后 `OCCLUSION_SETTLE` 悬停约 0.55 秒运行 ReID。RC 速度不是位移闭环，需用地面标记实测并微调持续时间。
5. 如果人物仍未露出，应进入 `OCCLUSION_LOCAL_SCAN_OUT`：右横移后输出 `yaw=-20`，左横移后输出 `yaw=+20`。飞控 yaw 累计到 30°后悬停识别，再进入 `OCCLUSION_LOCAL_SCAN_RETURN` 回到横移前航向。局部扫描期间 `lr/fb/ud` 必须为 0。
6. 人物重新露出后，应在 `REACQUIRE_VERIFY` 状态悬停连续验证 5 帧，之后才恢复跟随。一帧匹配不得直接追踪。
7. 横移、悬停、30°局部扫描和回正仍未找到人物时，直接转入原有完整搜索，不得再次累计1米横移。

测试时必须安装桨叶保护罩，横移方向至少保留 2.5 m 净空，后方和上下方保持净空，并安排一人专门操作急停。`e` 为急停降落，`q` 为正常停止并降落。出现横移和偏航同时非零、一次横移明显超过 1 m 仍不停、首次锁定前出现平移、局部扫描明显超过 30°仍不停止，均应立即按 `e`。
