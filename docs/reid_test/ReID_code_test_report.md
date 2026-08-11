# PhantomFilmer ReID 测试报告

- 测试日期：2026-08-11
- 输入包：`PhantomFilmer-ReID.zip`
- 测试方式：解压到隔离工作目录，仅执行无真机、无飞控指令的代码测试与启动前检查
- 测试 Python：3.12.13（Codex 隔离运行时）

## 结论

ReID 的核心匹配策略、检测器接入与起飞前安全检查通过现有自动化测试，可以进入“补齐模型和样本后做离线图像/视频实测”的阶段。目前不能判定真实人物识别效果合格，因为压缩包没有提供 ReID 权重、参考人物照片或离线测试视频，运行环境也未安装模型推理依赖。

## 测试结果

### ReID 专项与集成测试

- 结果：20/20 通过
- 覆盖内容：
  - 选择与参考特征最相似的人物
  - 低于相似度阈值时拒绝目标
  - 前两名过于接近时判定为歧义并拒绝
  - 短时目标丢失预测
  - `reset()` 清除历史目标
  - 非法图像安全返回未找到
  - 参考图和阈值配置解析
  - `person_reid` 检测器工厂接入
  - 新跟随会话重置检测器状态
  - 模型准备失败发生在起飞之前

执行命令：

```bash
python3 -m unittest -v \
  tests.test_person_reid_detector \
  tests.test_detector_factory \
  tests.test_follow_session_detector_reset
```

### 全项目回归

- 总数：169
- 通过：138
- 跳过：30（运行环境未安装 OpenCV，主要影响 ArUco、障碍物和 Fake 摄像头视觉测试）
- 失败：1（控制台 LLM 本地 HTTP 分类测试，与 ReID 路径无关）

## 实际模型启动检查

`PersonReIDDetector.prepare()` 被正确阻止，首先报告缺少 `requirements-reid.txt` 中的模型依赖。进一步检查发现以下实测资产也不在压缩包中：

- `data/reid_target/front.jpg`
- `data/reid_target/side.jpg`
- `weights/osnet_x0_25_msmt17.pth`
- 用于评估的离线人物图片或视频

此外，压缩包中的 `config.yaml` 当前仍为：

```yaml
vision:
  detector_type: aruco
```

因此即使直接运行程序，也不会启用人物 ReID；实测前需改为 `person_reid`。

## 风险判断

- 已验证的是代码逻辑与安全接入，不是模型精度。
- 目前没有数据可以计算 Rank-1、mAP、误匹配率、漏检率或实时 FPS。
- 无人机俯视、遮挡、逆光、换衣和低分辨率可能显著降低 OSNet 外观匹配可靠性。
- 在离线图片/视频和 `follow-dry-run` 验证完成前，不应进行真机起飞跟随。

## 完成真实 ReID 测试所需材料

1. 目标人物 3 张以上不同方向、不同距离的清晰全身参考图。
2. 与 `osnet_x0_25` 匹配的预训练权重文件。
3. 至少一段包含目标人物、相似衣着干扰人物、遮挡和进出画面的测试视频。
4. 安装基础依赖和 `requirements-reid.txt` 依赖的 Python 3.10/3.11 环境。

补齐这些材料后，应先运行 `follow-dry-run`，记录每帧相似度、误匹配、漏检、歧义拒绝和 FPS，再决定是否允许进入真机地面联调。
