# ReID 模型与测试材料

Git 仓库只保存代码、配置示例和测试报告，不保存模型权重、人物照片或完整数据集。
以下路径已被 `.gitignore` 排除：

```text
weights/
data/reid_target/
data/reid_test/
data/datasets/
```

## 必需权重

```text
weights/yolov8n.pt
weights/osnet_x0_25_msmt17.pth
```

已验证文件的 SHA-256：

```text
31e20dde3def09e2cf938c7be6fe23d9150bbbe503982af13345706515f2ef95  yolov8n.pt
cf55163d78fc44c62c82f85ab62d39f10438679b5abe8c698ae08cfa84aa6e18  osnet_x0_25_msmt17.pth
```

官方来源：

- YOLOv8：https://huggingface.co/Ultralytics/YOLOv8
- OSNet：https://huggingface.co/kaiyangzhou/osnet
- Torchreid Model Zoo：https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO

不要使用来源不明或哈希不一致的 pickle/PyTorch 权重。

## 参考人物照片

在取得本人同意后，准备至少 5–10 张不同角度、距离和姿态的清晰全身照片，放入
`data/reid_target/`。实际衣着应与验证视频一致。不得把公开数据集中的人物当作
真实跟随目标。

## PRID 2011 离线测试集

完整清洗版可从 TU Graz 官方页面获取：

https://www.tugraz.at/institute/icg/research/team-bischof/learning-recognition-surveillance/downloads/prid11/

解压后的预期路径：

```text
data/datasets/prid_2011/multi_shot/cam_a/
data/datasets/prid_2011/multi_shot/cam_b/
```

PRID 是裁剪后的人物图，只能验证 ReID 特征匹配，不能替代完整场景中的 YOLO
行人检测测试。使用数据时应遵守官方条款并引用原论文。

## 环境与复现

```bash
bash scripts/setup_reid_env.sh python3
MPLCONFIGDIR=.matplotlib YOLO_CONFIG_DIR=.ultralytics \
  .venv-reid/bin/python tools/reid_offline_eval.py
```

配置示例见 `config.reid.offline-snippet.yaml`，已验证结果见 `docs/reid_test/`。
示例阈值只用于离线标定，不得直接授权真机飞行。
