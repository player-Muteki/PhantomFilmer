# DroneUmbrella ArUco 视觉检测模块

## 概述

`vision/aruco_detect.py` 提供了 `ArucoTargetDetector` 类，用于检测指定 ID 的 ArUco 标记。
其 `detect(frame)` 和 `draw_debug(frame, result)` 接口与现有的 `TargetDetector`（红色目标检测）完全兼容，
因此 `FollowController`、`FollowSession` 和控制链路可以无缝切换检测方式，无需修改代码。

## 默认配置

- **字典**: `DICT_4X4_50`
- **目标 ID**: `23`
- **平滑系数**: `0.30`
- **短暂丢失容忍帧数**: `3`
- **最小标记面积**: `300.0 px²`

## 检测流程

1. 校验输入帧（`None`、非 `ndarray`、空图 → 返回标准失败字典）。
2. 转灰度图。
3. 用 `cv2.aruco.ArucoDetector` 检测所有 markers。
4. 只保留 `marker_id == 23` 的标记。
5. 计算四边形中心 `(cx, cy)`，用 `cv2.contourArea()` 计算像素面积。
6. 从四角点极值计算 `bbox=(x, y, w, h)`。
7. 对 `cx`、`cy`、`area` 做指数平滑（首次直接初始化）。
8. 短暂丢失容忍：连续 N 帧检测失败，但仍在容忍帧数内时，返回最后一次有效值，`found=True`。
9. 超过容忍帧数后返回标准失败字典。

## 接口

### `detect(frame) -> dict`

| 字段 | 类型 | 说明 |
|------|------|------|
| `found` | bool | 是否检测到目标 ID |
| `center` | `(int,int) \| None` | 平滑后的中心像素坐标 |
| `target_center_x` | `int \| None` | center[0] |
| `target_center_y` | `int \| None` | center[1] |
| `area` | float | 四边形像素面积（经平滑） |
| `bbox` | `(int,int,int,int) \| None` | `(x, y, w, h)` |
| `marker_id` | `int \| None` | 检测到的 marker ID |
| `corners` | `list \| None` | 四个角点坐标 |
| `detector_type` | str | 固定为 `"aruco"` |

### `draw_debug(frame, result) -> frame`

在帧上绘制四边形轮廓、中心点、十字准星、ID 标签、面积和状态。

### `reset() -> None`

重置平滑和丢失状态。

## `detector_factory.py`

`vision/detector_factory.py` 提供 `create_detector(config)` 工厂函数。
根据 `config.yaml` 的 `vision.detector_type` 字段（`"red"` 或 `"aruco"`）创建对应的检测器。

```python
from vision.detector_factory import create_detector

detector = create_detector(config)
result = detector.detect(frame)
debug = detector.draw_debug(frame, result)
```

## 配置文件

```yaml
vision:
  detector_type: red           # "red" 或 "aruco"
  aruco_dictionary: DICT_4X4_50
  target_marker_id: 23
  smoothing_alpha: 0.30
  temporary_lost_frames: 3
  min_marker_area: 300.0
```

`detector_type: red` 时行为与之前一致，`detector_type: aruco` 时切换到 ArUco 检测。
配置缺失时默认使用红色检测，不影响旧模式。

## ArUco Marker 生成与打印

运行以下命令生成 ID 23 的可打印 PNG：

```bash
python3 tools/generate_aruco_marker.py
```

生成位置：`docs/aruco_marker_23.png`，1000×1000 px，四周 100 px 白色静区。

打印参数：
- **尺寸**: 15–20 cm 正方形
- **要求**: 禁止拉伸变形，保留黑色边框，四周至少 2 cm 白色静区
- **建议**: 贴在硬纸板或 KT 板上

## 测试

```bash
python3 -m unittest tests.test_aruco_detector -v
```

覆盖：
- ID 23 检测成功
- 无 marker / 其他 ID 不误检
- 多 marker 中只选 ID 23
- 面积过滤
- 异常输入保护
- 字段兼容性
- 平滑行为
- 短暂丢失与恢复
