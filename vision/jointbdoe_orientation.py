"""Optional JointBDOE body-orientation inference.

JointBDOE is loaded lazily from ``third_party/JointBDOE`` because its released
checkpoint contains pickled classes from the original YOLOv5-based project.
The estimator runs on the full frame, then matches its person prediction to the
person box already accepted by ReID.
"""

from __future__ import annotations

import importlib
import math
import sys
import types
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from typing import Any, Deque, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class JointBDOEPrediction:
    """One person prediction produced by JointBDOE."""

    bbox_xyxy: Tuple[float, float, float, float]
    angle_deg: float
    detection_confidence: float


class JointBDOEOrientationEstimator:
    """Estimate the accepted ReID target's body orientation in degrees."""

    def __init__(
        self,
        source_path: str = "third_party/JointBDOE",
        model_path: str = "weights/jointbdoe_s.pt",
        device: str = "cpu",
        image_size: int = 640,
        confidence_threshold: float = 0.30,
        iou_threshold: float = 0.45,
        match_iou_threshold: float = 0.20,
        smoothing_window: int = 5,
    ) -> None:
        self.source_path = _resolve_project_path(source_path)
        self.model_path = _resolve_project_path(model_path)
        self.device_name = str(device).strip() or "cpu"
        self.image_size = max(128, int(image_size))
        self.confidence_threshold = _clamp(confidence_threshold, 0.01, 1.0, 0.30)
        self.iou_threshold = _clamp(iou_threshold, 0.01, 1.0, 0.45)
        self.match_iou_threshold = _clamp(match_iou_threshold, 0.0, 1.0, 0.20)
        self._angles: Deque[float] = deque(maxlen=max(1, int(smoothing_window)))
        self._prepared = False
        self._model: Any = None
        self._torch: Any = None
        self._torchvision: Any = None
        self._cv2: Any = None
        self._device: Any = None
        self._stride = 64

    @classmethod
    def from_config(cls, config: Mapping[str, Any]) -> "JointBDOEOrientationEstimator":
        vision = config.get("vision", {})
        values = vision if isinstance(vision, Mapping) else config
        return cls(
            source_path=str(values.get("jointbdoe_source_path", "third_party/JointBDOE")),
            model_path=str(values.get("jointbdoe_model_path", "weights/jointbdoe_s.pt")),
            device=str(values.get("jointbdoe_device", values.get("reid_device", "cpu"))),
            image_size=_safe_int(values.get("jointbdoe_image_size"), 640),
            confidence_threshold=_safe_float(
                values.get("jointbdoe_confidence_threshold"), 0.30
            ),
            iou_threshold=_safe_float(values.get("jointbdoe_iou_threshold"), 0.45),
            match_iou_threshold=_safe_float(
                values.get("jointbdoe_match_iou_threshold"), 0.20
            ),
            smoothing_window=_safe_int(values.get("jointbdoe_smoothing_window"), 5),
        )

    def prepare(self) -> None:
        """Load the official source classes and released checkpoint."""
        if self._prepared:
            return
        if not self.source_path.is_dir():
            raise RuntimeError(
                f"JointBDOE 源码目录不存在：{self.source_path}。"
                "请运行 scripts/setup_jointbdoe.sh。"
            )
        if not (self.source_path / "models" / "yolo.py").is_file():
            raise RuntimeError(f"JointBDOE 模型源码不完整：{self.source_path}")
        if not self.model_path.is_file():
            raise RuntimeError(
                f"JointBDOE 权重不存在：{self.model_path}。"
                "请运行 scripts/setup_jointbdoe.sh。"
            )

        try:
            import cv2
            import torch
            import torchvision
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "缺少 JointBDOE 推理依赖，请安装 requirements-reid.txt。"
            ) from exc

        _install_pkg_resources_compatibility()
        _activate_official_source(self.source_path)
        # Import these modules before torch.load so pickle can resolve the
        # original ``models.*`` class names stored in the released checkpoint.
        importlib.import_module("models.common")
        importlib.import_module("models.yolo")

        device = _select_device(torch, self.device_name)
        load_kwargs: Dict[str, Any] = {"map_location": device}
        # PyTorch 2.6+ defaults to weights_only=True, but this trusted official
        # checkpoint contains a serialized model object from JointBDOE.
        if "weights_only" in _callable_parameters(torch.load):
            load_kwargs["weights_only"] = False
        checkpoint = torch.load(str(self.model_path), **load_kwargs)
        if not isinstance(checkpoint, Mapping):
            raise RuntimeError("JointBDOE 权重格式无效：缺少模型字典。")
        model = checkpoint.get("ema") or checkpoint.get("model")
        if model is None:
            raise RuntimeError("JointBDOE 权重格式无效：缺少 model/ema。")
        model = model.float().to(device).eval()

        stride_value = getattr(model, "stride", None)
        if stride_value is not None:
            self._stride = max(1, int(stride_value.max()))
        self.image_size = _round_up(self.image_size, self._stride)
        self._torch = torch
        self._torchvision = torchvision
        self._cv2 = cv2
        self._device = device
        self._model = model
        self._prepared = True

    def detect_people(self, frame: Any) -> List[JointBDOEPrediction]:
        """Return every JointBDOE person box and its continuous angle."""
        if not _valid_frame(frame):
            return []
        self.prepare()
        image, ratio, padding = _letterbox(
            frame, self.image_size, self._stride, self._cv2
        )
        image = image[:, :, ::-1].transpose(2, 0, 1)
        image = np.ascontiguousarray(image)
        tensor = self._torch.from_numpy(image).to(self._device).float() / 255.0
        tensor = tensor.unsqueeze(0)

        with self._torch.inference_mode():
            output = self._model(tensor)
        raw = output[0] if isinstance(output, (tuple, list)) else output
        if raw.ndim != 3 or raw.shape[0] != 1 or raw.shape[2] < 7:
            raise RuntimeError(f"JointBDOE 返回了异常张量形状：{tuple(raw.shape)}")
        rows = _jointbdoe_nms(
            raw[0],
            self.confidence_threshold,
            self.iou_threshold,
            self._torch,
            self._torchvision,
        )
        if rows.shape[0] == 0:
            return []

        boxes = rows[:, :4].clone()
        _scale_boxes_to_original(boxes, ratio, padding, frame.shape[:2])
        values = rows.detach().cpu().numpy()
        boxes_np = boxes.detach().cpu().numpy()
        return [
            JointBDOEPrediction(
                bbox_xyxy=tuple(float(value) for value in box),
                detection_confidence=float(row[4]),
                angle_deg=float((row[6] * 360.0) % 360.0),
            )
            for box, row in zip(boxes_np, values)
        ]

    def estimate(
        self, frame: Any, target_bbox_xywh: Sequence[float]
    ) -> Optional[Dict[str, Any]]:
        """Match JointBDOE output to a ReID target and return stable fields."""
        if len(target_bbox_xywh) != 4:
            return None
        started = perf_counter()
        predictions = self.detect_people(frame)
        target = _xywh_to_xyxy(target_bbox_xywh)
        matched = _best_iou_match(target, predictions)
        if matched is None or matched[1] < self.match_iou_threshold:
            return None
        prediction, match_iou = matched
        raw_angle = prediction.angle_deg % 360.0
        self._angles.append(raw_angle)
        angle = _circular_mean_deg(self._angles)
        return {
            "body_orientation_model": "jointbdoe",
            "body_orientation_angle": angle,
            "body_orientation_raw_angle": raw_angle,
            "body_orientation_detection_confidence": prediction.detection_confidence,
            "body_orientation_match_iou": match_iou,
            "body_orientation_latency_ms": (perf_counter() - started) * 1000.0,
        }

    def reset(self) -> None:
        self._angles.clear()


def _jointbdoe_nms(
    raw: Any,
    confidence_threshold: float,
    iou_threshold: float,
    torch: Any,
    torchvision: Any,
) -> Any:
    """Decode the released one-class, one-angle JointBDOE head."""
    if raw.shape[0] == 0:
        return torch.zeros((0, 7), device=raw.device)
    candidates = raw[raw[:, 4] > confidence_threshold]
    if candidates.shape[0] == 0:
        return torch.zeros((0, 7), device=raw.device)
    scores = candidates[:, 4] * candidates[:, 5]
    keep_confident = scores > confidence_threshold
    candidates = candidates[keep_confident]
    scores = scores[keep_confident]
    if candidates.shape[0] == 0:
        return torch.zeros((0, 7), device=raw.device)

    boxes = _xywh_tensor_to_xyxy(candidates[:, :4], torch)
    keep = torchvision.ops.nms(boxes, scores, iou_threshold)
    classes = torch.zeros((keep.shape[0], 1), device=raw.device)
    return torch.cat(
        (
            boxes[keep],
            scores[keep, None],
            classes,
            candidates[keep, -1:],
        ),
        dim=1,
    )


def _xywh_tensor_to_xyxy(boxes: Any, torch: Any) -> Any:
    result = torch.empty_like(boxes)
    result[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    result[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    result[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    result[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    return result


def _letterbox(
    image: Any, image_size: int, stride: int, cv2: Any
) -> Tuple[Any, float, Tuple[float, float]]:
    height, width = image.shape[:2]
    ratio = min(image_size / height, image_size / width)
    resized_width = int(round(width * ratio))
    resized_height = int(round(height * ratio))
    pad_width = (image_size - resized_width) % stride
    pad_height = (image_size - resized_height) % stride
    half_width = pad_width / 2.0
    half_height = pad_height / 2.0
    if (resized_width, resized_height) != (width, height):
        image = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    top = int(round(half_height - 0.1))
    bottom = int(round(half_height + 0.1))
    left = int(round(half_width - 0.1))
    right = int(round(half_width + 0.1))
    image = cv2.copyMakeBorder(
        image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    return image, ratio, (half_width, half_height)


def _scale_boxes_to_original(
    boxes: Any,
    ratio: float,
    padding: Tuple[float, float],
    original_shape: Sequence[int],
) -> None:
    boxes[:, [0, 2]] -= padding[0]
    boxes[:, [1, 3]] -= padding[1]
    boxes[:, :4] /= ratio
    height, width = int(original_shape[0]), int(original_shape[1])
    boxes[:, [0, 2]].clamp_(0, width)
    boxes[:, [1, 3]].clamp_(0, height)


def _best_iou_match(
    target_xyxy: Sequence[float], predictions: Sequence[JointBDOEPrediction]
) -> Optional[Tuple[JointBDOEPrediction, float]]:
    if not predictions:
        return None
    scored = [(_bbox_iou(target_xyxy, item.bbox_xyxy), item) for item in predictions]
    match_iou, prediction = max(scored, key=lambda value: value[0])
    return prediction, float(match_iou)


def _bbox_iou(first: Sequence[float], second: Sequence[float]) -> float:
    x1 = max(float(first[0]), float(second[0]))
    y1 = max(float(first[1]), float(second[1]))
    x2 = min(float(first[2]), float(second[2]))
    y2 = min(float(first[3]), float(second[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, float(first[2]) - float(first[0])) * max(
        0.0, float(first[3]) - float(first[1])
    )
    second_area = max(0.0, float(second[2]) - float(second[0])) * max(
        0.0, float(second[3]) - float(second[1])
    )
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _xywh_to_xyxy(box: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, width, height = (float(value) for value in box)
    return x, y, x + width, y + height


def _circular_mean_deg(angles: Sequence[float]) -> float:
    if not angles:
        raise ValueError("角度序列不能为空。")
    radians = np.deg2rad(np.asarray(angles, dtype=np.float64))
    sine = float(np.sin(radians).mean())
    cosine = float(np.cos(radians).mean())
    if abs(sine) < 1e-12 and abs(cosine) < 1e-12:
        return float(angles[-1]) % 360.0
    return float(math.degrees(math.atan2(sine, cosine)) % 360.0)


def _activate_official_source(source_path: Path) -> None:
    source = str(source_path)
    for package_name in ("models", "utils"):
        existing = sys.modules.get(package_name)
        existing_file = getattr(existing, "__file__", "") if existing else ""
        if existing_file and not str(existing_file).startswith(source):
            raise RuntimeError(
                f"JointBDOE 无法加载：模块名 {package_name} 已被其他库占用。"
            )
    if source not in sys.path:
        sys.path.insert(0, source)


def _install_pkg_resources_compatibility() -> None:
    try:
        import pkg_resources  # noqa: F401
        return
    except ModuleNotFoundError:
        pass
    try:
        from packaging.requirements import Requirement
        from packaging.version import parse
    except ModuleNotFoundError as exc:
        raise RuntimeError("缺少 packaging，无法加载 JointBDOE 旧版源码。") from exc

    module = types.ModuleType("pkg_resources")
    module.parse_version = parse
    module.parse_requirements = lambda lines: (
        Requirement(line.strip())
        for line in lines
        if line.strip() and not line.lstrip().startswith("#")
    )
    module.require = lambda _requirement: None
    sys.modules["pkg_resources"] = module


def _select_device(torch: Any, requested: str) -> Any:
    normalized = requested.strip().lower()
    if normalized.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("配置要求 CUDA，但当前环境没有可用 CUDA。")
    if normalized == "mps" and not torch.backends.mps.is_available():
        raise RuntimeError("配置要求 MPS，但当前环境没有可用 MPS。")
    return torch.device(normalized)


def _callable_parameters(callable_value: Any) -> Dict[str, Any]:
    import inspect

    try:
        return dict(inspect.signature(callable_value).parameters)
    except (TypeError, ValueError):
        return {}


def _resolve_project_path(raw_path: str) -> Path:
    path = Path(raw_path).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def _round_up(value: int, divisor: int) -> int:
    return int(math.ceil(value / divisor) * divisor)


def _clamp(value: Any, lower: float, upper: float, fallback: float) -> float:
    parsed = _safe_float(value, fallback)
    return max(lower, min(upper, parsed))


def _safe_float(value: Any, fallback: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _safe_int(value: Any, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


def _valid_frame(frame: Any) -> bool:
    return frame is not None and hasattr(frame, "shape") and getattr(frame, "size", 0) > 0
