"""Person detection plus appearance-based target re-identification.

The detector keeps the same small result contract as the red and ArUco
detectors.  Heavy third-party models are loaded lazily so existing modes keep
working when the optional ReID dependencies are not installed.
"""

import os
from pathlib import Path
from time import perf_counter
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from vision.detector_protocol import DetectionResult

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class UltralyticsPersonDetector:
    """Return person bounding boxes from an Ultralytics detection model."""

    def __init__(self, model_path: str, confidence: float, device: str) -> None:
        # RoboMaster TT / Tello Wi-Fi normally has no internet access.  Without
        # this hint, Ultralytics performs an online-status DNS lookup while it
        # is imported and macOS can wait about a minute for that lookup to time
        # out.  setdefault keeps an explicit caller override available.
        os.environ.setdefault("YOLO_OFFLINE", "1")
        try:
            from ultralytics import YOLO
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "缺少 ReID 行人检测依赖：请安装 requirements-reid.txt。"
            ) from exc
        self.model = YOLO(model_path)
        self.confidence = confidence
        self.device = device

    def detect_people(self, frame: Any) -> List[DetectionResult]:
        results = self.model.predict(
            source=frame,
            classes=[0],
            conf=self.confidence,
            device=self.device,
            verbose=False,
        )
        people: List[DetectionResult] = []
        if not results:
            return people
        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return people
        xyxy_values = boxes.xyxy.detach().cpu().numpy()
        confidence_values = boxes.conf.detach().cpu().numpy()
        for xyxy, confidence in zip(xyxy_values, confidence_values):
            people.append(
                {
                    "bbox_xyxy": tuple(float(value) for value in xyxy[:4]),
                    "confidence": float(confidence),
                }
            )
        return people


class TorchreidFeatureExtractor:
    """Extract OSNet embeddings using Torchreid's public feature API."""

    def __init__(self, model_name: str, model_path: str, device: str) -> None:
        try:
            from torchreid.utils import FeatureExtractor
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "缺少 Torchreid：请安装 requirements-reid.txt，并准备 ReID 权重。"
            ) from exc
        if not model_path:
            raise RuntimeError("未配置 vision.reid_model_path，无法加载 ReID 权重。")
        resolved_path = _resolve_project_path(model_path)
        if not resolved_path.is_file():
            raise RuntimeError(f"ReID 权重不存在：{resolved_path}")
        self.extractor = FeatureExtractor(
            model_name=model_name,
            model_path=str(resolved_path),
            device=device,
        )

    def extract(self, rgb_images: Sequence[Any]) -> np.ndarray:
        features = self.extractor(list(rgb_images))
        if hasattr(features, "detach"):
            features = features.detach().cpu().numpy()
        return np.asarray(features, dtype=np.float32)


class PersonReIDDetector:
    """Find the registered person and expose a follow-controller result.

    A target is accepted only when its cosine similarity reaches the configured
    threshold and is sufficiently better than the second-best candidate.  This
    avoids commanding the drone toward an ambiguous person in a crowd.
    """

    def __init__(
        self,
        reference_image_paths: Sequence[str],
        detector_model_path: str = "weights/yolov8n.pt",
        reid_model_name: str = "osnet_x0_25",
        reid_model_path: str = "",
        device: str = "cpu",
        detection_confidence: float = 0.45,
        similarity_threshold: float = 0.65,
        ambiguity_margin: float = 0.05,
        temporary_lost_frames: int = 0,
        person_detector: Optional[Any] = None,
        feature_extractor: Optional[Any] = None,
        reference_features: Optional[Any] = None,
    ) -> None:
        self.reference_image_paths = [str(path) for path in reference_image_paths if str(path).strip()]
        self.detector_model_path = detector_model_path
        self.reid_model_name = reid_model_name
        self.reid_model_path = reid_model_path
        self.device = str(device).strip() or "cpu"
        self.detection_confidence = _clamp_float(detection_confidence, 0.01, 1.0, 0.45)
        self.similarity_threshold = _clamp_float(similarity_threshold, -1.0, 1.0, 0.65)
        self.ambiguity_margin = _clamp_float(ambiguity_margin, 0.0, 2.0, 0.05)
        self.temporary_lost_frames = max(0, int(temporary_lost_frames))
        self._person_detector = person_detector
        self._feature_extractor = feature_extractor
        self._reference_feature: Optional[np.ndarray] = None
        self._lost_count = 0
        self._last_valid_result: Optional[DetectionResult] = None
        if reference_features is not None:
            self._reference_feature = self._average_normalized(reference_features)

    @classmethod
    def from_config(cls, config: Dict[str, object]) -> "PersonReIDDetector":
        vision = config.get("vision", {})
        cfg = vision if isinstance(vision, dict) else config
        reference_images = _parse_reference_images(cfg.get("reference_images", ""))
        reference_features = None
        profile_name = str(cfg.get("reference_profile", "")).strip()
        if profile_name:
            from vision.reid_profiles import load_reid_profile

            reference_features, _manifest = load_reid_profile(profile_name, config)
            reference_images = []
        return cls(
            reference_image_paths=reference_images,
            detector_model_path=str(
                cfg.get("person_detector_model", "weights/yolov8n.pt")
            ),
            reid_model_name=str(cfg.get("reid_model_name", "osnet_x0_25")),
            reid_model_path=str(cfg.get("reid_model_path", "")),
            device=str(cfg.get("reid_device", "cpu")),
            detection_confidence=_safe_float(cfg.get("person_detection_confidence"), 0.45),
            similarity_threshold=_safe_float(cfg.get("reid_similarity_threshold"), 0.65),
            ambiguity_margin=_safe_float(cfg.get("reid_ambiguity_margin"), 0.05),
            temporary_lost_frames=_safe_int(cfg.get("reid_temporary_lost_frames"), 0),
            reference_features=reference_features,
        )

    def detect(self, frame: Any) -> DetectionResult:
        if not _valid_frame(frame):
            return self._empty_result()
        self._ensure_ready()
        detections = self._person_detector.detect_people(frame)
        candidates = self._prepare_candidates(frame, detections)
        if not candidates:
            return self._handle_not_found()

        rgb_crops = [candidate[1][:, :, ::-1].copy() for candidate in candidates]
        features = self._normalize_rows(self._feature_extractor.extract(rgb_crops))
        if features.shape[0] != len(candidates):
            raise RuntimeError("ReID 特征数量与行人候选数量不一致。")
        similarities = features @ self._reference_feature
        order = np.argsort(similarities)[::-1]
        best_index = int(order[0])
        best_similarity = float(similarities[best_index])
        second_similarity = float(similarities[int(order[1])]) if len(order) > 1 else -1.0
        if best_similarity < self.similarity_threshold:
            return self._handle_not_found(best_similarity=best_similarity)
        if len(order) > 1 and best_similarity - second_similarity < self.ambiguity_margin:
            return self._handle_not_found(best_similarity=best_similarity, ambiguous=True)

        x, y, width, height = candidates[best_index][0]
        center = (x + width // 2, y + height // 2)
        result: DetectionResult = {
            "found": True,
            "is_predicted": False,
            "center": center,
            "target_center_x": center[0],
            "target_center_y": center[1],
            "area": float(width * height),
            "bbox": (x, y, width, height),
            "detector_type": "person_reid",
            "similarity": best_similarity,
            "second_similarity": second_similarity if len(order) > 1 else None,
            "ambiguous": False,
            "candidate_count": len(candidates),
        }
        self._lost_count = 0
        self._last_valid_result = dict(result)
        return result

    def prepare(self) -> None:
        """Load models and reference features before any flight can start."""
        self._ensure_ready()

    @property
    def reference_feature(self) -> np.ndarray:
        """Return a copy of the prepared normalized reference embedding."""
        if self._reference_feature is None:
            raise RuntimeError("人物参考特征尚未准备。")
        return self._reference_feature.copy()

    def draw_debug(self, frame: Any, result: DetectionResult) -> Any:
        if not _valid_frame(frame):
            return frame
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 opencv-python 依赖：请先安装 requirements.txt。") from exc
        debug = frame.copy()
        height, width = debug.shape[:2]
        frame_center = (width // 2, height // 2)
        cv2.line(debug, (frame_center[0], 0), (frame_center[0], height), (255, 0, 0), 1)
        cv2.line(debug, (0, frame_center[1]), (width, frame_center[1]), (255, 0, 0), 1)
        if result.get("found"):
            x, y, box_width, box_height = result["bbox"]
            color = (0, 180, 255) if result.get("is_predicted") else (0, 255, 0)
            cv2.rectangle(debug, (x, y), (x + box_width, y + box_height), color, 2)
            similarity = float(result.get("similarity") or 0.0)
            state = "PREDICTED" if result.get("is_predicted") else "MATCH"
            cv2.putText(
                debug,
                f"ReID {state} sim={similarity:.3f}",
                (max(0, x), max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
        else:
            reason = "AMBIGUOUS" if result.get("ambiguous") else "LOST"
            cv2.putText(debug, f"ReID {reason}", (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return debug

    def reset(self) -> None:
        self._lost_count = 0
        self._last_valid_result = None

    def _ensure_ready(self) -> None:
        prepare_started = perf_counter()
        initialized_component = False
        if self._person_detector is None:
            step_started = perf_counter()
            print("ReID 准备 1/3：正在加载 YOLO 行人检测模型...")
            self._person_detector = UltralyticsPersonDetector(
                self.detector_model_path,
                self.detection_confidence,
                self.device,
            )
            print(f"ReID 准备 1/3 完成：{perf_counter() - step_started:.2f} 秒")
            initialized_component = True
        if self._feature_extractor is None:
            step_started = perf_counter()
            print("ReID 准备 2/3：正在加载 OSNet 特征模型...")
            self._feature_extractor = TorchreidFeatureExtractor(
                self.reid_model_name,
                self.reid_model_path,
                self.device,
            )
            print(f"ReID 准备 2/3 完成：{perf_counter() - step_started:.2f} 秒")
            initialized_component = True
        if self._reference_feature is None:
            step_started = perf_counter()
            print("ReID 准备 3/3：正在从参考照片计算人物特征...")
            self._reference_feature = self._load_reference_feature()
            print(f"ReID 准备 3/3 完成：{perf_counter() - step_started:.2f} 秒")
            initialized_component = True
        elif initialized_component:
            print("ReID 准备 3/3：已直接加载人物档案特征，无需计算照片。")
        if initialized_component:
            print(f"ReID 模型准备完成：共 {perf_counter() - prepare_started:.2f} 秒")

    def _load_reference_feature(self) -> np.ndarray:
        if not self.reference_image_paths:
            raise RuntimeError(
                "未配置目标人物参考图。请在 vision.reference_images 中填写至少一张照片。"
            )
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 opencv-python 依赖：请先安装 requirements.txt。") from exc
        images = []
        missing = []
        for raw_path in self.reference_image_paths:
            path = _resolve_project_path(raw_path)
            image = cv2.imread(str(path)) if path.is_file() else None
            if image is None:
                missing.append(str(path))
            else:
                detections = self._person_detector.detect_people(image)
                candidates = self._prepare_candidates(image, detections)
                if not candidates:
                    raise RuntimeError(
                        f"参考照片中未检测到完整人物：{path}。"
                        "请使用人物全身清晰、光线充足的照片。"
                    )
                if len(candidates) != 1:
                    raise RuntimeError(
                        f"参考照片中检测到多个人：{path}。"
                        "请重新拍摄只包含目标人物的照片。"
                    )
                # Embed the detected person crop rather than the whole photo so
                # the enrollment representation matches runtime person crops.
                _, crop = candidates[0]
                images.append(crop[:, :, ::-1].copy())
        if missing:
            raise RuntimeError("目标人物参考图不存在或无法读取：" + ", ".join(missing))
        return self._average_normalized(self._feature_extractor.extract(images))

    def _prepare_candidates(
        self, frame: Any, detections: Iterable[DetectionResult]
    ) -> List[Tuple[Tuple[int, int, int, int], Any]]:
        frame_height, frame_width = frame.shape[:2]
        candidates = []
        for detection in detections:
            raw_box = detection.get("bbox_xyxy")
            if raw_box is None or len(raw_box) != 4:
                continue
            x1 = max(0, min(frame_width, int(round(float(raw_box[0])))))
            y1 = max(0, min(frame_height, int(round(float(raw_box[1])))))
            x2 = max(0, min(frame_width, int(round(float(raw_box[2])))))
            y2 = max(0, min(frame_height, int(round(float(raw_box[3])))))
            if x2 <= x1 or y2 <= y1:
                continue
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            candidates.append(((x1, y1, x2 - x1, y2 - y1), crop))
        return candidates

    def _handle_not_found(
        self, best_similarity: Optional[float] = None, ambiguous: bool = False
    ) -> DetectionResult:
        self._lost_count += 1
        if self._last_valid_result is not None and self._lost_count <= self.temporary_lost_frames:
            predicted = dict(self._last_valid_result)
            predicted["is_predicted"] = True
            predicted["candidate_count"] = 0
            return predicted
        return self._empty_result(best_similarity, ambiguous)

    @staticmethod
    def _empty_result(
        best_similarity: Optional[float] = None, ambiguous: bool = False
    ) -> DetectionResult:
        return {
            "found": False,
            "is_predicted": False,
            "center": None,
            "target_center_x": None,
            "target_center_y": None,
            "area": 0.0,
            "bbox": None,
            "detector_type": "person_reid",
            "similarity": best_similarity,
            "second_similarity": None,
            "ambiguous": ambiguous,
            "candidate_count": 0,
        }

    @classmethod
    def _average_normalized(cls, features: Any) -> np.ndarray:
        normalized = cls._normalize_rows(features)
        if normalized.shape[0] == 0:
            raise RuntimeError("没有可用的目标人物 ReID 特征。")
        average = normalized.mean(axis=0)
        norm = float(np.linalg.norm(average))
        if norm <= 1e-12:
            raise RuntimeError("目标人物 ReID 特征无效。")
        return (average / norm).astype(np.float32)

    @staticmethod
    def _normalize_rows(features: Any) -> np.ndarray:
        values = np.asarray(features, dtype=np.float32)
        if values.ndim == 1:
            values = values.reshape(1, -1)
        if values.ndim != 2:
            raise RuntimeError("ReID 特征必须是二维数组。")
        norms = np.linalg.norm(values, axis=1, keepdims=True)
        if np.any(norms <= 1e-12):
            raise RuntimeError("ReID 模型返回了零向量。")
        return values / norms


def _parse_reference_images(value: Any) -> List[str]:
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def _resolve_project_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def _valid_frame(frame: Any) -> bool:
    return isinstance(frame, np.ndarray) and frame.ndim >= 2 and frame.size > 0


def _safe_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _clamp_float(value: Any, lower: float, upper: float, default: float) -> float:
    return max(lower, min(upper, _safe_float(value, default)))
