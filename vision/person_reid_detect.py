"""Person detection plus appearance-based target re-identification.

Heavy third-party models are loaded lazily so non-vision modes keep working
when the optional ReID dependencies are not installed.
"""

import math
import os
from pathlib import Path
from time import perf_counter
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from vision.debug_overlay import BoxAnnotation, draw_box_annotations
from vision.detector_protocol import DetectionResult

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VISUAL_OBJECT_CLASSES = (
    "bicycle",
    "car",
    "motorcycle",
    "bus",
    "truck",
    "bench",
    "chair",
    "couch",
    "suitcase",
    "backpack",
)
VISUAL_OBJECT_LABELS = {
    "bicycle": "自行车",
    "car": "汽车",
    "motorcycle": "摩托车",
    "bus": "公交车",
    "truck": "卡车",
    "bench": "长椅",
    "chair": "椅子",
    "couch": "沙发",
    "suitcase": "行李箱",
    "backpack": "背包",
}


class UltralyticsPersonDetector:
    """Return person boxes and optional visual object boxes."""

    def __init__(
        self,
        model_path: str,
        confidence: float,
        device: str,
        visual_object_detection_enabled: bool = False,
        visual_object_confidence: float = 0.35,
        visual_object_classes: Sequence[str] = DEFAULT_VISUAL_OBJECT_CLASSES,
    ) -> None:
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
        self.visual_object_detection_enabled = bool(visual_object_detection_enabled)
        self.visual_object_confidence = visual_object_confidence
        self.visual_object_classes = {
            str(value).strip().lower()
            for value in visual_object_classes
            if str(value).strip()
        }

    def detect_people(self, frame: Any) -> List[DetectionResult]:
        return self.detect_scene(frame)["people"]

    def detect_scene(self, frame: Any) -> Dict[str, List[DetectionResult]]:
        confidence = (
            min(self.confidence, self.visual_object_confidence)
            if self.visual_object_detection_enabled
            else self.confidence
        )
        results = self.model.predict(
            source=frame,
            classes=None if self.visual_object_detection_enabled else [0],
            conf=confidence,
            device=self.device,
            verbose=False,
        )
        people: List[DetectionResult] = []
        visual_objects: List[DetectionResult] = []
        if not results:
            return {"people": people, "visual_objects": visual_objects}
        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return {"people": people, "visual_objects": visual_objects}
        xyxy_values = boxes.xyxy.detach().cpu().numpy()
        confidence_values = boxes.conf.detach().cpu().numpy()
        class_values = getattr(boxes, "cls", None)
        if class_values is not None:
            class_values = class_values.detach().cpu().numpy()
        else:
            class_values = np.zeros(len(xyxy_values), dtype=np.float32)
        for xyxy, confidence_value, class_value in zip(
            xyxy_values, confidence_values, class_values
        ):
            class_id = int(class_value)
            class_name = self._class_name(class_id)
            confidence_float = float(confidence_value)
            detection = {
                "bbox_xyxy": tuple(float(value) for value in xyxy[:4]),
                "confidence": confidence_float,
                "class_id": class_id,
                "class_name": class_name,
            }
            if class_id == 0:
                if confidence_float >= self.confidence:
                    people.append(detection)
            elif (
                self.visual_object_detection_enabled
                and confidence_float >= self.visual_object_confidence
                and class_name.lower() in self.visual_object_classes
            ):
                detection["display_label"] = _visual_object_label(class_name)
                visual_objects.append(detection)
        return {"people": people, "visual_objects": visual_objects}

    def _class_name(self, class_id: int) -> str:
        names = getattr(self.model, "names", {})
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)



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
        detector_model_path: str = "weights/yolo11n.pt",
        reid_model_name: str = "osnet_x0_25",
        reid_model_path: str = "",
        device: str = "cpu",
        detection_confidence: float = 0.45,
        similarity_threshold: float = 0.65,
        ambiguity_margin: float = 0.05,
        temporary_lost_frames: int = 0,
        target_area_ratio_min: float = 0.03,
        target_area_ratio_max: float = 0.08,
        visual_object_detection_enabled: bool = False,
        visual_object_confidence: float = 0.35,
        visual_object_classes: Sequence[str] = DEFAULT_VISUAL_OBJECT_CLASSES,
        person_detector: Optional[Any] = None,
        feature_extractor: Optional[Any] = None,
        orientation_estimator: Optional[Any] = None,
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
        self.target_area_ratio_min = _clamp_float(target_area_ratio_min, 0.0, 1.0, 0.03)
        self.target_area_ratio_max = _clamp_float(target_area_ratio_max, 0.0, 1.0, 0.08)
        if self.target_area_ratio_max <= self.target_area_ratio_min:
            self.target_area_ratio_min = 0.03
            self.target_area_ratio_max = 0.08
        self.visual_object_detection_enabled = bool(visual_object_detection_enabled)
        self.visual_object_confidence = _clamp_float(
            visual_object_confidence, 0.01, 1.0, 0.35
        )
        self.visual_object_classes = tuple(
            str(value).strip().lower()
            for value in visual_object_classes
            if str(value).strip()
        )
        self._last_visual_objects: List[DetectionResult] = []
        self._person_detector = person_detector
        self._feature_extractor = feature_extractor
        self._orientation_estimator = orientation_estimator
        self._orientation_prepared = False
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
        orientation_estimator = None
        if _safe_bool(cfg.get("jointbdoe_enabled"), False):
            from vision.jointbdoe_orientation import JointBDOEOrientationEstimator

            orientation_estimator = JointBDOEOrientationEstimator.from_config(config)
        return cls(
            reference_image_paths=reference_images,
            detector_model_path=str(
                cfg.get("person_detector_model", "weights/yolo11n.pt")
            ),
            reid_model_name=str(cfg.get("reid_model_name", "osnet_x0_25")),
            reid_model_path=str(cfg.get("reid_model_path", "")),
            device=str(cfg.get("reid_device", "cpu")),
            visual_object_detection_enabled=_safe_bool(
                cfg.get("visual_object_detection_enabled"), False
            ),
            visual_object_confidence=_safe_float(
                cfg.get("visual_object_confidence"), 0.35
            ),
            visual_object_classes=_parse_string_list(
                cfg.get("visual_object_classes", DEFAULT_VISUAL_OBJECT_CLASSES)
            ),
            detection_confidence=_safe_float(cfg.get("person_detection_confidence"), 0.45),
            similarity_threshold=_safe_float(cfg.get("reid_similarity_threshold"), 0.65),
            ambiguity_margin=_safe_float(cfg.get("reid_ambiguity_margin"), 0.05),
            temporary_lost_frames=_safe_int(cfg.get("reid_temporary_lost_frames"), 0),
            target_area_ratio_min=_safe_float(config.get("target_area_ratio_min"), 0.03),
            target_area_ratio_max=_safe_float(config.get("target_area_ratio_max"), 0.08),
            orientation_estimator=orientation_estimator,
            reference_features=reference_features,
        )

    def detect(self, frame: Any) -> DetectionResult:
        if not _valid_frame(frame):
            return self._empty_result(similarity_threshold=self.similarity_threshold)
        self._ensure_ready()
        scene = self._detect_scene(frame)
        self._last_visual_objects = list(scene["visual_objects"])
        detections = scene["people"]
        candidates = self._prepare_candidates(frame, detections)
        if not candidates:
            return self._handle_not_found()

        rgb_crops = [candidate[1][:, :, ::-1].copy() for candidate in candidates]
        features = self._normalize_rows(self._feature_extractor.extract(rgb_crops))
        if features.shape[0] != len(candidates):
            raise RuntimeError("ReID 特征数量与行人候选数量不一致。")
        similarities = features @ self._reference_feature
        frame_area = max(1, int(frame.shape[0]) * int(frame.shape[1]))
        candidate_diagnostics = [
            {
                "bbox": candidate[0],
                "similarity": float(similarity),
                "area_ratio": float(candidate[0][2] * candidate[0][3]) / frame_area,
                "distance_state": self._distance_state(
                    float(candidate[0][2] * candidate[0][3]) / frame_area
                ),
                "role": "unclassified",
                "display_label": "人物",
            }
            for candidate, similarity in zip(candidates, similarities)
        ]
        order = np.argsort(similarities)[::-1]
        best_index = int(order[0])
        best_similarity = float(similarities[best_index])
        second_similarity = float(similarities[int(order[1])]) if len(order) > 1 else -1.0
        if best_similarity < self.similarity_threshold:
            self._set_candidate_roles(candidate_diagnostics, "non_target")
            return self._handle_not_found(
                best_similarity=best_similarity,
                second_similarity=second_similarity if len(order) > 1 else None,
                candidate_diagnostics=candidate_diagnostics,
            )
        if len(order) > 1 and best_similarity - second_similarity < self.ambiguity_margin:
            self._set_candidate_roles(candidate_diagnostics, "ambiguous")
            return self._handle_not_found(
                best_similarity=best_similarity,
                second_similarity=second_similarity,
                ambiguous=True,
                candidate_diagnostics=candidate_diagnostics,
            )

        self._set_candidate_roles(candidate_diagnostics, "non_target", best_index)
        x, y, width, height = candidates[best_index][0]
        center = (x + width // 2, y + height // 2)
        result: DetectionResult = {
            "found": True,
            "is_predicted": False,
            "center": center,
            "target_center_x": center[0],
            "target_center_y": center[1],
            "area": float(width * height),
            "area_ratio": float(width * height) / frame_area,
            "bbox": (x, y, width, height),
            "similarity": best_similarity,
            "second_similarity": second_similarity if len(order) > 1 else None,
            "ambiguous": False,
            "candidate_count": len(candidates),
            "candidates": candidate_diagnostics,
            "visual_objects": list(self._last_visual_objects),
            "similarity_threshold": self.similarity_threshold,
        }
        if self._orientation_estimator is not None:
            orientation = self._orientation_estimator.estimate(frame, result["bbox"])
            if orientation:
                result.update(orientation)
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
        accepted_bbox = result.get("bbox") if result.get("found") else None
        candidate_diagnostics = result.get("candidates") or []
        annotations: List[BoxAnnotation] = []
        accepted_box_drawn = False
        for index, candidate in enumerate(candidate_diagnostics, start=1):
            bbox = candidate.get("bbox")
            if bbox is None or len(bbox) != 4:
                continue
            x, y, box_width, box_height = (int(value) for value in bbox)
            accepted = accepted_bbox is not None and tuple(bbox) == tuple(accepted_bbox)
            accepted_box_drawn = accepted_box_drawn or accepted
            similarity = candidate.get("similarity")
            similarity_text = "N/A" if similarity is None else f"{float(similarity):.3f}"
            area_ratio = float(candidate.get("area_ratio") or 0.0)
            distance_state = str(candidate.get("distance_state") or "N/A")
            role = str(candidate.get("role") or "unclassified")
            display_label = str(candidate.get("display_label") or "人物")
            color = (0, 255, 0) if role == "target" else (0, 165, 255)
            if role == "ambiguous":
                color = (0, 255, 255)
            annotations.append(
                BoxAnnotation(
                    (x, y, box_width, box_height),
                    f"{display_label} {similarity_text}",
                    color,
                )
            )
            accepted_box_drawn = accepted_box_drawn or accepted

        for visual_object in result.get("visual_objects") or []:
            bbox = visual_object.get("bbox_xyxy")
            if bbox is None or len(bbox) != 4:
                continue
            x1, y1, x2, y2 = (int(round(float(value))) for value in bbox)
            object_label = str(visual_object.get("display_label") or "视觉物体")
            annotations.append(
                BoxAnnotation(
                    (x1, y1, max(1, x2 - x1), max(1, y2 - y1)),
                    object_label,
                    (0, 0, 255),
                )
            )

        debug = draw_box_annotations(debug, annotations)

        if result.get("found") and not accepted_box_drawn:
            x, y, box_width, box_height = result["bbox"]
            color = (0, 180, 255) if result.get("is_predicted") else (0, 255, 0)
            cv2.rectangle(debug, (x, y), (x + box_width, y + box_height), color, 2)

        candidate_count = int(result.get("candidate_count") or 0)
        best_similarity = result.get("similarity")
        best_text = "N/A" if best_similarity is None else f"{float(best_similarity):.3f}"
        threshold = float(result.get("similarity_threshold", self.similarity_threshold))
        cv2.putText(
            debug,
            f"YOLO people={candidate_count} objects={len(result.get('visual_objects') or [])} best={best_text} threshold={threshold:.3f}",
            (20, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        if result.get("found"):
            state = "PREDICTED" if result.get("is_predicted") else "MATCH"
            status_color = (0, 180, 255) if result.get("is_predicted") else (0, 255, 0)
        elif result.get("ambiguous"):
            state = "AMBIGUOUS"
            status_color = (0, 165, 255)
        elif candidate_count:
            state = "BELOW THRESHOLD"
            status_color = (0, 0, 255)
        else:
            state = "NO PERSON"
            status_color = (0, 0, 255)
        best_candidate = None
        if candidate_diagnostics:
            best_candidate = max(
                candidate_diagnostics,
                key=lambda item: float(item.get("similarity") or -1.0),
            )
        best_area_text = "N/A"
        best_distance_text = "N/A"
        if best_candidate is not None:
            best_area_text = f"{float(best_candidate.get('area_ratio') or 0.0):.3f}"
            best_distance_text = str(best_candidate.get("distance_state") or "N/A")
        cv2.putText(
            debug,
            f"ReID {state} | BEST AREA={best_area_text} {best_distance_text}",
            (20, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            status_color,
            2,
        )
        angle = result.get("body_orientation_angle")
        if result.get("found") and angle is not None:
            detection_confidence = float(
                result.get("body_orientation_detection_confidence") or 0.0
            )
            match_iou = float(result.get("body_orientation_match_iou") or 0.0)
            latency_ms = float(result.get("body_orientation_latency_ms") or 0.0)
            cv2.putText(
                debug,
                f"JointBDOE angle={float(angle):.1f} deg det={detection_confidence:.2f} iou={match_iou:.2f} {latency_ms:.0f} ms",
                (20, 88),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
            x, y, box_width, box_height = result["bbox"]
            start = (int(x + box_width / 2), int(y + box_height / 2))
            arrow_length = max(12, int(min(box_width, box_height) / 3))
            radians = math.radians(float(angle))
            end = (
                int(start[0] - arrow_length * math.sin(radians)),
                int(start[1] - arrow_length * math.cos(radians)),
            )
            cv2.arrowedLine(
                debug,
                start,
                end,
                (0, 255, 255),
                2,
                line_type=cv2.LINE_AA,
                tipLength=0.3,
            )
        return debug

    def _distance_state(self, area_ratio: float) -> str:
        """Classify a person-box ratio using the active follow distance band."""
        if area_ratio < self.target_area_ratio_min:
            return "FAR"
        if area_ratio > self.target_area_ratio_max:
            return "NEAR"
        return "OK"

    def reset(self) -> None:
        self._last_visual_objects = []
        self._lost_count = 0
        self._last_valid_result = None
        if self._orientation_estimator is not None and hasattr(
            self._orientation_estimator, "reset"
        ):
            self._orientation_estimator.reset()

    def _detect_scene(self, frame: Any) -> Dict[str, List[DetectionResult]]:
        if self._person_detector is not None and hasattr(
            self._person_detector, "detect_scene"
        ):
            return self._person_detector.detect_scene(frame)
        return {
            "people": self._person_detector.detect_people(frame),
            "visual_objects": [],
        }

    def _set_candidate_roles(
        self,
        diagnostics: List[Dict[str, object]],
        role: str,
        target_index: Optional[int] = None,
    ) -> None:
        labels = {
            "target": "目标人物",
            "non_target": "非目标人物",
            "ambiguous": "身份未确认",
        }
        for index, candidate in enumerate(diagnostics):
            candidate_role = "target" if target_index == index else role
            candidate["role"] = candidate_role
            candidate["display_label"] = labels.get(candidate_role, "人物")

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
                self.visual_object_detection_enabled,
                self.visual_object_confidence,
                self.visual_object_classes,
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
        if self._orientation_estimator is not None and not self._orientation_prepared:
            step_started = perf_counter()
            print("JointBDOE 准备：正在加载人体朝向模型...")
            if hasattr(self._orientation_estimator, "prepare"):
                self._orientation_estimator.prepare()
            self._orientation_prepared = True
            print(f"JointBDOE 准备完成：{perf_counter() - step_started:.2f} 秒")
            initialized_component = True
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
        self,
        best_similarity: Optional[float] = None,
        second_similarity: Optional[float] = None,
        ambiguous: bool = False,
        candidate_diagnostics: Optional[List[Dict[str, object]]] = None,
    ) -> DetectionResult:
        diagnostics = list(candidate_diagnostics or [])
        self._lost_count += 1
        if self._last_valid_result is not None and self._lost_count <= self.temporary_lost_frames:
            predicted = dict(self._last_valid_result)
            predicted["is_predicted"] = True
            predicted["candidate_count"] = len(diagnostics)
            predicted["candidates"] = diagnostics
            predicted["similarity"] = best_similarity
            predicted["second_similarity"] = second_similarity
            predicted["ambiguous"] = ambiguous
            predicted["visual_objects"] = list(self._last_visual_objects)
            return predicted
        result = self._empty_result(
            best_similarity,
            second_similarity,
            ambiguous,
            diagnostics,
            self.similarity_threshold,
        )
        result["visual_objects"] = list(self._last_visual_objects)
        return result

    @staticmethod
    def _empty_result(
        best_similarity: Optional[float] = None,
        second_similarity: Optional[float] = None,
        ambiguous: bool = False,
        candidate_diagnostics: Optional[List[Dict[str, object]]] = None,
        similarity_threshold: float = 0.65,
    ) -> DetectionResult:
        diagnostics = list(candidate_diagnostics or [])
        return {
            "found": False,
            "is_predicted": False,
            "center": None,
            "target_center_x": None,
            "target_center_y": None,
            "area": 0.0,
            "bbox": None,
            "similarity": best_similarity,
            "second_similarity": second_similarity,
            "ambiguous": ambiguous,
            "candidate_count": len(diagnostics),
            "candidates": diagnostics,
            "visual_objects": [],
            "similarity_threshold": similarity_threshold,
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


def _visual_object_label(class_name: str) -> str:
    display_name = VISUAL_OBJECT_LABELS.get(class_name.lower(), class_name)
    return f"障碍物候选：{display_name}"


def _parse_string_list(value: Any) -> List[str]:
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def _safe_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    return default


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
