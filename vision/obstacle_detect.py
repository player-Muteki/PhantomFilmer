"""Monocular obstacle observation for the low-speed flight pipeline."""

from dataclasses import dataclass, field
from time import monotonic
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


Box = Tuple[int, int, int, int]


@dataclass
class ObstacleCandidate:
    """One obstacle candidate extracted from a camera frame."""

    bbox: Box
    center: Tuple[int, int]
    area: float
    area_ratio: float
    risk_zone_coverage: float
    side: str
    confidence: float
    motion_score: float = 0.0
    ttc_seconds: Optional[float] = None

    def to_dict(self, frame_width: int, frame_height: int) -> Dict[str, object]:
        """Return a compact LLM-friendly representation."""
        x, y, width, height = self.bbox
        return {
            "bbox": list(self.bbox),
            "bbox_norm": [
                round(x / max(1, frame_width), 4),
                round(y / max(1, frame_height), 4),
                round(width / max(1, frame_width), 4),
                round(height / max(1, frame_height), 4),
            ],
            "center": list(self.center),
            "center_norm": [
                round(self.center[0] / max(1, frame_width), 4),
                round(self.center[1] / max(1, frame_height), 4),
            ],
            "area": round(self.area, 2),
            "area_ratio": round(self.area_ratio, 6),
            "risk_zone_coverage": round(self.risk_zone_coverage, 6),
            "side": self.side,
            "confidence": round(self.confidence, 4),
            "motion_score": round(self.motion_score, 4),
            "ttc_seconds": None if self.ttc_seconds is None else round(self.ttc_seconds, 3),
        }


@dataclass
class ObstacleResult:
    """Obstacle-risk result for one frame, with legacy fields preserved."""

    found: bool = False
    state: str = "CLEAR"
    center: Optional[Tuple[int, int]] = None
    bbox: Optional[Box] = None
    area: float = 0.0
    area_ratio: float = 0.0
    risk_zone: Optional[Box] = None
    risk_zone_coverage: float = 0.0
    side: str = "none"
    candidates: List[ObstacleCandidate] = field(default_factory=list)
    free_space: Dict[str, float] = field(default_factory=dict)
    confidence: float = 0.0
    motion_score: float = 0.0
    ttc_seconds: Optional[float] = None
    frame_size: Optional[Tuple[int, int]] = None
    frame_index: int = 0
    timestamp: float = 0.0
    data_quality: str = "ok"
    consecutive_found_frames: int = 0
    consecutive_clear_frames: int = 0

    def to_observation(self, frame_width: int, frame_height: int) -> Dict[str, object]:
        """Serialize the observation without including image bytes."""
        return {
            "frame": {"index": self.frame_index, "width": frame_width, "height": frame_height},
            "found": self.found,
            "state": self.state,
            "center": None if self.center is None else list(self.center),
            "bbox": None if self.bbox is None else list(self.bbox),
            "area": round(self.area, 2),
            "area_ratio": round(self.area_ratio, 6),
            "risk_zone": None if self.risk_zone is None else list(self.risk_zone),
            "risk_zone_coverage": round(self.risk_zone_coverage, 6),
            "side": self.side,
            "obstacles": [candidate.to_dict(frame_width, frame_height) for candidate in self.candidates],
            "free_space": {key: round(value, 4) for key, value in self.free_space.items()},
            "confidence": round(self.confidence, 4),
            "motion_score": round(self.motion_score, 4),
            "ttc_seconds": None if self.ttc_seconds is None else round(self.ttc_seconds, 3),
            "frame_size": None if self.frame_size is None else list(self.frame_size),
            "data_quality": self.data_quality,
            "consecutive_found_frames": self.consecutive_found_frames,
            "consecutive_clear_frames": self.consecutive_clear_frames,
        }


class ObstacleDetector:
    """Extract temporally-stabilized local obstacle observations from video."""

    def __init__(
        self,
        risk_zone_width_ratio: float = 0.55,
        risk_zone_height_ratio: float = 0.65,
        risk_zone_vertical_offset_ratio: float = 0.05,
        minimum_obstacle_area: int = 1500,
        caution_area_ratio: float = 0.08,
        blocked_area_ratio: float = 0.18,
        temporal_window_frames: int = 5,
        sector_count: int = 5,
    ) -> None:
        self.risk_zone_width_ratio = self._clamp_float(risk_zone_width_ratio, 0.05, 1.0, 0.55)
        self.risk_zone_height_ratio = self._clamp_float(risk_zone_height_ratio, 0.05, 1.0, 0.65)
        self.risk_zone_vertical_offset_ratio = self._clamp_float(
            risk_zone_vertical_offset_ratio, -0.5, 0.5, 0.05
        )
        self.minimum_obstacle_area = self._positive_int(minimum_obstacle_area, 1500)
        self.caution_area_ratio = self._clamp_float(caution_area_ratio, 0.0, 1.0, 0.08)
        self.blocked_area_ratio = self._clamp_float(blocked_area_ratio, self.caution_area_ratio, 1.0, 0.18)
        self.temporal_window_frames = self._positive_int(temporal_window_frames, 5)
        self.sector_count = max(3, self._positive_int(sector_count, 5))
        self.last_result = ObstacleResult()
        self.last_mask = None
        self._frame_index = 0
        self._found_frames = 0
        self._clear_frames = 0
        self._previous_signature: Optional[Tuple[float, float, float]] = None
        self._previous_at: Optional[float] = None

    @classmethod
    def from_config(cls, config: Dict[str, object]) -> "ObstacleDetector":
        """Build an obstacle detector from config.yaml."""
        obstacle = config.get("obstacle", {}) if isinstance(config, dict) else {}
        if not isinstance(obstacle, dict):
            obstacle = {}
        return cls(
            risk_zone_width_ratio=cls._config_float(obstacle, "risk_zone_width_ratio", 0.55),
            risk_zone_height_ratio=cls._config_float(obstacle, "risk_zone_height_ratio", 0.65),
            risk_zone_vertical_offset_ratio=cls._config_float(obstacle, "risk_zone_vertical_offset_ratio", 0.05),
            minimum_obstacle_area=cls._config_int(obstacle, "minimum_obstacle_area", 1500),
            caution_area_ratio=cls._config_float(obstacle, "caution_area_ratio", 0.08),
            blocked_area_ratio=cls._config_float(obstacle, "blocked_area_ratio", 0.18),
            temporal_window_frames=cls._config_int(obstacle, "temporal_window_frames", 5),
            # 时间确认（detect/clear confirm frames）由 planner 基于
            # consecutive_found_frames 执行，detector 不读取这两个配置项。
            sector_count=cls._config_int(obstacle, "sector_count", 5),
        )

    def detect(self, frame: Any, target_result: Dict[str, object]) -> ObstacleResult:
        """Return all meaningful local obstacle candidates for one frame."""
        self._frame_index += 1
        timestamp = monotonic()
        if not self._valid_frame(frame):
            self.last_mask = None
            self._found_frames = 0
            self._clear_frames += 1
            self.last_result = ObstacleResult(
                frame_index=self._frame_index,
                timestamp=timestamp,
                data_quality="invalid_frame",
                consecutive_clear_frames=self._clear_frames,
            )
            return self.last_result

        cv2 = _import_cv2()
        frame_height, frame_width = frame.shape[:2]
        frame_area = max(1, frame_width * frame_height)
        # 先裁剪到风险区再做 CV，避免对整帧做模糊/Canny/形态学，节省约 2-3x 计算量。
        zone_x, zone_y, zone_width, zone_height = self._risk_zone(frame_width, frame_height)
        risk_zone = (zone_x, zone_y, zone_width, zone_height)
        risk_area = max(1, zone_width * zone_height)
        zone = frame[zone_y:zone_y + zone_height, zone_x:zone_x + zone_width]
        zone_mask = np.full((zone_height, zone_width), 255, dtype=np.uint8)
        self._erase_target_region(zone_mask, target_result, zone_x, zone_y, zone_width, zone_height)

        gray = cv2.cvtColor(zone, cv2.COLOR_BGR2GRAY) if zone.ndim == 3 else zone
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        edges = cv2.bitwise_and(edges, zone_mask)
        kernel_size = 3 if self.temporal_window_frames < 4 else 5
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.dilate(edges, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        self.last_mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: List[ObstacleCandidate] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.minimum_obstacle_area:
                continue
            bx, by, bw, bh = cv2.boundingRect(contour)
            # 轮廓来自裁剪后的风险区，坐标映射回全帧，与下游（free_space/日志/调试）保持一致。
            bbox = (bx + zone_x, by + zone_y, bw, bh)
            overlap = self._intersect_boxes(bbox, risk_zone)
            overlap_area = self._box_area(overlap)
            if overlap_area <= 0:
                continue
            center = (bbox[0] + bw // 2, bbox[1] + bh // 2)
            area_ratio = area / frame_area
            coverage = overlap_area / risk_area
            confidence = self._confidence(area_ratio, coverage)
            candidates.append(
                ObstacleCandidate(
                    bbox=bbox,
                    center=center,
                    area=area,
                    area_ratio=area_ratio,
                    risk_zone_coverage=coverage,
                    side=self._side(center[0], frame_width),
                    confidence=confidence,
                )
            )

        candidates.sort(key=lambda candidate: self._candidate_score(candidate), reverse=True)
        best = candidates[0] if candidates else None
        motion_score, ttc_seconds = self._update_motion(best, frame_width, frame_height, timestamp)
        if best is not None:
            best.motion_score = motion_score
            best.ttc_seconds = ttc_seconds
            self._found_frames += 1
            self._clear_frames = 0
            state = "BLOCKED" if best.area_ratio >= self.blocked_area_ratio else "CAUTION"
        else:
            self._found_frames = 0
            self._clear_frames += 1
            state = "CLEAR"

        free_space = self._free_space(candidates, risk_zone, frame_width)
        self.last_result = ObstacleResult(
            found=best is not None,
            state=state,
            center=best.center if best else None,
            bbox=best.bbox if best else None,
            area=best.area if best else 0.0,
            area_ratio=best.area_ratio if best else 0.0,
            risk_zone=risk_zone,
            risk_zone_coverage=best.risk_zone_coverage if best else 0.0,
            side=best.side if best else "none",
            candidates=candidates,
            free_space=free_space,
            confidence=best.confidence if best else 0.0,
            motion_score=motion_score,
            ttc_seconds=ttc_seconds,
            frame_size=(frame_width, frame_height),
            frame_index=self._frame_index,
            timestamp=timestamp,
            consecutive_found_frames=self._found_frames,
            consecutive_clear_frames=self._clear_frames,
        )
        return self.last_result

    def draw_debug(self, frame: Any, result: Optional[ObstacleResult]) -> Any:
        """Draw risk-zone, candidate, free-space, and obstacle overlays."""
        if not self._valid_frame(frame):
            return frame
        cv2 = _import_cv2()
        debug = frame.copy()
        result = result or self.last_result
        if result.risk_zone is not None:
            x, y, width, height = result.risk_zone
            cv2.rectangle(debug, (x, y), (x + width, y + height), (255, 180, 0), 2)
        for candidate in result.candidates:
            x, y, width, height = candidate.bbox
            color = (0, 0, 255) if candidate is result.candidates[0] and result.state == "BLOCKED" else (0, 165, 255)
            cv2.rectangle(debug, (x, y), (x + width, y + height), color, 2)
            cv2.circle(debug, candidate.center, 4, color, -1)
        if result.found and result.bbox is not None:
            x, y, _, _ = result.bbox
            cv2.putText(
                debug,
                f"OBS {result.state} {result.side} {result.area_ratio:.3f} ttc={result.ttc_seconds or 0:.1f}",
                (max(0, x), max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255) if result.state == "BLOCKED" else (0, 165, 255),
                2,
            )
        return debug

    def reset(self) -> None:
        """Clear cached detection and temporal state."""
        self.last_result = ObstacleResult()
        self.last_mask = None
        self._frame_index = 0
        self._found_frames = 0
        self._clear_frames = 0
        self._previous_signature = None
        self._previous_at = None

    def _risk_zone(self, frame_width: int, frame_height: int) -> Box:
        zone_width = max(1, int(frame_width * self.risk_zone_width_ratio))
        zone_height = max(1, int(frame_height * self.risk_zone_height_ratio))
        x = max(0, min(frame_width - zone_width, (frame_width - zone_width) // 2))
        y_center = frame_height // 2 + int(frame_height * self.risk_zone_vertical_offset_ratio)
        y = max(0, min(frame_height - zone_height, y_center - zone_height // 2))
        return (x, y, zone_width, zone_height)

    def _erase_target_region(
        self,
        mask: Any,
        target_result: Dict[str, object],
        zone_x: int = 0,
        zone_y: int = 0,
        zone_width: Optional[int] = None,
        zone_height: Optional[int] = None,
    ) -> None:
        """将跟随目标区域从风险掩膜中擦除（掩膜为风险区局部坐标）。"""
        cv2 = _import_cv2()
        if zone_width is None or zone_height is None:
            zone_width, zone_height = mask.shape[1], mask.shape[0]
        bbox = target_result.get("bbox") if isinstance(target_result, dict) else None
        if bbox is not None:
            x, y, width, height = bbox  # type: ignore[misc]
            pad = max(8, int(max(width, height) * 0.18))
            x1 = max(0, int(x) - pad - zone_x)
            y1 = max(0, int(y) - pad - zone_y)
            x2 = min(zone_width, int(x + width) + pad - zone_x)
            y2 = min(zone_height, int(y + height) + pad - zone_y)
            if x2 > x1 and y2 > y1:
                mask[y1:y2, x1:x2] = 0
        corners = target_result.get("corners") if isinstance(target_result, dict) else None
        if corners is not None:
            points = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
            points[:, :, 0] -= zone_x
            points[:, :, 1] -= zone_y
            cv2.fillPoly(mask, [points], 0)

    def _free_space(self, candidates: List[ObstacleCandidate], risk_zone: Box, frame_width: int) -> Dict[str, float]:
        labels = self._sector_labels()
        _, _, zone_width, zone_height = risk_zone
        values = {label: 1.0 for label in labels}
        sector_width = zone_width / len(labels)
        for candidate in candidates:
            x, _, width, _ = candidate.bbox
            for index, label in enumerate(labels):
                sector = (risk_zone[0] + int(index * sector_width), risk_zone[1], max(1, int(sector_width)), zone_height)
                overlap = self._box_area(self._intersect_boxes(candidate.bbox, sector))
                if overlap:
                    values[label] = min(values[label], max(0.0, 1.0 - overlap / max(1, sector[2] * sector[3])))
        return values

    def _sector_labels(self) -> List[str]:
        if self.sector_count == 3:
            return ["left", "center", "right"]
        if self.sector_count == 5:
            return ["far_left", "left", "center", "right", "far_right"]
        return [f"sector_{index}" for index in range(self.sector_count)]

    def _update_motion(
        self,
        candidate: Optional[ObstacleCandidate],
        frame_width: int,
        frame_height: int,
        timestamp: float,
    ) -> Tuple[float, Optional[float]]:
        if candidate is None:
            self._previous_signature = None
            self._previous_at = timestamp
            return 0.0, None
        signature = (candidate.center[0] / max(1, frame_width), candidate.center[1] / max(1, frame_height), candidate.area_ratio)
        motion_score = 0.0
        ttc_seconds = None
        if self._previous_signature is not None and self._previous_at is not None:
            elapsed = max(0.001, timestamp - self._previous_at)
            position_delta = abs(signature[0] - self._previous_signature[0]) + abs(signature[1] - self._previous_signature[1])
            growth_rate = (signature[2] - self._previous_signature[2]) / elapsed
            motion_score = min(1.0, position_delta / elapsed)
            if growth_rate > 0.0001:
                ttc_seconds = max(0.05, min(60.0, signature[2] / growth_rate))
        self._previous_signature = signature
        self._previous_at = timestamp
        return motion_score, ttc_seconds

    def _candidate_score(self, candidate: ObstacleCandidate) -> float:
        return candidate.area_ratio * 0.6 + candidate.risk_zone_coverage * 0.4

    @staticmethod
    def _confidence(area_ratio: float, coverage: float) -> float:
        return max(0.0, min(1.0, area_ratio * 3.0 + coverage * 0.7))

    @staticmethod
    def _side(center_x: int, frame_width: int) -> str:
        if center_x < frame_width / 3:
            return "left"
        if center_x > frame_width * 2 / 3:
            return "right"
        return "center"

    @staticmethod
    def _valid_frame(frame: Any) -> bool:
        return frame is not None and isinstance(frame, np.ndarray) and frame.size > 0 and frame.ndim >= 2

    @staticmethod
    def _intersect_boxes(first: Box, second: Box) -> Optional[Box]:
        ax, ay, aw, ah = first
        bx, by, bw, bh = second
        x1, y1 = max(ax, bx), max(ay, by)
        x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        if x2 <= x1 or y2 <= y1:
            return None
        return (x1, y1, x2 - x1, y2 - y1)

    @staticmethod
    def _box_area(box: Optional[Box]) -> int:
        return 0 if box is None else max(0, int(box[2])) * max(0, int(box[3]))

    @staticmethod
    def _clamp_float(value: float, lower: float, upper: float, default: float) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            numeric = default
        return max(lower, min(upper, numeric))

    @staticmethod
    def _positive_int(value: int, default: int) -> int:
        try:
            numeric = int(value)
        except (TypeError, ValueError):
            numeric = default
        return max(1, numeric)

    @staticmethod
    def _config_float(config: Dict[str, object], key: str, default: float) -> float:
        try:
            return float(config.get(key, default))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _config_int(config: Dict[str, object], key: str, default: int) -> int:
        try:
            return int(config.get(key, default))
        except (TypeError, ValueError):
            return default


def _import_cv2():
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("缺少 opencv-python 依赖：请先安装 requirements.txt。") from exc
    return cv2
