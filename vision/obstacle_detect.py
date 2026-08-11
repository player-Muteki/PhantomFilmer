"""Visual obstacle-risk detection for the follow pipeline."""

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np


Box = Tuple[int, int, int, int]


@dataclass
class ObstacleResult:
    """Obstacle-risk result for one frame."""

    found: bool = False
    state: str = "CLEAR"
    center: Optional[Tuple[int, int]] = None
    bbox: Optional[Box] = None
    area: float = 0.0
    area_ratio: float = 0.0
    risk_zone: Optional[Box] = None
    risk_zone_coverage: float = 0.0
    side: str = "none"


class ObstacleDetector:
    """Detect non-target visual obstacles inside a configurable forward risk zone."""

    def __init__(
        self,
        risk_zone_width_ratio: float = 0.55,
        risk_zone_height_ratio: float = 0.65,
        risk_zone_vertical_offset_ratio: float = 0.05,
        minimum_obstacle_area: int = 1500,
        caution_area_ratio: float = 0.08,
        blocked_area_ratio: float = 0.18,
    ) -> None:
        self.risk_zone_width_ratio = self._clamp_float(risk_zone_width_ratio, 0.05, 1.0, 0.55)
        self.risk_zone_height_ratio = self._clamp_float(risk_zone_height_ratio, 0.05, 1.0, 0.65)
        self.risk_zone_vertical_offset_ratio = self._clamp_float(
            risk_zone_vertical_offset_ratio, -0.5, 0.5, 0.05
        )
        self.minimum_obstacle_area = self._positive_int(minimum_obstacle_area, 1500)
        self.caution_area_ratio = self._clamp_float(caution_area_ratio, 0.0, 1.0, 0.08)
        self.blocked_area_ratio = self._clamp_float(
            blocked_area_ratio, self.caution_area_ratio, 1.0, 0.18
        )
        self.last_result = ObstacleResult()
        self.last_mask = None

    @classmethod
    def from_config(cls, config: Dict[str, object]) -> "ObstacleDetector":
        """Build an obstacle detector from config.yaml."""
        obstacle = config.get("obstacle", {}) if isinstance(config, dict) else {}
        if not isinstance(obstacle, dict):
            obstacle = {}
        return cls(
            risk_zone_width_ratio=cls._config_float(obstacle, "risk_zone_width_ratio", 0.55),
            risk_zone_height_ratio=cls._config_float(obstacle, "risk_zone_height_ratio", 0.65),
            risk_zone_vertical_offset_ratio=cls._config_float(
                obstacle, "risk_zone_vertical_offset_ratio", 0.05
            ),
            minimum_obstacle_area=cls._config_int(obstacle, "minimum_obstacle_area", 1500),
            caution_area_ratio=cls._config_float(obstacle, "caution_area_ratio", 0.08),
            blocked_area_ratio=cls._config_float(obstacle, "blocked_area_ratio", 0.18),
        )

    def detect(self, frame: Any, target_result: Dict[str, object]) -> ObstacleResult:
        """Return the highest-risk non-target obstacle candidate in one frame."""
        if not self._valid_frame(frame):
            self.last_mask = None
            self.last_result = ObstacleResult()
            return self.last_result

        cv2 = _import_cv2()
        frame_height, frame_width = frame.shape[:2]
        frame_area = max(1, frame_width * frame_height)
        risk_zone = self._risk_zone(frame_width, frame_height)
        risk_area = max(1, risk_zone[2] * risk_zone[3])

        risk_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
        x, y, w, h = risk_zone
        risk_mask[y:y + h, x:x + w] = 255
        self._erase_target_region(risk_mask, target_result)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        edges = cv2.bitwise_and(edges, risk_mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(edges, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        self.last_mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.minimum_obstacle_area:
                continue
            bbox = cv2.boundingRect(contour)
            risk_overlap = self._box_area(self._intersect_boxes(bbox, risk_zone))
            if risk_overlap <= 0:
                continue
            area_ratio = area / frame_area
            coverage = risk_overlap / risk_area
            score = area_ratio * 0.6 + coverage * 0.4
            if score > best_score:
                best_score = score
                best = (bbox, area, area_ratio, coverage)

        if best is None:
            self.last_result = ObstacleResult(risk_zone=risk_zone)
            return self.last_result

        bbox, area, area_ratio, coverage = best
        bx, by, bw, bh = bbox
        center = (bx + bw // 2, by + bh // 2)
        state = "BLOCKED" if area_ratio >= self.blocked_area_ratio else "CAUTION"
        side = self._side(center[0], frame_width)
        self.last_result = ObstacleResult(
            found=True,
            state=state,
            center=center,
            bbox=bbox,
            area=area,
            area_ratio=area_ratio,
            risk_zone=risk_zone,
            risk_zone_coverage=coverage,
            side=side,
        )
        return self.last_result

    def draw_debug(self, frame: Any, result: Optional[ObstacleResult]) -> Any:
        """Draw risk-zone and obstacle overlays on a frame."""
        if not self._valid_frame(frame):
            return frame
        cv2 = _import_cv2()
        debug = frame.copy()
        result = result or self.last_result
        if result.risk_zone is not None:
            x, y, w, h = result.risk_zone
            cv2.rectangle(debug, (x, y), (x + w, y + h), (255, 180, 0), 2)
        if result.found and result.bbox is not None:
            x, y, w, h = result.bbox
            color = (0, 0, 255) if result.state == "BLOCKED" else (0, 165, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            if result.center is not None:
                cv2.circle(debug, result.center, 5, color, -1)
            cv2.putText(
                debug,
                f"OBS {result.state} {result.side} {result.area_ratio:.3f}",
                (max(0, x), max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
        return debug

    def reset(self) -> None:
        """Clear cached detection state."""
        self.last_result = ObstacleResult()
        self.last_mask = None

    def _risk_zone(self, frame_width: int, frame_height: int) -> Box:
        zone_width = max(1, int(frame_width * self.risk_zone_width_ratio))
        zone_height = max(1, int(frame_height * self.risk_zone_height_ratio))
        x = max(0, min(frame_width - zone_width, (frame_width - zone_width) // 2))
        y_center = frame_height // 2 + int(frame_height * self.risk_zone_vertical_offset_ratio)
        y = max(0, min(frame_height - zone_height, y_center - zone_height // 2))
        return (x, y, zone_width, zone_height)

    def _erase_target_region(self, mask: Any, target_result: Dict[str, object]) -> None:
        cv2 = _import_cv2()
        bbox = target_result.get("bbox") if isinstance(target_result, dict) else None
        if bbox is not None:
            x, y, w, h = bbox  # type: ignore[misc]
            pad = max(8, int(max(w, h) * 0.18))
            x1 = max(0, int(x) - pad)
            y1 = max(0, int(y) - pad)
            x2 = min(mask.shape[1], int(x + w) + pad)
            y2 = min(mask.shape[0], int(y + h) + pad)
            mask[y1:y2, x1:x2] = 0

        corners = target_result.get("corners") if isinstance(target_result, dict) else None
        if corners is not None:
            points = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(mask, [points], 0)

    @staticmethod
    def _side(center_x: int, frame_width: int) -> str:
        left_limit = frame_width / 3
        right_limit = frame_width * 2 / 3
        if center_x < left_limit:
            return "left"
        if center_x > right_limit:
            return "right"
        return "center"

    @staticmethod
    def _valid_frame(frame: Any) -> bool:
        return frame is not None and isinstance(frame, np.ndarray) and frame.size > 0 and frame.ndim >= 2

    @staticmethod
    def _intersect_boxes(first: Box, second: Box) -> Optional[Box]:
        ax, ay, aw, ah = first
        bx, by, bw, bh = second
        x1 = max(ax, bx)
        y1 = max(ay, by)
        x2 = min(ax + aw, bx + bw)
        y2 = min(ay + ah, by + bh)
        if x2 <= x1 or y2 <= y1:
            return None
        return (x1, y1, x2 - x1, y2 - y1)

    @staticmethod
    def _box_area(box: Optional[Box]) -> int:
        if box is None:
            return 0
        return max(0, int(box[2])) * max(0, int(box[3]))

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
