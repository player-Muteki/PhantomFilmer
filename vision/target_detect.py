"""OpenCV red target detection for umbrella-following experiments."""

from typing import Any, Dict, Optional

import numpy as np


DetectionResult = Dict[str, Optional[object]]


class TargetDetector:
    """Detect a red target block using simple OpenCV color thresholding.

    Project frames are expected to be OpenCV BGR images.
    """

    def __init__(self, min_area: int = 500, min_area_ratio: float = 0.002) -> None:
        self.min_area = min_area
        self.min_area_ratio = max(0.0, float(min_area_ratio))
        self.last_mask = None

    @classmethod
    def from_config(cls, config: Dict[str, object]) -> "TargetDetector":
        """Build a detector from config.yaml with safe defaults."""
        try:
            min_area_ratio = float(config.get("detector_min_area_ratio", 0.002))
        except (TypeError, ValueError):
            min_area_ratio = 0.002
        try:
            min_area = int(config.get("detector_min_area", 500))
        except (TypeError, ValueError):
            min_area = 500
        return cls(min_area=min_area, min_area_ratio=min_area_ratio)

    def detect(self, frame: Any) -> DetectionResult:
        """Detect a red target and return found, center, area, and bbox."""
        cv2 = _import_cv2()
        if frame is None:
            self.last_mask = None
            return self._empty_result()

        mask = self.create_red_mask(frame)
        self.last_mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return self._empty_result()

        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        frame_height, frame_width = frame.shape[:2]
        dynamic_min_area = max(
            float(self.min_area),
            float(frame_width * frame_height) * self.min_area_ratio,
        )
        if area < dynamic_min_area:
            return self._empty_result(area=area)

        x, y, w, h = cv2.boundingRect(largest)
        center = (x + w // 2, y + h // 2)
        return {
            "found": True,
            "is_predicted": False,
            "center": center,
            "target_center_x": center[0],
            "target_center_y": center[1],
            "area": area,
            "bbox": (x, y, w, h),
        }

    def create_red_mask(self, frame: Any) -> Any:
        """Create a denoised mask for vivid red regions from a BGR frame."""
        cv2 = _import_cv2()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([8, 255, 255])
        lower_red2 = np.array([172, 120, 120])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        hsv_mask = mask1 | mask2

        blue = frame[:, :, 0].astype(np.float32)
        green = frame[:, :, 1].astype(np.float32)
        red = frame[:, :, 2].astype(np.float32)
        red_dominance = (
            (red > 120)
            & (red > green * 1.45)
            & (red > blue * 1.70)
            & ((red - green) > 45)
        ).astype(np.uint8) * 255
        mask = cv2.bitwise_and(hsv_mask, red_dominance)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def draw_debug(self, frame: Any, result: DetectionResult) -> Any:
        """Draw target box, target center, frame center, area, and found status."""
        cv2 = _import_cv2()
        if frame is None:
            return frame

        debug_frame = frame.copy()
        height, width = debug_frame.shape[:2]
        frame_center = (width // 2, height // 2)

        cv2.line(debug_frame, (frame_center[0], 0), (frame_center[0], height), (255, 0, 0), 1)
        cv2.line(debug_frame, (0, frame_center[1]), (width, frame_center[1]), (255, 0, 0), 1)
        cv2.circle(debug_frame, frame_center, 6, (255, 0, 0), -1)
        cv2.putText(
            debug_frame,
            "frame center",
            (frame_center[0] + 8, frame_center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1,
        )

        found = bool(result.get("found"))
        area = float(result.get("area") or 0.0)
        if found:
            bbox = result.get("bbox")
            center = result.get("center")
            if bbox is not None:
                x, y, w, h = bbox  # type: ignore[misc]
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            if center is not None:
                cx, cy = center  # type: ignore[misc]
                cv2.circle(debug_frame, (cx, cy), 6, (0, 255, 255), -1)
                cv2.line(debug_frame, frame_center, (int(cx), int(cy)), (0, 255, 255), 2)

        cv2.rectangle(debug_frame, (12, 12), (280, 76), (0, 0, 0), -1)
        cv2.putText(
            debug_frame,
            f"found: {found}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0) if found else (0, 0, 255),
            2,
        )
        cv2.putText(
            debug_frame,
            f"area: {area:.1f}",
            (20, 63),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        return debug_frame

    def _empty_result(self, area: float = 0.0) -> DetectionResult:
        """Return the standard empty detection result."""
        return {
            "found": False,
            "is_predicted": False,
            "center": None,
            "target_center_x": None,
            "target_center_y": None,
            "area": area,
            "bbox": None,
        }


def _import_cv2():
    """Import OpenCV only when image processing is actually used."""
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("缺少 opencv-python 依赖：请先安装 requirements.txt。") from exc
    return cv2


ColorTargetDetector = TargetDetector
