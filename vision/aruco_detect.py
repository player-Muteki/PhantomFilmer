"""ArUco marker detection for drone umbrella target tracking.

Provides an ArucoTargetDetector whose detect(frame) and draw_debug(frame, result)
interfaces are fully compatible with the existing TargetDetector, so that
FollowController, FollowSession, and the control pipeline can switch between
red-target and ArUco detection without any code changes.
"""

from typing import Any, Dict, Optional

import numpy as np


DetectionResult = Dict[str, Optional[object]]

# Supported ArUco dictionary names.
_SUPPORTED_DICTS = frozenset({
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
    "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
    "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
})


class ArucoTargetDetector:
    """Detect a specific ArUco marker for target following.

    Default configuration targets marker ID 23 from the DICT_4X4_50 dictionary.
    Features configurable coordinate/area smoothing, configurable temporary-loss
    tolerance (reporting found=True for the first N consecutive failed frames),
    and a ``reset()`` method for clearing internal state between tasks.
    """

    def __init__(
        self,
        dictionary_name: str = "DICT_4X4_50",
        target_marker_id: int = 23,
        smoothing_alpha: float = 0.30,
        temporary_lost_frames: int = 3,
        min_marker_area: float = 300.0,
    ) -> None:
        self.dictionary_name = dictionary_name
        self.target_marker_id = target_marker_id
        self.smoothing_alpha = max(0.0, min(1.0, float(smoothing_alpha)))
        self.temporary_lost_frames = max(0, int(temporary_lost_frames))
        self.min_marker_area = float(min_marker_area)

        # Smoothing state
        self._smooth_x: Optional[float] = None
        self._smooth_y: Optional[float] = None
        self._smooth_area: Optional[float] = None
        self._smooth_initialized = False

        # Temporary loss tracking
        self._lost_count = 0

        # Last-known valid values (reused during temp-lost windows)
        self._last_valid_center: Optional[tuple] = None
        self._last_valid_bbox: Optional[tuple] = None
        self._last_valid_area: float = 0.0

        # Lazy-initialized ArUco detector
        self._detector: Any = None

    # ------------------------------------------------------------------
    # Public interface (compatible with TargetDetector)
    # ------------------------------------------------------------------

    @classmethod
    def from_config(cls, config: Dict[str, object]) -> "ArucoTargetDetector":
        """Build a detector from config.yaml with safe defaults.

        Reads from a ``vision`` subsection when present, otherwise falls
        back to top-level keys.
        """
        if isinstance(config.get("vision"), dict):
            cfg = config["vision"]
        else:
            cfg = config

        def _safe(val, key, cast, default):
            try:
                return cast(cfg.get(key, default))
            except (TypeError, ValueError):
                return default

        return cls(
            dictionary_name=_safe(cfg, "aruco_dictionary", str, "DICT_4X4_50"),
            target_marker_id=_safe(cfg, "target_marker_id", int, 23),
            smoothing_alpha=_safe(cfg, "smoothing_alpha", float, 0.30),
            temporary_lost_frames=_safe(cfg, "temporary_lost_frames", int, 3),
            min_marker_area=_safe(cfg, "min_marker_area", float, 300.0),
        )

    def detect(self, frame: Any) -> DetectionResult:
        """Detect the target ArUco marker and return a compatible result dict.

        Returns standard fields (found, center, target_center_x/y, area, bbox)
        plus extra fields (marker_id, corners, detector_type).
        """
        err = self._validate_frame(frame)
        if err:
            return err

        cv2 = _import_cv2()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

        # Lazy detector init
        if self._detector is None:
            self._init_detector(cv2)

        # --- detect ---
        try:
            corners, ids, _ = self._detector.detectMarkers(gray)
        except Exception:
            return self._handle_not_found()

        target_idx = self._find_target_marker(ids, corners)
        if target_idx is None:
            return self._handle_not_found()

        marker_corners = corners[target_idx][0]  # shape (4, 2)

        # Geometry
        cx = float(np.mean(marker_corners[:, 0]))
        cy = float(np.mean(marker_corners[:, 1]))
        area = float(cv2.contourArea(marker_corners.astype(np.float32)))

        if area < self.min_marker_area:
            return self._handle_not_found()

        # Bounding box from extreme corners
        x = int(np.min(marker_corners[:, 0]))
        y = int(np.min(marker_corners[:, 1]))
        w = int(np.max(marker_corners[:, 0]) - x)
        h = int(np.max(marker_corners[:, 1]) - y)
        bbox = (x, y, w, h)

        # Smoothing
        sx, sy, sa = self._apply_smoothing(cx, cy, area)

        self._lost_count = 0
        center = (int(round(sx)), int(round(sy)))
        self._last_valid_center = center
        self._last_valid_bbox = bbox
        self._last_valid_area = sa

        return {
            "found": True,
            "is_predicted": False,
            "center": center,
            "target_center_x": center[0],
            "target_center_y": center[1],
            "area": float(sa),
            "bbox": bbox,
            "marker_id": self.target_marker_id,
            "corners": marker_corners.tolist(),
            "detector_type": "aruco",
        }

    def draw_debug(self, frame: Any, result: DetectionResult) -> Any:
        """Draw debug overlay (quadrilateral, centre, ID, area, status)."""
        if not self._is_valid_draw_frame(frame):
            return frame

        cv2 = _import_cv2()

        debug = frame.copy()
        h, w = debug.shape[:2]
        fc = (w // 2, h // 2)

        # Crosshair
        cv2.line(debug, (fc[0], 0), (fc[0], h), (255, 0, 0), 1)
        cv2.line(debug, (0, fc[1]), (w, fc[1]), (255, 0, 0), 1)
        cv2.circle(debug, fc, 6, (255, 0, 0), -1)

        if bool(result.get("found")):
            self._draw_found_overlay(debug, result, fc, cv2)
        else:
            self._draw_lost_overlay(debug, cv2)

        return debug

    def reset(self) -> None:
        """Reset smoothing and loss state.  Safe to call between tasks."""
        self._smooth_x = None
        self._smooth_y = None
        self._smooth_area = None
        self._smooth_initialized = False
        self._lost_count = 0
        self._last_valid_center = None
        self._last_valid_bbox = None
        self._last_valid_area = 0.0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init_detector(self, cv2: Any) -> None:
        """Create the OpenCV ArUco detector (OpenCV >= 4.7 / 5.x API)."""
        dict_name = self.dictionary_name
        if dict_name not in _SUPPORTED_DICTS:
            dict_name = "DICT_4X4_50"
        const = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
        dictionary = cv2.aruco.getPredefinedDictionary(const)
        params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)

    def _validate_frame(self, frame: Any) -> Optional[DetectionResult]:
        """Return an empty result if the frame is invalid, else None."""
        if frame is None:
            return self._empty_result()
        if not isinstance(frame, np.ndarray):
            return self._empty_result()
        if frame.size == 0 or frame.ndim < 2:
            return self._empty_result()
        return None

    @staticmethod
    def _is_valid_draw_frame(frame: Any) -> bool:
        return (
            frame is not None
            and isinstance(frame, np.ndarray)
            and frame.size > 0
        )

    def _find_target_marker(
        self, ids: Optional[np.ndarray], corners: Any
    ) -> Optional[int]:
        """Return the index of the target marker in *corners*, or None."""
        if ids is None or len(ids) == 0:
            return None
        for i, mid in enumerate(ids.flatten()):
            if int(mid) == self.target_marker_id:
                return i
        return None

    def _apply_smoothing(
        self, cx: float, cy: float, area: float
    ) -> tuple:
        """Exponential moving-average smoothing.  First detection initialises."""
        if not self._smooth_initialized:
            self._smooth_x = cx
            self._smooth_y = cy
            self._smooth_area = area
            self._smooth_initialized = True
            return cx, cy, area
        a = self.smoothing_alpha
        self._smooth_x = (1 - a) * self._smooth_x + a * cx  # type: ignore[operator]
        self._smooth_y = (1 - a) * self._smooth_y + a * cy  # type: ignore[operator]
        self._smooth_area = (1 - a) * self._smooth_area + a * area  # type: ignore[operator]
        return self._smooth_x, self._smooth_y, self._smooth_area

    def _handle_not_found(self) -> DetectionResult:
        """Increment loss counter.  Return stale values or empty result."""
        self._lost_count += 1

        if self._lost_count <= self.temporary_lost_frames and self._smooth_initialized:
            return {
                "found": True,
                "is_predicted": True,
                "center": self._last_valid_center,
                "target_center_x": self._last_valid_center[0] if self._last_valid_center else None,
                "target_center_y": self._last_valid_center[1] if self._last_valid_center else None,
                "area": self._last_valid_area,
                "bbox": self._last_valid_bbox,
                "marker_id": self.target_marker_id,
                "corners": None,
                "detector_type": "aruco",
            }

        return self._empty_result()

    def _empty_result(self) -> DetectionResult:
        return {
            "found": False,
            "is_predicted": False,
            "center": None,
            "target_center_x": None,
            "target_center_y": None,
            "area": 0.0,
            "bbox": None,
            "marker_id": None,
            "corners": None,
            "detector_type": "aruco",
        }

    @staticmethod
    def _draw_found_overlay(
        debug: Any, result: DetectionResult, fc: tuple, cv2: Any
    ) -> None:
        """Draw detection-success overlay on *debug* (mutates in-place)."""
        corners_raw = result.get("corners")
        center = result.get("center")
        marker_id = result.get("marker_id")
        area_val = float(result.get("area") or 0.0)

        if corners_raw is not None:
            pts = np.array(corners_raw, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug, [pts], True, (0, 255, 0), 2)

        if center is not None:
            cx, cy = int(center[0]), int(center[1])
            cv2.circle(debug, (cx, cy), 6, (0, 255, 255), -1)
            cv2.line(debug, fc, (cx, cy), (0, 255, 255), 2)

        # Marker ID label near the marker's top-left corner
        if corners_raw is not None:
            pts = np.array(corners_raw)
            label_x = max(int(np.min(pts[:, 0])), 0)
            label_y = max(int(np.min(pts[:, 1])) - 10, 0)
            cv2.putText(
                debug, f"ID: {marker_id}",
                (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )

        # Info panel (upper-left)
        cv2.rectangle(debug, (12, 12), (285, 95), (0, 0, 0), -1)
        cv2.putText(
            debug, f"ARUCO: FOUND  ID: {marker_id}",
            (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
        )
        cv2.putText(
            debug, f"area: {area_val:.1f}",
            (20, 57), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
        )
        cx = result.get("target_center_x")
        cy = result.get("target_center_y")
        cv2.putText(
            debug, f"center: ({cx}, {cy})",
            (20, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
        )

    @staticmethod
    def _draw_lost_overlay(debug: Any, cv2: Any) -> None:
        """Draw detection-lost overlay (mutates in-place)."""
        cv2.rectangle(debug, (12, 12), (195, 48), (0, 0, 0), -1)
        cv2.putText(
            debug, "ARUCO: LOST",
            (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
        )


def _import_cv2():
    """Lazy import of OpenCV (same pattern as target_detect.py)."""
    try:
        import cv2  # noqa: F811
    except ModuleNotFoundError as exc:
        raise RuntimeError("缺少 opencv-python 依赖：请先安装 requirements.txt。") from exc
    return cv2
