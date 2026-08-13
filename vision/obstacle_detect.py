"""Distance-only obstacle data contract and neutral frame adapter.

Obstacle decisions are produced exclusively from the RoboMaster TT top/front
ToF sensor. Camera frames are accepted only to preserve the shared motion
pipeline interface; no pixels are analysed for obstacle detection.
"""

from dataclasses import dataclass, field
from time import monotonic
from typing import Any, Dict, List, Optional, Tuple


Box = Tuple[int, int, int, int]


@dataclass
class ObstacleCandidate:
    """Legacy log DTO retained for schema compatibility; never camera-generated."""

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
            "area": round(self.area, 2),
            "area_ratio": round(self.area_ratio, 6),
            "risk_zone_coverage": round(self.risk_zone_coverage, 6),
            "side": self.side,
            "confidence": round(self.confidence, 4),
            "motion_score": round(self.motion_score, 4),
            "ttc_seconds": self.ttc_seconds,
        }


@dataclass
class ObstacleResult:
    """One obstacle observation consumed by the existing safety planner."""

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
    front_distance_cm: Optional[float] = None
    front_distance_status: str = "disabled"
    front_distance_age_seconds: Optional[float] = None

    def to_observation(self, frame_width: int, frame_height: int) -> Dict[str, object]:
        return {
            "frame": {"index": self.frame_index, "width": frame_width, "height": frame_height},
            "found": self.found,
            "state": self.state,
            "center": None if self.center is None else list(self.center),
            "bbox": None if self.bbox is None else list(self.bbox),
            "area": round(self.area, 2),
            "area_ratio": round(self.area_ratio, 6),
            "risk_zone": None,
            "risk_zone_coverage": 0.0,
            "side": self.side,
            "obstacles": [],
            "free_space": {},
            "confidence": round(self.confidence, 4),
            "motion_score": 0.0,
            "ttc_seconds": None,
            "frame_size": None if self.frame_size is None else list(self.frame_size),
            "data_quality": self.data_quality,
            "consecutive_found_frames": self.consecutive_found_frames,
            "consecutive_clear_frames": self.consecutive_clear_frames,
            "front_distance_cm": self.front_distance_cm,
            "front_distance_status": self.front_distance_status,
            "front_distance_age_seconds": self.front_distance_age_seconds,
        }


class DistanceOnlyObstacleDetector:
    """Produce neutral CLEAR observations before front-ToF fusion."""

    def __init__(self) -> None:
        self.last_result = ObstacleResult()
        self._frame_index = 0
        self._clear_frames = 0

    def detect(self, frame: Any, target_result: Dict[str, object]) -> ObstacleResult:
        del target_result
        self._frame_index += 1
        self._clear_frames += 1
        frame_size = None
        shape = getattr(frame, "shape", None)
        if shape is not None and len(shape) >= 2:
            frame_size = (int(shape[1]), int(shape[0]))
        self.last_result = ObstacleResult(
            state="CLEAR",
            frame_size=frame_size,
            frame_index=self._frame_index,
            timestamp=monotonic(),
            consecutive_clear_frames=self._clear_frames,
        )
        return self.last_result

    def draw_debug(self, frame: Any, result: Optional[ObstacleResult]) -> Any:
        del result
        return frame

    def reset(self) -> None:
        self.last_result = ObstacleResult()
        self._frame_index = 0
        self._clear_frames = 0
