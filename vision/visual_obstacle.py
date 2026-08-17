"""Visual-only risk advisory based on the already-detected scene.

This layer consumes the camera scene returned by the ReID detector and never
invokes a second model pass. Its output is advisory only and may never override
an authoritative infrared safety conclusion.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, Optional, Tuple


@dataclass(frozen=True)
class VisualObstacleCandidate:
    label: str
    class_name: str
    bbox: Tuple[int, int, int, int]
    confidence: float
    zone: str
    area_ratio: float
    approach_score: float
    is_centered: bool


@dataclass(frozen=True)
class VisualObstacleRisk:
    state: str
    candidates: Tuple[VisualObstacleCandidate, ...]
    primary_candidate: Optional[VisualObstacleCandidate]
    confidence: float
    reason: str


def classify_zone(
    center_x: int,
    frame_width: int,
    center_zone_ratio: float = 0.30,
) -> str:
    normalized_ratio = max(0.10, min(0.90, float(center_zone_ratio)))
    half_center_width = float(frame_width) * normalized_ratio / 2.0
    left_limit = float(frame_width) / 2.0 - half_center_width
    right_limit = float(frame_width) / 2.0 + half_center_width
    if center_x < left_limit:
        return "left"
    if center_x > right_limit:
        return "right"
    return "center"


class VisualObstacleAdvisor:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config or {}
        self.enabled = bool(self.config.get("visual_assist_enabled", True))
        self.confidence_threshold = float(
            self.config.get("visual_assist_confidence", 0.35)
        )
        self.center_zone_ratio = float(
            self.config.get("visual_assist_center_zone_ratio", 0.30)
        )
        self.approach_frames = max(
            2, int(self.config.get("visual_assist_approach_frames", 3))
        )
        self.approach_growth_ratio = max(
            0.0, float(self.config.get("visual_assist_approach_growth_ratio", 0.08))
        )
        self._area_history: Dict[Tuple[str, str], Deque[float]] = {}

    @property
    def history_size(self) -> int:
        return sum(len(values) for values in self._area_history.values())

    def reset(self) -> None:
        self._area_history.clear()

    def evaluate(self, frame: Any, target_result: Dict[str, Any]) -> VisualObstacleRisk:
        if not self.enabled:
            self.reset()
            return self._empty_risk("visual obstacle assistance is disabled")

        visual_objects = list(target_result.get("visual_objects") or [])
        if not visual_objects:
            self.reset()
            if target_result.get("found"):
                return VisualObstacleRisk(
                    state="TARGET_PERSON_ONLY",
                    candidates=(),
                    primary_candidate=None,
                    confidence=0.0,
                    reason=(
                        "target person is visible; no non-target obstacle "
                        "candidates detected"
                    ),
                )
            return self._empty_risk("no visual obstacle candidates in frame")

        shape = getattr(frame, "shape", None)
        if isinstance(shape, (tuple, list)) and len(shape) >= 2:
            frame_height = max(1, int(shape[0]))
            frame_width = max(1, int(shape[1]))
        else:
            frame_height = 480
            frame_width = 640

        candidates = []
        visible_keys = set()
        for item in visual_objects:
            bbox = item.get("bbox_xyxy") or item.get("bbox")
            if not isinstance(bbox, (tuple, list)) or len(bbox) != 4:
                continue
            x1, y1, x2, y2 = (float(value) for value in bbox)
            width = max(1.0, x2 - x1)
            height = max(1.0, y2 - y1)
            center_x = int(round((x1 + x2) / 2.0))
            area_ratio = (width * height) / float(frame_width * frame_height)
            zone = classify_zone(center_x, frame_width, self.center_zone_ratio)
            confidence = float(item.get("confidence", 0.0))
            if confidence < self.confidence_threshold:
                continue

            class_name = str(item.get("class_name") or "unknown")
            key = (class_name, zone)
            visible_keys.add(key)
            history = self._area_history.setdefault(
                key, deque(maxlen=self.approach_frames)
            )
            history.append(max(0.0, float(area_ratio)))
            approach_score = self._approach_score(history)
            candidates.append(
                VisualObstacleCandidate(
                    label=str(
                        item.get("display_label")
                        or item.get("class_name")
                        or "障碍物候选"
                    ),
                    class_name=class_name,
                    bbox=(int(x1), int(y1), int(x2), int(y2)),
                    confidence=confidence,
                    zone=zone,
                    area_ratio=max(0.0, float(area_ratio)),
                    approach_score=approach_score,
                    is_centered=zone == "center",
                )
            )

        for key in tuple(self._area_history):
            if key not in visible_keys:
                del self._area_history[key]

        if not candidates:
            self.reset()
            return self._empty_risk("visual obstacle confidence below threshold")

        primary = max(
            candidates,
            key=lambda item: (
                item.is_centered,
                item.approach_score,
                item.area_ratio,
                item.confidence,
            ),
        )
        if not primary.is_centered:
            return VisualObstacleRisk(
                state="SIDE_OBJECT",
                candidates=tuple(candidates),
                primary_candidate=primary,
                confidence=primary.confidence,
                reason=f"visual obstacle detected on the {primary.zone} side",
            )
        if primary.approach_score > 0.0:
            return VisualObstacleRisk(
                state="APPROACHING_OBJECT",
                candidates=tuple(candidates),
                primary_candidate=primary,
                confidence=primary.confidence,
                reason="central visual obstacle has grown across consecutive frames",
            )
        return VisualObstacleRisk(
            state="CENTER_OBJECT",
            candidates=tuple(candidates),
            primary_candidate=primary,
            confidence=primary.confidence,
            reason="central obstacle candidate is visible",
        )

    def _approach_score(self, history: Deque[float]) -> float:
        if len(history) < self.approach_frames:
            return 0.0
        values = list(history)
        growing = all(
            current >= previous * (1.0 + self.approach_growth_ratio)
            for previous, current in zip(values, values[1:])
        )
        if not growing or values[0] <= 1e-12:
            return 0.0
        return max(0.0, (values[-1] - values[0]) / values[0])

    @staticmethod
    def _empty_risk(reason: str) -> VisualObstacleRisk:
        return VisualObstacleRisk(
            state="CLEAR",
            candidates=(),
            primary_candidate=None,
            confidence=0.0,
            reason=reason,
        )
