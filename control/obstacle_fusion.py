"""Authoritative front-ToF and visual obstacle fusion contracts.

This module sits between the raw sensor snapshots and the legacy flight planner
so the motion arbiter can reason about a single fused risk state instead of
stringing together ad hoc sensor fields.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from drone.front_tof import FrontToFSnapshot
from vision.visual_obstacle import (
    VisualObstacleAdvisor,
    VisualObstacleCandidate,
    VisualObstacleRisk,
)

__all__ = [
    "InfraredObservation",
    "VisualObstacleCandidate",
    "VisualObstacleRisk",
    "FusedObstacleState",
    "InfraredArraySnapshot",
    "InfraredArrayProvider",
    "normalized_obstacle_config",
    "classify_infrared_snapshot",
    "ObstacleFusionEngine",
    "choose_bypass_direction",
    "score_distance",
    "VisualObstacleAdvisor",
]

INFRARED_STATES = {"VALID", "OUT_OF_RANGE", "STALE", "ERROR", "NOT_READY"}
VISUAL_STATES = {
    "CLEAR",
    "SIDE_OBJECT",
    "CENTER_OBJECT",
    "APPROACHING_OBJECT",
    "TARGET_PERSON_ONLY",
    "UNKNOWN",
}


@dataclass(frozen=True)
class InfraredObservation:
    state: str
    distance_cm: Optional[float]
    status: str
    age_seconds: Optional[float]
    sequence: int
    is_fresh: bool
    is_safe_to_advance: bool


@dataclass(frozen=True)
class FusedObstacleState:
    state: str
    risk_level: str
    primary_source: str
    infrared: InfraredObservation
    visual: VisualObstacleRisk
    distance_cm: Optional[float]
    recommended_direction: str
    forward_speed_limit: Optional[int]
    confidence: float
    reason: str


@dataclass(frozen=True)
class InfraredArraySnapshot:
    front_left_cm: Optional[float]
    front_center_cm: Optional[float]
    front_right_cm: Optional[float]
    left_status: str
    center_status: str
    right_status: str
    sequence: int
    age_seconds: float


class InfraredArrayProvider:
    def snapshot(self) -> InfraredArraySnapshot:
        raise NotImplementedError


def normalized_obstacle_config(config: Dict[str, Any]) -> Dict[str, Any]:
    raw_obstacle = config.get("obstacle", {}) if isinstance(config, dict) else {}
    obstacle = dict(raw_obstacle) if isinstance(raw_obstacle, dict) else {}

    blocked_distance = float(obstacle.get("front_tof_blocked_distance_cm", 60.0))
    caution_distance = float(
        obstacle.get("front_tof_caution_distance_cm", max(blocked_distance, 100.0))
    )
    obstacle.setdefault(
        "front_tof_enabled", bool(obstacle.get("front_tof_enabled", False))
    )
    obstacle.setdefault("front_tof_blocked_distance_cm", blocked_distance)
    obstacle.setdefault(
        "front_tof_caution_distance_cm", max(caution_distance, blocked_distance)
    )
    obstacle.setdefault(
        "front_tof_clear_distance_cm",
        float(obstacle.get("front_tof_clear_distance_cm", 70.0)),
    )
    obstacle.setdefault(
        "front_tof_clear_confirm_samples",
        int(
            obstacle.get(
                "front_tof_clear_confirm_samples",
                obstacle.get("clear_confirm_frames", 5),
            )
        ),
    )
    obstacle.setdefault(
        "front_tof_failure_limit",
        int(
            obstacle.get(
                "front_tof_failure_limit", obstacle.get("lost_tof_failure_limit", 5)
            )
        ),
    )
    obstacle.setdefault(
        "visual_assist_enabled", bool(obstacle.get("visual_assist_enabled", True))
    )
    obstacle.setdefault(
        "visual_assist_confidence",
        float(obstacle.get("visual_assist_confidence", 0.35)),
    )
    obstacle.setdefault(
        "visual_assist_center_zone_ratio",
        float(obstacle.get("visual_assist_center_zone_ratio", 0.30)),
    )
    obstacle.setdefault(
        "visual_assist_approach_frames",
        int(obstacle.get("visual_assist_approach_frames", 3)),
    )
    obstacle.setdefault(
        "visual_assist_approach_growth_ratio",
        float(obstacle.get("visual_assist_approach_growth_ratio", 0.08)),
    )
    obstacle.setdefault(
        "visual_assist_forward_speed_ratio",
        float(obstacle.get("visual_assist_forward_speed_ratio", 0.40)),
    )
    obstacle.setdefault(
        "visual_assist_can_clear_infrared",
        bool(obstacle.get("visual_assist_can_clear_infrared", False)),
    )
    obstacle.setdefault(
        "center_loss_forward_enabled",
        bool(obstacle.get("center_loss_forward_enabled", False)),
    )
    obstacle.setdefault(
        "ir_probe_enabled", bool(obstacle.get("ir_probe_enabled", True))
    )
    obstacle.setdefault("ir_probe_speed", int(obstacle.get("ir_probe_speed", 8)))
    obstacle.setdefault(
        "ir_probe_pulse_seconds", float(obstacle.get("ir_probe_pulse_seconds", 0.25))
    )
    obstacle.setdefault(
        "ir_probe_max_attempts", int(obstacle.get("ir_probe_max_attempts", 3))
    )
    obstacle.setdefault("timeout_action", str(obstacle.get("timeout_action", "land")))
    return obstacle


def classify_infrared_snapshot(
    snapshot: Optional[FrontToFSnapshot],
    *,
    caution_distance_cm: float,
    blocked_distance_cm: float,
    max_age_seconds: float,
) -> InfraredObservation:
    if snapshot is None:
        return InfraredObservation(
            state="NOT_READY",
            distance_cm=None,
            status="not_ready",
            age_seconds=None,
            sequence=0,
            is_fresh=False,
            is_safe_to_advance=False,
        )

    raw_status = str(snapshot.status).strip().lower()
    raw_distance = snapshot.distance_cm
    age = snapshot.age_seconds
    age_seconds = float(age) if age is not None else float("inf")
    state = "VALID"
    normalized_status = raw_status
    if raw_status in {"not_ready", "disabled"}:
        state = "NOT_READY"
    elif raw_status in {"error", "failed"}:
        state = "ERROR"
    elif raw_status == "stale" or age_seconds > max_age_seconds:
        state = "STALE"
    elif raw_status == "out_of_range":
        state = "OUT_OF_RANGE"
    elif raw_status not in {"valid", "ok"}:
        state = "ERROR"

    is_fresh = state in {"VALID", "OUT_OF_RANGE"} and age_seconds <= max_age_seconds
    is_safe_to_advance = (
        state == "OUT_OF_RANGE"
        or (
            state == "VALID"
            and raw_distance is not None
            and raw_distance > blocked_distance_cm
        )
    ) and age_seconds <= max_age_seconds
    return InfraredObservation(
        state=state,
        distance_cm=raw_distance,
        status=normalized_status,
        age_seconds=age,
        sequence=int(snapshot.sequence),
        is_fresh=is_fresh,
        is_safe_to_advance=is_safe_to_advance,
    )


class ObstacleFusionEngine:
    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = normalized_obstacle_config(config or {})
        self.blocked_distance_cm = float(
            self.config.get("front_tof_blocked_distance_cm", 60.0)
        )
        self.caution_distance_cm = float(
            self.config.get(
                "front_tof_caution_distance_cm", max(self.blocked_distance_cm, 100.0)
            )
        )
        self.max_age_seconds = float(self.config.get("front_tof_max_age_seconds", 0.8))

    def fuse(
        self,
        infrared: InfraredObservation,
        visual: VisualObstacleRisk,
    ) -> FusedObstacleState:
        if infrared.state in {"STALE", "ERROR", "NOT_READY"}:
            return FusedObstacleState(
                state="IR_UNKNOWN",
                risk_level="high",
                primary_source="infrared",
                infrared=infrared,
                visual=visual,
                distance_cm=infrared.distance_cm,
                recommended_direction="unknown",
                forward_speed_limit=0,
                confidence=0.0,
                reason="infrared sensor is stale, unavailable, or not valid enough to authorize forward motion",
            )

        if infrared.distance_cm is not None:
            if infrared.distance_cm <= self.blocked_distance_cm:
                return FusedObstacleState(
                    state="IR_BLOCKED",
                    risk_level="high",
                    primary_source="infrared",
                    infrared=infrared,
                    visual=visual,
                    distance_cm=infrared.distance_cm,
                    recommended_direction="right",
                    forward_speed_limit=0,
                    confidence=1.0,
                    reason="front infrared distance below blocked threshold",
                )
            if infrared.distance_cm <= self.caution_distance_cm:
                return FusedObstacleState(
                    state="IR_CAUTION",
                    risk_level="medium",
                    primary_source="infrared",
                    infrared=infrared,
                    visual=visual,
                    distance_cm=infrared.distance_cm,
                    recommended_direction="right",
                    forward_speed_limit=0,
                    confidence=max(
                        0.0,
                        min(
                            1.0,
                            1.0
                            - (
                                infrared.distance_cm
                                / max(self.caution_distance_cm, 1.0)
                            ),
                        ),
                    ),
                    reason="front infrared distance in caution band",
                )

        if visual.state in {"APPROACHING_OBJECT", "CENTER_OBJECT"}:
            return FusedObstacleState(
                state="VISUAL_CAUTION",
                risk_level="medium",
                primary_source="visual",
                infrared=infrared,
                visual=visual,
                distance_cm=infrared.distance_cm,
                recommended_direction="none",
                forward_speed_limit=None,
                confidence=visual.confidence,
                reason=visual.reason,
            )

        return FusedObstacleState(
            state="CLEAR",
            risk_level="none",
            primary_source="none",
            infrared=infrared,
            visual=visual,
            distance_cm=infrared.distance_cm,
            recommended_direction="none",
            forward_speed_limit=None,
            confidence=0.0,
            reason="no active infrared or visual obstruction",
        )


def choose_bypass_direction(
    infrared: InfraredArraySnapshot,
    visual: VisualObstacleRisk,
) -> str:
    left_score = score_distance(infrared.front_left_cm)
    right_score = score_distance(infrared.front_right_cm)
    if visual.state in {"SIDE_OBJECT"}:
        left_score -= (
            0.2
            if visual.primary_candidate and visual.primary_candidate.zone == "left"
            else 0.0
        )
        right_score -= (
            0.2
            if visual.primary_candidate and visual.primary_candidate.zone == "right"
            else 0.0
        )
    if max(left_score, right_score) < 0.05:
        return "unknown"
    return "left" if left_score > right_score else "right"


def score_distance(distance_cm: Optional[float]) -> float:
    if distance_cm is None:
        return 0.0
    if distance_cm <= 0:
        return 0.0
    return max(0.0, min(1.0, distance_cm / 200.0))
