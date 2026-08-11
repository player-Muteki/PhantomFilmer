"""Unified obstacle observation, local planning, and decision logging."""

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from queue import Full, Queue
from threading import Thread
from time import monotonic
from typing import Any, Dict, Optional
from uuid import uuid4

from control.follow_control import RCCommand
from control.obstacle_avoidance import AvoidanceDecision, ObstacleAvoidancePlanner
from vision.obstacle_detect import ObstacleDetector, ObstacleResult


@dataclass(frozen=True)
class MotionContext:
    """Facts the arbiter needs about one autonomous motion tick."""

    mode: str
    target_result: Dict[str, object]


class MotionArbiter:
    """Deep module for deterministic obstacle-aware command selection."""

    def __init__(
        self,
        detector: ObstacleDetector,
        planner: ObstacleAvoidancePlanner,
        config: Optional[Dict[str, object]] = None,
    ) -> None:
        self.detector = detector
        self.planner = planner
        self.config = config or {}
        obstacle = self.config.get("obstacle", {})
        self._obstacle_config = obstacle if isinstance(obstacle, dict) else {}
        self._log_enabled = bool(self._obstacle_config.get("log_enabled", False))
        self._log_dir = Path(str(self._obstacle_config.get("log_dir", "logs/avoidance")))
        self._log_every_n_frames = max(1, self._config_int("log_every_n_frames", 2))
        self._writer: Optional[_JsonlEventWriter] = None
        self.session_id = ""
        self.mode = "unknown"
        self.last_observation: Optional[ObstacleResult] = None
        self.last_decision: Optional[AvoidanceDecision] = None
        self.last_latency_ms = 0.0
        self._active = False

    def reset(self, mode: str = "unknown") -> None:
        """Start a fresh planning and logging session."""
        self.close()
        self.detector.reset()
        self.planner.reset()
        self.session_id = uuid4().hex
        self.mode = str(mode)
        self.last_observation = None
        self.last_decision = None
        self.last_latency_ms = 0.0
        self._active = True
        if self._log_enabled:
            self._writer = _JsonlEventWriter(self._session_log_path())

    def decide(
        self,
        desired_command: RCCommand,
        frame: Any,
        context: MotionContext,
    ) -> AvoidanceDecision:
        """Observe one frame and return a bounded deterministic command."""
        if not self._active:
            self.reset(context.mode)
        started_at = monotonic()
        try:
            observation = self.detector.detect(frame, context.target_result)
            decision = self.planner.plan(desired_command, observation)
        except Exception as exc:
            observation = ObstacleResult(
                state="UNKNOWN",
                data_quality="planner_error",
                timestamp=monotonic(),
            )
            decision = AvoidanceDecision(
                command=self._zero_command(),
                state="FAILSAFE",
                action="HOVER",
                reason=f"obstacle pipeline error: {type(exc).__name__}",
                confidence=0.0,
                plan_id="error",
                observation=observation,
            )
        self.last_latency_ms = (monotonic() - started_at) * 1000.0
        self.last_observation = observation
        self.last_decision = decision
        self._record(desired_command, context, observation, decision)
        return decision

    def close(self) -> None:
        """Flush the optional event writer without affecting flight output."""
        if self._writer is not None:
            self._writer.close()
        self._writer = None
        self._active = False

    @property
    def log_path(self) -> Optional[Path]:
        """Return the active session log path when logging is enabled."""
        return self._writer.path if self._writer is not None else None

    @property
    def is_active(self) -> bool:
        """Return whether a planning session has already been initialized."""
        return self._active

    @property
    def dropped_log_events(self) -> int:
        """Return the number of events dropped to protect the control loop."""
        return self._writer.dropped if self._writer is not None else 0

    def _record(
        self,
        desired_command: RCCommand,
        context: MotionContext,
        observation: ObstacleResult,
        decision: AvoidanceDecision,
    ) -> None:
        if self._writer is None:
            return
        important = observation.found or decision.state not in {"CLEAR", "RECOVERING"}
        if not important and observation.frame_index % self._log_every_n_frames:
            return
        frame_width, frame_height = self._frame_dimensions(observation, context.target_result)
        payload = {
            "schema_version": 1,
            "event": "obstacle_decision",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "monotonic_timestamp": round(observation.timestamp, 6),
            "session_id": self.session_id,
            "mode": context.mode,
            "latency_ms": round(self.last_latency_ms, 3),
            "target": self._target_payload(context.target_result, frame_width, frame_height),
            "observation": observation.to_observation(frame_width, frame_height),
            "desired_command": self._command_payload(desired_command),
            "decision": decision.to_dict(),
            "final_command": self._command_payload(decision.command),
        }
        self._writer.write(payload)

    def _zero_command(self) -> RCCommand:
        limited = self.planner.safety_manager.limit_rc_command(0, 0, 0, 0)
        return RCCommand(*limited)

    def _session_log_path(self) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        return self._log_dir / f"{stamp}-{self.session_id}.jsonl"

    def _config_int(self, key: str, default: int) -> int:
        try:
            return int(self._obstacle_config.get(key, default))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _frame_dimensions(
        observation: ObstacleResult,
        target_result: Dict[str, object],
    ) -> tuple[int, int]:
        if observation.frame_size is not None:
            return observation.frame_size
        frame_size = target_result.get("frame_size")
        if isinstance(frame_size, (list, tuple)) and len(frame_size) == 2:
            return max(1, int(frame_size[0])), max(1, int(frame_size[1]))
        return 1, 1

    @staticmethod
    def _target_payload(
        target: Dict[str, object],
        frame_width: int,
        frame_height: int,
    ) -> Dict[str, object]:
        center = target.get("center")
        bbox = target.get("bbox")
        payload: Dict[str, object] = {
            "found": bool(target.get("found")),
            "center": None if center is None else list(center),  # type: ignore[arg-type]
            "bbox": None if bbox is None else list(bbox),  # type: ignore[arg-type]
            "area": round(float(target.get("area") or 0.0), 2),
        }
        if center is not None:
            payload["center_norm"] = [
                round(float(center[0]) / max(1, frame_width), 4),  # type: ignore[index]
                round(float(center[1]) / max(1, frame_height), 4),  # type: ignore[index]
            ]
        if bbox is not None:
            payload["bbox_norm"] = [
                round(float(bbox[0]) / max(1, frame_width), 4),  # type: ignore[index]
                round(float(bbox[1]) / max(1, frame_height), 4),  # type: ignore[index]
                round(float(bbox[2]) / max(1, frame_width), 4),  # type: ignore[index]
                round(float(bbox[3]) / max(1, frame_height), 4),  # type: ignore[index]
            ]
        return payload

    @staticmethod
    def _command_payload(command: RCCommand) -> Dict[str, int]:
        return {
            "left_right": command.left_right,
            "forward_backward": command.forward_backward,
            "up_down": command.up_down,
            "yaw": command.yaw,
        }


class _JsonlEventWriter:
    """Bounded background writer; dropped logs never delay control."""

    def __init__(self, path: Path, queue_size: int = 256) -> None:
        self.path = path
        self.dropped = 0
        self._queue: Queue[Optional[Dict[str, object]]] = Queue(maxsize=queue_size)
        self._thread = Thread(target=self._run, name="ObstacleEventWriter", daemon=True)
        self._thread.start()

    def write(self, payload: Dict[str, object]) -> None:
        try:
            self._queue.put_nowait(payload)
        except Full:
            self.dropped += 1

    def close(self) -> None:
        if not self._thread.is_alive():
            return
        try:
            self._queue.put(None, timeout=0.1)
        except Full:
            self.dropped += 1
            return
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as output:
                while True:
                    payload = self._queue.get()
                    if payload is None:
                        return
                    output.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
                    output.write("\n")
                    output.flush()
        except OSError:
            while True:
                try:
                    payload = self._queue.get_nowait()
                except Exception:
                    return
                if payload is None:
                    return
                self.dropped += 1
