"""Unified obstacle observation, local planning, and decision logging."""

import json
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from queue import Full, Queue
from threading import Thread
from time import monotonic
from typing import Any, Callable, Dict, Optional
from uuid import uuid4

from control.follow_control import RCCommand
from control.obstacle_avoidance import AvoidanceDecision, ObstacleAvoidancePlanner
from vision.obstacle_detect import DistanceOnlyObstacleDetector, ObstacleResult
from drone.front_tof import FrontToFSnapshot


@dataclass(frozen=True)
class MotionContext:
    """Facts the arbiter needs about one autonomous motion tick."""

    mode: str
    target_result: Dict[str, object]
    yaw_deg: Optional[int] = None


class MotionArbiter:
    """Deep module for deterministic obstacle-aware command selection."""

    def __init__(
        self,
        detector: DistanceOnlyObstacleDetector,
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
        # 每 N 帧才重新跑一次检测器，中间帧复用上次观测，只保留规划/仲裁开销。
        self._detect_every_n_frames = max(1, self._config_int("detect_every_n_frames", 1))
        self._detect_counter = 1
        self._detector_ran = False
        self._writer: Optional[_JsonlEventWriter] = None
        self.session_id = ""
        self.mode = "unknown"
        self.last_observation: Optional[ObstacleResult] = None
        self.last_decision: Optional[AvoidanceDecision] = None
        self.last_latency_ms = 0.0
        self._active = False
        self._front_tof_enabled = bool(self._obstacle_config.get("front_tof_enabled", False))
        self._front_tof_blocked_distance_cm = self._config_float(
            "front_tof_blocked_distance_cm", 60.0
        )
        self._front_tof_provider: Optional[Callable[[], FrontToFSnapshot]] = None
        self._target_seen = False
        self._target_was_found = False
        self._lost_episode_counter = 0
        self._active_lost_episode_id: Optional[int] = None
        self._lost_tof_failure_limit = max(
            1, self._config_int("lost_tof_failure_limit", 5)
        )
        self._lost_tof_failures = 0
        self._latest_front_sequence = -1
        self._last_failed_front_sequence = -1

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
        self._detect_counter = 1
        self._detector_ran = False
        self._target_seen = False
        self._target_was_found = False
        self._lost_episode_counter = 0
        self._active_lost_episode_id = None
        self._lost_tof_failures = 0
        self._latest_front_sequence = -1
        self._last_failed_front_sequence = -1
        self._active = True
        if self._log_enabled:
            self._writer = _JsonlEventWriter(self._session_log_path())

    def decide(
        self,
        desired_command: RCCommand,
        frame: Any,
        context: MotionContext,
        obstacle_priority: bool = False,
    ) -> AvoidanceDecision:
        """Observe one frame and return a bounded deterministic command.

        obstacle_priority 用于目标丢失场景：即使期望指令全零，也允许规划器
        对已确认的阻挡障碍主动绕行，而不是命中静止刹车分支。默认 False，
        普通跟随路径行为不变。
        """
        if not self._active:
            self.reset(context.mode)
        fresh_target = (
            bool(context.target_result.get("found"))
            and not bool(context.target_result.get("is_predicted"))
            and not bool(context.target_result.get("ambiguous"))
        )
        if fresh_target:
            self._target_seen = True
            self._target_was_found = True
            self._active_lost_episode_id = None
            self.planner.cancel_lost_target_recovery()
        elif self._target_was_found:
            self._target_was_found = False
            self._lost_episode_counter += 1
            self._active_lost_episode_id = self._lost_episode_counter
        started_at = monotonic()
        try:
            self._detect_counter += 1
            if self.last_observation is None or self._detect_counter % self._detect_every_n_frames == 0:
                self._detector_ran = True
                observation = self.detector.detect(frame, context.target_result)
                self.last_latency_ms = (monotonic() - started_at) * 1000.0
            else:
                self._detector_ran = False
                observation = self.last_observation
                if observation is not None and observation.found:
                    # 跳过帧复用上次观测时，按实际帧数推进"连续确认"计数，
                    # 否则 detect_confirm_frames 变成按检测次数计数、确认明显变慢。
                    observation = replace(
                        observation,
                        consecutive_found_frames=observation.consecutive_found_frames + 1,
                    )
            observation = self._fuse_front_tof(observation)
            self._lost_tof_failures = 0
            self._last_failed_front_sequence = -1
            decision = self.planner.plan(
                desired_command,
                observation,
                obstacle_priority,
                lost_episode_id=(
                    self._active_lost_episode_id
                    if obstacle_priority and self._target_seen
                    else None
                ),
                yaw_deg=context.yaw_deg,
            )
            self.last_observation = observation
        except Exception as exc:
            if obstacle_priority and self._active_lost_episode_id is not None:
                if self._latest_front_sequence != self._last_failed_front_sequence:
                    self._lost_tof_failures += 1
                    self._last_failed_front_sequence = self._latest_front_sequence
            else:
                self._lost_tof_failures = 0
            should_land = self._lost_tof_failures >= self._lost_tof_failure_limit
            observation = ObstacleResult(
                state="UNKNOWN",
                data_quality="planner_error",
                timestamp=monotonic(),
            )
            decision = AvoidanceDecision(
                command=self._zero_command(),
                state="FAILSAFE",
                action="LAND" if should_land else "HOVER",
                reason=(
                    f"front ToF unavailable {self._lost_tof_failures}/"
                    f"{self._lost_tof_failure_limit}: {type(exc).__name__}"
                    if obstacle_priority and self._active_lost_episode_id is not None
                    else f"obstacle pipeline error: {type(exc).__name__}"
                ),
                confidence=0.0,
                plan_id="error",
                requires_landing=should_land,
                owns_motion=True,
                observation=observation,
            )
            # 失败时不缓存错误观测：跳过帧若复用它，会把它当作"未发现障碍"，
            # 把每帧的 FAILSAFE 稀释成全速跟随。置空让下一帧强制重测。
            self.last_observation = None
        self.last_decision = decision
        self._record(desired_command, context, observation, decision)
        return decision

    def invalidate_observation(self) -> None:
        """Force the next decide() to run a fresh detection (e.g. after a pause)."""
        self.last_observation = None

    def set_front_tof_provider(
        self, provider: Optional[Callable[[], FrontToFSnapshot]]
    ) -> None:
        """Attach a non-blocking front-distance snapshot provider."""
        self._front_tof_provider = provider

    def _fuse_front_tof(self, visual: ObstacleResult) -> ObstacleResult:
        if not self._front_tof_enabled:
            return visual
        if self._front_tof_provider is None:
            raise RuntimeError("front ToF provider is not attached")
        sample = self._front_tof_provider()
        self._latest_front_sequence = sample.sequence
        if sample.status not in {"valid", "out_of_range"}:
            raise RuntimeError(f"front ToF sample is {sample.status}")
        updates: Dict[str, object] = {
            "front_distance_cm": sample.distance_cm,
            "front_distance_status": sample.status,
            "front_distance_age_seconds": round(sample.age_seconds, 3),
            "front_distance_sequence": sample.sequence,
        }
        if (
            sample.distance_cm is not None
            and sample.distance_cm <= self._front_tof_blocked_distance_cm
        ):
            updates.update(
                found=True,
                state="BLOCKED",
                side=visual.side if visual.side in {"left", "right"} else "center",
                confidence=1.0,
                consecutive_found_frames=max(
                    visual.consecutive_found_frames, sample.consecutive_blocked
                ),
                consecutive_clear_frames=0,
            )
        return replace(visual, **updates)

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
            "detector_ran": bool(self._detector_ran),
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

    def _config_float(self, key: str, default: float) -> float:
        try:
            return float(self._obstacle_config.get(key, default))
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
        except (OSError, TypeError):
            # 写盘失败或 payload 不可序列化时丢弃剩余事件，绝不让日志影响控制循环。
            while True:
                try:
                    payload = self._queue.get_nowait()
                except Exception:
                    return
                if payload is None:
                    return
                self.dropped += 1
