"""Non-blocking structured JSONL telemetry for side-follow flights."""

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Thread
from time import monotonic
from typing import TYPE_CHECKING
from uuid import uuid4

from control.follow_control import RCCommand

if TYPE_CHECKING:
    from control.side_follow_control import SideFollowDebugInfo
    from control.target_search import TargetSearchController


@dataclass(frozen=True)
class SideFollowLogConfig:
    """Configuration for optional loss-tolerant side-follow telemetry."""

    enabled: bool = False
    log_dir: Path = Path("logs/side_follow")
    log_every_n_frames: int = 1
    queue_size: int = 256

    @classmethod
    def from_config(cls, config: dict[str, object]) -> "SideFollowLogConfig":
        """Build logging settings from the shared application configuration."""
        section = config.get("side_follow", {}) if isinstance(config, dict) else {}
        if not isinstance(section, dict):
            section = {}
        return cls(
            enabled=bool(section.get("log_enabled", False)),
            log_dir=Path(str(section.get("log_dir", "logs/side_follow"))),
            log_every_n_frames=cls._positive_int(section.get("log_every_n_frames"), 1),
            queue_size=cls._positive_int(section.get("log_queue_size"), 256),
        )

    @staticmethod
    def _positive_int(value: object, default: int) -> int:
        if not isinstance(value, (int, float, str)):
            return default
        try:
            return max(1, int(value))
        except (OverflowError, ValueError):
            return default


class SideFollowEventRecorder:
    """Queue per-tick flight facts without blocking the control loop.

    Queue saturation and disk failures drop telemetry. They never delay or
    alter the command sent to the aircraft.
    """

    def __init__(self, config: SideFollowLogConfig) -> None:
        self.config = config
        self.session_id = ""
        self._frame_index = 0
        self._writer: _JsonlEventWriter | None = None

    def reset(self, mode: str) -> None:
        """Start a fresh correlated log after side-follow mode is selected."""
        self.close()
        self.session_id = uuid4().hex
        self._frame_index = 0
        if self.config.enabled:
            self._writer = _JsonlEventWriter(
                self._session_log_path(mode), queue_size=self.config.queue_size
            )

    def record(
        self,
        *,
        mode: str,
        follow_mode: str = "side",
        target_result: dict[str, object],
        debug: "SideFollowDebugInfo",
        command: RCCommand,
        state: str,
        reason: str,
        frame_width: int,
        frame_height: int,
        battery_percent: int | None = None,
        height_cm: int | None = None,
        aircraft_yaw_deg: int | None = None,
        control_hz: float = 0.0,
        vision_fps: float = 0.0,
        search: "TargetSearchController | None" = None,
    ) -> None:
        """Queue one structured flight decision without performing disk I/O."""
        writer = self._writer
        if writer is None:
            return
        self._frame_index += 1
        if self._frame_index % self.config.log_every_n_frames:
            return
        payload: dict[str, object] = {
            "schema_version": 1,
            "event": "side_follow_decision",
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "monotonic_timestamp": round(monotonic(), 6),
            "session_id": self.session_id,
            "mode": mode,
            "follow_mode": follow_mode,
            "frame_index": self._frame_index,
            "state": state,
            "reason": reason,
            "target": self._target_payload(target_result, frame_width, frame_height),
            "flight": {
                "battery_percent": battery_percent,
                "height_cm": height_cm,
                "aircraft_yaw_deg": aircraft_yaw_deg,
                "control_hz": round(control_hz, 3),
                "vision_fps": round(vision_fps, 3),
            },
            "side_follow": {
                "controller_state": debug.state,
                "current_angle": self._round_or_none(debug.current_angle),
                "selected_angle": debug.selected_angle,
                "angle_error": self._round_or_none(debug.angle_error),
                "stable_samples": debug.stable_samples,
                "lock_frames": debug.lock_frames,
                "side_locked": debug.side_locked,
                "position_priority": debug.position_priority,
                "side_reselect_pending": debug.side_reselect_pending,
                "centered_angle_samples": debug.centered_angle_samples,
                "centered_angle_stable": debug.centered_angle_stable,
                "center_tolerance_ratio": self._round_or_none(
                    debug.center_tolerance_ratio
                ),
                "horizontal_error": self._round_or_none(debug.horizontal_error),
                "orbit_active": debug.orbit_active,
                "orbit_direction": debug.orbit_direction,
                "tracking_lateral": debug.tracking_lateral,
                "orbit_lateral": debug.orbit_lateral,
                "yaw_feedforward": debug.yaw_feedforward,
                "yaw_feedback": debug.yaw_feedback,
            },
            "search": {
                "state": search.state if search is not None else "IDLE",
                "searching": search.searching if search is not None else False,
                "last_horizontal_direction": (
                    search.last_horizontal_direction if search is not None else 0
                ),
                "layer_index": search.layer_index if search is not None else 0,
                "search_height_cm": (
                    search.search_height_cm if search is not None else None
                ),
                "close_attempts": search.close_attempts if search is not None else 0,
                "rotation_progress_degrees": self._round_or_none(
                    search.rotation_progress_degrees if search is not None else 0.0
                ),
                "reacquire_progress": (
                    search.reacquire_progress if search is not None else "0/0"
                ),
            },
            "final_command": self._command_payload(command),
            "dropped_events_before_record": writer.dropped,
        }
        writer.write(payload)

    def close(self) -> None:
        """Flush queued records during cleanup without touching flight output."""
        if self._writer is not None:
            self._writer.close()
        self._writer = None

    @property
    def log_path(self) -> Path | None:
        """Return the active session path, if logging is running."""
        return self._writer.path if self._writer is not None else None

    @property
    def dropped_events(self) -> int:
        """Return records dropped to protect the control loop."""
        return self._writer.dropped if self._writer is not None else 0

    def _session_log_path(self, mode: str) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        safe_mode = "".join(char if char.isalnum() else "-" for char in mode).strip("-")
        prefix = safe_mode.lower() or "side-follow"
        return self.config.log_dir / f"{stamp}-{prefix}-{self.session_id}.jsonl"

    @staticmethod
    def _target_payload(
        target: dict[str, object], frame_width: int, frame_height: int
    ) -> dict[str, object]:
        center = SideFollowEventRecorder._numeric_list(target.get("center"), 2)
        bbox = SideFollowEventRecorder._numeric_list(target.get("bbox"), 4)
        width = max(1, frame_width)
        height = max(1, frame_height)
        payload: dict[str, object] = {
            "found": bool(target.get("found")),
            "is_predicted": bool(target.get("is_predicted", False)),
            "ambiguous": bool(target.get("ambiguous", False)),
            "similarity": SideFollowEventRecorder._round_or_none(
                target.get("similarity")
            ),
            "area": SideFollowEventRecorder._round_or_none(target.get("area")),
            "center": center,
            "bbox": bbox,
            "body_orientation_angle": SideFollowEventRecorder._round_or_none(
                target.get("body_orientation_angle")
            ),
            "body_orientation_detection_confidence": (
                SideFollowEventRecorder._round_or_none(
                    target.get("body_orientation_detection_confidence")
                )
            ),
            "body_orientation_match_iou": SideFollowEventRecorder._round_or_none(
                target.get("body_orientation_match_iou")
            ),
        }
        if center is not None:
            payload["center_norm"] = [
                round(center[0] / width, 4),
                round(center[1] / height, 4),
            ]
        if bbox is not None:
            payload["bbox_norm"] = [
                round(bbox[0] / width, 4),
                round(bbox[1] / height, 4),
                round(bbox[2] / width, 4),
                round(bbox[3] / height, 4),
            ]
        return payload

    @staticmethod
    def _command_payload(command: RCCommand) -> dict[str, int]:
        return {
            "left_right": command.left_right,
            "forward_backward": command.forward_backward,
            "up_down": command.up_down,
            "yaw": command.yaw,
        }

    @staticmethod
    def _numeric_list(value: object, length: int) -> list[float] | None:
        if not isinstance(value, (tuple, list)) or len(value) != length:
            return None
        numbers = [SideFollowEventRecorder._finite_number(item) for item in value]
        if any(number is None for number in numbers):
            return None
        return [number for number in numbers if number is not None]

    @staticmethod
    def _round_or_none(value: object) -> float | None:
        number = SideFollowEventRecorder._finite_number(value)
        return None if number is None else round(number, 4)

    @staticmethod
    def _finite_number(value: object) -> float | None:
        if not isinstance(value, (int, float, str)):
            return None
        try:
            number = float(value)
        except ValueError:
            return None
        return number if math.isfinite(number) else None


class _JsonlEventWriter:
    """Bounded background writer; dropped logs never delay flight control."""

    def __init__(self, path: Path, queue_size: int) -> None:
        self.path = path
        self.dropped = 0
        self._queue: Queue[dict[str, object] | None] = Queue(maxsize=queue_size)
        self._thread = Thread(
            target=self._run, name="SideFollowEventWriter", daemon=True
        )
        self._thread.start()

    def write(self, payload: dict[str, object]) -> None:
        """Enqueue one payload or count it as dropped when saturated."""
        try:
            self._queue.put_nowait(payload)
        except Full:
            self.dropped += 1

    def close(self) -> None:
        """Request a bounded flush and stop the writer thread."""
        if not self._thread.is_alive():
            return
        try:
            self._queue.put(None, timeout=0.1)
        except Full:
            try:
                self._queue.get_nowait()
                self.dropped += 1
                self._queue.put_nowait(None)
            except (Empty, Full):
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
                    output.write(
                        json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                    )
                    output.write("\n")
                    output.flush()
        except (OSError, TypeError, ValueError):
            self._drop_remaining()

    def _drop_remaining(self) -> None:
        while True:
            try:
                payload = self._queue.get_nowait()
            except Empty:
                return
            if payload is None:
                return
            self.dropped += 1
