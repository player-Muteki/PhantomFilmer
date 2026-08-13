"""Non-blocking sampler for the RoboMaster TT top/front ToF module."""

from dataclasses import dataclass
from threading import Event, Lock, Thread
from time import monotonic
from typing import Optional

from drone.drone_adapter import DroneAdapter


@dataclass(frozen=True)
class FrontToFSnapshot:
    """Latest front-distance sample safe for use by the control loop."""

    distance_cm: Optional[float]
    status: str
    timestamp: float
    age_seconds: float
    sequence: int
    consecutive_blocked: int


class FrontToFMonitor:
    """Poll the expansion ToF outside the 20 Hz flight-control loop."""

    def __init__(
        self,
        drone: DroneAdapter,
        *,
        blocked_distance_cm: float = 60.0,
        poll_interval_seconds: float = 0.2,
        max_age_seconds: float = 0.8,
    ) -> None:
        self.drone = drone
        self.blocked_distance_cm = max(1.0, float(blocked_distance_cm))
        self.poll_interval_seconds = max(0.05, float(poll_interval_seconds))
        self.max_age_seconds = max(self.poll_interval_seconds, float(max_age_seconds))
        self._lock = Lock()
        self._stop_event = Event()
        self._thread: Optional[Thread] = None
        self._distance_cm: Optional[float] = None
        self._status = "not_ready"
        self._timestamp = 0.0
        self._sequence = 0
        self._consecutive_blocked = 0

    @classmethod
    def from_config(cls, drone: DroneAdapter, config: dict) -> "FrontToFMonitor":
        obstacle = config.get("obstacle", {}) if isinstance(config, dict) else {}
        if not isinstance(obstacle, dict):
            obstacle = {}
        return cls(
            drone,
            blocked_distance_cm=float(obstacle.get("front_tof_blocked_distance_cm", 60.0)),
            poll_interval_seconds=float(obstacle.get("front_tof_poll_interval_seconds", 0.2)),
            max_age_seconds=float(obstacle.get("front_tof_max_age_seconds", 0.8)),
        )

    def prepare(self) -> None:
        """Verify the module once on the ground; never silently disable it."""
        try:
            self._poll_once()
        except Exception as exc:
            raise RuntimeError(
                "前方顶部 ToF 距离模块没有响应；已禁止起飞。"
                "请检查 RoboMaster TT 顶部扩展模块和排线。"
            ) from exc

    def start(self) -> None:
        """Start background polling after takeoff succeeds."""
        if self._thread is not None and self._thread.is_alive():
            return
        if self._status == "not_ready":
            self.prepare()
        self._stop_event.clear()
        self._thread = Thread(target=self._run, name="front-tof-monitor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop polling before landing/stream shutdown commands are sent."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=max(1.0, self.poll_interval_seconds * 3.0))
        self._thread = None

    def snapshot(self) -> FrontToFSnapshot:
        """Return a cached sample without performing SDK I/O."""
        now = monotonic()
        with self._lock:
            age = max(0.0, now - self._timestamp) if self._timestamp else float("inf")
            status = self._status
            if status in {"valid", "out_of_range"} and age > self.max_age_seconds:
                status = "stale"
            return FrontToFSnapshot(
                distance_cm=self._distance_cm,
                status=status,
                timestamp=self._timestamp,
                age_seconds=age,
                sequence=self._sequence,
                consecutive_blocked=self._consecutive_blocked,
            )

    def _run(self) -> None:
        while not self._stop_event.wait(self.poll_interval_seconds):
            try:
                self._poll_once()
            except Exception:
                with self._lock:
                    self._distance_cm = None
                    self._status = "error"
                    self._timestamp = monotonic()
                    self._sequence += 1
                    self._consecutive_blocked = 0

    def _poll_once(self) -> None:
        distance = self.drone.get_front_distance_cm()
        now = monotonic()
        status = "out_of_range" if distance is None else "valid"
        if distance is not None and (distance <= 0 or distance > 120.0):
            raise RuntimeError(f"无效的前向 ToF 距离：{distance}")
        with self._lock:
            self._distance_cm = distance
            self._status = status
            self._timestamp = now
            self._sequence += 1
            if distance is not None and distance <= self.blocked_distance_cm:
                self._consecutive_blocked += 1
            else:
                self._consecutive_blocked = 0
