"""Feature protocol and shared data types for the kernel + feature-SDK architecture.

The kernel is a deterministic scheduler + safety shell. Each autonomous capability
(follow, obstacle avoidance, target search, safety, scripted maneuver) is a
MotionFeature: a bounded proposer that returns a FeatureProposal. The kernel picks
exactly one owner per tick through its arbitration table and emits only that owner's
command through the single RC emission seam.

Three invariants are enforced at this boundary:
  1. Single RC emission: every autonomous command flows through KernelSession._emit.
  2. Any feature exception degrades to a zero-command fail-safe.
  3. Timing/priority belong to the kernel: a feature advances its own state machine
     only while it is selected (pause/preemption freezes its timers naturally).
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Protocol, runtime_checkable

from control.follow_control import RCCommand
from vision.obstacle_detect import ObstacleResult


class FeatureError(Exception):
    """A feature failed internally; the kernel degrades to a zero-command fail-safe."""


@dataclass(frozen=True)
class FeatureProposal:
    """What one feature wants the kernel to do this tick."""

    command: RCCommand
    state: str = "IDLE"
    reason: str = ""
    feature: str = ""
    active: bool = True
    requires_landing: bool = False
    # 区分"立即落地"（避障 5s 超时，内核立刻清零落地）与"延迟落地"
    # （目标丢失计时 8s / 搜索 30s，必须先跑完电量/高度检查再落地）。
    landing_kind: str = ""  # "" | "obstacle_failsafe" | "target_lost"


@dataclass
class ArbitrationContext:
    """Facts the kernel assembles once per tick for every feature to read."""

    phase: Any
    target_result: Dict[str, object]
    frame: Any
    frame_width: int = 0
    frame_height: int = 0
    mode: str = "FOLLOW"
    height_cm: Optional[float] = None
    battery: Optional[int] = None
    yaw_deg: Optional[int] = None
    paused: bool = False
    emergency: bool = False
    stop_requested: bool = False
    last_obstacle: Optional[ObstacleResult] = None
    front_tof_snapshot: Optional[Any] = None
    now: float = 0.0
    extra: Dict[str, Any] = field(default_factory=dict)


@runtime_checkable
class MotionFeature(Protocol):
    """Protocol every feature SDK must implement."""

    feature_name: str

    def propose(self, ctx: ArbitrationContext, now: float) -> FeatureProposal:
        """Return this feature's proposal for the current tick (bounded runtime)."""
        ...

    def reset(self) -> None:
        """Reset internal state at session start or after a mode switch."""
        ...
