"""Lifecycle phase handlers for the kernel.

Each handler owns one lifecycle phase and returns the next phase to enter, or
None to abort the lifecycle (kernel then does fail-safe cleanup and returns the
current session state). Handlers orchestrate the session's lifecycle
sub-routines, which keep their bodies in FollowSession so the existing tests
that patch ``control.follow_session.sleep/monotonic`` keep working.
"""

from dataclasses import dataclass, field
from typing import Any, Dict

from control.kernel.phase_handlers.climb import ClimbHandler
from control.kernel.phase_handlers.control_ready import ControlReadyHandler
from control.kernel.phase_handlers.follow import FollowHandler
from control.kernel.phase_handlers.ground_lock import GroundLockHandler
from control.kernel.phase_handlers.height_verify import HeightVerifyHandler
from control.kernel.phase_handlers.stabilize import StabilizeHandler
from control.kernel.phase_handlers.takeoff import TakeoffHandler
from control.kernel.phases import KernelPhase


@dataclass
class LifecycleContext:
    """State carried between lifecycle phase handlers."""

    locked_result: Dict[str, object] = field(default_factory=dict)


def build_phase_handlers() -> Dict[KernelPhase, Any]:
    """Assemble the phase → handler table for one autonomous session."""
    return {
        KernelPhase.PRE_FLIGHT: GroundLockHandler(),
        KernelPhase.TAKEOFF: TakeoffHandler(),
        KernelPhase.STABILIZING: StabilizeHandler(),
        KernelPhase.HEIGHT_VERIFY: HeightVerifyHandler(),
        KernelPhase.CLIMB: ClimbHandler(),
        KernelPhase.CONTROL_READY: ControlReadyHandler(),
        KernelPhase.FOLLOW: FollowHandler(),
    }


__all__ = [
    "LifecycleContext",
    "build_phase_handlers",
]
