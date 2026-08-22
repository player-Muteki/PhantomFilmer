"""Lifecycle phases and phase-handler protocol for the kernel.

The kernel runs a small phase FSM extracted from FollowSession.run(). Each phase is
owned by one PhaseHandler; a handler returns the next phase to enter, or None to
stay in place. RC emission only happens inside FOLLOW (via the arbitration table),
CLIMB (vertical-only), and during fail-safe
cleanup.
"""

from enum import Enum
from typing import Any, Optional, Protocol, runtime_checkable


class KernelPhase(Enum):
    """The ordered lifecycle phases of one autonomous session."""

    PRE_FLIGHT = "pre_flight"
    TAKEOFF = "takeoff"
    STABILIZING = "stabilizing"
    HEIGHT_VERIFY = "height_verify"
    CLIMB = "climb"
    CONTROL_READY = "control_ready"
    FOLLOW = "follow"
    FAILSAFE = "failsafe"
    LANDING = "landing"


@runtime_checkable
class PhaseHandler(Protocol):
    """A handler owns exactly one lifecycle phase and drives its transitions."""

    phase: KernelPhase

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        """Advance one tick of this phase.

        Return the next phase to switch into, or None to stay in this phase.
        """
        ...
