"""CLIMB handler: reach the base hover height before following.

Climb commands go through the obstacle feature when arbitration is enabled.
Aborts (returns None) with zero output and a landing on any safety check
failure (height sensor, battery, over-limit altitude, persistent obstacle).
"""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class ClimbHandler:
    """Owns the base-hover climb phase (S2.4)."""

    phase = KernelPhase.CLIMB

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        if not session._reach_base_hover_height():
            session._safe_zero_output()
            if session.airborne:
                session._safe_land()
            return None
        return KernelPhase.PRE_FOLLOW
