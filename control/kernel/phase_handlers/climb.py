"""CLIMB handler: reach the base hover height before following.

Climb commands are vertical-only and deliberately ignore front-ToF avoidance
until the first target has been accepted. Aborts (returns None) with zero output
and a landing on height-sensor, battery, or over-limit-altitude failures.
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
