"""HEIGHT_VERIFY handler: confirm TOF ground clearance after takeoff.

Aborts (returns None) with zero output and a landing when height readings never
become valid within the timeout.
"""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class HeightVerifyHandler:
    """Owns the TOF height-verification phase (S2.3)."""

    phase = KernelPhase.HEIGHT_VERIFY

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        if not session._verify_takeoff_height():
            session._safe_zero_output()
            if session.airborne:
                session._safe_land()
            return None
        return KernelPhase.CLIMB
