"""PRE_FLIGHT handler: grounded ReID identity lock.

Only runs when ``initial_target_lock_frames > 0`` (ReID tasks). Drives no RC;
aborts the lifecycle when the target cannot be locked in time.
"""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class GroundLockHandler:
    """Owns the grounded ReID lock phase (S1.2)."""

    phase = KernelPhase.PRE_FLIGHT

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        if session.initial_target_lock_frames <= 0:
            return KernelPhase.TAKEOFF

        ctx.locked_result = session._wait_for_initial_target_lock()
        if not ctx.locked_result:
            if session.session_state != "STOPPED":
                session.session_state = "TARGET_LOCK_FAILED"
            return None
        return KernelPhase.TAKEOFF
