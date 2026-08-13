"""TAKEOFF handler: human confirmation, fresh-target re-check, then takeoff.

Preserves the original ordering from FollowSession.run(): window confirmation
(or pre-takeoff callback + final fresh check) runs only for ReID tasks, then the
drone takes off under the lifecycle lock. Aborts (returns None) on cancellation
or a stop request.
"""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class TakeoffHandler:
    """Owns the takeoff phase (S2.1/S2.2)."""

    phase = KernelPhase.TAKEOFF

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        if session.initial_target_lock_frames > 0:
            if session.window_takeoff_confirmation and session.display_enabled:
                ctx.locked_result = session._wait_for_window_takeoff_confirmation()
                if not ctx.locked_result:
                    if session.session_state != "TAKEOFF_CANCELLED":
                        session.session_state = "TARGET_LOCK_FAILED"
                    return None
            elif session.pre_takeoff_confirmation is not None:
                if not session.pre_takeoff_confirmation(ctx.locked_result):
                    print("已取消起飞：现场人员未确认目标身份。")
                    session.session_state = "TAKEOFF_CANCELLED"
                    return None
                if not session._verify_fresh_target_before_takeoff():
                    print(
                        "人工确认后目标已离开或身份变得模糊，"
                        "未起飞。"
                    )
                    session.session_state = "TARGET_LOCK_FAILED"
                    return None

        with session._lifecycle_lock:
            if session.stop_event.is_set():
                session.session_state = "STOPPED"
                return None
            if ctx.locked_result:
                authorize_takeoff = getattr(session.drone, "authorize_next_takeoff", None)
                if callable(authorize_takeoff):
                    authorize_takeoff()
            session.drone.takeoff()
            session.airborne = True
            start_front_tof = getattr(session, "_start_front_tof", None)
            if callable(start_front_tof):
                start_front_tof()
        return KernelPhase.STABILIZING
