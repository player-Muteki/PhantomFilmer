"""KernelSession: the lean kernel.

Owns the lifecycle work loop (phase FSM), the single RC emission seam
(``_emit``), the feature fail-safe (``_failsafe``), and landing cleanup.
``FollowSession`` is a thin facade whose ``run()`` delegates here.

Blocking lifecycle loops (ground lock / stabilization / height verify / climb
/ follow tick) deliberately stay on the session object so that tests patching
``control.follow_session.sleep``/``monotonic`` keep working; the kernel drives
those loops via the phase handlers instead of reimplementing them.
"""

from threading import Lock
from typing import Any

from control.kernel.phase_handlers import LifecycleContext, build_phase_handlers
from control.kernel.phases import KernelPhase


class KernelSession:
    """Work-loop kernel around one follow session runtime."""

    def __init__(self, session: Any) -> None:
        self._session = session
        self._emit_lock = Lock()

    def run(self) -> Any:
        """Drive the lifecycle: reset → camera → phase FSM → safe cleanup.

        Returns the same ``FollowSessionResult`` (state/airborne/streaming) that
        FollowSession.run() historically returned.
        """
        from control.follow_session import FollowSessionResult

        session = self._session
        try:
            session._reset_tracking_state()
            session._prepare_detector()
            with session._lifecycle_lock:
                if session.stop_event.is_set():
                    session.session_state = "STOPPED"
                    return FollowSessionResult(
                        state=session.session_state,
                        airborne=session.airborne,
                        streaming=session.streaming,
                    )
                session._start_camera()
                prepare_front_tof = getattr(session, "_prepare_front_tof", None)
                if callable(prepare_front_tof):
                    prepare_front_tof()

            ctx = LifecycleContext()
            phase = KernelPhase.PRE_FLIGHT
            handlers = build_phase_handlers()
            while phase is not None:
                handler = handlers.get(phase)
                if handler is None:
                    break
                phase = handler.run(session, ctx)
        except KeyboardInterrupt:
            print("收到 Ctrl+C，正在安全停止跟随任务。")
            session.session_state = "STOPPED"
        finally:
            self._land_and_cleanup()

        return FollowSessionResult(
            state=session.session_state,
            airborne=session.airborne,
            streaming=session.streaming,
        )

    def _emit(self, command: Any) -> None:
        """Single RC emission seam (invariant 1).

        Emergency / pause / stop always collapse to a zero command, then the
        command is clamped by ``SafetyManager.limit_rc_command`` before reaching
        ``DroneAdapter.move_rc``. No other module emits autonomous motion.
        """
        session = self._session
        # Keep state validation and the physical SDK write atomic.  Without a
        # global emission lock, an external emergency zero could complete while
        # an older non-zero tick was paused immediately before move_rc(), then
        # that stale tick could resume and become the final aircraft command.
        with self._emit_lock:
            if session.emergency_stop or session.paused or session.stop_event.is_set():
                command = session.follow_controller.hover()
            limited = session.safety_manager.limit_rc_command(*command.as_tuple())
            session.last_command = type(command)(*limited)
            session.drone.move_rc(*session.last_command.as_tuple())

    def _failsafe(self, exc: Exception) -> None:
        """Feature fail-safe (invariant 2): any feature error → zero output.

        The exception is never allowed to reach the flight controller; it is
        logged and the current output is collapsed to zero.
        """
        session = self._session
        print(f"功能模块异常，控制输出已清零：{exc}")
        session.search_reason = f"feature_error:{exc}"
        self._emit(session.follow_controller.hover())

    def _land_and_cleanup(self) -> None:
        """Zero motion, land, and release resources in a safe order."""
        session = self._session
        cancel_manual_watchdog = getattr(session, "_cancel_manual_watchdog", None)
        if callable(cancel_manual_watchdog):
            cancel_manual_watchdog(force_hover=True)
        # Stop motion before waiting for a sensor thread or issuing a blocking
        # landing command.  The previous order could leave the final manual
        # direction active during FrontToFMonitor.stop().
        session._safe_zero_output()
        stop_front_tof = getattr(session, "_stop_front_tof", None)
        if callable(stop_front_tof):
            stop_front_tof()
        if session.airborne:
            session._safe_land()
        session._stop_camera()
        if session.motion_arbiter is not None:
            session.motion_arbiter.close()
        session._destroy_window()
