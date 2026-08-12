"""PRE_FOLLOW handler: optional fixed-demo maneuver before target following.

When a pre-follow maneuver is configured, runs it via the session's obstacle
arbiter seam; an aborted maneuver stops the lifecycle, a completed one advances
to FOLLOW after resetting tracking state.
"""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class PreFollowHandler:
    """Owns the pre-follow maneuver phase (S9)."""

    phase = KernelPhase.PRE_FOLLOW

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        maneuver = session.pre_follow_maneuver
        if maneuver is None:
            return KernelPhase.FOLLOW

        session.session_state = "FIXED_DEMO"
        print("固定演示航线已启动；航线完成后自动进入目标跟随。")
        print("窗口按键：q 停止并降落，e 急停并降落。")
        completed = maneuver.run(
            send_command=session.send_motion_command,
            should_abort=session._pre_follow_should_abort,
            on_progress=session._show_pre_follow_progress,
            is_avoiding=session._fixed_demo_is_avoiding,
        )
        if not completed:
            if session.emergency_stop:
                session.session_state = "EMERGENCY_STOP"
            elif session.session_state != "STOPPED":
                session.session_state = "STOPPED"
            print("固定演示航线已中止，准备安全降落。")
            return None

        session._reset_tracking_state()
        print("固定演示航线完成，控制输出已清零，开始目标跟随。")
        return KernelPhase.FOLLOW
