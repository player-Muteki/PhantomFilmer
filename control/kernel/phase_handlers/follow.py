"""FOLLOW handler: enter the target-following work loop.

This phase is driven by the session's blocking ``_loop()`` (per-tick arbitration
through the feature engine). The phase FSM stops here: _loop returns only when
the session ends, which triggers kernel cleanup.
"""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class FollowHandler:
    """Owns the FOLLOW work-loop phase (S3)."""

    phase = KernelPhase.FOLLOW

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        session.session_state = "FOLLOWING"
        print(f"跟随任务已启动，当前运行模式：{session.mode_label}")
        if session.allow_pause:
            print("窗口按键：p 暂停/继续，q 停止并降落，e 急停并降落。")
        else:
            print("窗口按键：q 停止并降落，e 急停并降落。")
        session._loop()
        return None
