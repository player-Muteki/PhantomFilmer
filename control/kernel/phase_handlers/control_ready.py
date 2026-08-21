"""Wait at base height until the operator selects a supported control mode."""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class ControlReadyHandler:
    """Own the indefinite zero-output mode choice after the 150 cm climb."""

    phase = KernelPhase.CONTROL_READY

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        controller = session.manual_controller
        if not controller.config.enabled and not getattr(
            session, "side_follow_available", False
        ) and not getattr(session, "front_follow_available", False):
            return KernelPhase.PRE_FOLLOW

        controller.make_available()
        selection = session._wait_for_control_selection()
        if selection == "manual":
            return KernelPhase.FOLLOW
        if selection == "side":
            return KernelPhase.FOLLOW
        if selection == "front":
            return KernelPhase.FOLLOW
        if selection == "auto":
            return KernelPhase.PRE_FOLLOW
        return None
