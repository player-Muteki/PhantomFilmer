"""Feature adapter for operator manual takeover."""

from control.kernel.features import ArbitrationContext, FeatureProposal
from control.manual_control import ManualControlController


class ManualFeature:
    """Propose the current guarded operator command while takeover is active."""

    feature_name = "manual"

    def __init__(self, controller: ManualControlController) -> None:
        self.controller = controller

    @property
    def active(self) -> bool:
        return self.controller.active

    def propose(self, ctx: ArbitrationContext, now: float) -> FeatureProposal:
        command = self.controller.command_for(
            now=now,
            height_cm=ctx.height_cm,
            front_tof_snapshot=ctx.front_tof_snapshot,
        )
        return FeatureProposal(
            command=command,
            state="MANUAL",
            reason=self.controller.last_guard_reason or "manual operator control",
            feature=self.feature_name,
        )

    def reset(self) -> None:
        self.controller.disable()
