"""Safety feature: propose the non-ReID target-loss hover/land fallback.

Wraps SafetyManager.update_target_lost. Only invoked by the arbitration table
when the target is lost, no obstacle is present, and ReID search is disabled
(recipe 5), so the 3s-hover / 8s-land timer advances exactly as before.
"""

from typing import Any, Optional

from control.kernel.features import ArbitrationContext, FeatureProposal


class SafetyFeature:
    """Recipe-5 proposer: target lost + no obstacle + no search → hover, then land."""

    feature_name = "safety"

    def __init__(self, *, safety_manager, follow_controller) -> None:
        self._safety = safety_manager
        self._follow = follow_controller

    def lost_hover(self, ctx: ArbitrationContext, now: float) -> FeatureProposal:
        lost_action = self._safety.update_target_lost(False)
        if lost_action == "land":
            return FeatureProposal(
                self._follow.hover(),
                state="TARGET_LOST_LANDING",
                reason="target lost",
                feature=self.feature_name,
                requires_landing=True,
                landing_kind="target_lost",
            )
        # keep / hover：不改变现有会话状态（旧代码在丢失悬停期间保持原状态）。
        return FeatureProposal(
            self._follow.hover(),
            state="",
            reason="",
            feature=self.feature_name,
        )

    def reset(self) -> None:
        pass
