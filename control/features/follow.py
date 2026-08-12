"""Follow feature: propose the next autonomous command when a target is in view.

Wraps FollowController (yaw-first horizontal tracking, area-ratio distance band)
and, in ReID search mode, also drives TargetSearchController's reacquire
verification exactly like the old FollowSession.process_detection search branch.
"""

from typing import Any, Dict, Optional

from control.follow_control import RCCommand
from control.kernel.features import ArbitrationContext, FeatureProposal
from control.target_search import TargetSearchController


def _height_int(height_cm: Optional[float]) -> Optional[int]:
    """TargetSearchController.update expects an optional integer height."""
    if height_cm is None:
        return None
    return int(height_cm)


class FollowFeature:
    """Recipe-6 proposer: target found → follow command (+ optional search state)."""

    feature_name = "follow"

    def __init__(
        self,
        *,
        follow_controller,
        safety_manager,
        target_search: Optional[TargetSearchController] = None,
        search_enabled: bool = False,
    ) -> None:
        self._follow = follow_controller
        self._safety = safety_manager
        self._search = target_search
        self._search_enabled = search_enabled

    def propose(self, ctx: ArbitrationContext, now: float) -> FeatureProposal:
        if not self._search_enabled:
            self._safety.update_target_lost(True)
            return FeatureProposal(
                self._follow.compute_command(
                    ctx.target_result, ctx.frame_width, ctx.frame_height
                ),
                state="FOLLOWING",
                reason="",
                feature=self.feature_name,
            )

        # ReID 模式：搜索状态机先校验目标是否可信；未进入搜索或普通跟随 →
        # update 返回 None → 走正常跟随。刚丢失又重见时返回 REACQUIRE_VERIFY。
        self._safety.update_target_lost(True)
        decision = self._search.update(
            ctx.target_result,
            ctx.frame_width,
            ctx.frame_height,
            _height_int(ctx.height_cm),
            now,
        )
        if decision is not None:
            return FeatureProposal(
                decision.command,
                state="TARGET_LOST_LANDING" if decision.action == "land" else decision.state,
                reason=decision.reason,
                feature=self.feature_name,
                requires_landing=(decision.action == "land"),
                landing_kind="target_lost" if decision.action == "land" else "",
            )

        command = self._follow.compute_command(
            ctx.target_result, ctx.frame_width, ctx.frame_height
        )
        self._search.observe_target(
            ctx.target_result, ctx.frame_width, ctx.frame_height, command
        )
        return FeatureProposal(
            command,
            state="FOLLOWING",
            reason="",
            feature=self.feature_name,
        )

    def reset(self) -> None:
        # 内核在会话开始时统一重置控制器；feature 自身无独立状态。
        pass
