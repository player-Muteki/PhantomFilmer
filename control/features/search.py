"""Search feature: propose the ReID re-acquisition plan when the target is lost.

Wraps TargetSearchController. It is only invoked while the arbitration table
selects it (target lost + no obstacle + search enabled). Search ends after one
complete three-layer round rather than after a wall-clock timeout.
"""

from typing import Any, Optional

from control.kernel.features import ArbitrationContext, FeatureProposal
from control.target_search import TargetSearchController


class SearchFeature:
    """Recipe-4 proposer: target lost + no obstacle → drive the search FSM."""

    feature_name = "search"

    def __init__(
        self,
        *,
        target_search: TargetSearchController,
        safety_manager,
        follow_controller,
    ) -> None:
        self._search = target_search
        self._safety = safety_manager
        self._follow = follow_controller

    def propose(self, ctx: ArbitrationContext, now: float) -> FeatureProposal:
        # 搜索启用时清掉旧的 8 秒丢失计时，避免安全层
        # 在三层完整搜索轮次完成前提前触发降落。
        self._safety.update_target_lost(True)
        decision = self._search.update(
            ctx.target_result,
            ctx.frame_width,
            ctx.frame_height,
            int(ctx.height_cm) if ctx.height_cm is not None else None,
            now,
            yaw_deg=ctx.yaw_deg,
        )
        if decision is None:
            # 目标实际可见才会走到这里（found=True），与 process_detection
            # 的"普通跟随"兜底保持一致。
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

        state = "TARGET_LOST_LANDING" if decision.action == "land" else decision.state
        return FeatureProposal(
            decision.command,
            state=state,
            reason=decision.reason,
            feature=self.feature_name,
            requires_landing=(decision.action == "land"),
            landing_kind="target_lost" if decision.action == "land" else "",
        )

    def close_recovery_has_priority(self) -> bool:
        """Expose only the bounded too-close backoff as a higher-priority action."""
        return self._search.close_recovery_has_priority()

    def reset(self) -> None:
        pass
