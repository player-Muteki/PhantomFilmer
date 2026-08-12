"""Obstacle feature: arbitrate any desired command through the avoidance planner.

Wraps MotionArbiter. It has two modes the kernel selects between:
  - own():        recipe 3 — target lost, obstacle probes for takeover, so a
                  confirmed block yields active avoidance instead of a hover.
  - arbitrate():  recipe 6 / climb / pre-follow — gate a desired command through
                  the normal avoidance pipeline (CLEAR pass / CAUTION scale /
                  BLOCKED brake → avoid).
"""

from typing import Any, Optional

from control.follow_control import RCCommand
from control.kernel.features import ArbitrationContext, FeatureProposal
from control.motion_arbiter import MotionArbiter, MotionContext


class ObstacleFeature:
    """Owns the motion arbiter; proposes avoidance-aware commands."""

    feature_name = "obstacle"

    def __init__(self, *, arbiter: MotionArbiter, mode_label: str = "FOLLOW") -> None:
        self._arbiter = arbiter
        self._mode_label = mode_label

    @property
    def last_observation(self):
        return self._arbiter.last_observation

    @property
    def last_decision(self):
        return self._arbiter.last_decision

    def _context(self, ctx: ArbitrationContext) -> MotionContext:
        return MotionContext(mode=self._mode_label, target_result=ctx.target_result)

    def own(self, ctx: ArbitrationContext, desired: RCCommand, now: float) -> FeatureProposal:
        """Target lost: probe for a block; if present, take over and avoid."""
        decision = self._arbiter.decide(
            desired_command=desired,
            frame=ctx.frame,
            context=self._context(ctx),
            obstacle_priority=True,
        )
        return FeatureProposal(
            decision.command,
            state="OBSTACLE_FIRST",
            reason="avoiding obstacle before search",
            feature=self.feature_name,
            requires_landing=decision.requires_landing,
            landing_kind="obstacle_failsafe" if decision.requires_landing else "",
        )

    def arbitrate(self, ctx: ArbitrationContext, desired: RCCommand, now: float) -> FeatureProposal:
        """Gate a desired autonomous command through the avoidance pipeline."""
        decision = self._arbiter.decide(
            desired_command=desired,
            frame=ctx.frame,
            context=self._context(ctx),
        )
        return FeatureProposal(
            decision.command,
            state=decision.state,
            reason=decision.reason,
            feature=self.feature_name,
            requires_landing=decision.requires_landing,
            landing_kind="obstacle_failsafe" if decision.requires_landing else "",
        )

    def reset(self) -> None:
        pass
