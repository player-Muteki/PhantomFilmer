"""Arbitration engine: the kernel's single decision point for follow motion.

Every FOLLOW tick the kernel assembles an ArbitrationContext and asks the engine to
pick exactly one owner from the recipe table (priorities 1-6). Only the selected
owner's command is emitted, so unselected features naturally freeze their internal
state machines and timers while preempted.

Recipes (see the architecture plan §3.1):
  1. paused / emergency → kernel hover, no feature called
  2. safety wants_landing (battery / height) → handled in the kernel loop
  3. target lost → passive obstacle probe → occlusion recovery may own a
     bounded lateral peek only after target-memory and overlap confirmation
  4. initial acquisition / unconfirmed loss → yaw-only acquisition or search
  5. target lost + no obstacle + search disabled → SafetyFeature.lost_hover
  6. target found → FollowFeature.propose → ObstacleFeature.arbitrate
"""

from dataclasses import dataclass
from typing import Any, Dict, Optional

from control.follow_control import FollowController, RCCommand
from control.kernel.features import ArbitrationContext, FeatureProposal
from control.motion_arbiter import MotionArbiter
from control.obstacle_avoidance import AvoidanceDecision
from vision.obstacle_detect import ObstacleResult


@dataclass
class FollowTickOutcome:
    """Result of one arbitration tick, with enough metadata for kernel bookkeeping."""

    command: RCCommand
    requires_landing: bool = False
    landing_state: Optional[str] = None
    state: str = ""  # non-empty → kernel applies as session_state
    reason: str = ""  # non-empty → kernel applies as search_reason
    obstacle_ran: bool = False
    obstacle_observation: Optional[ObstacleResult] = None
    avoidance_decision: Optional[AvoidanceDecision] = None
    lost_land: bool = False


class ArbitrationEngine:
    """Recipe table 1-6: the only place that decides who controls motion this tick."""

    def __init__(
        self,
        *,
        features: Dict[str, Any],
        follow_controller: FollowController,
        mode_label: str = "FOLLOW",
    ) -> None:
        self._features = features
        self._follow = features.get("follow")
        self._obstacle = features.get("obstacle")
        self._occlusion = features.get("occlusion_recovery")
        self._search = features.get("search")
        self._safety = features.get("safety")
        self._follow_controller = follow_controller
        self._mode_label = mode_label

    def arbitrate(self, ctx: ArbitrationContext) -> FollowTickOutcome:
        hover = self._follow_controller.hover()

        # 配方 1：暂停/急停 → 内核直接悬停，不调用任何 feature。
        if ctx.paused or ctx.emergency:
            return FollowTickOutcome(
                command=hover, state="PAUSED" if ctx.paused else ""
            )

        # Target acquisition is a mission rule, not an obstacle-avoidance rule:
        # the initial full-turn scan remains active even if obstacle CV is disabled.
        if (
            self._obstacle is None
            and self._occlusion is not None
            and not ctx.target_result.get("found")
        ):
            recovery = self._occlusion.propose(ctx, None, ctx.now)
            if recovery is not None:
                return self._finish(recovery)

        # 新配方 3：目标丢失时避障仅被动探测。是否确为“挡住已锁定目标的
        # 那个障碍物”，由恢复状态机按目标记忆、时空重合和连续帧判断。
        if self._obstacle is not None and not ctx.target_result.get("found"):
            probe = self._obstacle.probe(ctx, ctx.now)
            observation = self._obstacle.last_observation
            decision = self._obstacle.last_decision
            if self._occlusion is not None:
                recovery = self._occlusion.propose(ctx, observation, ctx.now)
                if recovery is not None:
                    return self._finish(
                        recovery,
                        obstacle_ran=True,
                        obstacle_observation=observation,
                        avoidance_decision=decision,
                    )
            elif observation is not None and observation.found:
                # 兼容显式关闭新恢复状态机的旧行为。
                own = self._obstacle.own(ctx, hover, ctx.now)
                return self._finish(
                    own,
                    obstacle_ran=True,
                    obstacle_observation=observation,
                    avoidance_decision=decision,
                )
            if probe.requires_landing:
                # 探测即要求落地（罕见：无阻挡观测但规划器超时）→ 立即落地。
                return FollowTickOutcome(
                    command=probe.command,
                    state="",
                    reason="",
                    requires_landing=True,
                    landing_state="OBSTACLE_FAILSAFE_LANDING",
                    obstacle_ran=True,
                    obstacle_observation=observation,
                    avoidance_decision=decision,
                )
            # 无障碍 → 配方 4/5（搜索透传 / 丢失悬停），obstacle 本 tick 只做探测。
            tracking = self._lost_tracking(ctx)
            tracking = self._gate_lost_search(tracking, observation)
            return self._finish(
                tracking,
                obstacle_ran=True,
                obstacle_observation=observation,
                avoidance_decision=decision,
            )

        # 目标重新出现时，恢复状态机要求连续身份确认；确认期间只悬停，
        # 通过后才把控制权交回正常跟随。
        if self._occlusion is not None:
            recovery = self._occlusion.propose(ctx, None, ctx.now)
            if recovery is not None:
                return self._finish(recovery)

        # 配方 6：目标存在 → follow 期望指令 → 避障仲裁（obstacle_priority=False）。
        if self._obstacle is not None:
            follow = self._follow.propose(ctx, ctx.now)
            arbitrated = self._obstacle.arbitrate(ctx, follow.command, ctx.now)
            return self._finish(
                arbitrated,
                state=follow.state,
                reason=follow.reason,
                obstacle_ran=True,
                obstacle_observation=self._obstacle.last_observation,
                avoidance_decision=self._obstacle.last_decision,
            )

        # 无 obstacle feature（避障关闭）→ 直接 follow / search / lost。
        if not ctx.target_result.get("found"):
            tracking = self._lost_tracking(ctx)
            return self._finish(tracking)
        follow = self._follow.propose(ctx, ctx.now)
        return self._finish(follow)

    @staticmethod
    def _gate_lost_search(
        proposal: FeatureProposal,
        observation: Optional[ObstacleResult],
    ) -> FeatureProposal:
        """Use any unassociated obstacle as a veto, never an active-motion owner.

        A lost-target search may normally change height or perform a bounded
        backoff. While an obstacle is still visible, retain only in-place yaw and
        suppress all translation. This prevents the fallback search from undoing
        the occlusion state's safety guarantee.
        """
        if observation is None:
            return proposal
        if observation.data_quality != "ok":
            return FeatureProposal(
                RCCommand(),
                state=proposal.state,
                reason=f"obstacle sensor hold: {observation.data_quality}",
                feature=proposal.feature,
                active=proposal.active,
                requires_landing=proposal.requires_landing,
                landing_kind=proposal.landing_kind,
            )
        if not observation.found:
            return proposal
        command = proposal.command
        if not (command.left_right or command.forward_backward or command.up_down):
            return proposal
        reason = proposal.reason
        suffix = "obstacle veto: lost-target translation suppressed"
        return FeatureProposal(
            RCCommand(yaw=command.yaw),
            state=proposal.state,
            reason=f"{reason}; {suffix}" if reason else suffix,
            feature=proposal.feature,
            active=proposal.active,
            requires_landing=proposal.requires_landing,
            landing_kind=proposal.landing_kind,
        )

    def _lost_tracking(self, ctx: ArbitrationContext) -> FeatureProposal:
        if self._search is not None:
            return self._search.propose(ctx, ctx.now)  # 配方 4
        return self._safety.lost_hover(ctx, ctx.now)  # 配方 5

    def _finish(
        self,
        proposal: FeatureProposal,
        *,
        state: Optional[str] = None,
        reason: Optional[str] = None,
        obstacle_ran: bool = False,
        obstacle_observation: Optional[ObstacleResult] = None,
        avoidance_decision: Optional[AvoidanceDecision] = None,
    ) -> FollowTickOutcome:
        """Map one proposal to a FollowTickOutcome, preserving landing semantics.

        obstacle_failsafe → 立即落地（清零后直接进降落序列）；
        target_lost → 延迟落地（先跑电量/高度检查，保留 TARGET_LOST_LANDING）。
        """
        requires_landing = proposal.requires_landing
        landing_state: Optional[str] = None
        lost_land = False
        if proposal.landing_kind == "obstacle_failsafe":
            requires_landing = True
            landing_state = "OBSTACLE_FAILSAFE_LANDING"
        elif proposal.landing_kind == "target_lost":
            # Deferred target-loss landing must first pass the loop's latest
            # battery/height checks; do not route it through the immediate
            # obstacle-failsafe branch.
            requires_landing = False
            lost_land = True
        elif requires_landing:
            landing_state = "OBSTACLE_FAILSAFE_LANDING"

        return FollowTickOutcome(
            command=proposal.command,
            requires_landing=requires_landing,
            landing_state=landing_state,
            state=proposal.state if state is None else state,
            reason=proposal.reason if reason is None else reason,
            obstacle_ran=obstacle_ran,
            obstacle_observation=obstacle_observation,
            avoidance_decision=avoidance_decision,
            lost_land=lost_land,
        )
