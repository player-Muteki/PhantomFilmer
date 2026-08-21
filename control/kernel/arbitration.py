"""Arbitration engine: the kernel's single decision point for follow motion.

Every FOLLOW tick the kernel assembles an ArbitrationContext and asks the engine to
pick exactly one owner from the recipe table (priorities 1-6). Only the selected
owner's command is emitted, so unselected features naturally freeze their internal
state machines and timers while preempted.

Recipes (see the architecture plan §3.1):
  1. paused / emergency → kernel hover, no feature called
  2. safety wants_landing (battery / height) → handled in the kernel loop
  3. manual takeover → ManualFeature.propose
  4. target lost + obstacle present → ObstacleFeature.own (OBSTACLE_FIRST)
  5. target lost + no obstacle + search enabled → SearchFeature.propose
  6. target lost + no obstacle + search disabled → SafetyFeature.lost_hover
  7. target found → FollowFeature.propose → ObstacleFeature.arbitrate
"""

from dataclasses import dataclass, replace
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
        self._manual = features.get("manual")
        self._obstacle = features.get("obstacle")
        self._search = features.get("search")
        self._safety = features.get("safety")
        self._follow_controller = follow_controller
        self._mode_label = mode_label
        # Top/front ToF avoidance stays inactive until this independent task
        # has completed the normal ReID acceptance path and entered FOLLOWING.
        # Non-search modes keep their historical obstacle behavior.  The gate
        # applies to ReID tasks that actually have a bounded search feature.
        self._target_ever_acquired = self._search is None

    @property
    def target_ever_acquired(self) -> bool:
        """Whether this task has entered normal following at least once."""
        return self._target_ever_acquired

    def reset(self) -> None:
        """Reset the first-acquisition gate for an independent flight task."""
        self._target_ever_acquired = self._search is None

    def arbitrate(self, ctx: ArbitrationContext) -> FollowTickOutcome:
        hover = self._follow_controller.hover()

        # 配方 1：暂停/急停 → 内核直接悬停，不调用任何 feature。
        if ctx.paused or ctx.emergency:
            return FollowTickOutcome(
                command=hover, state="PAUSED" if ctx.paused else ""
            )

        # Manual takeover is available after the 150 cm base-height phase and
        # deliberately precedes first-target acquisition, search, and obstacle
        # route ownership.  Its own controller applies only conservative
        # forward/height guards; it never starts an autonomous bypass.
        if self._manual is not None and self._manual.active:
            return self._finish(self._manual.propose(ctx, ctx.now))

        # Before the first accepted target, ToF must not influence motion: a
        # missing target goes directly to the bounded search, while a visible
        # candidate is handled only by the existing ReID verification path.
        if not self._target_ever_acquired:
            if not ctx.target_result.get("found"):
                return self._finish(self._lost_tracking(ctx))
            first_target = self._follow.propose(ctx, ctx.now)
            if first_target.state == "FOLLOWING":
                self._target_ever_acquired = True
            return self._finish(first_target)

        # A target that left through the left/right frame edge has an explicit
        # visual direction.  Keep that entire reacquisition attempt in SEARCH
        # and do not let front ToF start probing or a bypass route.
        if (
            not ctx.target_result.get("found")
            and self._horizontal_edge_exit_has_priority()
        ):
            return self._finish(self._lost_tracking(ctx))

        # 唯一高于避障的目标丢失动作：视觉历史明确表明人物贴得太近时，
        # 先完成有界后退/停顿。其他所有搜索动作仍保持避障优先。
        if not ctx.target_result.get("found") and self._close_recovery_has_priority():
            return self._finish(self._lost_tracking(ctx))

        # 配方 3：目标丢失 → 先障碍探测（own）；有阻挡则避障接管、暂停找人。
        if self._obstacle is not None and not ctx.target_result.get("found"):
            own = self._obstacle.own(ctx, hover, ctx.now)
            observation = self._obstacle.last_observation
            decision = self._obstacle.last_decision
            if (
                observation is not None
                and (observation.found or (decision is not None and decision.owns_motion))
            ):
                return self._finish(
                    own,
                    obstacle_ran=True,
                    obstacle_observation=observation,
                    avoidance_decision=decision,
                )
            if own.requires_landing:
                # 探测即要求落地（罕见：无阻挡观测但规划器超时）→ 立即落地。
                return FollowTickOutcome(
                    command=own.command,
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
            return self._finish(
                tracking,
                obstacle_ran=True,
                obstacle_observation=observation,
                avoidance_decision=decision,
            )

        # 配方 6：目标存在 → follow 期望指令 → 避障仲裁（obstacle_priority=False）。
        if self._obstacle is not None:
            follow = self._follow.propose(ctx, ctx.now)
            # A visible target that has become too large is a close-range
            # recovery case, not an obstacle-bypass case.  Back away first so
            # the front ToF cannot replace the safer reverse command with a
            # lateral sidestep.
            visible_close_recovery = self._visible_close_recovery_has_priority(ctx)
            if visible_close_recovery:
                command = self._visible_close_recovery_command()
                follow = replace(follow, command=command, reason="visible target too close; backing away")
                return self._finish(follow)
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

    def _lost_tracking(self, ctx: ArbitrationContext) -> FeatureProposal:
        if self._search is not None:
            return self._search.propose(ctx, ctx.now)  # 配方 4
        return self._safety.lost_hover(ctx, ctx.now)  # 配方 5

    def _close_recovery_has_priority(self) -> bool:
        checker = getattr(self._search, "close_recovery_has_priority", None)
        return bool(checker()) if callable(checker) else False

    def _visible_close_recovery_has_priority(self, ctx: ArbitrationContext) -> bool:
        checker = getattr(self._search, "visible_close_recovery_has_priority", None)
        return bool(checker(ctx.target_result, ctx.frame_width, ctx.frame_height)) if callable(checker) else False

    def _visible_close_recovery_command(self) -> RCCommand:
        command_factory = getattr(self._search, "visible_close_recovery_command", None)
        if callable(command_factory):
            return command_factory()
        return RCCommand()

    def _horizontal_edge_exit_has_priority(self) -> bool:
        checker = getattr(self._search, "horizontal_edge_exit_has_priority", None)
        return bool(checker()) if callable(checker) else False

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
