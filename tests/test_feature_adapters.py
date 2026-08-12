"""Unit tests for the feature-SDK adapters (control/features/*).

These guard each MotionFeature adapter against a stubbed context, plus the
arbitration engine's recipe isolation (an unselected feature is not invoked).
"""

import unittest
from time import monotonic

import numpy as np

from control.features import build_features
from control.features.obstacle import ObstacleFeature
from control.features.safety import SafetyFeature
from control.features.search import SearchFeature
from control.follow_control import FollowController, RCCommand
from control.kernel.arbitration import ArbitrationEngine
from control.kernel.features import ArbitrationContext, FeatureProposal
from control.kernel.phases import KernelPhase
from control.motion_arbiter import MotionArbiter
from control.obstacle_avoidance import AvoidanceDecision, ObstacleAvoidancePlanner
from control.target_search import TargetSearchController
from drone.safety import SafetyConfig, SafetyManager
from vision.obstacle_detect import ObstacleResult


def build_safety() -> SafetyManager:
    return SafetyManager(SafetyConfig(30, 20, 220, 60, 35, 3, 8))


def build_search(config=None) -> TargetSearchController:
    return TargetSearchController(
        {"target_search": {"enabled": True, **(config or {})}},
        min_height_cm=20,
        max_height_cm=220,
    )


def build_ctx(
    *,
    target_result=None,
    frame=None,
    height_cm=None,
    paused=False,
    emergency=False,
    now=0.0,
) -> ArbitrationContext:
    return ArbitrationContext(
        phase=KernelPhase.FOLLOW,
        target_result=target_result or {"found": False, "center": None, "area": 0.0, "bbox": None},
        frame=frame if frame is not None else np.zeros((480, 640, 3), dtype=np.uint8),
        frame_width=640,
        frame_height=480,
        height_cm=height_cm,
        paused=paused,
        emergency=emergency,
        now=now,
    )


def found_result(center=(320, 240), area=12000) -> dict:
    return {"found": True, "center": center, "area": area, "bbox": (280, 200, 80, 80)}


class FoundTargetDetector:
    def detect(self, frame):
        return found_result()


class StubArbiter:
    """MotionArbiter stand-in whose decide returns a canned decision."""

    def __init__(self, decision: AvoidanceDecision) -> None:
        self._decision = decision
        self.last_observation = decision.observation
        self.last_decision = decision
        self.decide_calls = 0

    def decide(self, desired_command, frame, context, obstacle_priority=False):
        self.decide_calls += 1
        self.last_decision = self._decision
        self.last_observation = self._decision.observation
        return self._decision


class CountingSearch:
    feature_name = "search"

    def __init__(self) -> None:
        self.calls = 0

    def propose(self, ctx, now):
        self.calls += 1
        return FeatureProposal(RCCommand(), state="SEARCH_HOLD", feature="search")

    def reset(self) -> None:
        pass


class FollowFeatureTestCase(unittest.TestCase):
    def test_found_target_proposes_follow_command(self) -> None:
        safety = build_safety()
        feature = build_features(
            follow_controller=FollowController(safety_manager=safety),
            safety_manager=safety,
        )["follow"]
        proposal = feature.propose(build_ctx(target_result=found_result()), monotonic())
        self.assertEqual(proposal.feature, "follow")
        self.assertEqual(proposal.state, "FOLLOWING")
        self.assertIsInstance(proposal.command, RCCommand)
        self.assertFalse(proposal.requires_landing)

    def test_search_mode_following_records_pose(self) -> None:
        safety = build_safety()
        search = build_search()
        feature = build_features(
            follow_controller=FollowController(safety_manager=safety),
            safety_manager=safety,
            target_search=search,
            search_enabled=True,
        )["follow"]
        proposal = feature.propose(build_ctx(target_result=found_result()), monotonic())
        self.assertEqual(proposal.state, "FOLLOWING")
        # 普通跟随分支调用 observe_target，记录最后一次可信位姿。
        self.assertEqual(search.last_frame_size, (640, 480))
        self.assertIsNotNone(search.last_bbox)

    def test_search_mode_reacquire_verify(self) -> None:
        safety = build_safety()
        search = build_search()
        feature = build_features(
            follow_controller=FollowController(safety_manager=safety),
            safety_manager=safety,
            target_search=search,
            search_enabled=True,
        )["follow"]
        # 先触发搜索（丢失），再出现真实目标 → REACQUIRE_VERIFY。
        lost = {"found": False, "is_predicted": False, "center": None, "area": 0.0, "bbox": None}
        feature.propose(build_ctx(target_result=lost), monotonic())
        proposal = feature.propose(build_ctx(target_result=found_result()), monotonic())
        self.assertEqual(proposal.state, "REACQUIRE_VERIFY")
        self.assertEqual(proposal.command.as_tuple(), (0, 0, 0, 0))


class ObstacleFeatureTestCase(unittest.TestCase):
    @staticmethod
    def real_arbiter(sequence) -> MotionArbiter:
        safety = build_safety()
        planner = ObstacleAvoidancePlanner(
            safety_manager=safety,
            avoidance_yaw_speed=18,
            avoidance_lateral_speed=12,
            detect_confirm_frames=3,
        )
        return MotionArbiter(
            detector=ScriptedObstacleDetector(sequence),
            planner=planner,
        )

    def test_own_returns_obstacle_first_on_blocked(self) -> None:
        blocked = ObstacleResult(found=True, state="BLOCKED", side="left", consecutive_found_frames=3)
        feature = ObstacleFeature(arbiter=self.real_arbiter([blocked]), mode_label="TEST")
        proposal = feature.own(build_ctx(target_result={"found": False}), RCCommand(), monotonic())
        self.assertEqual(proposal.state, "OBSTACLE_FIRST")
        self.assertEqual(proposal.reason, "avoiding obstacle before search")
        self.assertTrue(feature.last_observation.found)
        self.assertFalse(proposal.requires_landing)

    def test_arbitrate_passes_clear_through(self) -> None:
        clear = ObstacleResult(found=False, state="CLEAR")
        feature = ObstacleFeature(arbiter=self.real_arbiter([clear]), mode_label="TEST")
        desired = RCCommand(left_right=0, forward_backward=30, up_down=0, yaw=0)
        proposal = feature.arbitrate(build_ctx(target_result=found_result()), desired, monotonic())
        self.assertEqual(proposal.command, desired)

    def test_arbitrate_avoids_confirmed_blocked(self) -> None:
        blocked = ObstacleResult(found=True, state="BLOCKED", side="left", consecutive_found_frames=3)
        feature = ObstacleFeature(arbiter=self.real_arbiter([blocked]), mode_label="TEST")
        desired = RCCommand(left_right=0, forward_backward=50, up_down=0, yaw=0)
        proposal = feature.arbitrate(build_ctx(target_result=found_result()), desired, monotonic())
        # 前进恒 0，横向/偏航绕行。
        self.assertEqual(proposal.command.forward_backward, 0)
        self.assertTrue(
            proposal.command.left_right != 0 or proposal.command.yaw != 0
        )

    def test_requires_landing_maps_to_obstacle_failsafe(self) -> None:
        decision = AvoidanceDecision(
            command=RCCommand(),
            state="FAILSAFE",
            action="HOVER",
            reason="avoidance timeout",
            confidence=0.0,
            plan_id="t",
            observation=ObstacleResult(state="UNKNOWN"),
            requires_landing=True,
        )
        feature = ObstacleFeature(arbiter=StubArbiter(decision), mode_label="TEST")
        proposal = feature.arbitrate(build_ctx(target_result=found_result()), RCCommand(), monotonic())
        self.assertTrue(proposal.requires_landing)
        self.assertEqual(proposal.landing_kind, "obstacle_failsafe")


class SearchFeatureTestCase(unittest.TestCase):
    def test_lost_target_starts_search_hold(self) -> None:
        safety = build_safety()
        search = build_search()
        feature = SearchFeature(
            target_search=search,
            safety_manager=safety,
            follow_controller=FollowController(safety_manager=safety),
        )
        proposal = feature.propose(
            build_ctx(target_result={"found": False, "center": None, "area": 0.0, "bbox": None}),
            monotonic(),
        )
        self.assertTrue(search.searching)
        self.assertEqual(proposal.command.as_tuple(), (0, 0, 0, 0))
        self.assertIn(proposal.state, {"LOST_HOLD", "SEARCH_HOLD", "SEARCH_LAST_DIRECTION"})
        self.assertFalse(proposal.requires_landing)

    def test_completed_round_requires_landing_target_lost(self) -> None:
        safety = build_safety()
        search = build_search()
        now = monotonic()
        search.search_height_cm = 150
        search.phase_started_at = now
        search.state = "RETURN_TO_BASE"
        feature = SearchFeature(
            target_search=search,
            safety_manager=safety,
            follow_controller=FollowController(safety_manager=safety),
        )
        proposal = feature.propose(
            build_ctx(
                target_result={"found": False, "center": None, "area": 0.0, "bbox": None},
                height_cm=150,
            ),
            now,
        )
        self.assertTrue(proposal.requires_landing)
        self.assertEqual(proposal.landing_kind, "target_lost")
        self.assertEqual(proposal.state, "TARGET_LOST_LANDING")


class SafetyFeatureTestCase(unittest.TestCase):
    def test_lost_hover_keep_leaves_state_untouched(self) -> None:
        safety = build_safety()
        feature = SafetyFeature(
            safety_manager=safety,
            follow_controller=FollowController(safety_manager=safety),
        )
        proposal = feature.lost_hover(build_ctx(target_result={"found": False}), monotonic())
        self.assertEqual(proposal.state, "")
        self.assertEqual(proposal.command.as_tuple(), (0, 0, 0, 0))
        self.assertFalse(proposal.requires_landing)

    def test_lost_hover_land_requires_deferred_landing(self) -> None:
        safety = build_safety()
        safety._target_lost_since = monotonic() - 8.0
        feature = SafetyFeature(
            safety_manager=safety,
            follow_controller=FollowController(safety_manager=safety),
        )
        proposal = feature.lost_hover(build_ctx(target_result={"found": False}), monotonic())
        self.assertTrue(proposal.requires_landing)
        self.assertEqual(proposal.landing_kind, "target_lost")
        self.assertEqual(proposal.state, "TARGET_LOST_LANDING")


class EngineRecipeIsolationTestCase(unittest.TestCase):
    """Recipe 3 (obstacle takeover) must not invoke the search feature."""

    def build_engine(self, obstacle_sequence, search_feature):
        safety = build_safety()
        planner = ObstacleAvoidancePlanner(
            safety_manager=safety,
            avoidance_yaw_speed=18,
            avoidance_lateral_speed=12,
            detect_confirm_frames=3,
        )
        arbiter = MotionArbiter(
            detector=ScriptedObstacleDetector(obstacle_sequence),
            planner=planner,
        )
        follow = FollowController(safety_manager=safety)
        features = {
            "follow": build_features(follow_controller=follow, safety_manager=safety)["follow"],
            "safety": build_features(follow_controller=follow, safety_manager=safety)["safety"],
            "search": search_feature,
            "obstacle": ObstacleFeature(arbiter=arbiter, mode_label="TEST"),
        }
        return ArbitrationEngine(features=features, follow_controller=follow, mode_label="TEST")

    def test_obstacle_takeover_freezes_search(self) -> None:
        blocked = ObstacleResult(found=True, state="BLOCKED", side="left", consecutive_found_frames=3)
        search = CountingSearch()
        engine = self.build_engine([blocked], search)
        outcome = engine.arbitrate(
            build_ctx(target_result={"found": False, "center": None, "area": 0.0, "bbox": None})
        )
        self.assertEqual(search.calls, 0)  # 障碍接管期间搜索不被调用（计时冻结）
        self.assertEqual(outcome.state, "OBSTACLE_FIRST")
        self.assertEqual(outcome.reason, "avoiding obstacle before search")

    def test_paused_or_emergency_emits_hover_and_invokes_no_feature(self) -> None:
        blocked = ObstacleResult(found=True, state="BLOCKED", side="left", consecutive_found_frames=3)
        search = CountingSearch()
        engine = self.build_engine([blocked], search)
        paused = engine.arbitrate(build_ctx(target_result=found_result(), paused=True))
        self.assertEqual(paused.command.as_tuple(), (0, 0, 0, 0))  # 配方 1：直接悬停
        self.assertEqual(paused.state, "PAUSED")
        self.assertEqual(search.calls, 0)  # 暂停期间不调用任何 feature

        emergency = engine.arbitrate(build_ctx(target_result=found_result(), emergency=True))
        self.assertEqual(emergency.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(emergency.state, "")
        self.assertEqual(search.calls, 0)  # 急停同样不调用 feature

    def test_clear_restores_search_calls(self) -> None:
        clear = ObstacleResult(found=False, state="CLEAR")
        search = CountingSearch()
        engine = self.build_engine([clear], search)
        engine.arbitrate(
            build_ctx(target_result={"found": False, "center": None, "area": 0.0, "bbox": None})
        )
        self.assertEqual(search.calls, 1)  # 无障碍 → 配方 4 调用搜索

    def test_found_target_uses_follow_then_obstacle_arbitrate(self) -> None:
        clear = ObstacleResult(found=False, state="CLEAR")
        engine = self.build_engine([clear], CountingSearch())
        outcome = engine.arbitrate(build_ctx(target_result=found_result()))
        self.assertEqual(outcome.state, "FOLLOWING")
        self.assertTrue(outcome.obstacle_ran)
        self.assertIsNotNone(outcome.avoidance_decision)


class BuildFeaturesTestCase(unittest.TestCase):
    def test_registry_composition(self) -> None:
        safety = build_safety()
        controller = FollowController(safety_manager=safety)
        search = build_search()
        with_obstacle = build_features(
            follow_controller=controller,
            safety_manager=safety,
            target_search=search,
            search_enabled=True,
            motion_arbiter=self._dummy_arbiter(),
        )
        self.assertEqual(set(with_obstacle.keys()), {"follow", "safety", "search", "obstacle"})

        without_obstacle = build_features(follow_controller=controller, safety_manager=safety)
        self.assertEqual(set(without_obstacle.keys()), {"follow", "safety"})

    @staticmethod
    def _dummy_arbiter():
        return MotionArbiter(
            detector=ScriptedObstacleDetector([ObstacleResult(found=False, state="CLEAR")]),
            planner=ObstacleAvoidancePlanner(
                safety_manager=build_safety(),
                avoidance_yaw_speed=18,
                avoidance_lateral_speed=12,
                detect_confirm_frames=3,
            ),
        )


class ScriptedObstacleDetector:
    """Return a fixed obstacle-observation sequence, one result per detect."""

    def __init__(self, sequence) -> None:
        self.sequence = list(sequence)
        self.reads = 0

    def detect(self, frame, target_result):
        index = min(self.reads, len(self.sequence) - 1)
        self.reads += 1
        return self.sequence[index]

    def reset(self) -> None:
        self.reads = 0


if __name__ == "__main__":
    unittest.main()
