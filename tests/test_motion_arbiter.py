"""Tests for the deterministic motion arbiter and local planner behavior."""

import json
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, Optional
from unittest.mock import patch

import numpy as np

from control.follow_control import RCCommand
from control.motion_arbiter import MotionArbiter, MotionContext
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from drone.safety import SafetyManager
from drone.front_tof import FrontToFSnapshot
from vision.obstacle_detect import ObstacleResult


def build_safety() -> SafetyManager:
    return SafetyManager.from_dict({})


class StubDetector:
    def __init__(
        self,
        result: ObstacleResult,
        error: Optional[Exception] = None,
    ) -> None:
        self.result = result
        self.error = error
        self.detect_calls = 0

    def detect(self, frame: Any, target_result: Dict[str, object]) -> ObstacleResult:
        self.detect_calls += 1
        if self.error is not None:
            raise self.error
        return self.result

    def reset(self) -> None:
        self.detect_calls = 0


class MotionArbiterTestCase(unittest.TestCase):
    @staticmethod
    def front_sample(
        distance_cm: Optional[float], *, status: str = "valid", count: int = 3,
        sequence: int = 1,
    ):
        return FrontToFSnapshot(
            distance_cm=distance_cm,
            status=status,
            timestamp=1.0,
            age_seconds=0.01,
            sequence=sequence,
            consecutive_blocked=count,
        )

    def test_front_tof_at_60_cm_overrides_visual_clear_as_blocked(self) -> None:
        safety = build_safety()
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"front_tof_enabled": True, "front_tof_blocked_distance_cm": 60}},
        )
        arbiter.set_front_tof_provider(lambda: self.front_sample(60.0))

        decision = arbiter.decide(
            RCCommand(0, 20, 0, 0), object(), MotionContext("test", {"found": True})
        )

        self.assertTrue(decision.observation.found)
        self.assertEqual(decision.observation.state, "BLOCKED")
        self.assertEqual(decision.observation.front_distance_cm, 60.0)
        self.assertIn(decision.state, {"AVOIDING", "SCAN"})

    def test_front_tof_above_60_cm_does_not_increase_visual_risk(self) -> None:
        safety = build_safety()
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"front_tof_enabled": True, "front_tof_blocked_distance_cm": 60}},
        )
        arbiter.set_front_tof_provider(lambda: self.front_sample(60.1, count=0))

        decision = arbiter.decide(
            RCCommand(0, 20, 0, 0), object(), MotionContext("test", {"found": True})
        )

        self.assertIsNotNone(arbiter.last_observation)
        self.assertFalse(arbiter.last_observation.found)
        self.assertEqual(decision.state, "CLEAR")
        self.assertEqual(decision.command.forward_backward, 20)

    def test_reacquired_target_cancels_active_post_bypass_turn(self) -> None:
        safety = build_safety()
        planner = ObstacleAvoidancePlanner(
            safety_manager=safety,
            avoidance_lateral_speed=20,
            bypass_lateral_distance_cm=100,
        )
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=planner,
        )
        arbiter.reset("test")
        blocked = ObstacleResult(found=True, state="BLOCKED", consecutive_found_frames=3)
        clear = ObstacleResult(found=False, state="CLEAR")
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 5.0]):
            planner.plan(RCCommand(), blocked, yaw_deg=170)
            planner.plan(RCCommand(), clear, yaw_deg=170)

        decision = arbiter.decide(
            RCCommand(0, 25, 0, 0), object(), MotionContext("test", {"found": True})
        )

        self.assertEqual(decision.action, "FOLLOW")
        self.assertEqual(decision.command.forward_backward, 25)

    def test_front_tof_stale_sample_fails_safe_to_hover(self) -> None:
        safety = build_safety()
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"front_tof_enabled": True}},
        )
        arbiter.set_front_tof_provider(
            lambda: self.front_sample(None, status="stale", count=0)
        )

        decision = arbiter.decide(
            RCCommand(0, 20, 0, 0), object(), MotionContext("test", {"found": True})
        )

        self.assertEqual(decision.state, "FAILSAFE")
        self.assertEqual(decision.command.as_tuple(), (0, 0, 0, 0))

    def test_decide_returns_planner_command_and_observation(self) -> None:
        safety = build_safety()
        result = ObstacleResult(
            found=True,
            state="BLOCKED",
            side="right",
            free_space={"left": 0.8, "right": 0.2},
            consecutive_found_frames=3,
            confidence=0.8,
        )
        arbiter = MotionArbiter(
            detector=StubDetector(result),
            planner=ObstacleAvoidancePlanner(
                safety_manager=safety,
                avoidance_lateral_speed=12,
            ),
        )
        decision = arbiter.decide(
            desired_command=RCCommand(0, 20, 0, 0),
            frame=object(),
            context=MotionContext(mode="test", target_result={"found": True}),
        )

        self.assertEqual(decision.observation, result)
        self.assertEqual(decision.state, "AVOIDING")
        self.assertGreater(decision.command.left_right, 0)
        self.assertEqual(decision.command.forward_backward, 0)
        self.assertGreater(decision.confidence, 0.0)

    def test_sub_sampling_reuses_last_observation_between_detections(self) -> None:
        safety = build_safety()
        detector = StubDetector(ObstacleResult(found=False))
        arbiter = MotionArbiter(
            detector=detector,
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"detect_every_n_frames": 2}},
        )
        context = MotionContext(mode="test", target_result={"found": True})
        first = arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        second = arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)

        # 5 次决策、每 2 帧检测一次 → 检测器只跑 3 次；复用的帧返回最近一次检测结果。
        self.assertEqual(detector.detect_calls, 3)
        self.assertIs(first.observation, second.observation)

    def test_default_keeps_detecting_every_frame(self) -> None:
        safety = build_safety()
        detector = StubDetector(ObstacleResult(found=False))
        arbiter = MotionArbiter(
            detector=detector,
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
        )
        context = MotionContext(mode="test", target_result={"found": True})
        for _ in range(3):
            arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        self.assertEqual(detector.detect_calls, 3)

    def test_decide_forwards_obstacle_priority(self) -> None:
        safety = build_safety()
        result = ObstacleResult(
            found=True,
            state="BLOCKED",
            side="left",
            consecutive_found_frames=3,
        )
        planner = ObstacleAvoidancePlanner(
            safety_manager=safety,
            avoidance_yaw_speed=18,
            avoidance_lateral_speed=12,
        )
        arbiter = MotionArbiter(detector=StubDetector(result), planner=planner)
        context = MotionContext(mode="test", target_result={"found": False})

        stationary_brake = arbiter.decide(
            desired_command=RCCommand(),
            frame=object(),
            context=context,
        )
        detour = arbiter.decide(
            desired_command=RCCommand(),
            frame=object(),
            context=context,
            obstacle_priority=True,
        )

        # 距离避障独立于跟随期望指令；即使当前悬停也会开始绕行。
        self.assertEqual(stationary_brake.state, "AVOIDING")
        self.assertNotEqual(stationary_brake.command.left_right, 0)
        self.assertEqual(detour.state, "AVOIDING")
        self.assertEqual(detour.command.forward_backward, 0)
        self.assertNotEqual(detour.command.left_right, 0)

    def test_pipeline_error_returns_zero_command(self) -> None:
        safety = build_safety()
        arbiter = MotionArbiter(
            detector=StubDetector(
                ObstacleResult(),
                error=RuntimeError("sensor failure"),
            ),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
        )
        decision = arbiter.decide(
            desired_command=RCCommand(0, 20, 0, 0),
            frame=object(),
            context=MotionContext(mode="test", target_result={"found": True}),
        )

        self.assertEqual(decision.state, "FAILSAFE")
        self.assertEqual(decision.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(decision.observation.data_quality, "planner_error")

    def test_lost_target_tof_failure_hovers_then_requests_landing(self) -> None:
        safety = build_safety()
        current = self.front_sample(90, sequence=1)
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={
                "obstacle": {
                    "front_tof_enabled": True,
                    "lost_tof_failure_limit": 3,
                }
            },
        )
        arbiter.set_front_tof_provider(lambda: current)
        # Establish that the target was previously visible.
        arbiter.decide(
            RCCommand(), object(), MotionContext("test", {"found": True})
        )
        failures = iter(
            self.front_sample(None, status="stale", count=0, sequence=sequence)
            for sequence in (2, 3, 4)
        )
        arbiter.set_front_tof_provider(lambda: next(failures))

        decisions = []
        for _ in range(3):
            decisions.append(
                arbiter.decide(
                    RCCommand(), object(), MotionContext("test", {"found": False}),
                    obstacle_priority=True,
                )
            )

        self.assertEqual([item.command.as_tuple() for item in decisions], [(0, 0, 0, 0)] * 3)
        self.assertFalse(decisions[0].requires_landing)
        self.assertFalse(decisions[1].requires_landing)
        self.assertTrue(decisions[2].requires_landing)
        self.assertEqual(decisions[2].action, "LAND")

    def test_repeated_same_failed_tof_sample_does_not_reach_landing_limit(self) -> None:
        safety = build_safety()
        current = self.front_sample(90, sequence=1)
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={
                "obstacle": {
                    "front_tof_enabled": True,
                    "lost_tof_failure_limit": 3,
                }
            },
        )
        arbiter.set_front_tof_provider(lambda: current)
        arbiter.decide(RCCommand(), object(), MotionContext("test", {"found": True}))
        current = self.front_sample(None, status="error", count=0, sequence=2)

        decisions = [
            arbiter.decide(
                RCCommand(), object(), MotionContext("test", {"found": False}),
                obstacle_priority=True,
            )
            for _ in range(10)
        ]

        self.assertTrue(all(not item.requires_landing for item in decisions))

    def test_fresh_reid_target_immediately_cancels_post_bypass_turn(self) -> None:
        safety = build_safety()
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"front_tof_enabled": True}},
        )
        arbiter.set_front_tof_provider(
            lambda: self.front_sample(100.0, count=0, sequence=8)
        )
        arbiter.reset("test")
        arbiter.planner._dynamic_lost_bypass = True
        arbiter.planner._bypass_phase = "POST_BYPASS_LEFT_TURN"
        arbiter.planner._phase_started_at = 0.0
        arbiter.planner._turn_last_yaw = 30.0

        desired = RCCommand(0, 18, 0, 7)
        decision = arbiter.decide(
            desired,
            object(),
            MotionContext("test", {"found": True}, yaw_deg=25),
            obstacle_priority=True,
        )

        self.assertEqual(decision.command, desired)
        self.assertEqual(decision.action, "FOLLOW")
        self.assertIsNone(arbiter.planner._bypass_phase)

    def test_center_loss_advances_until_tof_detects_object_within_120_cm(self) -> None:
        safety = build_safety()
        current = self.front_sample(None, status="out_of_range", count=0, sequence=1)
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(
                safety_manager=safety,
                avoidance_lateral_speed=20,
                detect_confirm_frames=3,
            ),
            config={
                "obstacle": {
                    "front_tof_enabled": True,
                    "center_loss_forward_enabled": True,
                    "center_loss_forward_speed": 25,
                    "center_loss_history_frames": 3,
                    "center_loss_edge_margin_ratio": 0.05,
                }
            },
        )
        arbiter.set_front_tof_provider(lambda: current)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        visible = {
            "found": True,
            "is_predicted": False,
            "ambiguous": False,
            "bbox": (250, 60, 140, 360),
        }
        for _ in range(3):
            arbiter.decide(RCCommand(), frame, MotionContext("test", visible))

        advancing = arbiter.decide(
            RCCommand(), frame, MotionContext("test", {"found": False}),
            obstacle_priority=True,
        )
        self.assertEqual(advancing.state, "CENTER_LOSS_ADVANCE")
        self.assertEqual(advancing.command.as_tuple(), (0, 25, 0, 0))

        current = self.front_sample(110, status="valid", count=0, sequence=2)
        avoiding = arbiter.decide(
            RCCommand(), frame, MotionContext("test", {"found": False}),
            obstacle_priority=True,
        )
        self.assertEqual(avoiding.action, "SIDE_STEP_OUT")
        self.assertEqual(avoiding.command.as_tuple(), (20, 0, 0, 0))
        self.assertTrue(avoiding.observation.found)

    def test_horizontal_edge_exit_does_not_start_blind_forward_recovery(self) -> None:
        safety = build_safety()
        arbiter = MotionArbiter(
            detector=StubDetector(ObstacleResult(found=False, state="CLEAR")),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"front_tof_enabled": True}},
        )
        arbiter.set_front_tof_provider(
            lambda: self.front_sample(None, status="out_of_range", count=0, sequence=1)
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        results = (
            {"found": True, "bbox": (250, 60, 140, 360)},
            {"found": True, "bbox": (350, 60, 140, 360)},
            {"found": True, "bbox": (510, 60, 130, 360)},
        )
        for result in results:
            arbiter.decide(RCCommand(), frame, MotionContext("test", result))

        lost = arbiter.decide(
            RCCommand(), frame, MotionContext("test", {"found": False}),
            obstacle_priority=True,
        )
        self.assertNotEqual(lost.state, "CENTER_LOSS_ADVANCE")
        self.assertEqual(lost.command.forward_backward, 0)

    def test_error_observation_is_not_reused_by_sub_sampling(self) -> None:
        safety = build_safety()
        detector = StubDetector(ObstacleResult(), error=RuntimeError("sensor failure"))
        arbiter = MotionArbiter(
            detector=detector,
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"detect_every_n_frames": 2}},
        )
        context = MotionContext(mode="test", target_result={"found": True})
        states = [
            arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context).state
            for _ in range(3)
        ]
        # 失败帧不缓存观测：即使处于跳过帧，每个决策帧都必须重新检测并 FAILSAFE，
        # 不能复用"未发现障碍"的错误结果变成全速跟随。
        self.assertEqual(detector.detect_calls, 3)
        self.assertEqual(states, ["FAILSAFE", "FAILSAFE", "FAILSAFE"])

    def test_sub_sampling_preserves_detection_confirmation_cadence(self) -> None:
        safety = build_safety()

        class CountingFoundDetector:
            def __init__(self) -> None:
                self.detect_calls = 0
                self.observation = ObstacleResult(
                    found=True,
                    state="BLOCKED",
                    side="center",
                    free_space={"left": 0.9, "right": 0.1},
                )

            def detect(self, frame: Any, target_result: Dict[str, object]) -> ObstacleResult:
                self.detect_calls += 1
                self.observation.consecutive_found_frames = self.detect_calls
                return self.observation

            def reset(self) -> None:
                self.detect_calls = 0

        detector = CountingFoundDetector()
        arbiter = MotionArbiter(
            detector=detector,  # type: ignore[arg-type]
            planner=ObstacleAvoidancePlanner(safety_manager=safety, detect_confirm_frames=3),
            config={"obstacle": {"detect_every_n_frames": 2}},
        )
        context = MotionContext(mode="test", target_result={"found": True})
        states = [
            arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context).state
            for _ in range(4)
        ]
        # 确认按实际帧数推进：第 4 帧（2 次真实检测 + 跳过帧递增）达到 3 次确认开始绕行；
        # 若跳过帧不递增，第 4 帧仍是 BRAKING，绕行会被推迟到第 5 帧。
        self.assertEqual(detector.detect_calls, 2)
        self.assertEqual(states, ["BRAKING", "BRAKING", "BRAKING", "AVOIDING"])

    def test_invalidate_observation_forces_fresh_detection(self) -> None:
        safety = build_safety()
        detector = StubDetector(ObstacleResult(found=False))
        arbiter = MotionArbiter(
            detector=detector,
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"detect_every_n_frames": 2}},
        )
        context = MotionContext(mode="test", target_result={"found": True})
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        self.assertEqual(detector.detect_calls, 1)  # 第二次是跳过帧
        arbiter.invalidate_observation()
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        self.assertEqual(detector.detect_calls, 2)

    def test_llm_friendly_jsonl_is_written_without_blocking_decision(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            safety = build_safety()
            result = ObstacleResult(
                found=True,
                state="CAUTION",
                side="left",
                frame_index=1,
            )
            arbiter = MotionArbiter(
                detector=StubDetector(result),
                planner=ObstacleAvoidancePlanner(safety_manager=safety),
                config={"obstacle": {"log_enabled": True, "log_dir": tmp, "log_every_n_frames": 1}},
            )
            decision = arbiter.decide(
                desired_command=RCCommand(0, 20, 0, 0),
                frame=object(),
                context=MotionContext(mode="test", target_result={"found": True}),
            )
            arbiter.close()

            files = list(Path(tmp).glob("*.jsonl"))
            self.assertEqual(len(files), 1)
            payload = json.loads(files[0].read_text(encoding="utf-8").splitlines()[0])
            self.assertEqual(payload["schema_version"], 1)
            self.assertEqual(payload["mode"], "test")
            self.assertEqual(payload["decision"]["state"], decision.state)
            self.assertEqual(payload["observation"]["state"], "CAUTION")


if __name__ == "__main__":
    unittest.main()
