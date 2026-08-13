"""Tests for the distance-only three-stage bypass route."""

import unittest
from unittest.mock import patch

from control.follow_control import RCCommand
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from drone.safety import SafetyConfig, SafetyManager
from vision.obstacle_detect import ObstacleResult


def safety(max_speed: int = 35) -> SafetyManager:
    return SafetyManager(SafetyConfig(30, 20, 220, 60, max_speed, 3, 8))


def distance(cm, *, blocked=False, count=3, status="valid") -> ObstacleResult:
    return ObstacleResult(
        found=blocked,
        state="BLOCKED" if blocked else "CLEAR",
        confidence=1.0,
        consecutive_found_frames=count if blocked else 0,
        front_distance_cm=cm,
        front_distance_status=status,
    )


class DistanceBypassPlannerTestCase(unittest.TestCase):
    def planner(self, **kwargs) -> ObstacleAvoidancePlanner:
        return ObstacleAvoidancePlanner(
            safety(),
            avoidance_lateral_speed=20,
            detect_confirm_frames=3,
            clearance_distance_cm=70,
            bypass_forward_distance_cm=120,
            bypass_forward_speed=35,
            max_sidestep_seconds=10,
            **kwargs,
        )

    def test_clear_passes_follow_command(self) -> None:
        planner = self.planner()
        desired = RCCommand(0, 20, 0, 5)
        decision = planner.plan(desired, distance(90))
        self.assertEqual(decision.command, desired)
        self.assertFalse(decision.owns_motion)

    def test_from_config_loads_requested_route(self) -> None:
        planner = ObstacleAvoidancePlanner.from_config(
            safety(),
            {
                "obstacle": {
                    "avoidance_lateral_speed": 20,
                    "front_tof_clear_distance_cm": 70,
                    "bypass_forward_distance_cm": 120,
                    "bypass_forward_speed": 35,
                    "bypass_lateral_direction": "right",
                }
            },
        )
        self.assertEqual(planner.avoidance_lateral_speed, 20)
        self.assertEqual(planner.clearance_distance_cm, 70)
        self.assertEqual(planner.bypass_forward_distance_cm, 120)
        self.assertEqual(planner.bypass_forward_speed, 35)
        self.assertEqual(planner.bypass_lateral_direction, "right")

    def test_blocked_brakes_until_three_samples_then_steps_right(self) -> None:
        planner = self.planner()
        first = planner.plan(RCCommand(), distance(60, blocked=True, count=1))
        with patch("control.obstacle_avoidance.monotonic", return_value=1.0):
            confirmed = planner.plan(RCCommand(), distance(59, blocked=True, count=3))
        self.assertEqual(first.action, "BRAKE")
        self.assertEqual(first.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(confirmed.action, "SIDE_STEP_OUT")
        self.assertEqual(confirmed.command.as_tuple(), (20, 0, 0, 0))

    def test_60_to_70_keeps_side_stepping_and_over_70_starts_forward(self) -> None:
        planner = self.planner()
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 1.0, 2.0]):
            planner.plan(RCCommand(), distance(60, blocked=True))
            still_side = planner.plan(RCCommand(), distance(70))
            forward = planner.plan(RCCommand(), distance(70.1))
        self.assertEqual(still_side.action, "SIDE_STEP_OUT")
        self.assertEqual(forward.action, "FORWARD_120CM")
        self.assertEqual(forward.command.as_tuple(), (0, 35, 0, 0))
        self.assertAlmostEqual(planner._bypass_lateral_seconds, 2.0)

    def test_out_of_range_is_clear_for_forward_transition(self) -> None:
        planner = self.planner()
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 1.0]):
            planner.plan(RCCommand(), distance(60, blocked=True))
            forward = planner.plan(RCCommand(), distance(None, status="out_of_range"))
        self.assertEqual(forward.action, "FORWARD_120CM")

    def test_forward_uses_120_over_35_seconds_then_returns_equal_side_time(self) -> None:
        planner = self.planner()
        forward_duration = 120 / 35
        with patch(
            "control.obstacle_avoidance.monotonic",
            side_effect=[0.0, 2.0, 2.0 + forward_duration, 3.0 + forward_duration, 4.0 + forward_duration],
        ):
            planner.plan(RCCommand(), distance(60, blocked=True))
            planner.plan(RCCommand(), distance(80))
            returning = planner.plan(RCCommand(), distance(80))
            still_returning = planner.plan(RCCommand(), distance(80))
            complete = planner.plan(RCCommand(), distance(80))
        self.assertEqual(returning.action, "SIDE_STEP_RETURN")
        self.assertEqual(returning.command.left_right, -20)
        self.assertEqual(still_returning.action, "SIDE_STEP_RETURN")
        self.assertEqual(complete.action, "BYPASS_COMPLETE")
        self.assertEqual(complete.command.left_right, 0)

    def test_forward_reblocked_pauses_progress_and_widens_route(self) -> None:
        planner = self.planner()
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 1.0, 2.0]):
            planner.plan(RCCommand(), distance(60, blocked=True))
            planner.plan(RCCommand(), distance(80))
            widened = planner.plan(RCCommand(), distance(65))
        self.assertEqual(widened.action, "SIDE_STEP_OUT")
        self.assertEqual(widened.command.left_right, 20)
        self.assertAlmostEqual(planner._bypass_forward_seconds, 1.0)

    def test_left_configuration_mirrors_lateral_commands(self) -> None:
        planner = self.planner(bypass_lateral_direction="left")
        with patch("control.obstacle_avoidance.monotonic", return_value=0.0):
            decision = planner.plan(RCCommand(), distance(60, blocked=True))
        self.assertEqual(decision.command.left_right, -20)

    def test_sidestep_timeout_fails_safe_and_stays_failed(self) -> None:
        planner = self.planner(timeout_action="land")
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 10.1, 11.0]):
            planner.plan(RCCommand(), distance(60, blocked=True))
            failed = planner.plan(RCCommand(), distance(65))
            still_failed = planner.plan(RCCommand(), distance(80))
        self.assertTrue(failed.requires_landing)
        self.assertEqual(failed.state, "FAILSAFE")
        self.assertEqual(still_failed.state, "FAILSAFE")

    def test_reset_clears_route_state(self) -> None:
        planner = self.planner()
        with patch("control.obstacle_avoidance.monotonic", return_value=0.0):
            planner.plan(RCCommand(), distance(60, blocked=True))
        planner.reset()
        decision = planner.plan(RCCommand(0, 10, 0, 0), distance(90))
        self.assertEqual(decision.action, "FOLLOW")
        self.assertIsNone(planner._bypass_phase)


if __name__ == "__main__":
    unittest.main()
