"""Tests for obstacle-avoidance command arbitration."""

import unittest

from control.follow_control import RCCommand
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from drone.safety import SafetyConfig, SafetyManager
from vision.obstacle_detect import ObstacleResult


def build_safety(max_speed: int = 35) -> SafetyManager:
    return SafetyManager(SafetyConfig(30, 20, 150, 60, max_speed, 3, 8))


class ObstacleAvoidancePlannerTestCase(unittest.TestCase):
    def test_clear_returns_follow_command(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety())
        command = RCCommand(0, 20, 0, 5)
        decision = planner.plan(command, ObstacleResult(found=False, state="CLEAR"))
        self.assertEqual(decision.command.as_tuple(), command.as_tuple())
        self.assertEqual(decision.state, "CLEAR")

    def test_caution_reduces_forward_speed(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety(), forward_speed_in_caution_ratio=0.25)
        decision = planner.plan(
            RCCommand(0, 20, 0, 6),
            ObstacleResult(found=True, state="CAUTION", side="center"),
        )
        self.assertEqual(decision.command.as_tuple(), (0, 5, 0, 6))
        self.assertEqual(decision.state, "CAUTION")

    def test_caution_does_not_reduce_backward_speed(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety(), forward_speed_in_caution_ratio=0.25)
        decision = planner.plan(
            RCCommand(0, -20, 0, 6),
            ObstacleResult(found=True, state="CAUTION", side="center"),
        )
        self.assertEqual(decision.command.as_tuple(), (0, -20, 0, 6))

    def test_left_obstacle_turns_right_without_forward(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety(), avoidance_yaw_speed=18)
        decision = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(found=True, state="BLOCKED", side="left"),
        )
        self.assertEqual(decision.command.forward_backward, 0)
        self.assertGreater(decision.command.yaw, 0)
        self.assertEqual(decision.state, "AVOIDING")

    def test_right_obstacle_turns_left_without_forward(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety(), avoidance_yaw_speed=18)
        decision = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(found=True, state="BLOCKED", side="right"),
        )
        self.assertEqual(decision.command.forward_backward, 0)
        self.assertLess(decision.command.yaw, 0)

    def test_center_obstacle_uses_last_direction(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety(), avoidance_yaw_speed=18)
        planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=True, state="BLOCKED", side="right"))
        decision = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(found=True, state="BLOCKED", side="center"),
        )
        self.assertLess(decision.command.yaw, 0)

    def test_recovery_requires_clear_frames(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety(), recovery_clear_frames=2)
        planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=True, state="BLOCKED", side="left"))
        first_clear = planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=False, state="CLEAR"))
        second_clear = planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=False, state="CLEAR"))
        self.assertEqual(first_clear.state, "RECOVERING")
        self.assertEqual(first_clear.command.forward_backward, 0)
        self.assertEqual(second_clear.state, "CLEAR")
        self.assertEqual(second_clear.command.forward_backward, 20)

    def test_outputs_are_safety_limited(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety(max_speed=10), avoidance_yaw_speed=30)
        decision = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(found=True, state="BLOCKED", side="left"),
        )
        self.assertEqual(decision.command.yaw, 10)

    def test_stationary_command_without_priority_brakes(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety())
        decision = planner.plan(
            RCCommand(),
            ObstacleResult(found=True, state="BLOCKED", side="left"),
        )
        self.assertEqual(decision.state, "BRAKING")
        self.assertEqual(decision.command.as_tuple(), (0, 0, 0, 0))

    def test_stationary_command_with_obstacle_priority_detours(self) -> None:
        planner = ObstacleAvoidancePlanner(
            build_safety(),
            avoidance_yaw_speed=18,
            avoidance_lateral_speed=12,
        )
        decision = planner.plan(
            RCCommand(),
            ObstacleResult(found=True, state="BLOCKED", side="left"),
            obstacle_priority=True,
        )
        self.assertEqual(decision.state, "AVOIDING")
        self.assertEqual(decision.command.forward_backward, 0)
        self.assertNotEqual(decision.command.left_right, 0)
        self.assertNotEqual(decision.command.yaw, 0)

    def test_obstacle_priority_still_confirms_then_detours(self) -> None:
        planner = ObstacleAvoidancePlanner(
            build_safety(),
            avoidance_yaw_speed=18,
            avoidance_lateral_speed=12,
            detect_confirm_frames=3,
        )
        first = planner.plan(
            RCCommand(),
            ObstacleResult(found=True, state="BLOCKED", side="left", consecutive_found_frames=1),
            obstacle_priority=True,
        )
        confirmed = planner.plan(
            RCCommand(),
            ObstacleResult(found=True, state="BLOCKED", side="left", consecutive_found_frames=3),
            obstacle_priority=True,
        )
        self.assertEqual(first.state, "BRAKING")
        self.assertEqual(first.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(confirmed.state, "AVOIDING")
        self.assertNotEqual(confirmed.command.left_right, 0)

    def test_caution_with_obstacle_priority_hovers(self) -> None:
        planner = ObstacleAvoidancePlanner(build_safety())
        decision = planner.plan(
            RCCommand(),
            ObstacleResult(found=True, state="CAUTION", side="center"),
            obstacle_priority=True,
        )
        self.assertEqual(decision.state, "CAUTION")
        self.assertEqual(decision.command.as_tuple(), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
