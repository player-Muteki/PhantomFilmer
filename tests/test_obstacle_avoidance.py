"""Tests for distance-only lateral/yaw avoidance."""

import unittest
from unittest.mock import patch

from control.follow_control import RCCommand
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from drone.safety import SafetyConfig, SafetyManager
from vision.obstacle_detect import ObstacleResult


def safety(max_speed: int = 35) -> SafetyManager:
    return SafetyManager(SafetyConfig(30, 20, 220, 60, max_speed, 3, 8))


def distance(cm, *, blocked=False, count=3, status="valid", sequence=0) -> ObstacleResult:
    return ObstacleResult(
        found=blocked,
        state="BLOCKED" if blocked else "CLEAR",
        confidence=1.0,
        consecutive_found_frames=count if blocked else 0,
        front_distance_cm=cm,
        front_distance_status=status,
        front_distance_sequence=sequence,
    )


class DistanceBypassPlannerTestCase(unittest.TestCase):
    def planner(self, **kwargs) -> ObstacleAvoidancePlanner:
        lateral_distance = kwargs.pop("bypass_lateral_distance_cm", 100)
        return ObstacleAvoidancePlanner(
            safety(),
            avoidance_lateral_speed=20,
            detect_confirm_frames=3,
            clearance_distance_cm=70,
            bypass_forward_distance_cm=120,
            bypass_forward_speed=35,
            bypass_lateral_distance_cm=lateral_distance,
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
                    "bypass_lateral_distance_cm": 100,
                }
            },
        )
        self.assertEqual(planner.avoidance_lateral_speed, 20)
        self.assertEqual(planner.clearance_distance_cm, 70)
        self.assertEqual(planner.bypass_lateral_direction, "right")
        self.assertEqual(planner.bypass_lateral_distance_cm, 100)

    def test_blocked_brakes_until_three_samples_then_steps_right(self) -> None:
        planner = self.planner()
        first = planner.plan(RCCommand(), distance(60, blocked=True, count=1))
        with patch("control.obstacle_avoidance.monotonic", return_value=1.0):
            confirmed = planner.plan(RCCommand(), distance(59, blocked=True, count=3))
        self.assertEqual(first.action, "BRAKE")
        self.assertEqual(first.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(confirmed.action, "SIDE_STEP_OUT")
        self.assertEqual(confirmed.command.as_tuple(), (20, 0, 0, 0))

    def test_right_shift_ignores_early_clearance_and_runs_estimated_one_metre(self) -> None:
        planner = self.planner()
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 1.0, 4.99, 5.0]):
            planner.plan(RCCommand(), distance(60, blocked=True))
            clear_early = planner.plan(RCCommand(), distance(100))
            almost = planner.plan(RCCommand(), distance(None, status="out_of_range"))
            turning = planner.plan(RCCommand(), distance(100), yaw_deg=30)
        self.assertEqual(clear_early.action, "SIDE_STEP_OUT")
        self.assertEqual(almost.action, "SIDE_STEP_OUT")
        self.assertEqual(turning.action, "POST_BYPASS_LEFT_TURN")
        self.assertEqual(turning.command.as_tuple(), (0, 0, 0, -12))
        self.assertEqual(planner._bypass_phase, "POST_BYPASS_LEFT_TURN")

    def test_out_of_range_does_not_end_fixed_right_shift_early(self) -> None:
        planner = self.planner()
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 1.0]):
            planner.plan(RCCommand(), distance(60, blocked=True))
            still_side = planner.plan(
                RCCommand(), distance(None, status="out_of_range"), yaw_deg=0
            )
        self.assertEqual(still_side.action, "SIDE_STEP_OUT")
        self.assertEqual(still_side.command.as_tuple(), (20, 0, 0, 0))

    def test_ordinary_bypass_turns_left_90_before_releasing_follow(self) -> None:
        planner = self.planner(post_bypass_turn_degrees=90, post_bypass_turn_speed=12)
        with patch(
            "control.obstacle_avoidance.monotonic",
            side_effect=[0.0, 5.0, 6.0, 7.0, 8.0],
        ):
            planner.plan(RCCommand(), distance(60, blocked=True), yaw_deg=170)
            planner.plan(RCCommand(), distance(80), yaw_deg=170)
            turning = planner.plan(RCCommand(), distance(80), yaw_deg=120)
            complete = planner.plan(RCCommand(), distance(80), yaw_deg=80)
            released = planner.plan(RCCommand(0, 25, 0, 0), distance(80), yaw_deg=79)
        self.assertEqual(turning.action, "POST_BYPASS_LEFT_TURN")
        self.assertEqual(turning.command.as_tuple(), (0, 0, 0, -12))
        self.assertEqual(complete.action, "BYPASS_COMPLETE")
        self.assertEqual(complete.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(released.action, "FOLLOW")
        self.assertEqual(released.command.forward_backward, 25)

    def test_reacquired_target_releases_follow_during_post_bypass_turn(self) -> None:
        planner = self.planner(post_bypass_turn_degrees=90, post_bypass_turn_speed=12)
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 5.0, 5.1]):
            planner.plan(RCCommand(), distance(60, blocked=True), yaw_deg=170)
            turning = planner.plan(RCCommand(), distance(80), yaw_deg=170)
            released = planner.cancel_post_bypass_turn_on_target_reacquired()
            follow = planner.plan(RCCommand(0, 25, 0, 0), distance(80), yaw_deg=169)

        self.assertEqual(turning.action, "POST_BYPASS_LEFT_TURN")
        self.assertTrue(released)
        self.assertEqual(follow.action, "FOLLOW")
        self.assertEqual(follow.command.forward_backward, 25)

    def test_left_configuration_mirrors_lateral_commands(self) -> None:
        planner = self.planner(bypass_lateral_direction="left")
        with patch("control.obstacle_avoidance.monotonic", return_value=0.0):
            decision = planner.plan(RCCommand(), distance(60, blocked=True))
        self.assertEqual(decision.command.left_right, -20)

    def test_sidestep_timeout_fails_safe_and_stays_failed(self) -> None:
        planner = self.planner(
            timeout_action="land", bypass_lateral_distance_cm=300
        )
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

    def test_lost_target_collects_five_fresh_samples_then_starts_dynamic_bypass(self) -> None:
        planner = self.planner()
        # First call establishes the current cache sequence; it is not counted.
        first = planner.plan(
            RCCommand(), distance(90, sequence=10), obstacle_priority=True, lost_episode_id=1
        )
        repeated = planner.plan(
            RCCommand(), distance(90, sequence=10), obstacle_priority=True, lost_episode_id=1
        )
        decisions = []
        with patch("control.obstacle_avoidance.monotonic", return_value=0.0):
            for index, value in enumerate((88, 90, 92, 89, 91), start=11):
                decisions.append(
                    planner.plan(
                        RCCommand(), distance(value, sequence=index),
                        obstacle_priority=True, lost_episode_id=1,
                    )
                )

        self.assertEqual(first.state, "IR_OCCLUSION_CHECK")
        self.assertEqual(repeated.reason, first.reason)
        self.assertTrue(all(item.command.as_tuple() == (0, 0, 0, 0) for item in decisions[:-1]))
        self.assertEqual(decisions[-1].action, "SIDE_STEP_OUT")
        self.assertEqual(decisions[-1].command.left_right, 20)
        self.assertEqual(planner._dynamic_reference_cm, 90.0)

    def test_lost_target_out_of_range_median_releases_normal_search(self) -> None:
        planner = self.planner()
        planner.plan(
            RCCommand(), distance(None, status="out_of_range", sequence=1),
            obstacle_priority=True, lost_episode_id=2,
        )
        final = None
        samples = (
            distance(None, status="out_of_range", sequence=2),
            distance(None, status="out_of_range", sequence=3),
            distance(None, status="out_of_range", sequence=4),
            distance(None, status="out_of_range", sequence=5),
            distance(None, status="out_of_range", sequence=6),
        )
        for sample in samples:
            final = planner.plan(
                RCCommand(), sample, obstacle_priority=True, lost_episode_id=2
            )
        self.assertIsNotNone(final)
        self.assertEqual(final.action, "SEARCH_RELEASE")
        self.assertFalse(final.owns_motion)
        self.assertIsNone(planner._bypass_phase)

    def test_target_loss_side_step_also_runs_estimated_one_metre(self) -> None:
        planner = self.planner()
        planner._dynamic_lost_bypass = True
        planner._dynamic_reference_cm = 80.0
        with patch("control.obstacle_avoidance.monotonic", side_effect=[0.0, 1.0, 4.99, 5.0]):
            planner._start_bypass(RCCommand(), distance(80, sequence=1), "1")
            one = planner.plan(RCCommand(), distance(None, status="out_of_range", sequence=2))
            two = planner.plan(RCCommand(), distance(115, sequence=3))
            three = planner.plan(RCCommand(), distance(115, sequence=4), yaw_deg=0)
        self.assertEqual(one.action, "SIDE_STEP_OUT")
        self.assertEqual(two.action, "SIDE_STEP_OUT")
        self.assertEqual(three.action, "POST_BYPASS_LEFT_TURN")
        self.assertEqual(three.command.as_tuple(), (0, 0, 0, -12))

    def test_dynamic_bypass_turns_left_90_before_releasing_search(self) -> None:
        planner = self.planner(
            post_bypass_turn_degrees=90,
            post_bypass_turn_speed=12,
        )
        planner._dynamic_lost_bypass = True
        planner._bypass_phase = "POST_BYPASS_LEFT_TURN"
        planner._phase_started_at = 0.0
        planner._turn_last_yaw = 170.0

        with patch(
            "control.obstacle_avoidance.monotonic",
            side_effect=[1.0, 2.0, 3.0, 4.0, 5.0],
        ):
            started = planner.plan(RCCommand(), distance(100), yaw_deg=140)
            turning_1 = planner.plan(RCCommand(), distance(100), yaw_deg=110)
            turning_2 = planner.plan(RCCommand(), distance(100), yaw_deg=80)
            released = planner.plan(RCCommand(), distance(100), yaw_deg=79)

        self.assertEqual(started.action, "POST_BYPASS_LEFT_TURN")
        self.assertEqual(started.command.yaw, -12)
        self.assertEqual(turning_1.command.yaw, -12)
        self.assertEqual(turning_2.action, "BYPASS_COMPLETE")
        self.assertEqual(turning_2.command.yaw, 0)
        self.assertEqual(released.action, "FOLLOW")
        self.assertFalse(released.owns_motion)

    def test_post_bypass_turn_waits_when_yaw_telemetry_is_unavailable(self) -> None:
        planner = self.planner()
        planner._dynamic_lost_bypass = True
        planner._bypass_phase = "POST_BYPASS_LEFT_TURN"
        planner._phase_started_at = 0.0
        with patch("control.obstacle_avoidance.monotonic", return_value=1.0):
            waiting = planner.plan(RCCommand(), distance(100), yaw_deg=None)
        self.assertEqual(waiting.action, "POST_BYPASS_TURN_WAIT")
        self.assertEqual(waiting.command.as_tuple(), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
