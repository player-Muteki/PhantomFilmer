"""Tests for bounded ReID target-loss search and close-target recovery."""

import unittest

from control.follow_control import RCCommand
from control.target_search import TargetSearchController


def target(
    *,
    center=(500, 240),
    area_ratio=0.20,
    bbox=(430, 80, 140, 320),
):
    return {
        "found": True,
        "is_predicted": False,
        "ambiguous": False,
        "center": center,
        "area_ratio": area_ratio,
        "area": int(640 * 480 * area_ratio),
        "bbox": bbox,
    }


LOST = {
    "found": False,
    "is_predicted": False,
    "ambiguous": False,
    "center": None,
    "area_ratio": 0.0,
    "area": 0,
    "bbox": None,
}


class TargetSearchControllerTests(unittest.TestCase):
    def build_controller(self, **overrides):
        search = {
            "hold_seconds": 1.0,
            "total_timeout_seconds": 30.0,
            "reacquire_frames": 5,
            "last_direction_yaw_speed": 25,
            "yaw_speed": 20,
            "vertical_speed": 20,
            "last_direction_seconds": 2.0,
            "full_rotation_degrees": 360,
            "full_rotation_fallback_seconds": 18.0,
            "close_area_ratio": 0.35,
            "close_very_area_ratio": 0.40,
            "close_backward_speed": 35,
            "close_pulse_seconds": 1.5,
            "close_pause_seconds": 0.50,
            "close_max_attempts": 2,
        }
        search.update(overrides)
        return TargetSearchController(
            {"target_area_ratio_max": 0.35, "target_search": search},
            min_height_cm=60,
            max_height_cm=220,
        )

    def test_visible_target_does_not_start_search(self):
        controller = self.build_controller()

        decision = controller.update(target(), 640, 480, 150, now=0.0)

        self.assertIsNone(decision)
        self.assertFalse(controller.searching)

    def test_never_seen_target_skips_nonexistent_last_direction(self):
        controller = self.build_controller()
        controller.update(LOST, 640, 480, 150, now=0.0)

        layer_start = controller.update(LOST, 640, 480, 150, now=1.01)
        first_sweep = controller.update(LOST, 640, 480, 150, now=1.02)

        self.assertEqual(layer_start.state, "LAYER_SCAN_FULL")
        self.assertEqual(layer_start.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(first_sweep.command.yaw, 20)

    def test_ordinary_loss_holds_then_turns_toward_last_horizontal_direction(self):
        controller = self.build_controller()
        result = target(center=(560, 240), area_ratio=0.20)
        controller.observe_target(result, 640, 480, RCCommand(yaw=20))

        hold = controller.update(LOST, 640, 480, 150, now=0.0)
        turn = controller.update(LOST, 640, 480, 150, now=1.01)

        self.assertEqual(hold.state, "LOST_HOLD")
        self.assertEqual(hold.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(turn.state, "SEARCH_LAST_DIRECTION")
        self.assertGreater(turn.command.yaw, 0)
        self.assertEqual(turn.command.left_right, 0)
        self.assertEqual(turn.command.forward_backward, 0)

    def test_vertical_position_does_not_create_a_separate_vertical_branch(self):
        controller = self.build_controller()
        result = target(center=(100, 80), area_ratio=0.20)
        controller.observe_target(result, 640, 480, RCCommand(up_down=12))

        controller.update(LOST, 640, 480, 150, now=0.0)
        decision = controller.update(LOST, 640, 480, 150, now=1.01)

        self.assertEqual(decision.state, "SEARCH_LAST_DIRECTION")
        self.assertEqual(decision.command.up_down, 0)
        self.assertEqual(decision.command.yaw, -25)

    def test_close_loss_uses_two_bounded_backward_pulses_then_general_search(self):
        controller = self.build_controller()
        close = target(
            center=(320, 240),
            area_ratio=0.35,
            bbox=(0, 20, 620, 450),
        )
        controller.observe_target(close, 640, 480, RCCommand(forward_backward=16))

        controller.update(LOST, 640, 480, 150, now=0.0)
        first = controller.update(LOST, 640, 480, 150, now=1.01)
        pause_one = controller.update(LOST, 640, 480, 150, now=2.52)
        controller.update(LOST, 640, 480, 150, now=3.03)
        second = controller.update(LOST, 640, 480, 150, now=3.04)
        pause_two = controller.update(LOST, 640, 480, 150, now=4.55)
        general = controller.update(LOST, 640, 480, 150, now=5.06)

        self.assertEqual(first.command.forward_backward, -35)
        self.assertEqual(pause_one.command.forward_backward, 0)
        self.assertEqual(second.command.forward_backward, -35)
        self.assertEqual(pause_two.command.forward_backward, 0)
        self.assertEqual(general.state, "SEARCH_LAST_DIRECTION")
        self.assertEqual(general.command.forward_backward, 0)

    def test_controller_backing_away_is_evidence_target_was_too_close(self):
        controller = self.build_controller()
        close = target(
            center=(320, 240),
            area_ratio=0.35,
            bbox=(180, 60, 280, 360),
        )
        controller.observe_target(close, 640, 480, RCCommand(forward_backward=-16))

        controller.update(LOST, 640, 480, 150, now=0.0)
        decision = controller.update(LOST, 640, 480, 150, now=1.01)

        self.assertEqual(decision.state, "CLOSE_BACKOFF")
        self.assertEqual(decision.command.forward_backward, -35)

    def test_last_direction_and_layer_sweeps_use_separate_yaw_speeds(self):
        controller = self.build_controller()
        controller.observe_target(target(center=(560, 240)), 640, 480, RCCommand())
        controller.update(LOST, 640, 480, 150, now=0.0)

        last_direction = controller.update(LOST, 640, 480, 150, now=1.01)
        controller.update(LOST, 640, 480, 150, now=3.02)
        controller.update(LOST, 640, 480, 150, now=3.03)
        layer_sweep = controller.update(LOST, 640, 480, 150, now=3.04)

        self.assertEqual(last_direction.command.yaw, 25)
        self.assertEqual(layer_sweep.command.yaw, 20)

    def test_full_rotation_uses_wrapped_yaw_telemetry_until_360_degrees(self):
        controller = self.build_controller(total_timeout_seconds=100)
        controller.search_height_cm = 150
        controller.search_started_at = 0.0
        controller.phase_started_at = 0.0
        controller.state = "LAYER_SCAN_FULL"
        controller.layer_index = 0

        headings = [170, -170, -90, 0, 90]
        decisions = [
            controller.update(LOST, 640, 480, 150, now=index * 0.1, yaw_deg=heading)
            for index, heading in enumerate(headings)
        ]
        completed = controller.update(
            LOST, 640, 480, 150, now=0.6, yaw_deg=-170
        )

        self.assertTrue(all(item.command.yaw == 20 for item in decisions))
        self.assertGreaterEqual(controller.rotation_progress_degrees, 360)
        self.assertEqual(completed.state, "MOVE_TO_LAYER")
        self.assertEqual(completed.command.yaw, 0)

    def test_adjacent_layers_reverse_full_rotation_direction(self):
        controller = self.build_controller(total_timeout_seconds=100)
        controller.search_height_cm = 150
        controller.search_started_at = 0.0
        controller.phase_started_at = 0.0
        controller.state = "LAYER_SCAN_FULL"

        controller.layer_index = 0
        current = controller.update(LOST, 640, 480, 150, now=0.0, yaw_deg=0)
        controller.layer_index = 1
        controller._reset_rotation_tracking()
        upper = controller.update(LOST, 640, 480, 170, now=0.1, yaw_deg=0)

        self.assertEqual(current.command.yaw, 20)
        self.assertEqual(upper.command.yaw, -20)

    def test_reacquisition_requires_five_consecutive_fresh_matches(self):
        controller = self.build_controller()
        controller.observe_target(target(), 640, 480, RCCommand())
        controller.update(LOST, 640, 480, 150, now=0.0)

        for index in range(4):
            decision = controller.update(target(), 640, 480, 150, now=0.1 + index * 0.1)
            self.assertEqual(decision.action, "search")
            self.assertEqual(decision.state, "REACQUIRE_VERIFY")

        decision = controller.update(target(), 640, 480, 150, now=0.5)
        self.assertEqual(decision.action, "reacquired")
        self.assertFalse(controller.searching)

    def test_interrupted_reacquisition_resets_progress_and_resumes_search(self):
        controller = self.build_controller()
        controller.observe_target(target(), 640, 480, RCCommand())
        controller.update(LOST, 640, 480, 150, now=0.0)
        controller.update(target(), 640, 480, 150, now=0.2)
        controller.update(target(), 640, 480, 150, now=0.3)

        resumed = controller.update(LOST, 640, 480, 150, now=0.4)
        restarted = controller.update(target(), 640, 480, 150, now=0.5)

        self.assertEqual(resumed.state, "LOST_HOLD")
        self.assertEqual(restarted.reason, "ReID 1/5")

    def test_failed_reacquisition_preserves_full_rotation_progress(self):
        controller = self.build_controller(total_timeout_seconds=100)
        controller.search_height_cm = 150
        controller.search_started_at = 0.0
        controller.phase_started_at = 0.0
        controller.state = "LAYER_SCAN_FULL"
        controller.update(LOST, 640, 480, 150, now=0.0, yaw_deg=0)
        controller.update(LOST, 640, 480, 150, now=0.1, yaw_deg=90)
        before = controller.rotation_progress_degrees
        controller.update(target(), 640, 480, 150, now=0.2, yaw_deg=90)

        resumed = controller.update(LOST, 640, 480, 150, now=0.5, yaw_deg=90)

        self.assertEqual(before, 90)
        self.assertEqual(controller.rotation_progress_degrees, 90)
        self.assertEqual(resumed.state, "LAYER_SCAN_FULL")
        self.assertEqual(resumed.command.yaw, 20)

    def test_search_timeout_requests_landing(self):
        controller = self.build_controller()
        controller.observe_target(target(), 640, 480, RCCommand())
        controller.update(LOST, 640, 480, 150, now=0.0)

        decision = controller.update(LOST, 640, 480, 150, now=30.01)

        self.assertEqual(decision.action, "land")
        self.assertEqual(decision.command.as_tuple(), (0, 0, 0, 0))

    def test_layer_targets_are_clamped_to_configured_height_bounds(self):
        controller = self.build_controller(min_height_cm=80, max_height_cm=200)
        controller.search_height_cm = 195
        controller.search_started_at = 0.0
        controller.phase_started_at = 0.0
        controller.layer_index = 1
        controller.state = "MOVE_TO_LAYER"

        climb = controller.update(LOST, 640, 480, 190, now=1.0)
        reached = controller.update(LOST, 640, 480, 196, now=1.1)

        self.assertGreater(climb.command.up_down, 0)
        self.assertEqual(reached.command.up_down, 0)
        self.assertEqual(reached.state, "LAYER_SCAN_FULL")

    def test_fixed_layer_order_is_current_then_upper_then_lower(self):
        controller = self.build_controller(
            total_timeout_seconds=100,
            full_rotation_fallback_seconds=1.0,
        )
        controller.search_height_cm = 150
        controller.search_started_at = 0.0
        controller.phase_started_at = 0.0
        controller.layer_index = 0
        controller.state = "MOVE_TO_LAYER"

        current = controller.update(LOST, 640, 480, 150, now=0.0)
        controller.update(LOST, 640, 480, 150, now=0.1)
        controller.update(LOST, 640, 480, 150, now=1.01)
        upward = controller.update(LOST, 640, 480, 150, now=1.02)
        upper = controller.update(LOST, 640, 480, 170, now=1.03)
        controller.update(LOST, 640, 480, 170, now=1.04)
        controller.update(LOST, 640, 480, 170, now=2.04)
        downward = controller.update(LOST, 640, 480, 170, now=2.05)
        lower = controller.update(LOST, 640, 480, 130, now=2.06)

        self.assertEqual(current.state, "LAYER_SCAN_FULL")
        self.assertEqual(upward.command.up_down, 20)
        self.assertEqual(upper.state, "LAYER_SCAN_FULL")
        self.assertEqual(downward.command.up_down, -20)
        self.assertEqual(lower.state, "LAYER_SCAN_FULL")


if __name__ == "__main__":
    unittest.main()
