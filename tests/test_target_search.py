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
            "last_direction_seconds": 2.0,
            "sweep_short_seconds": 1.5,
            "sweep_long_seconds": 3.0,
            "close_area_ratio": 0.30,
            "close_very_area_ratio": 0.40,
            "close_pulse_seconds": 0.35,
            "close_pause_seconds": 0.50,
            "close_max_attempts": 2,
        }
        search.update(overrides)
        return TargetSearchController(
            {"target_area_ratio_max": 0.30, "target_search": search},
            min_height_cm=60,
            max_height_cm=220,
        )

    def test_visible_target_does_not_start_search(self):
        controller = self.build_controller()

        decision = controller.update(target(), 640, 480, 150, now=0.0)

        self.assertIsNone(decision)
        self.assertFalse(controller.searching)

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

    def test_vertical_loss_uses_last_vertical_direction_first(self):
        controller = self.build_controller()
        result = target(center=(320, 80), area_ratio=0.20)
        controller.observe_target(result, 640, 480, RCCommand(up_down=12))

        controller.update(LOST, 640, 480, 150, now=0.0)
        decision = controller.update(LOST, 640, 480, 150, now=1.01)

        self.assertEqual(decision.state, "SEARCH_LAST_DIRECTION")
        self.assertGreater(decision.command.up_down, 0)
        self.assertEqual(decision.command.yaw, 0)

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
        pause_one = controller.update(LOST, 640, 480, 150, now=1.37)
        controller.update(LOST, 640, 480, 150, now=1.88)
        second = controller.update(LOST, 640, 480, 150, now=1.89)
        pause_two = controller.update(LOST, 640, 480, 150, now=2.25)
        general = controller.update(LOST, 640, 480, 150, now=2.76)

        self.assertEqual(first.command.forward_backward, -10)
        self.assertEqual(pause_one.command.forward_backward, 0)
        self.assertEqual(second.command.forward_backward, -10)
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
        self.assertEqual(decision.command.forward_backward, -10)

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
        controller.state = "MOVE_UP"

        climb = controller.update(LOST, 640, 480, 190, now=1.0)
        reached = controller.update(LOST, 640, 480, 196, now=1.1)

        self.assertGreater(climb.command.up_down, 0)
        self.assertEqual(reached.command.up_down, 0)
        self.assertEqual(reached.state, "SWEEP_UP_SHORT")


if __name__ == "__main__":
    unittest.main()
