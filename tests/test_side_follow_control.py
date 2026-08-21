"""Unit tests for JointBDOE-driven 90/270-degree side following."""

import unittest

from control.follow_control import FollowController
from control.side_follow_control import SideFollowConfig, SideFollowController
from drone.safety import SafetyConfig, SafetyManager


def result(angle, *, confidence=0.9, iou=0.8, center=(320, 240), area=80_000):
    return {
        "found": True,
        "is_predicted": False,
        "center": center,
        "area": area,
        "body_orientation_angle": angle,
        "body_orientation_detection_confidence": confidence,
        "body_orientation_match_iou": iou,
    }


def controller(**overrides):
    safety = SafetyManager(SafetyConfig(20, 5, 220, 60, 35, 1, 3))
    follow = FollowController(
        safety,
        target_area_ratio_min=0.22,
        target_area_ratio_max=0.32,
    )
    settings = {
        "enabled": True,
        "orientation_stable_frames": 3,
        "lock_stable_frames": 2,
        "centered_turn_stable_frames": 1,
    }
    settings.update(overrides)
    config = SideFollowConfig(**settings)
    return SideFollowController(follow, config)


def lock_initial_side(side, *, angle=90):
    """Feed centered observations until the first side lock is confirmed."""
    for now in range(20):
        side.compute_command(result(angle), 640, 480, now)
        if side.last_debug.side_locked:
            return now + 1
    raise AssertionError("initial side did not lock")


class SideFollowControllerTestCase(unittest.TestCase):
    def test_waits_for_stable_angles_then_selects_nearest_90_side(self):
        side = controller()

        self.assertEqual(
            side.compute_command(result(78), 640, 480, 0).as_tuple(),
            (0, 0, 0, 0),
        )
        self.assertEqual(
            side.compute_command(result(82), 640, 480, 1).as_tuple(),
            (0, 0, 0, 0),
        )
        command = side.compute_command(result(80), 640, 480, 2)

        self.assertEqual(side.selected_angle, 90)
        self.assertEqual(command.left_right, 0)
        self.assertEqual(side.last_debug.state, "SIDE_TRACKING")

    def test_selects_nearest_270_side_and_never_switches_it(self):
        side = controller()
        for now, angle in enumerate((250, 252, 251)):
            side.compute_command(result(angle), 640, 480, now)

        self.assertEqual(side.selected_angle, 270)
        side.compute_command(result(100), 640, 480, 4)
        self.assertEqual(side.selected_angle, 270)

    def test_circular_samples_across_zero_are_stable_and_use_tie_break(self):
        side = controller(tie_break_target_angle=270)
        for now, angle in enumerate((359, 1, 0)):
            side.compute_command(result(angle), 640, 480, now)

        self.assertEqual(side.selected_angle, 270)

    def test_low_quality_or_predicted_angle_keeps_hovering(self):
        side = controller()

        low_confidence = side.compute_command(result(90, confidence=0.1), 640, 480, 0)
        predicted = result(90)
        predicted["is_predicted"] = True
        predicted_command = side.compute_command(predicted, 640, 480, 1)

        self.assertEqual(low_confidence.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(predicted_command.as_tuple(), (0, 0, 0, 0))
        self.assertIsNone(side.selected_angle)

    def test_enters_orbit_after_persistent_error_and_exits_when_side_recovers(self):
        side = controller()
        for now in range(3):
            side.compute_command(result(91), 640, 480, now)

        for now in range(3, 6):
            orbiting = side.compute_command(result(60), 640, 480, now)
        self.assertLess(orbiting.left_right, 0)
        self.assertEqual(side.last_debug.state, "SIDE_ORBITING")
        self.assertEqual(side.last_debug.orbit_direction, "CLOCKWISE")

        side.compute_command(result(90), 640, 480, 6)
        tracking = side.compute_command(result(90), 640, 480, 7)
        self.assertEqual(tracking.left_right, 0)
        self.assertEqual(side.last_debug.state, "SIDE_TRACKING")

    def test_runner_moving_right_causes_lateral_tracking_without_yaw(self):
        side = controller()
        for now in range(3):
            side.compute_command(result(90), 640, 480, now)

        command = side.compute_command(result(90, center=(500, 240)), 640, 480, 3)

        self.assertEqual(command.left_right, 25)
        self.assertEqual(command.yaw, 0)
        self.assertEqual(side.last_debug.state, "SIDE_TRACKING")

    def test_center_tolerance_is_reduced_by_thirty_percent_after_side_lock(self):
        side = controller()
        now = lock_initial_side(side)

        inside = side.compute_command(result(90, center=(337, 240)), 640, 480, now)
        outside = side.compute_command(result(90, center=(338, 240)), 640, 480, now + 1)

        self.assertEqual(inside.left_right, 0)
        self.assertGreater(outside.left_right, 0)
        self.assertAlmostEqual(side.last_debug.center_tolerance_ratio, 0.056)
        self.assertTrue(side.last_debug.position_priority)

    def test_position_priority_then_reselects_nearest_side_after_centering(self):
        side = controller()
        now = lock_initial_side(side)

        position_command = side.compute_command(
            result(220, center=(500, 240)), 640, 480, now
        )

        self.assertEqual(side.selected_angle, 90)
        self.assertEqual(side.last_debug.state, "SIDE_POSITION_TRACKING")
        self.assertTrue(side.last_debug.position_priority)
        self.assertTrue(side.last_debug.side_reselect_pending)
        self.assertGreater(position_command.left_right, 0)
        self.assertEqual(position_command.yaw, 0)

        centered = side.compute_command(result(220), 640, 480, now + 1)

        self.assertEqual(side.selected_angle, 270)
        self.assertEqual(side.last_debug.state, "SIDE_TRACKING")
        self.assertEqual(centered.left_right, 0)
        self.assertEqual(centered.yaw, 0)

        orbiting = side.compute_command(result(220), 640, 480, now + 2)

        self.assertEqual(side.last_debug.state, "SIDE_ORBITING")
        self.assertLess(orbiting.left_right, 0)
        self.assertGreater(orbiting.yaw, 0)

    def test_single_noisy_turn_frame_does_not_start_orbit(self):
        side = controller()
        for now in range(3):
            side.compute_command(result(90), 640, 480, now)

        command = side.compute_command(result(50), 640, 480, 3)

        self.assertEqual(command.left_right, 0)
        self.assertEqual(command.yaw, 0)
        self.assertEqual(side.last_debug.state, "SIDE_TRACKING")

    def test_centered_turn_must_stabilize_before_reselecting_and_orbiting(self):
        side = controller(
            centered_turn_stable_frames=3,
            centered_turn_max_deviation_deg=2.0,
        )
        now = lock_initial_side(side)

        for offset, angle in enumerate((130, 150, 170, 220, 220), start=0):
            command = side.compute_command(result(angle), 640, 480, now + offset)
            self.assertEqual(command.left_right, 0)
            self.assertEqual(command.yaw, 0)
            self.assertEqual(side.last_debug.state, "SIDE_TURN_STABILIZING")
            self.assertEqual(side.selected_angle, 90)

        stable = side.compute_command(result(221), 640, 480, now + 5)
        self.assertEqual(side.selected_angle, 270)
        self.assertTrue(side.last_debug.centered_angle_stable)
        self.assertEqual(stable.left_right, 0)

        orbiting = side.compute_command(result(220), 640, 480, now + 6)
        self.assertEqual(side.last_debug.state, "SIDE_ORBITING")
        self.assertNotEqual(orbiting.left_right, 0)

    def test_larger_target_angle_orbits_clockwise_to_increase_angle(self):
        side = controller()
        for now in range(5):
            command = side.compute_command(result(40), 640, 480, now)

        self.assertLess(command.left_right, 0)
        self.assertGreater(command.yaw, 0)
        self.assertEqual(side.last_debug.orbit_direction, "CLOCKWISE")

    def test_orbit_uses_faster_gain_and_speed_limit(self):
        side = controller(orbit_entry_frames=1)
        for now in range(3):
            command = side.compute_command(result(40), 640, 480, now)

        self.assertEqual(command.left_right, -18)
        self.assertEqual(command.yaw, 14)
        self.assertEqual(side.last_debug.yaw_feedforward, 14)
        self.assertEqual(side.last_debug.yaw_feedback, 0)

        limited = side.compute_command(result(350), 640, 480, 4)
        self.assertEqual(limited.left_right, -25)

    def test_smaller_target_angle_orbits_counterclockwise_to_decrease_angle(self):
        side = controller()
        for now in range(5):
            command = side.compute_command(result(140), 640, 480, now)

        self.assertGreater(command.left_right, 0)
        self.assertLess(command.yaw, 0)
        self.assertEqual(side.last_debug.orbit_direction, "COUNTERCLOCKWISE")

    def test_orbit_yaw_combines_feedforward_and_center_feedback_with_limit(self):
        side = controller(orbit_entry_frames=1)
        off_center = result(40, center=(600, 240))
        for now in range(3):
            command = side.compute_command(off_center, 640, 480, now)

        self.assertGreater(side.last_debug.yaw_feedforward, 0)
        self.assertGreater(side.last_debug.yaw_feedback, 0)
        self.assertEqual(command.yaw, 30)

    def test_keeps_center_distance_and_height_axes_while_orbiting(self):
        side = controller()
        off_center = result(60, center=(500, 100), area=20_000)
        for now in range(5):
            command = side.compute_command(off_center, 640, 480, now)

        self.assertGreater(command.left_right, 0)
        self.assertGreater(command.yaw, 0)
        self.assertGreater(command.forward_backward, 0)
        self.assertGreater(command.up_down, 0)

    def test_unfinished_orbit_continues_without_time_limit(self):
        side = controller()
        for now in range(5):
            side.compute_command(result(10), 640, 480, now)

        continuing = side.compute_command(result(10), 640, 480, 10_000.0)

        self.assertNotEqual(continuing.left_right, 0)
        self.assertEqual(side.last_debug.state, "SIDE_ORBITING")

    def test_manual_suspend_can_preserve_selected_side(self):
        side = controller()
        for now in range(3):
            side.compute_command(result(250), 640, 480, now)
        side.reset(preserve_selection=True)

        self.assertEqual(side.selected_angle, 270)
        side.compute_command(result(100), 640, 480, 10)
        self.assertEqual(side.selected_angle, 270)


if __name__ == "__main__":
    unittest.main()
