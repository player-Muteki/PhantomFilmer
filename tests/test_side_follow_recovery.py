"""Tests for bounded target-loss recovery in side-follow mode."""

import unittest

from control.side_follow_recovery import (
    SideFollowRecoveryConfig,
    SideFollowRecoveryController,
)


def target(center_x: int, *, found: bool = True) -> dict[str, object]:
    return {
        "found": found,
        "is_predicted": False,
        "ambiguous": False,
        "center": (center_x, 240),
    }


class SideFollowRecoveryControllerTestCase(unittest.TestCase):
    def controller(self) -> SideFollowRecoveryController:
        return SideFollowRecoveryController(
            SideFollowRecoveryConfig(
                lateral_search_seconds=3.0,
                lateral_search_speed=20,
                rotation_yaw_speed=30,
                rotation_degrees=360.0,
                rotation_fallback_seconds=12.0,
                final_hover_seconds=3.0,
            )
        )

    def test_moves_toward_right_exit_then_rotates_right_once(self) -> None:
        recovery = self.controller()
        recovery.observe_target(target(600), 640)

        lateral = recovery.update(target(0, found=False), 640, 10.0, yaw_deg=170)
        self.assertIsNotNone(lateral)
        assert lateral is not None
        self.assertEqual(lateral.state, "SIDE_LOST_LATERAL")
        self.assertEqual(lateral.command.as_tuple(), (20, 0, 0, 0))

        rotating = recovery.update(target(0, found=False), 640, 13.0, yaw_deg=170)
        self.assertIsNotNone(rotating)
        assert rotating is not None
        self.assertEqual(rotating.state, "SIDE_LOST_ROTATING")
        self.assertEqual(rotating.command.yaw, 30)

        recovery.update(target(0, found=False), 640, 13.1, yaw_deg=-170)
        recovery.update(target(0, found=False), 640, 13.2, yaw_deg=-10)
        recovery.update(target(0, found=False), 640, 13.3, yaw_deg=169)
        hover = recovery.update(target(0, found=False), 640, 13.4, yaw_deg=170)

        self.assertIsNotNone(hover)
        assert hover is not None
        self.assertEqual(hover.state, "SIDE_LOST_FINAL_HOVER")
        self.assertEqual(hover.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(recovery.rotation_progress_degrees, 360.0)

        landing = recovery.update(target(0, found=False), 640, 16.5, yaw_deg=170)
        self.assertIsNotNone(landing)
        assert landing is not None
        self.assertEqual(landing.state, "SIDE_LOST_LANDING")
        self.assertTrue(landing.requires_landing)

    def test_left_exit_uses_negative_lateral_and_yaw_commands(self) -> None:
        recovery = self.controller()
        recovery.observe_target(target(40), 640)

        lateral = recovery.update(target(0, found=False), 640, 1.0, yaw_deg=0)
        rotating = recovery.update(target(0, found=False), 640, 4.0, yaw_deg=0)

        assert lateral is not None and rotating is not None
        self.assertEqual(lateral.command.left_right, -20)
        self.assertEqual(rotating.command.yaw, -30)

    def test_reacquisition_stops_blind_motion_immediately(self) -> None:
        recovery = self.controller()
        recovery.observe_target(target(600), 640)
        recovery.update(target(0, found=False), 640, 1.0, yaw_deg=0)

        decision = recovery.update(target(500), 640, 1.1, yaw_deg=0)

        self.assertIsNone(decision)
        self.assertEqual(recovery.state, "IDLE")
        self.assertTrue(recovery.has_observed_target)
        self.assertEqual(recovery.last_horizontal_direction, 1)

    def test_rotation_uses_time_fallback_when_yaw_is_unavailable(self) -> None:
        recovery = self.controller()
        recovery.observe_target(target(600), 640)
        recovery.update(target(0, found=False), 640, 0.0, yaw_deg=None)
        recovery.update(target(0, found=False), 640, 3.0, yaw_deg=None)

        rotating = recovery.update(target(0, found=False), 640, 14.9, yaw_deg=None)
        hover = recovery.update(target(0, found=False), 640, 15.0, yaw_deg=None)

        assert rotating is not None and hover is not None
        self.assertEqual(rotating.state, "SIDE_LOST_ROTATING")
        self.assertEqual(hover.state, "SIDE_LOST_FINAL_HOVER")

    def test_never_seen_target_hovers_then_lands_without_blind_translation(
        self,
    ) -> None:
        recovery = self.controller()

        hover = recovery.update(target(0, found=False), 640, 1.0, yaw_deg=0)
        landing = recovery.update(target(0, found=False), 640, 4.0, yaw_deg=0)

        assert hover is not None and landing is not None
        self.assertEqual(hover.state, "SIDE_LOST_FINAL_HOVER")
        self.assertEqual(hover.command.as_tuple(), (0, 0, 0, 0))
        self.assertTrue(landing.requires_landing)


if __name__ == "__main__":
    unittest.main()
