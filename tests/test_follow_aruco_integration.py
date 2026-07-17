"""Verify ArUco-compatible result dictionaries work with FollowController."""

import unittest

from control.follow_control import FollowController
from drone.safety import SafetyConfig, SafetyManager


class FollowArucoIntegrationTestCase(unittest.TestCase):
    frame_width = 640
    frame_height = 480

    def setUp(self) -> None:
        safety = SafetyManager(
            SafetyConfig(30, 20, 150, 60, 35, 3, 8)
        )
        self.controller = FollowController(safety_manager=safety)

    @staticmethod
    def aruco_result(center, area):
        x, y = center
        return {
            "found": True,
            "is_predicted": False,
            "center": center,
            "target_center_x": x,
            "target_center_y": y,
            "area": area,
            "bbox": (x - 10, y - 10, 20, 20),
            "marker_id": 23,
            "corners": None,
            "detector_type": "aruco",
        }

    def command_for(self, center, area=6000):
        return self.controller.compute_command(
            self.aruco_result(center, area), self.frame_width, self.frame_height
        )

    def test_left_marker_yaws_left(self) -> None:
        self.assertLess(self.command_for((120, 240)).yaw, 0)

    def test_right_marker_yaws_right(self) -> None:
        self.assertGreater(self.command_for((520, 240)).yaw, 0)

    def test_upper_and_lower_markers_control_height(self) -> None:
        self.assertGreater(self.command_for((320, 100)).up_down, 0)
        self.assertLess(self.command_for((320, 380)).up_down, 0)

    def test_small_and_large_marker_areas_control_distance(self) -> None:
        self.assertGreater(self.command_for((320, 240), area=100).forward_backward, 0)
        self.assertLess(self.command_for((320, 240), area=40000).forward_backward, 0)

    def test_off_center_marker_adjusts_distance_at_slow_speed_while_turning(self) -> None:
        command = self.command_for((120, 240), area=100)
        self.assertLess(command.yaw, 0)
        self.assertEqual(command.forward_backward, self.controller.minimum_forward_speed)

    def test_height_is_aligned_before_distance_adjustment(self) -> None:
        command = self.command_for((320, 100), area=100)
        self.assertGreater(command.up_down, 0)
        self.assertEqual(command.forward_backward, 0)

    def test_lost_marker_hovers(self) -> None:
        command = self.controller.compute_command(
            {
                "found": False,
                "is_predicted": False,
                "center": None,
                "area": 0.0,
                "bbox": None,
                "detector_type": "aruco",
            },
            self.frame_width,
            self.frame_height,
        )
        self.assertEqual(command.as_tuple(), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
