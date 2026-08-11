"""Import tests for the PhantomFilmer project skeleton."""

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


class ImportTestCase(unittest.TestCase):
    """Verify that all project modules can be imported."""

    def test_import_all_modules(self) -> None:
        import console.console_controller
        import console.command_parser
        import console.commands
        import console.follow_session
        import console.llm_client
        import console.tools
        import control.follow_session
        import control.follow_control
        import control.obstacle_avoidance
        import drone.drone_adapter
        import drone.fake_adapter
        import drone.safety
        import drone.tello_adapter
        import main
        import ui.dashboard
        import vision.camera
        import vision.target_detect
        import vision.aruco_detect
        import vision.detector_factory
        import vision.obstacle_detect

        self.assertIsNotNone(main)
        self.assertIsNotNone(console.console_controller)
        self.assertIsNotNone(console.command_parser)
        self.assertIsNotNone(console.commands)
        self.assertIsNotNone(console.follow_session)
        self.assertIsNotNone(console.llm_client)
        self.assertIsNotNone(console.tools)
        self.assertIsNotNone(control.follow_session)
        self.assertIsNotNone(control.follow_control)
        self.assertIsNotNone(control.obstacle_avoidance)
        self.assertIsNotNone(drone.drone_adapter)
        self.assertIsNotNone(drone.fake_adapter)
        self.assertIsNotNone(drone.safety)
        self.assertIsNotNone(drone.tello_adapter)
        self.assertIsNotNone(ui.dashboard)
        self.assertIsNotNone(vision.camera)
        self.assertIsNotNone(vision.target_detect)
        self.assertIsNotNone(vision.aruco_detect)
        self.assertIsNotNone(vision.detector_factory)
        self.assertIsNotNone(vision.obstacle_detect)

    def test_safety_manager_import(self) -> None:
        from drone.safety import SafetyConfig, SafetyManager

        config = SafetyConfig(
            min_battery_takeoff=30,
            low_battery_land=20,
            max_height_cm=150,
            min_height_cm=60,
            max_rc_speed=25,
            target_lost_hover_seconds=3,
            target_lost_land_seconds=8,
        )
        safety = SafetyManager(config)
        self.assertTrue(safety.can_takeoff(30))
        self.assertFalse(safety.can_takeoff(29))
        self.assertEqual(safety.limit_rc_command(-100, -30, 0, 88), (-25, -25, 0, 25))


if __name__ == "__main__":
    unittest.main()
