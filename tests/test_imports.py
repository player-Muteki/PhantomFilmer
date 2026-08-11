"""Import tests for the DroneUmbrella project skeleton."""

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


class ImportTestCase(unittest.TestCase):
    """Verify that all project modules can be imported."""

    def test_import_all_modules(self) -> None:
        import agent.agent_controller
        import agent.command_parser
        import agent.commands
        import agent.llm_client
        import agent.tools
        import control.follow_session
        import control.follow_control
        import drone.drone_adapter
        import drone.fake_adapter
        import drone.safety
        import drone.tello_adapter
        import main
        import swarm.formation_sim
        import swarm.fake_swarm
        import swarm.formation_control
        import swarm.swarm_manager
        import swarm.swarm_node
        import swarm.swarm_safety
        import vision.camera
        import vision.target_detect
        import vision.aruco_detect
        import vision.detector_factory

        self.assertIsNotNone(main)
        self.assertIsNotNone(agent.agent_controller)
        self.assertIsNotNone(agent.command_parser)
        self.assertIsNotNone(agent.commands)
        self.assertIsNotNone(agent.llm_client)
        self.assertIsNotNone(agent.tools)
        self.assertIsNotNone(control.follow_session)
        self.assertIsNotNone(control.follow_control)
        self.assertIsNotNone(drone.drone_adapter)
        self.assertIsNotNone(drone.fake_adapter)
        self.assertIsNotNone(drone.safety)
        self.assertIsNotNone(drone.tello_adapter)
        self.assertIsNotNone(swarm.formation_sim)
        self.assertIsNotNone(swarm.fake_swarm)
        self.assertIsNotNone(swarm.formation_control)
        self.assertIsNotNone(swarm.swarm_manager)
        self.assertIsNotNone(swarm.swarm_node)
        self.assertIsNotNone(swarm.swarm_safety)
        self.assertIsNotNone(vision.camera)
        self.assertIsNotNone(vision.target_detect)
        self.assertIsNotNone(vision.aruco_detect)
        self.assertIsNotNone(vision.detector_factory)

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

    def test_detector_factory_supports_configured_detectors(self) -> None:
        from vision.aruco_detect import ArucoTargetDetector
        from vision.detector_factory import create_detector
        from vision.target_detect import TargetDetector

        self.assertIsInstance(create_detector({}), TargetDetector)
        self.assertIsInstance(
            create_detector({"vision": {"detector_type": "aruco"}}),
            ArucoTargetDetector,
        )

        with self.assertRaises(ValueError):
            create_detector({"vision": {"detector_type": "unsupported"}})


if __name__ == "__main__":
    unittest.main()
