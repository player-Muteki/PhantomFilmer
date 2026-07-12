import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from swarm.fake_swarm import create_fake_swarm_nodes
from swarm.swarm_safety import SwarmSafetyConfig, SwarmSafetyManager


class SwarmSafetyManagerTestCase(unittest.TestCase):
    def test_limits_each_rc_channel_independently(self) -> None:
        safety = SwarmSafetyManager(SwarmSafetyConfig(30, 10, 11, 12, 13))

        self.assertEqual(safety.limit_rc_command((-50, 50, -50, 50)), (-10, 11, -12, 13))

    def test_low_battery_blocks_takeoff(self) -> None:
        nodes = create_fake_swarm_nodes()
        for node in nodes:
            node.connect()
        nodes[2].adapter.battery_percent = 20
        safety = SwarmSafetyManager(SwarmSafetyConfig(minimum_takeoff_battery=30))

        blockers = safety.check_takeoff_allowed(nodes)

        self.assertIn("drone_3", blockers)

    def test_emergency_blocks_nonzero_rc(self) -> None:
        nodes = create_fake_swarm_nodes()
        for node in nodes:
            node.connect()
        safety = SwarmSafetyManager(SwarmSafetyConfig())

        self.assertTrue(safety.allow_nonzero_rc(nodes))
        safety.activate_emergency()
        self.assertFalse(safety.allow_nonzero_rc(nodes))


if __name__ == "__main__":
    unittest.main()
