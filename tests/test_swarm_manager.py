import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from swarm.fake_swarm import create_fake_swarm_nodes
from swarm.swarm_manager import SwarmManager
from swarm.swarm_safety import SwarmSafetyConfig, SwarmSafetyManager


def build_manager(nodes=None) -> SwarmManager:
    return SwarmManager(
        nodes=nodes or create_fake_swarm_nodes(),
        safety_manager=SwarmSafetyManager(SwarmSafetyConfig()),
        command_interval_ms=0,
        takeoff_interval_s=0,
    )


class SwarmManagerTestCase(unittest.TestCase):
    def test_rejects_duplicate_node_ids(self) -> None:
        nodes = create_fake_swarm_nodes()
        nodes[1].drone_id = nodes[0].drone_id

        with self.assertRaises(ValueError):
            build_manager(nodes)

    def test_connect_all_returns_per_node_results(self) -> None:
        manager = build_manager()
        result = manager.connect_all()

        self.assertTrue(result.success)
        self.assertEqual(set(result.results), {"drone_1", "drone_2", "drone_3", "drone_4"})

    def test_one_connect_failure_does_not_crash_other_nodes(self) -> None:
        manager = build_manager(create_fake_swarm_nodes(failing_ids=["drone_2"]))
        result = manager.connect_all()

        self.assertFalse(result.success)
        self.assertFalse(result.results["drone_2"].success)
        self.assertTrue(result.results["drone_1"].success)
        self.assertTrue(result.results["drone_3"].success)

    def test_status_all_returns_independent_status(self) -> None:
        nodes = create_fake_swarm_nodes()
        manager = build_manager(nodes)
        manager.connect_all()
        nodes[0].adapter.battery_percent = 71
        nodes[1].adapter.battery_percent = 72

        result = manager.status_all()

        self.assertEqual(result.results["drone_1"].status.battery, 71)
        self.assertEqual(result.results["drone_2"].status.battery, 72)

    def test_send_rc_all_limits_all_nodes(self) -> None:
        nodes = create_fake_swarm_nodes()
        manager = build_manager(nodes)
        manager.connect_all()

        result = manager.send_rc_all((99, -99, 50, -50))

        self.assertTrue(result.success)
        for node in nodes:
            self.assertEqual(node.adapter.last_rc_command, (10, -10, 10, -10))

    def test_zero_rc_all_sends_zero_to_all_nodes(self) -> None:
        nodes = create_fake_swarm_nodes()
        manager = build_manager(nodes)
        manager.connect_all()
        manager.send_rc_all((5, 5, 5, 5))

        result = manager.zero_rc_all()

        self.assertTrue(result.success)
        for node in nodes:
            self.assertEqual(node.adapter.last_rc_command, (0, 0, 0, 0))

    def test_emergency_stop_blocks_future_nonzero_commands(self) -> None:
        nodes = create_fake_swarm_nodes()
        manager = build_manager(nodes)
        manager.connect_all()

        emergency = manager.emergency_stop_all()
        blocked = manager.send_rc_all((5, 0, 0, 0))

        self.assertTrue(emergency.success)
        self.assertEqual(blocked.action, "send_rc_all_blocked")
        for node in nodes:
            self.assertEqual(node.adapter.last_rc_command, (0, 0, 0, 0))

    def test_low_battery_blocks_takeoff_sequence(self) -> None:
        nodes = create_fake_swarm_nodes()
        manager = build_manager(nodes)
        manager.connect_all()
        nodes[0].adapter.battery_percent = 10

        result = manager.takeoff_sequence()

        self.assertFalse(result.success)
        self.assertFalse(result.results["drone_1"].success)
        self.assertFalse(nodes[0].airborne)

    def test_offline_node_blocks_nonzero_rc(self) -> None:
        nodes = create_fake_swarm_nodes()
        manager = build_manager(nodes)
        manager.connect_all()
        nodes[1].mark_offline("simulated link loss")

        result = manager.send_rc_all((5, 0, 0, 0))

        self.assertEqual(result.action, "send_rc_all_blocked")
        for node in nodes:
            self.assertEqual(node.adapter.last_rc_command, (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
