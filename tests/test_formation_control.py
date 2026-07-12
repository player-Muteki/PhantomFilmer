import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from control.follow_control import RCCommand
from swarm.formation_control import FormationController, FormationCorrection


class FormationControllerTestCase(unittest.TestCase):
    def test_distributes_base_command_to_all_nodes(self) -> None:
        controller = FormationController()
        commands = controller.distribute(["drone_1", "drone_2"], RCCommand(1, 2, 3, 4))

        self.assertEqual(commands["drone_1"], (1, 2, 3, 4))
        self.assertEqual(commands["drone_2"], (1, 2, 3, 4))

    def test_applies_reserved_per_node_correction(self) -> None:
        controller = FormationController(
            {"drone_2": FormationCorrection(left_right=1, forward_backward=-2)}
        )
        commands = controller.distribute(["drone_1", "drone_2"], (5, 5, 0, 0))

        self.assertEqual(commands["drone_1"], (5, 5, 0, 0))
        self.assertEqual(commands["drone_2"], (6, 3, 0, 0))


if __name__ == "__main__":
    unittest.main()
