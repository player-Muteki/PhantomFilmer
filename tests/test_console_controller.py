"""Tests for ConsoleController natural-language command dispatch."""

import sys
import unittest
from io import StringIO
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from console.console_controller import ConsoleController
from console.commands import ConsoleCommand


class StubParser:
    """Return predefined actions for each input."""

    def __init__(self, actions):
        self.actions = list(actions)

    def parse(self, user_text: str) -> ConsoleCommand:
        return self.actions.pop(0)


class StubTools:
    """Record which tool methods the controller calls."""

    def __init__(self) -> None:
        self.calls = []

    def close(self) -> None:
        self.calls.append("close")

    def get_status(self):
        self.calls.append("get_status")
        return {"battery": 88, "height": 120, "mode": "待机"}

    def can_start_task(self):
        self.calls.append("can_start_task")
        return True, "允许开始任务。"

    def start_follow_task(self) -> bool:
        self.calls.append("start_follow_task")
        return True

    def stop_task(self) -> None:
        self.calls.append("stop_task")

    def emergency_stop(self) -> None:
        self.calls.append("emergency_stop")


class ConsoleControllerTestCase(unittest.TestCase):
    """Verify controller dispatch remains inside safety-wrapped tools."""

    def test_unknown_command_does_not_call_tools(self) -> None:
        tools = StubTools()
        controller = ConsoleController(tools=tools, parser=StubParser([ConsoleCommand.UNKNOWN, ConsoleCommand.EXIT]))

        with patch("builtins.input", side_effect=["模糊指令", "退出"]), patch("sys.stdout", new=StringIO()) as stdout:
            result = controller.run()

        self.assertEqual(result, 0)
        self.assertEqual(tools.calls, ["close"])
        self.assertIn("无法安全识别", stdout.getvalue())

    def test_start_follow_still_requires_local_confirmation(self) -> None:
        tools = StubTools()
        controller = ConsoleController(tools=tools, parser=StubParser([ConsoleCommand.START_FOLLOW, ConsoleCommand.EXIT]))

        with patch("builtins.input", side_effect=["帮我开始跟随目标", "yes", "退出"]), patch("sys.stdout", new=StringIO()):
            result = controller.run()

        self.assertEqual(result, 0)
        self.assertEqual(tools.calls, ["can_start_task", "start_follow_task", "close"])

    def test_status_command_dispatches_to_tools(self) -> None:
        tools = StubTools()
        controller = ConsoleController(tools=tools, parser=StubParser([ConsoleCommand.GET_STATUS, ConsoleCommand.EXIT]))

        with patch("builtins.input", side_effect=["看看现在无人机状态", "退出"]), patch("sys.stdout", new=StringIO()) as stdout:
            result = controller.run()

        self.assertEqual(result, 0)
        self.assertEqual(tools.calls, ["get_status", "close"])
        self.assertIn("电量：88%", stdout.getvalue())

    def test_emergency_stop_dispatches_locally(self) -> None:
        tools = StubTools()
        controller = ConsoleController(tools=tools, parser=StubParser([ConsoleCommand.EMERGENCY_STOP, ConsoleCommand.EXIT]))

        with patch("builtins.input", side_effect=["立即急停", "退出"]), patch("sys.stdout", new=StringIO()):
            result = controller.run()

        self.assertEqual(result, 0)
        self.assertEqual(tools.calls, ["emergency_stop", "close"])


if __name__ == "__main__":
    unittest.main()
