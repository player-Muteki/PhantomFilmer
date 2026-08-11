import unittest
from unittest.mock import patch

from drone.tello_adapter import TelloDroneAdapter


class RecordingTello:
    def __init__(self) -> None:
        self.takeoff_calls = 0

    def takeoff(self) -> None:
        self.takeoff_calls += 1


class TelloDroneAdapterTestCase(unittest.TestCase):
    def build_adapter(self) -> tuple[TelloDroneAdapter, RecordingTello]:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()
        adapter._tello = tello
        adapter.connected = True
        return adapter, tello

    def test_authorization_skips_one_takeoff_prompt(self) -> None:
        adapter, tello = self.build_adapter()
        adapter.authorize_next_takeoff()

        with patch("builtins.input") as input_mock:
            adapter.takeoff()

        input_mock.assert_not_called()
        self.assertEqual(tello.takeoff_calls, 1)

    def test_authorization_is_consumed_after_one_takeoff(self) -> None:
        adapter, tello = self.build_adapter()
        adapter.authorize_next_takeoff()
        adapter.takeoff()

        with patch("builtins.input", return_value="NO"):
            with self.assertRaisesRegex(RuntimeError, "已取消起飞"):
                adapter.takeoff()

        self.assertEqual(tello.takeoff_calls, 1)

    def test_revoked_authorization_requires_prompt(self) -> None:
        adapter, tello = self.build_adapter()
        adapter.authorize_next_takeoff()
        adapter.revoke_takeoff_authorization()

        with patch("builtins.input", return_value="NO"):
            with self.assertRaisesRegex(RuntimeError, "已取消起飞"):
                adapter.takeoff()

        self.assertEqual(tello.takeoff_calls, 0)


if __name__ == "__main__":
    unittest.main()
