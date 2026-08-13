import unittest
from unittest.mock import patch

from drone.tello_adapter import TelloDroneAdapter


class RecordingTello:
    def __init__(self) -> None:
        self.takeoff_calls = 0
        self.read_response = "tof 600"

    def takeoff(self) -> None:
        self.takeoff_calls += 1

    def get_distance_tof(self) -> int:
        return 147

    def get_height(self) -> int:
        return -20

    def get_yaw(self) -> int:
        return -173

    def send_command_with_return(self, command: str, timeout: float = 0.0) -> str:
        self.last_read_command = command
        self.last_read_timeout = timeout
        return self.read_response


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

    def test_control_height_uses_downward_tof_not_raw_h(self) -> None:
        adapter, _tello = self.build_adapter()

        self.assertEqual(adapter.get_height(), 147)
        self.assertEqual(adapter.get_estimated_height(), -20)

    def test_yaw_uses_flight_controller_telemetry(self) -> None:
        adapter, _tello = self.build_adapter()

        self.assertEqual(adapter.get_yaw(), -173)

    def test_front_tof_uses_expansion_command_and_converts_mm_to_cm(self) -> None:
        adapter, tello = self.build_adapter()

        self.assertEqual(adapter.get_front_distance_cm(), 60.0)
        self.assertEqual(tello.last_read_command, "EXT tof?")

    def test_front_tof_8192_means_out_of_range(self) -> None:
        adapter, tello = self.build_adapter()
        tello.read_response = "tof 8192"

        self.assertIsNone(adapter.get_front_distance_cm())

    def test_front_tof_rejects_malformed_response(self) -> None:
        adapter, tello = self.build_adapter()
        tello.read_response = "error"

        with self.assertRaisesRegex(RuntimeError, "返回格式无效"):
            adapter.get_front_distance_cm()


if __name__ == "__main__":
    unittest.main()
