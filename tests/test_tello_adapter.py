import unittest
import json
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from app.modes import run_connection_test
from drone.tello_adapter import TelloDroneAdapter


class RecordingTello:
    def __init__(self) -> None:
        self.takeoff_calls = 0
        self.land_calls = 0
        self.read_response = "tof 600"
        self.responses = {
            "command": "ok",
            "battery?": "87",
        }
        self.background_frame_read = None
        self.stream_on = False
        self.state_battery = 86

    def streamon(self) -> None:
        self.stream_on = True

    def streamoff(self) -> None:
        self.stream_on = False

    def takeoff(self) -> None:
        self.takeoff_calls += 1

    def land(self) -> None:
        self.land_calls += 1

    def get_distance_tof(self) -> int:
        return 147

    def get_battery(self) -> int:
        return self.state_battery

    def get_height(self) -> int:
        return -20

    def get_yaw(self) -> int:
        return -173

    def send_command_with_return(self, command: str, timeout: float = 0.0) -> str:
        if not isinstance(timeout, int):
            raise TypeError("timeout must be int")
        self.last_read_command = command
        self.last_read_timeout = timeout
        if command == "EXT tof?":
            return self.read_response
        response = self.responses[command]
        if isinstance(response, Exception):
            raise response
        return response


class RecordingWorker:
    def __init__(self, events) -> None:
        self.events = events

    def join(self, timeout=None) -> None:
        self.events.append(("join", timeout))

    def is_alive(self) -> bool:
        return False


class RecordingContainer:
    def __init__(self, events) -> None:
        self.events = events

    def close(self) -> None:
        self.events.append("container_close")


class RecordingFrameReader:
    def __init__(self, events) -> None:
        self.events = events
        self.container = RecordingContainer(events)
        self.worker = RecordingWorker(events)

    def stop(self) -> None:
        self.events.append("reader_stop")


class ConnectionTestDrone:
    def __init__(self) -> None:
        self.connect_calls = 0
        self.battery_calls = 0
        self.stop_calls = 0

    def connect(self) -> None:
        self.connect_calls += 1

    def get_battery(self) -> int:
        self.battery_calls += 1
        return 91

    def stop(self) -> None:
        self.stop_calls += 1

    def takeoff(self) -> None:
        raise AssertionError("connection-test must not take off")

    def stream_on(self) -> None:
        raise AssertionError("connection-test must not start the camera")


class TelloDroneAdapterTestCase(unittest.TestCase):
    def build_adapter(self) -> tuple[TelloDroneAdapter, RecordingTello]:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()
        adapter._tello = tello
        adapter.connected = True
        return adapter, tello

    def test_connect_requires_ok_and_verifies_battery(self) -> None:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()

        with patch.object(adapter, "_create_tello", return_value=tello), redirect_stdout(
            StringIO()
        ) as output:
            adapter.connect()

        self.assertTrue(adapter.connected)
        self.assertEqual(adapter.last_connection_battery, 87)
        self.assertIn("Connecting to 192.168.10.1:8889", output.getvalue())
        self.assertIn("Received: ok", output.getvalue())
        self.assertIn("Connection verified, battery=87%", output.getvalue())

    def test_connect_reports_udp_timeout(self) -> None:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()
        tello.responses["command"] = (
            "Aborting command 'command'. Did not receive a response after 5 seconds"
        )

        with patch.object(adapter, "_create_tello", return_value=tello):
            with self.assertRaisesRegex(RuntimeError, "UDP timeout"):
                adapter.connect()

        self.assertFalse(adapter.connected)

    def test_connect_rejects_unexpected_response(self) -> None:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()
        tello.responses["command"] = "hello"

        with patch.object(adapter, "_create_tello", return_value=tello):
            with self.assertRaisesRegex(RuntimeError, "unexpected response"):
                adapter.connect()

        self.assertFalse(adapter.connected)

    def test_connect_reports_sdk_rejection(self) -> None:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()
        tello.responses["command"] = "error"

        with patch.object(adapter, "_create_tello", return_value=tello):
            with self.assertRaisesRegex(RuntimeError, "SDK command rejected"):
                adapter.connect()

    def test_connect_keeps_sdk_connection_when_battery_query_fails(self) -> None:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()
        tello.responses["battery?"] = (
            "Aborting command 'battery?'. Did not receive a response after 5 seconds"
        )

        with patch.object(adapter, "_create_tello", return_value=tello), redirect_stdout(
            StringIO()
        ) as output:
            adapter.connect()

        self.assertTrue(adapter.connected)
        self.assertIsNone(adapter.last_connection_battery)
        self.assertIn("WARNING", output.getvalue())

    def test_connect_reports_socket_error(self) -> None:
        adapter = TelloDroneAdapter()
        tello = RecordingTello()
        tello.responses["command"] = OSError("network unreachable")

        with patch.object(adapter, "_create_tello", return_value=tello):
            with self.assertRaisesRegex(RuntimeError, "socket error"):
                adapter.connect()

    def test_get_battery_uses_command_response_instead_of_state_cache(self) -> None:
        adapter, tello = self.build_adapter()

        self.assertEqual(adapter.get_battery(), 87)
        self.assertEqual(tello.last_read_command, "battery?")

    def test_cached_battery_uses_nonblocking_state_stream(self) -> None:
        adapter, tello = self.build_adapter()
        tello.last_read_command = "sentinel"

        self.assertEqual(adapter.get_cached_battery(), 86)
        self.assertEqual(tello.last_read_command, "sentinel")

    def test_connection_mode_only_connects_queries_battery_and_stops(self) -> None:
        drone = ConnectionTestDrone()

        with patch("app.modes.create_drone_adapter", return_value=drone), redirect_stdout(
            StringIO()
        ):
            result = run_connection_test(use_fake=False)

        self.assertEqual(result, 0)
        self.assertEqual(drone.connect_calls, 1)
        self.assertEqual(drone.battery_calls, 1)
        self.assertEqual(drone.stop_calls, 1)

    def test_authorization_skips_one_takeoff_prompt(self) -> None:
        adapter, tello = self.build_adapter()
        adapter.authorize_next_takeoff()

        with patch("builtins.input") as input_mock:
            adapter.takeoff()

        input_mock.assert_not_called()
        self.assertEqual(tello.takeoff_calls, 1)

    def test_land_audit_records_caller_process_thread_and_context(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            adapter = TelloDroneAdapter(
                flight_audit_enabled=True,
                flight_audit_log_dir=temporary_directory,
            )
            tello = RecordingTello()
            adapter._tello = tello
            adapter.connected = True
            adapter.set_land_context(
                session_state="TARGET_LOST_LANDING",
                follow_mode="side",
                height_cm=121,
            )

            with redirect_stdout(StringIO()):
                adapter.land()

            self.assertEqual(tello.land_calls, 1)
            self.assertIsNotNone(adapter.flight_audit_path)
            records = [
                json.loads(line)
                for line in Path(adapter.flight_audit_path).read_text(
                    encoding="utf-8"
                ).splitlines()
            ]
            self.assertEqual(
                [(record["event"], record["outcome"]) for record in records],
                [
                    ("land_command", "requested"),
                    ("land_command", "succeeded"),
                ],
            )
            requested = records[0]
            self.assertGreater(requested["process_id"], 0)
            self.assertEqual(requested["thread_name"], "MainThread")
            self.assertEqual(requested["context"]["session_state"], "TARGET_LOST_LANDING")
            self.assertEqual(requested["context"]["follow_mode"], "side")
            self.assertTrue(
                any("test_land_audit_records" in frame for frame in requested["call_stack"])
            )

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

    def test_front_tof_8190_through_8192_mean_out_of_range(self) -> None:
        adapter, tello = self.build_adapter()
        for sentinel in (8190, 8191, 8192):
            with self.subTest(sentinel=sentinel):
                tello.read_response = f"tof {sentinel}"
                self.assertIsNone(adapter.get_front_distance_cm())

    def test_front_tof_rejects_malformed_response(self) -> None:
        adapter, tello = self.build_adapter()
        tello.read_response = "error"

        with self.assertRaisesRegex(RuntimeError, "返回格式无效"):
            adapter.get_front_distance_cm()

    def test_stream_off_releases_reader_before_stopping_aircraft_stream(self) -> None:
        adapter, tello = self.build_adapter()
        events = []
        tello.background_frame_read = RecordingFrameReader(events)
        tello.stream_on = True
        adapter.streaming = True
        original_streamoff = tello.streamoff

        def record_streamoff() -> None:
            events.append("streamoff")
            original_streamoff()

        tello.streamoff = record_streamoff

        adapter.stream_off()

        self.assertEqual(
            events,
            ["reader_stop", "container_close", ("join", 2.0), "streamoff"],
        )
        self.assertIsNone(tello.background_frame_read)
        self.assertFalse(adapter.streaming)

    def test_stream_on_discards_stale_reader_before_restart(self) -> None:
        adapter, tello = self.build_adapter()
        events = []
        tello.background_frame_read = RecordingFrameReader(events)
        original_streamon = tello.streamon

        def record_streamon() -> None:
            events.append("streamon")
            original_streamon()

        tello.streamon = record_streamon

        adapter.stream_on()

        self.assertEqual(
            events,
            ["reader_stop", "container_close", ("join", 2.0), "streamon"],
        )
        self.assertIsNone(tello.background_frame_read)
        self.assertTrue(adapter.streaming)


if __name__ == "__main__":
    unittest.main()
