import unittest
from http.client import HTTPConnection
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread
from time import sleep

from web_api.server import DroneWebService, create_server


class FrameStub:
    size = 1

    def any(self) -> bool:
        return True


class RealAdapterStub:
    def __init__(self, battery: int = 76, front_distance: float | None = None) -> None:
        self.connected = False
        self.streaming = False
        self.last_connection_battery = battery
        self.battery = battery
        self.stopped = False
        self.landed = False
        self.taken_off = False
        self.takeoff_authorized = False
        self.rc_commands = []
        self.front_distance = front_distance

    def connect(self) -> None:
        self.connected = True

    def stream_on(self) -> None:
        if not self.connected:
            raise RuntimeError("not connected")
        self.streaming = True

    def get_frame(self) -> FrameStub:
        if not self.streaming:
            raise RuntimeError("stream off")
        return FrameStub()

    def get_cached_battery(self) -> int:
        return self.battery

    def get_height(self) -> int:
        return 12

    def get_front_distance_cm(self):
        return self.front_distance

    def move_rc(self, *values: int) -> None:
        self.rc_commands.append(values)

    def authorize_next_takeoff(self) -> None:
        self.takeoff_authorized = True

    def takeoff(self) -> None:
        if not self.takeoff_authorized:
            raise RuntimeError("takeoff was not authorized")
        self.takeoff_authorized = False
        self.taken_off = True

    def land(self) -> None:
        self.landed = True
        self.taken_off = False

    def stop(self) -> None:
        self.stopped = True
        self.streaming = False
        self.connected = False


class DroneWebServiceTests(unittest.TestCase):
    def test_connect_opens_stream_only_after_real_connection(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)

        result = service.connect()

        self.assertTrue(adapter.connected)
        self.assertTrue(adapter.streaming)
        self.assertTrue(result["videoReady"])
        self.assertEqual(result["battery"], 76)
        self.assertEqual(result["heightCm"], 12)
        self.assertTrue(result["canTakeoff"])

    def test_status_rejects_requests_before_connection(self) -> None:
        service = DroneWebService(adapter_factory=RealAdapterStub)

        with self.assertRaisesRegex(RuntimeError, "真机未连接"):
            service.status()

    def test_stop_releases_stream_and_connection(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()

        service.stop()

        self.assertTrue(adapter.stopped)
        self.assertFalse(service.connected)

    def test_emergency_land_clears_output_and_closes_connection(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()

        service.emergency_land()

        self.assertTrue(adapter.landed)
        self.assertTrue(adapter.stopped)
        self.assertFalse(service.connected)

    def test_takeoff_requires_preflight_and_enables_manual_control(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()

        result = service.takeoff()

        self.assertTrue(adapter.taken_off)
        self.assertTrue(result["airborne"])
        self.assertTrue(result["rcEnabled"])
        self.assertEqual(result["phase"], "手动飞行")

    def test_low_battery_blocks_takeoff(self) -> None:
        adapter = RealAdapterStub(battery=12)
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()

        with self.assertRaisesRegex(RuntimeError, "起飞检查未通过"):
            service.takeoff()

        self.assertFalse(adapter.taken_off)

    def test_front_obstacle_blocks_takeoff(self) -> None:
        adapter = RealAdapterStub(front_distance=45)
        service = DroneWebService(adapter_factory=lambda: adapter)

        result = service.connect()

        self.assertFalse(result["preflight"]["frontTof"])
        self.assertFalse(result["canTakeoff"])
        with self.assertRaisesRegex(RuntimeError, "起飞检查未通过"):
            service.takeoff()

    def test_manual_control_rejects_commands_on_ground(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()

        with self.assertRaisesRegex(RuntimeError, "手动控制不可用"):
            service.move_rc({"forwardBack": 20})

    def test_manual_control_is_bounded_and_hover_clears_channels(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()
        service.takeoff()

        service.move_rc({"leftRight": 99, "forwardBack": -20, "upDown": 0, "yaw": 0})
        service.hover()

        self.assertIn((35, -20, 0, 0), adapter.rc_commands)
        self.assertEqual(adapter.rc_commands[-1], (0, 0, 0, 0))


class SidecarServerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.service = DroneWebService(
            adapter_factory=RealAdapterStub,
            data_dir=self.temporary_directory.name,
        )
        self.server = create_server(
            session_token="test-session-token",
            data_dir=self.temporary_directory.name,
            service=self.service,
        )
        self.thread = Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self._close_server)

    def _close_server(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.service.shutdown()
        self.thread.join(timeout=2)

    def _request(
        self,
        method: str,
        path: str,
        *,
        token: str | None = "test-session-token",
    ) -> tuple[int, str]:
        connection = HTTPConnection("127.0.0.1", self.server.server_address[1], timeout=2)
        headers = {"X-Phantom-Token": token} if token is not None else {}
        connection.request(method, path, headers=headers)
        response = connection.getresponse()
        body = response.read().decode("utf-8")
        connection.close()
        return response.status, body

    def test_random_port_and_data_directory(self) -> None:
        self.assertGreater(self.server.server_address[1], 0)
        self.assertEqual(
            self.service.data_dir,
            Path(self.temporary_directory.name).resolve(),
        )

    def test_all_regular_endpoints_require_session_token(self) -> None:
        status, _ = self._request("GET", "/api/health", token=None)
        wrong_status, _ = self._request("POST", "/api/drone/connect", token="wrong")

        self.assertEqual(status, 401)
        self.assertEqual(wrong_status, 401)

    def test_valid_token_can_connect_and_query_health(self) -> None:
        connect_status, _ = self._request("POST", "/api/drone/connect")
        health_status, body = self._request("GET", "/api/health")

        self.assertEqual(connect_status, 200)
        self.assertEqual(health_status, 200)
        self.assertIn('"connected": true', body)

    def test_video_token_is_short_lived_and_single_use(self) -> None:
        payload = self.server.issue_video_token(lifetime_seconds=10)

        self.assertTrue(self.server.consume_video_token(payload["token"]))
        self.assertFalse(self.server.consume_video_token(payload["token"]))

    def test_shutdown_endpoint_releases_service(self) -> None:
        self._request("POST", "/api/drone/connect")
        status, _ = self._request("POST", "/api/sidecar/shutdown")
        self.thread.join(timeout=2)

        self.assertEqual(status, 200)
        self.assertFalse(self.thread.is_alive())
        self.assertFalse(self.service.connected)

    def test_forward_control_stops_when_front_obstacle_appears(self) -> None:
        adapter = RealAdapterStub(front_distance=100)
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()
        service.takeoff()
        adapter.front_distance = 45
        service._refresh_front_tof(force=True)

        with self.assertRaisesRegex(RuntimeError, "存在障碍"):
            service.move_rc({"forwardBack": 20})

        self.assertEqual(adapter.rc_commands[-1], (0, 0, 0, 0))

    def test_stop_lands_before_disconnecting_when_airborne(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()
        service.takeoff()

        service.stop()

        self.assertTrue(adapter.landed)
        self.assertTrue(adapter.stopped)
        self.assertFalse(service.connected)

    def test_stop_releases_connection_even_when_landing_fails(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()
        service.takeoff()

        def fail_land() -> None:
            raise RuntimeError("land failed")

        adapter.land = fail_land  # type: ignore[method-assign]
        with self.assertRaisesRegex(RuntimeError, "land failed"):
            service.stop()

        self.assertTrue(adapter.stopped)
        self.assertFalse(service.connected)

    def test_normal_land_keeps_video_connection(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        service.connect()
        service.takeoff()

        result = service.land()

        self.assertTrue(adapter.landed)
        self.assertTrue(service.connected)
        self.assertTrue(result["videoReady"])
        self.assertFalse(result["airborne"])

    def test_watchdog_hovers_after_manual_command_expires(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        self.addCleanup(service.shutdown)
        service.connect()
        service.takeoff()

        service.move_rc({"leftRight": 20})
        sleep(0.55)

        self.assertEqual(adapter.rc_commands[-1], (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
