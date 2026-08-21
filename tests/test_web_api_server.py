import unittest
from time import sleep

from web_api.server import DroneWebService


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
