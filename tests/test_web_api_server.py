import unittest

from web_api.server import DroneWebService


class FrameStub:
    size = 1

    def any(self) -> bool:
        return True


class RealAdapterStub:
    def __init__(self) -> None:
        self.connected = False
        self.streaming = False
        self.last_connection_battery = 76
        self.stopped = False
        self.landed = False

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
        return 75

    def get_height(self) -> int:
        return 12

    def get_front_distance_cm(self):
        return None

    def move_rc(self, *_values: int) -> None:
        pass

    def land(self) -> None:
        self.landed = True

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
        self.assertEqual(result["battery"], 75)
        self.assertEqual(result["heightCm"], 12)

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


if __name__ == "__main__":
    unittest.main()
