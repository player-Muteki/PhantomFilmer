import unittest
from unittest.mock import patch

from drone.front_tof import FrontToFMonitor
from vision.obstacle_detect import DistanceOnlyObstacleDetector, ObstacleResult


class FakeFrontDrone:
    def __init__(self, readings):
        self.readings = iter(readings)

    def get_front_distance_cm(self):
        value = next(self.readings)
        if isinstance(value, Exception):
            raise value
        return value


class FrontToFMonitorTestCase(unittest.TestCase):
    def test_distance_only_detector_never_inspects_camera_pixels(self) -> None:
        class Frame:
            shape = (480, 640, 3)

            def __getattribute__(self, name):
                if name == "shape":
                    return (480, 640, 3)
                raise AssertionError(f"camera pixel access is forbidden: {name}")

        result = DistanceOnlyObstacleDetector().detect(Frame(), {"found": True})

        self.assertFalse(result.found)
        self.assertEqual(result.state, "CLEAR")
        self.assertEqual(result.frame_size, (640, 480))

    def test_prepare_accepts_out_of_range_as_healthy_sensor(self) -> None:
        monitor = FrontToFMonitor(FakeFrontDrone([None]))

        monitor.prepare()

        sample = monitor.snapshot()
        self.assertEqual(sample.status, "out_of_range")
        self.assertIsNone(sample.distance_cm)

    def test_blocked_obstacle_draws_tof_status_label(self) -> None:
        result = ObstacleResult(
            found=True,
            state="BLOCKED",
            front_distance_cm=50.0,
        )
        frame = object()
        with patch(
            "vision.obstacle_detect.draw_status_label",
            return_value="rendered",
        ) as draw_status:
            rendered = DistanceOnlyObstacleDetector().draw_debug(frame, result)

        self.assertEqual(rendered, "rendered")
        draw_status.assert_called_once_with(
            frame,
            "障碍物（ToF） 50cm",
            (0, 0, 255),
            top=84,
        )

    def test_blocked_counter_counts_sensor_samples(self) -> None:
        monitor = FrontToFMonitor(FakeFrontDrone([60.0, 59.0]), blocked_distance_cm=60)

        monitor._poll_once()
        monitor._poll_once()

        self.assertEqual(monitor.snapshot().consecutive_blocked, 2)

    def test_prepare_fails_closed_when_module_does_not_reply(self) -> None:
        monitor = FrontToFMonitor(FakeFrontDrone([RuntimeError("timeout")]))

        with self.assertRaisesRegex(RuntimeError, "禁止起飞"):
            monitor.prepare()


if __name__ == "__main__":
    unittest.main()
