"""Integration tests for post-climb manual takeover in FollowSession."""

import time
import unittest
from threading import Event, Thread
from unittest.mock import Mock, patch

import numpy as np

from control.fixed_demo import FixedDemoManeuver, FixedDemoStep
from control.follow_control import FollowController, RCCommand
from control.follow_session import FollowSession
from control.kernel.arbitration import FollowTickOutcome
from control.kernel.phase_handlers import LifecycleContext
from control.kernel.phase_handlers.pre_follow import PreFollowHandler
from control.kernel.phases import KernelPhase
from drone.fake_adapter import FakeDroneAdapter
from drone.front_tof import FrontToFSnapshot
from drone.safety import SafetyConfig, SafetyManager


FRESH_TARGET = {
    "found": True,
    "is_predicted": False,
    "ambiguous": False,
    "center": (420, 240),
    "area": 12_000.0,
    "area_ratio": 0.04,
    "bbox": (380, 180, 80, 120),
}


class RecordingDrone(FakeDroneAdapter):
    def __init__(self, *, battery_delay: float = 0.0) -> None:
        super().__init__(verbose_rc=False)
        self.height_cm = 150
        self.battery_delay = battery_delay
        self.command_log = []

    def move_rc(self, left_right, forward_backward, up_down, yaw) -> None:
        super().move_rc(left_right, forward_backward, up_down, yaw)
        self.command_log.append(
            (time.monotonic(), (left_right, forward_backward, up_down, yaw))
        )

    def get_battery(self) -> int:
        if self.battery_delay:
            time.sleep(self.battery_delay)
        return super().get_battery()

    def get_cached_battery(self) -> int:
        if self.battery_delay:
            time.sleep(self.battery_delay)
        return self.battery_percent


class BlockingEmissionDrone(RecordingDrone):
    def __init__(self) -> None:
        super().__init__()
        self.nonzero_started = Event()
        self.release_nonzero = Event()
        self._blocked_once = False

    def move_rc(self, left_right, forward_backward, up_down, yaw) -> None:
        command = (left_right, forward_backward, up_down, yaw)
        if command != (0, 0, 0, 0) and not self._blocked_once:
            self._blocked_once = True
            self.nonzero_started.set()
            self.release_nonzero.wait(timeout=1.0)
        super().move_rc(*command)


class CountingDetector:
    def __init__(self, script=None) -> None:
        self.script = list(script or [])
        self.detect_calls = 0
        self.reset_calls = 0

    def detect(self, frame):
        self.detect_calls += 1
        item = self.script.pop(0) if self.script else dict(FRESH_TARGET)
        if isinstance(item, Exception):
            raise item
        return dict(item)

    def reset(self) -> None:
        self.reset_calls += 1

    def draw_debug(self, frame, result):
        return frame


class ScriptedCamera:
    def __init__(self, session, sequence) -> None:
        self.session = session
        self.sequence = list(sequence)
        self.reads = 0

    def read_frame(self):
        self.reads += 1
        if not self.sequence:
            self.session.stop_event.set()
            return None
        return self.sequence.pop(0)


class SnapshotMonitor:
    def __init__(self, status="valid", distance=100.0) -> None:
        self.status = status
        self.distance = distance
        self.prepare_calls = 0
        self.start_calls = 0
        self.stop_calls = 0

    def prepare(self) -> None:
        self.prepare_calls += 1

    def start(self) -> None:
        self.start_calls += 1

    def stop(self) -> None:
        self.stop_calls += 1

    def snapshot(self):
        return FrontToFSnapshot(
            distance_cm=self.distance,
            status=self.status,
            timestamp=1.0,
            age_seconds=0.0,
            sequence=1,
            consecutive_blocked=1 if self.distance and self.distance <= 60 else 0,
        )


class CountingMotionArbiter:
    def __init__(self) -> None:
        self.provider = None
        self.reset_calls = 0
        self.decide_calls = 0
        self.close_calls = 0
        self.is_active = False

    def set_front_tof_provider(self, provider) -> None:
        self.provider = provider

    def reset(self, mode="unknown") -> None:
        self.reset_calls += 1
        self.is_active = True

    def decide(self, *args, **kwargs):
        self.decide_calls += 1
        raise AssertionError("manual takeover must not invoke MotionArbiter")

    def close(self) -> None:
        self.close_calls += 1
        self.is_active = False

    def invalidate_observation(self) -> None:
        pass


class SlowResetMotionArbiter(CountingMotionArbiter):
    def __init__(self) -> None:
        super().__init__()
        self.reset_started = Event()
        self.release_reset = Event()

    def reset(self, mode="unknown") -> None:
        self.reset_started.set()
        self.release_reset.wait(timeout=1.0)
        super().reset(mode)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeTimer:
    instances = []

    def __init__(self, interval, function, args=()) -> None:
        self.interval = interval
        self.function = function
        self.args = args
        self.cancelled = False
        self.daemon = False
        self.instances.append(self)

    def start(self) -> None:
        pass

    def cancel(self) -> None:
        self.cancelled = True

    def fire(self) -> None:
        self.function(*self.args)


def frame():
    return np.zeros((480, 640, 3), dtype=np.uint8)


def build_session(
    *,
    detector=None,
    drone=None,
    display=False,
    motion_arbiter=None,
    manual_overrides=None,
):
    drone = drone or RecordingDrone()
    detector = detector or CountingDetector()
    safety = SafetyManager(SafetyConfig(20, 5, 220, 60, 35, 3, 8))
    manual = {
        "enabled": True,
        "forward_speed": 20,
        "lateral_speed": 20,
        "vertical_speed": 15,
        "yaw_speed": 20,
        "command_timeout_seconds": 0.25,
        "reacquire_frames": 5,
        "front_tof_guard_enabled": True,
        "front_stop_distance_cm": 60,
        "block_forward_when_tof_invalid": True,
        "minimum_descent_height_cm": 40,
        "maximum_ascent_height_cm": 200,
        **(manual_overrides or {}),
    }
    session = FollowSession(
        drone=drone,
        safety_manager=safety,
        detector=detector,
        follow_controller=FollowController(safety_manager=safety),
        config={
            "display_console_camera": display,
            "control_interval": 0.02,
            "frame_failure_limit": 10,
            "height_failure_limit": 3,
            "height_filter_window": 1,
            "max_height_cm": 220,
            "manual_control": manual,
            "obstacle": {"enabled": False, "front_tof_enabled": False},
        },
        mode_label="MANUAL TEST",
        motion_arbiter=motion_arbiter,
    )
    return session, drone, detector


def enter_manual(session, now=10.0) -> None:
    session.manual_controller.make_available()
    with patch("control.follow_session.monotonic", return_value=now):
        if not session._enter_manual_mode():
            raise AssertionError("manual mode did not become active")


class FollowSessionManualIntegrationTestCase(unittest.TestCase):
    def test_control_ready_hovers_without_reid_until_m_is_pressed(self) -> None:
        session, drone, detector = build_session(display=True)
        session.camera = ScriptedCamera(session, [frame(), frame(), frame()])
        session.manual_controller.make_available()

        with patch("cv2.imshow"), patch("cv2.putText"), patch(
            "cv2.waitKey", side_effect=[255, 255, ord("m")]
        ), patch.object(
            session, "_draw_state_label", side_effect=lambda image, *_: image
        ), patch(
            "control.follow_session.monotonic", return_value=10.0
        ), patch(
            "control.follow_session.sleep", return_value=None
        ):
            selection = session._wait_for_control_selection()

        self.assertEqual(selection, "manual")
        self.assertTrue(session.manual_controller.active)
        self.assertEqual(detector.detect_calls, 0)
        self.assertEqual(session.camera.reads, 3)
        self.assertTrue(drone.command_log)
        self.assertTrue(all(command == (0, 0, 0, 0) for _, command in drone.command_log))
        session._cancel_manual_watchdog(force_hover=True)

    def test_control_ready_low_battery_lands_before_accepting_a_mode(self) -> None:
        session, drone, _detector = build_session(display=True)
        drone.battery_percent = 5

        selection = session._wait_for_control_selection()

        self.assertIsNone(selection)
        self.assertEqual(session.session_state, "LOW_BATTERY_LANDING")

    def test_control_ready_a_selects_auto_without_running_reid(self) -> None:
        session, _drone, detector = build_session(display=True)
        session.manual_controller.make_available()
        session.camera = ScriptedCamera(session, [frame()])

        with patch("cv2.imshow"), patch("cv2.putText"), patch(
            "cv2.waitKey", return_value=ord("a")
        ), patch.object(
            session, "_draw_state_label", side_effect=lambda image, *_: image
        ), patch(
            "control.follow_session.sleep", return_value=None
        ):
            selection = session._wait_for_control_selection()

        self.assertEqual(selection, "auto")
        self.assertFalse(session.manual_controller.active)
        self.assertEqual(detector.detect_calls, 0)

    def test_control_ready_sensor_failures_converge_to_landing_states(self) -> None:
        high_session, high_drone, _ = build_session(display=True)
        high_session.manual_controller.make_available()
        high_drone.height_cm = 221
        self.assertIsNone(high_session._wait_for_control_selection())
        self.assertEqual(high_session.session_state, "HEIGHT_LIMIT_LANDING")

        height_session, height_drone, _ = build_session(display=True)
        height_session.manual_controller.make_available()
        height_session.height_failure_limit = 2
        height_drone.height_cm = 0
        height_session.camera = ScriptedCamera(height_session, [frame()])
        with patch("cv2.imshow"), patch("cv2.putText"), patch(
            "cv2.waitKey", return_value=255
        ), patch.object(
            height_session, "_draw_state_label", side_effect=lambda image, *_: image
        ), patch(
            "control.follow_session.sleep", return_value=None
        ):
            self.assertIsNone(height_session._wait_for_control_selection())
        self.assertEqual(height_session.session_state, "HEIGHT_SENSOR_LANDING")

        frame_session, _frame_drone, _ = build_session(display=True)
        frame_session.manual_controller.make_available()
        frame_session.frame_failure_limit = 2
        frame_session.camera = ScriptedCamera(frame_session, [None, None])
        with patch("control.follow_session.sleep", return_value=None):
            self.assertIsNone(frame_session._wait_for_control_selection())
        self.assertEqual(frame_session.session_state, "FRAME_LOST_LANDING")

    def test_control_ready_q_and_e_keep_global_landing_priority(self) -> None:
        for key, expected in (
            (ord("q"), "STOPPED"),
            (ord("e"), "EMERGENCY_STOP"),
        ):
            with self.subTest(key=chr(key)):
                session, drone, _detector = build_session(display=True)
                session.manual_controller.make_available()
                session.camera = ScriptedCamera(session, [frame()])
                with patch("cv2.imshow"), patch("cv2.putText"), patch(
                    "cv2.waitKey", return_value=key
                ), patch.object(
                    session, "_draw_state_label", side_effect=lambda image, *_: image
                ), patch(
                    "control.follow_session.sleep", return_value=None
                ):
                    selection = session._wait_for_control_selection()

                self.assertIsNone(selection)
                self.assertEqual(session.session_state, expected)
                self.assertEqual(drone.command_log[-1][1], (0, 0, 0, 0))

    def test_manual_loop_skips_reid_and_emits_only_operator_lateral_motion(self) -> None:
        session, _drone, detector = build_session()
        session.camera = ScriptedCamera(session, [frame(), frame()])
        emitted = []
        session.send_command = emitted.append
        enter_manual(session)
        with patch("control.follow_session.monotonic", return_value=10.0):
            session.handle_key(ord("d"))
            with patch("control.follow_session.sleep", return_value=None):
                session._loop()

        self.assertEqual(detector.detect_calls, 0)
        self.assertEqual([command.as_tuple() for command in emitted], [(20, 0, 0, 0)] * 2)
        session._cancel_manual_watchdog(force_hover=True)

    def test_repeated_m_events_require_a_quiet_gap_before_second_toggle(self) -> None:
        session, _drone, _detector = build_session()
        enter_manual(session, now=10.0)

        with patch("control.follow_session.monotonic", return_value=10.5):
            session.handle_key(ord("m"))
        self.assertTrue(session.manual_controller.active)
        with patch("control.follow_session.monotonic", return_value=11.0):
            session.handle_key(ord("m"))
        self.assertTrue(session.manual_controller.active)
        with patch("control.follow_session.monotonic", return_value=12.0):
            session.handle_key(ord("m"))
        self.assertFalse(session.manual_controller.active)

    def test_refreshed_direction_cancels_old_watchdog_generation(self) -> None:
        session, drone, _detector = build_session()
        enter_manual(session)
        FakeTimer.instances = []

        with patch("control.follow_session.Timer", FakeTimer), patch(
            "control.follow_session.monotonic", return_value=10.0
        ):
            session.handle_key(ord("d"))
            first = FakeTimer.instances[-1]
            session.handle_key(ord("d"))
            second = FakeTimer.instances[-1]
            session.send_command(RCCommand(left_right=20))

            self.assertTrue(first.cancelled)
            first.fire()
            self.assertEqual(drone.command_log[-1][1], (20, 0, 0, 0))
            second.fire()
            self.assertEqual(drone.command_log[-1][1], (0, 0, 0, 0))

    def test_blocked_front_tof_cannot_start_an_autonomous_sidestep(self) -> None:
        arbiter = CountingMotionArbiter()
        session, drone, _detector = build_session(motion_arbiter=arbiter)
        session.front_tof_monitor = SnapshotMonitor(distance=40)
        enter_manual(session)
        arbiter.decide_calls = 0

        session.send_motion_command(RCCommand())

        self.assertEqual(arbiter.decide_calls, 0)
        self.assertEqual(drone.command_log[-1][1], (0, 0, 0, 0))

    def test_deadman_zeroes_output_while_battery_read_is_blocked(self) -> None:
        drone = RecordingDrone(battery_delay=0.20)
        session, _drone, _detector = build_session(
            drone=drone,
            manual_overrides={"command_timeout_seconds": 0.05},
        )
        session.camera = ScriptedCamera(session, [frame()])
        session.manual_controller.make_available()
        self.assertTrue(session._enter_manual_mode())
        session.handle_key(ord("d"))
        session.send_command(RCCommand(left_right=20))

        with patch("control.follow_session.sleep", return_value=None):
            session._loop()

        nonzero_index = next(
            index for index, (_, command) in enumerate(drone.command_log) if command != (0, 0, 0, 0)
        )
        zero_index = next(
            index
            for index in range(nonzero_index + 1, len(drone.command_log))
            if drone.command_log[index][1] == (0, 0, 0, 0)
        )
        zero_time = drone.command_log[zero_index][0]
        nonzero_time = drone.command_log[nonzero_index][0]
        self.assertLess(zero_time - nonzero_time, 0.15)
        self.assertTrue(
            all(
                command == (0, 0, 0, 0)
                for _, command in drone.command_log[zero_index:]
            )
        )
        session._cancel_manual_watchdog(force_hover=True)

    def test_external_emergency_zero_cannot_be_overwritten_by_stale_tick(self) -> None:
        drone = BlockingEmissionDrone()
        session, _drone, _detector = build_session(drone=drone)
        stale_tick = Thread(
            target=session._kernel._emit,
            args=(RCCommand(left_right=20),),
        )
        stale_tick.start()
        self.assertTrue(drone.nonzero_started.wait(timeout=0.5))

        emergency = Thread(target=session.request_emergency_stop)
        emergency.start()
        time.sleep(0.02)
        drone.release_nonzero.set()
        stale_tick.join(timeout=1.0)
        emergency.join(timeout=1.0)

        self.assertFalse(stale_tick.is_alive())
        self.assertFalse(emergency.is_alive())
        self.assertEqual(drone.command_log[-1][1], (0, 0, 0, 0))

    def test_manual_takeover_zeroes_before_slow_arbiter_reset(self) -> None:
        arbiter = SlowResetMotionArbiter()
        session, drone, _detector = build_session(motion_arbiter=arbiter)
        session.manual_controller.make_available()
        session._kernel._emit(RCCommand(forward_backward=20))

        takeover = Thread(target=session._enter_manual_mode)
        takeover.start()
        self.assertTrue(arbiter.reset_started.wait(timeout=0.5))
        self.assertEqual(drone.command_log[-1][1], (0, 0, 0, 0))

        arbiter.release_reset.set()
        takeover.join(timeout=1.0)
        self.assertFalse(takeover.is_alive())
        self.assertTrue(session.manual_controller.active)

    def test_cleanup_zeroes_before_waiting_for_front_tof_stop(self) -> None:
        session, drone, _detector = build_session()

        class CheckingMonitor(SnapshotMonitor):
            command_seen_at_stop = None

            def stop(inner_self) -> None:
                inner_self.command_seen_at_stop = drone.last_rc_command
                super().stop()

        monitor = CheckingMonitor()
        session.front_tof_monitor = monitor
        session._kernel._emit(RCCommand(yaw=20))

        session._kernel._land_and_cleanup()

        self.assertEqual(monitor.command_seen_at_stop, (0, 0, 0, 0))

    def test_manual_release_hovers_for_five_fresh_frames_then_allows_auto(self) -> None:
        session, _drone, detector = build_session()
        enter_manual(session)
        session._leave_manual_mode()
        session.camera = ScriptedCamera(session, [frame()] * 6)
        emitted = []
        session.send_command = emitted.append
        session._arbitration.arbitrate = Mock(
            return_value=FollowTickOutcome(
                command=RCCommand(left_right=17), state="FOLLOWING"
            )
        )

        with patch("control.follow_session.sleep", return_value=None):
            session._loop()

        self.assertEqual(detector.detect_calls, 6)
        self.assertEqual(session._arbitration.arbitrate.call_count, 1)
        self.assertEqual([command.as_tuple() for command in emitted[:5]], [(0, 0, 0, 0)] * 5)
        self.assertEqual(emitted[5].as_tuple(), (17, 0, 0, 0))

    def test_reacquire_count_resets_across_inference_failure(self) -> None:
        script = [
            dict(FRESH_TARGET),
            dict(FRESH_TARGET),
            RuntimeError("inference gap"),
            *[dict(FRESH_TARGET) for _ in range(6)],
        ]
        detector = CountingDetector(script)
        session, _drone, _detector = build_session(detector=detector)
        enter_manual(session)
        session._leave_manual_mode()
        session.camera = ScriptedCamera(session, [frame()] * len(script))
        emitted = []
        session.send_command = emitted.append
        session._arbitration.arbitrate = Mock(
            return_value=FollowTickOutcome(
                command=RCCommand(yaw=13), state="FOLLOWING"
            )
        )

        with patch("control.follow_session.sleep", return_value=None):
            session._loop()

        self.assertEqual(session._arbitration.arbitrate.call_count, 1)
        self.assertEqual([command.as_tuple() for command in emitted[:-1]], [(0, 0, 0, 0)] * 7)
        self.assertEqual(emitted[-1].as_tuple(), (0, 0, 0, 13))

    def test_reacquire_count_resets_across_missing_video_frame(self) -> None:
        session, _drone, detector = build_session()
        enter_manual(session)
        session._leave_manual_mode()
        session.camera = ScriptedCamera(
            session,
            [frame(), frame(), None, *[frame() for _ in range(6)]],
        )
        emitted = []
        session.send_command = emitted.append
        session._arbitration.arbitrate = Mock(
            return_value=FollowTickOutcome(
                command=RCCommand(up_down=11), state="FOLLOWING"
            )
        )

        with patch("control.follow_session.sleep", return_value=None):
            session._loop()

        self.assertEqual(detector.detect_calls, 8)
        self.assertEqual(session._arbitration.arbitrate.call_count, 1)
        self.assertEqual([command.as_tuple() for command in emitted[:-1]], [(0, 0, 0, 0)] * 7)
        self.assertEqual(emitted[-1].as_tuple(), (0, 0, 11, 0))

    def test_manual_front_guard_creates_and_reuses_one_monitor(self) -> None:
        monitor = SnapshotMonitor(status="out_of_range", distance=None)
        with patch(
            "control.follow_session.FrontToFMonitor.from_config",
            return_value=monitor,
        ) as factory:
            session, _drone, _detector = build_session()

        factory.assert_called_once()
        self.assertIs(session.front_tof_monitor, monitor)
        session._prepare_front_tof()
        session._start_front_tof()
        session._stop_front_tof()
        self.assertEqual(
            (monitor.prepare_calls, monitor.start_calls, monitor.stop_calls),
            (1, 1, 1),
        )

    def test_fixed_demo_real_m_key_stops_route_without_landing_or_restart(self) -> None:
        session, drone, _detector = build_session()
        clock = FakeClock()
        session.pre_follow_maneuver = FixedDemoManeuver(
            steps=(
                FixedDemoStep(
                    "forward", RCCommand(forward_backward=20), 1.0, 0.0
                ),
            ),
            control_interval=0.05,
            clock=clock,
            sleep_fn=clock.sleep,
        )
        session.airborne = True
        session.manual_controller.make_available()
        pressed = False

        def press_m_once(_progress):
            nonlocal pressed
            if not pressed:
                pressed = True
                session.handle_key(ord("m"))
            return True

        session._show_pre_follow_progress = press_m_once
        phase = PreFollowHandler().run(session, LifecycleContext())

        self.assertEqual(phase, KernelPhase.FOLLOW)
        self.assertTrue(session.manual_controller.active)
        self.assertTrue(session.airborne)
        self.assertEqual(session.session_state, "MANUAL")
        first_motion = next(
            index for index, (_, command) in enumerate(drone.command_log) if command != (0, 0, 0, 0)
        )
        self.assertTrue(
            all(command == (0, 0, 0, 0) for _, command in drone.command_log[first_motion + 1 :])
        )


if __name__ == "__main__":
    unittest.main()
