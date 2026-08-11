"""Tests for the independent fixed-demo route and follow handoff."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import main
from control.fixed_demo import FIXED_DEMO_STEPS, FixedDemoManeuver, FixedDemoStep
from control.follow_control import FollowController, RCCommand
from control.follow_session import FollowSession
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager


class FakeClock:
    """Deterministic clock that makes multi-second route tests instantaneous."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class ResetDetector:
    """Minimal detector used to verify the follow state is reset at handoff."""

    def __init__(self) -> None:
        self.reset_count = 0

    def reset(self) -> None:
        self.reset_count += 1


class RecordingManeuver:
    """Maneuver stub that records its position relative to the follow loop."""

    def __init__(self, events: list[str]) -> None:
        self.events = events

    def run(self, send_command, should_abort, on_progress=None, is_avoiding=None) -> bool:
        self.events.append("maneuver")
        self.assert_not_aborted = not should_abort()
        send_command(RCCommand(forward_backward=24))
        send_command(RCCommand())
        return True


class AbortingManeuver:
    """Maneuver stub used to verify an abort lands without entering follow."""

    def run(self, send_command, should_abort, on_progress=None, is_avoiding=None) -> bool:
        send_command(RCCommand(left_right=-16))
        return False


class RecordingSession(FollowSession):
    """No-camera session used to test takeoff/maneuver/follow ordering."""

    def __init__(self, events: list[str], **kwargs) -> None:
        self.events = events
        super().__init__(**kwargs)

    def _start_camera(self) -> None:
        self.streaming = True

    def _stop_camera(self) -> None:
        self.streaming = False

    def _destroy_window(self) -> None:
        pass

    def _loop(self) -> None:
        self.events.append("follow")


def build_safety() -> SafetyManager:
    return SafetyManager(
        SafetyConfig(
            min_battery_takeoff=30,
            low_battery_land=20,
            max_height_cm=150,
            min_height_cm=60,
            max_rc_speed=25,
            target_lost_hover_seconds=3,
            target_lost_land_seconds=8,
        )
    )


class FixedDemoManeuverTestCase(unittest.TestCase):
    def test_route_matches_the_agreed_speed_and_timing(self) -> None:
        self.assertEqual(
            [step.command.as_tuple() for step in FIXED_DEMO_STEPS],
            [(-16, 0, 0, 0), (0, 24, 0, 0), (16, 0, 0, 0)],
        )
        self.assertEqual(
            [step.duration_seconds for step in FIXED_DEMO_STEPS],
            [3.0, 2.0, 3.0],
        )
        self.assertEqual(
            [step.settle_seconds for step in FIXED_DEMO_STEPS],
            [0.5, 0.5, 1.0],
        )

    def test_route_refreshes_commands_and_zeros_between_segments(self) -> None:
        clock = FakeClock()
        commands = []
        maneuver = FixedDemoManeuver(
            control_interval=0.05,
            clock=clock,
            sleep_fn=clock.sleep,
        )

        completed = maneuver.run(commands.append, lambda: False)

        self.assertTrue(completed)
        self.assertGreater(commands.count(RCCommand(forward_backward=24)), 2)
        self.assertIn(RCCommand(left_right=-16), commands)
        self.assertIn(RCCommand(left_right=16), commands)
        self.assertEqual(commands[-1], RCCommand())
        first_forward = commands.index(RCCommand(forward_backward=24))
        first_right = commands.index(RCCommand(left_right=16))
        self.assertEqual(commands[0], RCCommand(left_right=-16))
        self.assertEqual(commands[first_forward - 1], RCCommand())
        self.assertEqual(commands[first_right - 1], RCCommand())

    def test_progress_callback_can_abort_and_output_is_zeroed(self) -> None:
        clock = FakeClock()
        commands = []
        maneuver = FixedDemoManeuver(clock=clock, sleep_fn=clock.sleep)

        completed = maneuver.run(
            commands.append,
            lambda: False,
            on_progress=lambda progress: progress.step_index < 2,
        )

        self.assertFalse(completed)
        self.assertEqual(commands[-1], RCCommand())
        self.assertNotIn(RCCommand(left_right=16), commands)

    def test_obstacle_avoidance_pauses_route_timer(self) -> None:
        clock = FakeClock()
        commands = []
        avoid_ticks = 3

        def is_avoiding() -> bool:
            nonlocal avoid_ticks
            if avoid_ticks > 0:
                avoid_ticks -= 1
                return True
            return False

        maneuver = FixedDemoManeuver(
            steps=(FixedDemoStep("one", RCCommand(forward_backward=10), 1.0, 0.0),),
            control_interval=0.1,
            clock=clock,
            sleep_fn=clock.sleep,
        )
        completed = maneuver.run(
            commands.append,
            lambda: False,
            is_avoiding=is_avoiding,
        )

        self.assertTrue(completed)
        self.assertEqual(avoid_ticks, 0)
        self.assertGreater(clock.now, 1.2)
        self.assertGreater(len(commands), 10)

    def test_avoidance_ticks_do_not_consume_route_time(self) -> None:
        clock = FakeClock()
        commands = []
        avoid_ticks = 10

        def is_avoiding() -> bool:
            nonlocal avoid_ticks
            if avoid_ticks > 0:
                avoid_ticks -= 1
                return True
            return False

        def send(command) -> None:
            commands.append(command)
            clock.now += 0.02  # 模拟读帧 + 避障决策的管线耗时

        maneuver = FixedDemoManeuver(
            steps=(FixedDemoStep("one", RCCommand(forward_backward=10), 1.0, 0.0),),
            control_interval=0.1,
            clock=clock,
            sleep_fn=clock.sleep,
        )
        completed = maneuver.run(send, lambda: False, is_avoiding=is_avoiding)

        self.assertTrue(completed)
        # route 的 1.0s 必须完整保留，不得被避障期间的管线耗时消耗：
        # 避障 10 次 ×（0.02 管线 + 0.1 控制间隔）= 1.2s 额外开销，
        # 另加 3 次零命令发送（段间清零、settle 段首帧、收尾）各 0.02s。
        # 修复前每帧避障管线耗时会被计入累计时间，总时长约 2.08s（少 0.2s）。
        self.assertAlmostEqual(clock.now, 1.0 + 10 * (0.02 + 0.1) + 3 * 0.02, places=1)


class FixedDemoIntegrationTestCase(unittest.TestCase):
    def test_session_runs_maneuver_then_resets_and_starts_follow(self) -> None:
        events = []
        drone = FakeDroneAdapter(verbose_rc=False)
        detector = ResetDetector()
        safety = build_safety()
        controller = FollowController(safety_manager=safety)
        maneuver = RecordingManeuver(events)
        session = RecordingSession(
            events=events,
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=controller,
            config={"display_console_camera": False},
            mode_label="FIXED-DEMO FAKE",
            pre_follow_maneuver=maneuver,
        )

        with patch("control.follow_session.sleep", return_value=None):
            result = session.run()

        self.assertEqual(events, ["maneuver", "follow"])
        self.assertTrue(maneuver.assert_not_aborted)
        self.assertEqual(detector.reset_count, 2)
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(drone.get_height(), 0)
        self.assertFalse(result.airborne)

    def test_default_follow_session_has_no_pre_follow_maneuver(self) -> None:
        safety = build_safety()
        session = FollowSession(
            drone=FakeDroneAdapter(verbose_rc=False),
            safety_manager=safety,
            detector=ResetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FAKE",
        )

        self.assertIsNone(session.pre_follow_maneuver)

    def test_aborted_maneuver_lands_and_never_enters_follow(self) -> None:
        events = []
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = RecordingSession(
            events=events,
            drone=drone,
            safety_manager=safety,
            detector=ResetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FIXED-DEMO FAKE",
            pre_follow_maneuver=AbortingManeuver(),
        )

        with patch("control.follow_session.sleep", return_value=None):
            result = session.run()

        self.assertEqual(events, [])
        self.assertEqual(result.state, "STOPPED")
        self.assertFalse(result.airborne)
        self.assertEqual(drone.get_height(), 0)
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))

    def test_fixed_demo_cli_dispatches_independently(self) -> None:
        args = SimpleNamespace(mode="fixed-demo", fake=False)
        with patch.object(main, "parse_args", return_value=args), patch.object(
            main, "run_fixed_demo", return_value=0
        ) as runner, patch.object(main, "prompt_obstacle_enabled", return_value=False):
            result = main.main()

        self.assertEqual(result, 0)
        runner.assert_called_once_with(use_fake=False, obstacle_enabled=False)


if __name__ == "__main__":
    unittest.main()
