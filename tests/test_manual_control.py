"""Tests for timed manual takeover and its conservative safety gates."""

import unittest

from control.features import build_features
from control.follow_control import FollowController, RCCommand
from control.kernel.arbitration import ArbitrationEngine
from control.kernel.features import ArbitrationContext
from control.kernel.phases import KernelPhase
from control.manual_control import ManualControlController
from drone.front_tof import FrontToFSnapshot
from drone.safety import SafetyConfig, SafetyManager


def safety(max_speed: int = 35) -> SafetyManager:
    return SafetyManager(SafetyConfig(20, 5, 220, 60, max_speed, 3, 8))


def controller(**overrides) -> ManualControlController:
    values = {
        "enabled": True,
        "forward_speed": 20,
        "lateral_speed": 20,
        "vertical_speed": 15,
        "yaw_speed": 20,
        "command_timeout_seconds": 0.25,
        "front_tof_guard_enabled": True,
        "front_stop_distance_cm": 60,
        "block_forward_when_tof_invalid": True,
        "minimum_descent_height_cm": 40,
        "maximum_ascent_height_cm": 200,
        **overrides,
    }
    result = ManualControlController.from_config(
        {"manual_control": values}, safety()
    )
    result.make_available()
    result.enable(0.0)
    return result


def snapshot(status="valid", distance=100.0) -> FrontToFSnapshot:
    return FrontToFSnapshot(
        distance_cm=distance,
        status=status,
        timestamp=0.0,
        age_seconds=0.0,
        sequence=1,
        consecutive_blocked=0,
    )


class ManualControlControllerTestCase(unittest.TestCase):
    def test_takeover_is_unavailable_before_base_height(self) -> None:
        manual = ManualControlController.from_config(
            {"manual_control": {"enabled": True}}, safety()
        )
        self.assertFalse(manual.enable(0.0))
        manual.make_available()
        self.assertTrue(manual.enable(0.0))

    def test_all_motion_keys_map_to_one_bounded_axis(self) -> None:
        manual = controller()
        expected = {
            "w": (0, 20, 0, 0),
            "s": (0, -20, 0, 0),
            "a": (-20, 0, 0, 0),
            "d": (20, 0, 0, 0),
            "r": (0, 0, 15, 0),
            "f": (0, 0, -15, 0),
            "j": (0, 0, 0, -20),
            "l": (0, 0, 0, 20),
        }
        for key, command in expected.items():
            self.assertTrue(manual.handle_key(ord(key), 1.0))
            actual = manual.command_for(
                now=1.1, height_cm=150, front_tof_snapshot=snapshot()
            )
            self.assertEqual(actual.as_tuple(), command)

    def test_command_expires_exactly_at_deadman_deadline(self) -> None:
        manual = controller(command_timeout_seconds=0.25)
        manual.handle_key(ord("w"), 1.0)
        self.assertEqual(
            manual.command_for(
                now=1.249, height_cm=150, front_tof_snapshot=snapshot()
            ).forward_backward,
            20,
        )
        self.assertEqual(
            manual.command_for(
                now=1.25, height_cm=150, front_tof_snapshot=snapshot()
            ).as_tuple(),
            (0, 0, 0, 0),
        )

    def test_space_immediately_hovers(self) -> None:
        manual = controller()
        manual.handle_key(ord("w"), 1.0)
        self.assertTrue(manual.handle_key(ord(" "), 1.1))
        self.assertEqual(
            manual.command_for(
                now=1.1, height_cm=150, front_tof_snapshot=snapshot()
            ).as_tuple(),
            (0, 0, 0, 0),
        )

    def test_front_tof_blocks_only_forward_motion(self) -> None:
        manual = controller()
        manual.handle_key(ord("w"), 1.0)
        self.assertEqual(
            manual.command_for(
                now=1.1, height_cm=150, front_tof_snapshot=snapshot(distance=60)
            ).forward_backward,
            0,
        )
        allowed = {
            "s": (0, -20, 0, 0),
            "a": (-20, 0, 0, 0),
            "d": (20, 0, 0, 0),
            "r": (0, 0, 15, 0),
            "f": (0, 0, -15, 0),
            "j": (0, 0, 0, -20),
            "l": (0, 0, 0, 20),
        }
        for index, (key, expected) in enumerate(allowed.items(), start=2):
            manual.handle_key(ord(key), float(index))
            actual = manual.command_for(
                now=float(index) + 0.1,
                height_cm=150,
                front_tof_snapshot=snapshot(distance=20),
            )
            self.assertEqual(actual.as_tuple(), expected, key)

    def test_tof_status_contract(self) -> None:
        manual = controller()
        for status in ("stale", "error", "not_ready"):
            manual.handle_key(ord("w"), 1.0)
            result = manual.command_for(
                now=1.1, height_cm=150, front_tof_snapshot=snapshot(status, None)
            )
            self.assertEqual(result.forward_backward, 0, status)
        manual.handle_key(ord("w"), 2.0)
        result = manual.command_for(
            now=2.1,
            height_cm=150,
            front_tof_snapshot=snapshot("out_of_range", None),
        )
        self.assertEqual(result.forward_backward, 20)

        manual.handle_key(ord("w"), 3.0)
        result = manual.command_for(
            now=3.1,
            height_cm=150,
            front_tof_snapshot=snapshot("valid", float("nan")),
        )
        self.assertEqual(result.forward_backward, 0)

    def test_height_guards_block_only_the_dangerous_vertical_direction(self) -> None:
        manual = controller()
        manual.handle_key(ord("r"), 1.0)
        self.assertEqual(
            manual.command_for(
                now=1.1, height_cm=200, front_tof_snapshot=snapshot()
            ).up_down,
            0,
        )

    def test_manual_height_ceiling_cannot_exceed_global_safety_ceiling(self) -> None:
        manager = SafetyManager(SafetyConfig(20, 5, 150, 60, 35, 3, 8))
        manual = ManualControlController.from_config(
            {
                "manual_control": {
                    "enabled": True,
                    "minimum_descent_height_cm": 40,
                    "maximum_ascent_height_cm": 200,
                },
            },
            manager,
        )
        self.assertEqual(manual.config.maximum_ascent_height_cm, 150)
        manual.make_available()
        self.assertTrue(manual.enable(0.0))
        manual.handle_key(ord("r"), 1.0)
        self.assertEqual(
            manual.command_for(
                now=1.1,
                height_cm=150,
                front_tof_snapshot=snapshot(),
            ).up_down,
            0,
        )
        manual.handle_key(ord("f"), 2.0)
        self.assertEqual(
            manual.command_for(
                now=2.1,
                height_cm=40,
                front_tof_snapshot=snapshot(),
            ).up_down,
            0,
        )
        manual.handle_key(ord("f"), 2.0)
        self.assertEqual(
            manual.command_for(
                now=2.1, height_cm=40, front_tof_snapshot=snapshot()
            ).up_down,
            0,
        )


class CountingSearch:
    feature_name = "search"

    def __init__(self) -> None:
        self.calls = 0

    def propose(self, ctx, now):
        self.calls += 1
        raise AssertionError("manual takeover must preempt search")


class ManualArbitrationTestCase(unittest.TestCase):
    def test_manual_preempts_first_target_gate_and_search(self) -> None:
        manager = safety()
        follow = FollowController(safety_manager=manager)
        manual = controller()
        search = CountingSearch()
        features = build_features(
            follow_controller=follow,
            safety_manager=manager,
            manual_controller=manual,
        )
        features["search"] = search
        engine = ArbitrationEngine(features=features, follow_controller=follow)
        manual.handle_key(ord("d"), 1.0)
        outcome = engine.arbitrate(
            ArbitrationContext(
                phase=KernelPhase.FOLLOW,
                target_result={"found": False},
                frame=object(),
                height_cm=150,
                front_tof_snapshot=snapshot(),
                now=1.1,
            )
        )
        self.assertEqual(outcome.state, "MANUAL")
        self.assertEqual(outcome.command, RCCommand(left_right=20))
        self.assertEqual(search.calls, 0)

    def test_pause_and_emergency_still_preempt_manual(self) -> None:
        manager = safety()
        follow = FollowController(safety_manager=manager)
        manual = controller()
        features = build_features(
            follow_controller=follow,
            safety_manager=manager,
            manual_controller=manual,
        )
        engine = ArbitrationEngine(features=features, follow_controller=follow)
        manual.handle_key(ord("w"), 1.0)
        for flags in ({"paused": True}, {"emergency": True}):
            outcome = engine.arbitrate(
                ArbitrationContext(
                    phase=KernelPhase.FOLLOW,
                    target_result={"found": True},
                    frame=object(),
                    height_cm=150,
                    front_tof_snapshot=snapshot(),
                    now=1.1,
                    **flags,
                )
            )
            self.assertEqual(outcome.command.as_tuple(), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
