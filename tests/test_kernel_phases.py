"""Tests for the kernel lifecycle phase handlers (control/kernel/phase_handlers)."""

import threading
import unittest
from types import SimpleNamespace

from control.kernel.phase_handlers import LifecycleContext, build_phase_handlers
from control.kernel.phase_handlers.climb import ClimbHandler
from control.kernel.phase_handlers.control_ready import ControlReadyHandler
from control.kernel.phase_handlers.follow import FollowHandler
from control.kernel.phase_handlers.ground_lock import GroundLockHandler
from control.kernel.phase_handlers.height_verify import HeightVerifyHandler
from control.kernel.phase_handlers.pre_follow import PreFollowHandler
from control.kernel.phase_handlers.stabilize import StabilizeHandler
from control.kernel.phase_handlers.takeoff import TakeoffHandler
from control.kernel.phases import KernelPhase


class RecordingDrone:
    def __init__(self) -> None:
        self.takeoff_calls = 0
        self.authorize_calls = 0

    def takeoff(self) -> None:
        self.takeoff_calls += 1

    def authorize_next_takeoff(self) -> None:
        self.authorize_calls += 1


class LoopRecorder:
    def __init__(self) -> None:
        self.loop_calls = 0

    def __call__(self) -> None:
        self.loop_calls += 1


class ManualControllerStub:
    def __init__(self, *, enabled: bool = False) -> None:
        self.config = SimpleNamespace(enabled=enabled)
        self.available = False
        self.active = False
        self.make_available_calls = 0

    def make_available(self) -> None:
        self.make_available_calls += 1
        self.available = bool(self.config.enabled)


class BaseSession:
    """Minimal mutable stand-in for FollowSession with the fields handlers touch."""

    def __init__(self) -> None:
        self.session_state = "IDLE"
        self.initial_target_lock_frames = 0
        self.window_takeoff_confirmation = False
        self.display_enabled = False
        self.pre_takeoff_confirmation = None
        self.stop_event = threading.Event()
        self.drone = RecordingDrone()
        self.airborne = False
        self.emergency_stop = False
        self.pre_follow_maneuver = None
        self.mode_label = "TEST"
        self.allow_pause = False
        self.emergency_stop = False
        self._lifecycle_lock = threading.Lock()
        self.safe_zero_calls = 0
        self.safe_land_calls = 0
        self.reset_calls = 0
        self._loop_impl = LoopRecorder()
        self.lock_result = {}
        self.stabilize_ok = True
        self.verify_height_ok = True
        self.climb_ok = True
        self.maneuver_completed = True
        self.config = {"takeoff_height_verify_enabled": False}
        self.manual_controller = ManualControllerStub()
        self.control_selection = "auto"
        self.control_selection_calls = 0

    def _safe_zero_output(self) -> None:
        self.safe_zero_calls += 1

    def _safe_land(self) -> None:
        self.safe_land_calls += 1

    def _reset_tracking_state(self) -> None:
        self.reset_calls += 1

    def _loop(self) -> None:
        self._loop_impl()

    def _wait_for_initial_target_lock(self):
        return dict(self.lock_result)

    def _wait_for_window_takeoff_confirmation(self):
        return dict(self.lock_result)

    def _verify_fresh_target_before_takeoff(self):
        return bool(self.lock_result)

    def _wait_for_takeoff_stabilization(self, duration):
        return self.stabilize_ok

    def _verify_takeoff_height(self):
        return self.verify_height_ok

    def _reach_base_hover_height(self):
        return self.climb_ok

    def _wait_for_control_selection(self):
        self.control_selection_calls += 1
        if self.control_selection == "manual":
            self.manual_controller.active = True
        return self.control_selection

    def send_motion_command(self, command) -> None:
        pass

    def _pre_follow_should_abort(self) -> bool:
        return self.stop_event.is_set()

    def _show_pre_follow_progress(self, progress):
        return True

    def _fixed_demo_is_avoiding(self) -> bool:
        return False


class GroundLockHandlerTestCase(unittest.TestCase):
    def test_skips_lock_when_no_reid(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        self.assertEqual(GroundLockHandler().run(session, ctx), KernelPhase.TAKEOFF)

    def test_locked_target_advances_to_takeoff(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.initial_target_lock_frames = 2
        session.lock_result = {"found": True}
        self.assertEqual(GroundLockHandler().run(session, ctx), KernelPhase.TAKEOFF)
        self.assertEqual(ctx.locked_result, {"found": True})

    def test_lock_failure_aborts_with_target_lock_failed(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.initial_target_lock_frames = 2
        self.assertIsNone(GroundLockHandler().run(session, ctx))
        self.assertEqual(session.session_state, "TARGET_LOCK_FAILED")

    def test_lock_cancel_preserves_stopped_state(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.initial_target_lock_frames = 2
        session.session_state = "STOPPED"
        self.assertIsNone(GroundLockHandler().run(session, ctx))
        self.assertEqual(session.session_state, "STOPPED")


class TakeoffHandlerTestCase(unittest.TestCase):
    def test_takeoff_sets_airborne_and_authorizes(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        ctx.locked_result = {"found": True}
        self.assertEqual(TakeoffHandler().run(session, ctx), KernelPhase.STABILIZING)
        self.assertTrue(session.airborne)
        self.assertEqual(session.drone.takeoff_calls, 1)
        self.assertEqual(session.drone.authorize_calls, 1)

    def test_pre_takeoff_confirmation_rejection_aborts(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.initial_target_lock_frames = 1
        session.lock_result = {"found": True}
        session.pre_takeoff_confirmation = lambda result: False
        self.assertIsNone(TakeoffHandler().run(session, ctx))
        self.assertEqual(session.session_state, "TAKEOFF_CANCELLED")
        self.assertEqual(session.drone.takeoff_calls, 0)

    def test_stop_request_before_takeoff_aborts_stopped(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.stop_event.set()
        self.assertIsNone(TakeoffHandler().run(session, ctx))
        self.assertEqual(session.session_state, "STOPPED")
        self.assertEqual(session.drone.takeoff_calls, 0)


class PostTakeoffHandlersTestCase(unittest.TestCase):
    def test_phase_chain_advances_to_follow(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        self.assertEqual(StabilizeHandler().run(session, ctx), KernelPhase.CLIMB)
        self.assertEqual(ClimbHandler().run(session, ctx), KernelPhase.CONTROL_READY)
        self.assertEqual(
            ControlReadyHandler().run(session, ctx), KernelPhase.PRE_FOLLOW
        )
        self.assertEqual(PreFollowHandler().run(session, ctx), KernelPhase.FOLLOW)

    def test_height_verify_can_be_explicitly_reenabled(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.config["takeoff_height_verify_enabled"] = True
        self.assertEqual(StabilizeHandler().run(session, ctx), KernelPhase.HEIGHT_VERIFY)
        self.assertEqual(HeightVerifyHandler().run(session, ctx), KernelPhase.CLIMB)

    def test_stabilize_failure_aborts(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.stabilize_ok = False
        self.assertIsNone(StabilizeHandler().run(session, ctx))

    def test_height_verify_failure_zeros_and_lands(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.airborne = True
        session.verify_height_ok = False
        self.assertIsNone(HeightVerifyHandler().run(session, ctx))
        self.assertEqual(session.safe_zero_calls, 1)
        self.assertEqual(session.safe_land_calls, 1)


class ControlReadyHandlerTestCase(unittest.TestCase):
    def test_disabled_manual_control_skips_selection_wait(self) -> None:
        session = BaseSession()

        phase = ControlReadyHandler().run(session, LifecycleContext())

        self.assertEqual(phase, KernelPhase.PRE_FOLLOW)
        self.assertEqual(session.control_selection_calls, 0)

    def test_manual_selection_enters_follow_without_pre_follow_route(self) -> None:
        session = BaseSession()
        session.manual_controller = ManualControllerStub(enabled=True)
        session.control_selection = "manual"

        phase = ControlReadyHandler().run(session, LifecycleContext())

        self.assertEqual(phase, KernelPhase.FOLLOW)
        self.assertTrue(session.manual_controller.available)
        self.assertTrue(session.manual_controller.active)

    def test_auto_selection_runs_pre_follow_route(self) -> None:
        session = BaseSession()
        session.manual_controller = ManualControllerStub(enabled=True)
        session.control_selection = "auto"

        phase = ControlReadyHandler().run(session, LifecycleContext())

        self.assertEqual(phase, KernelPhase.PRE_FOLLOW)
        self.assertTrue(session.manual_controller.available)

    def test_stopped_selection_ends_lifecycle(self) -> None:
        session = BaseSession()
        session.manual_controller = ManualControllerStub(enabled=True)
        session.control_selection = None

        self.assertIsNone(ControlReadyHandler().run(session, LifecycleContext()))

    def test_climb_failure_zeros_and_lands(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.airborne = True
        session.climb_ok = False
        self.assertIsNone(ClimbHandler().run(session, ctx))
        self.assertEqual(session.safe_zero_calls, 1)
        self.assertEqual(session.safe_land_calls, 1)


class PreFollowHandlerTestCase(unittest.TestCase):
    class AbortableManeuver:
        def __init__(self, completed: bool) -> None:
            self.completed = completed
            self.ran = False

        def run(self, send_command, should_abort, on_progress, is_avoiding):
            self.ran = True
            return self.completed

    def test_aborted_maneuver_stops_lifecycle(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        maneuver = self.AbortableManeuver(False)
        session.pre_follow_maneuver = maneuver
        self.assertIsNone(PreFollowHandler().run(session, ctx))
        self.assertTrue(maneuver.ran)
        self.assertEqual(session.session_state, "STOPPED")

    def test_completed_maneuver_advances_and_resets(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        maneuver = self.AbortableManeuver(True)
        session.pre_follow_maneuver = maneuver
        self.assertEqual(PreFollowHandler().run(session, ctx), KernelPhase.FOLLOW)
        self.assertEqual(session.reset_calls, 1)
        self.assertEqual(session.manual_controller.make_available_calls, 1)

    def test_manual_takeover_during_maneuver_enters_follow_without_landing(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.manual_controller = ManualControllerStub(enabled=True)
        session.manual_controller.make_available()
        session.manual_controller.active = True
        maneuver = self.AbortableManeuver(False)
        session.pre_follow_maneuver = maneuver

        self.assertEqual(PreFollowHandler().run(session, ctx), KernelPhase.FOLLOW)
        self.assertEqual(session.session_state, "MANUAL")
        self.assertEqual(session.safe_zero_calls, 1)

    def test_success_result_cannot_override_same_tick_manual_takeover(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.manual_controller = ManualControllerStub(enabled=True)
        session.manual_controller.make_available()

        class LastTickTakeover:
            def run(inner_self, **_kwargs):
                session.manual_controller.active = True
                return True

        session.pre_follow_maneuver = LastTickTakeover()

        self.assertEqual(PreFollowHandler().run(session, ctx), KernelPhase.FOLLOW)
        self.assertEqual(session.session_state, "MANUAL")
        self.assertEqual(session.reset_calls, 0)
        self.assertEqual(session.safe_zero_calls, 1)

    def test_no_maneuver_advances_directly(self) -> None:
        ctx = LifecycleContext()
        self.assertEqual(PreFollowHandler().run(BaseSession(), ctx), KernelPhase.FOLLOW)


class FollowHandlerTestCase(unittest.TestCase):
    def test_enters_following_and_runs_loop(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        self.assertIsNone(FollowHandler().run(session, ctx))
        self.assertEqual(session.session_state, "FOLLOWING")
        self.assertEqual(session._loop_impl.loop_calls, 1)

    def test_preserves_manual_owner_on_entry(self) -> None:
        ctx = LifecycleContext()
        session = BaseSession()
        session.manual_controller = ManualControllerStub(enabled=True)
        session.manual_controller.active = True

        self.assertIsNone(FollowHandler().run(session, ctx))
        self.assertEqual(session.session_state, "MANUAL")


class BuildHandlersTestCase(unittest.TestCase):
    def test_registry_has_all_phases(self) -> None:
        handlers = build_phase_handlers()
        self.assertEqual(
            set(handlers.keys()),
            {
                KernelPhase.PRE_FLIGHT,
                KernelPhase.TAKEOFF,
                KernelPhase.STABILIZING,
                KernelPhase.HEIGHT_VERIFY,
                KernelPhase.CLIMB,
                KernelPhase.CONTROL_READY,
                KernelPhase.PRE_FOLLOW,
                KernelPhase.FOLLOW,
            },
        )


if __name__ == "__main__":
    unittest.main()
