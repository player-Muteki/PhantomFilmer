import json
import sys
import unittest
from unittest.mock import patch
from http.client import HTTPConnection
from pathlib import Path
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from threading import Event, Thread
from time import monotonic, sleep, time

from app.runtime.commands import (
    ConnectCommand,
    SelectControlModeCommand,
    StartMissionCommand,
    StartPreviewCommand,
    StopMissionCommand,
    StopPreviewCommand,
    TakeoffCommand,
    command_from_payload,
)
from app.runtime.models import AllowedAction, ControlMode, MissionKind, RuntimePhase
from control.operator_commands import OperatorCommand
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
        self.height = 12
        self.battery_failure = False
        self.height_failure = False

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
        if self.battery_failure:
            raise RuntimeError("battery unavailable")
        return self.battery

    def get_height(self) -> int:
        if self.height_failure:
            raise RuntimeError("height unavailable")
        return self.height

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


class MissionSessionStub:
    def __init__(self) -> None:
        self.started = Event()
        self.finished = Event()
        self.session_state = "PREPARING"
        self.airborne = False
        self.last_battery = 76
        self.last_height = 150
        self.control_hz = 12.5
        self.follow_mode = "normal"
        self.manual_controller = SimpleNamespace(active=False)
        self.emergency = False

    def run(self):
        self.airborne = True
        self.session_state = "CONTROL_READY"
        self.started.set()
        self.finished.wait(timeout=2)
        self.airborne = False
        self.session_state = "EMERGENCY_STOP" if self.emergency else "STOPPED"
        return SimpleNamespace(
            state=self.session_state,
            airborne=False,
            streaming=False,
        )

    def request_stop(self) -> None:
        self.finished.set()

    def request_emergency_stop(self) -> None:
        self.emergency = True
        self.finished.set()


class PreviewDetectorStub:
    def __init__(self) -> None:
        self.prepared = False
        self.reset_count = 0

    def prepare(self) -> None:
        self.prepared = True

    def reset(self) -> None:
        self.reset_count += 1

    def detect(self, _frame):
        return {
            "found": True,
            "is_predicted": False,
            "similarity": 0.82,
            "similarity_threshold": 0.65,
            "candidate_count": 1,
            "area_ratio": 0.25,
            "body_orientation_angle": 90.0,
        }

    def draw_debug(self, frame, _result):
        return frame


class Cv2Stub:
    IMWRITE_JPEG_QUALITY = 1

    @staticmethod
    def imencode(_extension, _frame, _options):
        return True, SimpleNamespace(tobytes=lambda: b"preview-jpeg")


class DroneWebServiceTests(unittest.TestCase):
    @staticmethod
    def _fast_runtime_config() -> dict:
        return {
            "min_battery_takeoff": 20,
            "low_battery_land": 8,
            "max_height_cm": 220,
            "min_height_cm": 60,
            "max_rc_speed": 35,
            "height_failure_limit": 2,
            "desktop_telemetry_poll_seconds": 0.01,
            "desktop_telemetry_max_age_seconds": 0.05,
            "manual_control": {
                "minimum_descent_height_cm": 40,
                "maximum_ascent_height_cm": 200,
                "front_stop_distance_cm": 60,
            },
            "obstacle": {"front_tof_max_age_seconds": 0.8},
        }

    def test_mission_payload_parses_profile_mode_and_obstacle_choice(self) -> None:
        command = command_from_payload(
            {
                "type": "mission.start",
                "mission": "reid_follow",
                "profileName": "operator-a",
                "initialControlMode": "side",
                "obstacleEnabled": True,
            }
        )

        self.assertIsInstance(command, StartMissionCommand)
        self.assertEqual(command.profile_name, "operator-a")
        self.assertEqual(command.initial_control_mode, ControlMode.SIDE)
        self.assertTrue(command.obstacle_enabled)

    def test_desktop_mission_cannot_disable_required_front_tof_safety(self) -> None:
        service = DroneWebService(adapter_factory=RealAdapterStub)
        self.addCleanup(service.shutdown)

        with self.assertRaisesRegex(RuntimeError, "强制项"):
            service.start_mission(
                StartMissionCommand(
                    mission=MissionKind.FOLLOW,
                    profile_name="operator-a",
                    obstacle_enabled=False,
                )
            )

    def test_preview_payload_requires_profile_name(self) -> None:
        command = command_from_payload(
            {"type": "preview.start", "profileName": "operator-a"}
        )
        self.assertIsInstance(command, StartPreviewCommand)
        self.assertEqual(command.profile_name, "operator-a")

        with self.assertRaisesRegex(ValueError, "profileName"):
            command_from_payload({"type": "preview.start", "profileName": ""})

    def test_ground_preview_is_optional_before_automatic_mission(self) -> None:
        adapter = RealAdapterStub()
        detector = PreviewDetectorStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        self.addCleanup(service.shutdown)
        service.connect()

        with patch("web_api.server.create_detector", return_value=detector):
            result = service.execute(StartPreviewCommand(profile_name="operator-a"))

        self.assertTrue(result["active"])
        self.assertTrue(detector.prepared)
        snapshot = service.runtime_snapshot()
        self.assertIn(AllowedAction.STOP_PREVIEW, snapshot.allowed_actions)
        self.assertIn(AllowedAction.START_MISSION, snapshot.allowed_actions)
        self.assertFalse(
            service.capabilities()["preview"]["requiredForAutomaticMission"]
        )

        with service._lock:
            service._preview_confirmed = True
            service._preview_found_frames = service._preview_stable_frames
            service._preview_last_at = monotonic()
        stopped = service.execute(StopPreviewCommand())
        self.assertFalse(stopped["active"])
        self.assertEqual(detector.reset_count, 1)

    def test_ground_preview_annotates_stream_and_reaches_stable_confirmation(
        self,
    ) -> None:
        adapter = RealAdapterStub()
        detector = PreviewDetectorStub()
        config = self._fast_runtime_config()
        config["reid_lock_stable_frames"] = 1
        service = DroneWebService(
            adapter_factory=lambda: adapter,
            runtime_config=config,
        )
        self.addCleanup(service.shutdown)
        service.connect()
        with patch("web_api.server.create_detector", return_value=detector):
            service.start_preview(StartPreviewCommand(profile_name="operator-a"))

        with patch.dict(sys.modules, {"cv2": Cv2Stub}):
            payload = next(service.mjpeg_frames())

        preview = service.runtime_snapshot().telemetry["preview"]
        self.assertIn(b"preview-jpeg", payload)
        self.assertTrue(preview["confirmed"])
        self.assertEqual(preview["stableFrames"], 1)
        self.assertEqual(preview["orientationDeg"], 90.0)

    def test_stopping_preview_while_models_prepare_cannot_reactivate_it(self) -> None:
        adapter = RealAdapterStub()
        detector = PreviewDetectorStub()
        prepare_started = Event()
        release_prepare = Event()
        errors = []

        def prepare() -> None:
            prepare_started.set()
            release_prepare.wait(timeout=1)

        detector.prepare = prepare
        service = DroneWebService(adapter_factory=lambda: adapter)
        self.addCleanup(service.shutdown)
        service.connect()

        def start_preview() -> None:
            try:
                service.start_preview(StartPreviewCommand(profile_name="operator-a"))
            except RuntimeError as exc:
                errors.append(str(exc))

        with patch("web_api.server.create_detector", return_value=detector):
            worker = Thread(target=start_preview)
            worker.start()
            self.assertTrue(prepare_started.wait(timeout=1))
            stopped = service.stop_preview()
            release_prepare.set()
            worker.join(timeout=1)

        self.assertFalse(worker.is_alive())
        self.assertFalse(stopped["active"])
        self.assertEqual(
            service.runtime_snapshot().telemetry["preview"]["state"], "idle"
        )
        self.assertEqual(detector.reset_count, 1)
        self.assertTrue(any("状态已变化" in error for error in errors))

    def test_background_mission_updates_snapshot_accepts_mode_and_stops(self) -> None:
        adapter = RealAdapterStub()
        session = MissionSessionStub()
        service = DroneWebService(
            adapter_factory=lambda: adapter,
            mission_session_factory=lambda _command, _adapter, _channel: session,
        )
        self.addCleanup(service.shutdown)
        service.connect()

        result = service.execute(
            StartMissionCommand(
                mission=MissionKind.FOLLOW,
                profile_name="operator-a",
                initial_control_mode=ControlMode.SIDE,
            )
        )
        self.assertTrue(session.started.wait(timeout=1))
        snapshot = service.runtime_snapshot()
        mode_result = service.execute(SelectControlModeCommand(mode=ControlMode.FRONT))
        service.execute(StopMissionCommand())

        self.assertTrue(result["ok"])
        self.assertEqual(snapshot.phase, RuntimePhase.AIRBORNE)
        self.assertEqual(snapshot.mission, MissionKind.FOLLOW)
        self.assertIn(AllowedAction.STOP_MISSION, snapshot.allowed_actions)
        self.assertIn(AllowedAction.TOGGLE_MISSION_PAUSE, snapshot.allowed_actions)
        self.assertEqual(mode_result["mode"], "front")
        self.assertFalse(service.runtime_snapshot().airborne)
        self.assertTrue(service.connected)

    def test_automatic_mission_rejects_parallel_manual_takeoff(self) -> None:
        session = MissionSessionStub()
        service = DroneWebService(
            adapter_factory=RealAdapterStub,
            mission_session_factory=lambda _command, _adapter, _channel: session,
        )
        self.addCleanup(service.shutdown)
        service.connect()
        service.start_mission(
            StartMissionCommand(
                mission=MissionKind.REID_FOLLOW,
                profile_name="operator-a",
            )
        )
        self.assertTrue(session.started.wait(timeout=1))

        with self.assertRaisesRegex(RuntimeError, "自动任务运行中"):
            service.takeoff()

        service.emergency_stop_mission()
        self.assertTrue(session.emergency)

    def test_legacy_input_key_uses_the_same_operator_channel_as_mode_buttons(
        self,
    ) -> None:
        session = MissionSessionStub()
        service = DroneWebService(
            adapter_factory=RealAdapterStub,
            mission_session_factory=lambda _command, _adapter, _channel: session,
        )
        self.addCleanup(service.shutdown)
        service.connect()
        service.start_mission(
            StartMissionCommand(
                mission=MissionKind.FOLLOW,
                profile_name="operator-a",
                initial_control_mode=ControlMode.MANUAL,
            )
        )
        self.assertTrue(session.started.wait(timeout=1))
        service._operator_commands.clear()

        result = service.input_key({"key": "2"})
        queued = service._operator_commands.receive()

        self.assertTrue(result["ok"])
        self.assertEqual(result["key"], "2")
        self.assertIsNotNone(queued)
        self.assertEqual(queued.command, OperatorCommand.SELECT_SIDE)

        session.manual_controller.active = True
        service._operator_commands.clear()
        service.input_key({"key": "w"})
        manual_queued = service._operator_commands.receive()
        self.assertIsNotNone(manual_queued)
        self.assertEqual(manual_queued.command, OperatorCommand.MOVE_FORWARD)

        service.stop_mission()

    def test_mission_manual_takeover_routes_leased_rc_through_operator_channel(
        self,
    ) -> None:
        session = MissionSessionStub()
        service = DroneWebService(
            adapter_factory=RealAdapterStub,
            mission_session_factory=lambda _command, _adapter, _channel: session,
        )
        self.addCleanup(service.shutdown)
        service.connect()
        service.start_mission(
            StartMissionCommand(
                mission=MissionKind.FOLLOW,
                profile_name="operator-a",
                initial_control_mode=ControlMode.MANUAL,
            )
        )
        self.assertTrue(session.started.wait(timeout=1))
        session.manual_controller.active = True
        service._operator_commands.clear()
        lease = service.acquire_rc_lease()
        service._operator_commands.clear()

        result = service.move_rc_with_lease(
            {
                "leaseId": lease["leaseId"],
                "sequence": 1,
                "issuedAt": int(time() * 1000),
                "leftRight": 20,
            }
        )

        self.assertTrue(result["ok"])
        self.assertEqual(
            service._operator_commands.receive().command.value,
            "move_right",
        )
        service._operator_commands.clear()
        hover_status = service.hover()
        self.assertEqual(
            service._operator_commands.receive().command.value,
            "hover",
        )
        self.assertTrue(hover_status["rcEnabled"])
        service.stop_mission()

    def test_typed_commands_publish_authoritative_snapshots(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        self.addCleanup(service.shutdown)

        connected = service.execute(ConnectCommand(command_id="connect-1"))
        airborne = service.execute(TakeoffCommand(command_id="takeoff-1"))
        events = service.events.events_since(0)

        self.assertTrue(connected["canTakeoff"])
        self.assertTrue(airborne["airborne"])
        self.assertEqual(
            [event.event_type for event in events],
            [
                "command.accepted",
                "command.completed",
                "command.accepted",
                "command.completed",
            ],
        )
        snapshot = events[-1].snapshot
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertEqual(snapshot.phase, RuntimePhase.AIRBORNE)
        self.assertIn(AllowedAction.LAND, snapshot.allowed_actions)
        self.assertNotIn(AllowedAction.TAKEOFF, snapshot.allowed_actions)

    def test_rejected_command_is_recorded_without_hiding_the_error(self) -> None:
        service = DroneWebService(adapter_factory=RealAdapterStub)
        self.addCleanup(service.shutdown)

        with self.assertRaisesRegex(RuntimeError, "真机未连接"):
            service.execute(TakeoffCommand(command_id="unsafe-takeoff"))

        events = service.events.events_since(0)
        self.assertEqual(
            [event.event_type for event in events],
            ["command.accepted", "command.rejected"],
        )
        self.assertEqual(events[-1].payload["commandId"], "unsafe-takeoff")
        self.assertEqual(events[-1].snapshot.phase, RuntimePhase.ERROR)

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

    def test_manual_safety_monitor_lands_on_low_battery(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(
            adapter_factory=lambda: adapter,
            runtime_config=self._fast_runtime_config(),
        )
        self.addCleanup(service.shutdown)
        service.connect()
        service.takeoff()

        adapter.battery = 8
        deadline = time() + 1
        while not adapter.landed and time() < deadline:
            sleep(0.01)

        self.assertTrue(adapter.landed)
        self.assertFalse(service.runtime_snapshot().airborne)
        self.assertIn("电量降至", service.runtime_snapshot().telemetry["safetyReason"])
        self.assertIn(
            "flight.safety_landing.completed",
            [event.event_type for event in service.events.events_since(0)],
        )

    def test_manual_safety_monitor_lands_after_height_telemetry_failures(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(
            adapter_factory=lambda: adapter,
            runtime_config=self._fast_runtime_config(),
        )
        self.addCleanup(service.shutdown)
        service.connect()
        service.takeoff()

        adapter.height_failure = True
        deadline = time() + 1
        while not adapter.landed and time() < deadline:
            sleep(0.01)

        self.assertTrue(adapter.landed)
        self.assertIn("底部 ToF", service.runtime_snapshot().telemetry["safetyReason"])

    def test_vertical_manual_control_rejects_missing_height_sample(self) -> None:
        adapter = RealAdapterStub()
        service = DroneWebService(adapter_factory=lambda: adapter)
        self.addCleanup(service.shutdown)
        service.connect()
        service.takeoff()
        adapter.height_failure = True
        service.status()

        with self.assertRaisesRegex(RuntimeError, "底部 ToF 数据无效或过期"):
            service.move_rc({"upDown": 20})

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
        json_body: dict | None = None,
    ) -> tuple[int, str]:
        connection = HTTPConnection(
            "127.0.0.1", self.server.server_address[1], timeout=2
        )
        headers = {"X-Phantom-Token": token} if token is not None else {}
        body = None
        if json_body is not None:
            body = json.dumps(json_body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        connection.request(method, path, body=body, headers=headers)
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
        self.assertEqual(
            [event.payload["command"] for event in self.service.events.events_since(0)],
            ["device.connect", "device.connect"],
        )

    def test_v1_command_snapshot_and_event_replay_contract(self) -> None:
        status, body = self._request(
            "POST",
            "/api/v1/commands",
            json_body={
                "type": "device.connect",
                "commandId": "v1-connect",
                "issuedAt": int(time() * 1000),
            },
        )
        command = json.loads(body)
        snapshot_status, snapshot_body = self._request(
            "GET", "/api/v1/runtime/snapshot"
        )
        events_status, events_body = self._request(
            "GET", "/api/v1/runtime/events?since=0"
        )
        snapshot = json.loads(snapshot_body)
        events = json.loads(events_body)

        self.assertEqual(status, 200)
        self.assertEqual(command["apiVersion"], "1")
        self.assertEqual(command["commandId"], "v1-connect")
        self.assertEqual(command["snapshot"]["phase"], "preflight")
        self.assertEqual(snapshot_status, 200)
        self.assertIn("takeoff", snapshot["snapshot"]["allowedActions"])
        self.assertEqual(events_status, 200)
        self.assertFalse(events["resetRequired"])
        self.assertEqual(
            [event["type"] for event in events["events"]],
            ["command.accepted", "command.completed"],
        )

    def test_v1_rc_lease_rejects_replayed_sequence_and_release_hovers(self) -> None:
        self._request("POST", "/api/drone/connect")
        self._request("POST", "/api/drone/takeoff")
        lease_status, lease_body = self._request("POST", "/api/v1/rc/lease")
        lease = json.loads(lease_body)
        payload = {
            "leaseId": lease["leaseId"],
            "sequence": 1,
            "issuedAt": int(time() * 1000),
            "leftRight": 20,
            "forwardBack": 0,
            "upDown": 0,
            "yaw": 0,
        }

        move_status, _ = self._request("POST", "/api/v1/rc", json_body=payload)
        replay_status, replay_body = self._request(
            "POST",
            "/api/v1/rc",
            json_body={**payload, "issuedAt": payload["issuedAt"] + 1},
        )
        release_status, release_body = self._request(
            "POST",
            "/api/v1/rc/release",
            json_body={"leaseId": lease["leaseId"]},
        )

        self.assertEqual(lease_status, 201)
        self.assertEqual(move_status, 200)
        self.assertEqual(replay_status, 409)
        self.assertEqual(
            json.loads(replay_body)["error"]["code"], "RC_COMMAND_REJECTED"
        )
        self.assertEqual(release_status, 200)
        self.assertTrue(json.loads(release_body)["released"])
        adapter = self.service._adapter
        self.assertIsNotNone(adapter)
        self.assertEqual(adapter.rc_commands[-1], (0, 0, 0, 0))

    def test_v1_rc_safety_rejection_revokes_unobserved_lease_state(self) -> None:
        self._request("POST", "/api/drone/connect")
        self._request("POST", "/api/drone/takeoff")
        _, lease_body = self._request("POST", "/api/v1/rc/lease")
        lease = json.loads(lease_body)
        adapter = self.service._adapter
        self.assertIsNotNone(adapter)
        adapter.front_distance = 45
        self.service._refresh_front_tof(force=True)
        payload = {
            "leaseId": lease["leaseId"],
            "sequence": 1,
            "issuedAt": int(time() * 1000),
            "forwardBack": 20,
        }

        blocked_status, _ = self._request("POST", "/api/v1/rc", json_body=payload)
        revoked_status, revoked_body = self._request(
            "POST",
            "/api/v1/rc",
            json_body={**payload, "sequence": 2, "issuedAt": payload["issuedAt"] + 1},
        )

        self.assertEqual(blocked_status, 409)
        self.assertEqual(revoked_status, 409)
        self.assertIn("无效或已释放", json.loads(revoked_body)["error"]["message"])
        self.assertEqual(adapter.rc_commands[-1], (0, 0, 0, 0))

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
