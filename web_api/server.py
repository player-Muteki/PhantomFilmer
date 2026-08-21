"""Serve real-device telemetry and MJPEG video to the desktop application.

The service binds to loopback only. It never creates an OpenCV window and it
does not connect to an aircraft until the user requests POST /api/drone/connect.
"""

from __future__ import annotations

import argparse
import hmac
import json
import logging
import secrets
import signal
from functools import partial
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Event, RLock, Thread
from time import monotonic, sleep, time
from typing import Any, Callable, Iterator, Optional
from urllib.parse import parse_qs, urlsplit

from app.runtime.commands import (
    ConnectCommand,
    EmergencyLandCommand,
    EmergencyStopCommand,
    HoverCommand,
    LandCommand,
    MoveRcCommand,
    RefreshStatusCommand,
    SelectControlModeCommand,
    StartMissionCommand,
    StopCommand,
    StopMissionCommand,
    TakeoffCommand,
    ToggleMissionPauseCommand,
    command_from_payload,
)
from app.builder import build_obstacle_modules
from app.config import load_runtime_config, read_control_interval
from app.runtime.mission_factory import MissionFactory
from app.runtime.mission_manager import MissionManager
from app.runtime.models import (
    AllowedAction,
    ControlMode,
    MissionKind,
    RuntimePhase,
    RuntimeSnapshot,
)
from app.runtime.rc_lease import RcLeaseManager
from control.fixed_demo import FixedDemoManeuver
from control.follow_control import FollowController
from control.operator_commands import OperatorCommand, OperatorCommandChannel
from drone.safety import SafetyManager
from vision.detector_factory import create_detector
from vision.reid_enrollment import build_reid_runtime_config
from vision.reid_enrollment import validate_reference_images
from vision.reid_profiles import list_reid_profiles, save_reid_profile

from .tello_adapter import RealTelloAdapter


HOST = "127.0.0.1"
PORT = 0
API_VERSION = "1"
VIDEO_BOUNDARY = "frame"
MIN_TAKEOFF_BATTERY = 20
MAX_RC_SPEED = 35
RC_WATCHDOG_SECONDS = 0.4
FRONT_STOP_DISTANCE_CM = 60.0
MIN_DESCENT_HEIGHT_CM = 40
MAX_ASCENT_HEIGHT_CM = 200


class DroneWebService(MissionManager):
    """Own one real-aircraft session shared by all local HTTP requests."""

    def __init__(
        self,
        adapter_factory: Optional[Callable[[], Any]] = None,
        *,
        data_dir: str | Path = ".",
        mission_session_factory: Optional[
            Callable[[StartMissionCommand, Any, OperatorCommandChannel], Any]
        ] = None,
    ) -> None:
        super().__init__()
        self.data_dir = Path(data_dir).expanduser().resolve()
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self._adapter_factory = adapter_factory or partial(RealTelloAdapter, self.data_dir)
        self._custom_mission_session_factory = mission_session_factory is not None
        self._mission_session_factory = (
            mission_session_factory or self._build_mission_session
        )
        self._adapter: Optional[Any] = None
        self._lock = RLock()
        self._video_ready = False
        self._last_frame_at: Optional[float] = None
        self._control_hz = 0.0
        self._battery: Optional[int] = None
        self._front_tof: Optional[float] = None
        self._front_tof_checked_at = 0.0
        self._front_tof_state = "unavailable"
        self._height: Optional[int] = None
        self._airborne = False
        self._flight_state = "未连接"
        self._phase = "连接"
        self._last_rc_at = 0.0
        self._rc_active = False
        self._rc_leases = RcLeaseManager(ttl_seconds=1.0)
        self._operator_commands = OperatorCommandChannel()
        self._mission_kind = MissionKind.IDLE
        self._mission_session: Optional[Any] = None
        self._mission_thread: Optional[Thread] = None
        self._mission_error: Optional[str] = None
        self._watchdog_stop = Event()
        self._watchdog = Thread(target=self._watch_rc, daemon=True)
        self._tof_monitor = Thread(target=self._monitor_front_tof, daemon=True)
        self._watchdog.start()
        self._tof_monitor.start()

    def capabilities(self) -> dict[str, Any]:
        """Describe implemented commands separately from local asset readiness."""

        missing_assets: list[str] = []
        if not self._custom_mission_session_factory:
            try:
                config = load_runtime_config()
                vision = config.get("vision", {})
                cfg = vision if isinstance(vision, dict) else {}
                project_root = Path(__file__).resolve().parents[1]
                for key in (
                    "person_detector_model",
                    "reid_model_path",
                    "jointbdoe_model_path",
                ):
                    value = str(cfg.get(key, "")).strip()
                    if not value:
                        missing_assets.append(key)
                        continue
                    path = Path(value).expanduser()
                    resolved = path if path.is_absolute() else project_root / path
                    if not resolved.is_file():
                        missing_assets.append(key)
            except Exception as exc:
                missing_assets.append(f"config:{exc}")
        return {
            "apiVersion": API_VERSION,
            "commands": [
                "device.connect",
                "device.status.refresh",
                "flight.takeoff",
                "flight.land",
                "flight.hover",
                "device.stop",
                "flight.emergency_land",
                "mission.start",
                "mission.stop",
                "mission.emergency_stop",
                "mission.control_mode.select",
                "mission.pause.toggle",
            ],
            "missions": ["manual", "follow", "reid_follow", "fixed_demo"],
            "eventReplay": True,
            "rcLease": {"required": True, "ttlMs": 1000},
            "missionReadiness": {
                "available": not missing_assets,
                "missingAssets": missing_assets,
                "profileRequired": True,
            },
        }

    def list_profiles(self) -> list[dict[str, object]]:
        return list_reid_profiles(self.data_dir / "reid_profiles")

    def enroll_profile(
        self,
        *,
        name: object,
        image_paths: object,
        overwrite: object = False,
    ) -> dict[str, object]:
        """Create a local profile from main-process-selected image paths."""

        if self.connected or self._mission_session is not None:
            raise RuntimeError("人物建档只能在无人机断开连接时执行。")
        if not isinstance(name, str) or not name.strip():
            raise RuntimeError("人物档案名不能为空。")
        if not isinstance(image_paths, list) or not all(
            isinstance(path, str) for path in image_paths
        ):
            raise RuntimeError("参考照片列表格式无效。")
        if not isinstance(overwrite, bool):
            raise RuntimeError("覆盖选项格式无效。")
        selected_images = validate_reference_images(image_paths)
        config = build_reid_runtime_config(load_runtime_config(), selected_images)
        config = dict(config)
        vision = config.get("vision", {})
        runtime_vision = dict(vision) if isinstance(vision, dict) else {}
        runtime_vision["jointbdoe_enabled"] = False
        config["vision"] = runtime_vision
        detector = create_detector(config)
        prepare = getattr(detector, "prepare", None)
        if not callable(prepare):
            raise RuntimeError("当前检测器不支持人物档案注册。")
        prepare()
        reference_feature = getattr(detector, "reference_feature", None)
        if reference_feature is None:
            raise RuntimeError("当前检测器未生成可保存的人物特征。")
        manifest = save_reid_profile(
            name.strip(),
            reference_feature,
            config,
            selected_images,
            overwrite=overwrite,
            profile_root=self.data_dir / "reid_profiles",
        )
        self.events.publish(
            "profile.enrolled",
            {"name": manifest["profile_name"]},
            snapshot=self.runtime_snapshot(),
        )
        return {
            "name": manifest["profile_name"],
            "createdAt": manifest["created_at"],
            "photoCount": manifest["photo_count"],
            "embeddingDimension": manifest["embedding_dimension"],
            "modelName": manifest["reid_model_name"],
        }

    @property
    def connected(self) -> bool:
        adapter = self._adapter
        return bool(adapter is not None and getattr(adapter, "connected", False))

    def runtime_snapshot(self, *, error: str | None = None) -> RuntimeSnapshot:
        """Build a non-blocking state snapshot for events and future GUI clients."""
        with self._lock:
            connected = self.connected
            mission_session = self._mission_session
            mission_active = mission_session is not None
            mission_airborne = bool(
                mission_active and getattr(mission_session, "airborne", False)
            )
            airborne = mission_airborne or self._airborne
            if error or self._mission_error:
                phase = RuntimePhase.ERROR
            elif not connected:
                phase = RuntimePhase.ERROR if error else RuntimePhase.DISCONNECTED
            elif mission_active:
                session_state = str(
                    getattr(mission_session, "session_state", "PREPARING")
                )
                if session_state in {
                    "STOPPED",
                    "EMERGENCY_STOP",
                    "LOW_BATTERY_LANDING",
                    "HEIGHT_LIMIT_LANDING",
                    "TARGET_LOST_LANDING",
                    "FRAME_LOST_LANDING",
                    "HEIGHT_SENSOR_LANDING",
                }:
                    phase = RuntimePhase.LANDING
                elif airborne:
                    phase = RuntimePhase.AIRBORNE
                else:
                    phase = RuntimePhase.TAKING_OFF
            elif self._phase == "连接":
                phase = RuntimePhase.CONNECTING
            elif self._phase == "起飞":
                phase = RuntimePhase.TAKING_OFF
            elif self._phase == "降落":
                phase = RuntimePhase.LANDING
            elif airborne:
                phase = RuntimePhase.AIRBORNE
            else:
                phase = RuntimePhase.PREFLIGHT

            allowed = []
            if not connected:
                allowed.append(AllowedAction.CONNECT)
            elif mission_active:
                allowed.extend(
                    (
                        AllowedAction.STOP_MISSION,
                        AllowedAction.EMERGENCY_STOP_MISSION,
                        AllowedAction.SELECT_CONTROL_MODE,
                    )
                )
                manual_controller = getattr(mission_session, "manual_controller", None)
                if airborne and getattr(manual_controller, "active", False):
                    allowed.extend((AllowedAction.HOVER, AllowedAction.MOVE_RC))
            else:
                allowed.extend((AllowedAction.REFRESH_STATUS, AllowedAction.STOP))
                front_tof_ready = self._front_tof_state in ("clear", "out_of_range")
                if (
                    not airborne
                    and self._video_ready
                    and self._battery is not None
                    and self._battery >= MIN_TAKEOFF_BATTERY
                    and self._height is not None
                    and front_tof_ready
                ):
                    allowed.append(AllowedAction.TAKEOFF)
                    allowed.append(AllowedAction.START_MISSION)
                if airborne:
                    allowed.extend(
                        (
                            AllowedAction.LAND,
                            AllowedAction.HOVER,
                            AllowedAction.MOVE_RC,
                            AllowedAction.EMERGENCY_LAND,
                        )
                    )

            if mission_active:
                manual_controller = getattr(mission_session, "manual_controller", None)
                if getattr(manual_controller, "active", False):
                    control_mode = ControlMode.MANUAL
                else:
                    try:
                        control_mode = ControlMode(
                            str(getattr(mission_session, "follow_mode", "normal"))
                        )
                    except ValueError:
                        control_mode = ControlMode.NONE
                mission = self._mission_kind
                flight_state = str(
                    getattr(mission_session, "session_state", self._flight_state)
                )
            else:
                control_mode = ControlMode.MANUAL if airborne else ControlMode.NONE
                mission = MissionKind.MANUAL if connected else MissionKind.IDLE
                flight_state = self._flight_state

            telemetry_battery = (
                getattr(mission_session, "last_battery", None)
                if mission_active
                else self._battery
            )
            telemetry_height = (
                getattr(mission_session, "last_height", None)
                if mission_active
                else self._height
            )
            telemetry_control_hz = (
                getattr(mission_session, "control_hz", 0.0)
                if mission_active
                else self._control_hz
            )

            return RuntimeSnapshot(
                sequence=self.events.latest_sequence,
                phase=phase,
                mission=mission,
                control_mode=control_mode,
                connected=connected,
                airborne=airborne,
                streaming=self._video_ready,
                flight_state=flight_state,
                allowed_actions=tuple(allowed),
                telemetry={
                    "battery": telemetry_battery,
                    "heightCm": telemetry_height,
                    "frontTofCm": self._front_tof,
                    "frontTofState": self._front_tof_state,
                    "controlHz": round(float(telemetry_control_hz), 1),
                },
                error=error or self._mission_error,
            )

    def connect(self) -> dict[str, Any]:
        """Connect to a real aircraft, then and only then request its stream."""
        with self._lock:
            if self.connected:
                return self.status(probe_video=True)

            adapter = self._adapter_factory()
            self._adapter = adapter
            self._video_ready = False
            self._control_hz = 0.0
            self._last_frame_at = None
            self._airborne = False
            self._flight_state = "正在连接"
            self._phase = "连接"
            try:
                adapter.connect()
                self._battery = getattr(adapter, "last_connection_battery", None)
                if self._battery is None:
                    self._battery = adapter.get_battery()
                adapter.stream_on()
                self._probe_video(timeout_seconds=5.0)
                self._refresh_front_tof(force=True)
                self._flight_state = "地面待机"
                self._phase = "检查"
                return self.status(probe_video=False)
            except Exception:
                try:
                    adapter.stop()
                finally:
                    self._adapter = None
                    self._video_ready = False
                    self._flight_state = "连接失败"
                raise

    def status(self, probe_video: bool = True) -> dict[str, Any]:
        """Return fresh real-device telemetry without manufacturing fallback data."""
        with self._lock:
            if not self.connected or self._adapter is None:
                raise RuntimeError("真机未连接。")
            if self._mission_session is not None:
                return self._mission_status_locked()

            adapter = self._adapter
            if probe_video and not self._video_ready:
                self._probe_video(timeout_seconds=0.2)

            try:
                self._battery = adapter.get_cached_battery()
            except RuntimeError:
                pass

            try:
                self._height = adapter.get_height()
            except RuntimeError:
                self._height = None

            front_tof_ready = self._front_tof_state in ("clear", "out_of_range")
            can_takeoff = bool(
                not self._airborne
                and self._video_ready
                and self._battery is not None
                and self._battery >= MIN_TAKEOFF_BATTERY
                and self._height is not None
                and front_tof_ready
            )

            return {
                "battery": self._battery,
                "heightCm": self._height,
                "frontTofCm": self._front_tof,
                "frontTofState": self._front_tof_state,
                "controlHz": round(self._control_hz, 1),
                "flightState": self._flight_state,
                "targetConfirmed": False,
                "phase": self._phase,
                "videoReady": self._video_ready,
                "airborne": self._airborne,
                "canTakeoff": can_takeoff,
                "rcEnabled": self._airborne,
                "preflight": {
                    "sdk": True,
                    "video": self._video_ready,
                    "battery": self._battery is not None and self._battery >= MIN_TAKEOFF_BATTERY,
                    "bottomTof": self._height is not None,
                    "frontTof": front_tof_ready,
                },
            }

    def takeoff(self) -> dict[str, Any]:
        """Take off only after all locally verifiable preflight gates pass."""
        with self._lock:
            self._require_no_active_mission()
            self._rc_leases.revoke()
            if not self.connected or self._adapter is None:
                raise RuntimeError("真机未连接，不能起飞。")
            if self._airborne:
                raise RuntimeError("真机已经处于空中状态。")
            current = self.status(probe_video=True)
            if not current["canTakeoff"]:
                raise RuntimeError("起飞检查未通过：请确认视频、电量和 ToF 传感器均正常。")
            self._flight_state = "正在起飞"
            self._phase = "起飞"
            try:
                authorize_takeoff = getattr(
                    self._adapter, "authorize_next_takeoff", None
                )
                if callable(authorize_takeoff):
                    authorize_takeoff()
                self._adapter.takeoff()
            except Exception:
                self._flight_state = "起飞失败"
                self._phase = "检查"
                raise
            self._airborne = True
            self._flight_state = "手动悬停"
            self._phase = "手动飞行"
            self._send_hover()
            return self.status(probe_video=False)

    def land(self) -> dict[str, Any]:
        """Clear movement and land while retaining telemetry and video."""
        with self._lock:
            self._require_no_active_mission()
            self._rc_leases.revoke()
            if not self.connected or self._adapter is None:
                raise RuntimeError("真机未连接，无法降落。")
            if not self._airborne:
                raise RuntimeError("真机当前不在空中。")
            self._flight_state = "正在降落"
            self._phase = "降落"
            self._send_hover()
            self._adapter.land()
            self._airborne = False
            self._flight_state = "地面待机"
            self._phase = "检查"
            return self.status(probe_video=False)

    def hover(self) -> dict[str, Any]:
        """Immediately clear all four RC channels."""
        with self._lock:
            self._require_no_active_mission()
            self._require_airborne()
            self._send_hover()
            self._flight_state = "手动悬停"
            return self.status(probe_video=False)

    def move_rc(self, command: dict[str, Any]) -> dict[str, Any]:
        """Apply one bounded manual RC command; a backend watchdog clears it."""
        with self._lock:
            self._require_no_active_mission()
            self._require_airborne()
            channels = {
                key: self._bounded_channel(command.get(key, 0))
                for key in ("leftRight", "forwardBack", "upDown", "yaw")
            }
            forward = channels["forwardBack"]
            vertical = channels["upDown"]
            if forward > 0:
                age = monotonic() - self._front_tof_checked_at
                if age > 1.2 or self._front_tof_state == "unavailable":
                    raise RuntimeError("前向 ToF 数据无效或过期，已禁止前进。")
                if self._front_tof is not None and self._front_tof <= FRONT_STOP_DISTANCE_CM:
                    raise RuntimeError("前方 60 cm 内存在障碍，已禁止前进。")
            if vertical < 0 and self._height is not None and self._height <= MIN_DESCENT_HEIGHT_CM:
                raise RuntimeError("当前高度过低，已禁止继续下降。")
            if vertical > 0 and self._height is not None and self._height >= MAX_ASCENT_HEIGHT_CM:
                raise RuntimeError("当前高度达到手动上升上限。")

            self._adapter.move_rc(
                channels["leftRight"],
                channels["forwardBack"],
                channels["upDown"],
                channels["yaw"],
            )
            self._last_rc_at = monotonic()
            self._rc_active = any(channels.values())
            self._flight_state = "手动飞行" if self._rc_active else "手动悬停"
            return {"ok": True, "flightState": self._flight_state}

    def start_mission(self, command: StartMissionCommand) -> dict[str, Any]:
        """Start one shared FollowSession without blocking the HTTP worker."""

        if command.mission not in {
            MissionKind.FOLLOW,
            MissionKind.REID_FOLLOW,
            MissionKind.FIXED_DEMO,
        }:
            raise RuntimeError(f"桌面端暂不支持任务：{command.mission.value}")
        with self._lock:
            if not self.connected or self._adapter is None:
                raise RuntimeError("真机未连接，无法启动自动任务。")
            self._require_no_active_mission()
            if self._airborne:
                raise RuntimeError("自动任务必须从地面启动。")
            current = self.status(probe_video=True)
            if not current["canTakeoff"]:
                raise RuntimeError("任务起飞检查未通过：请确认视频、电量和 ToF 均正常。")

            self._rc_leases.revoke()
            self._operator_commands.clear()
            session = self._mission_session_factory(
                command, self._adapter, self._operator_commands
            )
            mode_command = {
                ControlMode.MANUAL: OperatorCommand.SELECT_MANUAL,
                ControlMode.NORMAL: OperatorCommand.SELECT_NORMAL,
                ControlMode.SIDE: OperatorCommand.SELECT_SIDE,
                ControlMode.FRONT: OperatorCommand.SELECT_FRONT,
            }.get(command.initial_control_mode)
            if mode_command is None:
                raise RuntimeError("自动任务初始控制模式无效。")
            self._operator_commands.submit(mode_command)
            authorize_takeoff = getattr(self._adapter, "authorize_next_takeoff", None)
            if callable(authorize_takeoff):
                authorize_takeoff()
            self._mission_kind = command.mission
            self._mission_session = session
            self._mission_error = None
            self._flight_state = "任务准备"
            self._phase = "起飞"
            thread = Thread(
                target=self._run_mission,
                args=(session, command.mission),
                name="phantomfilmer-mission",
                daemon=True,
            )
            self._mission_thread = thread
            thread.start()
            return {
                "ok": True,
                "mission": command.mission.value,
                "initialControlMode": command.initial_control_mode.value,
            }

    def stop_mission(self) -> None:
        """Request normal mission cleanup and wait for its landing boundary."""

        session, thread = self._active_mission_handles()
        session.request_stop()
        thread.join(timeout=10.0)
        if thread.is_alive():
            raise RuntimeError("任务停止超时；无人机状态不确定，请目视确认并准备急停。")

    def emergency_stop_mission(self) -> None:
        """Collapse output immediately, then wait for mission landing cleanup."""

        session, thread = self._active_mission_handles()
        session.request_emergency_stop()
        thread.join(timeout=10.0)
        if thread.is_alive():
            raise RuntimeError("任务急停清理超时；请目视确认无人机状态。")

    def select_control_mode(
        self, command: SelectControlModeCommand
    ) -> dict[str, Any]:
        """Queue an idempotent semantic mode choice for the active mission."""

        operator_command = {
            ControlMode.MANUAL: OperatorCommand.SELECT_MANUAL,
            ControlMode.NORMAL: OperatorCommand.SELECT_NORMAL,
            ControlMode.SIDE: OperatorCommand.SELECT_SIDE,
            ControlMode.FRONT: OperatorCommand.SELECT_FRONT,
        }.get(command.mode)
        if operator_command is None:
            raise RuntimeError("控制模式不能为 none。")
        with self._lock:
            self._require_active_mission()
            envelope = self._operator_commands.submit(operator_command)
            return {
                "ok": True,
                "operatorSequence": envelope.sequence,
                "mode": command.mode.value,
            }

    def toggle_mission_pause(self) -> dict[str, Any]:
        with self._lock:
            self._require_active_mission()
            envelope = self._operator_commands.submit(OperatorCommand.TOGGLE_PAUSE)
            return {"ok": True, "operatorSequence": envelope.sequence}

    def stop(self) -> None:
        """Land first when necessary, then stop the stream and connection."""
        self._stop_active_mission_if_present(emergency=False)
        with self._lock:
            self._rc_leases.revoke()
            adapter = self._adapter
            if adapter is not None:
                try:
                    if self._airborne:
                        self._send_hover()
                        adapter.land()
                finally:
                    try:
                        adapter.stop()
                    finally:
                        self._clear_session()
            else:
                self._clear_session()

    def emergency_land(self) -> None:
        """Request landing, then close the connection regardless of outcome."""
        with self._lock:
            had_active_mission = self._mission_session is not None
        self._stop_active_mission_if_present(emergency=True)
        with self._lock:
            self._rc_leases.revoke()
            adapter = self._adapter
            if adapter is None or not getattr(adapter, "connected", False):
                raise RuntimeError("真机未连接，无法发送降落指令。")
            try:
                if not had_active_mission:
                    adapter.move_rc(0, 0, 0, 0)
                    adapter.land()
            finally:
                self._adapter = None
                self._clear_session()
                adapter.stop()

    def acquire_rc_lease(self) -> dict[str, object]:
        """Grant exclusive, short-lived manual control authority while airborne."""
        with self._lock:
            if self._mission_session is not None:
                self._require_mission_manual_control()
                self._operator_commands.submit(OperatorCommand.HOVER)
            else:
                self._require_airborne()
                self._send_hover()
            lease = self._rc_leases.acquire()
            self.events.publish(
                "rc.lease.acquired",
                lease.to_dict(),
                snapshot=self.runtime_snapshot(),
            )
            return lease.to_dict()

    def move_rc_with_lease(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Validate lease freshness before forwarding a versioned RC command."""
        with self._lock:
            mission_manual = self._mission_session is not None
            if mission_manual:
                self._require_mission_manual_control()
            else:
                self._require_airborne()
            lease = self._rc_leases.validate_and_refresh(
                lease_id=payload.get("leaseId"),
                sequence=payload.get("sequence"),
                issued_at_ms=payload.get("issuedAt"),
            )
            try:
                if mission_manual:
                    operator_command = self._operator_command_from_rc(payload)
                    envelope = self._operator_commands.submit(operator_command)
                    result = {
                        "ok": True,
                        "flightState": "自动任务手动接管",
                        "operatorSequence": envelope.sequence,
                    }
                    self.events.publish(
                        "mission.manual_rc.accepted",
                        {
                            "operatorSequence": envelope.sequence,
                            "command": operator_command.value,
                        },
                        snapshot=self.runtime_snapshot(),
                    )
                else:
                    result = self.execute(
                        MoveRcCommand(
                            left_right=payload.get("leftRight", 0),
                            forward_back=payload.get("forwardBack", 0),
                            up_down=payload.get("upDown", 0),
                            yaw=payload.get("yaw", 0),
                        )
                    )
            except Exception:
                # A sequence is consumed during validation. Revoke authority so
                # the client cannot continue from a state it did not observe.
                if mission_manual:
                    self._operator_commands.submit(OperatorCommand.HOVER)
                else:
                    self._send_hover()
                raise
            return {**result, **lease.to_dict()}

    def release_rc_lease(self, lease_id: object) -> dict[str, object]:
        """Release manual authority and always collapse output to hover."""
        with self._lock:
            released = self._rc_leases.release(lease_id)
            if self._mission_session is not None:
                self._operator_commands.submit(OperatorCommand.HOVER)
            elif self._airborne:
                self._send_hover()
                self._flight_state = "手动悬停"
            self.events.publish(
                "rc.lease.released",
                {"leaseId": lease_id, "released": released},
                snapshot=self.runtime_snapshot(),
            )
            return {"ok": True, "released": released}

    def shutdown(self) -> None:
        """Release the aircraft and stop the local RC watchdog."""
        self._watchdog_stop.set()
        try:
            self.stop()
        finally:
            self._watchdog.join(timeout=1.0)
            self._tof_monitor.join(timeout=3.0)

    def _refresh_front_tof(self, force: bool = False) -> None:
        adapter = self._adapter
        if adapter is None:
            self._front_tof = None
            self._front_tof_state = "unavailable"
            return
        now = monotonic()
        if not force and now - self._front_tof_checked_at < 2.0:
            return
        self._front_tof_checked_at = now
        try:
            distance = adapter.get_front_distance_cm()
            self._front_tof = distance
            self._front_tof_state = "out_of_range" if distance is None else (
                "blocked" if distance <= FRONT_STOP_DISTANCE_CM else "clear"
            )
        except RuntimeError:
            self._front_tof = None
            self._front_tof_state = "unavailable"

    def _require_airborne(self) -> None:
        if not self.connected or self._adapter is None:
            raise RuntimeError("真机未连接。")
        if not self._airborne:
            raise RuntimeError("真机尚未起飞，手动控制不可用。")

    def _require_mission_manual_control(self) -> None:
        session = self._mission_session
        if session is None or not bool(getattr(session, "airborne", False)):
            raise RuntimeError("自动任务尚未进入空中手动接管状态。")
        manual_controller = getattr(session, "manual_controller", None)
        if not bool(getattr(manual_controller, "active", False)):
            raise RuntimeError("请先把自动任务切换到手动接管模式。")

    def _operator_command_from_rc(self, payload: dict[str, Any]) -> OperatorCommand:
        channels = {
            key: self._bounded_channel(payload.get(key, 0))
            for key in ("leftRight", "forwardBack", "upDown", "yaw")
        }
        nonzero = [(key, value) for key, value in channels.items() if value]
        if not nonzero:
            return OperatorCommand.HOVER
        if len(nonzero) != 1:
            raise RuntimeError("自动任务手动接管每次只接受一个运动轴。")
        key, value = nonzero[0]
        return {
            ("leftRight", 1): OperatorCommand.MOVE_RIGHT,
            ("leftRight", -1): OperatorCommand.MOVE_LEFT,
            ("forwardBack", 1): OperatorCommand.MOVE_FORWARD,
            ("forwardBack", -1): OperatorCommand.MOVE_BACKWARD,
            ("upDown", 1): OperatorCommand.MOVE_UP,
            ("upDown", -1): OperatorCommand.MOVE_DOWN,
            ("yaw", 1): OperatorCommand.YAW_RIGHT,
            ("yaw", -1): OperatorCommand.YAW_LEFT,
        }[(key, 1 if value > 0 else -1)]

    @staticmethod
    def _bounded_channel(value: Any) -> int:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError("手动控制量格式无效。")
        return max(-MAX_RC_SPEED, min(MAX_RC_SPEED, int(value)))

    def _send_hover(self) -> None:
        adapter = self._adapter
        if adapter is None:
            return
        adapter.move_rc(0, 0, 0, 0)
        self._last_rc_at = monotonic()
        self._rc_active = False
        self._rc_leases.revoke()

    def _watch_rc(self) -> None:
        while not self._watchdog_stop.wait(0.1):
            with self._lock:
                if (
                    self._airborne
                    and self._rc_active
                    and monotonic() - self._last_rc_at > RC_WATCHDOG_SECONDS
                ):
                    try:
                        self._send_hover()
                        self._flight_state = "手动悬停"
                    except RuntimeError:
                        self._rc_active = False

    def _monitor_front_tof(self) -> None:
        """Poll the blocking expansion command without blocking manual RC handling."""
        while not self._watchdog_stop.wait(0.2):
            with self._lock:
                adapter = (
                    self._adapter
                    if self.connected and self._mission_session is None
                    else None
                )
            if adapter is None:
                continue
            try:
                distance = adapter.get_front_distance_cm()
                state = "out_of_range" if distance is None else (
                    "blocked" if distance <= FRONT_STOP_DISTANCE_CM else "clear"
                )
            except RuntimeError:
                distance = None
                state = "unavailable"
            with self._lock:
                if self._adapter is adapter:
                    self._front_tof = distance
                    self._front_tof_state = state
                    self._front_tof_checked_at = monotonic()

    def _run_mission(self, session: Any, mission: MissionKind) -> None:
        """Own the background session result and publish its terminal state."""

        result = None
        failure: Optional[str] = None
        try:
            result = session.run()
        except Exception as exc:
            failure = str(exc)
            logging.exception("automatic mission failed")
        with self._lock:
            if self._mission_session is not session:
                return
            self._airborne = bool(getattr(result, "airborne", False))
            self._mission_error = failure
            self._mission_session = None
            self._mission_thread = None
            self._mission_kind = MissionKind.IDLE
            self._operator_commands.clear()
            if failure:
                self._flight_state = "任务失败"
                self._phase = "检查"
            elif self._airborne:
                self._flight_state = "任务结束但仍报告空中"
                self._phase = "降落"
            else:
                self._flight_state = "地面待机"
                self._phase = "检查"
        self.events.publish(
            "mission.failed" if failure else "mission.finished",
            {
                "mission": mission.value,
                "state": str(getattr(result, "state", "ERROR" if failure else "STOPPED")),
                "error": failure,
            },
            snapshot=self.runtime_snapshot(),
        )

    def _build_mission_session(
        self,
        command: StartMissionCommand,
        adapter: Any,
        operator_commands: OperatorCommandChannel,
    ) -> Any:
        """Build the same detector/controller/kernel stack used by the CLI."""

        if not command.profile_name:
            raise RuntimeError("自动任务必须选择一个本地人物档案。")
        config = load_runtime_config(command.obstacle_enabled)
        config = build_reid_runtime_config(
            config,
            profile_name=command.profile_name,
        )
        config = dict(config)
        config["display_console_camera"] = False
        vision = config.get("vision", {})
        runtime_vision = dict(vision) if isinstance(vision, dict) else {}
        runtime_vision["reid_profile_root"] = str(self.data_dir / "reid_profiles")
        config["vision"] = runtime_vision
        safety = SafetyManager.from_dict(config)
        detector = create_detector(config)
        controller = FollowController.from_config(
            safety_manager=safety,
            config=config,
        )
        _, _, motion_arbiter = build_obstacle_modules(config, safety)
        maneuver = (
            FixedDemoManeuver(control_interval=read_control_interval(config))
            if command.mission is MissionKind.FIXED_DEMO
            else None
        )
        return MissionFactory(
            drone=adapter,
            safety_manager=safety,
            detector=detector,
            follow_controller=controller,
            config=config,
            motion_arbiter=motion_arbiter,
            operator_commands=operator_commands,
            manage_camera_stream=False,
        ).create_follow_session(
            mission=command.mission,
            mode_label=f"DESKTOP {command.mission.value.upper()}",
            window_name="PhantomFilmer Desktop Mission",
            state_label="REID" if command.mission is MissionKind.REID_FOLLOW else "FOLLOW",
            allow_pause=True,
            pre_follow_maneuver=maneuver,
            initial_target_lock_frames=0,
            enable_target_search=True,
        )

    def _active_mission_handles(self) -> tuple[Any, Thread]:
        with self._lock:
            self._require_active_mission()
            session = self._mission_session
            thread = self._mission_thread
            if session is None or thread is None:
                raise RuntimeError("自动任务状态不完整，无法安全控制。")
            return session, thread

    def _stop_active_mission_if_present(self, *, emergency: bool) -> None:
        with self._lock:
            active = self._mission_session is not None
        if not active:
            return
        if emergency:
            self.emergency_stop_mission()
        else:
            self.stop_mission()

    def _require_active_mission(self) -> None:
        if self._mission_session is None:
            raise RuntimeError("当前没有正在运行的自动任务。")

    def _require_no_active_mission(self) -> None:
        if self._mission_session is not None:
            raise RuntimeError("自动任务运行中，不能执行独立手动飞行命令。")

    def _mission_status_locked(self) -> dict[str, Any]:
        session = self._mission_session
        if session is None:
            raise RuntimeError("自动任务状态已经结束，请重试。")
        battery = getattr(session, "last_battery", None)
        height = getattr(session, "last_height", None)
        if battery is None:
            battery = self._battery
        if height is None:
            height = self._height
        airborne = bool(getattr(session, "airborne", False))
        manual_controller = getattr(session, "manual_controller", None)
        return {
            "battery": battery,
            "heightCm": height,
            "frontTofCm": self._front_tof,
            "frontTofState": self._front_tof_state,
            "controlHz": round(float(getattr(session, "control_hz", 0.0)), 1),
            "flightState": str(getattr(session, "session_state", "任务准备")),
            "targetConfirmed": False,
            "phase": "自动任务",
            "videoReady": self._video_ready,
            "airborne": airborne,
            "canTakeoff": False,
            "rcEnabled": bool(getattr(manual_controller, "active", False)),
            "preflight": {
                "sdk": True,
                "video": self._video_ready,
                "battery": battery is not None and int(battery) >= MIN_TAKEOFF_BATTERY,
                "bottomTof": height is not None,
                "frontTof": self._front_tof_state in ("clear", "out_of_range"),
            },
        }

    def _clear_session(self) -> None:
        self._rc_leases.revoke()
        self._adapter = None
        self._video_ready = False
        self._last_frame_at = None
        self._control_hz = 0.0
        self._front_tof = None
        self._front_tof_checked_at = 0.0
        self._front_tof_state = "unavailable"
        self._height = None
        self._airborne = False
        self._flight_state = "未连接"
        self._phase = "连接"
        self._last_rc_at = 0.0
        self._rc_active = False
        self._mission_kind = MissionKind.IDLE
        self._mission_session = None
        self._mission_thread = None
        self._mission_error = None
        self._operator_commands.clear()

    def mjpeg_frames(self) -> Iterator[bytes]:
        """Yield JPEG frames for one browser client without any GUI window."""
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 opencv-python，无法编码网页视频流。") from exc

        while self.connected:
            adapter = self._adapter
            if adapter is None:
                break
            try:
                frame = adapter.get_frame()
                if not self._is_real_frame(frame):
                    sleep(0.05)
                    continue
                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
                if not ok:
                    sleep(0.05)
                    continue
                self._mark_frame()
                payload = encoded.tobytes()
                yield (
                    f"--{VIDEO_BOUNDARY}\r\n".encode()
                    + b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(payload)}\r\n\r\n".encode()
                    + payload
                    + b"\r\n"
                )
                sleep(0.03)
            except RuntimeError:
                if not self.connected:
                    break
                sleep(0.1)

    def _probe_video(self, timeout_seconds: float) -> None:
        adapter = self._adapter
        if adapter is None or not self.connected:
            return
        deadline = monotonic() + timeout_seconds
        while monotonic() <= deadline:
            try:
                frame = adapter.get_frame()
                if self._is_real_frame(frame):
                    self._video_ready = True
                    self._mark_frame()
                    return
            except RuntimeError:
                pass
            sleep(0.05)

    @staticmethod
    def _is_real_frame(frame: Any) -> bool:
        """Reject djitellopy's all-zero placeholder before the first UDP frame."""
        if frame is None or getattr(frame, "size", 0) == 0:
            return False
        any_pixel = getattr(frame, "any", None)
        return bool(any_pixel()) if callable(any_pixel) else True

    def _mark_frame(self) -> None:
        now = monotonic()
        if self._last_frame_at is not None:
            interval = now - self._last_frame_at
            if interval > 0:
                instant_hz = min(60.0, 1.0 / interval)
                self._control_hz = (
                    instant_hz if self._control_hz == 0 else self._control_hz * 0.8 + instant_hz * 0.2
                )
        self._last_frame_at = now


class DroneRequestHandler(BaseHTTPRequestHandler):
    """Minimal authenticated loopback API used by Electron main."""

    server_version = "PhantomFilmerSidecar/1.0"

    @property
    def application(self) -> "SidecarServer":
        return self.server  # type: ignore[return-value]

    @property
    def service(self) -> DroneWebService:
        return self.application.service

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlsplit(self.path)
        path = parsed.path
        if path == "/api/drone/video/stream":
            if not self._authorized_video():
                self._json(HTTPStatus.UNAUTHORIZED, {"error": "视频会话已失效。"})
                return
            self._video_stream()
            return
        if not self._authorized_request():
            self._json(HTTPStatus.UNAUTHORIZED, {"error": "会话令牌无效。"})
            return
        if path == "/api/v1/health":
            self._json(
                HTTPStatus.OK,
                {"apiVersion": API_VERSION, "ok": True, "connected": self.service.connected},
            )
            return
        if path == "/api/v1/capabilities":
            self._json(HTTPStatus.OK, self.service.capabilities())
            return
        if path == "/api/v1/runtime/snapshot":
            self._json(
                HTTPStatus.OK,
                {"apiVersion": API_VERSION, "snapshot": self.service.runtime_snapshot().to_dict()},
            )
            return
        if path == "/api/v1/profiles":
            self._json(
                HTTPStatus.OK,
                {"apiVersion": API_VERSION, "profiles": self.service.list_profiles()},
            )
            return
        if path == "/api/v1/runtime/events":
            query = parse_qs(parsed.query)
            try:
                since = int(query.get("since", ["0"])[0])
                if since < 0:
                    raise ValueError
            except ValueError:
                self._v1_error(HTTPStatus.BAD_REQUEST, "INVALID_SEQUENCE", "since 必须是非负整数。")
                return
            events = self.service.events.events_since(since)
            oldest = self.service.events.oldest_sequence
            self._json(
                HTTPStatus.OK,
                {
                    "apiVersion": API_VERSION,
                    "latestSequence": self.service.events.latest_sequence,
                    "resetRequired": since < oldest - 1,
                    "events": [event.to_dict() for event in events],
                },
            )
            return
        if path == "/api/health":
            self._json(HTTPStatus.OK, {"ok": True, "connected": self.service.connected})
            return
        if path == "/api/drone/status":
            self._run_command(RefreshStatusCommand())
            return
        self._json(HTTPStatus.NOT_FOUND, {"error": "接口不存在。"})

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized_request():
            self._json(HTTPStatus.UNAUTHORIZED, {"error": "会话令牌无效。"})
            return
        path = urlsplit(self.path).path
        if path == "/api/v1/commands":
            try:
                command = command_from_payload(self._read_json())
            except (RuntimeError, ValueError) as exc:
                self._v1_error(HTTPStatus.BAD_REQUEST, "INVALID_COMMAND", str(exc))
                return
            self._run_v1_command(command)
            return
        if path == "/api/v1/profiles/enroll":
            try:
                payload = self._read_json()
                profile = self.service.enroll_profile(
                    name=payload.get("name"),
                    image_paths=payload.get("imagePaths"),
                    overwrite=payload.get("overwrite", False),
                )
                self._json(
                    HTTPStatus.CREATED,
                    {"apiVersion": API_VERSION, "profile": profile},
                )
            except Exception as exc:
                self._v1_error(HTTPStatus.CONFLICT, "PROFILE_ENROLL_FAILED", str(exc))
            return
        if path == "/api/v1/rc/lease":
            try:
                lease = self.service.acquire_rc_lease()
                self._json(HTTPStatus.CREATED, {"apiVersion": API_VERSION, **lease})
            except Exception as exc:
                self._v1_error(HTTPStatus.CONFLICT, "RC_LEASE_REJECTED", str(exc))
            return
        if path == "/api/v1/rc":
            try:
                result = self.service.move_rc_with_lease(self._read_json())
                self._json(HTTPStatus.OK, {"apiVersion": API_VERSION, **result})
            except Exception as exc:
                self._v1_error(HTTPStatus.CONFLICT, "RC_COMMAND_REJECTED", str(exc))
            return
        if path == "/api/v1/rc/release":
            try:
                result = self.service.release_rc_lease(self._read_json().get("leaseId"))
                self._json(HTTPStatus.OK, {"apiVersion": API_VERSION, **result})
            except Exception as exc:
                self._v1_error(HTTPStatus.CONFLICT, "RC_RELEASE_REJECTED", str(exc))
            return
        if path == "/api/drone/connect":
            self._run_command(ConnectCommand())
            return
        if path == "/api/drone/takeoff":
            self._run_command(TakeoffCommand())
            return
        if path == "/api/drone/land":
            self._run_command(LandCommand())
            return
        if path == "/api/drone/hover":
            self._run_command(HoverCommand())
            return
        if path == "/api/drone/rc":
            try:
                payload = self._read_json()
                command = MoveRcCommand(
                    left_right=payload.get("leftRight", 0),
                    forward_back=payload.get("forwardBack", 0),
                    up_down=payload.get("upDown", 0),
                    yaw=payload.get("yaw", 0),
                )
                self._json(HTTPStatus.OK, self.service.execute(command))
            except Exception as exc:
                self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})
            return
        if path == "/api/drone/stop":
            self._run_command(StopCommand())
            return
        if path == "/api/drone/emergency-land":
            self._run_command(EmergencyLandCommand())
            return
        if path == "/api/drone/video-token":
            self._json(HTTPStatus.OK, self.application.issue_video_token())
            return
        if path == "/api/sidecar/shutdown":
            self._safe_shutdown()
            return
        self._json(HTTPStatus.NOT_FOUND, {"error": "接口不存在。"})

    def log_message(self, format_string: str, *args: Any) -> None:
        logging.info("%s - %s", self.address_string(), format_string % args)

    def _authorized_request(self) -> bool:
        supplied = self.headers.get("X-Phantom-Token", "")
        return hmac.compare_digest(supplied, self.application.session_token)

    def _authorized_video(self) -> bool:
        query = parse_qs(urlsplit(self.path).query)
        supplied = query.get("token", [""])[0]
        return self.application.consume_video_token(supplied)

    def _run_command(self, command: Any) -> None:
        try:
            self._json(HTTPStatus.OK, self.service.execute(command))
        except Exception as exc:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})

    def _run_v1_command(self, command: Any) -> None:
        try:
            result = self.service.execute(command)
            self._json(
                HTTPStatus.OK,
                {
                    "apiVersion": API_VERSION,
                    "commandId": command.command_id,
                    "result": result,
                    "snapshot": self.service.runtime_snapshot().to_dict(),
                },
            )
        except Exception as exc:
            self._v1_error(HTTPStatus.CONFLICT, "COMMAND_REJECTED", str(exc))

    def _v1_error(self, status: HTTPStatus, code: str, message: str) -> None:
        self._json(
            status,
            {"apiVersion": API_VERSION, "error": {"code": code, "message": message}},
        )

    def _video_stream(self) -> None:
        if not self.service.connected:
            self._json(HTTPStatus.CONFLICT, {"error": "真机未连接，视频流未开启。"})
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={VIDEO_BOUNDARY}")
        self.end_headers()
        try:
            for frame in self.service.mjpeg_frames():
                self.wfile.write(frame)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _safe_shutdown(self) -> None:
        try:
            self.service.shutdown()
            self._json(HTTPStatus.OK, {"ok": True})
        except Exception as exc:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})
        finally:
            Thread(target=self.application.shutdown, daemon=True).start()

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 4096:
            raise RuntimeError("请求内容为空或过大。")
        try:
            body = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("请求 JSON 无效。") from exc
        if not isinstance(body, dict):
            raise RuntimeError("请求必须是 JSON 对象。")
        return body

    def _json(self, status: HTTPStatus, body: dict[str, Any]) -> None:
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)


class SidecarServer(ThreadingHTTPServer):
    """HTTP server carrying all per-process state without module globals."""

    daemon_threads = True
    allow_reuse_address = False

    def __init__(
        self,
        address: tuple[str, int],
        *,
        session_token: str,
        service: DroneWebService,
    ) -> None:
        super().__init__(address, DroneRequestHandler)
        self.session_token = session_token
        self.service = service
        self._video_tokens: dict[str, float] = {}
        self._token_lock = RLock()

    def issue_video_token(self, lifetime_seconds: float = 30.0) -> dict[str, Any]:
        token = secrets.token_urlsafe(32)
        expires_at = time() + lifetime_seconds
        with self._token_lock:
            now = time()
            self._video_tokens = {
                value: expiry for value, expiry in self._video_tokens.items() if expiry > now
            }
            self._video_tokens[token] = expires_at
        return {"token": token, "expiresAt": int(expires_at * 1000)}

    def consume_video_token(self, token: str) -> bool:
        if not token:
            return False
        with self._token_lock:
            expires_at = self._video_tokens.pop(token, 0.0)
        return expires_at > time()


def create_server(
    *,
    host: str = HOST,
    port: int = PORT,
    session_token: str,
    data_dir: str | Path,
    service: Optional[DroneWebService] = None,
) -> SidecarServer:
    """Create one isolated sidecar server without starting background HTTP work."""
    if host != HOST:
        raise ValueError("sidecar 只能监听 127.0.0.1。")
    if not session_token:
        raise ValueError("必须提供非空会话令牌。")
    root = Path(data_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    return SidecarServer(
        (host, port),
        session_token=session_token,
        service=service or DroneWebService(data_dir=root),
    )


def _configure_logging(data_dir: Path) -> Path:
    log_dir = data_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "sidecar.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[logging.FileHandler(log_path, encoding="utf-8")],
        force=True,
    )
    return log_path


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PhantomFilmer desktop sidecar")
    parser.add_argument("--host", default=HOST, choices=[HOST])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--token", required=True)
    parser.add_argument("--data-dir", required=True)
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    """Run the local bridge until interrupted, always releasing the aircraft."""
    args = parse_args(argv)
    data_dir = Path(args.data_dir).expanduser().resolve()
    log_path = _configure_logging(data_dir)
    server = create_server(
        host=args.host,
        port=args.port,
        session_token=args.token,
        data_dir=data_dir,
    )

    def shutdown(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    ready = {
        "event": "ready",
        "host": HOST,
        "port": server.server_address[1],
        "pid": __import__("os").getpid(),
        "logPath": str(log_path),
    }
    print(json.dumps(ready, ensure_ascii=False), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        server.service.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
