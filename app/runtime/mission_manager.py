"""Shared command boundary for every PhantomFilmer operator interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from threading import RLock
from typing import Any

from app.runtime.commands import (
    ConnectCommand,
    EmergencyLandCommand,
    EmergencyStopCommand,
    HoverCommand,
    LandCommand,
    MoveRcCommand,
    RefreshStatusCommand,
    RuntimeCommand,
    SelectControlModeCommand,
    StartMissionCommand,
    StopCommand,
    StopMissionCommand,
    TakeoffCommand,
    command_name,
)
from app.runtime.event_bus import EventBus
from app.runtime.models import RuntimeSnapshot


class MissionManager(ABC):
    """Serialize typed commands and publish their authoritative outcomes."""

    def __init__(self, *, event_bus: EventBus | None = None) -> None:
        self.events = event_bus or EventBus()
        self._command_lock = RLock()

    def execute(self, command: RuntimeCommand) -> Any:
        """Execute one command atomically and emit accepted/completed/rejected events."""

        name = command_name(command)
        metadata = {"commandId": command.command_id, "command": name}
        with self._command_lock:
            self.events.publish("command.accepted", metadata)
            try:
                result = self._dispatch(command)
            except Exception as exc:
                snapshot = self.runtime_snapshot(error=str(exc))
                self.events.publish(
                    "command.rejected",
                    {**metadata, "error": str(exc)},
                    snapshot=snapshot,
                )
                raise
            snapshot = self.runtime_snapshot()
            self.events.publish("command.completed", metadata, snapshot=snapshot)
            return result

    def _dispatch(self, command: RuntimeCommand) -> Any:
        if isinstance(command, ConnectCommand):
            return self.connect()
        if isinstance(command, RefreshStatusCommand):
            return self.status()
        if isinstance(command, TakeoffCommand):
            return self.takeoff()
        if isinstance(command, LandCommand):
            return self.land()
        if isinstance(command, HoverCommand):
            return self.hover()
        if isinstance(command, MoveRcCommand):
            return self.move_rc(
                {
                    "leftRight": command.left_right,
                    "forwardBack": command.forward_back,
                    "upDown": command.up_down,
                    "yaw": command.yaw,
                }
            )
        if isinstance(command, StopCommand):
            self.stop()
            return {"ok": True}
        if isinstance(command, EmergencyLandCommand):
            self.emergency_land()
            return {"ok": True}
        if isinstance(command, StartMissionCommand):
            return self.start_mission(command)
        if isinstance(command, StopMissionCommand):
            self.stop_mission()
            return {"ok": True}
        if isinstance(command, EmergencyStopCommand):
            self.emergency_stop_mission()
            return {"ok": True}
        if isinstance(command, SelectControlModeCommand):
            return self.select_control_mode(command)
        raise TypeError(f"unsupported runtime command: {type(command).__name__}")

    @abstractmethod
    def runtime_snapshot(self, *, error: str | None = None) -> RuntimeSnapshot:
        """Return current state without performing blocking device probes."""

    def connect(self) -> dict[str, Any]:
        raise RuntimeError("当前运行时不支持连接设备。")

    def status(self, probe_video: bool = True) -> dict[str, Any]:
        del probe_video
        raise RuntimeError("当前运行时不支持读取设备状态。")

    def takeoff(self) -> dict[str, Any]:
        raise RuntimeError("当前运行时不支持独立起飞。")

    def land(self) -> dict[str, Any]:
        raise RuntimeError("当前运行时不支持独立降落。")

    def hover(self) -> dict[str, Any]:
        raise RuntimeError("当前运行时不支持独立悬停。")

    def move_rc(self, command: dict[str, Any]) -> dict[str, Any]:
        del command
        raise RuntimeError("当前运行时不支持手动 RC。")

    def stop(self) -> None:
        raise RuntimeError("当前运行时不支持停止设备。")

    def emergency_land(self) -> None:
        raise RuntimeError("当前运行时不支持紧急降落。")

    def start_mission(self, command: StartMissionCommand) -> Any:
        del command
        raise RuntimeError("当前运行时不支持启动任务。")

    def stop_mission(self) -> None:
        raise RuntimeError("当前运行时没有可停止的任务。")

    def emergency_stop_mission(self) -> None:
        raise RuntimeError("当前运行时没有可急停的任务。")

    def select_control_mode(self, command: SelectControlModeCommand) -> Any:
        del command
        raise RuntimeError("当前运行时不支持切换控制模式。")
