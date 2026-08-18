"""Shared real-aircraft application state for the WebUI adapter layer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from console.tools import ConsoleTools
from web.services import ConnectionService, ConnectionState, VideoHub, VideoOwner


@dataclass
class AppState:
    """Coordinate verified connection, task gate, telemetry and video ownership."""

    tools: ConsoleTools
    connection: ConnectionService
    video: VideoHub

    @classmethod
    def create(cls, tools: ConsoleTools, web_config: dict[str, Any]) -> "AppState":
        connection = ConnectionService(
            tools,
            health_interval_seconds=float(web_config.get("health_interval_seconds", 2.0)),
            failure_limit=int(web_config.get("health_failure_limit", 3)),
            freshness_seconds=float(web_config.get("status_freshness_seconds", 5.0)),
        )
        video = VideoHub(
            tools,
            jpeg_quality=int(web_config.get("video_jpeg_quality", 80)),
        )
        state = cls(tools=tools, connection=connection, video=video)
        connection.set_degraded_callback(state._on_connection_degraded)
        tools.set_web_callbacks(video.publish_task_frame, state._on_task_finished)
        return state

    def require_tools(self) -> ConsoleTools:
        if self.tools is None:
            raise RuntimeError("无人机系统尚未初始化")
        return self.tools

    def connect(self) -> dict[str, Any]:
        return self.connection.connect()

    def can_start_task(self) -> tuple[bool, str]:
        try:
            self.connection.require_verified()
        except RuntimeError as exc:
            return False, str(exc)
        if self.tools.is_task_active() or self.tools.airborne:
            return False, "已有飞行任务正在运行。"
        try:
            return self.tools.can_start_task()
        except RuntimeError as exc:
            self.connection.record_command_failure(exc)
            return False, f"即时电量检查失败：{exc}"

    def start_task(self) -> bool:
        self.connection.require_verified()
        if self.tools.is_task_active() or self.tools.airborne:
            raise RuntimeError("已有飞行任务正在运行。")
        allowed, message = self.can_start_task()
        if not allowed:
            raise RuntimeError(message)
        self.video.handoff_to_task()
        try:
            started = self.tools.start_follow_task()
        except Exception:
            self._on_task_finished()
            raise
        if not started:
            self._on_task_finished()
        return started

    def telemetry_snapshot(self) -> dict[str, Any]:
        snapshot = self.connection.snapshot()
        snapshot.update(
            {
                "mode": self.tools.current_mode,
                "connected": snapshot["connection_verified"],
                "airborne": bool(self.tools.airborne),
                "streaming": self.video.active,
                "video_owner": self.video.owner.value,
                "video_error": self.video.last_error,
                "task_active": bool(self.tools.is_task_active()),
            }
        )
        return snapshot

    def prepare_video(self) -> None:
        self.connection.require_verified()
        if self.video.owner != VideoOwner.FOLLOW_SESSION:
            self.video.start_preview()

    def close(self) -> None:
        self.connection.close()
        try:
            self.tools.close()
        finally:
            self.video.stop()

    def _on_connection_degraded(self) -> None:
        # Do not inject a new flight command into an active session. Its own
        # fail-safe remains responsible for landing; only new work is blocked.
        if not self.tools.is_task_active():
            self.video.stop_preview()

    def _on_task_finished(self) -> None:
        self.video.task_finished()
        if self.connection.state == ConnectionState.VERIFIED and self.connection.is_fresh():
            try:
                self.video.start_preview()
            except RuntimeError:
                pass
