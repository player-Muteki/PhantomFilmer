"""Web adapter tests use an isolated test double and never access aircraft hardware."""

from time import sleep

import numpy as np
from fastapi.testclient import TestClient

from web.server import create_app
from web.services import ConnectionService, ConnectionState, VideoHub


class StubDrone:
    def __init__(self) -> None:
        self.stream_on_count = 0
        self.stream_off_count = 0

    def get_yaw(self):
        return 12

    def get_front_distance_cm(self):
        raise RuntimeError("optional ToF not installed")

    def stream_on(self):
        self.stream_on_count += 1

    def stream_off(self):
        self.stream_off_count += 1

    def get_frame(self):
        sleep(0.01)
        return np.zeros((24, 32, 3), dtype=np.uint8)


class StubTools:
    def __init__(self, battery: int = 80) -> None:
        self._drone = StubDrone()
        self.battery = battery
        self.connected = False
        self.airborne = False
        self.streaming = False
        self.current_mode = "未连接"
        self.frame_width = 32
        self.frame_height = 24
        self.status_reads = 0
        self.task_active = False
        self.frame_sink = None
        self.finished_callback = None

    def set_web_callbacks(self, frame_sink, finished_callback):
        self.frame_sink = frame_sink
        self.finished_callback = finished_callback

    def connect(self):
        self.connected = True
        self.current_mode = "待机"

    def get_status(self):
        if not self.connected:
            raise RuntimeError("not connected")
        self.status_reads += 1
        return {"battery": self.battery, "height": 18, "mode": self.current_mode}

    def can_start_task(self):
        if self.battery >= 30:
            return True, f"当前电量 {self.battery}%，允许开始任务。"
        return False, "电量不足，禁止起飞。"

    def start_follow_task(self):
        self.task_active = True
        self.current_mode = "跟随任务"
        return True

    def stop_task(self):
        self.task_active = False
        self.current_mode = "待机"

    def emergency_stop(self):
        self.task_active = False
        self.current_mode = "急停"

    def is_task_active(self):
        return self.task_active

    def close(self):
        self.connected = False
        self.current_mode = "已退出"


def build_test_app(tools: StubTools):
    app = create_app(tools=tools)
    app.state.app_state.connection.health_interval_seconds = 60.0
    return app


def test_status_and_video_require_verified_real_connection():
    with TestClient(build_test_app(StubTools())) as client:
        assert client.get("/api/status").status_code == 503
        assert client.get("/video/stream").status_code == 503


def test_connect_runs_second_status_probe_and_marks_verified():
    tools = StubTools()
    with TestClient(build_test_app(tools)) as client:
        response = client.post("/api/connect")
        assert response.status_code == 200
        assert response.json()["connection_state"] == "VERIFIED"
        assert response.json()["connection_verified"] is True
        assert tools.status_reads == 1


def test_optional_front_tof_does_not_block_connection():
    with TestClient(build_test_app(StubTools())) as client:
        client.post("/api/connect")
        with client.websocket_connect("/ws/telemetry") as socket:
            payload = socket.receive_json()
        assert payload["front_tof_supported"] is False
        assert payload["front_distance"] is None
        assert payload["connection_verified"] is True


def test_websocket_distributes_cache_without_sdk_queries():
    tools = StubTools()
    with TestClient(build_test_app(tools)) as client:
        client.post("/api/connect")
        reads_after_connect = tools.status_reads
        with client.websocket_connect("/ws/telemetry") as socket:
            first = socket.receive_json()
            second = socket.receive_json()
        assert first["battery"] == 80
        assert second["battery"] == 80
        assert tools.status_reads == reads_after_connect


def test_task_requires_confirmation_verified_connection_and_battery():
    tools = StubTools(battery=80)
    with TestClient(build_test_app(tools)) as client:
        assert client.post("/api/task/start", json={"confirmed": True}).status_code == 503
        client.post("/api/connect")
        assert client.post("/api/task/start", json={"confirmed": False}).status_code == 400
        assert client.post("/api/task/start", json={"confirmed": True}).status_code == 200


def test_low_battery_is_rejected_by_server_gate():
    tools = StubTools(battery=20)
    with TestClient(build_test_app(tools)) as client:
        client.post("/api/connect")
        response = client.post("/api/task/start", json={"confirmed": True})
        assert response.status_code == 503
        assert tools.task_active is False


def test_three_health_failures_mark_connection_degraded():
    tools = StubTools()
    service = ConnectionService(tools, health_interval_seconds=60)
    service.connect()
    for _ in range(3):
        service.record_command_failure(RuntimeError("UDP timeout"))
    assert service.state == ConnectionState.DEGRADED
    assert service.snapshot()["connection_verified"] is False
    service.close()


def test_video_hub_starts_only_one_camera_producer():
    tools = StubTools()
    hub = VideoHub(tools)
    hub.start_preview()
    hub.start_preview()
    sleep(0.05)
    hub.stop()
    assert tools._drone.stream_on_count == 1
    assert tools._drone.stream_off_count == 1


def test_emergency_stop_remains_whitelisted():
    tools = StubTools()
    with TestClient(build_test_app(tools)) as client:
        client.post("/api/connect")
        response = client.post("/api/emergency-stop")
        assert response.status_code == 200
        assert response.json()["ok"] is True
