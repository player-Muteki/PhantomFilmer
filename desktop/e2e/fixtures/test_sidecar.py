import argparse
import base64
import json
import os
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit


PIXEL = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAQAAAADCAIAAAA7ljmRAAAAEElEQVR4nGNomLAAjhhwcgCJlhRBlbH/hgAAAABJRU5ErkJggg=="
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--token", required=True)
    parser.add_argument("--data-dir", required=True)
    return parser.parse_args()


args = parse_args()
state = {
    "connected": False,
    "airborne": False,
    "video_token": "",
    "rc_lease": "",
    "rc_sequence": 0,
    "mission": "idle",
    "control_mode": "none",
    "preview_active": False,
    "preview_confirmed": False,
    "preview_profile": None,
    "sequence": 0,
    "events": [],
    "profiles": [{"name": "operator-a", "photoCount": 3, "modelName": "osnet_x0_25"}],
}


def status():
    airborne = state["airborne"]
    mission_active = state["mission"] != "idle"
    manual = not mission_active or state["control_mode"] == "manual"
    return {
        "battery": 88,
        "heightCm": 72 if airborne else 12,
        "frontTofCm": 150,
        "frontTofState": "clear",
        "controlHz": 30.0,
        "flightState": "FOLLOWING" if mission_active else "手动悬停" if airborne else "地面待机",
        "phase": "自动任务" if mission_active else "手动飞行" if airborne else "检查",
        "videoReady": True,
        "airborne": airborne,
        "canTakeoff": not airborne,
        "rcEnabled": airborne and manual,
        "preflight": {
            "sdk": True,
            "video": True,
            "battery": True,
            "bottomTof": True,
            "frontTof": True,
        },
    }


def runtime_snapshot():
    airborne = state["airborne"]
    connected = state["connected"]
    mission = state["mission"] if state["mission"] != "idle" else "manual" if connected else "idle"
    allowed = ["connect"]
    if connected:
        if state["mission"] != "idle":
            allowed = ["stop_mission", "emergency_stop_mission", "select_control_mode", "toggle_mission_pause"]
        else:
            allowed = ["refresh_status", "stop"]
            allowed.append("stop_preview" if state["preview_active"] else "start_preview")
            allowed.append("start_mission")
            if not airborne:
                allowed.append("takeoff")
    preview = {
        "active": state["preview_active"],
        "state": "running" if state["preview_active"] else "idle",
        "profileName": state["preview_profile"],
        "confirmed": state["preview_confirmed"],
        "stableFrames": 10 if state["preview_confirmed"] else 0,
        "requiredStableFrames": 10,
        "found": state["preview_confirmed"],
        "ambiguous": False,
        "similarity": 0.82 if state["preview_confirmed"] else None,
        "candidateCount": 1 if state["preview_confirmed"] else 0,
        "orientationDeg": 90.0 if state["preview_confirmed"] else None,
        "fps": 12.0 if state["preview_active"] else 0.0,
    }
    return {
        "sequence": state["sequence"],
        "phase": "airborne" if airborne else "preflight" if connected else "disconnected",
        "mission": mission,
        "controlMode": state["control_mode"] if state["mission"] != "idle" else "manual" if airborne else "none",
        "connected": connected,
        "airborne": airborne,
        "streaming": connected,
        "flightState": status()["flightState"],
        "allowedActions": allowed,
        "telemetry": {"preview": preview, "paused": False},
        "error": None,
    }


class Handler(BaseHTTPRequestHandler):
    def _authorized(self):
        return self.headers.get("X-Phantom-Token") == args.token

    def _json(self, code, body):
        payload = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        return json.loads(self.rfile.read(length)) if length else {}

    def _v1_command(self, payload):
        command = payload.get("type")
        if command == "device.connect":
            state["connected"] = True
            state["airborne"] = os.environ.get("PHANTOMFILMER_TEST_AIRBORNE") == "1"
            result = status()
            crash_after = os.environ.get("PHANTOMFILMER_TEST_CRASH_AFTER_CONNECT") == "1"
        elif command == "device.status.refresh":
            result = status()
            crash_after = False
        elif command == "flight.takeoff":
            state["airborne"] = True
            if os.environ.get("PHANTOMFILMER_TEST_CRASH_DURING_TAKEOFF") == "1":
                os._exit(24)
            result = status()
            crash_after = False
        elif command == "flight.land":
            state["airborne"] = False
            state["rc_lease"] = ""
            result = status()
            crash_after = False
        elif command == "flight.hover":
            state["rc_lease"] = ""
            result = status()
            crash_after = False
        elif command in ("device.stop", "flight.emergency_land"):
            state["airborne"] = False
            state["connected"] = False
            state["rc_lease"] = ""
            result = {"ok": True}
            crash_after = False
        elif command == "preview.start":
            state["preview_active"] = True
            state["preview_confirmed"] = True
            state["preview_profile"] = payload.get("profileName")
            result = {"ok": True, **runtime_snapshot()["telemetry"]["preview"]}
            crash_after = False
        elif command == "preview.stop":
            state["preview_active"] = False
            state["preview_confirmed"] = False
            state["preview_profile"] = None
            result = {"ok": True, **runtime_snapshot()["telemetry"]["preview"]}
            crash_after = False
        elif command == "mission.start":
            state["mission"] = payload.get("mission", "reid_follow")
            state["control_mode"] = payload.get("initialControlMode", "normal")
            state["preview_active"] = False
            state["airborne"] = True
            if os.environ.get("PHANTOMFILMER_TEST_CRASH_DURING_TAKEOFF") == "1":
                os._exit(24)
            result = {"ok": True, "mission": state["mission"]}
            crash_after = False
        elif command in ("mission.stop", "mission.emergency_stop"):
            state["mission"] = "idle"
            state["control_mode"] = "none"
            state["airborne"] = False
            result = {"ok": True}
            crash_after = False
        elif command == "mission.control_mode.select":
            state["control_mode"] = payload.get("mode", "normal")
            result = {"ok": True, "mode": state["control_mode"]}
            crash_after = False
        elif command == "mission.pause.toggle":
            result = {"ok": True}
            crash_after = False
        else:
            self._json(400, {"apiVersion": "1", "error": {"message": "bad command"}})
            return
        state["sequence"] += 1
        snapshot = runtime_snapshot()
        state["events"].append(
            {
                "sequence": state["sequence"],
                "occurredAt": 0,
                "type": "command.completed",
                "payload": {"command": command},
                "snapshot": snapshot,
            }
        )
        self._json(
            200,
            {
                "apiVersion": "1",
                "commandId": payload.get("commandId", "fixture"),
                "result": result,
                "snapshot": snapshot,
            },
        )
        if crash_after:
            threading.Timer(0.15, lambda: os._exit(23)).start()

    def do_GET(self):
        parts = urlsplit(self.path)
        if parts.path.endswith("/video/stream"):
            token = parse_qs(parts.query).get("token", [""])[0]
            if not token or token != state["video_token"]:
                self._json(401, {"error": "bad video token"})
                return
            state["video_token"] = ""
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(PIXEL)))
            self.end_headers()
            self.wfile.write(PIXEL)
            return
        if not self._authorized():
            self._json(401, {"error": "unauthorized"})
            return
        if parts.path == "/api/health":
            self._json(200, {"ok": True, "connected": state["connected"]})
        elif parts.path == "/api/v1/health":
            self._json(200, {"apiVersion": "1", "ok": True, "connected": state["connected"]})
        elif parts.path == "/api/v1/capabilities":
            self._json(
                200,
                {
                    "apiVersion": "1",
                    "commands": ["device.connect", "flight.takeoff", "flight.land", "preview.start", "preview.stop", "mission.start", "mission.stop", "mission.emergency_stop", "mission.control_mode.select", "mission.pause.toggle"],
                    "missions": ["manual", "follow", "reid_follow", "fixed_demo"],
                    "eventReplay": True,
                    "rcLease": {"required": True, "ttlMs": 1000},
                    "preview": {"requiredForAutomaticMission": False, "stableFrames": 0, "maxAgeMs": 0},
                    "missionReadiness": {"available": True, "missingAssets": [], "profileRequired": True},
                },
            )
        elif parts.path == "/api/v1/profiles":
            self._json(200, {"apiVersion": "1", "profiles": state["profiles"]})
        elif parts.path.startswith("/api/v1/profiles/"):
            name = unquote(parts.path.removeprefix("/api/v1/profiles/"))
            profile = next((item for item in state["profiles"] if item["name"] == name), None)
            if profile is None:
                self._json(404, {"error": {"message": "profile missing"}})
            else:
                self._json(200, {"apiVersion": "1", "profile": {**profile, "photos": []}})
        elif parts.path == "/api/v1/runtime/snapshot":
            self._json(200, {"apiVersion": "1", "snapshot": runtime_snapshot()})
        elif parts.path == "/api/v1/runtime/events":
            since = int(parse_qs(parts.query).get("since", ["0"])[0])
            events = [event for event in state["events"] if event["sequence"] > since]
            self._json(
                200,
                {
                    "apiVersion": "1",
                    "latestSequence": state["sequence"],
                    "resetRequired": False,
                    "events": events,
                },
            )
        elif parts.path == "/api/drone/status":
            self._json(200, status())
        else:
            self._json(404, {"error": "missing"})

    def do_POST(self):
        if not self._authorized():
            self._json(401, {"error": "unauthorized"})
            return
        path = urlsplit(self.path).path
        if path == "/api/v1/commands":
            self._v1_command(self._read_json())
        elif path == "/api/v1/rc/lease":
            if not state["airborne"]:
                self._json(409, {"apiVersion": "1", "error": {"message": "not airborne"}})
                return
            state["rc_lease"] = secrets.token_urlsafe(16)
            state["rc_sequence"] = 0
            self._json(
                201,
                {
                    "apiVersion": "1",
                    "leaseId": state["rc_lease"],
                    "lastSequence": 0,
                    "expiresAt": 9999999999999,
                },
            )
        elif path == "/api/v1/rc":
            payload = self._read_json()
            if (
                payload.get("leaseId") != state["rc_lease"]
                or payload.get("sequence", 0) <= state["rc_sequence"]
            ):
                self._json(409, {"apiVersion": "1", "error": {"message": "stale lease"}})
                return
            state["rc_sequence"] = payload["sequence"]
            self._json(
                200,
                {
                    "apiVersion": "1",
                    "ok": True,
                    "flightState": "手动飞行",
                    "leaseId": state["rc_lease"],
                    "lastSequence": state["rc_sequence"],
                    "expiresAt": 9999999999999,
                },
            )
        elif path == "/api/v1/rc/release":
            payload = self._read_json()
            released = payload.get("leaseId") == state["rc_lease"]
            state["rc_lease"] = ""
            self._json(200, {"apiVersion": "1", "ok": True, "released": released})
        elif path == "/api/drone/connect":
            state["connected"] = True
            state["airborne"] = os.environ.get("PHANTOMFILMER_TEST_AIRBORNE") == "1"
            self._json(200, status())
            if os.environ.get("PHANTOMFILMER_TEST_CRASH_AFTER_CONNECT") == "1":
                threading.Timer(0.15, lambda: os._exit(23)).start()
        elif path == "/api/drone/takeoff":
            state["airborne"] = True
            if os.environ.get("PHANTOMFILMER_TEST_CRASH_DURING_TAKEOFF") == "1":
                os._exit(24)
            self._json(200, status())
        elif path == "/api/drone/land":
            state["airborne"] = False
            self._json(200, status())
        elif path == "/api/drone/hover":
            self._json(200, status())
        elif path == "/api/drone/rc":
            self._json(200, {"ok": True, "flightState": "手动飞行"})
        elif path == "/api/drone/input":
            key = str(self._read_json().get("key", "")).lower()
            mode_for_key = {"1": "normal", "2": "side", "3": "front", "m": "manual"}
            if key in mode_for_key and state["mission"] != "idle":
                state["control_mode"] = mode_for_key[key]
            self._json(200, {"ok": True, "operatorSequence": state["sequence"], "key": key})
        elif path in ("/api/drone/stop", "/api/drone/emergency-land"):
            state["airborne"] = False
            state["connected"] = False
            self._json(200, {"ok": True})
        elif path == "/api/drone/video-token":
            state["video_token"] = secrets.token_urlsafe(16)
            self._json(200, {"token": state["video_token"], "expiresAt": 9999999999999})
        elif path == "/api/sidecar/shutdown":
            state["airborne"] = False
            self._json(200, {"ok": True})
            threading.Thread(target=self.server.shutdown, daemon=True).start()
        else:
            self._json(404, {"error": "missing"})

    def do_PATCH(self):
        if not self._authorized():
            self._json(401, {"error": "unauthorized"})
            return
        path = urlsplit(self.path).path
        if not path.startswith("/api/v1/profiles/"):
            self._json(404, {"error": "missing"})
            return
        name = unquote(path.removeprefix("/api/v1/profiles/"))
        new_name = self._read_json().get("name", "")
        profile = next((item for item in state["profiles"] if item["name"] == name), None)
        if profile is None or not new_name:
            self._json(409, {"error": {"message": "rename failed"}})
            return
        profile["name"] = new_name
        self._json(200, {"apiVersion": "1", "profile": {**profile, "photos": []}})

    def do_DELETE(self):
        if not self._authorized():
            self._json(401, {"error": "unauthorized"})
            return
        path = urlsplit(self.path).path
        name = unquote(path.removeprefix("/api/v1/profiles/"))
        before = len(state["profiles"])
        state["profiles"] = [item for item in state["profiles"] if item["name"] != name]
        if len(state["profiles"]) == before:
            self._json(409, {"error": {"message": "delete failed"}})
            return
        self._json(200, {"apiVersion": "1", "deleted": {"name": name, "deletedAt": "2026-01-01T00:00:00Z", "recoverable": True}})

    def log_message(self, _format, *_args):
        pass


server = ThreadingHTTPServer((args.host, args.port), Handler)
print(json.dumps({"event": "ready", "host": args.host, "port": server.server_address[1], "pid": os.getpid(), "logPath": os.path.join(args.data_dir, "logs", "test.log")}), flush=True)
server.serve_forever(poll_interval=0.05)
server.server_close()
