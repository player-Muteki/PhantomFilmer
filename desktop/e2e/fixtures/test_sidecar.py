import argparse
import base64
import json
import os
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit


PIXEL = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
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
}


def status():
    airborne = state["airborne"]
    return {
        "battery": 88,
        "heightCm": 72 if airborne else 12,
        "frontTofCm": 150,
        "frontTofState": "clear",
        "controlHz": 30.0,
        "flightState": "手动悬停" if airborne else "地面待机",
        "phase": "手动飞行" if airborne else "检查",
        "videoReady": True,
        "airborne": airborne,
        "canTakeoff": not airborne,
        "rcEnabled": airborne,
        "preflight": {
            "sdk": True,
            "video": True,
            "battery": True,
            "bottomTof": True,
            "frontTof": True,
        },
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
        else:
            self._json(400, {"apiVersion": "1", "error": {"message": "bad command"}})
            return
        self._json(
            200,
            {
                "apiVersion": "1",
                "commandId": payload.get("commandId", "fixture"),
                "result": result,
                "snapshot": {},
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

    def log_message(self, _format, *_args):
        pass


server = ThreadingHTTPServer((args.host, args.port), Handler)
print(json.dumps({"event": "ready", "host": args.host, "port": server.server_address[1], "pid": os.getpid(), "logPath": os.path.join(args.data_dir, "logs", "test.log")}), flush=True)
server.serve_forever(poll_interval=0.05)
server.server_close()
