import json
import subprocess
import sys
from http.client import HTTPConnection
from pathlib import Path
from tempfile import TemporaryDirectory


def test_sidecar_prints_ready_json_and_accepts_protected_shutdown() -> None:
    with TemporaryDirectory() as temporary_directory:
        process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "web_api.server",
                "--port",
                "0",
                "--token",
                "lifecycle-token",
                "--data-dir",
                temporary_directory,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            assert process.stdout is not None
            ready = json.loads(process.stdout.readline())
            assert ready["event"] == "ready"
            assert ready["port"] > 0
            assert Path(ready["logPath"]).parent == Path(temporary_directory) / "logs"

            connection = HTTPConnection("127.0.0.1", ready["port"], timeout=3)
            connection.request(
                "POST",
                "/api/sidecar/shutdown",
                headers={"X-Phantom-Token": "lifecycle-token"},
            )
            response = connection.getresponse()
            response.read()
            connection.close()

            assert response.status == 200
            assert process.wait(timeout=5) == 0
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
