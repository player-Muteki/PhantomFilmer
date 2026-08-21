import json
import os
import shutil
import subprocess
import sys
from http.client import HTTPConnection
from pathlib import Path
from queue import Empty, Queue
from tempfile import TemporaryDirectory
from threading import Thread
from typing import TextIO


def _read_ready_line(stream: TextIO, output: Queue[str]) -> None:
    """Read the sidecar's single startup record without blocking the builder."""
    output.put(stream.readline())


def _smoke_test(executable: Path) -> None:
    """Start the packaged sidecar and verify authenticated graceful shutdown."""
    token = "phantomfilmer-build-smoke"
    with TemporaryDirectory(prefix="phantomfilmer-sidecar-smoke-") as data_dir:
        process = subprocess.Popen(
            [
                str(executable),
                "--host",
                "127.0.0.1",
                "--port",
                "0",
                "--token",
                token,
                "--data-dir",
                data_dir,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            if process.stdout is None or process.stderr is None:
                raise RuntimeError("sidecar 冒烟测试无法捕获子进程输出")
            ready_lines: Queue[str] = Queue(maxsize=1)
            reader = Thread(
                target=_read_ready_line,
                args=(process.stdout, ready_lines),
                daemon=True,
            )
            reader.start()
            # macOS may perform first-launch verification on a newly written
            # Mach-O bundle. Keep this above Electron's steady-state startup
            # time while still bounding a genuinely wedged build.
            reader.join(timeout=30)
            if reader.is_alive():
                raise RuntimeError(
                    "sidecar 冒烟测试等待 ready 超时" f"（进程状态：{process.poll()}）"
                )
            try:
                ready_line = ready_lines.get_nowait()
            except Empty as exc:
                raise RuntimeError("sidecar 冒烟测试未收到 ready 输出") from exc
            if not ready_line:
                detail = process.stderr.read().strip()
                raise RuntimeError(f"sidecar 启动失败：{detail or '无错误输出'}")
            try:
                ready = json.loads(ready_line)
                port = int(ready["port"])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise RuntimeError(
                    f"sidecar ready 输出无效：{ready_line.strip()}"
                ) from exc

            connection = HTTPConnection("127.0.0.1", port, timeout=5)
            try:
                connection.request(
                    "POST",
                    "/api/sidecar/shutdown",
                    headers={"X-Phantom-Token": token},
                )
                response = connection.getresponse()
                response.read()
            finally:
                connection.close()
            if response.status != 200:
                raise RuntimeError(
                    f"sidecar 冒烟测试关闭请求失败：HTTP {response.status}"
                )
            if process.wait(timeout=10) != 0:
                detail = process.stderr.read().strip()
                raise RuntimeError(f"sidecar 冒烟测试异常退出：{detail}")
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


def _smoke_test_models(executable: Path) -> None:
    """Load every model from the packaged filesystem before shipping it."""
    with TemporaryDirectory(prefix="phantomfilmer-model-smoke-") as data_dir:
        environment = os.environ.copy()
        environment.update(
            {
                "MPLCONFIGDIR": data_dir,
                "YOLO_CONFIG_DIR": str(Path(data_dir) / "ultralytics"),
                "YOLO_OFFLINE": "1",
            }
        )
        try:
            result = subprocess.run(
                [str(executable), "--verify-models"],
                cwd=executable.parent,
                env=environment,
                capture_output=True,
                text=True,
                timeout=300,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("模型冒烟测试超时（300 秒）") from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip() or "无错误输出"
            raise RuntimeError(f"模型冒烟测试失败：{detail}")
        records = []
        for line in result.stdout.splitlines():
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(record, dict):
                records.append(record)
        if not any(record.get("event") == "model-runtime-ready" for record in records):
            raise RuntimeError(
                "模型冒烟测试未报告成功：" + (result.stdout.strip() or "无标准输出")
            )


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    output_dir = project_root / "dist" / "sidecar"
    work_dir = project_root / "build" / "pyinstaller"
    if output_dir.exists():
        shutil.rmtree(output_dir)
    if work_dir.exists():
        shutil.rmtree(work_dir)
    output_dir.mkdir(parents=True)
    work_dir.mkdir(parents=True)
    subprocess.run(
        [
            sys.executable,
            "-m",
            "PyInstaller",
            "--noconfirm",
            "--clean",
            "--distpath",
            str(output_dir),
            "--workpath",
            str(work_dir),
            str(project_root / "sidecar" / "phantomfilmer_sidecar.spec"),
        ],
        cwd=project_root,
        check=True,
    )
    executable_name = (
        "phantomfilmer-sidecar.exe"
        if sys.platform == "win32"
        else "phantomfilmer-sidecar"
    )
    executable = output_dir / "phantomfilmer-sidecar" / executable_name
    if not executable.is_file():
        raise RuntimeError(f"sidecar 构建未生成预期文件：{executable}")
    _smoke_test(executable)
    _smoke_test_models(executable)
    print(executable)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
