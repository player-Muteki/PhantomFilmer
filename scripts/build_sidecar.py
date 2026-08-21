import shutil
import subprocess
import sys
from pathlib import Path


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
    executable_name = "phantomfilmer-sidecar.exe" if sys.platform == "win32" else "phantomfilmer-sidecar"
    executable = output_dir / "phantomfilmer-sidecar" / executable_name
    if not executable.is_file():
        raise RuntimeError(f"sidecar 构建未生成预期文件：{executable}")
    print(executable)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
