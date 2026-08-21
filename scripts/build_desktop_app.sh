#!/usr/bin/env bash
# Build a self-contained native PhantomFilmer desktop application.
#
# This script deliberately keeps Python packages, Node packages and runtime
# caches inside the repository. It downloads the ignored model assets only
# through the verified preparation script, then embeds them in the sidecar.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bash scripts/build_desktop_app.sh

Environment overrides:
  PYTHON_BIN  Python 3.12 executable to create the isolated build environment.
  VENV_DIR    Build virtual environment directory (default: .venv-desktop-build).

The build requires Git, Python 3.12, Node.js 22 or newer and the native build
tools for the current platform. It produces a native artifact in release/.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

PYTHON_BIN="${PYTHON_BIN:-python3.12}"
VENV_DIR="${VENV_DIR:-$PROJECT_ROOT/.venv-desktop-build}"
BUILD_PYTHON="$VENV_DIR/bin/python"
CACHE_ROOT="$PROJECT_ROOT/.build-cache"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少必需工具：$1" >&2
    exit 1
  fi
}

require_command git
require_command node
require_command npm
require_command "$PYTHON_BIN"

"$PYTHON_BIN" - <<'PY'
import sys

if sys.version_info[:2] != (3, 12):
    raise SystemExit(
        f"需要 Python 3.12，当前为 {sys.version.split()[0]}。"
    )
PY

node_major="$(node -p 'process.versions.node.split(".")[0]')"
if [[ "$node_major" -lt 22 ]]; then
  echo "需要 Node.js 22 或更新版本，当前为 $(node --version)。" >&2
  exit 1
fi

if [[ "$(uname -s)" == "Darwin" ]] && ! xcode-select -p >/dev/null 2>&1; then
  echo "macOS 需要安装 Xcode Command Line Tools：xcode-select --install" >&2
  exit 1
fi

if [[ ! -x "$BUILD_PYTHON" ]]; then
  "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

"$BUILD_PYTHON" - <<'PY'
import sys

if sys.version_info[:2] != (3, 12):
    raise SystemExit("现有构建虚拟环境不是 Python 3.12；请删除 .venv-desktop-build 后重试。")
PY

mkdir -p "$CACHE_ROOT/matplotlib" "$CACHE_ROOT/ultralytics"
export MPLCONFIGDIR="$CACHE_ROOT/matplotlib"
export YOLO_CONFIG_DIR="$CACHE_ROOT/ultralytics"
export YOLO_OFFLINE=1

"$BUILD_PYTHON" -m pip install --upgrade pip setuptools wheel
"$BUILD_PYTHON" -m pip install -r requirements-desktop-build.txt
# Torchreid's legacy build backend needs the already installed bootstrap
# packages. This is still isolated inside VENV_DIR, never the system Python.
"$BUILD_PYTHON" -m pip install --no-build-isolation --no-deps -r requirements-desktop-torchreid.txt

"$BUILD_PYTHON" scripts/prepare_desktop_model_assets.py
"$BUILD_PYTHON" scripts/build_sidecar.py

(
  cd desktop
  npm ci
  npm run dist
)

if [[ "$(uname -s)" == "Darwin" ]]; then
  dmg_path="$(find "$PROJECT_ROOT/release" -maxdepth 1 -type f -name '*.dmg' -print -quit)"
  if [[ -z "$dmg_path" ]]; then
    echo "未找到 macOS DMG 产物。" >&2
    exit 1
  fi
  hdiutil verify "$dmg_path"
fi

"$BUILD_PYTHON" - <<'PY'
from hashlib import sha256
from pathlib import Path

release = Path("release")
artifacts = sorted(
    path for path in release.iterdir()
    if path.is_file() and path.name != "SHA256SUMS" and not path.name.endswith(".blockmap")
)
if not artifacts:
    raise SystemExit("未找到可发布的桌面产物。")
lines = []
for artifact in artifacts:
    digest = sha256(artifact.read_bytes()).hexdigest()
    lines.append(f"{digest}  {artifact.name}")
(release / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")
print("\n".join(lines))
PY

echo "桌面构建完成：$PROJECT_ROOT/release"
