#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

command -v "$PYTHON_BIN" >/dev/null 2>&1 || { echo "未找到 Python 3。" >&2; exit 1; }
command -v node >/dev/null 2>&1 || { echo "未找到 Node.js 20 或更高版本。" >&2; exit 1; }
command -v npm >/dev/null 2>&1 || { echo "未找到 npm。" >&2; exit 1; }

cd "$PROJECT_DIR"
"$PYTHON_BIN" -m venv .venv-webui
.venv-webui/bin/python -m pip install --upgrade pip
.venv-webui/bin/python -m pip install -r requirements-webui.txt

cd "$PROJECT_DIR/webui"
npm ci
npm run build

echo
echo "安装完成。连接 RMTT-XXXXXX Wi-Fi 后运行："
echo "  bash scripts/start_webui.sh"
