#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="$PROJECT_DIR/.venv-webui/bin/python"
BACKEND_PID=""

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "缺少 .venv-webui，请先在有互联网时运行 bash scripts/install_webui.sh" >&2
  exit 1
fi
if [[ ! -d "$PROJECT_DIR/webui/.next" ]]; then
  echo "缺少 WebUI 构建结果，请先在有互联网时运行 bash scripts/install_webui.sh" >&2
  exit 1
fi

cleanup() {
  if [[ -n "$BACKEND_PID" ]] && kill -0 "$BACKEND_PID" 2>/dev/null; then
    kill "$BACKEND_PID" 2>/dev/null || true
    wait "$BACKEND_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

cd "$PROJECT_DIR"
"$PYTHON_BIN" -m web_api.server &
BACKEND_PID=$!
sleep 1
if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
  echo "真机服务启动失败。" >&2
  exit 1
fi

echo
echo "WebUI 已启动：http://127.0.0.1:3000"
echo "保持本终端运行；按 Ctrl+C 同时关闭网页服务与真机连接。"
echo
cd "$PROJECT_DIR/webui"
npm run start
