"""Live flight telemetry over WebSocket."""

import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect


router = APIRouter(tags=["telemetry"])


@router.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket):
    await websocket.accept()
    state = websocket.app.state.app_state
    interval = float(websocket.app.state.telemetry_interval)
    try:
        while True:
            # The WebSocket only distributes the cached snapshot. It never
            # sends SDK queries at its 5 Hz browser update rate.
            await websocket.send_json(state.telemetry_snapshot())
            await asyncio.sleep(interval)
    except (WebSocketDisconnect, RuntimeError):
        return
