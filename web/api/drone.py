"""Safe REST commands mapped to the ConsoleTools task whitelist."""

import asyncio
from typing import Any, Callable

from fastapi import APIRouter, HTTPException, Request, status
from pydantic import BaseModel


router = APIRouter(prefix="/api", tags=["drone"])


class StatusResponse(BaseModel):
    battery: int
    height: int
    mode: str


class StartTaskRequest(BaseModel):
    confirmed: bool = False


async def _run(command: Callable[..., Any], *args: Any) -> Any:
    try:
        return await asyncio.to_thread(command, *args)
    except RuntimeError as exc:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail=str(exc)) from exc


def _tools(request: Request):
    return request.app.state.app_state.require_tools()


@router.post("/connect")
async def connect(request: Request):
    state = request.app.state.app_state
    snapshot = await _run(state.connect)
    return {
        "ok": True,
        "mode": state.tools.current_mode,
        "connection_state": snapshot["connection_state"],
        "connection_verified": snapshot["connection_verified"],
        "battery": snapshot["battery"],
    }


@router.get("/status", response_model=StatusResponse)
async def get_status(request: Request):
    state = request.app.state.app_state
    await _run(state.connection.require_verified)
    data = state.telemetry_snapshot()
    return StatusResponse(battery=data["battery"], height=data["height"], mode=data["mode"])


@router.get("/task/can-start")
async def can_start_task(request: Request):
    allowed, message = await _run(request.app.state.app_state.can_start_task)
    return {"allowed": allowed, "message": message}


@router.post("/task/start")
async def start_task(payload: StartTaskRequest, request: Request):
    if not payload.confirmed:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="必须明确确认飞行环境安全后才能启动任务。",
        )
    state = request.app.state.app_state
    started = await _run(state.start_task)
    if not started:
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="跟随任务未启动，请检查就绪状态。")
    return {"ok": True, "mode": state.tools.current_mode}


@router.post("/task/stop")
async def stop_task(request: Request):
    tools = _tools(request)
    await _run(tools.stop_task)
    return {"ok": True, "mode": tools.current_mode}


@router.post("/emergency-stop")
async def emergency_stop(request: Request):
    tools = _tools(request)
    await _run(tools.emergency_stop)
    return {"ok": True, "mode": tools.current_mode}


@router.get("/task/active")
async def task_active(request: Request):
    return {"active": await _run(_tools(request).is_task_active)}
