"""MJPEG camera stream for browser-native rendering."""

import asyncio

from fastapi import APIRouter, HTTPException, Request, status
from fastapi.responses import StreamingResponse


router = APIRouter(tags=["video"])


@router.get("/video/stream")
async def video_stream(request: Request):
    state = request.app.state.app_state
    try:
        await asyncio.to_thread(state.prepare_video)
    except RuntimeError as exc:
        raise HTTPException(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, detail=str(exc)) from exc
    return StreamingResponse(
        state.video.iter_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
