"""FastAPI application entry point for PhantomFilmer WebUI."""

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from app.builder import build_system
from app.config import load_config
from console.tools import ConsoleTools
from web.api import drone, telemetry, video
from web.state import AppState


FRONTEND_DIST = Path(__file__).resolve().parent / "frontend" / "dist"


def create_app(
    obstacle_enabled: Optional[bool] = None,
    *,
    tools: Optional[ConsoleTools] = None,
) -> FastAPI:
    """Create a real-aircraft WebUI; ``tools`` exists only for isolated tests."""
    config = load_config()
    web_config = config.get("web", {}) if isinstance(config.get("web", {}), dict) else {}
    if tools is None:
        controller = build_system(use_fake=False, obstacle_enabled=obstacle_enabled)
        tools = controller.tools
    state = AppState.create(tools=tools, web_config=web_config)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        yield
        await asyncio.to_thread(state.close)

    app = FastAPI(title="PhantomFilmer WebUI", lifespan=lifespan)
    if bool(web_config.get("cors_enabled", True)):
        app.add_middleware(
            CORSMiddleware,
            allow_origins=[
                "http://localhost:5173",
                "http://127.0.0.1:5173",
                "http://localhost:8080",
                "http://127.0.0.1:8080",
            ],
            allow_methods=["*"],
            allow_headers=["*"],
        )
    app.state.app_state = state
    app.state.telemetry_interval = max(0.1, float(web_config.get("telemetry_interval_seconds", 0.2)))
    app.include_router(drone.router)
    app.include_router(video.router)
    app.include_router(telemetry.router)
    if FRONTEND_DIST.exists():
        app.mount("/", StaticFiles(directory=FRONTEND_DIST, html=True), name="frontend")
    return app
