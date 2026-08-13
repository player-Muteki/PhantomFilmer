"""On-screen current-mode overlay for the live follow camera window.

This module is intentionally self-contained: it only derives a short,
human-readable mode label from plain state values and draws a prominent banner
onto an already-composed frame.  It imports nothing from ``control``,
``vision``, or ``drone``, so it can be dropped into any display path without
affecting flight logic or safety behaviour.
"""

from typing import Any, Optional

import numpy as np

# 避障规划器的非 CLEAR 状态 → 更易读的中文标签。
_AVOIDANCE_LABELS = {
    "BLOCKED": "避障绕行 AVOID",
    "AVOIDING": "避障绕行 AVOID",
    "CAUTION": "避障减速 CAUTION",
    "SCAN": "避障扫描 SCAN",
    "BRAKING": "避障刹车 BRAKE",
    "RECOVERING": "避障恢复 RECOVER",
    "FAILSAFE": "避障保护 FAILSAFE",
}

_SEARCH_STATES = (
    "SEARCH",
    "LOST_HOLD",
    "CLOSE_BACKOFF",
    "MOVE_TO_LAYER",
    "LAYER_SCAN",
    "RETURN_TO_BASE",
    "FALLBACK_SEARCH",
)

_OCCLUSION_STATES = (
    "OCCLUSION",
    "LOSS_UNCERTAIN",
)


def resolve_mode_text(
    *,
    session_state: str = "",
    avoidance_state: Optional[str] = None,
    paused: bool = False,
    emergency: bool = False,
) -> str:
    """Return a short label describing the current flight mode.

    The label is derived purely from already-available session/planner state and
    does not change any control output.  Priority order (highest first):
    emergency, pause, landing/stop, scripted route, search, occlusion recovery,
    initial acquisition, obstacle takeover, and normal following.
    """
    if emergency:
        return "急停 EMERGENCY"
    if paused:
        return "暂停 PAUSED"

    state = (session_state or "").strip().upper()

    if not state or state == "IDLE":
        return "待机 IDLE"
    if "LANDING" in state or state in ("STOPPED", "EMERGENCY_STOP"):
        return "降落 LANDING"
    if state == "FIXED_DEMO":
        return "固定航线 FIXED DEMO"
    if state == "OBSTACLE_FIRST":
        return "避障优先 OBSTACLE FIRST"

    if state.startswith(_SEARCH_STATES):
        return "目标搜索 SEARCH"
    if state.startswith(_OCCLUSION_STATES):
        return "遮挡恢复 RECOVERY"
    if state.startswith("REACQUIRE"):
        return "身份确认 REACQUIRE"
    if state.startswith("INITIAL"):
        return "初次采集 ACQUIRE"

    # 跟随过程中，避障规划器一旦离开 CLEAR 即视为避障接管。
    if avoidance_state and str(avoidance_state).strip().upper() != "CLEAR":
        return _AVOIDANCE_LABELS.get(
            str(avoidance_state).strip().upper(),
            f"避障 {avoidance_state}",
        )

    return "跟随 FOLLOWING"


def draw_mode_overlay(frame: Any, mode_text: str) -> Any:
    """Draw a prominent full-width mode banner at the top of *frame* in place.

    The banner is drawn last, so it never depends on or mutates the underlying
    detection/telemetry overlays.  Returns the same frame for easy chaining.
    """
    try:
        import cv2
    except ModuleNotFoundError:
        return frame

    if (
        frame is None
        or not isinstance(frame, np.ndarray)
        or frame.ndim < 2
        or frame.size == 0
    ):
        return frame

    height, width = frame.shape[:2]
    bar_height = 48
    if width < 220 or height < bar_height:
        return frame

    cv2.rectangle(frame, (0, 0), (width, bar_height), (15, 15, 15), -1)
    cv2.putText(
        frame,
        "当前模式",
        (14, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 200, 255),
        2,
        cv2.LINE_AA,
    )

    font_scale = 0.9
    thickness = 2
    (text_width, _text_height), _baseline = cv2.getTextSize(
        mode_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
    )
    text_x = max(0, (width - text_width) // 2)
    cv2.putText(
        frame,
        mode_text,
        (text_x, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (0, 255, 255),
        thickness,
        cv2.LINE_AA,
    )
    cv2.line(frame, (0, bar_height - 1), (width, bar_height - 1), (0, 200, 255), 1)
    return frame
