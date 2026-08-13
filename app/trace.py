"""Runtime behaviour tracing (Rust-style ``dbg!``/``#[instrument]``).

When enabled (via ``--trace``), every drone behaviour-interface call is written
to a JSONL file in real time.  When disabled the module is a no-op, so flight
paths are completely unchanged.

Design
------
* :class:`TraceLogger` owns one append-mode file stream and flushes every event,
  so a crash or power cut never loses the last recorded behaviour.
* :func:`trace_drone` wraps a :class:`drone.drone_adapter.DroneAdapter` in a thin
  proxy that logs every method call (arguments, result, latency, exceptions)
  without touching the adapter's implementation.
* :func:`trace_call` is a decorator for recording higher-level decision methods
  (e.g. ``FollowController.compute_command``) the same way.
"""

from __future__ import annotations

import atexit
import functools
import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from time import monotonic
from typing import Any, Callable, Optional

_TRACE_DIR = Path("logs/trace")
_TRACE_LOGGER: Optional["TraceLogger"] = None
_GLOBAL_LOCK = threading.Lock()
_TRACED_MARKER = "_phantomfilmer_trace_proxy"


def _default_trace_path() -> Path:
    """Return a timestamped trace file path under logs/trace/."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return _TRACE_DIR / f"{stamp}.jsonl"


class TraceLogger:
    """Thread-safe, real-time JSONL file sink for behaviour traces."""

    def __init__(self, path: Path, flush: bool = True) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")
        self._lock = threading.Lock()
        self._flush = flush
        self._closed = False

    def log(self, event: str, **fields: Any) -> None:
        """Write one JSONL event, optionally flushing immediately."""
        if self._closed:
            return
        payload: dict[str, Any] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "monotonic": round(monotonic(), 6),
            "event": event,
        }
        payload.update(fields)
        try:
            line = json.dumps(payload, ensure_ascii=False, default=_json_default)
        except (TypeError, ValueError):
            line = json.dumps(
                {
                    "timestamp": payload["timestamp"],
                    "event": event,
                    "serialization_error": True,
                },
                ensure_ascii=False,
            )
        with self._lock:
            if self._closed:
                return
            self._file.write(line + "\n")
            if self._flush:
                self._file.flush()

    def close(self) -> None:
        """Flush and close the underlying file stream (idempotent)."""
        with self._lock:
            if self._closed:
                return
            try:
                self._file.flush()
                self._file.close()
            except (OSError, ValueError):
                pass
            finally:
                self._closed = True


def enable_trace(path: Optional[str] = None) -> TraceLogger:
    """Open the trace file and start recording (idempotent).

    The logger is closed automatically at interpreter exit via ``atexit``, so
    callers only need to enable it once and keep flying.
    """
    global _TRACE_LOGGER
    with _GLOBAL_LOCK:
        if _TRACE_LOGGER is not None:
            return _TRACE_LOGGER
        target = Path(path) if path else _default_trace_path()
        _TRACE_LOGGER = TraceLogger(target)
        atexit.register(_close_trace)
        _TRACE_LOGGER.log(
            "trace_start",
            pid=os.getpid(),
            trace_file=str(_TRACE_LOGGER.path),
        )
        return _TRACE_LOGGER


def disable_trace() -> None:
    """Stop tracing and close the file stream (safe to call multiple times)."""
    _close_trace()


def _close_trace() -> None:
    global _TRACE_LOGGER
    logger = _TRACE_LOGGER
    _TRACE_LOGGER = None
    if logger is not None:
        try:
            logger.log("trace_end")
        finally:
            logger.close()


def is_trace_enabled() -> bool:
    """Return whether a trace logger is currently active."""
    return _TRACE_LOGGER is not None


def get_trace_logger() -> Optional[TraceLogger]:
    """Return the active trace logger, or ``None`` when tracing is disabled."""
    return _TRACE_LOGGER


def prompt_trace_enabled(default_enabled: bool = True) -> Optional[bool]:
    """Ask whether behaviour tracing should be enabled for this run.

    Returns the user's choice, or ``None`` when the user cancels via EOF or
    Ctrl+C.  An empty answer falls back to ``default_enabled``.
    """
    default_label = "开启" if default_enabled else "关闭"
    while True:
        try:
            answer = input(
                f"本次运行是否开启行为跟踪？[y/n]（默认：{default_label}）："
            ).strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n已取消本次运行。")
            return None
        if not answer:
            return default_enabled
        if answer in {"y", "yes", "是", "开启"}:
            return True
        if answer in {"n", "no", "否", "关闭"}:
            return False
        print("请输入 y/n、是/否或直接回车使用默认值。")


def trace_drone(drone: Any, logger: Optional[TraceLogger] = None) -> Any:
    """Wrap a drone adapter so every behaviour call is recorded.

    Returns the original object when tracing is disabled or the object is
    already wrapped, so this is always safe to call.
    """
    if not is_trace_enabled() and logger is None:
        return drone
    if getattr(drone, _TRACED_MARKER, False) is True:
        return drone
    return _TracedDroneProxy(drone, logger or get_trace_logger())


class _TracedDroneProxy:
    """Transparent proxy that logs each DroneAdapter method invocation."""

    def __init__(self, drone: Any, logger: Optional[TraceLogger]) -> None:
        object.__setattr__(self, "_drone", drone)
        object.__setattr__(self, "_logger", logger)
        object.__setattr__(self, _TRACED_MARKER, True)

    def __getattr__(self, name: str) -> Any:
        drone = object.__getattribute__(self, "_drone")
        logger = object.__getattribute__(self, "_logger")
        attr = getattr(drone, name)
        if not callable(attr):
            return attr

        @functools.wraps(attr)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            if logger is None:
                return attr(*args, **kwargs)
            started = monotonic()
            try:
                result = attr(*args, **kwargs)
                logger.log(
                    "drone_call",
                    interface=name,
                    args=_summarize(args),
                    kwargs=_summarize(kwargs),
                    result=_summarize(result),
                    elapsed_ms=round((monotonic() - started) * 1000.0, 3),
                    status="ok",
                )
                return result
            except Exception as exc:
                logger.log(
                    "drone_call",
                    interface=name,
                    args=_summarize(args),
                    kwargs=_summarize(kwargs),
                    elapsed_ms=round((monotonic() - started) * 1000.0, 3),
                    status="error",
                    error_type=type(exc).__name__,
                    error=str(exc),
                )
                raise

        return wrapper


def trace_call(func: Optional[Callable[..., Any]] = None, *, name: Optional[str] = None) -> Any:
    """Decorate an instance method to record calls when tracing is enabled.

    The first positional argument (``self``) is omitted from the trace.  When
    tracing is disabled the decorator is a transparent pass-through.
    """

    def decorator(fn: Callable[..., Any]) -> Callable[..., Any]:
        fn_name = name or f"{fn.__module__}.{fn.__qualname__}"

        @functools.wraps(fn)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            logger = get_trace_logger()
            if logger is None:
                return fn(*args, **kwargs)
            started = monotonic()
            call_args = args[1:] if args else ()
            try:
                result = fn(*args, **kwargs)
                logger.log(
                    "call",
                    function=fn_name,
                    args=_summarize(call_args),
                    kwargs=_summarize(kwargs),
                    result=_summarize(result),
                    elapsed_ms=round((monotonic() - started) * 1000.0, 3),
                    status="ok",
                )
                return result
            except Exception as exc:
                logger.log(
                    "call",
                    function=fn_name,
                    args=_summarize(call_args),
                    kwargs=_summarize(kwargs),
                    elapsed_ms=round((monotonic() - started) * 1000.0, 3),
                    status="error",
                    error_type=type(exc).__name__,
                    error=str(exc),
                )
                raise

        return wrapper

    if func is not None:
        return decorator(func)
    return decorator


def _summarize(value: Any, max_len: int = 200, depth: int = 0) -> Any:
    """Reduce a value to a JSON-safe, bounded representation.

    Scalars pass through, strings are truncated, numpy-like arrays become a
    shape/dtype descriptor, and other objects become a type name plus a short
    repr.  This keeps high-frequency calls (``move_rc``) and large returns
    (``get_frame``) from flooding the trace file.
    """
    if value is None or isinstance(value, (bool, int, float)):
        return value
    if isinstance(value, str):
        return value if len(value) <= max_len else value[:max_len] + "..."
    if depth >= 3:
        return f"<{type(value).__module__}.{type(value).__name__}>"
    if isinstance(value, (list, tuple)):
        return [_summarize(v, max_len, depth + 1) for v in value[:32]]
    if isinstance(value, dict):
        return {
            str(key): _summarize(item, max_len, depth + 1)
            for key, item in list(value.items())[:32]
        }
    shape = getattr(value, "shape", None)
    dtype = getattr(value, "dtype", None)
    if shape is not None or dtype is not None:
        return {
            "type": f"{type(value).__module__}.{type(value).__name__}",
            "shape": list(shape) if shape is not None else None,
            "dtype": str(dtype) if dtype is not None else None,
        }
    return {
        "type": f"{type(value).__module__}.{type(value).__name__}",
        "repr": repr(value)[:max_len],
    }


def _json_default(obj: Any) -> Any:
    """Last-resort JSON encoder for values that escaped ``_summarize``."""
    try:
        return str(obj)
    except Exception:
        return f"<{type(obj).__name__}>"
