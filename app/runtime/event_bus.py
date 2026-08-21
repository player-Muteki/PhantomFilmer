"""Thread-safe sequenced runtime event stream with bounded replay."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import RLock
from time import time
from typing import Any, Callable, Deque, Mapping, Optional

from app.runtime.models import RuntimeSnapshot


@dataclass(frozen=True)
class RuntimeEvent:
    sequence: int
    occurred_at: float
    event_type: str
    payload: Mapping[str, Any]
    snapshot: Optional[RuntimeSnapshot] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "occurredAt": self.occurred_at,
            "type": self.event_type,
            "payload": dict(self.payload),
            "snapshot": self.snapshot.to_dict() if self.snapshot is not None else None,
        }


EventListener = Callable[[RuntimeEvent], None]


class EventBus:
    """Publish monotonic events and replay a bounded history to late clients."""

    def __init__(self, history_limit: int = 1024) -> None:
        if history_limit < 1:
            raise ValueError("history_limit must be at least 1")
        self._lock = RLock()
        self._history: Deque[RuntimeEvent] = deque(maxlen=history_limit)
        self._listeners: dict[int, EventListener] = {}
        self._next_listener_id = 1
        self._sequence = 0

    @property
    def latest_sequence(self) -> int:
        with self._lock:
            return self._sequence

    def publish(
        self,
        event_type: str,
        payload: Optional[Mapping[str, Any]] = None,
        *,
        snapshot: Optional[RuntimeSnapshot] = None,
    ) -> RuntimeEvent:
        with self._lock:
            self._sequence += 1
            event = RuntimeEvent(
                sequence=self._sequence,
                occurred_at=time(),
                event_type=event_type,
                payload=dict(payload or {}),
                snapshot=snapshot,
            )
            self._history.append(event)
            listeners = tuple(self._listeners.values())
        for listener in listeners:
            try:
                listener(event)
            except Exception:
                # Diagnostics must never be able to interrupt a flight command.
                continue
        return event

    def events_since(self, sequence: int) -> tuple[RuntimeEvent, ...]:
        with self._lock:
            return tuple(event for event in self._history if event.sequence > sequence)

    def subscribe(self, listener: EventListener) -> Callable[[], None]:
        with self._lock:
            listener_id = self._next_listener_id
            self._next_listener_id += 1
            self._listeners[listener_id] = listener

        def unsubscribe() -> None:
            with self._lock:
                self._listeners.pop(listener_id, None)

        return unsubscribe
