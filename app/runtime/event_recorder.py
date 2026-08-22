"""Non-blocking JSONL persistence for authoritative runtime events."""

from __future__ import annotations

import json
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from typing import Optional

from app.runtime.event_bus import EventBus, RuntimeEvent


class RuntimeEventRecorder:
    """Persist events on a bounded worker queue without delaying flight commands."""

    def __init__(self, event_bus: EventBus, path: Path, *, queue_size: int = 512) -> None:
        self.path = Path(path)
        self._queue: Queue[Optional[RuntimeEvent]] = Queue(maxsize=max(1, queue_size))
        self._closed = Event()
        self._close_lock = Lock()
        self._unsubscribe = event_bus.subscribe(self._enqueue)
        self._worker = Thread(target=self._run, name="runtime-event-recorder", daemon=True)
        self._worker.start()

    def _enqueue(self, event: RuntimeEvent) -> None:
        if self._closed.is_set():
            return
        try:
            self._queue.put_nowait(event)
        except Full:
            # Diagnostics are intentionally lossy under load; flight control wins.
            return

    def _run(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as output:
                while True:
                    try:
                        event = self._queue.get(timeout=0.25)
                    except Empty:
                        if self._closed.is_set():
                            break
                        continue
                    if event is None:
                        break
                    output.write(json.dumps(event.to_dict(), ensure_ascii=False) + "\n")
                    output.flush()
        except OSError:
            # A diagnostics write failure must never affect a command path.
            return

    def close(self) -> None:
        with self._close_lock:
            if self._closed.is_set():
                return
            self._closed.set()
            self._unsubscribe()
            try:
                self._queue.put_nowait(None)
            except Full:
                try:
                    self._queue.get_nowait()
                except Empty:
                    pass
                try:
                    self._queue.put_nowait(None)
                except Full:
                    pass
        self._worker.join(timeout=1.0)
