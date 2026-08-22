import json
from pathlib import Path

from app.runtime.event_bus import EventBus
from app.runtime.event_recorder import RuntimeEventRecorder


def test_recorder_persists_structured_events_without_changing_sequence(tmp_path: Path) -> None:
    bus = EventBus()
    path = tmp_path / "events.jsonl"
    recorder = RuntimeEventRecorder(bus, path)

    first = bus.publish("mission.started", {"mode": "normal"})
    second = bus.publish("flight.behavior", {"action": "hover"})
    recorder.close()

    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert [record["sequence"] for record in records] == [first.sequence, second.sequence]
    assert records[1]["payload"]["action"] == "hover"
