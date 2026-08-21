import unittest

from app.runtime.event_bus import EventBus
from app.runtime.models import ControlMode, MissionKind, RuntimePhase, RuntimeSnapshot


class EventBusTestCase(unittest.TestCase):
    def test_sequences_events_and_replays_only_newer_entries(self) -> None:
        bus = EventBus(history_limit=3)

        first = bus.publish("one", {"value": 1})
        second = bus.publish("two", {"value": 2})
        third = bus.publish("three", {"value": 3})
        fourth = bus.publish("four", {"value": 4})

        self.assertEqual((first.sequence, second.sequence, fourth.sequence), (1, 2, 4))
        self.assertEqual(
            [event.event_type for event in bus.events_since(first.sequence)],
            ["two", "three", "four"],
        )
        self.assertEqual(
            [event.event_type for event in bus.events_since(second.sequence)],
            ["three", "four"],
        )

    def test_listener_failure_cannot_interrupt_other_subscribers(self) -> None:
        bus = EventBus()
        received = []
        bus.subscribe(lambda _event: (_ for _ in ()).throw(RuntimeError("broken")))
        unsubscribe = bus.subscribe(received.append)

        event = bus.publish("safe")
        unsubscribe()
        bus.publish("after-unsubscribe")

        self.assertEqual(received, [event])

    def test_event_stamps_attached_snapshot_with_the_same_sequence(self) -> None:
        bus = EventBus()
        snapshot = RuntimeSnapshot(
            sequence=0,
            phase=RuntimePhase.PREFLIGHT,
            mission=MissionKind.IDLE,
            control_mode=ControlMode.NONE,
            connected=True,
            airborne=False,
            streaming=True,
            flight_state="ready",
        )

        event = bus.publish("state.changed", snapshot=snapshot)

        self.assertEqual(event.sequence, 1)
        self.assertIsNotNone(event.snapshot)
        self.assertEqual(event.snapshot.sequence, event.sequence)
        self.assertEqual(snapshot.sequence, 0)


if __name__ == "__main__":
    unittest.main()
