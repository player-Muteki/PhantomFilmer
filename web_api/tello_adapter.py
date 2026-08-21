"""Desktop-sidecar configuration for the shared real-device adapter."""

from pathlib import Path

from drone.tello_adapter import TelloDroneAdapter


class RealTelloAdapter(TelloDroneAdapter):
    """Use the backend adapter with flight auditing enabled for desktop sessions."""

    def __init__(self, data_dir: str | Path) -> None:
        audit_dir = Path(data_dir) / "logs" / "flight_events"
        super().__init__(flight_audit_enabled=True, flight_audit_log_dir=audit_dir)
