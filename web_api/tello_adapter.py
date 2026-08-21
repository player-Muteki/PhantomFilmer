"""WebUI configuration for the shared real-device adapter."""

from drone.tello_adapter import TelloDroneAdapter


class RealTelloAdapter(TelloDroneAdapter):
    """Use the backend adapter with flight auditing enabled for WebUI sessions."""

    def __init__(self) -> None:
        super().__init__(flight_audit_enabled=True)
