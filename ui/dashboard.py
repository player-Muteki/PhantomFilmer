"""Dashboard skeleton for displaying prototype status."""


class Dashboard:
    """Simple placeholder dashboard for future telemetry display."""

    def __init__(self) -> None:
        self.latest_status = {}

    def update_status(self, status: dict) -> None:
        """Store the latest system status values."""
        self.latest_status = dict(status)

    def render_text(self) -> str:
        """Render status as plain text for early testing."""
        if not self.latest_status:
            return "No status available."
        return "\n".join(f"{key}: {value}" for key, value in self.latest_status.items())
