"""Single drone node state and adapter wrapper for swarm control."""

from dataclasses import dataclass
from time import monotonic
from typing import Optional

from drone.drone_adapter import DroneAdapter


@dataclass
class NodeStatus:
    """Snapshot of one swarm node."""

    drone_id: str
    ip: str
    role: str
    connected: bool
    airborne: bool
    battery: Optional[int]
    height: Optional[int]
    last_error: Optional[str]


class SwarmDroneNode:
    """Represent one real or simulated drone in the swarm."""

    def __init__(self, drone_id: str, ip: str, role: str, adapter: DroneAdapter) -> None:
        self.drone_id = drone_id
        self.ip = ip
        self.role = role
        self.adapter = adapter
        self.connected = False
        self.airborne = False
        self.battery: Optional[int] = None
        self.height: Optional[int] = None
        self.last_error: Optional[str] = None
        self.last_seen_at: Optional[float] = None

    def connect(self) -> NodeStatus:
        """Connect the adapter and return the latest status."""
        try:
            self.adapter.connect()
            self.connected = True
            self.last_error = None
            self.last_seen_at = monotonic()
            return self.get_status()
        except Exception as exc:
            self.mark_offline(str(exc))
            return self.status_snapshot()

    def get_status(self) -> NodeStatus:
        """Read battery and height from the adapter."""
        if not self.connected:
            return self.status_snapshot()
        try:
            self.battery = int(self.adapter.get_battery())
            self.height = int(self.adapter.get_height())
            self.last_error = None
            self.last_seen_at = monotonic()
        except Exception as exc:
            self.mark_offline(str(exc))
        return self.status_snapshot()

    def takeoff(self) -> NodeStatus:
        """Command this node to take off."""
        if not self.connected:
            self.last_error = "node is not connected"
            return self.status_snapshot()
        try:
            self.adapter.takeoff()
            self.airborne = True
            self.last_error = None
            self.last_seen_at = monotonic()
            return self.get_status()
        except Exception as exc:
            self.last_error = str(exc)
            return self.status_snapshot()

    def land(self) -> NodeStatus:
        """Command this node to land."""
        if not self.connected:
            self.last_error = "node is not connected"
            return self.status_snapshot()
        try:
            self.adapter.land()
            self.airborne = False
            self.last_error = None
            self.last_seen_at = monotonic()
            return self.get_status()
        except Exception as exc:
            self.last_error = str(exc)
            return self.status_snapshot()

    def send_rc(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> NodeStatus:
        """Send one RC command to this node."""
        if not self.connected:
            self.last_error = "node is not connected"
            return self.status_snapshot()
        try:
            self.adapter.move_rc(left_right, forward_backward, up_down, yaw)
            self.last_error = None
            self.last_seen_at = monotonic()
        except Exception as exc:
            self.mark_offline(str(exc))
        return self.status_snapshot()

    def stop(self) -> NodeStatus:
        """Zero motion, land if needed, and release adapter resources."""
        errors = []
        if self.connected:
            try:
                self.adapter.move_rc(0, 0, 0, 0)
            except Exception as exc:
                errors.append(f"zero RC failed: {exc}")

        if self.airborne:
            try:
                self.adapter.land()
                self.airborne = False
            except Exception as exc:
                errors.append(f"landing failed: {exc}")

        try:
            self.adapter.stop()
        except Exception as exc:
            errors.append(f"adapter stop failed: {exc}")

        self.connected = False
        self.last_error = "; ".join(errors) if errors else None
        return self.status_snapshot()

    def mark_offline(self, reason: str) -> None:
        """Mark the node offline with a reason."""
        self.connected = False
        self.last_error = reason

    def status_snapshot(self) -> NodeStatus:
        """Return local node state without reading the adapter."""
        return NodeStatus(
            drone_id=self.drone_id,
            ip=self.ip,
            role=self.role,
            connected=self.connected,
            airborne=self.airborne,
            battery=self.battery,
            height=self.height,
            last_error=self.last_error,
        )
