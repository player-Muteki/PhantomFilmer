"""Swarm-level safety rules for multi-drone control."""

from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

from .swarm_node import SwarmDroneNode


RC_Tuple = Tuple[int, int, int, int]


@dataclass
class SwarmSafetyConfig:
    """Safety limits for four-drone operations."""

    minimum_takeoff_battery: int = 30
    max_lr_speed: int = 10
    max_fb_speed: int = 10
    max_ud_speed: int = 10
    max_yaw_speed: int = 10

    @classmethod
    def from_dict(cls, data: dict) -> "SwarmSafetyConfig":
        """Create config from the top-level config.yaml dictionary."""
        swarm = data.get("swarm", {}) if isinstance(data, dict) else {}
        if not isinstance(swarm, dict):
            swarm = {}
        return cls(
            minimum_takeoff_battery=int(swarm.get("minimum_takeoff_battery", 30)),
            max_lr_speed=int(swarm.get("max_lr_speed", 10)),
            max_fb_speed=int(swarm.get("max_fb_speed", 10)),
            max_ud_speed=int(swarm.get("max_ud_speed", 10)),
            max_yaw_speed=int(swarm.get("max_yaw_speed", 10)),
        )


class SwarmSafetyManager:
    """Apply swarm-level preflight and command safety rules."""

    def __init__(self, config: SwarmSafetyConfig) -> None:
        self.config = config
        self.emergency_active = False

    @classmethod
    def from_dict(cls, data: dict) -> "SwarmSafetyManager":
        """Create manager from config.yaml data."""
        return cls(SwarmSafetyConfig.from_dict(data))

    def check_takeoff_allowed(self, nodes: Iterable[SwarmDroneNode]) -> Dict[str, str]:
        """Return per-node takeoff blockers."""
        blockers: Dict[str, str] = {}
        for node in nodes:
            status = node.get_status()
            if not status.connected:
                blockers[node.drone_id] = status.last_error or "node is not connected"
                continue
            if status.battery is None:
                blockers[node.drone_id] = "battery is unknown"
                continue
            if status.battery < self.config.minimum_takeoff_battery:
                blockers[node.drone_id] = (
                    f"battery {status.battery}% below "
                    f"{self.config.minimum_takeoff_battery}% takeoff threshold"
                )
        return blockers

    def allow_nonzero_rc(self, nodes: Iterable[SwarmDroneNode]) -> bool:
        """Return False if emergency or offline nodes block movement."""
        if self.emergency_active:
            return False
        return all(node.connected for node in nodes)

    def limit_rc_command(self, command: RC_Tuple) -> RC_Tuple:
        """Clamp one swarm RC command by channel."""
        left_right, forward_backward, up_down, yaw = command
        return (
            self._limit(left_right, self.config.max_lr_speed),
            self._limit(forward_backward, self.config.max_fb_speed),
            self._limit(up_down, self.config.max_ud_speed),
            self._limit(yaw, self.config.max_yaw_speed),
        )

    def activate_emergency(self) -> None:
        """Block future nonzero commands until a new manager is created."""
        self.emergency_active = True

    @staticmethod
    def is_zero_command(command: RC_Tuple) -> bool:
        """Return True when all RC channels are zero."""
        return tuple(int(value) for value in command) == (0, 0, 0, 0)

    @staticmethod
    def _limit(value: int, limit: int) -> int:
        bound = abs(int(limit))
        return max(-bound, min(bound, int(value)))
