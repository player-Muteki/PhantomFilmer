"""Map one base RC command into per-node swarm commands."""

from dataclasses import dataclass
from time import monotonic
from typing import Dict, Iterable, Mapping, Optional, Tuple, Union

from control.follow_control import RCCommand


RC_Tuple = Tuple[int, int, int, int]


@dataclass(frozen=True)
class FormationCorrection:
    """Reserved per-node correction for future position feedback."""

    left_right: int = 0
    forward_backward: int = 0
    up_down: int = 0
    yaw: int = 0

    def as_tuple(self) -> RC_Tuple:
        """Return correction in DroneAdapter.move_rc order."""
        return (self.left_right, self.forward_backward, self.up_down, self.yaw)


class FormationController:
    """Distribute a base command to all nodes with optional corrections."""

    def __init__(
        self,
        corrections: Optional[Mapping[str, FormationCorrection]] = None,
        require_feedback: bool = False,
        feedback_timeout_s: float = 0.5,
    ) -> None:
        self.corrections: Dict[str, FormationCorrection] = dict(corrections or {})
        self.require_feedback = bool(require_feedback)
        self.feedback_timeout_s = max(0.05, float(feedback_timeout_s))
        self._feedback_updated_at: Optional[float] = (
            monotonic() if corrections is not None else None
        )

    def update_feedback(
        self,
        corrections: Mapping[str, FormationCorrection],
    ) -> None:
        """Store a complete set of externally calculated formation corrections."""
        self.corrections = dict(corrections)
        self._feedback_updated_at = monotonic()

    def has_fresh_feedback(self, drone_ids: Iterable[str]) -> bool:
        """Return whether movement has fresh feedback for every requested node."""
        if not self.require_feedback:
            return True
        if self._feedback_updated_at is None:
            return False
        if monotonic() - self._feedback_updated_at > self.feedback_timeout_s:
            return False
        return set(drone_ids).issubset(self.corrections)

    def distribute(self, drone_ids: Iterable[str], base_command: Union[RCCommand, RC_Tuple]) -> Dict[str, RC_Tuple]:
        """Return one command per node."""
        base = self._as_tuple(base_command)
        commands: Dict[str, RC_Tuple] = {}
        for drone_id in drone_ids:
            correction = self.corrections.get(drone_id, FormationCorrection()).as_tuple()
            commands[drone_id] = tuple(base[index] + correction[index] for index in range(4))  # type: ignore[assignment]
        return commands

    @staticmethod
    def _as_tuple(command: Union[RCCommand, RC_Tuple]) -> RC_Tuple:
        if hasattr(command, "as_tuple"):
            return command.as_tuple()  # type: ignore[return-value]
        left_right, forward_backward, up_down, yaw = command
        return (int(left_right), int(forward_backward), int(up_down), int(yaw))
