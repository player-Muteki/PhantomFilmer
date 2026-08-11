"""Multi-node swarm manager with structured action results."""

from dataclasses import dataclass, field
from time import monotonic, sleep
from typing import Callable, Dict, Iterable, List, Optional, Tuple, Union

from control.follow_control import RCCommand

from .formation_control import FormationController, FormationCorrection
from .swarm_node import NodeStatus, SwarmDroneNode
from .swarm_safety import RC_Tuple, SwarmSafetyManager


@dataclass
class SwarmActionResult:
    """Result for one node action."""

    drone_id: str
    success: bool
    action: str
    elapsed_ms: float
    status: NodeStatus
    error: Optional[str] = None
    command: Optional[RC_Tuple] = None


@dataclass
class SwarmBatchResult:
    """Structured result for a swarm-wide action."""

    action: str
    elapsed_ms: float
    results: Dict[str, SwarmActionResult] = field(default_factory=dict)

    @property
    def success(self) -> bool:
        """Return True only if every node succeeded."""
        return all(result.success for result in self.results.values())


class SwarmManager:
    """Coordinate connection, status, RC, landing, and emergency handling."""

    def __init__(
        self,
        nodes: Iterable[SwarmDroneNode],
        safety_manager: SwarmSafetyManager,
        formation_controller: Optional[FormationController] = None,
        command_interval_ms: int = 200,
        takeoff_interval_s: float = 1.0,
    ) -> None:
        self.nodes: Dict[str, SwarmDroneNode] = {}
        for node in nodes:
            if node.drone_id in self.nodes:
                raise ValueError(f"duplicate swarm drone id: {node.drone_id}")
            self.nodes[node.drone_id] = node
        self.safety_manager = safety_manager
        self.formation_controller = formation_controller or FormationController()
        self.command_interval_ms = max(0, int(command_interval_ms))
        self.takeoff_interval_s = max(0.0, float(takeoff_interval_s))

    @classmethod
    def from_config(
        cls,
        config: dict,
        nodes: Iterable[SwarmDroneNode],
        require_formation_feedback: Optional[bool] = None,
    ) -> "SwarmManager":
        """Build manager from config.yaml data and supplied nodes."""
        swarm = config.get("swarm", {}) if isinstance(config, dict) else {}
        if not isinstance(swarm, dict):
            swarm = {}
        feedback_required = bool(swarm.get("require_formation_feedback", True))
        if require_formation_feedback is not None:
            feedback_required = bool(require_formation_feedback)
        return cls(
            nodes=nodes,
            safety_manager=SwarmSafetyManager.from_dict(config),
            formation_controller=FormationController(
                require_feedback=feedback_required,
                feedback_timeout_s=float(swarm.get("formation_feedback_timeout_s", 0.5)),
            ),
            command_interval_ms=int(swarm.get("command_interval_ms", 200)),
            takeoff_interval_s=float(swarm.get("takeoff_interval_s", 1.0)),
        )

    def update_formation_feedback(
        self,
        corrections: Dict[str, FormationCorrection],
    ) -> None:
        """Provide fresh per-node corrections from an external position tracker."""
        self.formation_controller.update_feedback(corrections)

    def connect_all(self) -> SwarmBatchResult:
        """Connect every node and keep per-node failures isolated."""
        return self._run_batch("connect_all", lambda node: node.connect())

    def status_all(self) -> SwarmBatchResult:
        """Read status from every node."""
        return self._run_batch("status_all", lambda node: node.get_status())

    def takeoff_sequence(self) -> SwarmBatchResult:
        """Take off sequentially only after swarm safety checks pass."""
        blockers = self.safety_manager.check_takeoff_allowed(self.nodes.values())
        if blockers:
            start = monotonic()
            results: Dict[str, SwarmActionResult] = {}
            for node in self.nodes.values():
                error = blockers.get(node.drone_id)
                status = node.status_snapshot()
                results[node.drone_id] = SwarmActionResult(
                    drone_id=node.drone_id,
                    success=error is None,
                    action="takeoff_sequence",
                    elapsed_ms=0.0,
                    status=status,
                    error=error,
                )
            return SwarmBatchResult("takeoff_sequence", self._elapsed_ms(start), results)
        return self._run_batch(
            "takeoff_sequence",
            lambda node: node.takeoff(),
            interval_s=self.takeoff_interval_s,
        )

    def land_sequence(self) -> SwarmBatchResult:
        """Land every node sequentially."""
        return self._run_batch(
            "land_sequence",
            lambda node: node.land(),
            interval_s=self.takeoff_interval_s,
        )

    def send_rc_all(self, command: Union[RCCommand, RC_Tuple], duration_s: float = 0.0) -> SwarmBatchResult:
        """Send a base command to all nodes after formation and safety checks."""
        base_command = self._as_tuple(command)
        if (
            not self.safety_manager.is_zero_command(base_command)
            and not self.formation_controller.has_fresh_feedback(self.nodes.keys())
        ):
            return self.zero_rc_all(action="send_rc_all_feedback_blocked")
        commands = self.formation_controller.distribute(self.nodes.keys(), base_command)
        nonzero_requested = any(not self.safety_manager.is_zero_command(cmd) for cmd in commands.values())
        if nonzero_requested and not self.safety_manager.allow_nonzero_rc(self.nodes.values()):
            return self.zero_rc_all(action="send_rc_all_blocked")
        result = self._send_commands("send_rc_all", commands)
        if nonzero_requested:
            sleep(max(0.0, float(duration_s)))
            self.zero_rc_all(action="send_rc_all_auto_zero")
        return result

    def send_node_rc(
        self,
        drone_id: str,
        command: Union[RCCommand, RC_Tuple],
        duration_s: float = 0.0,
    ) -> SwarmBatchResult:
        """Send one command to a selected node."""
        if drone_id not in self.nodes:
            raise KeyError(f"unknown swarm drone id: {drone_id}")
        rc_command = self._as_tuple(command)
        if (
            not self.safety_manager.is_zero_command(rc_command)
            and not self.formation_controller.has_fresh_feedback(self.nodes.keys())
        ):
            return self.zero_rc_all(action="send_node_rc_feedback_blocked")
        if not self.safety_manager.is_zero_command(rc_command) and not self.safety_manager.allow_nonzero_rc(
            self.nodes.values()
        ):
            return self.zero_rc_all(action="send_node_rc_blocked")
        result = self._send_commands("send_node_rc", {drone_id: rc_command})
        if not self.safety_manager.is_zero_command(rc_command):
            sleep(max(0.0, float(duration_s)))
            self.zero_rc_all(action="send_node_rc_auto_zero")
        return result

    def zero_rc_all(self, action: str = "zero_rc_all") -> SwarmBatchResult:
        """Send zero velocity to every node."""
        return self._send_commands(action, {drone_id: (0, 0, 0, 0) for drone_id in self.nodes})

    def emergency_stop_all(self) -> SwarmBatchResult:
        """Immediately zero RC and stop every node."""
        self.safety_manager.activate_emergency()
        start = monotonic()
        zero_result = self.zero_rc_all(action="emergency_zero_rc")
        results = dict(zero_result.results)
        for node in self.nodes.values():
            zero_action = results.get(node.drone_id)
            node_start = monotonic()
            status = node.stop()
            elapsed = self._elapsed_ms(node_start)
            errors = []
            if zero_action is not None and zero_action.error:
                errors.append(zero_action.error)
            if status.last_error:
                errors.append(status.last_error)
            error = "; ".join(errors) if errors else None
            results[node.drone_id] = SwarmActionResult(
                drone_id=node.drone_id,
                success=(zero_action is None or zero_action.success) and error is None,
                action="emergency_stop_all",
                elapsed_ms=elapsed,
                status=status,
                error=error,
                command=(0, 0, 0, 0),
            )
        return SwarmBatchResult("emergency_stop_all", self._elapsed_ms(start), results)

    def all_ready(self) -> bool:
        """Return True when all nodes are connected and have enough battery."""
        return not self.safety_manager.check_takeoff_allowed(self.nodes.values())

    def get_failed_nodes(self) -> List[SwarmDroneNode]:
        """Return nodes that are disconnected or have a recorded error."""
        return [node for node in self.nodes.values() if not node.connected or node.last_error]

    def _send_commands(self, action: str, commands: Dict[str, RC_Tuple]) -> SwarmBatchResult:
        start = monotonic()
        results: Dict[str, SwarmActionResult] = {}
        for drone_id, raw_command in commands.items():
            node = self.nodes[drone_id]
            command = self.safety_manager.limit_rc_command(raw_command)
            node_start = monotonic()
            status = node.send_rc(*command)
            elapsed = self._elapsed_ms(node_start)
            error = status.last_error
            results[drone_id] = SwarmActionResult(
                drone_id=drone_id,
                success=error is None and status.connected,
                action=action,
                elapsed_ms=elapsed,
                status=status,
                error=error,
                command=command,
            )
            self._sleep_command_interval()
        return SwarmBatchResult(action, self._elapsed_ms(start), results)

    def _run_batch(
        self,
        action: str,
        operation: Callable[[SwarmDroneNode], NodeStatus],
        interval_s: float = 0.0,
    ) -> SwarmBatchResult:
        start = monotonic()
        results: Dict[str, SwarmActionResult] = {}
        for node in self.nodes.values():
            node_start = monotonic()
            try:
                status = operation(node)
                error = status.last_error
                success = error is None and (status.connected or action == "land_sequence")
            except Exception as exc:
                node.mark_offline(str(exc))
                status = node.status_snapshot()
                error = str(exc)
                success = False
            results[node.drone_id] = SwarmActionResult(
                drone_id=node.drone_id,
                success=success,
                action=action,
                elapsed_ms=self._elapsed_ms(node_start),
                status=status,
                error=error,
            )
            if interval_s > 0:
                sleep(interval_s)
        return SwarmBatchResult(action, self._elapsed_ms(start), results)

    def _sleep_command_interval(self) -> None:
        if self.command_interval_ms > 0:
            sleep(self.command_interval_ms / 1000.0)

    @staticmethod
    def _as_tuple(command: Union[RCCommand, RC_Tuple]) -> RC_Tuple:
        if hasattr(command, "as_tuple"):
            return command.as_tuple()  # type: ignore[return-value]
        left_right, forward_backward, up_down, yaw = command
        return (int(left_right), int(forward_backward), int(up_down), int(yaw))

    @staticmethod
    def _elapsed_ms(start: float) -> float:
        return (monotonic() - start) * 1000.0
