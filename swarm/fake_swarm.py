"""Factory helpers for four-node fake swarm validation."""

from typing import Dict, Iterable, List, Optional

from drone.fake_adapter import FakeDroneAdapter

from .swarm_node import SwarmDroneNode


DEFAULT_FAKE_DRONES = (
    {"id": "drone_1", "ip": "192.168.1.101", "role": "leader"},
    {"id": "drone_2", "ip": "192.168.1.102", "role": "follower"},
    {"id": "drone_3", "ip": "192.168.1.103", "role": "follower"},
    {"id": "drone_4", "ip": "192.168.1.104", "role": "follower"},
)


class FailableFakeDroneAdapter(FakeDroneAdapter):
    """Fake adapter that can simulate connection loss in tests."""

    def __init__(self, fail_connect: bool = False, fail_rc: bool = False, **kwargs) -> None:
        super().__init__(**kwargs)
        self.fail_connect = fail_connect
        self.fail_rc = fail_rc

    def connect(self) -> None:
        if self.fail_connect:
            raise RuntimeError("simulated connect failure")
        super().connect()

    def move_rc(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> None:
        if self.fail_rc:
            raise RuntimeError("simulated rc failure")
        super().move_rc(left_right, forward_backward, up_down, yaw)


def create_fake_swarm_nodes(
    drone_configs: Optional[Iterable[dict]] = None,
    failing_ids: Optional[Iterable[str]] = None,
    rc_failing_ids: Optional[Iterable[str]] = None,
    verbose_rc: bool = False,
) -> List[SwarmDroneNode]:
    """Create fake nodes from swarm config."""
    failing = set(failing_ids or ())
    rc_failing = set(rc_failing_ids or ())
    configs = list(drone_configs or DEFAULT_FAKE_DRONES)
    nodes: List[SwarmDroneNode] = []
    for item in configs:
        drone_id = str(item.get("id"))
        adapter = FailableFakeDroneAdapter(
            fail_connect=drone_id in failing,
            fail_rc=drone_id in rc_failing,
            verbose_rc=verbose_rc,
        )
        nodes.append(
            SwarmDroneNode(
                drone_id=drone_id,
                ip=str(item.get("ip", "")),
                role=str(item.get("role", "follower")),
                adapter=adapter,
            )
        )
    return nodes


def default_swarm_config() -> Dict[str, object]:
    """Return a default fake swarm config block."""
    return {
        "enabled": False,
        "command_interval_ms": 200,
        "takeoff_interval_s": 1.0,
        "max_lr_speed": 10,
        "max_fb_speed": 10,
        "max_ud_speed": 10,
        "max_yaw_speed": 10,
        "minimum_takeoff_battery": 30,
        "leader_id": "drone_1",
        "video_drone_id": "drone_1",
        "drones": [dict(item) for item in DEFAULT_FAKE_DRONES],
    }
