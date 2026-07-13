"""Factory helpers for real RoboMaster TT swarm nodes."""

from typing import Iterable, List, Optional

from drone.tello_adapter import TelloDroneAdapter

from .fake_swarm import DEFAULT_FAKE_DRONES
from .swarm_node import SwarmDroneNode


def create_real_swarm_nodes(drone_configs: Optional[Iterable[dict]] = None) -> List[SwarmDroneNode]:
    """Create real swarm nodes from config without opening video streams."""
    configs = list(drone_configs or DEFAULT_FAKE_DRONES)
    nodes: List[SwarmDroneNode] = []
    for item in configs:
        drone_id = str(item.get("id"))
        ip = str(item.get("ip", "192.168.10.1"))
        nodes.append(
            SwarmDroneNode(
                drone_id=drone_id,
                ip=ip,
                role=str(item.get("role", "follower")),
                adapter=TelloDroneAdapter(host=ip),
            )
        )
    return nodes
