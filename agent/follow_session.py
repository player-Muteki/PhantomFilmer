"""Compatibility exports for the Agent follow session."""

from typing import Dict, Optional

from control.follow_control import FollowController
from control.follow_session import FollowSession, FollowSessionResult
from drone.drone_adapter import DroneAdapter
from drone.safety import SafetyManager
from vision.detector_protocol import DetectorProtocol


class AgentFollowSession(FollowSession):
    """Agent-facing name for the shared follow session."""

    def __init__(
        self,
        drone: DroneAdapter,
        safety_manager: SafetyManager,
        detector: DetectorProtocol,
        follow_controller: FollowController,
        config: Dict[str, object],
        mode_label: str,
        window_name: Optional[str] = None,
        state_label: str = "AGENT",
        allow_pause: bool = True,
    ) -> None:
        super().__init__(
            drone=drone,
            safety_manager=safety_manager,
            detector=detector,
            follow_controller=follow_controller,
            config=config,
            mode_label=mode_label,
            window_name=window_name
            or str(config.get("agent_window_name", "DroneUmbrella Agent Follow")),
            state_label=state_label,
            allow_pause=allow_pause,
        )


AgentFollowSessionResult = FollowSessionResult
