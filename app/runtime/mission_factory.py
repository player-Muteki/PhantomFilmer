"""Single construction path for CLI and GUI follow-session missions."""

from __future__ import annotations

from threading import Event
from typing import Optional

from app.runtime.models import MissionKind
from control.fixed_demo import FixedDemoManeuver
from control.follow_control import FollowController
from control.follow_session import FollowSession
from control.motion_arbiter import MotionArbiter
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from control.operator_commands import OperatorCommandChannel
from drone.drone_adapter import DroneAdapter
from drone.safety import SafetyManager
from vision.detector_protocol import DetectorProtocol
from vision.obstacle_detect import DistanceOnlyObstacleDetector


class MissionFactory:
    """Build FollowSession variants from one dependency bundle."""

    def __init__(
        self,
        *,
        drone: DroneAdapter,
        safety_manager: SafetyManager,
        detector: DetectorProtocol,
        follow_controller: FollowController,
        config: dict,
        motion_arbiter: Optional[MotionArbiter] = None,
        obstacle_detector: Optional[DistanceOnlyObstacleDetector] = None,
        obstacle_planner: Optional[ObstacleAvoidancePlanner] = None,
        operator_commands: Optional[OperatorCommandChannel] = None,
        manage_camera_stream: bool = True,
    ) -> None:
        self.drone = drone
        self.safety_manager = safety_manager
        self.detector = detector
        self.follow_controller = follow_controller
        self.config = config
        self.motion_arbiter = motion_arbiter
        self.obstacle_detector = obstacle_detector
        self.obstacle_planner = obstacle_planner
        self.operator_commands = operator_commands
        self.manage_camera_stream = bool(manage_camera_stream)

    def create_follow_session(
        self,
        *,
        mission: MissionKind,
        mode_label: str,
        window_name: str,
        state_label: str = "FOLLOW",
        allow_pause: bool = False,
        stop_event: Optional[Event] = None,
        pre_follow_maneuver: Optional[FixedDemoManeuver] = None,
        initial_target_lock_frames: int = 0,
        enable_target_search: Optional[bool] = None,
    ) -> FollowSession:
        if mission not in {
            MissionKind.FOLLOW,
            MissionKind.REID_FOLLOW,
            MissionKind.FIXED_DEMO,
        }:
            raise ValueError(f"unsupported FollowSession mission: {mission.value}")
        if mission is MissionKind.FIXED_DEMO and pre_follow_maneuver is None:
            raise ValueError("fixed_demo mission requires pre_follow_maneuver")
        return FollowSession(
            drone=self.drone,
            safety_manager=self.safety_manager,
            detector=self.detector,
            follow_controller=self.follow_controller,
            config=self.config,
            mode_label=mode_label,
            window_name=window_name,
            state_label=state_label,
            allow_pause=allow_pause,
            stop_event=stop_event,
            pre_follow_maneuver=pre_follow_maneuver,
            obstacle_detector=self.obstacle_detector,
            obstacle_planner=self.obstacle_planner,
            motion_arbiter=self.motion_arbiter,
            initial_target_lock_frames=initial_target_lock_frames,
            enable_target_search=enable_target_search,
            operator_commands=self.operator_commands,
            manage_camera_stream=self.manage_camera_stream,
        )
