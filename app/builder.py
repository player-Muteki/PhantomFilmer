"""Dependency wiring for PhantomFilmer runtime systems."""

from typing import Optional

from app.config import load_config, load_runtime_config, selected_detector_type
from app.trace import is_trace_enabled, trace_drone
from console.command_parser import CommandParser
from console.console_controller import ConsoleController
from console.llm_client import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    DEFAULT_TIMEOUT_SECONDS,
    LLMClient,
)
from console.tools import ConsoleTools
from control.follow_control import FollowController
from control.motion_arbiter import MotionArbiter
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from drone.drone_adapter import DroneAdapter
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager
from drone.tello_adapter import TelloDroneAdapter
from vision.detector_factory import create_detector
from vision.obstacle_detect import ObstacleDetector


def build_safety_manager() -> SafetyManager:
    """Create the safety manager from config.yaml."""
    return SafetyManager.from_dict(load_config())


def build_obstacle_modules(
    config: dict,
    safety_manager: SafetyManager,
) -> tuple[
    Optional[ObstacleDetector],
    Optional[ObstacleAvoidancePlanner],
    Optional[MotionArbiter],
]:
    """Create obstacle modules only when enabled in config.yaml."""
    obstacle_config = config.get("obstacle", {})
    if not isinstance(obstacle_config, dict) or not bool(obstacle_config.get("enabled", False)):
        return None, None, None
    detector = ObstacleDetector.from_config(config)
    planner = ObstacleAvoidancePlanner.from_config(
        safety_manager=safety_manager,
        config=config,
    )
    return detector, planner, MotionArbiter(detector=detector, planner=planner, config=config)


def create_drone_adapter(
    use_fake: bool,
    verbose_fake_rc: bool = True,
    config: Optional[dict] = None,
) -> DroneAdapter:
    """Create either the fake drone adapter or the real Tello adapter."""
    if use_fake:
        config = config or {}
        vision_config = config.get("vision", {})
        if not isinstance(vision_config, dict):
            vision_config = {}
        drone: DroneAdapter = FakeDroneAdapter(
            verbose_rc=verbose_fake_rc,
            camera_width=int(config.get("camera_width", 640)),
            camera_height=int(config.get("camera_height", 480)),
            target_speed=int(config.get("fake_target_speed", 3)),
            target_lost_interval_seconds=float(
                config.get("fake_target_lost_interval_seconds", 12)
            ),
            target_lost_duration_seconds=float(
                config.get("fake_target_lost_duration_seconds", 2)
            ),
            takeoff_height_cm=int(config.get("base_hover_height_cm", 70)),
            detector_type=selected_detector_type(config),
            aruco_dictionary=str(
                vision_config.get("aruco_dictionary", "DICT_4X4_50")
            ),
            target_marker_id=int(vision_config.get("target_marker_id", 23)),
        )
    else:
        drone = TelloDroneAdapter()
    if is_trace_enabled():
        return trace_drone(drone)
    return drone


def build_system(
    use_fake: bool = False,
    obstacle_enabled: Optional[bool] = None,
) -> ConsoleController:
    """Create the natural-language Console with safety-wrapped tools."""
    config = load_runtime_config(obstacle_enabled)
    safety_manager = SafetyManager(SafetyConfig.from_dict(config))
    detector = create_detector(config)
    follow_controller = FollowController.from_config(
        safety_manager=safety_manager,
        config=config,
    )
    _, _, motion_arbiter = build_obstacle_modules(
        config,
        safety_manager,
    )
    tools = ConsoleTools(
        drone=create_drone_adapter(use_fake, verbose_fake_rc=False, config=config),
        safety_manager=safety_manager,
        detector=detector,
        follow_controller=follow_controller,
        # 生产装配统一使用 MotionArbiter 作为避障管线；原始检测器/规划器保留在
        # ConsoleTools 参数中仅作直接装配（测试）时的回退，这里不重复传入。
        motion_arbiter=motion_arbiter,
        config=config,
        mode_label="FAKE" if use_fake else "REAL",
        frame_width=int(config.get("camera_width", 640)),
        frame_height=int(config.get("camera_height", 480)),
    )
    llm_client = LLMClient(
        base_url=str(config.get("llm_base_url", DEFAULT_BASE_URL)),
        model=str(config.get("llm_model", DEFAULT_MODEL)),
        timeout_seconds=float(config.get("llm_timeout_seconds", DEFAULT_TIMEOUT_SECONDS)),
        enabled=bool(config.get("llm_enabled", False)),
    )
    parser = CommandParser(llm_client=llm_client)
    return ConsoleController(tools=tools, parser=parser, llm_client=llm_client)
