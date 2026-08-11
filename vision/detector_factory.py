"""Factory for creating vision detectors from project configuration.

Usage::

    import yaml
    with open("config.yaml") as f:
        config = yaml.safe_load(f)
    detector = create_detector(config)
    result = detector.detect(frame)
    debug_frame = detector.draw_debug(frame, result)
"""

from typing import Dict

from vision.detector_protocol import DetectorProtocol, DetectionResult


def create_detector(config: Dict) -> DetectorProtocol:
    """Create a vision detector based on configuration.

    Args:
        config: Full config dictionary (e.g. contents of config.yaml).

    Returns:
        A detector instance providing ``detect(frame) -> dict`` and
        ``draw_debug(frame, result) -> frame``.

    Raises:
        ValueError: If *detector_type* is not recognised.
    """
    cfg = config.get("vision", {})
    if not isinstance(cfg, dict):
        cfg = config

    detector_type = str(cfg.get("detector_type", "red")).strip().lower()

    if detector_type == "red":
        from vision.target_detect import TargetDetector
        return TargetDetector.from_config(config)

    if detector_type == "aruco":
        from vision.aruco_detect import ArucoTargetDetector
        return ArucoTargetDetector.from_config(config)

    if detector_type == "person_reid":
        from vision.person_reid_detect import PersonReIDDetector
        return PersonReIDDetector.from_config(config)

    raise ValueError(
        f"Unsupported detector_type: '{detector_type}'. "
        "Supported types: 'red', 'aruco', 'person_reid'."
    )
