"""Factory for creating the project's person ReID detector.

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

    """
    from vision.person_reid_detect import PersonReIDDetector

    return PersonReIDDetector.from_config(config)
