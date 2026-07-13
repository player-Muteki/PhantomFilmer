"""Shared minimal interface for vision detectors used by follow workflows."""

from typing import Any, Dict, Protocol


DetectionResult = Dict[str, Any]


class DetectorProtocol(Protocol):
    """Detector contract consumed by camera and follow sessions."""

    def detect(self, frame: Any) -> DetectionResult:
        """Return a normalized detection-result dictionary for one frame."""

    def draw_debug(self, frame: Any, result: DetectionResult) -> Any:
        """Return a debug-rendered copy of one frame."""
