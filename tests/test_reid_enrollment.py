"""Tests for on-site ReID enrollment and grounded target authorization."""

from pathlib import Path
from typing import Callable

import numpy as np
import pytest

from control.follow_control import FollowController
from control.follow_session import FollowSession
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager
from vision.reid_enrollment import (
    TargetLockTracker,
    build_reid_runtime_config,
    collect_reference_images,
    validate_reference_directory,
    validate_reference_images,
)


class SequenceDetector:
    """Return predefined results while satisfying the detector protocol."""

    def __init__(self, results: list[dict[str, object]]) -> None:
        self.results = results
        self.index = 0
        self.prepare_calls = 0

    def prepare(self) -> None:
        self.prepare_calls += 1

    def reset(self) -> None:
        self.index = 0

    def detect(self, frame: object) -> dict[str, object]:
        result = self.results[min(self.index, len(self.results) - 1)]
        self.index += 1
        return dict(result)

    def draw_debug(self, frame: object, result: dict[str, object]) -> object:
        return frame


class GroundPreviewSession(FollowSession):
    """Use synthetic frames and stop immediately after takeoff."""

    def _start_camera(self) -> None:
        self.streaming = True

    def _read_frame(self) -> np.ndarray:
        return np.zeros((80, 100, 3), dtype=np.uint8)

    def _loop(self) -> None:
        self.session_state = "STOPPED"


class RecordingFakeDrone(FakeDroneAdapter):
    """Record whether grounded authorization reached the takeoff command."""

    def __init__(self) -> None:
        super().__init__(verbose_rc=False)
        self.takeoff_calls = 0

    def takeoff(self) -> None:
        self.takeoff_calls += 1
        super().takeoff()


def build_ground_session(
    detector: SequenceDetector,
    confirmation: Callable[[dict[str, object]], bool],
    required_frames: int = 2,
) -> GroundPreviewSession:
    """Construct a no-window session for grounded-lock tests."""
    safety = SafetyManager(SafetyConfig(30, 20, 150, 60, 35, 3, 8))
    return GroundPreviewSession(
        drone=RecordingFakeDrone(),
        safety_manager=safety,
        detector=detector,
        follow_controller=FollowController(safety_manager=safety),
        config={"display_console_camera": False, "control_interval": 0.02},
        mode_label="TEST",
        initial_target_lock_frames=required_frames,
        initial_target_lock_timeout_seconds=1.0,
        pre_takeoff_confirmation=confirmation,
    )


def test_lock_tracker_requires_fresh_unambiguous_consecutive_matches() -> None:
    tracker = TargetLockTracker(required_frames=2)
    assert not tracker.observe({"found": True, "is_predicted": False, "ambiguous": False})
    assert tracker.progress == "1/2"
    assert not tracker.observe({"found": True, "is_predicted": True, "ambiguous": False})
    assert tracker.progress == "0/2"
    assert not tracker.observe({"found": True, "is_predicted": False, "ambiguous": True})
    assert not tracker.observe({"found": True, "is_predicted": False, "ambiguous": False})
    assert tracker.observe({"found": True, "is_predicted": False, "ambiguous": False})


def test_runtime_config_enables_reid_without_mutating_project_default(tmp_path: Path) -> None:
    photo = tmp_path / "person.jpg"
    photo.write_bytes(b"test")
    original = {"vision": {"detector_type": "aruco", "reid_device": "cpu"}}

    runtime = build_reid_runtime_config(original, [photo])

    assert runtime["vision"]["detector_type"] == "person_reid"
    assert runtime["vision"]["reference_images"] == [str(photo)]
    assert original["vision"]["detector_type"] == "aruco"
    assert "reference_images" not in original["vision"]


def test_validate_reference_images_supports_repeated_and_comma_values(
    tmp_path: Path,
) -> None:
    first = tmp_path / "front view.jpg"
    second = tmp_path / "side.jpg"
    first.write_bytes(b"first")
    second.write_bytes(b"second")

    assert validate_reference_images([f'"{first}"', f"{second},{first}"]) == [
        first.resolve(),
        second.resolve(),
        first.resolve(),
    ]


def test_validate_reference_images_rejects_missing_file(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="不存在"):
        validate_reference_images([str(tmp_path / "missing.jpg")])


def test_validate_reference_directory_selects_supported_images(tmp_path: Path) -> None:
    (tmp_path / "front.JPG").write_bytes(b"front")
    (tmp_path / "side.png").write_bytes(b"side")
    (tmp_path / "ignored.HEIC").write_bytes(b"heic")
    (tmp_path / "notes.txt").write_text("notes")

    assert validate_reference_directory(str(tmp_path)) == [
        (tmp_path / "front.JPG").resolve(),
        (tmp_path / "side.png").resolve(),
    ]


def test_collection_rejects_mixed_path_and_camera_modes(tmp_path: Path) -> None:
    photo = tmp_path / "person.jpg"
    photo.write_bytes(b"test")
    with pytest.raises(RuntimeError, match="不能同时"):
        collect_reference_images([str(photo)], True, 0, 3)


def test_human_rejection_keeps_aircraft_grounded(monkeypatch) -> None:
    detector = SequenceDetector(
        [{"found": True, "is_predicted": False, "ambiguous": False, "similarity": 0.9}]
    )
    session = build_ground_session(detector, confirmation=lambda result: False)
    monkeypatch.setattr("control.follow_session.sleep", lambda seconds: None)

    result = session.run()

    assert result.state == "TAKEOFF_CANCELLED"
    assert session.drone.height_cm == 0
    assert session.drone.takeoff_calls == 0
    assert detector.prepare_calls == 1


def test_stable_lock_and_human_confirmation_allow_takeoff(monkeypatch) -> None:
    detector = SequenceDetector(
        [{"found": True, "is_predicted": False, "ambiguous": False, "similarity": 0.91}]
    )
    session = build_ground_session(detector, confirmation=lambda result: True)
    monkeypatch.setattr("control.follow_session.sleep", lambda seconds: None)

    result = session.run()

    assert result.state == "STOPPED"
    assert detector.index >= 2
    assert detector.prepare_calls == 1
    assert session.drone.height_cm == 0
    assert session.drone.takeoff_calls == 1


def test_target_must_still_be_present_after_human_confirmation(monkeypatch) -> None:
    detector = SequenceDetector(
        [
            {"found": True, "is_predicted": False, "ambiguous": False},
            {"found": True, "is_predicted": False, "ambiguous": False},
            {"found": False, "is_predicted": False, "ambiguous": False},
        ]
    )
    session = build_ground_session(detector, confirmation=lambda result: True)
    monkeypatch.setattr("control.follow_session.sleep", lambda seconds: None)

    result = session.run()

    assert result.state == "TARGET_LOCK_FAILED"
    assert session.drone.takeoff_calls == 0
