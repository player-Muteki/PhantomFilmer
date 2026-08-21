"""Tests for persistent local ReID profiles."""

import json
from pathlib import Path

import numpy as np
import pytest

from vision.reid_enrollment import build_reid_runtime_config
from vision.reid_profiles import (
    list_reid_profiles,
    load_reid_profile,
    save_reid_profile,
    validate_profile_name,
)


def build_config(tmp_path: Path) -> dict[str, object]:
    yolo = tmp_path / "yolo11n.pt"
    osnet = tmp_path / "osnet.pth"
    yolo.write_bytes(b"yolo-v1")
    osnet.write_bytes(b"osnet-v1")
    return {
        "vision": {
            "person_detector_model": str(yolo),
            "reid_model_name": "osnet_x0_25",
            "reid_model_path": str(osnet),
        }
    }


def test_profile_round_trip_is_normalized_and_pickle_free(tmp_path: Path) -> None:
    config = build_config(tmp_path)
    photo = tmp_path / "front.jpg"
    photo.write_bytes(b"photo")
    profile_root = tmp_path / "profiles"

    manifest = save_reid_profile(
        "person-a-current-outfit",
        np.array([3.0, 4.0], dtype=np.float32),
        config,
        [photo],
        profile_root=profile_root,
    )
    embedding, loaded_manifest = load_reid_profile(
        "person-a-current-outfit", config, profile_root=profile_root
    )

    assert np.allclose(embedding, np.array([0.6, 0.8], dtype=np.float32))
    assert manifest == loaded_manifest
    assert loaded_manifest["photo_count"] == 1
    assert loaded_manifest["embedding_dimension"] == 2


def test_profile_rejects_changed_model_weight(tmp_path: Path) -> None:
    config = build_config(tmp_path)
    photo = tmp_path / "front.jpg"
    photo.write_bytes(b"photo")
    profile_root = tmp_path / "profiles"
    save_reid_profile("person-a", [1.0, 0.0], config, [photo], profile_root=profile_root)

    Path(config["vision"]["reid_model_path"]).write_bytes(b"osnet-v2")

    with pytest.raises(RuntimeError, match="模型不兼容"):
        load_reid_profile("person-a", config, profile_root=profile_root)


def test_profile_rejects_tampered_embedding(tmp_path: Path) -> None:
    config = build_config(tmp_path)
    photo = tmp_path / "front.jpg"
    photo.write_bytes(b"photo")
    profile_root = tmp_path / "profiles"
    save_reid_profile("person-a", [1.0, 0.0], config, [photo], profile_root=profile_root)
    (profile_root / "person-a" / "embedding.npz").write_bytes(b"tampered")

    with pytest.raises(RuntimeError, match="校验失败"):
        load_reid_profile("person-a", config, profile_root=profile_root)


def test_profile_requires_explicit_overwrite(tmp_path: Path) -> None:
    config = build_config(tmp_path)
    photo = tmp_path / "front.jpg"
    photo.write_bytes(b"photo")
    profile_root = tmp_path / "profiles"
    save_reid_profile("person-a", [1.0, 0.0], config, [photo], profile_root=profile_root)

    with pytest.raises(RuntimeError, match="已存在"):
        save_reid_profile("person-a", [0.0, 1.0], config, [photo], profile_root=profile_root)

    save_reid_profile(
        "person-a",
        [0.0, 1.0],
        config,
        [photo],
        overwrite=True,
        profile_root=profile_root,
    )
    embedding, _manifest = load_reid_profile("person-a", config, profile_root=profile_root)
    assert np.allclose(embedding, [0.0, 1.0])


def test_profile_name_rejects_path_traversal() -> None:
    with pytest.raises(RuntimeError, match="路径分隔符"):
        validate_profile_name("../person-a")


def test_runtime_config_selects_profile_without_mutating_default() -> None:
    original = {
        "vision": {
            "reference_images": ["old.jpg"],
            "reid_device": "cpu",
        }
    }

    runtime = build_reid_runtime_config(original, profile_name="person-a")

    assert runtime["vision"]["reference_profile"] == "person-a"
    assert "reference_images" not in runtime["vision"]
    assert original["vision"]["reid_device"] == "cpu"


def test_manifest_is_plain_json_without_source_paths_or_filenames(tmp_path: Path) -> None:
    config = build_config(tmp_path)
    photo = tmp_path / "private" / "front.jpg"
    photo.parent.mkdir()
    photo.write_bytes(b"photo")
    profile_root = tmp_path / "profiles"
    save_reid_profile("person-a", [1.0, 0.0], config, [photo], profile_root=profile_root)

    manifest_text = (profile_root / "person-a" / "manifest.json").read_text()
    manifest = json.loads(manifest_text)
    assert str(photo.parent) not in manifest_text
    assert "front.jpg" not in manifest_text
    assert manifest["photos"][0]["index"] == 1


def test_profile_listing_returns_safe_summary_and_skips_corrupt_entries(tmp_path: Path) -> None:
    config = build_config(tmp_path)
    photo = tmp_path / "front.jpg"
    photo.write_bytes(b"photo")
    profile_root = tmp_path / "profiles"
    save_reid_profile("person-a", [1.0, 0.0], config, [photo], profile_root=profile_root)
    corrupt = profile_root / "corrupt"
    corrupt.mkdir()
    (corrupt / "manifest.json").write_text("not-json", encoding="utf-8")

    profiles = list_reid_profiles(profile_root)

    assert [profile["name"] for profile in profiles] == ["person-a"]
    assert profiles[0]["photoCount"] == 1
