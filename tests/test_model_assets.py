"""Tests for the desktop model asset registry."""

import pytest

from vision.model_assets import (
    PERSON_DETECTOR_ASSET,
    ModelAsset,
    configured_model_path,
    sha256_file,
    verify_desktop_asset,
)


def test_person_detector_registry_uses_the_official_yolo26n_artifact() -> None:
    assert PERSON_DETECTOR_ASSET.relative_path == "weights/yolo26n.pt"
    assert PERSON_DETECTOR_ASSET.url == (
        "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt"
    )
    assert PERSON_DETECTOR_ASSET.sha256 == (
        "9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef"
    )


def test_configured_model_path_requires_an_explicit_value(tmp_path) -> None:
    with pytest.raises(RuntimeError, match="person_detector_model"):
        configured_model_path({"vision": {}}, "person_detector_model", project_root=tmp_path)


def test_verify_desktop_asset_validates_the_pinned_digest(tmp_path) -> None:
    model_path = tmp_path / "weights" / "model.pt"
    model_path.parent.mkdir()
    model_path.write_bytes(b"model-bytes")
    asset = ModelAsset(
        config_key="test_model",
        relative_path="weights/model.pt",
        sha256=sha256_file(model_path),
    )

    assert verify_desktop_asset(asset, project_root=tmp_path) == model_path

    model_path.write_bytes(b"tampered")
    with pytest.raises(RuntimeError, match="模型校验失败"):
        verify_desktop_asset(asset, project_root=tmp_path)
