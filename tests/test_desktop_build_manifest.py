"""Regression checks for the self-contained desktop sidecar bundle."""

from pathlib import Path

from vision.model_assets import (
    JOINTBDOE_MODEL_ASSET,
    PERSON_DETECTOR_ASSET,
    REID_MODEL_ASSET,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_sidecar_bundle_includes_yaml_runtime_configuration() -> None:
    requirements = (PROJECT_ROOT / "requirements-sidecar.txt").read_text(
        encoding="utf-8"
    )
    spec = (PROJECT_ROOT / "sidecar/phantomfilmer_sidecar.spec").read_text(
        encoding="utf-8"
    )

    assert "PyYAML==6.0.3" in requirements
    assert "opencv-python-headless==4.10.0.84" in requirements
    assert "Pillow==12.3.0" in requirements
    assert 'project_root / "config.yaml"' in spec
    assert '"yaml"' in spec


def test_sidecar_bundle_includes_vision_models_and_runtimes() -> None:
    desktop_requirements = (PROJECT_ROOT / "requirements-desktop-build.txt").read_text(
        encoding="utf-8"
    )
    torchreid_requirements = (
        PROJECT_ROOT / "requirements-desktop-torchreid.txt"
    ).read_text(encoding="utf-8")
    spec = (PROJECT_ROOT / "sidecar/phantomfilmer_sidecar.spec").read_text(
        encoding="utf-8"
    )
    builder = (PROJECT_ROOT / "scripts/build_sidecar.py").read_text(encoding="utf-8")
    asset_preparer = (
        PROJECT_ROOT / "scripts/prepare_desktop_model_assets.py"
    ).read_text(encoding="utf-8")

    assert "requirements-reid-bootstrap.txt" in desktop_requirements
    assert "f8cd150fdf77e8d9e1ed143b7f308c2c609ded50" in torchreid_requirements
    for config_key in (
        "person_detector_model",
        "reid_model_path",
        "jointbdoe_model_path",
        "jointbdoe_source_path",
    ):
        assert config_key in spec
    assert 'for runtime_package in ("torchreid",)' in spec
    assert 'collect_submodules("scipy._external.array_api_compat")' in spec
    assert '"seaborn"' in spec
    assert '"--verify-models"' in builder
    assert "DESKTOP_MODEL_ASSETS" in asset_preparer
    assert "PERSON_DETECTOR_ASSET" in (
        PROJECT_ROOT / "vision/model_assets.py"
    ).read_text(encoding="utf-8")
    assert PERSON_DETECTOR_ASSET.relative_path == "weights/yolo26n.pt"
    assert PERSON_DETECTOR_ASSET.sha256 == (
        "9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef"
    )
    assert REID_MODEL_ASSET.sha256 == (
        "cf55163d78fc44c62c82f85ab62d39f10438679b5abe8c698ae08cfa84aa6e18"
    )
    assert JOINTBDOE_MODEL_ASSET.sha256 == (
        "bc6d63ee0f685a888e5ff94a84d8244ce23a817223010e100459137bacae3e27"
    )
