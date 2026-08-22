"""Immutable desktop vision-model metadata and verification helpers."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class ModelAsset:
    """One model embedded in an offline desktop release."""

    config_key: str
    relative_path: str
    sha256: str
    url: str | None = None
    google_drive_id: str | None = None


YOLO26N_SHA256 = "9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef"

PERSON_DETECTOR_ASSET = ModelAsset(
    config_key="person_detector_model",
    relative_path="weights/yolo26n.pt",
    url="https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt",
    sha256=YOLO26N_SHA256,
)
REID_MODEL_ASSET = ModelAsset(
    config_key="reid_model_path",
    relative_path="weights/osnet_x0_25_msmt17.pth",
    google_drive_id="1Kkx2zW89jq_NETu4u42CFZTMVD5Hwm6e",
    sha256="cf55163d78fc44c62c82f85ab62d39f10438679b5abe8c698ae08cfa84aa6e18",
)
JOINTBDOE_MODEL_ASSET = ModelAsset(
    config_key="jointbdoe_model_path",
    relative_path="weights/jointbdoe_s.pt",
    url="https://huggingface.co/HoyerChou/JointBDOE/resolve/main/coco_s_1024_e500_t010_w005_best.pt",
    sha256="bc6d63ee0f685a888e5ff94a84d8244ce23a817223010e100459137bacae3e27",
)
DESKTOP_MODEL_ASSETS = (
    PERSON_DETECTOR_ASSET,
    REID_MODEL_ASSET,
    JOINTBDOE_MODEL_ASSET,
)


def configured_model_path(
    config: dict[str, object],
    config_key: str,
    *,
    project_root: Path = PROJECT_ROOT,
) -> Path:
    """Resolve a required model path from the vision configuration."""

    vision = config.get("vision", {})
    cfg = vision if isinstance(vision, dict) else config
    value = str(cfg.get(config_key, "")).strip()
    if not value:
        raise RuntimeError(f"未配置 vision.{config_key}。")
    return resolve_model_path(value, project_root=project_root)


def resolve_model_path(value: str, *, project_root: Path = PROJECT_ROOT) -> Path:
    """Resolve an absolute path or a project-relative model path."""

    path = Path(value).expanduser()
    return path if path.is_absolute() else Path(project_root) / path


def sha256_file(path: Path) -> str:
    """Return a streaming SHA-256 digest for a model or profile file."""

    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise RuntimeError(f"无法读取文件进行校验：{path}") from exc
    return digest.hexdigest()


def verify_desktop_asset(
    asset: ModelAsset,
    *,
    project_root: Path = PROJECT_ROOT,
) -> Path:
    """Verify an expected offline release asset and return its path."""

    path = Path(project_root) / asset.relative_path
    if not path.is_file():
        raise RuntimeError(f"缺少桌面端模型资源：{path}")
    actual = sha256_file(path)
    if actual != asset.sha256:
        raise RuntimeError(
            f"模型校验失败：{asset.relative_path}，实际 SHA-256={actual}"
        )
    return path


def desktop_asset_issues(
    config: dict[str, object],
    *,
    project_root: Path = PROJECT_ROOT,
) -> list[str]:
    """Return missing configured desktop assets without loading model runtimes."""

    issues: list[str] = []
    for asset in DESKTOP_MODEL_ASSETS:
        try:
            path = configured_model_path(config, asset.config_key, project_root=project_root)
        except RuntimeError:
            issues.append(asset.config_key)
            continue
        if not path.is_file():
            issues.append(asset.config_key)
    return issues


def asset_manifest() -> tuple[ModelAsset, ...]:
    """Expose the single immutable desktop asset list to build tooling."""

    return DESKTOP_MODEL_ASSETS
