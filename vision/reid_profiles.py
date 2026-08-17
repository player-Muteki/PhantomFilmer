"""Persistent, local-only person ReID profile storage."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE_ROOT = PROJECT_ROOT / "data" / "reid_profiles"
PROFILE_SCHEMA_VERSION = 1
PREPROCESSING_VERSION = "yolo-person-crop-rgb-osnet-v1"
EMBEDDING_FILENAME = "embedding.npz"
MANIFEST_FILENAME = "manifest.json"


def validate_profile_name(value: str) -> str:
    """Return a safe single-directory profile name."""
    name = str(value).strip()
    if not name:
        raise RuntimeError("人物档案名不能为空。")
    if len(name) > 64:
        raise RuntimeError("人物档案名不能超过 64 个字符。")
    if name in {".", ".."} or Path(name).name != name:
        raise RuntimeError("人物档案名不能包含路径分隔符。")
    if any(ord(character) < 32 for character in name):
        raise RuntimeError("人物档案名不能包含控制字符。")
    return name


def profile_directory(
    profile_name: str,
    profile_root: Path = DEFAULT_PROFILE_ROOT,
) -> Path:
    """Resolve a profile directory below the local profile root."""
    return Path(profile_root).resolve() / validate_profile_name(profile_name)


def save_reid_profile(
    profile_name: str,
    embedding: Any,
    config: dict[str, object],
    reference_images: Sequence[Path],
    *,
    overwrite: bool = False,
    profile_root: Path = DEFAULT_PROFILE_ROOT,
) -> dict[str, object]:
    """Atomically save a normalized embedding and compatibility manifest."""
    name = validate_profile_name(profile_name)
    directory = profile_directory(name, profile_root)
    manifest_path = directory / MANIFEST_FILENAME
    embedding_path = directory / EMBEDDING_FILENAME
    if (manifest_path.exists() or embedding_path.exists()) and not overwrite:
        raise RuntimeError(
            f"人物档案已存在：{name}。如需替换，请显式使用覆盖选项。"
        )

    normalized = _validated_embedding(embedding)
    model_info = _current_model_info(config)
    photos = [
        {
            "index": index,
            "sha256": _sha256_file(Path(path)),
        }
        for index, path in enumerate(reference_images, start=1)
    ]
    if not photos:
        raise RuntimeError("保存人物档案时至少需要一张参考照片。")

    directory.mkdir(parents=True, exist_ok=True)
    _atomic_save_embedding(embedding_path, normalized)
    manifest: dict[str, object] = {
        "schema_version": PROFILE_SCHEMA_VERSION,
        "profile_name": name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "preprocessing_version": PREPROCESSING_VERSION,
        "embedding_file": EMBEDDING_FILENAME,
        "embedding_sha256": _sha256_file(embedding_path),
        "embedding_dimension": int(normalized.shape[0]),
        "photo_count": len(photos),
        "photos": photos,
        **model_info,
    }
    _atomic_write_json(manifest_path, manifest)
    return manifest


def load_reid_profile(
    profile_name: str,
    config: dict[str, object],
    *,
    profile_root: Path = DEFAULT_PROFILE_ROOT,
) -> tuple[np.ndarray, dict[str, object]]:
    """Load a profile only when its files and model fingerprints are valid."""
    name = validate_profile_name(profile_name)
    directory = profile_directory(name, profile_root)
    manifest_path = directory / MANIFEST_FILENAME
    embedding_path = directory / EMBEDDING_FILENAME
    if not manifest_path.is_file() or not embedding_path.is_file():
        raise RuntimeError(f"人物档案不存在或不完整：{name}")

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"人物档案清单损坏：{name}") from exc
    if not isinstance(manifest, dict):
        raise RuntimeError(f"人物档案清单格式无效：{name}")
    if manifest.get("schema_version") != PROFILE_SCHEMA_VERSION:
        raise RuntimeError(f"人物档案版本不兼容，请重新注册：{name}")
    if manifest.get("profile_name") != name:
        raise RuntimeError(f"人物档案名称不匹配：{name}")
    if manifest.get("preprocessing_version") != PREPROCESSING_VERSION:
        raise RuntimeError(f"人物档案预处理版本已变化，请重新注册：{name}")

    current_model_info = _current_model_info(config)
    for key in (
        "reid_model_name",
        "reid_model_sha256",
        "person_detector_model_sha256",
    ):
        if manifest.get(key) != current_model_info[key]:
            raise RuntimeError(f"人物档案与当前模型不兼容，请重新注册：{name}")
    if manifest.get("embedding_sha256") != _sha256_file(embedding_path):
        raise RuntimeError(f"人物档案特征文件校验失败：{name}")

    try:
        with np.load(embedding_path, allow_pickle=False) as values:
            embedding = np.asarray(values["embedding"], dtype=np.float32)
    except (OSError, ValueError, KeyError) as exc:
        raise RuntimeError(f"人物档案特征文件损坏：{name}") from exc
    normalized = _validated_embedding(embedding)
    if manifest.get("embedding_dimension") != int(normalized.shape[0]):
        raise RuntimeError(f"人物档案特征维度不匹配：{name}")
    return normalized, manifest


def _current_model_info(config: dict[str, object]) -> dict[str, str]:
    vision = config.get("vision", {})
    cfg = vision if isinstance(vision, dict) else config
    reid_model_path = _resolve_project_path(
        str(cfg.get("reid_model_path", "weights/osnet_x0_25_msmt17.pth"))
    )
    detector_model_path = _resolve_project_path(
        str(cfg.get("person_detector_model", "weights/yolo11n.pt"))
    )
    for label, path in (
        ("ReID", reid_model_path),
        ("YOLO", detector_model_path),
    ):
        if not path.is_file():
            raise RuntimeError(f"{label} 权重不存在：{path}")
    return {
        "reid_model_name": str(cfg.get("reid_model_name", "osnet_x0_25")),
        "reid_model_sha256": _sha256_file(reid_model_path),
        "person_detector_model_sha256": _sha256_file(detector_model_path),
    }


def _validated_embedding(value: Any) -> np.ndarray:
    values = np.asarray(value, dtype=np.float32)
    if values.ndim == 2 and values.shape[0] == 1:
        values = values[0]
    if values.ndim != 1 or values.size == 0:
        raise RuntimeError("人物档案特征必须是一维非空向量。")
    if not np.all(np.isfinite(values)):
        raise RuntimeError("人物档案特征包含无效数值。")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise RuntimeError("人物档案特征是零向量。")
    return (values / norm).astype(np.float32)


def _resolve_project_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else PROJECT_ROOT / path


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise RuntimeError(f"无法读取文件进行校验：{path}") from exc
    return digest.hexdigest()


def _atomic_save_embedding(path: Path, embedding: np.ndarray) -> None:
    descriptor, raw_temp_path = tempfile.mkstemp(
        prefix=".embedding-", suffix=".npz", dir=path.parent
    )
    temp_path = Path(raw_temp_path)
    try:
        with os.fdopen(descriptor, "wb") as file:
            np.savez_compressed(file, embedding=embedding)
        os.replace(temp_path, path)
    finally:
        if temp_path.exists():
            temp_path.unlink()


def _atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    descriptor, raw_temp_path = tempfile.mkstemp(
        prefix=".manifest-", suffix=".json", dir=path.parent
    )
    temp_path = Path(raw_temp_path)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as file:
            json.dump(payload, file, ensure_ascii=False, indent=2, sort_keys=True)
            file.write("\n")
        os.replace(temp_path, path)
    finally:
        if temp_path.exists():
            temp_path.unlink()
