"""Persistent, local-only person ReID profile storage."""

from __future__ import annotations

import json
import os
import tempfile
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence
from uuid import uuid4

import numpy as np

from vision.model_assets import configured_model_path, sha256_file


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE_ROOT = PROJECT_ROOT / "data" / "reid_profiles"
PROFILE_SCHEMA_VERSION = 2
SUPPORTED_PROFILE_SCHEMA_VERSIONS = {1, PROFILE_SCHEMA_VERSION}
PREPROCESSING_VERSION = "yolo-person-crop-rgb-osnet-v1"
EMBEDDING_FILENAME = "embedding.npz"
MANIFEST_FILENAME = "manifest.json"
TRASH_DIRECTORY = ".trash"


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


def list_reid_profiles(
    profile_root: Path = DEFAULT_PROFILE_ROOT,
    *,
    config: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    """List readable local manifests without loading model weights or embeddings."""

    root = Path(profile_root).resolve()
    if not root.is_dir():
        return []
    profiles: list[dict[str, object]] = []
    for directory in sorted(root.iterdir(), key=lambda path: path.name.casefold()):
        manifest_path = directory / MANIFEST_FILENAME
        if not directory.is_dir() or not manifest_path.is_file():
            continue
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if (
            not isinstance(manifest, dict)
            or manifest.get("schema_version") not in SUPPORTED_PROFILE_SCHEMA_VERSIONS
        ):
            continue
        try:
            name = validate_profile_name(str(manifest.get("profile_name", "")))
        except RuntimeError:
            continue
        if name != directory.name:
            continue
        profiles.append(_public_profile(manifest, config=config, include_photos=False))
    return profiles


def get_reid_profile(
    profile_name: str,
    profile_root: Path = DEFAULT_PROFILE_ROOT,
    *,
    config: dict[str, object] | None = None,
) -> dict[str, object]:
    """Return one validated, privacy-safe profile manifest."""

    name = validate_profile_name(profile_name)
    directory = profile_directory(name, profile_root)
    manifest = _read_valid_manifest(directory, expected_name=name)
    return _public_profile(manifest, config=config)


def rename_reid_profile(
    profile_name: str,
    new_name: str,
    profile_root: Path = DEFAULT_PROFILE_ROOT,
    *,
    config: dict[str, object] | None = None,
) -> dict[str, object]:
    """Rename a complete profile and roll back if its manifest cannot be updated."""

    old_name = validate_profile_name(profile_name)
    replacement = validate_profile_name(new_name)
    if old_name == replacement:
        return get_reid_profile(old_name, profile_root, config=config)
    old_directory = profile_directory(old_name, profile_root)
    new_directory = profile_directory(replacement, profile_root)
    manifest = _read_valid_manifest(old_directory, expected_name=old_name)
    if new_directory.exists():
        raise RuntimeError(f"人物档案已存在：{replacement}")

    updated = deepcopy(manifest)
    updated["profile_name"] = replacement
    updated["updated_at"] = datetime.now(timezone.utc).isoformat()
    try:
        os.replace(old_directory, new_directory)
        _atomic_write_json(new_directory / MANIFEST_FILENAME, updated)
    except Exception:
        if new_directory.exists() and not old_directory.exists():
            os.replace(new_directory, old_directory)
        raise
    return _public_profile(updated, config=config)


def delete_reid_profile(
    profile_name: str,
    profile_root: Path = DEFAULT_PROFILE_ROOT,
) -> dict[str, object]:
    """Recoverably delete a complete profile by moving it below the local trash."""

    name = validate_profile_name(profile_name)
    root = Path(profile_root).resolve()
    directory = profile_directory(name, root)
    manifest = _read_valid_manifest(directory, expected_name=name)
    trash = root / TRASH_DIRECTORY
    trash.mkdir(parents=True, exist_ok=True)
    tombstone = f"{name}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')}-{uuid4().hex[:8]}"
    destination = trash / tombstone
    os.replace(directory, destination)
    return {
        "name": name,
        "deletedAt": datetime.now(timezone.utc).isoformat(),
        "recoverable": True,
        "photoCount": manifest.get("photo_count"),
    }


def save_reid_profile(
    profile_name: str,
    embeddings: Any,
    config: dict[str, object],
    reference_images: Sequence[Path],
    *,
    overwrite: bool = False,
    profile_root: Path = DEFAULT_PROFILE_ROOT,
) -> dict[str, object]:
    """Atomically save normalized per-photo templates and their centroid."""
    name = validate_profile_name(profile_name)
    directory = profile_directory(name, profile_root)
    manifest_path = directory / MANIFEST_FILENAME
    embedding_path = directory / EMBEDDING_FILENAME
    if (manifest_path.exists() or embedding_path.exists()) and not overwrite:
        raise RuntimeError(
            f"人物档案已存在：{name}。如需替换，请显式使用覆盖选项。"
        )

    templates = _validated_embeddings(embeddings)
    centroid = _normalized_centroid(templates)
    model_info = _current_model_info(config)
    photos = [
        {
            "index": index,
            "sha256": sha256_file(Path(path)),
        }
        for index, path in enumerate(reference_images, start=1)
    ]
    if not photos:
        raise RuntimeError("保存人物档案时至少需要一张参考照片。")
    if templates.shape[0] != len(photos):
        raise RuntimeError("人物档案模板数量必须与参考照片数量一致。")

    directory.mkdir(parents=True, exist_ok=True)
    _atomic_save_embedding(embedding_path, templates, centroid)
    manifest: dict[str, object] = {
        "schema_version": PROFILE_SCHEMA_VERSION,
        "profile_name": name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "preprocessing_version": PREPROCESSING_VERSION,
        "embedding_file": EMBEDDING_FILENAME,
        "embedding_sha256": sha256_file(embedding_path),
        "embedding_dimension": int(templates.shape[1]),
        "template_count": int(templates.shape[0]),
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
    schema_version = manifest.get("schema_version")
    if schema_version not in SUPPORTED_PROFILE_SCHEMA_VERSIONS:
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
    if manifest.get("embedding_sha256") != sha256_file(embedding_path):
        raise RuntimeError(f"人物档案特征文件校验失败：{name}")

    try:
        with np.load(embedding_path, allow_pickle=False) as values:
            if "templates" in values:
                embeddings = np.asarray(values["templates"], dtype=np.float32)
            else:
                embeddings = np.asarray(values["embedding"], dtype=np.float32)
    except (OSError, ValueError, KeyError) as exc:
        raise RuntimeError(f"人物档案特征文件损坏：{name}") from exc
    normalized = _validated_embeddings(embeddings)
    if manifest.get("embedding_dimension") != int(normalized.shape[1]):
        raise RuntimeError(f"人物档案特征维度不匹配：{name}")
    if schema_version == PROFILE_SCHEMA_VERSION and manifest.get(
        "template_count"
    ) != int(normalized.shape[0]):
        raise RuntimeError(f"人物档案模板数量不匹配：{name}")
    return normalized, manifest


def _read_valid_manifest(directory: Path, *, expected_name: str) -> dict[str, object]:
    manifest_path = directory / MANIFEST_FILENAME
    embedding_path = directory / EMBEDDING_FILENAME
    if not directory.is_dir() or not manifest_path.is_file() or not embedding_path.is_file():
        raise RuntimeError(f"人物档案不存在或不完整：{expected_name}")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"人物档案清单损坏：{expected_name}") from exc
    if not isinstance(manifest, dict):
        raise RuntimeError(f"人物档案清单格式无效：{expected_name}")
    if manifest.get("schema_version") not in SUPPORTED_PROFILE_SCHEMA_VERSIONS:
        raise RuntimeError(f"人物档案版本不兼容，请重新注册：{expected_name}")
    if manifest.get("profile_name") != expected_name:
        raise RuntimeError(f"人物档案名称不匹配：{expected_name}")
    return manifest


def profile_compatibility(
    manifest: dict[str, object],
    config: dict[str, object],
) -> tuple[bool, bool, str | None]:
    """Return whether a profile can be used by the current model pipeline."""

    if manifest.get("schema_version") != PROFILE_SCHEMA_VERSION:
        return False, True, "profile_schema_changed"
    if manifest.get("preprocessing_version") != PREPROCESSING_VERSION:
        return False, True, "preprocessing_changed"
    try:
        current = _current_model_info(config)
    except RuntimeError:
        return False, False, "model_assets_unavailable"
    if manifest.get("person_detector_model_sha256") != current[
        "person_detector_model_sha256"
    ]:
        return False, True, "person_detector_model_changed"
    if manifest.get("reid_model_name") != current["reid_model_name"]:
        return False, True, "reid_model_changed"
    if manifest.get("reid_model_sha256") != current["reid_model_sha256"]:
        return False, True, "reid_model_changed"
    return True, False, None


def _public_profile(
    manifest: dict[str, object],
    *,
    config: dict[str, object] | None = None,
    include_photos: bool = True,
) -> dict[str, object]:
    """Expose metadata only; source paths and raw embeddings never leave storage."""

    photos = manifest.get("photos")
    safe_photos: list[dict[str, object]] = []
    if isinstance(photos, list):
        for photo in photos:
            if not isinstance(photo, dict):
                continue
            safe_photos.append(
                {
                    "index": photo.get("index"),
                    "sha256": photo.get("sha256"),
                }
            )
    result: dict[str, object] = {
        "name": manifest.get("profile_name"),
        "createdAt": manifest.get("created_at"),
        "updatedAt": manifest.get("updated_at"),
        "photoCount": manifest.get("photo_count"),
        "embeddingDimension": manifest.get("embedding_dimension"),
        "modelName": manifest.get("reid_model_name"),
    }
    if include_photos:
        result["photos"] = safe_photos
    if config is not None:
        compatible, requires_reenrollment, reason = profile_compatibility(manifest, config)
        result.update(
            {
                "compatible": compatible,
                "requiresReenrollment": requires_reenrollment,
                "incompatibilityReason": reason,
            }
        )
    return result


def _current_model_info(config: dict[str, object]) -> dict[str, str]:
    vision = config.get("vision", {})
    cfg = vision if isinstance(vision, dict) else config
    reid_model_path = configured_model_path(config, "reid_model_path")
    detector_model_path = configured_model_path(config, "person_detector_model")
    for label, path in (
        ("ReID", reid_model_path),
        ("YOLO", detector_model_path),
    ):
        if not path.is_file():
            raise RuntimeError(f"{label} 权重不存在：{path}")
    return {
        "reid_model_name": str(cfg.get("reid_model_name", "osnet_x0_25")),
        "reid_model_sha256": sha256_file(reid_model_path),
        "person_detector_model_sha256": sha256_file(detector_model_path),
    }


def _validated_embeddings(value: Any) -> np.ndarray:
    values = np.asarray(value, dtype=np.float32)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    if values.ndim != 2 or values.shape[0] == 0 or values.shape[1] == 0:
        raise RuntimeError("人物档案特征必须是二维非空模板矩阵。")
    if not np.all(np.isfinite(values)):
        raise RuntimeError("人物档案特征包含无效数值。")
    norms = np.linalg.norm(values, axis=1, keepdims=True)
    if np.any(norms <= 1e-12):
        raise RuntimeError("人物档案特征是零向量。")
    return (values / norms).astype(np.float32)


def _normalized_centroid(templates: np.ndarray) -> np.ndarray:
    centroid = templates.mean(axis=0)
    norm = float(np.linalg.norm(centroid))
    if norm <= 1e-12:
        raise RuntimeError("人物档案特征中心无效。")
    return (centroid / norm).astype(np.float32)


def _atomic_save_embedding(
    path: Path, templates: np.ndarray, centroid: np.ndarray
) -> None:
    descriptor, raw_temp_path = tempfile.mkstemp(
        prefix=".embedding-", suffix=".npz", dir=path.parent
    )
    temp_path = Path(raw_temp_path)
    try:
        with os.fdopen(descriptor, "wb") as file:
            # ``embedding`` preserves a readable centroid for older tooling;
            # schema-v2 runtimes use the independent ``templates`` matrix.
            np.savez_compressed(file, templates=templates, embedding=centroid)
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
