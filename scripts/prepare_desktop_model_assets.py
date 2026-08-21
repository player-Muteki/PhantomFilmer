"""Download and verify the model assets embedded in desktop releases."""

from __future__ import annotations

import hashlib
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
from urllib.request import Request, urlopen

PROJECT_ROOT = Path(__file__).resolve().parents[1]
JOINTBDOE_COMMIT = "362f999e22bd50a4e73aca882b58c13f8a96a13c"
JOINTBDOE_REPOSITORY = "https://github.com/hnuzhy/JointBDOE.git"


@dataclass(frozen=True)
class ModelAsset:
    """One immutable release-model download."""

    relative_path: str
    sha256: str
    url: str | None = None
    google_drive_id: str | None = None


MODEL_ASSETS = (
    ModelAsset(
        relative_path="weights/yolov8n.pt",
        url="https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt",
        sha256="31e20dde3def09e2cf938c7be6fe23d9150bbbe503982af13345706515f2ef95",
    ),
    ModelAsset(
        relative_path="weights/osnet_x0_25_msmt17.pth",
        google_drive_id="1Kkx2zW89jq_NETu4u42CFZTMVD5Hwm6e",
        sha256="cf55163d78fc44c62c82f85ab62d39f10438679b5abe8c698ae08cfa84aa6e18",
    ),
    ModelAsset(
        relative_path="weights/jointbdoe_s.pt",
        url="https://huggingface.co/HoyerChou/JointBDOE/resolve/main/coco_s_1024_e500_t010_w005_best.pt",
        sha256="bc6d63ee0f685a888e5ff94a84d8244ce23a817223010e100459137bacae3e27",
    ),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _download(asset: ModelAsset, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_name(destination.name + ".part")
    if partial.exists():
        partial.unlink()
    try:
        if asset.google_drive_id:
            try:
                import gdown
            except ModuleNotFoundError as exc:
                raise RuntimeError(
                    "下载 OSNet 权重前必须安装 requirements-desktop-build.txt"
                ) from exc
            result = gdown.download(
                id=asset.google_drive_id,
                output=str(partial),
                quiet=False,
            )
            if not result:
                raise RuntimeError(f"Google Drive 下载失败：{asset.relative_path}")
        elif asset.url:
            request = Request(
                asset.url,
                headers={"User-Agent": "PhantomFilmer desktop builder"},
            )
            with urlopen(request, timeout=120) as response, partial.open(
                "wb"
            ) as output:
                shutil.copyfileobj(response, output)
        else:
            raise RuntimeError(f"模型未配置下载地址：{asset.relative_path}")

        actual = _sha256(partial)
        if actual != asset.sha256:
            raise RuntimeError(
                f"模型校验失败：{asset.relative_path}，实际 SHA-256={actual}"
            )
        partial.replace(destination)
    finally:
        if partial.exists():
            partial.unlink()


def _prepare_model(asset: ModelAsset) -> None:
    destination = PROJECT_ROOT / asset.relative_path
    if destination.is_file():
        actual = _sha256(destination)
        if actual != asset.sha256:
            raise RuntimeError(
                f"现有模型校验失败且不会自动覆盖：{destination}，"
                f"实际 SHA-256={actual}"
            )
        print(f"模型已验证：{asset.relative_path}")
        return
    _download(asset, destination)
    print(f"模型已下载并验证：{asset.relative_path}")


def _prepare_jointbdoe_source() -> None:
    source = PROJECT_ROOT / "third_party" / "JointBDOE"
    if source.exists():
        if not (source / ".git").is_dir():
            raise RuntimeError(f"JointBDOE 目录存在但不是 Git 仓库：{source}")
    else:
        source.parent.mkdir(parents=True, exist_ok=True)
        with TemporaryDirectory(
            prefix="jointbdoe-source-", dir=source.parent
        ) as temporary:
            checkout = Path(temporary) / "checkout"
            subprocess.run(
                ["git", "init", "--quiet", str(checkout)],
                check=True,
            )
            subprocess.run(
                [
                    "git",
                    "-C",
                    str(checkout),
                    "fetch",
                    "--depth=1",
                    JOINTBDOE_REPOSITORY,
                    JOINTBDOE_COMMIT,
                ],
                check=True,
            )
            subprocess.run(
                ["git", "-C", str(checkout), "checkout", "--quiet", "FETCH_HEAD"],
                check=True,
            )
            shutil.move(str(checkout), source)

    revision = subprocess.run(
        ["git", "-C", str(source), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if revision != JOINTBDOE_COMMIT:
        raise RuntimeError(
            f"JointBDOE 源码版本不匹配：期望 {JOINTBDOE_COMMIT}，实际 {revision}"
        )
    for runtime_entry in ("models/yolo.py", "utils/general.py", "Arial.ttf"):
        if not (source / runtime_entry).is_file():
            raise RuntimeError(f"JointBDOE 源码不完整：{source / runtime_entry}")
    print(f"JointBDOE 源码已验证：{revision}")


def main() -> int:
    """Prepare every ignored asset required by a self-contained desktop build."""
    for asset in MODEL_ASSETS:
        _prepare_model(asset)
    _prepare_jointbdoe_source()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
