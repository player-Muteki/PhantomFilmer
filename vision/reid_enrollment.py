"""On-site reference-photo enrollment helpers for person ReID demos."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ENROLLMENT_DIR = PROJECT_ROOT / "data" / "reid_target" / "现场注册"
SUPPORTED_REFERENCE_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp", ".tif", ".tiff"}


@dataclass
class TargetLockTracker:
    """Count consecutive trustworthy ReID matches before takeoff."""

    required_frames: int
    consecutive_frames: int = 0

    def __post_init__(self) -> None:
        """Validate and normalize the requested lock duration."""
        self.required_frames = max(1, int(self.required_frames))

    def observe(self, result: dict[str, object]) -> bool:
        """Record one ReID result and return whether the target is locked.

        Predicted positions and ambiguous matches are deliberately rejected for
        ground authorization.  Only fresh identity matches count toward takeoff.
        """
        trustworthy = (
            bool(result.get("found"))
            and not bool(result.get("is_predicted"))
            and not bool(result.get("ambiguous"))
        )
        self.consecutive_frames = self.consecutive_frames + 1 if trustworthy else 0
        return self.consecutive_frames >= self.required_frames

    def reset(self) -> None:
        """Discard partial progress after a frame or inference failure."""
        self.consecutive_frames = 0

    @property
    def progress(self) -> str:
        """Return a compact progress label for the preview window."""
        return f"{min(self.consecutive_frames, self.required_frames)}/{self.required_frames}"


def build_reid_runtime_config(
    config: dict[str, object],
    reference_images: Sequence[Path] | None = None,
    profile_name: str | None = None,
) -> dict[str, object]:
    """Return an in-memory config that enables ReID with photos or a profile.

    The project YAML is never rewritten during a live demonstration, which
    avoids leaving a machine in an unsafe or surprising default mode.
    """
    images = list(reference_images or [])
    selected_profile = str(profile_name or "").strip()
    if bool(images) == bool(selected_profile):
        raise ValueError("必须且只能选择参考照片或一个人物档案。")
    runtime = dict(config)
    raw_vision = config.get("vision", {})
    vision = dict(raw_vision) if isinstance(raw_vision, dict) else {}
    vision["detector_type"] = "person_reid"
    if selected_profile:
        vision["reference_profile"] = selected_profile
        vision.pop("reference_images", None)
    else:
        vision["reference_images"] = [str(path) for path in images]
        vision.pop("reference_profile", None)
    runtime["vision"] = vision
    # ReID 人物距离带与默认 ArUco 分开标定：仅在 ReID 运行期覆盖到 shared 顶层
    # 键，避免默认 ArUco 跟随被静默调近。
    for key in (
        "target_area_ratio_min",
        "target_area_ratio_max",
        "target_lock_exit_area_ratio_min",
        "target_lock_exit_area_ratio_max",
    ):
        reid_key = f"reid_{key}"
        if reid_key in runtime:
            runtime[key] = runtime[reid_key]
    return runtime


def validate_reference_images(values: Sequence[str]) -> list[Path]:
    """Resolve CLI photo values and reject missing or non-file paths."""
    paths: list[Path] = []
    for value in values:
        for item in str(value).split(","):
            cleaned = item.strip().strip("\"").strip("'")
            if not cleaned:
                continue
            path = Path(cleaned).expanduser()
            resolved = path.resolve() if path.is_absolute() else (Path.cwd() / path).resolve()
            if not resolved.is_file():
                raise RuntimeError(f"目标人物照片不存在：{resolved}")
            paths.append(resolved)
    if not paths:
        raise RuntimeError("未提供可用的目标人物照片。")
    return paths


def validate_reference_directory(value: str) -> list[Path]:
    """Return supported image files directly inside one reference directory."""
    raw = str(value).strip().strip("\"").strip("'")
    directory = Path(raw).expanduser()
    resolved = (
        directory.resolve()
        if directory.is_absolute()
        else (Path.cwd() / directory).resolve()
    )
    if not resolved.is_dir():
        raise RuntimeError(f"目标人物照片目录不存在：{resolved}")
    paths = sorted(
        path.resolve()
        for path in resolved.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_REFERENCE_SUFFIXES
    )
    if not paths:
        raise RuntimeError(
            f"目标人物照片目录中没有可用图片：{resolved}。"
            "请使用 JPG、PNG、WebP 或 TIFF。"
        )
    return paths


def prompt_reference_source() -> tuple[list[str], bool]:
    """Ask for photo paths or request capture from the computer camera."""
    print("请录入目标人物：")
    print("- 输入一张或多张照片路径（多张用英文逗号分隔）")
    print("- 输入 CAMERA 使用电脑摄像头现场连拍")
    try:
        answer = input("照片路径或 CAMERA：").strip()
    except (EOFError, KeyboardInterrupt) as exc:
        raise RuntimeError("已取消目标人物录入。") from exc
    if answer.upper() == "CAMERA":
        return [], True
    return [answer], False


def capture_reference_images(
    camera_index: int = 0,
    image_count: int = 3,
    output_dir: Path = DEFAULT_ENROLLMENT_DIR,
) -> list[Path]:
    """Capture full-body reference photos with the computer camera.

    Press SPACE to save each view and Q to cancel.  Capturing several angles
    produces a more representative averaged ReID embedding.
    """
    try:
        import cv2
    except ModuleNotFoundError as exc:
        raise RuntimeError("缺少 OpenCV，无法使用电脑摄像头拍照。") from exc

    requested_count = max(1, int(image_count))
    output_dir.mkdir(parents=True, exist_ok=True)
    camera = cv2.VideoCapture(int(camera_index))
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"无法打开电脑摄像头 {camera_index}。")

    captured: list[Path] = []
    session_id = datetime.now().strftime("%Y%m%d-%H%M%S")
    window_name = "PhantomFilmer ReID Enrollment"
    print("请让目标人物全身入镜，按空格键拍摄，按 q 取消。")
    try:
        while len(captured) < requested_count:
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError("电脑摄像头读取画面失败。")
            preview = frame.copy()
            cv2.putText(
                preview,
                f"Full body | SPACE capture {len(captured) + 1}/{requested_count} | Q cancel",
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(20) & 0xFF
            if key == ord("q"):
                raise RuntimeError("已取消目标人物拍照。")
            if key != ord(" "):
                continue
            path = output_dir / f"{session_id}-{len(captured) + 1}.jpg"
            if not cv2.imwrite(str(path), frame):
                raise RuntimeError(f"保存参考照片失败：{path}")
            captured.append(path.resolve())
            print(f"已拍摄：{path}")
    finally:
        camera.release()
        try:
            cv2.destroyWindow(window_name)
        except Exception:
            # Some headless/OpenCV builds raise when a window was never shown.
            pass
    return captured


def collect_reference_images(
    provided_values: Sequence[str] | None,
    capture_from_camera: bool,
    camera_index: int,
    image_count: int,
) -> list[Path]:
    """Resolve the complete interactive/CLI enrollment workflow."""
    values = list(provided_values or [])
    should_capture = bool(capture_from_camera)
    if not values and not should_capture:
        values, should_capture = prompt_reference_source()
    if values and should_capture:
        raise RuntimeError("照片路径和 --capture-reference 不能同时使用。")
    if should_capture:
        return capture_reference_images(camera_index, image_count)
    return validate_reference_images(values)
