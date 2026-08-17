"""Shared camera debug annotations with Chinese-capable labels."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Tuple

import numpy as np

Box = Tuple[int, int, int, int]
Color = Tuple[int, int, int]


@dataclass(frozen=True)
class BoxAnnotation:
    bbox: Box
    label: str
    color: Color
    thickness: int = 2


def draw_box_annotations(frame: Any, annotations: Iterable[BoxAnnotation]) -> Any:
    if not _valid_frame(frame):
        return frame
    import cv2

    debug = frame.copy()
    valid_annotations = [item for item in annotations if _valid_box(item.bbox)]
    for item in valid_annotations:
        x, y, width, height = item.bbox
        cv2.rectangle(
            debug,
            (x, y),
            (x + width, y + height),
            item.color,
            item.thickness,
        )
    if not valid_annotations:
        return debug
    return _draw_pillow_labels(debug, valid_annotations)


def draw_status_label(
    frame: Any,
    text: str,
    color: Color,
    *,
    top: int = 84,
) -> Any:
    if not _valid_frame(frame) or not text:
        return frame
    return _draw_pillow_labels(
        frame,
        [BoxAnnotation((12, top, 1, 1), text, color, 0)],
        status_only=True,
    )


def _draw_pillow_labels(
    frame: Any,
    annotations: list[BoxAnnotation],
    *,
    status_only: bool = False,
) -> Any:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ModuleNotFoundError:
        return _draw_ascii_labels(frame, annotations, status_only=status_only)

    image = Image.fromarray(frame[:, :, ::-1].copy())
    draw = ImageDraw.Draw(image)
    font = _load_font(ImageFont)
    for item in annotations:
        x, y, width, height = item.bbox
        if status_only:
            text_x = max(12, x)
            text_y = max(12, y)
        else:
            text_x = max(0, x)
            text_y = y - 8
            try:
                text_bbox = draw.textbbox((text_x, text_y), item.label, font=font, stroke_width=2)
            except (OSError, UnicodeEncodeError):
                return _draw_ascii_labels(frame, annotations, status_only=status_only)
            if text_bbox[1] < 0:
                text_y = min(frame.shape[0] - 22, y + max(2, height) + 2)
        try:
            text_bbox = draw.textbbox((text_x, text_y), item.label, font=font, stroke_width=2)
            left, top, right, bottom = text_bbox
            background = (0, 0, 0)
            draw.rectangle((left - 3, top - 2, right + 3, bottom + 2), fill=background)
            red, green, blue = item.color[2], item.color[1], item.color[0]
            draw.text(
                (text_x, text_y),
                item.label,
                font=font,
                fill=(red, green, blue),
                stroke_width=2,
                stroke_fill=background,
            )
        except (OSError, UnicodeEncodeError):
            return _draw_ascii_labels(frame, annotations, status_only=status_only)
    return np.asarray(image)[:, :, ::-1].copy()


def _draw_ascii_labels(
    frame: Any,
    annotations: list[BoxAnnotation],
    *,
    status_only: bool,
) -> Any:
    import cv2

    debug = frame.copy()
    for item in annotations:
        x, y, width, height = item.bbox
        if not status_only:
            cv2.rectangle(debug, (x, y), (x + width, y + height), item.color, item.thickness)
        cv2.putText(
            debug,
            item.label.encode("ascii", "replace").decode("ascii"),
            (max(0, x), max(20, y - 8) if not status_only else max(20, y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            item.color,
            2,
        )
    return debug


def _load_font(image_font: Any) -> Any:
    candidates = (
        "/System/Library/Fonts/STHeiti Medium.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    )
    font_path = next((path for path in candidates if Path(path).is_file()), None)
    if font_path is not None:
        return image_font.truetype(font_path, 20)
    return image_font.load_default()


def _valid_frame(frame: Any) -> bool:
    return isinstance(frame, np.ndarray) and frame.ndim >= 3 and frame.size > 0


def _valid_box(bbox: Box) -> bool:
    return len(bbox) == 4 and bbox[2] > 0 and bbox[3] > 0
