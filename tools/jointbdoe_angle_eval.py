#!/usr/bin/env python3
"""Run JointBDOE on local images without connecting to a drone."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from time import perf_counter
from typing import Iterable, List

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision.jointbdoe_orientation import (  # noqa: E402
    JointBDOEOrientationEstimator,
    JointBDOEPrediction,
)


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="在本地图片上验证 JointBDOE 人体框和 0～360°朝向角，不连接无人机。"
    )
    parser.add_argument("input", type=Path, help="单张图片或图片目录")
    parser.add_argument("--source-path", default="third_party/JointBDOE")
    parser.add_argument("--model-path", default="weights/jointbdoe_s.pt")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--conf-thres", type=float, default=0.30)
    parser.add_argument("--iou-thres", type=float, default=0.45)
    parser.add_argument("--output-dir", type=Path, help="可选：保存标注图片的目录")
    return parser.parse_args()


def iter_images(input_path: Path) -> Iterable[Path]:
    if input_path.is_file():
        if input_path.suffix.lower() in IMAGE_SUFFIXES:
            yield input_path
        return
    if input_path.is_dir():
        for path in sorted(input_path.rglob("*")):
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES:
                yield path


def draw_predictions(image, predictions: List[JointBDOEPrediction], cv2):
    output = image.copy()
    for prediction in predictions:
        x1, y1, x2, y2 = (int(round(value)) for value in prediction.bbox_xyxy)
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
        center = ((x1 + x2) // 2, (y1 + y2) // 2)
        length = max(12, min(x2 - x1, y2 - y1) // 3)
        radians = math.radians(prediction.angle_deg)
        end = (
            int(center[0] - length * math.sin(radians)),
            int(center[1] - length * math.cos(radians)),
        )
        cv2.arrowedLine(
            output,
            center,
            end,
            (0, 255, 255),
            2,
            line_type=cv2.LINE_AA,
            tipLength=0.3,
        )
        cv2.putText(
            output,
            f"{prediction.angle_deg:.1f} deg det={prediction.detection_confidence:.2f}",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
    return output


def main() -> int:
    args = parse_args()
    try:
        import cv2
    except ModuleNotFoundError:
        print("缺少 opencv-python，请安装 requirements-reid.txt。", file=sys.stderr)
        return 2

    paths = list(iter_images(args.input.expanduser()))
    if not paths:
        print(f"没有找到可测试的图片：{args.input}", file=sys.stderr)
        return 2

    estimator = JointBDOEOrientationEstimator(
        source_path=args.source_path,
        model_path=args.model_path,
        device=args.device,
        image_size=args.image_size,
        confidence_threshold=args.conf_thres,
        iou_threshold=args.iou_thres,
        smoothing_window=1,
    )
    try:
        started = perf_counter()
        estimator.prepare()
        print(f"JointBDOE 加载完成：{perf_counter() - started:.2f} 秒")
    except Exception as exc:
        print(f"JointBDOE 准备失败：{exc}", file=sys.stderr)
        return 1

    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)

    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f"跳过无法读取的图片：{path}", file=sys.stderr)
            continue
        started = perf_counter()
        predictions = estimator.detect_people(image)
        latency_ms = (perf_counter() - started) * 1000.0
        angles = ", ".join(
            f"{item.angle_deg:.1f}°(det={item.detection_confidence:.2f})"
            for item in predictions
        ) or "未检测到人体"
        print(f"{path}: {angles}; {latency_ms:.1f} ms")
        if args.output_dir:
            output = draw_predictions(image, predictions, cv2)
            cv2.imwrite(str(args.output_dir / path.name), output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
