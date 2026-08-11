#!/usr/bin/env python3
"""Evaluate the packaged OSNet target verification setup on PRID 2011.

This script tests cropped-person ReID only. It does not test a raw camera frame,
YOLO person detection, flight control, or drone connectivity.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision.person_reid_detect import TorchreidFeatureExtractor


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("data/datasets/prid_2011/multi_shot"),
    )
    parser.add_argument(
        "--reference-dir",
        type=Path,
        default=Path("data/reid_target"),
    )
    parser.add_argument("--target-person", default="person_0001")
    parser.add_argument("--negative-identities", type=int, default=50)
    parser.add_argument(
        "--model-path",
        type=Path,
        default=Path("weights/osnet_x0_25_msmt17.pth"),
    )
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--thresholds",
        type=float,
        nargs="+",
        default=[0.65, 0.60, 0.55],
    )
    parser.add_argument("--chunk-size", type=int, default=256)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def load_rgb(paths: Sequence[Path]) -> list[np.ndarray]:
    images = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f"Cannot read image: {path}")
        images.append(image[:, :, ::-1].copy())
    return images


def normalize(values: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(values, axis=1, keepdims=True)
    if np.any(norms <= 1e-12):
        raise RuntimeError("Feature extractor returned a zero vector")
    return values / norms


def extract_in_chunks(
    extractor: TorchreidFeatureExtractor,
    paths: Sequence[Path],
    chunk_size: int,
) -> tuple[np.ndarray, float]:
    if not paths:
        raise RuntimeError("No images found for evaluation")
    chunks = []
    started = time.perf_counter()
    for index in range(0, len(paths), chunk_size):
        chunks.append(extractor.extract(load_rgb(paths[index : index + chunk_size])))
    return normalize(np.concatenate(chunks, axis=0)), time.perf_counter() - started


def distribution(values: np.ndarray) -> dict[str, float]:
    return {
        "min": round(float(values.min()), 6),
        "p10": round(float(np.percentile(values, 10)), 6),
        "median": round(float(np.median(values)), 6),
        "mean": round(float(values.mean()), 6),
        "p90": round(float(np.percentile(values, 90)), 6),
        "max": round(float(values.max()), 6),
    }


def threshold_metrics(
    positive_scores: np.ndarray,
    negative_scores: np.ndarray,
    threshold: float,
) -> dict[str, float | int]:
    tp = int((positive_scores >= threshold).sum())
    fn = len(positive_scores) - tp
    fp = int((negative_scores >= threshold).sum())
    tn = len(negative_scores) - fp
    recall = tp / len(positive_scores)
    false_positive_rate = fp / len(negative_scores)
    specificity = tn / len(negative_scores)
    return {
        "threshold": round(float(threshold), 4),
        "tp": tp,
        "fn": fn,
        "fp": fp,
        "tn": tn,
        "recall": round(recall, 6),
        "false_positive_rate": round(false_positive_rate, 6),
        "specificity": round(specificity, 6),
        "balanced_accuracy": round((recall + specificity) / 2, 6),
    }


def image_paths(directory: Path) -> list[Path]:
    supported = {".jpg", ".jpeg", ".png", ".bmp"}
    return [
        path
        for path in sorted(directory.iterdir())
        if path.is_file() and path.suffix.lower() in supported
    ]


def main() -> None:
    args = parse_args()
    if args.negative_identities < 1:
        raise ValueError("--negative-identities must be positive")
    if args.chunk_size < 1:
        raise ValueError("--chunk-size must be positive")

    references = image_paths(args.reference_dir)
    positives = image_paths(args.dataset_root / "cam_b" / args.target_person)
    negative_dirs = [
        path
        for path in sorted((args.dataset_root / "cam_b").glob("person_*"))
        if path.is_dir() and path.name != args.target_person
    ][: args.negative_identities]
    negatives = [path for directory in negative_dirs for path in image_paths(directory)]

    extractor = TorchreidFeatureExtractor(
        "osnet_x0_25",
        str(args.model_path),
        args.device,
    )
    reference_features, reference_seconds = extract_in_chunks(
        extractor, references, args.chunk_size
    )
    centroid = reference_features.mean(axis=0)
    centroid /= np.linalg.norm(centroid)
    positive_features, positive_seconds = extract_in_chunks(
        extractor, positives, args.chunk_size
    )
    negative_features, negative_seconds = extract_in_chunks(
        extractor, negatives, args.chunk_size
    )

    positive_scores = positive_features @ centroid
    negative_scores = negative_features @ centroid
    greater = positive_scores[:, None] > negative_scores[None, :]
    equal = positive_scores[:, None] == negative_scores[None, :]
    auc = float(greater.mean() + 0.5 * equal.mean())
    total_seconds = reference_seconds + positive_seconds + negative_seconds
    total_images = len(references) + len(positives) + len(negatives)

    result = {
        "reference_images": len(references),
        "positive_frames": len(positives),
        "negative_identities": len(negative_dirs),
        "negative_frames": len(negatives),
        "positive_scores": distribution(positive_scores),
        "negative_scores": distribution(negative_scores),
        "auc": round(auc, 6),
        "thresholds": {
            str(threshold): threshold_metrics(
                positive_scores, negative_scores, threshold
            )
            for threshold in args.thresholds
        },
        "inference_seconds": {
            "total": round(total_seconds, 4),
            "per_image": round(total_seconds / total_images, 6),
        },
    }

    payload = json.dumps(result, ensure_ascii=False, indent=2)
    print(payload)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
