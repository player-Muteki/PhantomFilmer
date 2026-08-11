#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${1:-python3}"
VENV_DIR=".venv-reid"
TORCHREID_COMMIT="f8cd150fdf77e8d9e1ed143b7f308c2c609ded50"

mkdir -p .matplotlib .ultralytics
"${PYTHON_BIN}" -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
"${VENV_DIR}/bin/python" -m pip install -r requirements-reid-bootstrap.txt
"${VENV_DIR}/bin/python" -m pip install --no-deps \
  "djitellopy==2.5.0" "av==18.0.0"
"${VENV_DIR}/bin/python" -m pip install --no-build-isolation --no-deps \
  "git+https://github.com/KaiyangZhou/deep-person-reid.git@${TORCHREID_COMMIT}"

MPLCONFIGDIR=.matplotlib YOLO_CONFIG_DIR=.ultralytics \
  "${VENV_DIR}/bin/python" - <<'PY'
import cv2
import av
import djitellopy
import numpy
import torch
import torchvision
import ultralytics
import torchreid
from torchreid.utils import FeatureExtractor

print("ReID environment ready")
print("numpy", numpy.__version__)
print("opencv", cv2.__version__)
print("av", av.__version__)
print("djitellopy", getattr(djitellopy, "__version__", "2.5.0"))
print("torch", torch.__version__)
print("torchvision", torchvision.__version__)
print("ultralytics", ultralytics.__version__)
print("torchreid", torchreid.__version__)
print("extractor", FeatureExtractor.__name__)
PY
