#!/usr/bin/env bash
set -euo pipefail

SOURCE_DIR="third_party/JointBDOE"
WEIGHT_PATH="weights/jointbdoe_s.pt"
WEIGHT_URL="https://huggingface.co/HoyerChou/JointBDOE/resolve/main/coco_s_1024_e500_t010_w005_best.pt"
WEIGHT_SHA256="bc6d63ee0f685a888e5ff94a84d8244ce23a817223010e100459137bacae3e27"

mkdir -p third_party weights

if [[ ! -d "${SOURCE_DIR}/.git" ]]; then
  git clone --depth 1 https://github.com/hnuzhy/JointBDOE.git "${SOURCE_DIR}"
else
  echo "JointBDOE 源码已存在：${SOURCE_DIR}"
fi

if [[ ! -f "${WEIGHT_PATH}" ]]; then
  curl -L --fail --retry 3 "${WEIGHT_URL}" -o "${WEIGHT_PATH}"
else
  echo "JointBDOE 权重已存在：${WEIGHT_PATH}"
fi

ACTUAL_SHA256="$(shasum -a 256 "${WEIGHT_PATH}" | awk '{print $1}')"
if [[ "${ACTUAL_SHA256}" != "${WEIGHT_SHA256}" ]]; then
  echo "JointBDOE 权重校验失败：${ACTUAL_SHA256}" >&2
  exit 1
fi

echo "JointBDOE 源码和 S 型权重准备完成。"
