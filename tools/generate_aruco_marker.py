"""Generate and save the target ArUco marker (ID 23, DICT_4X4_50).

Usage::

    python3 tools/generate_aruco_marker.py

The output image is saved to ``docs/aruco_marker_23.png`` at 1000×1000 px,
suitable for printing at 15–20 cm with at least 2 cm white margin.
"""

import sys
from pathlib import Path


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

try:
    import cv2
    import numpy as np
except ModuleNotFoundError:
    raise RuntimeError("请先安装 requirements.txt：pip install -r requirements.txt")


def generate_aruco_marker(
    marker_id: int = 23,
    dictionary_name: str = "DICT_4X4_50",
    pixel_size: int = 1000,
    margin_px: int = 100,
    output_path: str = "",
) -> str:
    """Generate a printable ArUco marker image.

    Args:
        marker_id: ArUco marker ID (0--49 for DICT_4X4_50).
        dictionary_name: OpenCV ArUco dictionary name.
        pixel_size: Marker region pixel size (excluding margin).
        margin_px: Extra white margin around the marker.
        output_path: Full path for the output PNG.  Auto-generated if empty.

    Returns:
        Path to the saved PNG file.
    """
    dict_const = getattr(cv2.aruco, dictionary_name, cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(dict_const)

    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, pixel_size)

    if margin_px > 0:
        canvas = np.ones(
            (pixel_size + 2 * margin_px, pixel_size + 2 * margin_px),
            dtype=np.uint8,
        ) * 255
        canvas[margin_px:margin_px + pixel_size, margin_px:margin_px + pixel_size] = marker
    else:
        canvas = marker

    if not output_path:
        docs_dir = _PROJECT_ROOT / "docs"
        docs_dir.mkdir(parents=True, exist_ok=True)
        output_path = str(docs_dir / f"aruco_marker_{marker_id}.png")

    cv2.imwrite(output_path, canvas)
    return output_path


def main() -> None:
    path = generate_aruco_marker(
        marker_id=23,
        dictionary_name="DICT_4X4_50",
        pixel_size=1000,
        margin_px=100,
    )
    print(f"ArUco marker ID 23 saved to: {path}")
    print()
    print("打印参数：")
    print("  Dictionary: DICT_4X4_50")
    print("  Marker ID:  23")
    print("  建议打印尺寸：15-20 cm")
    print("  四周白色静区：至少 2 cm")
    print("  打印时保持正方形，禁止拉伸变形")


if __name__ == "__main__":
    main()
