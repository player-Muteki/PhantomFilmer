# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

import yaml
from PyInstaller.utils.hooks import collect_all, collect_submodules

project_root = Path(SPEC).resolve().parents[1]

config_path = project_root / "config.yaml"
runtime_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
vision_config = runtime_config.get("vision", {})
if not isinstance(vision_config, dict):
    raise ValueError("config.yaml 的 vision 必须是映射")

datas = [(str(config_path), ".")]
for config_key in (
    "person_detector_model",
    "reid_model_path",
    "jointbdoe_model_path",
):
    relative_path = Path(str(vision_config.get(config_key, "")))
    if (
        not relative_path.parts
        or relative_path.is_absolute()
        or ".." in relative_path.parts
    ):
        raise ValueError(f"{config_key} 必须是项目内相对路径：{relative_path}")
    source_path = project_root / relative_path
    if not source_path.is_file():
        raise FileNotFoundError(f"缺少桌面端模型资源：{source_path}")
    datas.append((str(source_path), str(relative_path.parent)))

jointbdoe_relative = Path(
    str(vision_config.get("jointbdoe_source_path", "third_party/JointBDOE"))
)
if jointbdoe_relative.is_absolute() or ".." in jointbdoe_relative.parts:
    raise ValueError(
        f"jointbdoe_source_path 必须是项目内相对路径：{jointbdoe_relative}"
    )
jointbdoe_source = project_root / jointbdoe_relative
for runtime_entry in ("models", "utils", "Arial.ttf"):
    source_path = jointbdoe_source / runtime_entry
    if not source_path.exists():
        raise FileNotFoundError(f"缺少 JointBDOE 推理源码：{source_path}")
    datas.append((str(source_path), str(jointbdoe_relative / runtime_entry)))

binaries = []
hiddenimports = [
    "cv2",
    "numpy",
    "djitellopy",
    "yaml",
    "torch",
    "torchvision",
    "ultralytics",
    "torchreid",
    # JointBDOE is shipped as runtime source because its checkpoint pickles
    # original models.* classes. PyInstaller cannot analyze imports from data
    # files, so declare the inference source's direct dependencies explicitly.
    "matplotlib",
    "matplotlib.pyplot",
    "pandas",
    "PIL",
    "requests",
    "seaborn",
    "tqdm",
]
for runtime_package in ("torchreid",):
    package_datas, package_binaries, package_hiddenimports = collect_all(
        runtime_package
    )
    datas += package_datas
    binaries += package_binaries
    hiddenimports += package_hiddenimports
hiddenimports += collect_submodules("scipy._external.array_api_compat")

analysis = Analysis(
    [str(project_root / "sidecar" / "entrypoint.py")],
    pathex=[str(project_root)],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["tkinter", "pytest"],
    noarchive=False,
    optimize=1,
)

python_archive = PYZ(analysis.pure)

executable = EXE(
    python_archive,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="phantomfilmer-sidecar",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,
)

collection = COLLECT(
    executable,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=False,
    name="phantomfilmer-sidecar",
)
