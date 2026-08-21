# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

project_root = Path(SPEC).resolve().parents[1]

analysis = Analysis(
    [str(project_root / "sidecar" / "entrypoint.py")],
    pathex=[str(project_root)],
    binaries=[],
    datas=[],
    hiddenimports=["cv2", "numpy", "djitellopy"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["tkinter", "matplotlib", "pytest"],
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
