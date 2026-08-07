from pathlib import Path

root = Path.cwd().resolve()

a = Analysis(
    [str(root / "src" / "foctwin" / "__main__.py")],
    pathex=[str(root / "src")],
    binaries=[],
    datas=[
        (str(root / "src" / "foctwin" / "resources" / "icon.png"), "foctwin/resources"),
        (str(root / "src" / "foctwin" / "resources" / "icon.ico"), "foctwin/resources"),
        (str(root / "profiles"), "profiles"),
        (str(root / "matlab"), "matlab"),
    ],
    hiddenimports=["keyring.backends.Windows"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="FOCTwin",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    icon=str(root / "src" / "foctwin" / "resources" / "icon.ico"),
)
coll = COLLECT(a.binaries, a.datas, exe, name="FOCTwin")
