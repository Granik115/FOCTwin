$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

& .venv\Scripts\python.exe -m unittest discover -s tests -v
& .venv\Scripts\python.exe -m ruff check src tests
& .venv\Scripts\pyinstaller.exe --noconfirm packaging\FOCTwin.spec
