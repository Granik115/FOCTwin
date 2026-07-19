from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class MatlabBackend:
    """Stable file-based boundary around MATLAB R2022b Engine calls."""

    def __init__(self, matlab_root: str | Path) -> None:
        self.matlab_root = Path(matlab_root).resolve()
        self.engine = None

    @property
    def available(self) -> bool:
        try:
            import matlab.engine  # noqa: F401
        except ImportError:
            return False
        return True

    @property
    def connected(self) -> bool:
        return self.engine is not None

    def start(self) -> None:
        if self.connected:
            return
        try:
            import matlab.engine
        except ImportError as exc:
            raise RuntimeError("MATLAB Engine for Python is not installed") from exc
        self.engine = matlab.engine.start_matlab("-nodesktop -nosplash")
        self.engine.addpath(self.engine.genpath(str(self.matlab_root)), nargout=0)

    def stop(self) -> None:
        if self.engine is not None:
            self.engine.quit()
            self.engine = None

    def simulate(self, request: dict[str, Any], work_dir: str | Path) -> dict[str, Any]:
        return self._run_file_api("simulate", request, work_dir)

    def tune(self, request: dict[str, Any], work_dir: str | Path) -> dict[str, Any]:
        return self._run_file_api("tune", request, work_dir)

    def _run_file_api(
        self,
        operation: str,
        request: dict[str, Any],
        work_dir: str | Path,
    ) -> dict[str, Any]:
        if not self.connected or self.engine is None:
            raise RuntimeError("MATLAB backend is not connected")
        work_dir = Path(work_dir).resolve()
        work_dir.mkdir(parents=True, exist_ok=True)
        request_path = work_dir / f"{operation}_request.json"
        result_path = work_dir / f"{operation}_result.json"
        request_path.write_text(json.dumps(request, ensure_ascii=False, indent=2), encoding="utf-8")
        entry_point = getattr(self.engine, f"foctwin_{operation}")
        entry_point(str(request_path), str(result_path), nargout=0)
        return json.loads(result_path.read_text(encoding="utf-8"))
