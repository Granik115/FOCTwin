from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication

from foctwin import __version__
from foctwin.theme import stylesheet
from foctwin.ui import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("FOCTwin")
    app.setApplicationVersion(__version__)
    app.setOrganizationName("FOCTwin")
    app.setStyleSheet(stylesheet())
    icon = QIcon(str(Path(__file__).with_name("resources") / "icon.ico"))
    if not icon.isNull():
        app.setWindowIcon(icon)
    window = MainWindow()
    window.show()
    return app.exec()
