"""Shared MolPlayer/Xray_labs-derived FOCTwin theme."""

ACCENT_FRAME = "#3e80a3"
ACCENT_GLOW = "#00bfff"
ACCENT_TEAL = "#40e0d0"
DEPTH_BLUE = "#215175"
BG_DARK = "#0f141b"
BG_SIDEBAR = "#0a111c"
BG_PANEL = "#0a1a2e"
BG_TRACK = "#12233a"
BG_SELECTED = "#1e3a5f"
TEXT_PRIMARY = "#e8f4ff"
TEXT_SECONDARY = "#a8d4f0"
TEXT_MUTED = "#5c7a9a"
ERROR = "#ff6b6b"
WARNING = "#f2c14e"
SUCCESS = "#40e0d0"


def stylesheet() -> str:
    return f"""
    QMainWindow, QDialog, QWidget {{
        background: {BG_DARK}; color: {TEXT_PRIMARY};
        font-family: "Segoe UI"; font-size: 10pt;
    }}
    QFrame#sidebar {{ background: {BG_SIDEBAR}; border-right: 1px solid {DEPTH_BLUE}; }}
    QFrame#panel, QGroupBox {{
        background: {BG_PANEL}; border: 1px solid {DEPTH_BLUE}; border-radius: 5px;
    }}
    QGroupBox {{ margin-top: 10px; padding: 12px 8px 8px 8px; font-weight: 600; }}
    QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; }}
    QLabel#pageTitle {{ font-size: 18pt; font-weight: 700; color: {TEXT_PRIMARY}; }}
    QLabel#hint {{ color: {TEXT_SECONDARY}; }}
    QLabel#danger {{ color: {ERROR}; font-weight: 700; }}
    QPushButton {{
        background: {DEPTH_BLUE}; border: 1px solid {ACCENT_FRAME}; border-radius: 4px;
        padding: 6px 12px; font-weight: 600;
    }}
    QPushButton:hover {{ background: {ACCENT_FRAME}; }}
    QPushButton:pressed {{ background: {ACCENT_GLOW}; color: {BG_DARK}; }}
    QPushButton#dangerButton {{ background: #5a2028; border-color: {ERROR}; }}
    QPushButton#dangerButton:hover {{ background: {ERROR}; color: {BG_DARK}; }}
    QLineEdit, QPlainTextEdit, QTextEdit, QSpinBox, QDoubleSpinBox, QComboBox,
    QTableWidget, QListWidget {{
        background: {BG_TRACK}; color: {TEXT_PRIMARY}; border: 1px solid {ACCENT_FRAME};
        border-radius: 3px; padding: 4px;
    }}
    QListWidget::item {{ padding: 9px 8px; border-radius: 4px; }}
    QListWidget::item:selected {{ background: {BG_SELECTED}; border-left: 3px solid {ACCENT_GLOW}; }}
    QHeaderView::section {{
        background: {BG_SELECTED}; color: {TEXT_PRIMARY}; padding: 6px;
        border: 0; border-right: 1px solid {DEPTH_BLUE};
    }}
    QTabBar::tab {{ background: {BG_TRACK}; padding: 7px 14px; }}
    QTabBar::tab:selected {{ background: {DEPTH_BLUE}; color: {ACCENT_GLOW}; }}
    QStatusBar {{ background: {BG_SIDEBAR}; color: {TEXT_SECONDARY}; }}
    QToolTip {{ background: {BG_PANEL}; color: {TEXT_PRIMARY}; border: 1px solid {ACCENT_GLOW}; }}
    """

