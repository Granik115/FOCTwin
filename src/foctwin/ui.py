from __future__ import annotations

import json
import math
import threading
import time
from bisect import bisect_left
from collections import deque
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import ClassVar

from PySide6.QtCore import QObject, QSettings, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QCloseEvent, QFont
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QStackedWidget,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from foctwin import __version__
from foctwin.domain import (
    MotionMode,
    MotorProfile,
    SafetyGuard,
    SafetyLimits,
    TelemetrySample,
    TorqueMode,
)
from foctwin.friction import (
    FRICTION_DEFAULT_RECOVERY_ATTEMPTS,
    FRICTION_DIRECT_VOLTAGE_SENTINEL,
    FRICTION_MAX_AUTOMATIC_POSITIONS,
    FRICTION_MAX_CURRENT_TRIP_A,
    FRICTION_MAX_MAP_PASSES,
    FRICTION_MAX_POSITION_BIN_WIDTH_RAD,
    FRICTION_MAX_RECOVERY_ATTEMPTS,
    FRICTION_MAX_TARGET_VELOCITY_RAD_S,
    FRICTION_MAX_VELOCITY_LIMIT_RAD_S,
    FRICTION_MAX_VOLTAGE_LIMIT_V,
    FRICTION_MIN_CURRENT_TRIP_A,
    FRICTION_MIN_POSITION_BIN_WIDTH_RAD,
    FRICTION_MIN_TARGET_VELOCITY_RAD_S,
    FRICTION_MIN_VOLTAGE_LIMIT_V,
    FRICTION_MONITOR_MASK,
    ActuatorPulseResult,
    BaselineDiagnostic,
    FrictionAction,
    FrictionEstimate,
    FrictionExperiment,
    FrictionPhase,
    FrictionPointResult,
    FrictionTestConfig,
    PositionFrictionObservation,
    PositioningResult,
)
from foctwin.matlab_backend import MatlabBackend
from foctwin.project_store import ProjectStore
from foctwin.protocol import (
    MONITOR_FIELDS,
    CommanderProtocol,
    CommanderResponse,
    is_monitor_candidate,
    parse_commander_response,
    parse_monitor_line,
)
from foctwin.scenario import ScenarioCompiler, ScenarioError
from foctwin.serial_device import SerialDevice
from foctwin.telemetry import TelemetryRecorder, TelemetryStatistics, monitor_stale_timeout

try:
    import pyqtgraph as pg
except ImportError:  # pragma: no cover
    pg = None


class DeviceSignals(QObject):
    line = Signal(float, str)
    state = Signal(bool, str)
    matlab_state = Signal(bool, str)


def spin(
    value: float,
    minimum: float = -1e9,
    maximum: float = 1e9,
    decimals: int = 6,
    step: float | None = None,
) -> QDoubleSpinBox:
    widget = QDoubleSpinBox()
    widget.setDecimals(decimals)
    widget.setRange(minimum, maximum)
    if step is not None:
        widget.setSingleStep(step)
    widget.setValue(value)
    widget.setKeyboardTracking(False)
    return widget


def titled_page(title: str, hint: str) -> tuple[QWidget, QVBoxLayout]:
    page = QWidget()
    layout = QVBoxLayout(page)
    title_label = QLabel(title)
    title_label.setObjectName("pageTitle")
    hint_label = QLabel(hint)
    hint_label.setObjectName("hint")
    hint_label.setWordWrap(True)
    layout.addWidget(title_label)
    layout.addWidget(hint_label)
    return page, layout


def table(headers: list[str], rows: int = 0) -> QTableWidget:
    widget = QTableWidget(rows, len(headers))
    widget.setHorizontalHeaderLabels(headers)
    widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    widget.verticalHeader().setVisible(False)
    widget.setAlternatingRowColors(True)
    return widget


class MainWindow(QMainWindow):
    SETTINGS_KEY = "manual/startup-configuration-v1"
    COMMAND_INTERVAL_MS = 45
    PID_FIELDS = ("p", "i", "d", "ramp", "lpf")
    PID_ROW_BY_FIELD: ClassVar[dict[str, int]] = {
        field: row for row, field in enumerate(PID_FIELDS)
    }
    PID_LIMIT_BINDINGS: ClassVar[dict[str, str]] = {
        "angle": "velocity_rad_s",
        "velocity": "current_a",
        "current_q": "voltage_v",
        "current_d": "voltage_v",
    }
    NAVIGATION = (
        "Обзор",
        "Ручное управление",
        "Идентификация",
        "Виртуальный тюнинг",
        "Доводка на моторе",
        "Анализ данных",
        "Консоль и сценарии",
        "Профили",
        "Журнал",
    )

    def __init__(self, settings: QSettings | None = None) -> None:
        super().__init__()
        self.setWindowTitle(f"FOCTwin {__version__} — Identify. Simulate. Tune.")
        self.resize(1200, 800)
        self.setMinimumSize(900, 600)
        self.profile = MotorProfile()
        self.protocol = CommanderProtocol(self.profile.command_id)
        self.device = SerialDevice(self.protocol)
        self.guard = SafetyGuard(self.profile.safety)
        self.settings = settings or QSettings(
            QSettings.Format.IniFormat,
            QSettings.Scope.UserScope,
            "FOCTwin",
            "FOCTwin",
        )
        self._settings_loading = False
        self.project: ProjectStore | None = None
        self.matlab = MatlabBackend(Path(__file__).resolve().parents[3] / "matlab")
        self.signals = DeviceSignals()
        self.signals.line.connect(self._on_device_line)
        self.signals.state.connect(self._on_device_state)
        self.signals.matlab_state.connect(self._on_matlab_state)
        self.device.on_line = self.signals.line.emit
        self.device.on_state = self.signals.state.emit
        self.monitor_mask = "1111111"
        self.telemetry_statistics = TelemetryStatistics()
        self.telemetry_recorder = TelemetryRecorder()
        self.friction_recorder = TelemetryRecorder()
        self._friction_experiment: FrictionExperiment | None = None
        self._friction_experiment_id: int | None = None
        self._friction_telemetry_paths: list[str] = []
        self._friction_restore_commands: list[str] = []
        self._friction_restore_mask = self.monitor_mask
        self._friction_last_estimate: FrictionEstimate | None = None
        self._friction_last_sample_sequence = 0
        self._friction_recovery_started_at: float | None = None
        self._friction_recovery_sound_enabled = True
        self._friction_recovery_alerted = False
        self._friction_resume_pending = False
        self._telemetry_sequence = 0
        self._rejected_telemetry_count = 0
        self._telemetry_series: dict[str, tuple[list[float], list[float]]] = {
            name: ([], []) for name in MONITOR_FIELDS
        }
        self._last_sample: TelemetrySample | None = None
        self._last_telemetry_received_at: float | None = None
        self._monitor_configured_at: float | None = None
        self._last_monitor_restart_at = 0.0
        self._monitor_restart_count = 0
        self._monitor_recovery_attempt = 0
        self._transport_recovery_count = 0
        self._transport_recovery_in_progress = False
        self._monitoring_requested = False
        self._displayed_telemetry_sequence = 0
        self._reported_recorder_error: str | None = None
        self._connection_requested = False
        self._connecting = False
        self._pwm_requested = False
        self._safety_latched = False
        self._configuration_apply_in_progress = False
        self._device_limit_copy_pending: set[str] = set()
        self._command_queue: deque[str | Callable[[], None]] = deque()
        self._started_at = time.monotonic()
        self._build_actions()
        self._build_shell()
        self._build_pages()
        self._command_timer = QTimer(self)
        self._command_timer.setSingleShot(True)
        self._command_timer.setInterval(self.COMMAND_INTERVAL_MS)
        self._command_timer.timeout.connect(self._dispatch_next_command)
        self._settings_save_timer = QTimer(self)
        self._settings_save_timer.setSingleShot(True)
        self._settings_save_timer.setInterval(250)
        self._settings_save_timer.timeout.connect(self._save_user_settings)
        self._restore_user_settings()
        self._connect_settings_persistence()
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setInterval(1000)
        self._reconnect_timer.timeout.connect(self._reconnect_if_needed)
        self._reconnect_timer.start()
        self._telemetry_ui_timer = QTimer(self)
        self._telemetry_ui_timer.setInterval(100)
        self._telemetry_ui_timer.timeout.connect(self._refresh_telemetry_ui)
        self._telemetry_ui_timer.start()
        self._telemetry_watchdog_timer = QTimer(self)
        self._telemetry_watchdog_timer.setInterval(500)
        self._telemetry_watchdog_timer.timeout.connect(self._check_telemetry_health)
        self._telemetry_watchdog_timer.start()
        self._friction_timer = QTimer(self)
        self._friction_timer.setInterval(50)
        self._friction_timer.timeout.connect(self._advance_friction_experiment)
        self._friction_timer.start()
        self._refresh_status()

    def _build_actions(self) -> None:
        toolbar = QToolBar("Проект")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        new_project = QAction("Новый проект", self)
        new_project.triggered.connect(self._new_project)
        open_project = QAction("Открыть проект", self)
        open_project.triggered.connect(self._open_project)
        matlab = QAction("Запустить MATLAB", self)
        matlab.triggered.connect(self._start_matlab)
        toolbar.addActions((new_project, open_project))
        toolbar.addSeparator()
        toolbar.addAction(matlab)
        toolbar.addSeparator()
        emergency = QAction("АВАРИЙНЫЙ СТОП", self)
        emergency.triggered.connect(self._emergency_stop)
        toolbar.addAction(emergency)

    def _build_shell(self) -> None:
        central = QWidget()
        shell = QHBoxLayout(central)
        shell.setContentsMargins(0, 0, 0, 0)
        self.navigation = QListWidget()
        self.navigation.setFixedWidth(230)
        self.navigation.addItems(self.NAVIGATION)
        self.navigation.setCurrentRow(0)
        self.navigation.setObjectName("navigation")
        self.stack = QStackedWidget()
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        side_layout = QVBoxLayout(sidebar)
        logo = QLabel("FOC<span style='color:#00bfff'>Twin</span>")
        logo.setTextFormat(Qt.TextFormat.RichText)
        logo.setFont(QFont("Segoe UI", 20, QFont.Weight.Bold))
        side_layout.addWidget(logo)
        side_layout.addWidget(QLabel("Полный контроль цифрового двойника"))
        side_layout.addSpacing(12)
        side_layout.addWidget(self.navigation, 1)
        self.side_connection = QLabel("● Мотор отключён")
        self.side_matlab = QLabel("● MATLAB не запущен")
        self.side_project = QLabel("Проект не открыт")
        self.side_project.setWordWrap(True)
        side_layout.addWidget(self.side_connection)
        side_layout.addWidget(self.side_matlab)
        side_layout.addWidget(self.side_project)
        shell.addWidget(sidebar)
        shell.addWidget(self.stack, 1)
        self.navigation.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar())

    def _build_pages(self) -> None:
        builders = (
            self._overview_page,
            self._manual_page,
            self._identification_page,
            self._virtual_tuning_page,
            self._real_tuning_page,
            self._analysis_page,
            self._console_page,
            self._profiles_page,
            self._log_page,
        )
        for builder in builders:
            self.stack.addWidget(builder())

    def _overview_page(self) -> QWidget:
        page, layout = titled_page(
            "Обзор",
            "Состояние проекта, реального привода, MATLAB и последней принятой конфигурации.",
        )
        warning = QLabel(
            "ВНИМАНИЕ: прошивка не имеет подтверждённого heartbeat. При потере USB гарантированно "
            "снять PWM программно невозможно — держите питание доступным для ручного отключения."
        )
        warning.setObjectName("danger")
        warning.setWordWrap(True)
        layout.addWidget(warning)
        status_group = QGroupBox("Состояние")
        status_grid = QGridLayout(status_group)
        self.dashboard_project = QLabel("Не открыт")
        self.dashboard_motor = QLabel("Отключён")
        self.dashboard_matlab = QLabel("Не запущен")
        self.dashboard_profile = QLabel(self.profile.name)
        for row, (label, value) in enumerate(
            (
                ("Проект", self.dashboard_project),
                ("Реальный мотор", self.dashboard_motor),
                ("MATLAB R2022b", self.dashboard_matlab),
                ("Профиль", self.dashboard_profile),
            )
        ):
            status_grid.addWidget(QLabel(label), row, 0)
            status_grid.addWidget(value, row, 1)
        layout.addWidget(status_group)
        workflow = QGroupBox("Рабочий процесс")
        flow = QHBoxLayout(workflow)
        for number, title in (
            ("1", "Запись и идентификация"),
            ("2", "Тюнинг цифрового двойника"),
            ("3", "Ограниченная доводка"),
        ):
            frame = QFrame()
            frame.setObjectName("panel")
            frame_layout = QVBoxLayout(frame)
            label = QLabel(f"{number}. {title}")
            label.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
            frame_layout.addWidget(label)
            frame_layout.addWidget(QLabel("Открыть соответствующий раздел слева"))
            flow.addWidget(frame)
        layout.addWidget(workflow)
        layout.addStretch(1)
        return page

    def _manual_page(self) -> QWidget:
        page, layout = titled_page(
            "Ручное управление",
            "Полный доступ к Commander, подтверждаемым параметрам платы, телеметрии и записи.",
        )
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setMinimumWidth(1040)
        left = QWidget()
        left.setMinimumWidth(410)
        left_layout = QVBoxLayout(left)

        connection = QGroupBox("Serial / Commander")
        connection_form = QFormLayout(connection)
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.addItem("COM3")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(("115200", "230400", "460800", "921600"))
        self.device_id_edit = QLineEdit("A")
        self.device_id_edit.setMaxLength(1)
        connection_buttons = QWidget()
        connection_buttons_layout = QHBoxLayout(connection_buttons)
        connection_buttons_layout.setContentsMargins(0, 0, 0, 0)
        self.connect_button = QPushButton("Подключить")
        self.connect_button.clicked.connect(self._toggle_connection)
        refresh_ports = QPushButton("Обновить порты")
        refresh_ports.clicked.connect(self._refresh_ports)
        connection_buttons_layout.addWidget(self.connect_button)
        connection_buttons_layout.addWidget(refresh_ports)
        self.safe_connect_checkbox = QCheckBox("При подключении сначала отправлять AE0")
        self.safe_connect_checkbox.setChecked(True)
        self.auto_reconnect_checkbox = QCheckBox("Автопереподключение")
        self.auto_reconnect_checkbox.setChecked(True)
        self.connection_details = QLabel("Не подключено")
        self.connection_details.setWordWrap(True)
        connection_form.addRow("COM-порт", self.port_combo)
        connection_form.addRow("Скорость", self.baud_combo)
        connection_form.addRow("ID мотора", self.device_id_edit)
        connection_form.addRow(self.safe_connect_checkbox)
        connection_form.addRow(self.auto_reconnect_checkbox)
        connection_form.addRow(connection_buttons)
        connection_form.addRow(self.connection_details)
        left_layout.addWidget(connection)

        control = QGroupBox("Режим и цель")
        control_form = QFormLayout(control)
        self.motion_combo = QComboBox()
        self.motion_combo.addItem("Положение", MotionMode.ANGLE)
        self.motion_combo.addItem("Скорость", MotionMode.VELOCITY)
        self.motion_combo.addItem("Момент", MotionMode.TORQUE)
        self.motion_combo.addItem("Скорость open-loop", MotionMode.VELOCITY_OPEN_LOOP)
        self.motion_combo.addItem("Положение open-loop", MotionMode.ANGLE_OPEN_LOOP)
        self.torque_combo = QComboBox()
        self.torque_combo.addItem("Voltage", TorqueMode.VOLTAGE)
        self.torque_combo.addItem("FOC Current", TorqueMode.FOC_CURRENT)
        self.torque_combo.addItem("DC Current", TorqueMode.DC_CURRENT)
        self.target_spin = spin(0.0, -100000, 100000)
        self.apply_modes_button = QPushButton("Применить режимы и всю конфигурацию")
        self.apply_modes_button.setToolTip(
            "Отправляет лимиты, PID/LPF, режимы, цель и настройки мониторинга. PWM не включает."
        )
        self.apply_modes_button.clicked.connect(self._apply_modes)
        send_target = QPushButton("Отправить цель")
        send_target.clicked.connect(self._send_target)
        self.enable_pwm_button = QPushButton("Включить PWM")
        self.enable_pwm_button.clicked.connect(self._enable_pwm)
        disable = QPushButton("Отключить PWM")
        disable.setObjectName("dangerButton")
        disable.clicked.connect(self._emergency_stop)
        self.pwm_state_label = QLabel("PWM: неизвестно")
        control_buttons = QWidget()
        control_buttons_layout = QGridLayout(control_buttons)
        control_buttons_layout.setContentsMargins(0, 0, 0, 0)
        control_buttons_layout.addWidget(self.apply_modes_button, 0, 0)
        control_buttons_layout.addWidget(send_target, 0, 1)
        control_buttons_layout.addWidget(self.enable_pwm_button, 1, 0)
        control_buttons_layout.addWidget(disable, 1, 1)
        control_buttons_layout.setColumnStretch(0, 1)
        control_buttons_layout.setColumnStretch(1, 1)
        self.allow_live_changes_checkbox = QCheckBox("Разрешить изменения при включённом PWM")
        self.allow_live_changes_checkbox.setToolTip(
            "Разрешает изменять режимы, PID/LPF и лимиты устройства при активном PWM."
        )
        self.allow_live_changes_checkbox.setChecked(False)
        control_form.addRow("Контур движения", self.motion_combo)
        control_form.addRow("Контур момента", self.torque_combo)
        control_form.addRow("Цель", self.target_spin)
        control_form.addRow(control_buttons)
        control_form.addRow(self.pwm_state_label)
        control_form.addRow(self.allow_live_changes_checkbox)
        left_layout.addWidget(control)

        device_limits = QGroupBox("Ограничения внутри SimpleFOC")
        device_limits_layout = QGridLayout(device_limits)
        device_limits_layout.addWidget(QLabel("Отпр."), 0, 0)
        device_limits_layout.addWidget(QLabel("Параметр"), 0, 1)
        device_limits_layout.addWidget(QLabel("Значение"), 0, 2)
        device_limits_layout.addWidget(QLabel("Ответ платы"), 0, 3)
        self.device_limit_checks: dict[str, QCheckBox] = {}
        self.device_limit_spins: dict[str, QDoubleSpinBox] = {}
        self.device_limit_confirmed: dict[str, QLabel] = {}
        device_limit_rows = (
            ("current_a", "Ток, А", 10.0, 0.001, 100.0),
            ("voltage_v", "Напряжение, В", 20.0, 0.001, 100.0),
            ("velocity_rad_s", "Скорость, рад/с", 0.7, 0.000001, 1000.0),
        )
        for row_index, (key, label, value, minimum, maximum) in enumerate(device_limit_rows, start=1):
            selected = QCheckBox()
            value_widget = spin(value, minimum, maximum)
            confirmed = QLabel("не считано")
            self.device_limit_checks[key] = selected
            self.device_limit_spins[key] = value_widget
            self.device_limit_confirmed[key] = confirmed
            device_limits_layout.addWidget(selected, row_index, 0)
            device_limits_layout.addWidget(QLabel(label), row_index, 1)
            device_limits_layout.addWidget(value_widget, row_index, 2)
            device_limits_layout.addWidget(confirmed, row_index, 3)
        device_limit_buttons = QHBoxLayout()
        read_limits = QPushButton("Считать с платы")
        read_limits.clicked.connect(lambda: self._read_device_limits(copy_to_inputs=True))
        apply_device_limits = QPushButton("Отправить выбранные")
        apply_device_limits.clicked.connect(self._apply_device_limits)
        device_limit_buttons.addWidget(read_limits)
        device_limit_buttons.addWidget(apply_device_limits)
        device_limits_layout.addLayout(device_limit_buttons, 4, 0, 1, 4)
        current_note = QLabel(
            "Важно: ALC1 действительно оставляет плате предел 1 А. В Voltage-режиме с известным "
            "сопротивлением это также предел выхода скоростного PI и его может не хватить для страгивания."
        )
        current_note.setObjectName("danger")
        current_note.setWordWrap(True)
        device_limits_layout.addWidget(current_note, 5, 0, 1, 4)
        left_layout.addWidget(device_limits)

        limits = QGroupBox("Программные аварийные пороги FOCTwin")
        limits_form = QFormLayout(limits)
        self.current_limit = spin(self.profile.safety.current_a, 0.001, 100.0)
        self.voltage_limit = spin(self.profile.safety.voltage_v, 0.001, 100.0)
        self.velocity_limit = spin(self.profile.safety.velocity_rad_s, 0.000001, 1000.0)
        self.angle_min = spin(self.profile.safety.angle_min_rad)
        self.angle_max = spin(self.profile.safety.angle_max_rad)
        self.software_guard_enabled = QCheckBox("Останавливать PWM при нарушении телеметрии")
        self.software_guard_enabled.setChecked(True)
        apply_limits = QPushButton("Применить только программные пороги")
        apply_limits.clicked.connect(self._apply_software_limits)
        limits_form.addRow("Ток, А", self.current_limit)
        limits_form.addRow("Напряжение, В", self.voltage_limit)
        limits_form.addRow("Скорость, рад/с", self.velocity_limit)
        limits_form.addRow("Координата min, рад", self.angle_min)
        limits_form.addRow("Координата max, рад", self.angle_max)
        limits_form.addRow(self.software_guard_enabled)
        limits_form.addRow(apply_limits)
        left_layout.addWidget(limits)
        left_layout.addStretch(1)

        right = QWidget()
        right.setMinimumWidth(600)
        right_layout = QVBoxLayout(right)
        pid_group = QGroupBox("Контуры SimpleFOC: PID / LPF")
        pid_layout = QVBoxLayout(pid_group)
        self.pid_tabs = QTabWidget()
        self.pid_tables: dict[str, QTableWidget] = {}
        self.pid_tab_loops: list[str] = []
        for loop, title, params in (
            ("angle", "Положение", self.profile.angle),
            ("velocity", "Скорость", self.profile.velocity),
            ("current_q", "Ток Q", self.profile.current_q),
            ("current_d", "Ток D", self.profile.current_d),
        ):
            pid_table = table(("Параметр", "Значение"), 5)
            values = (
                ("P", params.p),
                ("I", params.i),
                ("D", params.d),
                ("Output ramp", params.output_ramp),
                ("LPF Tf", params.lpf_tf),
            )
            for row_index, (name, value) in enumerate(values):
                pid_table.setItem(row_index, 0, QTableWidgetItem(name))
                value_item = QTableWidgetItem(f"{value:g}")
                pid_table.setItem(row_index, 1, value_item)
            self.pid_tables[loop] = pid_table
            self.pid_tab_loops.append(loop)
            self.pid_tabs.addTab(pid_table, title)
        pid_layout.addWidget(self.pid_tabs)
        pid_limit_note = QLabel(
            "Output limit задаётся только в блоке «Ограничения внутри SimpleFOC»: "
            "Положение → Скорость (ALV), Скорость → Ток (ALC), Ток Q/D → Напряжение (ALU). "
            "Kc отсутствует в прошивке и остаётся параметром модели Simulink."
        )
        pid_limit_note.setObjectName("hint")
        pid_limit_note.setWordWrap(True)
        pid_layout.addWidget(pid_limit_note)
        pid_buttons = QGridLayout()
        read_pid = QPushButton("Считать выбранный контур")
        read_pid.clicked.connect(self._read_selected_pid)
        apply_pid = QPushButton("Применить выбранный контур")
        apply_pid.clicked.connect(self._apply_selected_pid)
        pid_buttons.addWidget(read_pid, 0, 0)
        pid_buttons.addWidget(apply_pid, 0, 1)
        pid_buttons.setColumnStretch(0, 1)
        pid_buttons.setColumnStretch(1, 1)
        pid_layout.addLayout(pid_buttons)
        right_layout.addWidget(pid_group)

        monitor = QGroupBox("Поток телеметрии")
        monitor_layout = QGridLayout(monitor)
        monitor_labels = (
            ("target", "Цель"),
            ("voltage_q_v", "Uq"),
            ("voltage_d_v", "Ud"),
            ("current_q_a", "Iq"),
            ("current_d_a", "Id"),
            ("velocity_rad_s", "Скорость"),
            ("angle_rad", "Угол"),
        )
        self.monitor_checks: dict[str, QCheckBox] = {}
        self.telemetry_values: dict[str, QLabel] = {}
        signal_columns = 2
        for index, (key, label) in enumerate(monitor_labels):
            checkbox = QCheckBox(label)
            checkbox.setChecked(True)
            value_label = QLabel("—")
            self.monitor_checks[key] = checkbox
            self.telemetry_values[key] = value_label
            row = index // signal_columns
            column = (index % signal_columns) * 2
            monitor_layout.addWidget(checkbox, row, column)
            monitor_layout.addWidget(value_label, row, column + 1)
        self.monitor_downsample_spin = QSpinBox()
        self.monitor_downsample_spin.setRange(1, 100000)
        self.monitor_downsample_spin.setValue(20)
        apply_monitor = QPushButton("Применить мониторинг")
        apply_monitor.clicked.connect(self._apply_monitoring)
        self.monitor_stats_label = QLabel("0 отсчётов · 0 Гц · jitter 0 мс")
        self.monitor_stats_label.setWordWrap(True)
        self.monitor_health_label = QLabel("Поток: ещё не настроен")
        self.monitor_health_label.setWordWrap(True)
        self.restart_monitor_button = QPushButton("Перезапустить поток")
        self.restart_monitor_button.clicked.connect(self._restart_monitoring)
        self.raw_telemetry_checkbox = QCheckBox("Показывать строки телеметрии в сырой консоли")
        self.raw_telemetry_checkbox.setChecked(False)
        self.raw_telemetry_checkbox.setToolTip(
            "Отключено по умолчанию, чтобы частые строки монитора не перегружали интерфейс."
        )
        self.record_button = QPushButton("Начать запись CSV")
        self.record_button.clicked.connect(self._toggle_recording)
        monitor_control_row = (len(monitor_labels) + signal_columns - 1) // signal_columns
        monitor_layout.addWidget(QLabel("Downsample"), monitor_control_row, 0)
        monitor_layout.addWidget(self.monitor_downsample_spin, monitor_control_row, 1)
        monitor_layout.addWidget(apply_monitor, monitor_control_row, 2, 1, 2)
        monitor_layout.addWidget(self.monitor_stats_label, monitor_control_row + 1, 0, 1, 4)
        monitor_layout.addWidget(self.monitor_health_label, monitor_control_row + 2, 0, 1, 2)
        monitor_layout.addWidget(self.restart_monitor_button, monitor_control_row + 2, 2, 1, 2)
        monitor_layout.addWidget(self.raw_telemetry_checkbox, monitor_control_row + 3, 0, 1, 4)
        monitor_layout.addWidget(self.record_button, monitor_control_row + 4, 0, 1, 4)
        monitor_layout.setColumnStretch(1, 1)
        monitor_layout.setColumnStretch(3, 1)
        right_layout.addWidget(monitor)

        telemetry = QGroupBox("Живая телеметрия")
        telemetry_layout = QVBoxLayout(telemetry)
        plot_controls = QGridLayout()
        self.plot_checks: dict[str, QCheckBox] = {}
        for index, (key, label) in enumerate(monitor_labels):
            checkbox = QCheckBox(label)
            checkbox.setChecked(key in {"angle_rad", "velocity_rad_s"})
            checkbox.toggled.connect(
                lambda checked, signal_name=key: self._on_plot_signal_toggled(signal_name, checked)
            )
            self.plot_checks[key] = checkbox
            plot_controls.addWidget(checkbox, index // 4, index % 4)
        self.plot_window_spin = QSpinBox()
        self.plot_window_spin.setRange(5, 600)
        self.plot_window_spin.setValue(30)
        self.plot_window_spin.setSuffix(" с")
        self.plot_follow_checkbox = QCheckBox("Следовать за временем")
        self.plot_follow_checkbox.setChecked(True)
        clear_plot = QPushButton("Очистить график")
        clear_plot.clicked.connect(self._clear_live_plot)
        plot_controls.addWidget(QLabel("Окно"), 2, 0)
        plot_controls.addWidget(self.plot_window_spin, 2, 1)
        plot_controls.addWidget(self.plot_follow_checkbox, 2, 2)
        plot_controls.addWidget(clear_plot, 2, 3)
        for column in range(4):
            plot_controls.setColumnStretch(column, 1)
        telemetry_layout.addLayout(plot_controls)
        if pg is not None:
            self.live_plot = pg.PlotWidget()
            self.live_plot.showGrid(x=True, y=True, alpha=0.25)
            self.live_plot.addLegend()
            colors = ("#f1f5f9", "#00bfff", "#875cff", "#40e0d0", "#ffb74d", "#ff5b79", "#3ddc97")
            self.telemetry_curves = {
                key: self.live_plot.plot(pen=pg.mkPen(color, width=2), name=label)
                for (key, label), color in zip(monitor_labels, colors, strict=True)
            }
            telemetry_layout.addWidget(self.live_plot)
        else:
            self.live_plot = None
            self.telemetry_curves = {}
            telemetry_layout.addWidget(QLabel("Установите pyqtgraph для живых графиков"))
        right_layout.addWidget(telemetry, 1)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 7)
        splitter.setSizes((480, 900))
        self.manual_splitter = splitter
        self.manual_scroll = QScrollArea()
        self.manual_scroll.setObjectName("manualScroll")
        self.manual_scroll.setWidgetResizable(True)
        self.manual_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.manual_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.manual_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.manual_scroll.setWidget(splitter)
        layout.addWidget(self.manual_scroll, 1)
        self._refresh_ports()
        return page

    def _identification_page(self) -> QWidget:
        page, layout = titled_page(
            "Идентификация цифрового двойника",
            "Двухэтапный опыт: проверка исполнительной части, затем трение на малых скоростях.",
        )
        warning = QLabel(
            "Сначала FOCTwin короткими импульсами повышает непосредственно Uq и немедленно "
            "обнуляет его при первом движении. Отправленная команда, фактический Uq и измеренный "
            "Iq сохраняются раздельно. Скоростной тест начнётся после подтверждённого движения "
            "в обе стороны; отсутствие устойчивого Iq не остановит диагностику, но запретит "
            "принять коэффициенты как физически измеренные. Держите питание доступным."
        )
        warning.setObjectName("danger")
        warning.setWordWrap(True)
        layout.addWidget(warning)

        content = QWidget()
        content_layout = QVBoxLayout(content)
        config_group = QGroupBox("Безопасный двухэтапный план")
        config_grid = QGridLayout(config_group)
        self.friction_evidence_mode = QCheckBox(
            "Доказательный режим 0.3.9: PWM off/on, повторы, фиксированный ALC"
        )
        self.friction_evidence_mode.setChecked(True)
        config_grid.addWidget(self.friction_evidence_mode, 0, 0, 1, 4)
        self.friction_low_speed = spin(
            0.02,
            FRICTION_MIN_TARGET_VELOCITY_RAD_S,
            FRICTION_MAX_TARGET_VELOCITY_RAD_S,
            4,
            0.005,
        )
        self.friction_high_speed = spin(
            0.05,
            FRICTION_MIN_TARGET_VELOCITY_RAD_S,
            FRICTION_MAX_TARGET_VELOCITY_RAD_S,
            4,
            0.005,
        )
        self.friction_current_trip = spin(
            1.0,
            FRICTION_MIN_CURRENT_TRIP_A,
            FRICTION_MAX_CURRENT_TRIP_A,
            3,
            0.1,
        )
        self.friction_voltage_limit = spin(
            12.0,
            FRICTION_MIN_VOLTAGE_LIMIT_V,
            FRICTION_MAX_VOLTAGE_LIMIT_V,
            2,
            0.5,
        )
        self.friction_velocity_limit = spin(
            0.3,
            FRICTION_MIN_TARGET_VELOCITY_RAD_S,
            FRICTION_MAX_VELOCITY_LIMIT_RAD_S,
            3,
            0.05,
        )
        self.friction_angle_min = spin(-3.0, -1e9, 1e9, 3, 0.1)
        self.friction_angle_max = spin(3.0, -1e9, 1e9, 3, 0.1)
        self.friction_pulse_start = spin(0.1, 0.001, 100.0, 3, 0.05)
        self.friction_pulse_step = spin(0.1, 0.001, 100.0, 3, 0.05)
        self.friction_pulse_max = spin(0.5, 0.001, 100.0, 3, 0.1)
        self.friction_pulse_duration = spin(0.5, 0.1, 5.0, 2, 0.1)
        self.friction_actuator_pause = spin(0.7, 0.2, 10.0, 2, 0.1)
        self.friction_baseline = spin(1.0, 0.5, 10.0, 2, 0.1)
        self.friction_movement_threshold = spin(0.001, 0.0001, 0.5, 4, 0.0005)
        self.friction_current_floor = spin(0.01, 0.001, 10.0, 3, 0.005)
        self.friction_breakaway_margin = spin(1.2, 1.01, 2.0, 2, 0.05)
        self.friction_settle = spin(2.0, 1.0, 30.0, 1, 0.5)
        self.friction_measure = spin(4.0, 2.0, 120.0, 1, 0.5)
        self.friction_pause = spin(1.0, 0.5, 30.0, 1, 0.5)
        self.friction_downsample = QSpinBox()
        self.friction_downsample.setRange(5, 100)
        self.friction_downsample.setValue(20)
        self.friction_recoveries = QSpinBox()
        self.friction_recoveries.setRange(0, FRICTION_MAX_RECOVERY_ATTEMPTS)
        self.friction_recoveries.setValue(FRICTION_DEFAULT_RECOVERY_ATTEMPTS)
        self.friction_position_bin = spin(
            0.1,
            FRICTION_MIN_POSITION_BIN_WIDTH_RAD,
            FRICTION_MAX_POSITION_BIN_WIDTH_RAD,
            3,
            0.05,
        )
        self.friction_automatic_positions = QSpinBox()
        self.friction_automatic_positions.setRange(1, FRICTION_MAX_AUTOMATIC_POSITIONS)
        self.friction_automatic_positions.setValue(1)
        self.friction_automatic_position_step = spin(
            1.0,
            -100.0,
            100.0,
            6,
            0.1,
        )
        self.friction_map_passes = QSpinBox()
        self.friction_map_passes.setRange(1, FRICTION_MAX_MAP_PASSES)
        self.friction_map_passes.setValue(2)
        self.friction_position_tolerance = spin(0.005, 0.0001, 0.1, 4, 0.001)
        self.friction_position_voltage_step = spin(0.25, 0.01, 20.0, 3, 0.05)
        self.friction_position_voltage_max = spin(3.0, 0.1, 100.0, 3, 0.25)
        self.friction_position_stall_window = spin(3.0, 1.0, 15.0, 1, 0.5)
        self.friction_position_min_progress = spin(0.002, 0.0001, 0.1, 4, 0.0005)
        self.friction_pwm_off_observation = spin(60.0, 5.0, 120.0, 1, 5.0)
        self.friction_breakaway_repeats = QSpinBox()
        self.friction_breakaway_repeats.setRange(1, 5)
        self.friction_breakaway_repeats.setValue(3)
        self.friction_residual_movement = spin(0.005, 0.0001, 0.1, 4, 0.001)
        self.friction_breakaway_verify = spin(1.0, 0.2, 10.0, 1, 0.2)
        self.friction_velocity_travel = spin(0.2, 0.02, 1.0, 3, 0.05)
        self.friction_fixed_velocity_voltage = spin(3.0, 0.1, 100.0, 3, 0.25)
        self.friction_adaptive_positioning = QCheckBox("Разрешить адаптивное повышение Uq")
        self.friction_adaptive_positioning.setChecked(False)
        self.friction_pole_pairs = QSpinBox()
        self.friction_pole_pairs.setRange(1, 100)
        self.friction_pole_pairs.setValue(self.profile.pole_pairs)
        self.friction_electrical_divisions = QSpinBox()
        self.friction_electrical_divisions.setRange(4, 32)
        self.friction_electrical_divisions.setValue(8)
        self.friction_position_validation = QCheckBox("Проверить позиционные шаги ±")
        self.friction_position_validation.setChecked(True)
        self.friction_position_validation_small = spin(0.1, 0.01, 2.0, 3, 0.05)
        self.friction_position_validation_medium = spin(0.3, 0.01, 2.0, 3, 0.05)
        self.friction_position_validation_large = spin(0.6, 0.01, 2.0, 3, 0.05)
        fields = (
            ("Начальный Uq, В", self.friction_pulse_start),
            ("Шаг Uq, В", self.friction_pulse_step),
            ("Максимальный Uq, В", self.friction_pulse_max),
            ("Импульс Uq, с", self.friction_pulse_duration),
            ("Пауза после импульса, с", self.friction_actuator_pause),
            ("Исходный спокойный ноль, с", self.friction_baseline),
            ("Порог движения, рад", self.friction_movement_threshold),
            ("Порог подтверждения Iq, А", self.friction_current_floor),
            ("Запас над страгиванием", self.friction_breakaway_margin),
            ("Малая скорость, рад/с", self.friction_low_speed),
            ("Большая скорость, рад/с", self.friction_high_speed),
            ("Аварийный измеренный ток, А", self.friction_current_trip),
            ("Предел напряжения, В", self.friction_voltage_limit),
            ("Предел скорости, рад/с", self.friction_velocity_limit),
            ("Координата min, рад", self.friction_angle_min),
            ("Координата max, рад", self.friction_angle_max),
            ("Стабилизация точки, с", self.friction_settle),
            ("Полезное измерение, с", self.friction_measure),
            ("Пауза на нуле, с", self.friction_pause),
            ("Downsample телеметрии", self.friction_downsample),
            ("Автовосстановлений", self.friction_recoveries),
            ("Интервал карты координат, рад", self.friction_position_bin),
            ("Автоматических положений", self.friction_automatic_positions),
            ("Шаг автосмещения, рад", self.friction_automatic_position_step),
            ("Проходов карты туда/обратно", self.friction_map_passes),
            ("Допуск автопозиции, рад", self.friction_position_tolerance),
            ("Шаг повышения Uq автосмещения, В", self.friction_position_voltage_step),
            ("Максимальный Uq автосмещения, В", self.friction_position_voltage_max),
            ("Окно обнаружения остановки, с", self.friction_position_stall_window),
            ("Минимальный прогресс за окно, рад", self.friction_position_min_progress),
            ("PWM отключён: наблюдение, с", self.friction_pwm_off_observation),
            ("Повторов страгивания", self.friction_breakaway_repeats),
            ("Остаточное перемещение, рад", self.friction_residual_movement),
            ("Проверка возврата после Uq, с", self.friction_breakaway_verify),
            ("Длина локальной скорости, рад", self.friction_velocity_travel),
            ("Фиксированный Uq скорости, В", self.friction_fixed_velocity_voltage),
            ("Пар полюсов", self.friction_pole_pairs),
            ("Делений электрического периода", self.friction_electrical_divisions),
            ("Малый позиционный шаг, рад", self.friction_position_validation_small),
            ("Средний позиционный шаг, рад", self.friction_position_validation_medium),
            ("Большой позиционный шаг, рад", self.friction_position_validation_large),
        )
        rows_per_column = (len(fields) + 1) // 2
        for index, (label, widget) in enumerate(fields):
            column = 0 if index < rows_per_column else 2
            row = index if index < rows_per_column else index - rows_per_column
            config_grid.addWidget(QLabel(label), row + 1, column)
            config_grid.addWidget(widget, row + 1, column + 1)
        options_row = rows_per_column + 1
        config_grid.addWidget(self.friction_adaptive_positioning, options_row, 0, 1, 2)
        config_grid.addWidget(self.friction_position_validation, options_row, 2, 1, 2)
        self.friction_duration_label = QLabel()
        self.friction_duration_label.setObjectName("hint")
        self.friction_duration_label.setWordWrap(True)
        config_grid.addWidget(self.friction_duration_label, options_row + 1, 0, 1, 4)
        self.friction_evidence_preset_button = QPushButton(
            "Настроить решающий план: 2 электрических периода"
        )
        self.friction_evidence_preset_button.clicked.connect(
            self._apply_friction_evidence_preset
        )
        config_grid.addWidget(
            self.friction_evidence_preset_button,
            options_row + 2,
            0,
            1,
            4,
        )
        content_layout.addWidget(config_group)

        controls = QGridLayout()
        self.friction_start_button = QPushButton("Запустить полный тест трения")
        self.friction_start_button.clicked.connect(self._start_friction_test)
        self.friction_resume_button = QPushButton("Продолжить из checkpoint")
        self.friction_resume_button.clicked.connect(self._resume_friction_checkpoint)
        self.friction_stop_button = QPushButton("СТОП И ОТКЛЮЧИТЬ PWM")
        self.friction_stop_button.setObjectName("dangerButton")
        self.friction_stop_button.clicked.connect(self._stop_friction_test_by_user)
        self.friction_stop_button.setEnabled(False)
        controls.addWidget(self.friction_start_button, 0, 0)
        controls.addWidget(self.friction_resume_button, 0, 1)
        controls.addWidget(self.friction_stop_button, 0, 2)
        for column in range(3):
            controls.setColumnStretch(column, 1)
        content_layout.addLayout(controls)
        self.friction_status_label = QLabel("Тест не запущен")
        self.friction_status_label.setWordWrap(True)
        content_layout.addWidget(self.friction_status_label)

        actuator_group = QGroupBox("Проверка исполнительной части")
        actuator_layout = QVBoxLayout(actuator_group)
        self.friction_actuator_note = QLabel(
            "Здесь отдельно появятся команда Uq, фактический Uq, измеренный Iq и движение."
        )
        self.friction_actuator_note.setWordWrap(True)
        actuator_layout.addWidget(self.friction_actuator_note)
        self.friction_actuator_table = table(
            [
                "Напр.",
                "Повтор",
                "Координата, рад",
                "Команда Uq, В",
                "Факт. Uq, В",
                "Iq сред., А",
                "Iq пик, А",
                "Δугол, рад",
                "Остаток, рад",
                "Статус",
            ],
            0,
        )
        actuator_layout.addWidget(self.friction_actuator_table)
        content_layout.addWidget(actuator_group)

        self.friction_points_table = table(
            [
                "Автопозиция, рад",
                "Цель, рад/с",
                "Средняя координата, рад",
                "Скорость по углу",
                "Iq измеренный, А",
                "Момент, Н·м",
                "Отсчёты",
                "Статус",
            ],
            len(FrictionTestConfig().targets),
        )
        for row, target in enumerate(FrictionTestConfig().targets):
            self.friction_points_table.setItem(row, 0, QTableWidgetItem("текущая"))
            self.friction_points_table.setItem(row, 1, QTableWidgetItem(f"{target:g}"))
            for column in range(2, self.friction_points_table.columnCount()):
                self.friction_points_table.setItem(row, column, QTableWidgetItem("—"))
        content_layout.addWidget(self.friction_points_table)

        positioning_group = QGroupBox("Диагностика автоматических смещений")
        positioning_layout = QVBoxLayout(positioning_group)
        self.friction_positioning_table = table(
            [
                "№",
                "Старт, рад",
                "Цель, рад",
                "Финиш, рад",
                "Ошибка, рад",
                "Время, с",
                "Uq нач./кон., В",
                "Повышений",
                "Статус",
            ],
            0,
        )
        positioning_layout.addWidget(self.friction_positioning_table)
        content_layout.addWidget(positioning_group)

        position_group = QGroupBox("Карта трения по координате")
        position_layout = QVBoxLayout(position_group)
        self.friction_position_note = QLabel(
            "Каждый скоростной проход будет разбит на интервалы координаты; измеренный Iq "
            "и диагностическая оценка по Uq сохраняются раздельно."
        )
        self.friction_position_note.setWordWrap(True)
        position_layout.addWidget(self.friction_position_note)
        self.friction_position_table = table(
            [
                "Координата, рад",
                "Цель, рад/с",
                "Факт. скорость",
                "Uq, В",
                "Iq, А",
                "Момент по Iq, Н·м",
                "Оценка по Uq, Н·м",
                "Отсчёты",
                "Статус",
            ],
            0,
        )
        position_layout.addWidget(self.friction_position_table)
        content_layout.addWidget(position_group)

        result_group = QGroupBox("Результат аппроксимации")
        result_layout = QVBoxLayout(result_group)
        self.friction_summary_table = table(["Параметр", "Значение", "Единица"], 7)
        summary_rows = (
            ("Coulomb, среднее", "Н·м"),
            ("Coulomb +", "Н·м"),
            ("Coulomb −", "Н·м"),
            ("Вязкое трение", "Н·м·с/рад"),
            ("Страгивание, грубо", "Н·м"),
            ("Асимметрия", "%"),
            ("R²", "—"),
        )
        for row, (name, unit) in enumerate(summary_rows):
            self.friction_summary_table.setItem(row, 0, QTableWidgetItem(name))
            self.friction_summary_table.setItem(row, 1, QTableWidgetItem("—"))
            self.friction_summary_table.setItem(row, 2, QTableWidgetItem(unit))
        result_layout.addWidget(self.friction_summary_table)
        self.friction_result_note = QLabel("После теста здесь появится оценка и её пригодность.")
        self.friction_result_note.setWordWrap(True)
        result_layout.addWidget(self.friction_result_note)
        self.friction_accept_button = QPushButton("Принять оценки трения в профиль")
        self.friction_accept_button.setEnabled(False)
        self.friction_accept_button.clicked.connect(self._accept_friction_estimate)
        result_layout.addWidget(self.friction_accept_button)
        content_layout.addWidget(result_group)

        diagnostic_group = QGroupBox("Диагностический отчёт")
        diagnostic_layout = QVBoxLayout(diagnostic_group)
        self.friction_diagnostic_report = QPlainTextEdit()
        self.friction_diagnostic_report.setReadOnly(True)
        self.friction_diagnostic_report.setMaximumHeight(320)
        self.friction_diagnostic_report.setPlainText(
            "После опыта здесь появятся однозначные выводы по автосмещению, трению, "
            "асимметрии, повторяемости, Iq, телеметрии и готовности данных для модели."
        )
        diagnostic_layout.addWidget(self.friction_diagnostic_report)
        content_layout.addWidget(diagnostic_group)
        content_layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidget(content)
        layout.addWidget(scroll, 1)
        for widget in (
            self.friction_pulse_start,
            self.friction_pulse_step,
            self.friction_pulse_max,
            self.friction_pulse_duration,
            self.friction_actuator_pause,
            self.friction_baseline,
            self.friction_low_speed,
            self.friction_high_speed,
            self.friction_settle,
            self.friction_measure,
            self.friction_pause,
            self.friction_position_bin,
            self.friction_automatic_positions,
            self.friction_automatic_position_step,
            self.friction_map_passes,
            self.friction_position_tolerance,
            self.friction_position_voltage_step,
            self.friction_position_voltage_max,
            self.friction_position_stall_window,
            self.friction_position_min_progress,
            self.friction_pwm_off_observation,
            self.friction_breakaway_repeats,
            self.friction_residual_movement,
            self.friction_breakaway_verify,
            self.friction_velocity_travel,
            self.friction_fixed_velocity_voltage,
            self.friction_pole_pairs,
            self.friction_electrical_divisions,
            self.friction_position_validation_small,
            self.friction_position_validation_medium,
            self.friction_position_validation_large,
            self.friction_map_passes,
            self.friction_position_tolerance,
            self.friction_position_voltage_step,
            self.friction_position_voltage_max,
            self.friction_position_stall_window,
            self.friction_position_min_progress,
            self.friction_pwm_off_observation,
            self.friction_breakaway_repeats,
            self.friction_residual_movement,
            self.friction_breakaway_verify,
            self.friction_velocity_travel,
            self.friction_fixed_velocity_voltage,
            self.friction_pole_pairs,
            self.friction_electrical_divisions,
            self.friction_position_validation_small,
            self.friction_position_validation_medium,
            self.friction_position_validation_large,
        ):
            widget.valueChanged.connect(self._refresh_friction_duration)
        self.friction_evidence_mode.toggled.connect(self._refresh_friction_duration)
        self.friction_adaptive_positioning.toggled.connect(self._refresh_friction_duration)
        self.friction_position_validation.toggled.connect(self._refresh_friction_duration)
        self._refresh_friction_duration()
        return page

    def _friction_config_from_widgets(self) -> FrictionTestConfig:
        return FrictionTestConfig(
            low_velocity_rad_s=self.friction_low_speed.value(),
            high_velocity_rad_s=self.friction_high_speed.value(),
            current_trip_limit_a=self.friction_current_trip.value(),
            voltage_limit_v=self.friction_voltage_limit.value(),
            velocity_limit_rad_s=self.friction_velocity_limit.value(),
            angle_min_rad=self.friction_angle_min.value(),
            angle_max_rad=self.friction_angle_max.value(),
            pulse_start_voltage_v=self.friction_pulse_start.value(),
            pulse_step_voltage_v=self.friction_pulse_step.value(),
            pulse_max_voltage_v=self.friction_pulse_max.value(),
            pulse_duration_s=self.friction_pulse_duration.value(),
            actuator_pause_s=self.friction_actuator_pause.value(),
            baseline_s=self.friction_baseline.value(),
            movement_threshold_rad=self.friction_movement_threshold.value(),
            measured_current_floor_a=self.friction_current_floor.value(),
            breakaway_margin=self.friction_breakaway_margin.value(),
            settle_s=self.friction_settle.value(),
            measure_s=self.friction_measure.value(),
            pause_s=self.friction_pause.value(),
            monitor_downsample=self.friction_downsample.value(),
            max_recovery_attempts=self.friction_recoveries.value(),
            position_bin_width_rad=self.friction_position_bin.value(),
            automatic_position_count=self.friction_automatic_positions.value(),
            automatic_position_step_rad=self.friction_automatic_position_step.value(),
            map_passes=self.friction_map_passes.value(),
            position_tolerance_rad=self.friction_position_tolerance.value(),
            positioning_voltage_step_v=self.friction_position_voltage_step.value(),
            positioning_voltage_max_v=self.friction_position_voltage_max.value(),
            position_stall_window_s=self.friction_position_stall_window.value(),
            position_min_progress_rad=self.friction_position_min_progress.value(),
            evidence_mode=self.friction_evidence_mode.isChecked(),
            pwm_off_observation_s=self.friction_pwm_off_observation.value(),
            breakaway_repeats=self.friction_breakaway_repeats.value(),
            residual_movement_threshold_rad=self.friction_residual_movement.value(),
            breakaway_verify_s=self.friction_breakaway_verify.value(),
            velocity_travel_rad=self.friction_velocity_travel.value(),
            fixed_velocity_voltage_limit_v=self.friction_fixed_velocity_voltage.value(),
            adaptive_positioning_enabled=self.friction_adaptive_positioning.isChecked(),
            pole_pairs=self.friction_pole_pairs.value(),
            electrical_divisions=self.friction_electrical_divisions.value(),
            position_validation_enabled=self.friction_position_validation.isChecked(),
            position_validation_small_rad=self.friction_position_validation_small.value(),
            position_validation_medium_rad=self.friction_position_validation_medium.value(),
            position_validation_large_rad=self.friction_position_validation_large.value(),
        )

    def _synchronize_friction_uq_ceilings(self) -> tuple[str, ...]:
        pulse_max = self.friction_pulse_max.value()
        voltage_limit = self.friction_voltage_limit.value()
        if pulse_max > voltage_limit:
            return ()

        adjustments: list[str] = []
        dependent_ceilings = [
            ("Максимальный Uq автосмещения", self.friction_position_voltage_max),
        ]
        if self.friction_evidence_mode.isChecked():
            dependent_ceilings.append(
                ("Фиксированный Uq скорости", self.friction_fixed_velocity_voltage)
            )
        for label, widget in dependent_ceilings:
            previous = widget.value()
            synchronized = min(max(previous, pulse_max), voltage_limit)
            if math.isclose(previous, synchronized, rel_tol=0.0, abs_tol=1e-12):
                continue
            widget.setValue(synchronized)
            adjustments.append(f"• {label}: {previous:g} → {synchronized:g} В")
        return tuple(adjustments)

    def _set_friction_config_widgets(self, config: FrictionTestConfig) -> None:
        self.friction_low_speed.setValue(config.low_velocity_rad_s)
        self.friction_high_speed.setValue(config.high_velocity_rad_s)
        self.friction_current_trip.setValue(config.current_trip_limit_a)
        self.friction_voltage_limit.setValue(config.voltage_limit_v)
        self.friction_velocity_limit.setValue(config.velocity_limit_rad_s)
        self.friction_angle_min.setValue(config.angle_min_rad)
        self.friction_angle_max.setValue(config.angle_max_rad)
        self.friction_pulse_start.setValue(config.pulse_start_voltage_v)
        self.friction_pulse_step.setValue(config.pulse_step_voltage_v)
        self.friction_pulse_max.setValue(config.pulse_max_voltage_v)
        self.friction_pulse_duration.setValue(config.pulse_duration_s)
        self.friction_actuator_pause.setValue(config.actuator_pause_s)
        self.friction_baseline.setValue(config.baseline_s)
        self.friction_movement_threshold.setValue(config.movement_threshold_rad)
        self.friction_current_floor.setValue(config.measured_current_floor_a)
        self.friction_breakaway_margin.setValue(config.breakaway_margin)
        self.friction_settle.setValue(config.settle_s)
        self.friction_measure.setValue(config.measure_s)
        self.friction_pause.setValue(config.pause_s)
        self.friction_downsample.setValue(config.monitor_downsample)
        self.friction_recoveries.setValue(config.max_recovery_attempts)
        self.friction_position_bin.setValue(config.position_bin_width_rad)
        self.friction_automatic_positions.setValue(config.automatic_position_count)
        self.friction_automatic_position_step.setValue(config.automatic_position_step_rad)
        self.friction_map_passes.setValue(config.map_passes)
        self.friction_position_tolerance.setValue(config.position_tolerance_rad)
        self.friction_position_voltage_step.setValue(config.positioning_voltage_step_v)
        self.friction_position_voltage_max.setValue(config.positioning_voltage_max_v)
        self.friction_position_stall_window.setValue(config.position_stall_window_s)
        self.friction_position_min_progress.setValue(config.position_min_progress_rad)
        self.friction_evidence_mode.setChecked(config.evidence_mode)
        self.friction_pwm_off_observation.setValue(config.pwm_off_observation_s)
        self.friction_breakaway_repeats.setValue(config.breakaway_repeats)
        self.friction_residual_movement.setValue(config.residual_movement_threshold_rad)
        self.friction_breakaway_verify.setValue(config.breakaway_verify_s)
        self.friction_velocity_travel.setValue(config.velocity_travel_rad)
        self.friction_fixed_velocity_voltage.setValue(config.fixed_velocity_voltage_limit_v)
        self.friction_adaptive_positioning.setChecked(config.adaptive_positioning_enabled)
        self.friction_pole_pairs.setValue(config.pole_pairs)
        self.friction_electrical_divisions.setValue(config.electrical_divisions)
        self.friction_position_validation.setChecked(config.position_validation_enabled)
        self.friction_position_validation_small.setValue(
            config.position_validation_small_rad
        )
        self.friction_position_validation_medium.setValue(
            config.position_validation_medium_rad
        )
        self.friction_position_validation_large.setValue(
            config.position_validation_large_rad
        )
        self._refresh_friction_duration()

    def _refresh_friction_duration(self, *_args: object) -> None:
        config = self._friction_config_from_widgets()
        evidence_text = (
            f" Доказательный режим: PWM off {config.pwm_off_observation_s:g} с и PWM on "
            f"{config.baseline_s:g} с в каждой точке; {config.breakaway_repeats} повтора "
            f"страгивания с остатком ≥{config.residual_movement_threshold_rad:g} рад; "
            f"каждая скорость проходит ровно {config.velocity_travel_rad:g} рад при одном "
            f"Uq max={config.fixed_velocity_voltage_limit_v:g} В. Электрический период "
            f"{config.electrical_period_rad:.6g} рад, рекомендуемый шаг "
            f"{config.recommended_electrical_step_rad:.6g} рад. "
            if config.evidence_mode
            else " Режим совместимости 0.3.7 без раздельного доказательства PWM off/on."
        )
        self.friction_duration_label.setText(
            f"Сначала Uq ±{config.pulse_start_voltage_v:g}…±{config.pulse_max_voltage_v:g} В "
            f"с шагом {config.pulse_step_voltage_v:g} В; затем три модуля скорости "
            f"{', '.join(f'{value:g}' for value in config.speed_levels)} рад/с "
            "в обе стороны. "
            f"Карта: интервалы по {config.position_bin_width_rad:g} рад. "
            f"Уникальных положений: {config.automatic_position_count}, проходов: "
            f"{config.map_passes}, измерений по координатам: "
            f"{config.measurement_position_count}, шаг {config.automatic_position_step_rad:+g} рад. "
            f"Автосмещение повышает Uq по {config.positioning_voltage_step_v:g} В до "
            f"{config.positioning_voltage_max_v:g} В при подтверждённом насыщении. "
            + evidence_text
            + f"Худший случай без восстановлений: около {config.estimated_duration_s:.0f} с."
        )
        self.friction_points_table.setRowCount(
            config.measurement_position_count * len(config.targets)
        )
        for position_index in range(config.measurement_position_count):
            for point_index, target in enumerate(config.targets):
                row = position_index * len(config.targets) + point_index
                position_label = (
                    "текущая"
                    if position_index == 0
                    else f"{position_index:+d} × шаг"
                )
                self.friction_points_table.setItem(
                    row,
                    0,
                    QTableWidgetItem(position_label),
                )
                self.friction_points_table.setItem(row, 1, QTableWidgetItem(f"{target:g}"))
                for column in range(2, self.friction_points_table.columnCount()):
                    self.friction_points_table.setItem(row, column, QTableWidgetItem("—"))

    def _apply_friction_evidence_preset(self) -> None:
        self.friction_evidence_mode.setChecked(True)
        self.friction_low_speed.setValue(0.03)
        self.friction_high_speed.setValue(0.2)
        self.friction_velocity_limit.setValue(max(0.3, self.friction_high_speed.value()))
        self.friction_pwm_off_observation.setValue(60.0)
        self.friction_breakaway_repeats.setValue(3)
        self.friction_movement_threshold.setValue(0.001)
        self.friction_residual_movement.setValue(0.005)
        self.friction_breakaway_verify.setValue(1.0)
        self.friction_velocity_travel.setValue(0.2)
        self.friction_fixed_velocity_voltage.setValue(
            min(3.0, self.friction_voltage_limit.value())
        )
        self.friction_position_voltage_max.setValue(
            min(3.0, self.friction_voltage_limit.value())
        )
        self.friction_adaptive_positioning.setChecked(False)
        self.friction_pole_pairs.setValue(self.profile.pole_pairs)
        self.friction_electrical_divisions.setValue(8)
        self.friction_automatic_positions.setValue(17)
        self.friction_automatic_position_step.setValue(
            math.tau
            / self.friction_pole_pairs.value()
            / self.friction_electrical_divisions.value()
        )
        self.friction_map_passes.setValue(2)
        self.friction_position_validation.setChecked(True)
        self.friction_position_validation_small.setValue(0.1)
        self.friction_position_validation_medium.setValue(0.3)
        self.friction_position_validation_large.setValue(0.6)
        self._refresh_friction_duration()

    def _friction_running(self) -> bool:
        experiment = self._friction_experiment
        return bool(
            experiment
            and experiment.phase not in {FrictionPhase.COMPLETE, FrictionPhase.ABORTED}
        )

    def _start_friction_test(self) -> None:
        self._prepare_friction_experiment()

    def _resume_friction_checkpoint(self) -> None:
        if self.project is None:
            QMessageBox.warning(self, "Checkpoint", "Сначала откройте проект FOCTwin")
            return
        try:
            payload = self.project.load_checkpoint("friction")
            if payload is None:
                raise ValueError("В проекте нет checkpoint теста трения")
            if int(payload.get("schema", 0)) != 8:
                raise ValueError(
                    "Checkpoint создан старым алгоритмом и несовместим со схемой 8; "
                    "начните новый комплексный диагностический тест"
                )
            points = [
                FrictionPointResult.from_dict(point)
                for point in payload.get("completed_points", [])
            ]
            actuator_attempts = [
                ActuatorPulseResult.from_dict(attempt)
                for attempt in payload.get("actuator_attempts", [])
            ]
            position_observations = [
                PositionFrictionObservation.from_dict(observation)
                for observation in payload.get("position_observations", [])
            ]
            baseline_diagnostics = [
                BaselineDiagnostic.from_dict(diagnostic)
                for diagnostic in payload.get("baseline_diagnostics", [])
            ]
            positioning_results = [
                PositioningResult.from_dict(result)
                for result in payload.get("positioning_results", [])
            ]
            position_targets = tuple(
                float(value) for value in payload.get("position_targets_rad", ())
            )
            if not position_targets:
                raise ValueError("Checkpoint не содержит автоматические координаты")
            config = FrictionTestConfig.from_dict(payload["config"])
            position_validation_active = bool(
                payload.get("position_validation_active", False)
            )
            position_validation_index = int(
                payload.get("position_validation_index", -1)
            )
            if (
                (
                    len(points) >= len(position_targets) * len(config.targets)
                    and not position_validation_active
                )
                or payload.get("phase") == FrictionPhase.COMPLETE.value
            ):
                raise ValueError("Тест из checkpoint уже завершён")
            experiment_id = payload.get("experiment_id")
            telemetry_paths = [str(path) for path in payload.get("telemetry_paths", [])]
            position_index = int(payload.get("position_index", 0))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            QMessageBox.warning(self, "Checkpoint", str(exc))
            return
        self._set_friction_config_widgets(config)
        self._prepare_friction_experiment(
            completed_points=points,
            actuator_attempts=actuator_attempts,
            position_observations=position_observations,
            baseline_diagnostics=baseline_diagnostics,
            positioning_results=positioning_results,
            experiment_id=int(experiment_id) if experiment_id is not None else None,
            telemetry_paths=telemetry_paths,
            position_targets_rad=position_targets,
            position_index=position_index,
            position_validation_active=position_validation_active,
            position_validation_index=position_validation_index,
            resumed=True,
        )

    def _prepare_friction_experiment(
        self,
        *,
        completed_points: list[FrictionPointResult] | None = None,
        actuator_attempts: list[ActuatorPulseResult] | None = None,
        position_observations: list[PositionFrictionObservation] | None = None,
        baseline_diagnostics: list[BaselineDiagnostic] | None = None,
        positioning_results: list[PositioningResult] | None = None,
        experiment_id: int | None = None,
        telemetry_paths: list[str] | None = None,
        position_targets_rad: tuple[float, ...] | None = None,
        position_index: int = 0,
        point_index: int | None = None,
        position_validation_active: bool = False,
        position_validation_index: int = -1,
        resumed: bool = False,
    ) -> None:
        if self._friction_running():
            QMessageBox.information(self, "Тест трения", "Тест уже выполняется")
            return
        if self.project is None:
            QMessageBox.warning(self, "Тест трения", "Сначала создайте или откройте проект FOCTwin")
            return
        if not self.device.connected:
            QMessageBox.warning(self, "Тест трения", "Сначала подключите мотор")
            return
        if self._pwm_requested:
            QMessageBox.warning(self, "Тест трения", "Перед стартом отключите PWM")
            return
        if self._configuration_apply_in_progress or self._command_queue or self._command_timer.isActive():
            QMessageBox.information(self, "Тест трения", "Дождитесь завершения текущих команд")
            return
        uq_adjustments = self._synchronize_friction_uq_ceilings()
        if uq_adjustments:
            QMessageBox.warning(
                self,
                "Автоматическое согласование Uq",
                "FOCTwin автоматически согласовал зависимые пределы Uq:\n"
                + "\n".join(uq_adjustments)
                + "\n\nОни не могут быть ниже максимального Uq импульсов "
                f"({self.friction_pulse_max.value():g} В) и выше предела напряжения опыта "
                f"({self.friction_voltage_limit.value():g} В). Запуск будет продолжен.",
            )
        config = self._friction_config_from_widgets()
        try:
            config.validate()
            pid_values = {loop: self._pid_values(loop) for loop in self.pid_tables}
        except ValueError as exc:
            QMessageBox.warning(self, "Тест трения", str(exc))
            return
        if self.profile.phase_resistance_ohm <= 0:
            QMessageBox.warning(
                self,
                "Тест трения",
                "Для безопасного двухэтапного опыта в профиле нужно положительное "
                "сопротивление фазы",
            )
            return
        sample = self._last_sample
        if sample is None or self._last_telemetry_received_at is None:
            QMessageBox.warning(self, "Тест трения", "Нет свежей распознанной телеметрии")
            return
        if time.monotonic() - self._last_telemetry_received_at > monitor_stale_timeout(
            self._active_monitor_downsample()
        ):
            QMessageBox.warning(self, "Тест трения", "Телеметрия устарела; дождитесь восстановления")
            return
        if sample.angle_rad is None or sample.current_q_a is None or sample.voltage_q_v is None:
            QMessageBox.warning(
                self,
                "Тест трения",
                "Для preflight нужны угол, Uq и Iq; включите их в мониторинге",
            )
            return
        if not config.angle_min_rad + config.position_margin_rad <= sample.angle_rad <= (
            config.angle_max_rad - config.position_margin_rad
        ):
            QMessageBox.warning(
                self,
                "Тест трения",
                "Текущая координата слишком близка к границе опыта; переместите вал ближе к середине",
            )
            return
        try:
            position_targets = (
                tuple(position_targets_rad)
                if position_targets_rad is not None
                else config.automatic_position_targets(sample.angle_rad)
            )
            if len(position_targets) != config.measurement_position_count:
                raise ValueError("Число сохранённых координат не совпадает с настройкой опыта")
            safe_min = config.angle_min_rad + config.position_margin_rad
            safe_max = config.angle_max_rad - config.position_margin_rad
            if any(not safe_min <= target <= safe_max for target in position_targets):
                raise ValueError(
                    "Сохранённые автоматические координаты больше не помещаются "
                    "в безопасные границы опыта"
                )
        except ValueError as exc:
            QMessageBox.warning(self, "Тест трения", str(exc))
            return
        confirmed, recovery_sound_enabled = self._confirm_friction_start(
            config,
            position_targets,
        )
        if not confirmed:
            return
        self._friction_recovery_sound_enabled = recovery_sound_enabled

        self._apply_friction_limits(config)
        self._save_user_settings()
        manual_mask = self.monitor_mask
        self._friction_restore_mask = manual_mask
        self._friction_restore_commands = [
            self.protocol.disable(),
            self.protocol.phase_resistance(self.profile.phase_resistance_ohm),
        ]
        self._friction_restore_commands.extend(
            self._full_configuration_commands(pid_values, manual_mask)
        )
        self._friction_telemetry_paths = list(telemetry_paths or [])
        self._friction_last_estimate = None
        self.friction_accept_button.setEnabled(False)
        self._clear_friction_results()
        completed = list(completed_points or [])
        attempts = list(actuator_attempts or [])
        observations = list(position_observations or [])
        baselines = list(baseline_diagnostics or [])
        moves = list(positioning_results or [])
        self._render_actuator_attempts(attempts)
        self._render_friction_points(completed)
        self._render_position_observations(observations)
        self._render_positioning_results(moves)
        self._friction_experiment = FrictionExperiment(
            config,
            self.profile.torque_constant_nm_per_a,
            completed,
            phase_resistance_ohm=self.profile.phase_resistance_ohm,
            back_emf_v_per_krpm=self.profile.back_emf_v_per_krpm,
            actuator_attempts=attempts,
            position_observations=observations,
            baseline_diagnostics=baselines,
            positioning_results=moves,
            position_targets_rad=position_targets,
            position_index=position_index,
            point_index=point_index,
            position_validation_active=position_validation_active,
            position_validation_index=position_validation_index,
        )
        self._friction_experiment.seed_angle(
            sample.angle_rad,
            continuous_reference_rad=(
                (
                    position_targets[0]
                    + config.position_validation_offsets[position_validation_index]
                    if position_validation_active
                    and 0
                    <= position_validation_index
                    < len(config.position_validation_offsets)
                    else position_targets[position_index]
                )
                if resumed
                else sample.angle_rad
            ),
        )
        self._friction_experiment_id = experiment_id
        if self._friction_experiment_id is None:
            self._friction_experiment_id = self.project.create_experiment(
                "friction_two_stage",
                config.to_dict(),
            )
        self.project.update_experiment(
            self._friction_experiment_id,
            "running",
            started_at=datetime.now(timezone.utc).isoformat(),
        )
        telemetry_path = self.project.new_telemetry_path(
            f"friction_{self._friction_experiment_id}_{len(self._friction_telemetry_paths) + 1}"
        )
        self.friction_recorder.start(telemetry_path)
        self._friction_telemetry_paths.append(str(telemetry_path))
        self._reported_recorder_error = None
        self._friction_last_sample_sequence = self._telemetry_sequence
        self._friction_recovery_started_at = None
        self._friction_recovery_alerted = False
        self._friction_resume_pending = False
        self.monitor_mask = FRICTION_MONITOR_MASK
        self._prepare_monitor_configuration(
            FRICTION_MONITOR_MASK,
            reset_statistics=True,
            reset_recovery=True,
        )
        self._mark_monitor_configuration_started()
        commands = self._friction_configuration_commands(
            config,
            pid_values,
            mode=self._friction_experiment.configuration_mode,
            working_current_limit_a=self._friction_experiment.working_current_limit_a,
        )
        self.friction_start_button.setEnabled(False)
        self.friction_resume_button.setEnabled(False)
        self.friction_stop_button.setEnabled(True)
        self.friction_status_label.setText("Применяется конфигурация исполнительного preflight…")
        self._log(
            "FRICTION",
            f"{'Продолжение' if resumed else 'Запуск'} опыта #{self._friction_experiment_id}: "
            f"Uq={config.pulse_levels}; скорости={config.targets}",
        )
        self._save_friction_checkpoint()
        self._queue_commands(commands, self._begin_friction_motion)

    def _confirm_friction_start(
        self,
        config: FrictionTestConfig,
        position_targets: tuple[float, ...],
    ) -> tuple[bool, bool]:
        dialog = QDialog(self)
        dialog.setWindowTitle("Запуск автоматического теста")
        dialog.setMinimumWidth(680)
        layout = QVBoxLayout(dialog)
        message = QLabel(self._friction_confirmation_text(config, position_targets))
        message.setWordWrap(True)
        layout.addWidget(message)
        recovery_sound = QCheckBox(
            "Подать короткий звуковой сигнал, если восстановление телеметрии заняло больше 5 с"
        )
        recovery_sound.setChecked(self._friction_recovery_sound_enabled)
        layout.addWidget(recovery_sound)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Yes | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.button(QDialogButtonBox.StandardButton.Yes).setText("Запустить")
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        confirmed = dialog.exec() == QDialog.DialogCode.Accepted
        return confirmed, recovery_sound.isChecked()

    def _friction_confirmation_text(
        self,
        config: FrictionTestConfig,
        position_targets: tuple[float, ...] | None = None,
    ) -> str:
        estimated_pulse_current = config.pulse_max_voltage_v / self.profile.phase_resistance_ohm
        maximum_velocity_alc = estimated_pulse_current * config.breakaway_margin
        speed_travel = [
            speed * (config.settle_s + config.measure_s)
            for speed in config.speed_levels
        ]
        positions = (
            position_targets
            if position_targets is not None
            else tuple(
                index * config.automatic_position_step_rad
                for index in range(config.automatic_position_count)
            )
        )
        position_text = ", ".join(f"{value:.6g}" for value in positions)
        paragraphs: list[str] = []
        if config.evidence_mode:
            paragraphs.append(
                f"Доказательный режим сначала оставит PWM отключённым на "
                f"{config.pwm_off_observation_s:g} с в каждой координате и запишет сырой угол "
                "до фильтра. Затем тот же ноль будет записан при включённом PWM."
            )
        paragraphs.append(
            "FOCTwin временно отключит компенсацию сопротивления фазы, включит torque + Voltage "
            "и подаст короткие прямые импульсы Uq. После подтверждённого движения Uq сразу "
            "станет нулём."
        )
        paragraphs.append(
            "Пределы будут согласованы автоматически:\n"
            f"• измеренный Iq: ±{config.current_trip_limit_a:g} А;\n"
            f"• напряжение: ±{config.voltage_limit_v:g} В;\n"
            f"• скорость по углу: ±{config.velocity_limit_rad_s:g} рад/с;\n"
            f"• координата: [{config.angle_min_rad:g}; {config.angle_max_rad:g}] рад."
        )
        if config.evidence_mode:
            actuator_text = (
                f"Страгивание подтверждается только остаточным перемещением не менее "
                f"{config.residual_movement_threshold_rad:g} рад после снятия Uq и повторяется "
                f"{config.breakaway_repeats} раза. Скоростной PI во всех точках получает один "
                f"предел, эквивалентный {config.fixed_velocity_voltage_limit_v:g} В Uq."
            )
            travel_text = f"ровно {config.velocity_travel_rad:g} рад на каждую команду"
        else:
            actuator_text = (
                "После найденного страгивания ALC скоростного PI рассчитается автоматически; "
                f"его возможный максимум {maximum_velocity_alc:.3g} А."
            )
            travel_text = (
                "около "
                f"{', '.join(f'{value:.3g}' for value in speed_travel)} рад"
            )
        paragraphs.append(
            f"При Uq max={config.pulse_max_voltage_v:g} В расчёт Uq/R даёт до "
            f"{estimated_pulse_current:.3g} А. Это оценка команды, а не измеренный ток, поэтому "
            f"аварийный порог Iq не будет искусственно повышен. {actuator_text} Если Iq не "
            "наблюдается в каком-либо направлении, тест продолжит диагностику, но не разрешит "
            "принять недостоверную модель трения."
        )
        paragraphs.append(
            "Скоростные участки действительно перемещают вал: расчётный путь одного участка "
            f"для трёх модулей скорости составляет {travel_text}. Во время прохода данные "
            f"будут разбиты на интервалы по {config.position_bin_width_rad:g} рад и сохранены "
            "как карта координат."
        )
        if config.evidence_mode and not config.adaptive_positioning_enabled:
            position_limit_text = (
                f"Позиционный ALC фиксирован на эквиваленте "
                f"{config.positioning_voltage_max_v:g} В и не меняется между подходами."
            )
        else:
            position_limit_text = (
                "Если вал остановится при насыщении, Uq будет повышаться по "
                f"{config.positioning_voltage_step_v:g} В, но не выше "
                f"{config.positioning_voltage_max_v:g} В."
            )
        paragraphs.append(
            f"Измерений по координатам: {len(positions)} ({position_text} рад), проходов карты "
            f"{config.map_passes}. Между ними FOCTwin включит angle + Voltage torque с "
            f"загруженными коэффициентами, ограничит скорость до "
            f"{config.velocity_limit_rad_s:g} рад/с и потребует ошибку не больше "
            f"{config.position_tolerance_rad:g} рад. {position_limit_text} Остановка без "
            "насыщения будет отмечена как ошибка регулятора/режима, а не как недостаток "
            "напряжения. В каждой позиции спокойный ноль, preflight и шесть скоростных точек "
            "выполняются заново."
        )
        if config.evidence_mode and config.position_validation_enabled:
            paragraphs.append(
                "После карты выполнятся фиксированные позиционные шаги "
                f"±{config.position_validation_small_rad:g}, "
                f"±{config.position_validation_medium_rad:g} и "
                f"±{config.position_validation_large_rad:g} рад с возвратом в исходную точку; "
                f"полный цикл повторится {config.breakaway_repeats} раз для оценки разброса "
                "целевой метрики."
            )
        paragraphs.append(
            f"Допускается до {config.max_recovery_attempts} восстановлений телеметрии. "
            "Держите питание доступным для ручного отключения. Запустить?"
        )
        return "\n\n".join(paragraphs)

    def _apply_friction_limits(self, config: FrictionTestConfig) -> None:
        self.current_limit.setValue(config.current_trip_limit_a)
        self.voltage_limit.setValue(config.voltage_limit_v)
        self.velocity_limit.setValue(config.velocity_limit_rad_s)
        self.angle_min.setValue(config.angle_min_rad)
        self.angle_max.setValue(config.angle_max_rad)
        self.device_limit_spins["current_a"].setValue(config.current_trip_limit_a)
        self.device_limit_spins["voltage_v"].setValue(config.voltage_limit_v)
        self.device_limit_spins["velocity_rad_s"].setValue(config.velocity_limit_rad_s)
        previous = self.guard.limits
        limits = SafetyLimits(
            current_a=config.current_trip_limit_a,
            voltage_v=config.voltage_limit_v,
            velocity_rad_s=config.velocity_limit_rad_s,
            angle_min_rad=config.angle_min_rad,
            angle_max_rad=config.angle_max_rad,
            trial_timeout_s=previous.trial_timeout_s,
            telemetry_timeout_s=previous.telemetry_timeout_s,
        )
        limits.validate()
        self.profile.safety = limits
        self.guard = SafetyGuard(limits)
        self._sync_device_limits_to_profile()

    def _friction_configuration_commands(
        self,
        config: FrictionTestConfig,
        pid_values: dict[str, dict[str, float]] | None = None,
        *,
        mode: str = "actuator",
        working_current_limit_a: float | None = None,
    ) -> list[str]:
        values = pid_values or {loop: self._pid_values(loop) for loop in self.pid_tables}
        if mode not in {"observer", "actuator", "velocity"}:
            raise ValueError(f"Неизвестный этап опыта: {mode}")
        current_limit = (
            config.current_trip_limit_a
            if mode in {"observer", "actuator"}
            else working_current_limit_a
        )
        if current_limit is None:
            raise ValueError("Для скоростного этапа не найден рабочий предел тока")
        commands = [
            self.protocol.disable(),
            self.protocol.phase_resistance(
                FRICTION_DIRECT_VOLTAGE_SENTINEL
                if mode == "actuator"
                else self.profile.phase_resistance_ohm
            ),
            self.protocol.current_limit(current_limit),
            self.protocol.voltage_limit(config.voltage_limit_v),
            self.protocol.velocity_limit(config.velocity_limit_rad_s),
        ]
        for loop in ("angle", "velocity", "current_q", "current_d"):
            commands.extend(
                self.protocol.pid(loop, field, values[loop][field]) for field in self.PID_FIELDS
            )
        commands.extend(
            (
                self.protocol.torque_mode(TorqueMode.VOLTAGE),
                self.protocol.motion_mode(
                    MotionMode.TORQUE
                    if mode in {"observer", "actuator"}
                    else MotionMode.VELOCITY
                ),
                self.protocol.target(0.0),
                self.protocol.monitor_clear(),
                self.protocol.monitor_downsample(config.monitor_downsample),
                self.protocol.monitor_variables(FRICTION_MONITOR_MASK),
            )
        )
        if mode != "observer":
            commands.append(self.protocol.enable())
        return commands

    def _friction_positioning_commands(
        self,
        config: FrictionTestConfig,
        target_position_rad: float,
        working_current_limit_a: float,
        pid_values: dict[str, dict[str, float]] | None = None,
    ) -> list[str]:
        values = pid_values or {loop: self._pid_values(loop) for loop in self.pid_tables}
        commands = [
            self.protocol.disable(),
            self.protocol.phase_resistance(self.profile.phase_resistance_ohm),
            self.protocol.current_limit(working_current_limit_a),
            self.protocol.voltage_limit(config.voltage_limit_v),
            self.protocol.velocity_limit(config.velocity_limit_rad_s),
        ]
        for loop in ("angle", "velocity", "current_q", "current_d"):
            commands.extend(
                self.protocol.pid(loop, field, values[loop][field])
                for field in self.PID_FIELDS
            )
        commands.extend(
            (
                self.protocol.torque_mode(TorqueMode.VOLTAGE),
                self.protocol.motion_mode(MotionMode.ANGLE),
                self.protocol.target(target_position_rad),
                self.protocol.monitor_clear(),
                self.protocol.monitor_downsample(config.monitor_downsample),
                self.protocol.monitor_variables(FRICTION_MONITOR_MASK),
                self.protocol.enable(),
            )
        )
        return commands

    def _friction_recovery_commands(self, experiment: FrictionExperiment) -> list[str]:
        if experiment.configuration_mode == "position":
            current_limit = experiment.positioning_current_limit_a
            if current_limit is None:
                raise ValueError("Не найден безопасный предел для автоматического смещения")
            return self._friction_positioning_commands(
                experiment.config,
                experiment.board_target_for_continuous(
                    experiment.current_position_target_rad
                ),
                current_limit,
            )
        return self._friction_configuration_commands(
            experiment.config,
            mode=experiment.configuration_mode,
            working_current_limit_a=experiment.working_current_limit_a,
        )

    def _begin_friction_motion(self) -> None:
        experiment = self._friction_experiment
        if experiment is None or experiment.phase != FrictionPhase.IDLE:
            return
        self._pwm_requested = experiment.configuration_mode != "observer"
        self._safety_latched = False
        self.guard.reset()
        self._mark_monitor_configuration_started()
        self._process_friction_actions(experiment.start(time.monotonic()))
        if experiment.configuration_mode == "observer":
            self.friction_status_label.setText(
                "PWM отключён: записывается сырая координата энкодера и ложная скорость…"
            )
        elif experiment.configuration_mode == "position":
            self.friction_status_label.setText(
                "Возврат к сохранённой автоматической координате…"
            )
        elif experiment.configuration_mode == "actuator":
            self.friction_status_label.setText(
                "PWM включён: проверяется спокойный ноль перед первым прямым импульсом Uq…"
            )
        else:
            self.friction_status_label.setText(
                "Исполнительная часть уже подтверждена; выдержка перед скоростными точками…"
            )

    def _advance_friction_experiment(self) -> None:
        experiment = self._friction_experiment
        if experiment is None or experiment.phase in {FrictionPhase.COMPLETE, FrictionPhase.ABORTED}:
            return
        now = time.monotonic()
        if experiment.phase == FrictionPhase.IDLE:
            return
        if experiment.phase in {
            FrictionPhase.CONFIGURING_VELOCITY,
            FrictionPhase.CONFIGURING_POSITION,
            FrictionPhase.CONFIGURING_OBSERVER,
            FrictionPhase.CONFIGURING_ACTUATOR,
        }:
            self._update_friction_status(now)
            return
        if experiment.phase == FrictionPhase.RECOVERING:
            self._advance_friction_recovery(now)
            return
        reference = self._last_telemetry_received_at or self._monitor_configured_at
        stale_timeout = monitor_stale_timeout(experiment.config.monitor_downsample)
        if reference is None or now - reference > stale_timeout:
            self._friction_last_sample_sequence = self._telemetry_sequence
            self._friction_recovery_started_at = now
            self._friction_recovery_alerted = False
            self._process_friction_actions(experiment.enter_recovery())
            self._save_friction_checkpoint()
            return
        self._process_friction_actions(experiment.tick(now))
        self._update_friction_status(now)

    def _advance_friction_recovery(self, now: float) -> None:
        experiment = self._friction_experiment
        if experiment is None or experiment.phase != FrictionPhase.RECOVERING:
            return
        if not self.device.connected or self._friction_resume_pending:
            return
        if self._command_queue or self._command_timer.isActive():
            return
        if self._telemetry_sequence <= self._friction_last_sample_sequence:
            started = self._friction_recovery_started_at or now
            self.friction_status_label.setText(
                f"Телеметрия восстанавливается автоматически: {now - started:.1f} с…"
            )
            return
        sample = self._last_sample
        if sample is None or sample.angle_rad is None:
            return
        if not experiment.config.angle_min_rad <= sample.angle_rad <= experiment.config.angle_max_rad:
            self._abort_friction_experiment("После восстановления координата вне границ опыта")
            return
        self._alert_slow_friction_recovery(now)
        self._friction_resume_pending = True
        self.monitor_mask = FRICTION_MONITOR_MASK
        self._prepare_monitor_configuration(
            FRICTION_MONITOR_MASK,
            reset_statistics=False,
            reset_recovery=False,
        )
        self._mark_monitor_configuration_started()
        try:
            commands = self._friction_recovery_commands(experiment)
        except ValueError as exc:
            self._abort_friction_experiment(str(exc))
            return
        self.friction_status_label.setText("Поток восстановлен; повторяется прерванная точка…")
        self._queue_commands(commands, self._resume_friction_motion)

    def _alert_slow_friction_recovery(self, now: float) -> bool:
        started = self._friction_recovery_started_at
        recovery_duration = 0.0 if started is None else max(0.0, now - started)
        if (
            recovery_duration <= 5.0
            or not self._friction_recovery_sound_enabled
            or self._friction_recovery_alerted
        ):
            return False
        QApplication.beep()
        self._friction_recovery_alerted = True
        self._log(
            "TELEMETRY_RECOVERY",
            f"Звуковой сигнал: поток отсутствовал {recovery_duration:.1f} с",
        )
        return True

    def _resume_friction_motion(self) -> None:
        experiment = self._friction_experiment
        self._friction_resume_pending = False
        self._friction_recovery_started_at = None
        self._friction_recovery_alerted = False
        if experiment is None:
            return
        self._pwm_requested = experiment.configuration_mode != "observer"
        self._safety_latched = False
        self.guard.reset()
        self._mark_monitor_configuration_started()
        self._process_friction_actions(experiment.resume_after_recovery(time.monotonic()))
        self._save_friction_checkpoint()

    def _process_friction_actions(self, actions: list[FrictionAction]) -> None:
        experiment = self._friction_experiment
        for action in actions:
            if action.kind == "target" and action.value is not None:
                self._send(self.protocol.target(action.value))
            elif action.kind == "position_target" and action.value is not None:
                if experiment is not None:
                    self._send(
                        self.protocol.target(
                            experiment.board_target_for_continuous(action.value)
                        )
                    )
            elif action.kind == "position_limit" and action.value is not None:
                self._send(self.protocol.current_limit(action.value))
                if experiment is not None:
                    voltage = action.value * self.profile.phase_resistance_ohm
                    self.friction_status_label.setText(
                        "Вал остановился при насыщении; предел автосмещения повышен до "
                        f"эквивалентных {voltage:.6g} В Uq "
                        f"({action.value:.6g} А в ALC)."
                    )
            elif action.kind == "checkpoint":
                if experiment is not None:
                    self._render_actuator_attempts(experiment.actuator_attempts)
                    self._render_friction_points(experiment.points)
                    self._render_position_observations(experiment.position_observations)
                    self._render_positioning_results(experiment.positioning_results)
                self._save_friction_checkpoint()
            elif action.kind == "configure_velocity":
                if experiment is None:
                    continue
                self.friction_status_label.setText(
                    "Страгивание найдено в обе стороны; PWM отключается и автоматически "
                    "настраивается командный ALC скоростного этапа…"
                )
                commands = self._friction_configuration_commands(
                    experiment.config,
                    mode="velocity",
                    working_current_limit_a=experiment.working_current_limit_a,
                )
                self._queue_commands(commands, self._finish_friction_velocity_configuration)
            elif action.kind == "configure_position":
                if experiment is None:
                    continue
                current_limit = experiment.positioning_current_limit_a
                if current_limit is None:
                    self._abort_friction_experiment(
                        "Не найден безопасный предел для автоматического смещения"
                    )
                    continue
                self.friction_status_label.setText(
                    "PWM отключается; включается позиционный контур для смещения к "
                    f"{experiment.current_position_target_rad:.6g} рад…"
                )
                commands = self._friction_positioning_commands(
                    experiment.config,
                    experiment.board_target_for_continuous(
                        experiment.current_position_target_rad
                    ),
                    current_limit,
                )
                self._queue_commands(commands, self._finish_friction_position_configuration)
            elif action.kind == "configure_actuator":
                if experiment is None:
                    continue
                self.friction_status_label.setText(
                    "Позиция достигнута; PWM отключается и включается прямой Uq для нового "
                    "локального preflight…"
                )
                commands = self._friction_configuration_commands(
                    experiment.config,
                    mode="actuator",
                )
                self._queue_commands(commands, self._finish_friction_actuator_configuration)
            elif action.kind == "configure_observer":
                if experiment is None:
                    continue
                self.friction_status_label.setText(
                    "Позиция достигнута; PWM отключается для независимого наблюдения "
                    "энкодера…"
                )
                commands = self._friction_configuration_commands(
                    experiment.config,
                    mode="observer",
                )
                self._queue_commands(commands, self._finish_friction_observer_configuration)
            elif action.kind == "safe_stop":
                if experiment is not None and experiment.phase == FrictionPhase.ABORTED:
                    self._cancel_queued_commands()
                sent = self.device.emergency_stop()
                self._pwm_requested = False
                self.pwm_state_label.setText("PWM: отключён тестом трения")
                self._log("FRICTION_STOP", f"Отправлено: {sent or 'нет связи'}")
                if experiment is not None and experiment.phase == FrictionPhase.ABORTED:
                    self._finalize_friction_experiment("failed", experiment.abort_reason)
            elif action.kind == "finish":
                self.device.emergency_stop()
                self._pwm_requested = False
                self._finalize_friction_experiment("completed")

    def _finish_friction_velocity_configuration(self) -> None:
        experiment = self._friction_experiment
        if experiment is None:
            return
        self._pwm_requested = True
        self._safety_latched = False
        self.guard.reset()
        self._mark_monitor_configuration_started()
        self._process_friction_actions(
            experiment.velocity_configuration_applied(time.monotonic())
        )
        self.friction_status_label.setText(
            "Исполнительная часть подтверждена; выдержка перед шестью скоростными точками…"
        )

    def _finish_friction_position_configuration(self) -> None:
        experiment = self._friction_experiment
        if experiment is None:
            return
        self._pwm_requested = True
        self._safety_latched = False
        self.guard.reset()
        self._mark_monitor_configuration_started()
        self._process_friction_actions(
            experiment.position_configuration_applied(time.monotonic())
        )

    def _finish_friction_actuator_configuration(self) -> None:
        experiment = self._friction_experiment
        if experiment is None:
            return
        self._pwm_requested = True
        self._safety_latched = False
        self.guard.reset()
        self._mark_monitor_configuration_started()
        self._process_friction_actions(
            experiment.actuator_configuration_applied(time.monotonic())
        )

    def _finish_friction_observer_configuration(self) -> None:
        experiment = self._friction_experiment
        if experiment is None:
            return
        self._pwm_requested = False
        self._safety_latched = False
        self.guard.reset()
        self._mark_monitor_configuration_started()
        self._process_friction_actions(
            experiment.observer_configuration_applied(time.monotonic())
        )

    def _update_friction_status(self, now: float) -> None:
        experiment = self._friction_experiment
        if experiment is None:
            return
        phase_names = {
            FrictionPhase.SENSOR_PWM_OFF: "наблюдение энкодера при PWM off",
            FrictionPhase.ACTUATOR_BASELINE: "проверка спокойного нуля",
            FrictionPhase.ACTUATOR_PULSE: "импульс прямого Uq",
            FrictionPhase.ACTUATOR_PAUSE: "Uq=0, ожидание остановки",
            FrictionPhase.CONFIGURING_VELOCITY: "переключение на скоростной этап",
            FrictionPhase.CONFIGURING_POSITION: "включение позиционного контура",
            FrictionPhase.POSITIONING: "автоматическое смещение",
            FrictionPhase.POSITION_SETTLING: "стабилизация координаты",
            FrictionPhase.CONFIGURING_ACTUATOR: "включение локального preflight",
            FrictionPhase.CONFIGURING_OBSERVER: "отключение PWM для наблюдения энкодера",
            FrictionPhase.ZERO: "выдержка на нуле",
            FrictionPhase.SETTLING: "стабилизация",
            FrictionPhase.MEASURING: "полезное измерение",
            FrictionPhase.PAUSE: "пауза на нулевой скорости",
        }
        if experiment.phase in {
            FrictionPhase.SENSOR_PWM_OFF,
            FrictionPhase.ACTUATOR_BASELINE,
            FrictionPhase.ACTUATOR_PULSE,
            FrictionPhase.ACTUATOR_PAUSE,
            FrictionPhase.CONFIGURING_VELOCITY,
            FrictionPhase.CONFIGURING_OBSERVER,
            FrictionPhase.CONFIGURING_ACTUATOR,
        }:
            pulse = experiment.current_pulse_voltage_v
            pulse_text = f" · команда {pulse:g} В" if pulse is not None else ""
            self.friction_status_label.setText(
                f"Исполнительная часть · {phase_names.get(experiment.phase, experiment.phase.value)}"
                f"{pulse_text} · {max(0.0, now - experiment.phase_started_s):.1f} с · "
                f"импульсов {len(experiment.actuator_attempts)} · "
                f"восстановлений {experiment.recovery_attempts}/"
                f"{experiment.config.max_recovery_attempts}"
            )
            return
        if experiment.phase in {
            FrictionPhase.CONFIGURING_POSITION,
            FrictionPhase.POSITIONING,
            FrictionPhase.POSITION_SETTLING,
        }:
            self.friction_status_label.setText(
                f"Положение {experiment.position_index + 1}/"
                f"{len(experiment.position_targets_rad)} · "
                f"{phase_names.get(experiment.phase, experiment.phase.value)} · цель "
                f"{experiment.current_position_target_rad:.6g} рад · "
                f"Uq max {experiment.positioning_voltage_limit_v or 0.0:.6g} В · "
                f"{max(0.0, now - experiment.phase_started_s):.1f} с · "
                f"восстановлений {experiment.recovery_attempts}/"
                f"{experiment.config.max_recovery_attempts}"
            )
            return
        target = experiment.current_target
        point = min(experiment.point_index + 1, len(experiment.config.targets))
        phase = phase_names.get(experiment.phase, experiment.phase.value)
        target_text = f" · цель {target:g} рад/с" if target is not None else ""
        self.friction_status_label.setText(
            f"Положение {experiment.position_index + 1}/"
            f"{len(experiment.position_targets_rad)} · точка {point}/"
            f"{len(experiment.config.targets)} · {phase}{target_text} · "
            f"{max(0.0, now - experiment.phase_started_s):.1f} с · "
            f"восстановлений {experiment.recovery_attempts}/{experiment.config.max_recovery_attempts}"
        )

    def _stop_friction_test_by_user(self) -> None:
        experiment = self._friction_experiment
        if experiment is None or not self._friction_running():
            return
        experiment.abort("Остановлено пользователем")
        self._cancel_queued_commands()
        self.device.emergency_stop()
        self._pwm_requested = False
        self._finalize_friction_experiment("interrupted", experiment.abort_reason)

    def _abort_friction_experiment(self, reason: str) -> None:
        experiment = self._friction_experiment
        if experiment is None:
            return
        experiment.abort(reason)
        self._cancel_queued_commands()
        self.device.emergency_stop()
        self._pwm_requested = False
        self._finalize_friction_experiment("failed", reason)

    def _finalize_friction_experiment(self, status: str, error: str = "") -> None:
        experiment = self._friction_experiment
        if experiment is None:
            return
        recorder_path = self.friction_recorder.stop()
        if recorder_path is not None and str(recorder_path) not in self._friction_telemetry_paths:
            self._friction_telemetry_paths.append(str(recorder_path))
        estimate = experiment.estimate()
        diagnostic_report = experiment.diagnostic_report()
        result = {
            "schema": 8,
            "status": status,
            "config": experiment.config.to_dict(),
            "automatic_positions": {
                "targets_rad": list(experiment.position_targets_rad),
                "completed_position_index": experiment.position_index,
            },
            "motor_parameters": {
                "torque_constant_nm_per_a": self.profile.torque_constant_nm_per_a,
                "phase_resistance_ohm": self.profile.phase_resistance_ohm,
                "back_emf_v_per_krpm": self.profile.back_emf_v_per_krpm,
            },
            "estimate": estimate.to_dict(),
            "actuator_preflight": {
                "attempts": [attempt.to_dict() for attempt in experiment.actuator_attempts],
                "current_detection_threshold_a": experiment.current_detection_threshold_a,
                "working_current_limit_a": experiment.working_current_limit_a,
                "positioning_current_limit_a": experiment.positioning_current_limit_a,
                "complete": experiment.all_actuator_positions_complete,
                "measured_current_complete": (
                    experiment.all_measured_current_positions_complete
                ),
            },
            "position_map": {
                "bin_width_rad": experiment.config.position_bin_width_rad,
                "observations": [
                    observation.to_dict()
                    for observation in experiment.position_observations
                ],
                "accepted_torque_source": "measured_iq_only",
                "voltage_estimate_status": "diagnostic_only",
            },
            "baseline_diagnostics": [
                diagnostic.to_dict() for diagnostic in experiment.baseline_diagnostics
            ],
            "positioning_results": [
                item.to_dict() for item in experiment.positioning_results
            ],
            "diagnostic_report": diagnostic_report,
            "telemetry_paths": self._friction_telemetry_paths,
            "interruption_count": experiment.interruption_count,
            "rejected_angle_samples": experiment.rejected_angle_samples,
            "error": error or None,
        }
        self._friction_last_estimate = estimate
        self._render_actuator_attempts(experiment.actuator_attempts)
        self._render_friction_points(experiment.points)
        self._render_position_observations(experiment.position_observations)
        self._render_positioning_results(experiment.positioning_results)
        self._render_friction_estimate(estimate)
        self._render_diagnostic_report(diagnostic_report)
        if self.project is not None and self._friction_experiment_id is not None:
            fields: dict[str, object] = {
                "finished_at": datetime.now(timezone.utc).isoformat(),
                "result_json": json.dumps(result, ensure_ascii=False),
            }
            if error:
                fields["error"] = error
            self.project.update_experiment(self._friction_experiment_id, status, **fields)
            export_path = self.project.save_export(
                f"friction_{self._friction_experiment_id}",
                result,
            )
            self._log("FRICTION", f"Результат сохранён: {export_path}")
            diagnostic_path = self.project.save_export(
                f"motor_diagnostic_{self._friction_experiment_id}",
                diagnostic_report,
            )
            self._log(
                "FRICTION",
                f"Короткий диагностический отчёт сохранён: {diagnostic_path}",
            )
            try:
                map_path = self._save_friction_position_history()
                if map_path is not None:
                    self._log("FRICTION", f"Накопительная карта координат сохранена: {map_path}")
            except Exception as exc:  # noqa: BLE001
                self._log("ERROR", f"Не удалось обновить карту координат: {exc}")
        self._save_friction_checkpoint()
        self.friction_start_button.setEnabled(True)
        self.friction_resume_button.setEnabled(True)
        self.friction_stop_button.setEnabled(False)
        self.friction_accept_button.setEnabled(status == "completed" and estimate.valid)
        if status == "completed":
            self.friction_status_label.setText(
                "Тест завершён; PWM отключён, ручная конфигурация восстанавливается. " + estimate.note
            )
        else:
            self.friction_status_label.setText(
                f"Тест остановлен: {error or status}. PWM отключён; checkpoint сохранён."
            )
        self._log("FRICTION", f"Опыт завершён со статусом {status}: {error or estimate.note}")
        restore_commands = list(self._friction_restore_commands)
        restore_mask = self._friction_restore_mask
        self._friction_experiment = None
        self._friction_resume_pending = False
        self.monitor_mask = restore_mask
        if self.device.connected and restore_commands:
            self._prepare_monitor_configuration(
                restore_mask,
                reset_statistics=False,
                reset_recovery=True,
            )
            self._mark_monitor_configuration_started()
            self._queue_commands(restore_commands)

    def _save_friction_position_history(self) -> Path | None:
        if self.project is None:
            return None
        dynamic: list[dict[str, object]] = []
        breakaway: list[dict[str, object]] = []
        for experiment_id, result in self.project.experiment_results("friction_two_stage"):
            position_map = result.get("position_map")
            if isinstance(position_map, dict):
                observations = position_map.get("observations", [])
                if isinstance(observations, list):
                    for observation in observations:
                        if isinstance(observation, dict):
                            dynamic.append({"experiment_id": experiment_id, **observation})
            preflight = result.get("actuator_preflight")
            if isinstance(preflight, dict):
                attempts = preflight.get("attempts", [])
                if isinstance(attempts, list):
                    for attempt in attempts:
                        if not isinstance(attempt, dict) or not attempt.get("movement_detected"):
                            continue
                        start_angle = attempt.get("start_angle_rad")
                        command_voltage = attempt.get("command_voltage_v")
                        if not isinstance(start_angle, (int, float)) or not isinstance(
                            command_voltage, (int, float)
                        ):
                            continue
                        measured_torque = None
                        mean_abs_current = attempt.get("mean_abs_measured_current_a")
                        if attempt.get("current_detected") and isinstance(
                            mean_abs_current, (int, float)
                        ):
                            measured_torque = (
                                abs(float(mean_abs_current))
                                * self.profile.torque_constant_nm_per_a
                            )
                        breakaway.append(
                            {
                                "experiment_id": experiment_id,
                                "position_rad": float(start_angle),
                                "direction": int(attempt.get("direction", 0) or 0),
                                "command_voltage_v": float(command_voltage),
                                "measured_torque_nm": measured_torque,
                                "voltage_equivalent_torque_nm": (
                                    abs(float(command_voltage))
                                    / self.profile.phase_resistance_ohm
                                    * self.profile.torque_constant_nm_per_a
                                ),
                                "current_detected": bool(attempt.get("current_detected")),
                            }
                        )
        if not dynamic and not breakaway:
            return None
        valid_dynamic = [item for item in dynamic if item.get("motion_valid")]
        measured_values = [
            float(item["measured_torque_nm"])
            for item in valid_dynamic
            if isinstance(item.get("measured_torque_nm"), (int, float))
        ]
        voltage_values = [
            float(item["voltage_equivalent_torque_nm"])
            for item in valid_dynamic
            if isinstance(item.get("voltage_equivalent_torque_nm"), (int, float))
        ]
        payload = {
            "schema": 1,
            "profile": self.profile.name,
            "dynamic_observations": dynamic,
            "breakaway_observations": breakaway,
            "envelope": {
                "measured_torque_min_nm": min(measured_values) if measured_values else None,
                "measured_torque_max_nm": max(measured_values) if measured_values else None,
                "voltage_equivalent_torque_min_nm": (
                    min(voltage_values) if voltage_values else None
                ),
                "voltage_equivalent_torque_max_nm": (
                    max(voltage_values) if voltage_values else None
                ),
            },
            "torque_policy": {
                "accepted": "measured_iq_only",
                "voltage_equivalent": "diagnostic_only",
            },
        }
        return self.project.save_export("friction_position_map", payload)

    def _save_friction_checkpoint(self) -> None:
        experiment = self._friction_experiment
        if self.project is None or experiment is None:
            return
        payload = experiment.checkpoint_payload(self._friction_experiment_id)
        payload["telemetry_paths"] = list(self._friction_telemetry_paths)
        self.project.save_checkpoint("friction", payload)

    def _clear_friction_results(self) -> None:
        self.friction_actuator_table.setRowCount(0)
        self.friction_position_table.setRowCount(0)
        self.friction_positioning_table.setRowCount(0)
        self.friction_actuator_note.setText(
            "Идёт проверка исполнительной части: команда Uq, фактический Uq и измеренный Iq "
            "не смешиваются."
        )
        for row in range(self.friction_points_table.rowCount()):
            for column in range(2, self.friction_points_table.columnCount()):
                self.friction_points_table.setItem(row, column, QTableWidgetItem("—"))
        for row in range(self.friction_summary_table.rowCount()):
            self.friction_summary_table.setItem(row, 1, QTableWidgetItem("—"))
        self.friction_result_note.setText(
            "Скоростной этап начнётся только после подтверждения исполнительной части."
        )
        self.friction_position_note.setText(
            "Карта появится после первого полезного скоростного участка. Оценка по Uq "
            "сохраняется только как диагностика, пока измерение Iq не подтверждено."
        )
        self.friction_diagnostic_report.setPlainText(
            "Диагностический вывод будет сформирован даже при безопасной остановке теста."
        )

    def _render_actuator_attempts(self, attempts: list[ActuatorPulseResult]) -> None:
        self.friction_actuator_table.setRowCount(len(attempts))
        for row, attempt in enumerate(attempts):
            values = (
                "+" if attempt.direction > 0 else "−",
                str(attempt.repeat_index + 1),
                (
                    "—"
                    if attempt.start_angle_rad is None
                    else f"{attempt.start_angle_rad:.6g}"
                ),
                f"{attempt.command_voltage_v:.6g}",
                f"{attempt.mean_voltage_q_v:.6g}",
                f"{attempt.mean_measured_current_q_a:.6g}",
                f"{attempt.peak_measured_current_q_a:.6g}",
                f"{attempt.angle_delta_rad:.6g}",
                (
                    "—"
                    if attempt.residual_angle_delta_rad is None
                    else f"{attempt.residual_angle_delta_rad:.6g}"
                ),
                attempt.note,
            )
            for column, value in enumerate(values):
                self.friction_actuator_table.setItem(row, column, QTableWidgetItem(value))
        found = {
            attempt.direction: attempt
            for attempt in attempts
            if (
                attempt.confirmed_breakaway is True
                or (
                    attempt.confirmed_breakaway is None
                    and attempt.movement_detected
                )
            )
        }
        if all(direction in found for direction in (1, -1)):
            self.friction_actuator_note.setText(
                "Страгивание: "
                f"Uq+={abs(found[1].command_voltage_v):g} В, "
                f"Uq−={abs(found[-1].command_voltage_v):g} В. "
                "Iq подтверждён в обе стороны."
                if all(found[direction].current_detected for direction in (1, -1))
                else "Страгивание найдено, но измеренный Iq подтверждён не в обе стороны."
            )

    def _render_friction_points(self, points: list[FrictionPointResult]) -> None:
        if len(points) > self.friction_points_table.rowCount():
            self.friction_points_table.setRowCount(len(points))
        for row, point in enumerate(points):
            values = (
                (
                    "—"
                    if point.measurement_position_rad is None
                    else f"{point.measurement_position_rad:.6g}"
                ),
                f"{point.target_velocity_rad_s:g}",
                "—" if point.mean_angle_rad is None else f"{point.mean_angle_rad:.6g}",
                f"{point.mean_velocity_rad_s:.6g}",
                f"{point.mean_current_q_a:.6g}",
                f"{point.friction_torque_nm:.6g}",
                str(point.sample_count),
                point.note,
            )
            for column, value in enumerate(values):
                self.friction_points_table.setItem(row, column, QTableWidgetItem(value))

    def _render_positioning_results(self, results: list[PositioningResult]) -> None:
        self.friction_positioning_table.setRowCount(len(results))
        for row, result in enumerate(results):
            values = (
                str(result.position_index + 1),
                (
                    "—"
                    if result.start_position_rad is None
                    else f"{result.start_position_rad:.6g}"
                ),
                f"{result.target_position_rad:.6g}",
                (
                    "—"
                    if result.final_position_rad is None
                    else f"{result.final_position_rad:.6g}"
                ),
                (
                    "—"
                    if result.final_error_rad is None
                    else f"{result.final_error_rad:.6g}"
                ),
                f"{result.duration_s:.3g}",
                (
                    f"{result.initial_voltage_limit_v:.6g} / "
                    f"{result.final_voltage_limit_v:.6g}"
                ),
                str(result.voltage_boost_count),
                result.note,
            )
            for column, value in enumerate(values):
                self.friction_positioning_table.setItem(
                    row,
                    column,
                    QTableWidgetItem(value),
                )

    def _render_diagnostic_report(self, report: dict[str, object]) -> None:
        lines = [str(report.get("verdict", "Диагностический вывод отсутствует."))]
        findings = report.get("findings", [])
        if isinstance(findings, list) and findings:
            lines.append("")
            lines.append("Обнаружено:")
            severity_labels = {
                "critical": "БЛОКИРУЕТ",
                "high": "ВАЖНО",
                "medium": "УЧЕСТЬ",
            }
            for index, item in enumerate(findings, 1):
                if not isinstance(item, dict):
                    continue
                severity = severity_labels.get(str(item.get("severity")), "ИНФО")
                lines.append(f"{index}. [{severity}] {item.get('conclusion', '')}")
                evidence = item.get("evidence")
                if isinstance(evidence, dict) and evidence:
                    rendered = ", ".join(
                        f"{key}={value:.6g}" if isinstance(value, float) else f"{key}={value}"
                        for key, value in evidence.items()
                        if value is not None
                    )
                    if rendered:
                        lines.append(f"   Данные: {rendered}")
                next_action = item.get("next_action")
                if next_action:
                    lines.append(f"   Дальше: {next_action}")
        recommendation = report.get("model_recommendation")
        if isinstance(recommendation, dict):
            lines.append("")
            lines.append("Для модели:")
            terms = recommendation.get("friction_terms", [])
            if isinstance(terms, list):
                lines.append("• трение: " + ", ".join(str(value) for value in terms))
            lines.append(
                "• измеренный Iq можно использовать как момент: "
                + (
                    "да"
                    if recommendation.get("use_measured_iq_as_torque")
                    else "нет"
                )
            )
            lines.append(
                "• абсолютную инерцию уже можно оценивать: "
                + (
                    "да"
                    if recommendation.get("absolute_inertia_identification_ready")
                    else "нет"
                )
            )
            lines.append(
                "• нужен диапазон/робастная оптимизация: "
                + (
                    "да"
                    if recommendation.get("robust_range_required")
                    else "пока не подтверждено"
                )
            )
            lines.append(
                "• нужна собственная stateful-подсистема трения: "
                + (
                    "да"
                    if recommendation.get(
                        "custom_stateful_friction_subsystem_recommended"
                    )
                    else "пока нет доказательств"
                )
            )
            direct_tuning = recommendation.get("direct_real_motor_tuning")
            if isinstance(direct_tuning, dict):
                lines.append(
                    "• прямая настройка на реальном моторе уже безопасно готова: "
                    + ("да" if direct_tuning.get("ready") else "нет")
                )
                lines.append(
                    "• повторов одной оценки PID: минимум "
                    f"{direct_tuning.get('minimum_repeats', 3)}, агрегирование — "
                    "медиана + штраф за MAD"
                )
        self.friction_diagnostic_report.setPlainText("\n".join(lines))

    def _render_position_observations(
        self,
        observations: list[PositionFrictionObservation],
    ) -> None:
        self.friction_position_table.setRowCount(len(observations))
        for row, observation in enumerate(observations):
            values = (
                f"{observation.position_center_rad:.6g}",
                f"{observation.target_velocity_rad_s:g}",
                f"{observation.mean_velocity_rad_s:.6g}",
                f"{observation.mean_voltage_q_v:.6g}",
                f"{observation.mean_measured_current_q_a:.6g}",
                (
                    "—"
                    if observation.measured_torque_nm is None
                    else f"{observation.measured_torque_nm:.6g}"
                ),
                f"{observation.voltage_equivalent_torque_nm:.6g}",
                str(observation.sample_count),
                observation.note,
            )
            for column, value in enumerate(values):
                self.friction_position_table.setItem(row, column, QTableWidgetItem(value))
        usable_voltage = [
            observation.voltage_equivalent_torque_nm
            for observation in observations
            if observation.motion_valid
        ]
        usable_measured = [
            observation.measured_torque_nm
            for observation in observations
            if observation.motion_valid and observation.measured_torque_nm is not None
        ]
        if not observations:
            return
        coordinate_min = min(observation.position_min_rad for observation in observations)
        coordinate_max = max(observation.position_max_rad for observation in observations)
        voltage_text = (
            "нет пригодного движения"
            if not usable_voltage
            else f"{min(usable_voltage):.6g}…{max(usable_voltage):.6g} Н·м"
        )
        measured_text = (
            "Iq пока не подтверждён"
            if not usable_measured
            else f"{min(usable_measured):.6g}…{max(usable_measured):.6g} Н·м"
        )
        self.friction_position_note.setText(
            f"Покрыта координата {coordinate_min:.6g}…{coordinate_max:.6g} рад. "
            f"Момент по измеренному Iq: {measured_text}. Диагностический диапазон по Uq: "
            f"{voltage_text}; он не принимается в модель как измеренный момент."
        )

    def _render_friction_estimate(self, estimate: FrictionEstimate) -> None:
        values = (
            estimate.coulomb_friction_nm,
            estimate.coulomb_positive_nm,
            estimate.coulomb_negative_nm,
            estimate.viscous_friction_nm_s_rad,
            estimate.breakaway_friction_nm,
            estimate.asymmetry_percent,
            estimate.r_squared,
        )
        for row, value in enumerate(values):
            rendered = "—" if value is None else f"{value:.8g}"
            self.friction_summary_table.setItem(row, 1, QTableWidgetItem(rendered))
        self.friction_result_note.setText(estimate.note)

    def _accept_friction_estimate(self) -> None:
        estimate = self._friction_last_estimate
        if estimate is None or not estimate.valid:
            QMessageBox.warning(self, "Профиль", "Нет пригодной оценки трения")
            return
        if self.project is None:
            QMessageBox.warning(self, "Профиль", "Сначала откройте проект")
            return
        assert estimate.coulomb_friction_nm is not None
        assert estimate.viscous_friction_nm_s_rad is not None
        assert estimate.breakaway_friction_nm is not None
        self.profile.coulomb_friction_nm = estimate.coulomb_friction_nm
        self.profile.viscous_friction_nm_s_rad = estimate.viscous_friction_nm_s_rad
        self.profile.breakaway_friction_nm = estimate.breakaway_friction_nm
        parameters = {
            "coulomb_friction_nm": estimate.coulomb_friction_nm,
            "viscous_friction_nm_s_rad": estimate.viscous_friction_nm_s_rad,
            "breakaway_friction_nm": estimate.breakaway_friction_nm,
            "coulomb_positive_nm": estimate.coulomb_positive_nm,
            "coulomb_negative_nm": estimate.coulomb_negative_nm,
        }
        self.project.save_profile(self.profile)
        self.project.accept_parameters(
            self.profile.name,
            "friction_two_stage",
            parameters,
            score=estimate.r_squared,
            note="Принято пользователем как грубая начальная оценка",
        )
        self.friction_accept_button.setEnabled(False)
        self.friction_result_note.setText(
            estimate.note + " Значения приняты и сохранены новой версией профиля."
        )
        self._log("FRICTION_ACCEPT", json.dumps(parameters, ensure_ascii=False))

    def _active_monitor_downsample(self) -> int:
        experiment = self._friction_experiment
        if experiment is not None and self._friction_running():
            return experiment.config.monitor_downsample
        return self.monitor_downsample_spin.value()

    def _manual_control_blocked_by_friction(self, operation: str) -> bool:
        if not self._friction_running():
            return False
        QMessageBox.warning(
            self,
            "Выполняется тест трения",
            f"{operation.capitalize()} заблокировано до завершения опыта. "
            "Для немедленной остановки используйте «СТОП И ОТКЛЮЧИТЬ PWM».",
        )
        return True

    def _virtual_tuning_page(self) -> QWidget:
        page, layout = titled_page(
            "Виртуальный тюнинг",
            "Последовательная настройка токовых, скоростного и позиционного контуров с необязательной совместной доводкой.",
        )
        top = QHBoxLayout()
        loops = QGroupBox("Контуры")
        loops_layout = QVBoxLayout(loops)
        for label, checked in (
            ("Ток Q", True),
            ("Ток D", True),
            ("Скорость", True),
            ("Положение", True),
            ("Совместная финальная оптимизация", False),
        ):
            checkbox = QCheckBox(label)
            checkbox.setChecked(checked)
            loops_layout.addWidget(checkbox)
        strategy = QComboBox()
        strategy.addItems(("По порядку: ток → скорость → положение", "Выбранный контур", "Все одновременно"))
        loops_layout.addWidget(strategy)
        objectives = QGroupBox("Цели и веса")
        objectives_layout = QFormLayout(objectives)
        for name, weight in (
            ("Ошибка установившегося режима", 5.0),
            ("IAE", 1.0),
            ("ITAE", 1.0),
            ("Перерегулирование", 2.0),
            ("Время установления", 1.0),
            ("Пиковый ток", 3.0),
            ("Насыщение", 3.0),
        ):
            objectives_layout.addRow(name, spin(weight, 0.0, 1000.0, 3))
        resources = QGroupBox("Оптимизатор и ресурсы")
        resources_form = QFormLayout(resources)
        evaluations = QSpinBox()
        evaluations.setRange(1, 100000)
        evaluations.setValue(400)
        workers = QSpinBox()
        workers.setRange(1, 64)
        workers.setValue(1)
        top_count = QSpinBox()
        top_count.setRange(1, 100)
        top_count.setValue(5)
        resources_form.addRow("Максимум итераций", evaluations)
        resources_form.addRow("Параллельные workers", workers)
        resources_form.addRow("Сохранять лучших", top_count)
        resources_form.addRow("Лимит времени, мин", spin(60, 1, 10000, 0))
        top.addWidget(loops)
        top.addWidget(objectives)
        top.addWidget(resources)
        layout.addLayout(top)
        self.virtual_results = table(("Место", "Контур", "P", "I", "D", "LPF", "Kc", "Score", "Проверка"), 0)
        layout.addWidget(self.virtual_results, 1)
        buttons = QHBoxLayout()
        buttons.addWidget(QPushButton("Запустить новый тюнинг"))
        buttons.addWidget(QPushButton("Продолжить"))
        buttons.addWidget(QPushButton("Пауза после текущего опыта"))
        buttons.addWidget(QPushButton("Принять выбранный набор"))
        layout.addLayout(buttons)
        return page

    def _real_tuning_page(self) -> QWidget:
        page, layout = titled_page(
            "Доводка на реальном моторе",
            "Ограниченные изменения вокруг принятого виртуального результата с фиксацией каждой попытки.",
        )
        warning = QLabel(
            "Каждый опыт начинается только из подтверждённого состояния. После обрыва связи текущая "
            "попытка помечается незавершённой и выполняется заново."
        )
        warning.setObjectName("danger")
        warning.setWordWrap(True)
        layout.addWidget(warning)
        settings = QGroupBox("Политика доводки")
        settings_form = QFormLayout(settings)
        approval = QComboBox()
        approval.addItems(("Подтверждать каждый опыт", "Полностью автоматически", "Подтверждать только ухудшения"))
        settings_form.addRow("Запуск опытов", approval)
        settings_form.addRow("Максимальный шаг, %", spin(10.0, 0.01, 100.0))
        settings_form.addRow("Максимум опытов", spin(200, 1, 100000, 0))
        settings_form.addRow("Цель по координате, угл. сек", spin(30, 0.001, 3600, 3))
        settings_form.addRow("Цель по скорости, угл. сек/с", spin(30, 0.001, 3600, 3))
        rollback = QCheckBox("Автоматически откатывать ухудшение к последнему принятому набору")
        rollback.setChecked(True)
        settings_form.addRow(rollback)
        layout.addWidget(settings)
        self.real_results = table(("№", "Время", "Изменённые параметры", "Результат", "Score", "Решение"), 0)
        layout.addWidget(self.real_results, 1)
        controls = QHBoxLayout()
        controls.addWidget(QPushButton("Начать доводку"))
        controls.addWidget(QPushButton("Продолжить прерванную сессию"))
        controls.addWidget(QPushButton("Принять результат"))
        stop = QPushButton("СТОП И ОТКЛЮЧИТЬ PWM")
        stop.setObjectName("dangerButton")
        stop.clicked.connect(self._emergency_stop)
        controls.addWidget(stop)
        layout.addLayout(controls)
        return page

    def _analysis_page(self) -> QWidget:
        page, layout = titled_page(
            "Анализ данных",
            "Произвольный выбор сигналов, реальных/виртуальных данных, интервала и вычисляемых метрик.",
        )
        controls = QHBoxLayout()
        for label, checked in (
            ("Цель", True),
            ("Координата", True),
            ("Скорость", True),
            ("Iq", False),
            ("Id", False),
            ("Uq", False),
            ("Ud", False),
            ("Виртуальные данные", False),
        ):
            checkbox = QCheckBox(label)
            checkbox.setChecked(checked)
            controls.addWidget(checkbox)
        controls.addStretch(1)
        controls.addWidget(QPushButton("Экспорт JSON"))
        controls.addWidget(QPushButton("Экспорт Excel"))
        layout.addLayout(controls)
        if pg is not None:
            plot = pg.PlotWidget()
            plot.showGrid(x=True, y=True, alpha=0.25)
            layout.addWidget(plot, 1)
        else:
            layout.addWidget(QLabel("График станет доступен после установки pyqtgraph"), 1)
        metrics = table(("Метрика", "Реальный мотор", "Модель", "Отклонение"), 6)
        for row, name in enumerate(("IAE", "ITAE", "Ошибка установления", "Перерегулирование", "Время установления", "Пиковый ток")):
            metrics.setItem(row, 0, QTableWidgetItem(name))
        layout.addWidget(metrics)
        return page

    def _console_page(self) -> QWidget:
        page, layout = titled_page(
            "Консоль и сценарии",
            "Сырые команды платы и проверяемый FOCTwin DSL. Сценарий компилируется в Commander-команды до запуска.",
        )
        tabs = QTabWidget()
        raw_tab = QWidget()
        raw_layout = QVBoxLayout(raw_tab)
        raw_row = QHBoxLayout()
        self.raw_command = QLineEdit("AMG6")
        raw_send = QPushButton("Отправить как есть")
        raw_send.clicked.connect(lambda: self._send(self.raw_command.text()))
        raw_row.addWidget(self.raw_command, 1)
        raw_row.addWidget(raw_send)
        raw_layout.addLayout(raw_row)
        self.raw_output = QPlainTextEdit()
        self.raw_output.setReadOnly(True)
        self.raw_output.document().setMaximumBlockCount(5000)
        raw_layout.addWidget(self.raw_output, 1)
        scenario_tab = QWidget()
        scenario_layout = QVBoxLayout(scenario_tab)
        self.scenario_editor = QPlainTextEdit(
            "# Пример безопасного сценария\n"
            "LIMIT CURRENT 1\n"
            "LIMIT VOLTAGE 12\n"
            "LIMIT VELOCITY 0.7\n"
            "MODE ANGLE\n"
            "TORQUE VOLTAGE\n"
            "EN\n"
            "TARGET 0.2\n"
            "WAIT 2\n"
            "TARGET 0\n"
            "STOP\n"
        )
        scenario_layout.addWidget(self.scenario_editor, 1)
        scenario_buttons = QHBoxLayout()
        validate = QPushButton("Проверить и показать трансляцию")
        validate.clicked.connect(self._compile_scenario)
        run = QPushButton("Запустить сценарий")
        run.setEnabled(False)
        scenario_buttons.addWidget(validate)
        scenario_buttons.addWidget(run)
        scenario_layout.addLayout(scenario_buttons)
        self.scenario_output = QPlainTextEdit()
        self.scenario_output.setReadOnly(True)
        scenario_layout.addWidget(self.scenario_output)
        tabs.addTab(raw_tab, "Сырые команды")
        tabs.addTab(scenario_tab, "FOCTwin DSL")
        layout.addWidget(tabs, 1)
        return page

    def _profiles_page(self) -> QWidget:
        page, layout = titled_page(
            "Профили установки",
            "Сохраняемые версии виртуального мотора, ограничений и принятых регуляторов.",
        )
        splitter = QSplitter(Qt.Orientation.Horizontal)
        profiles = QListWidget()
        profiles.addItem(self.profile.name)
        profiles.addItem("Создать новый профиль…")
        editor = QPlainTextEdit(json.dumps(self.profile.to_dict(), ensure_ascii=False, indent=2))
        splitter.addWidget(profiles)
        splitter.addWidget(editor)
        splitter.setSizes((300, 1000))
        layout.addWidget(splitter, 1)
        buttons = QHBoxLayout()
        buttons.addWidget(QPushButton("Сохранить новую версию"))
        buttons.addWidget(QPushButton("Сделать активным"))
        buttons.addWidget(QPushButton("Импорт JSON"))
        buttons.addWidget(QPushButton("Экспорт JSON"))
        layout.addLayout(buttons)
        return page

    def _log_page(self) -> QWidget:
        page, layout = titled_page(
            "Подробный журнал",
            "Все команды, ответы, состояния, checkpoints и ошибки. Файлы проекта можно передать для диагностики.",
        )
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(10000)
        layout.addWidget(self.log_view, 1)
        buttons = QHBoxLayout()
        buttons.addWidget(QPushButton("Открыть папку логов"))
        buttons.addWidget(QPushButton("Сохранить диагностический пакет"))
        buttons.addWidget(QPushButton("Очистить только экран"))
        layout.addLayout(buttons)
        return page

    def _new_project(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "Выберите родительскую папку проекта")
        if not directory:
            return
        name, accepted = self._ask_project_name()
        if not accepted:
            return
        self._activate_project(Path(directory) / f"{name}.foctwin", initialize=True)

    def _open_project(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "Откройте папку *.foctwin")
        if directory:
            self._activate_project(Path(directory), initialize=False)

    def _ask_project_name(self) -> tuple[str, bool]:
        dialog = QDialog(self)
        dialog.setWindowTitle("Новый проект FOCTwin")
        layout = QVBoxLayout(dialog)
        edit = QLineEdit("azimuth-baseline")
        layout.addWidget(QLabel("Имя проекта"))
        layout.addWidget(edit)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        accepted = dialog.exec() == QDialog.DialogCode.Accepted
        name = "".join(char if char.isalnum() or char in "-_" else "-" for char in edit.text().strip())
        return name or "foctwin-project", accepted

    def _activate_project(self, root: Path, initialize: bool) -> None:
        self.project = ProjectStore(root)
        if initialize or not self.project.db_path.exists():
            self.project.initialize(self.profile)
        self.side_project.setText(str(root))
        self.dashboard_project.setText(str(root))
        self.statusBar().showMessage(f"Проект открыт: {root}", 5000)
        self._log("INFO", f"Проект открыт: {root}")

    def _connect_settings_persistence(self) -> None:
        spin_boxes = [
            self.target_spin,
            *self.device_limit_spins.values(),
            self.current_limit,
            self.voltage_limit,
            self.velocity_limit,
            self.angle_min,
            self.angle_max,
            self.monitor_downsample_spin,
            self.plot_window_spin,
            self.friction_low_speed,
            self.friction_high_speed,
            self.friction_current_trip,
            self.friction_voltage_limit,
            self.friction_velocity_limit,
            self.friction_angle_min,
            self.friction_angle_max,
            self.friction_pulse_start,
            self.friction_pulse_step,
            self.friction_pulse_max,
            self.friction_pulse_duration,
            self.friction_actuator_pause,
            self.friction_baseline,
            self.friction_movement_threshold,
            self.friction_current_floor,
            self.friction_breakaway_margin,
            self.friction_settle,
            self.friction_measure,
            self.friction_pause,
            self.friction_downsample,
            self.friction_recoveries,
            self.friction_position_bin,
            self.friction_automatic_positions,
            self.friction_automatic_position_step,
        ]
        for widget in spin_boxes:
            widget.valueChanged.connect(self._schedule_user_settings_save)
        for widget in (
            self.motion_combo,
            self.torque_combo,
            self.baud_combo,
        ):
            widget.currentIndexChanged.connect(self._schedule_user_settings_save)
        self.port_combo.currentTextChanged.connect(self._schedule_user_settings_save)
        self.device_id_edit.textChanged.connect(self._schedule_user_settings_save)
        checkboxes = [
            self.safe_connect_checkbox,
            self.auto_reconnect_checkbox,
            self.software_guard_enabled,
            self.raw_telemetry_checkbox,
            self.plot_follow_checkbox,
            self.friction_evidence_mode,
            self.friction_adaptive_positioning,
            self.friction_position_validation,
            *self.device_limit_checks.values(),
            *self.monitor_checks.values(),
            *self.plot_checks.values(),
        ]
        for checkbox in checkboxes:
            checkbox.toggled.connect(self._schedule_user_settings_save)
        for pid_table in self.pid_tables.values():
            pid_table.itemChanged.connect(self._schedule_user_settings_save)

    def _schedule_user_settings_save(self, *_args: object) -> None:
        if not self._settings_loading:
            self._settings_save_timer.start()

    def _settings_payload(self) -> dict[str, object]:
        return {
            "schema": 1,
            "connection": {
                "port": self.port_combo.currentText().strip(),
                "baud": self.baud_combo.currentText(),
                "device_id": self.device_id_edit.text().strip() or "A",
                "safe_connect": self.safe_connect_checkbox.isChecked(),
                "auto_reconnect": self.auto_reconnect_checkbox.isChecked(),
            },
            "control": {
                "motion": self._combo_data_value(self.motion_combo),
                "torque": self._combo_data_value(self.torque_combo),
                "target": self.target_spin.value(),
            },
            "device_limits": {
                key: {
                    "value": widget.value(),
                    "selected": self.device_limit_checks[key].isChecked(),
                }
                for key, widget in self.device_limit_spins.items()
            },
            "software_limits": {
                "current_a": self.current_limit.value(),
                "voltage_v": self.voltage_limit.value(),
                "velocity_rad_s": self.velocity_limit.value(),
                "angle_min_rad": self.angle_min.value(),
                "angle_max_rad": self.angle_max.value(),
                "enabled": self.software_guard_enabled.isChecked(),
            },
            "pid": {
                loop: {
                    field: table_widget.item(row, 1).text()
                    for row, field in enumerate(self.PID_FIELDS)
                }
                for loop, table_widget in self.pid_tables.items()
            },
            "monitor": {
                "mask": "".join(
                    "1" if self.monitor_checks[name].isChecked() else "0"
                    for name in MONITOR_FIELDS
                ),
                "downsample": self.monitor_downsample_spin.value(),
                "raw_console": self.raw_telemetry_checkbox.isChecked(),
            },
            "plot": {
                "signals": [name for name in MONITOR_FIELDS if self.plot_checks[name].isChecked()],
                "window_s": self.plot_window_spin.value(),
                "follow": self.plot_follow_checkbox.isChecked(),
            },
            "friction": self._friction_config_from_widgets().to_dict(),
        }

    def _save_user_settings(self) -> None:
        if self._settings_loading:
            return
        self.settings.setValue(
            self.SETTINGS_KEY,
            json.dumps(self._settings_payload(), ensure_ascii=False, separators=(",", ":")),
        )
        self.settings.sync()

    def _restore_user_settings(self) -> None:
        raw = self.settings.value(self.SETTINGS_KEY, "")
        if not raw:
            return
        try:
            payload = json.loads(str(raw))
            if not isinstance(payload, dict) or payload.get("schema") != 1:
                raise ValueError("unsupported settings schema")
            self._settings_loading = True
            connection = payload.get("connection", {})
            self.port_combo.setCurrentText(str(connection.get("port", "COM3")))
            baud = str(connection.get("baud", "115200"))
            baud_index = self.baud_combo.findText(baud)
            if baud_index >= 0:
                self.baud_combo.setCurrentIndex(baud_index)
            self.device_id_edit.setText(str(connection.get("device_id", "A"))[:1] or "A")
            self.safe_connect_checkbox.setChecked(bool(connection.get("safe_connect", True)))
            self.auto_reconnect_checkbox.setChecked(bool(connection.get("auto_reconnect", True)))

            control = payload.get("control", {})
            self._set_combo_enum(self.motion_combo, MotionMode, control.get("motion"))
            self._set_combo_enum(self.torque_combo, TorqueMode, control.get("torque"))
            self.target_spin.setValue(float(control.get("target", 0.0)))

            for key, saved in payload.get("device_limits", {}).items():
                if key not in self.device_limit_spins or not isinstance(saved, dict):
                    continue
                self.device_limit_spins[key].setValue(float(saved.get("value", 0.0)))
                self.device_limit_checks[key].setChecked(bool(saved.get("selected", False)))

            software = payload.get("software_limits", {})
            for key, widget in (
                ("current_a", self.current_limit),
                ("voltage_v", self.voltage_limit),
                ("velocity_rad_s", self.velocity_limit),
                ("angle_min_rad", self.angle_min),
                ("angle_max_rad", self.angle_max),
            ):
                if key in software:
                    widget.setValue(float(software[key]))
            self.software_guard_enabled.setChecked(bool(software.get("enabled", True)))

            for loop, saved_fields in payload.get("pid", {}).items():
                if loop not in self.pid_tables or not isinstance(saved_fields, dict):
                    continue
                for row, field in enumerate(self.PID_FIELDS):
                    if field in saved_fields:
                        self.pid_tables[loop].item(row, 1).setText(str(saved_fields[field]))

            monitor = payload.get("monitor", {})
            mask = str(monitor.get("mask", "1111111"))
            if len(mask) == len(MONITOR_FIELDS) and set(mask) <= {"0", "1"} and "1" in mask:
                for name, enabled in zip(MONITOR_FIELDS, mask, strict=True):
                    self.monitor_checks[name].setChecked(enabled == "1")
                self.monitor_mask = mask
            self.monitor_downsample_spin.setValue(int(monitor.get("downsample", 20)))
            self.raw_telemetry_checkbox.setChecked(bool(monitor.get("raw_console", False)))

            plot = payload.get("plot", {})
            plot_signals = set(plot.get("signals", ()))
            for name in MONITOR_FIELDS:
                self.plot_checks[name].setChecked(name in plot_signals)
            self.plot_window_spin.setValue(int(plot.get("window_s", 30)))
            self.plot_follow_checkbox.setChecked(bool(plot.get("follow", True)))

            friction = payload.get("friction")
            if isinstance(friction, dict):
                try:
                    config = FrictionTestConfig.from_dict(friction)
                except (TypeError, ValueError):
                    self._log("ERROR", "Настройки теста трения не восстановлены: значения некорректны")
                else:
                    self._set_friction_config_widgets(config)
            self._sync_profile_from_widgets()
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._log("ERROR", f"Последние настройки не восстановлены: {exc}")
        finally:
            self._settings_loading = False

    @staticmethod
    def _combo_data_value(combo: QComboBox) -> str:
        data = combo.currentData()
        return str(getattr(data, "value", data))

    @staticmethod
    def _set_combo_enum(combo: QComboBox, enum_type: type[MotionMode | TorqueMode], value: object) -> None:
        try:
            expected = enum_type(str(value))
        except ValueError:
            return
        index = combo.findData(expected)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _sync_profile_from_widgets(self) -> None:
        limits = SafetyLimits(
            current_a=self.current_limit.value(),
            voltage_v=self.voltage_limit.value(),
            velocity_rad_s=self.velocity_limit.value(),
            angle_min_rad=self.angle_min.value(),
            angle_max_rad=self.angle_max.value(),
            trial_timeout_s=self.profile.safety.trial_timeout_s,
            telemetry_timeout_s=self.profile.safety.telemetry_timeout_s,
        )
        limits.validate()
        self.profile.safety = limits
        self.guard = SafetyGuard(limits)
        for loop in self.pid_tables:
            values = self._pid_values(loop)
            params = getattr(self.profile, loop)
            params.p = values["p"]
            params.i = values["i"]
            params.d = values["d"]
            params.output_ramp = values["ramp"]
            params.lpf_tf = values["lpf"]
        self._sync_device_limits_to_profile()

    def _sync_device_limits_to_profile(self) -> None:
        self.profile.angle.output_limit = self.device_limit_spins["velocity_rad_s"].value()
        self.profile.velocity.output_limit = self.device_limit_spins["current_a"].value()
        voltage_limit = self.device_limit_spins["voltage_v"].value()
        self.profile.current_q.output_limit = voltage_limit
        self.profile.current_d.output_limit = voltage_limit

    def _queue_commands(
        self,
        commands: list[str] | tuple[str, ...],
        on_complete: Callable[[], None] | None = None,
    ) -> None:
        self._command_queue.extend(commands)
        if on_complete is not None:
            self._command_queue.append(on_complete)
        if not self._command_timer.isActive():
            self._dispatch_next_command()

    def _dispatch_next_command(self) -> None:
        if not self._command_queue:
            return
        item = self._command_queue.popleft()
        if callable(item):
            item()
            if self._command_queue and not self._command_timer.isActive():
                self._command_timer.start(0)
            return
        self._send(item)
        if self._command_queue:
            self._command_timer.start()

    def _cancel_queued_commands(self) -> None:
        self._command_timer.stop()
        self._command_queue.clear()

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText().strip() or "COM3"
        ports = self.device.available_ports()
        self.port_combo.clear()
        for device, description in ports:
            self.port_combo.addItem(device, description)
        if self.port_combo.findText(current) < 0:
            self.port_combo.addItem(current)
        self.port_combo.setCurrentText(current)
        if ports:
            details = "; ".join(f"{device}: {description}" for device, description in ports)
            self.connection_details.setText(details)

    def _toggle_connection(self) -> None:
        if self.device.connected or self._connection_requested:
            self._connection_requested = False
            self._connecting = False
            self._cancel_queued_commands()
            self.device.disconnect()
            self.connect_button.setText("Подключить")
            return
        self._connection_requested = True
        self._attempt_connect(show_error=True)

    def _attempt_connect(self, show_error: bool) -> None:
        if self.device.connected or self._connecting:
            return
        self._connecting = True
        self.protocol = CommanderProtocol(self.device_id_edit.text().strip() or "A")
        self.device.protocol = self.protocol
        try:
            self.device.connect(self.port_combo.currentText().strip(), int(self.baud_combo.currentText()))
        except Exception as exc:  # noqa: BLE001
            self._connecting = False
            self._log("ERROR", f"Не удалось подключиться: {exc}")
            self.connection_details.setText(f"Ошибка подключения: {exc}")
            if show_error:
                QMessageBox.critical(self, "Serial", str(exc))

    def _reconnect_if_needed(self) -> None:
        if (
            self._connection_requested
            and self.auto_reconnect_checkbox.isChecked()
            and not self.device.connected
            and not self._connecting
        ):
            self._attempt_connect(show_error=False)

    def _initialize_connection(self) -> None:
        if not self.device.connected:
            return
        if self.safe_connect_checkbox.isChecked():
            self._send(self.protocol.disable())
            self._send(self.protocol.phase_resistance(self.profile.phase_resistance_ohm))
        experiment = self._friction_experiment
        if experiment is not None and experiment.phase == FrictionPhase.RECOVERING:
            self.monitor_mask = FRICTION_MONITOR_MASK
            self._prepare_monitor_configuration(
                FRICTION_MONITOR_MASK,
                reset_statistics=False,
                reset_recovery=False,
            )
            self._send_monitor_configuration("восстанавливается для теста трения")
            return
        self._apply_monitoring()
        self._read_device_configuration()

    def _apply_modes(self) -> None:
        if self._manual_control_blocked_by_friction("применение ручной конфигурации"):
            return
        if not self.device.connected:
            QMessageBox.warning(self, "Конфигурация", "Сначала подключите мотор")
            return
        if not self._allow_parameter_change("изменение режимов"):
            return
        if not self._apply_software_limits(show_status=False):
            return
        try:
            pid_values = {loop: self._pid_values(loop) for loop in self.pid_tables}
        except ValueError as exc:
            QMessageBox.warning(self, "PID/LPF", str(exc))
            return
        target_error = self._target_validation_error()
        if target_error:
            QMessageBox.warning(self, "Цель отклонена", target_error)
            return
        mask = self._monitor_mask_from_ui()
        if mask is None:
            return
        for loop, values in pid_values.items():
            params = getattr(self.profile, loop)
            params.p = values["p"]
            params.i = values["i"]
            params.d = values["d"]
            params.output_ramp = values["ramp"]
            params.lpf_tf = values["lpf"]
        self._sync_device_limits_to_profile()
        self._save_user_settings()
        if self.project:
            self.project.save_profile(self.profile)
        self._prepare_monitor_configuration(mask, reset_statistics=True, reset_recovery=True)
        self._mark_monitor_configuration_started()
        commands = self._full_configuration_commands(pid_values, mask)
        self._configuration_apply_in_progress = True
        self.apply_modes_button.setEnabled(False)
        self.apply_modes_button.setText("Применяется вся конфигурация…")
        self.statusBar().showMessage(
            f"Отправляется полная конфигурация: {len(commands)} команд с безопасными паузами"
        )
        self._queue_commands(commands, self._finish_configuration_apply)

    def _full_configuration_commands(
        self,
        pid_values: dict[str, dict[str, float]],
        monitor_mask: str,
    ) -> list[str]:
        commands = [
            self.protocol.phase_resistance(self.profile.phase_resistance_ohm),
            self.protocol.current_limit(self.device_limit_spins["current_a"].value()),
            self.protocol.voltage_limit(self.device_limit_spins["voltage_v"].value()),
            self.protocol.velocity_limit(self.device_limit_spins["velocity_rad_s"].value()),
        ]
        for loop in ("angle", "velocity", "current_q", "current_d"):
            commands.extend(
                self.protocol.pid(loop, field, pid_values[loop][field])
                for field in self.PID_FIELDS
            )
        commands.extend(
            (
                self.protocol.torque_mode(self.torque_combo.currentData()),
                self.protocol.motion_mode(self.motion_combo.currentData()),
                self.protocol.target(self.target_spin.value()),
                self.protocol.monitor_clear(),
                self.protocol.monitor_downsample(self.monitor_downsample_spin.value()),
                self.protocol.monitor_variables(monitor_mask),
            )
        )
        return commands

    def _finish_configuration_apply(self) -> None:
        self._configuration_apply_in_progress = False
        self.apply_modes_button.setEnabled(True)
        self.apply_modes_button.setText("Применить режимы и всю конфигурацию")
        self.statusBar().showMessage(
            "Вся конфигурация отправлена. Теперь можно включить PWM.", 8000
        )
        self._log(
            "CONFIG",
            "Отправлены лимиты SimpleFOC, программные пороги, PID/LPF, режимы, цель и мониторинг",
        )

    def _read_device_modes(self) -> None:
        self._queue_commands(
            [
                self.protocol.motion_mode(),
                self.protocol.torque_mode(),
                self.protocol.enable(None),
            ]
        )

    def _read_device_configuration(self) -> None:
        # Do not overwrite the desired startup modes/PID values with firmware defaults.
        self._queue_commands([self.protocol.enable(None)])
        self._read_device_limits(copy_to_inputs=False)

    def _send_target(self) -> None:
        if self._manual_control_blocked_by_friction("ручная отправка цели"):
            return
        error = self._target_validation_error()
        if error:
            QMessageBox.warning(self, "Цель отклонена", error)
            return
        self._save_user_settings()
        self._send(self.protocol.target(self.target_spin.value()))

    def _target_validation_error(self) -> str | None:
        target = self.target_spin.value()
        motion = self.motion_combo.currentData()
        limits = self.guard.limits
        if motion in {MotionMode.ANGLE, MotionMode.ANGLE_OPEN_LOOP}:
            if not limits.angle_min_rad <= target <= limits.angle_max_rad:
                return "Цель положения выходит за программный диапазон"
        elif motion in {MotionMode.VELOCITY, MotionMode.VELOCITY_OPEN_LOOP}:
            if abs(target) > limits.velocity_rad_s:
                return "Цель скорости превышает программный порог"
        elif motion == MotionMode.TORQUE:
            torque_mode = self.torque_combo.currentData()
            limit = limits.voltage_v if torque_mode == TorqueMode.VOLTAGE else limits.current_a
            if abs(target) > limit:
                return "Цель момента превышает программный порог выбранного режима"
        return None

    def _enable_pwm(self) -> None:
        if self._manual_control_blocked_by_friction("ручное включение PWM"):
            return
        if not self.device.connected:
            QMessageBox.warning(self, "PWM", "Сначала подключите мотор")
            return
        if self._configuration_apply_in_progress or self._command_queue or self._command_timer.isActive():
            QMessageBox.information(
                self,
                "PWM",
                "Дождитесь сообщения «Вся конфигурация отправлена», затем включите PWM.",
            )
            return
        if self.software_guard_enabled.isChecked():
            if self._last_sample is None or self._last_telemetry_received_at is None:
                QMessageBox.warning(self, "PWM", "Нет распознанной телеметрии; сначала настройте мониторинг")
                return
            age = time.monotonic() - self._last_telemetry_received_at
            if age > self.guard.limits.telemetry_timeout_s:
                QMessageBox.warning(self, "PWM", "Телеметрия устарела; включение отклонено")
                return
            violations = self.guard.check(self._last_sample)
            if violations:
                message = "; ".join(violation.message for violation in violations)
                QMessageBox.warning(self, "PWM", f"Текущее состояние нарушает пороги: {message}")
                return
        self._safety_latched = False
        self._send(self.protocol.enable())
        QTimer.singleShot(100, lambda: self._send(self.protocol.enable(None)))

    def _read_device_limits(self, copy_to_inputs: bool = False) -> None:
        if copy_to_inputs:
            self._device_limit_copy_pending.update(self.device_limit_spins)
        self._queue_commands(
            [
                self.protocol.current_limit(),
                self.protocol.voltage_limit(),
                self.protocol.velocity_limit(),
            ]
        )

    def _apply_device_limits(self) -> None:
        if self._manual_control_blocked_by_friction("изменение ограничений SimpleFOC"):
            return
        if not self._allow_parameter_change("изменение ограничений SimpleFOC"):
            return
        selected = [key for key, checkbox in self.device_limit_checks.items() if checkbox.isChecked()]
        if not selected:
            QMessageBox.information(self, "Ограничения", "Отметьте параметры в колонке «Отпр.»")
            return
        if "current_a" in selected and self.device_limit_spins["current_a"].value() <= 1.0:
            answer = QMessageBox.warning(
                self,
                "Низкий предел тока",
                "Предел 1 А или ниже может не позволить установке начать движение. "
                "Он останется в плате до следующего изменения или перезапуска. Отправить?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        command_builders = {
            "current_a": self.protocol.current_limit,
            "voltage_v": self.protocol.voltage_limit,
            "velocity_rad_s": self.protocol.velocity_limit,
        }
        commands: list[str] = []
        for key in selected:
            self.device_limit_confirmed[key].setText("ожидание ответа…")
            commands.append(command_builders[key](self.device_limit_spins[key].value()))
        self._sync_device_limits_to_profile()
        self._save_user_settings()
        self._queue_commands(commands)
        self._read_device_limits(copy_to_inputs=False)

    def _apply_software_limits(self, show_status: bool = True) -> bool:
        if self._manual_control_blocked_by_friction("изменение аварийных порогов"):
            return False
        limits = SafetyLimits(
            current_a=self.current_limit.value(),
            voltage_v=self.voltage_limit.value(),
            velocity_rad_s=self.velocity_limit.value(),
            angle_min_rad=self.angle_min.value(),
            angle_max_rad=self.angle_max.value(),
            trial_timeout_s=self.profile.safety.trial_timeout_s,
            telemetry_timeout_s=self.profile.safety.telemetry_timeout_s,
        )
        try:
            limits.validate()
        except ValueError as exc:
            QMessageBox.warning(self, "Ограничения", str(exc))
            return False
        self.profile.safety = limits
        self.guard = SafetyGuard(limits)
        if self.project:
            self.project.save_profile(self.profile)
        self._save_user_settings()
        if show_status:
            self.statusBar().showMessage(
                "Программные пороги применены; на плату команды не отправлялись", 6000
            )
        return True

    def _selected_pid_loop(self) -> str:
        return self.pid_tab_loops[self.pid_tabs.currentIndex()]

    def _pid_values(self, loop: str) -> dict[str, float]:
        table_widget = self.pid_tables[loop]
        values: dict[str, float] = {}
        try:
            for row, field in enumerate(self.PID_FIELDS):
                values[field] = float(table_widget.item(row, 1).text().replace(",", "."))
        except (AttributeError, ValueError) as exc:
            raise ValueError(f"Контур {loop}: все значения должны быть числами") from exc
        if any(value < 0 for value in values.values()):
            raise ValueError(f"Контур {loop}: отрицательные PID/LPF значения отклонены")
        return values

    def _read_selected_pid(self) -> None:
        loop = self._selected_pid_loop()
        commands = [self.protocol.pid(loop, field) for field in self.PID_FIELDS]
        limit_readers = {
            "current_a": self.protocol.current_limit,
            "voltage_v": self.protocol.voltage_limit,
            "velocity_rad_s": self.protocol.velocity_limit,
        }
        commands.append(limit_readers[self.PID_LIMIT_BINDINGS[loop]]())
        self._queue_commands(commands)

    def _apply_selected_pid(self) -> None:
        if self._manual_control_blocked_by_friction("изменение PID/LPF"):
            return
        if not self._allow_parameter_change("изменение PID/LPF"):
            return
        loop = self._selected_pid_loop()
        try:
            values = self._pid_values(loop)
        except ValueError as exc:
            QMessageBox.warning(self, "PID", str(exc))
            return
        commands = [self.protocol.pid(loop, field, value) for field, value in values.items()]
        profile_params = getattr(self.profile, loop)
        profile_params.p = values["p"]
        profile_params.i = values["i"]
        profile_params.d = values["d"]
        profile_params.output_ramp = values["ramp"]
        profile_params.lpf_tf = values["lpf"]
        if self.project:
            self.project.save_profile(self.profile)
        self._save_user_settings()
        self._queue_commands(commands, self._read_selected_pid)

    def _allow_parameter_change(self, operation: str) -> bool:
        if not self._pwm_requested:
            return True
        if not self.allow_live_changes_checkbox.isChecked():
            QMessageBox.warning(
                self,
                "Изменение отклонено",
                f"PWM сейчас включён. Сначала отключите его или явно разрешите {operation} при работающем моторе.",
            )
            return False
        answer = QMessageBox.warning(
            self,
            "Live-tuning",
            f"{operation.capitalize()} при включённом PWM может вызвать рывок. Продолжить?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        return answer == QMessageBox.StandardButton.Yes

    def _apply_monitoring(self) -> None:
        if self._manual_control_blocked_by_friction("ручная настройка мониторинга"):
            return
        mask = self._monitor_mask_from_ui()
        if mask is None:
            return
        self._prepare_monitor_configuration(mask, reset_statistics=True, reset_recovery=True)
        self._save_user_settings()
        self._send_monitor_configuration("настроен пользователем")

    def _monitor_mask_from_ui(self) -> str | None:
        for name, checkbox in self.plot_checks.items():
            if checkbox.isChecked():
                self.monitor_checks[name].setChecked(True)
        mask = "".join(
            "1" if self.monitor_checks[name].isChecked() else "0" for name in MONITOR_FIELDS
        )
        if "1" not in mask:
            QMessageBox.warning(self, "Мониторинг", "Выберите хотя бы один сигнал")
            return None
        return mask

    def _prepare_monitor_configuration(
        self,
        mask: str,
        *,
        reset_statistics: bool,
        reset_recovery: bool,
    ) -> None:
        self.monitor_mask = mask
        if reset_statistics:
            self.telemetry_statistics.reset()
            self._rejected_telemetry_count = 0
        self._monitoring_requested = True
        if reset_recovery:
            self._monitor_restart_count = 0
            self._monitor_recovery_attempt = 0

    def _send_monitor_configuration(self, reason: str) -> None:
        self._mark_monitor_configuration_started()
        self._queue_commands(
            [
                self.protocol.monitor_clear(),
                self.protocol.monitor_downsample(self._active_monitor_downsample()),
                self.protocol.monitor_variables(self.monitor_mask),
            ]
        )
        self._log(
            "INFO",
            f"Мониторинг {reason}: mask={self.monitor_mask}; токи потока переводятся из мА в А",
        )

    def _mark_monitor_configuration_started(self) -> None:
        now = time.monotonic()
        self._monitor_configured_at = now
        self._last_monitor_restart_at = now
        self._last_telemetry_received_at = None
        self.monitor_health_label.setText("Поток: ожидание первого отсчёта…")

    def _restart_monitoring(self) -> None:
        if not self.device.connected:
            QMessageBox.information(self, "Мониторинг", "Сначала подключите мотор")
            return
        self._monitoring_requested = True
        self._monitor_restart_count += 1
        self._monitor_recovery_attempt += 1
        self._start_transport_recovery("перезапущен вручную")

    def _start_transport_recovery(self, reason: str) -> None:
        if self._transport_recovery_in_progress or not self.device.connected:
            return
        if self._configuration_apply_in_progress or self._command_queue or self._command_timer.isActive():
            self.monitor_health_label.setText(
                "Поток: восстановление ожидает завершения отправки конфигурации"
            )
            return
        try:
            self.device.discard_pending_input()
            self.device.set_dtr(False)
        except Exception as exc:  # noqa: BLE001
            self._log("ERROR", f"Не удалось опустить DTR для восстановления телеметрии: {exc}")
            self._send_monitor_configuration("повторно настроен без DTR")
            return
        self._transport_recovery_in_progress = True
        self.monitor_health_label.setText(
            "Поток: USB-передача зависла — восстанавливается без изменения PWM"
        )
        self._log("TELEMETRY_RECOVERY", "DTR выключен на 80 мс; состояние мотора не меняется")
        QTimer.singleShot(80, lambda: self._finish_transport_recovery(reason))

    def _finish_transport_recovery(self, reason: str) -> None:
        if not self.device.connected:
            self._transport_recovery_in_progress = False
            return
        try:
            self.device.set_dtr(True)
        except Exception as exc:  # noqa: BLE001
            self._transport_recovery_in_progress = False
            self._log("ERROR", f"Не удалось поднять DTR после восстановления: {exc}")
            return
        self._transport_recovery_in_progress = False
        self._transport_recovery_count += 1
        self._send_monitor_configuration(f"{reason}; USB DTR восстановлен")

    def _on_plot_signal_toggled(self, name: str, checked: bool) -> None:
        if not checked or self.monitor_checks[name].isChecked():
            return
        self.monitor_checks[name].setChecked(True)
        if self.device.connected:
            self._apply_monitoring()

    def _clear_live_plot(self) -> None:
        for times, values in self._telemetry_series.values():
            times.clear()
            values.clear()
        for curve in self.telemetry_curves.values():
            curve.setData([], [])

    def _toggle_recording(self) -> None:
        if self.telemetry_recorder.active:
            path = self.telemetry_recorder.stop()
            self.record_button.setText("Начать запись CSV")
            self._log("RECORD", f"Запись остановлена: {path}")
            return
        if self.project is None:
            QMessageBox.warning(self, "Запись", "Сначала создайте или откройте проект FOCTwin")
            return
        path = self.telemetry_recorder.start(self.project.new_telemetry_path("manual"))
        self._reported_recorder_error = None
        self.record_button.setText("Остановить запись")
        self._log("RECORD", f"Запись начата: {path}")

    def _send(self, command: str) -> bool:
        self._log("TX", command)
        try:
            self.device.send(command)
        except Exception as exc:  # noqa: BLE001
            self._log("ERROR", str(exc))
            self.statusBar().showMessage(str(exc), 5000)
            return False
        if command == self.protocol.enable():
            self._pwm_requested = True
            self.pwm_state_label.setText("PWM: включение запрошено")
        elif command == self.protocol.disable():
            self._pwm_requested = False
            self.pwm_state_label.setText("PWM: отключение запрошено")
        return True

    def _emergency_stop(self) -> None:
        self._cancel_queued_commands()
        self._configuration_apply_in_progress = False
        self.apply_modes_button.setEnabled(True)
        self.apply_modes_button.setText("Применить режимы и всю конфигурацию")
        sent = self.device.emergency_stop()
        self._pwm_requested = False
        self.pwm_state_label.setText("PWM: отключён (best-effort)")
        self._log("EMERGENCY", f"Best-effort stop; sent: {sent or 'nothing (not connected)'}")
        self.statusBar().showMessage("Аварийный стоп отправлен; при сомнениях отключите питание", 10000)
        experiment = self._friction_experiment
        if experiment is not None and self._friction_running():
            experiment.abort("Аварийный стоп FOCTwin")
            self._finalize_friction_experiment("interrupted", experiment.abort_reason)

    def _compile_scenario(self) -> None:
        compiler = ScenarioCompiler(self.protocol, self.profile.safety)
        try:
            steps = compiler.compile(self.scenario_editor.toPlainText())
        except (ScenarioError, ValueError) as exc:
            self.scenario_output.setPlainText(f"ОШИБКА: {exc}")
            return
        output: list[str] = []
        for step in steps:
            rendered = ", ".join(step.commander_commands) or "локальная операция"
            output.append(f"{step.line_number:03d}: {step.operation} → {rendered}")
        self.scenario_output.setPlainText("\n".join(output) + "\n\nСценарий прошёл проверку.")

    def _start_matlab(self) -> None:
        if self.matlab.connected:
            self.statusBar().showMessage("MATLAB уже запущен", 3000)
            return
        self.side_matlab.setText("● MATLAB запускается…")

        def worker() -> None:
            try:
                self.matlab.start()
            except Exception as exc:  # noqa: BLE001
                self.signals.matlab_state.emit(False, str(exc))
            else:
                self.signals.matlab_state.emit(True, "MATLAB Engine запущен")

        threading.Thread(target=worker, name="foctwin-matlab", daemon=True).start()

    def _on_matlab_state(self, connected: bool, message: str) -> None:
        self.side_matlab.setText("● MATLAB запущен" if connected else "● MATLAB недоступен")
        self.dashboard_matlab.setText("Запущен" if connected else "Недоступен")
        self._log("MATLAB", message)

    def _on_device_state(self, connected: bool, message: str) -> None:
        self._connecting = False
        if connected:
            self._connection_requested = True
            self.connect_button.setText("Отключить")
            self.connection_details.setText(message)
            QTimer.singleShot(100, self._initialize_connection)
        else:
            self._cancel_queued_commands()
            self._configuration_apply_in_progress = False
            self._transport_recovery_in_progress = False
            self.apply_modes_button.setEnabled(True)
            self.apply_modes_button.setText("Применить режимы и всю конфигурацию")
            self._pwm_requested = False
            self._last_sample = None
            self._last_telemetry_received_at = None
            self._monitor_configured_at = None
            self.monitor_health_label.setText("Поток: устройство отключено")
            self.pwm_state_label.setText("PWM: состояние неизвестно после разрыва")
            reconnecting = self._connection_requested and self.auto_reconnect_checkbox.isChecked()
            self.connect_button.setText("Отменить переподключение" if reconnecting else "Подключить")
            self.connection_details.setText(
                f"{message}. Ожидание переподключения…" if reconnecting else message
            )
        self.side_connection.setText("● Мотор подключён" if connected else "● Мотор отключён")
        self.dashboard_motor.setText("Подключён" if connected else "Отключён")
        self._log("SERIAL", message)
        experiment = self._friction_experiment
        if not connected and experiment is not None and self._friction_running():
            self._friction_last_sample_sequence = self._telemetry_sequence
            self._friction_recovery_started_at = time.monotonic()
            self._friction_recovery_alerted = False
            self._process_friction_actions(experiment.enter_recovery())
            self._save_friction_checkpoint()

    def _on_device_line(self, received_at: float, line: str) -> None:
        response = parse_commander_response(line)
        if response is not None:
            self.raw_output.appendPlainText(f"RX  {line}")
            self._log("RX", line)
            self._apply_commander_response(response)
            return
        parsed = parse_monitor_line(line, self.monitor_mask)
        if not parsed:
            self.raw_output.appendPlainText(f"RX  {line}")
            if is_monitor_candidate(line, self.monitor_mask):
                self._rejected_telemetry_count += 1
                # Preserve evidence without flooding the persistent log when a cable is noisy.
                count = self._rejected_telemetry_count
                if count <= 3 or count & (count - 1) == 0:
                    self._log(
                        "TELEMETRY_DROP",
                        f"Отброшена повреждённая строка #{count}: {line}",
                    )
            else:
                self._log("RX", line)
            return
        if self.raw_telemetry_checkbox.isChecked():
            self.raw_output.appendPlainText(f"RX  {line}")
        self._telemetry_sequence += 1
        timestamp_s = received_at - self._started_at
        sample = TelemetrySample(
            timestamp_s=timestamp_s,
            sequence=self._telemetry_sequence,
            received_at_utc=datetime.now(timezone.utc).isoformat(),
            raw_angle_rad=parsed.get("angle_rad"),
            raw=line,
            **parsed,
        )
        experiment = self._friction_experiment
        if experiment is not None and self._friction_running():
            sample = experiment.prepare_sample(sample)
        self._last_sample = sample
        self._last_telemetry_received_at = received_at
        if self._monitor_recovery_attempt:
            self._log(
                "TELEMETRY_RECOVERY",
                f"Поток восстановлен после {self._monitor_recovery_attempt} попыток",
            )
            self._monitor_recovery_attempt = 0
        self.telemetry_statistics.add(timestamp_s)
        self.telemetry_recorder.append(sample)
        self.friction_recorder.append(sample)
        for name in MONITOR_FIELDS:
            value = getattr(sample, name)
            if value is None:
                continue
            times, values = self._telemetry_series[name]
            times.append(timestamp_s)
            values.append(value)
            if len(times) > 60000:
                del times[:-60000]
                del values[:-60000]

        if experiment is not None and self._friction_running():
            violation, actions = experiment.add_sample(
                sample,
                received_at,
                angle_prepared=True,
            )
            self._process_friction_actions(actions)
            if violation:
                self._abort_friction_experiment(violation)
                return

        ignored = (
            frozenset({"velocity_rad_s", "angle_rad"})
            if experiment is not None and self._friction_running()
            else frozenset()
        )
        violations = (
            self.guard.check(sample, ignored_signals=ignored)
            if self._pwm_requested and self.software_guard_enabled.isChecked()
            else []
        )
        if self._pwm_requested and violations and not self._safety_latched:
            self._safety_latched = True
            self._log("SAFETY", "; ".join(violation.message for violation in violations))
            self._emergency_stop()

    def _refresh_telemetry_ui(self) -> None:
        sample = self._last_sample
        if sample is not None and sample.sequence != self._displayed_telemetry_sequence:
            self._displayed_telemetry_sequence = sample.sequence
            for name in MONITOR_FIELDS:
                value = getattr(sample, name)
                if value is not None:
                    self.telemetry_values[name].setText(f"{value:.6g}")
        rejected = (
            f" · повреждено/отброшено: {self._rejected_telemetry_count}"
            if self._rejected_telemetry_count
            else ""
        )
        self.monitor_stats_label.setText(
            f"{self.telemetry_statistics.sample_count} отсчётов · "
            f"{self.telemetry_statistics.frequency_hz:.1f} Гц · "
            f"jitter {self.telemetry_statistics.jitter_s * 1000:.2f} мс{rejected}"
        )
        self._refresh_live_plot()
        recorder_error = self.telemetry_recorder.last_error
        if recorder_error and recorder_error != self._reported_recorder_error:
            self._reported_recorder_error = recorder_error
            self.record_button.setText("Начать запись CSV")
            self._log("ERROR", f"Запись телеметрии остановлена: {recorder_error}")
        friction_error = self.friction_recorder.last_error
        if friction_error and self._friction_running():
            self._abort_friction_experiment(f"Ошибка записи данных опыта: {friction_error}")

    def _refresh_live_plot(self) -> None:
        if self.live_plot is None:
            return
        window_s = float(self.plot_window_spin.value())
        last_timestamp: float | None = None
        for name, curve in self.telemetry_curves.items():
            visible = self.plot_checks[name].isChecked()
            curve.setVisible(visible)
            if visible:
                times, values = self._telemetry_series[name]
                if times:
                    start = bisect_left(times, times[-1] - window_s)
                    curve.setData(times[start:], values[start:])
                    last_timestamp = max(last_timestamp or times[-1], times[-1])
                else:
                    curve.setData([], [])
        if self.plot_follow_checkbox.isChecked() and last_timestamp is not None:
            self.live_plot.setXRange(max(0.0, last_timestamp - window_s), last_timestamp, padding=0.0)

    def _check_telemetry_health(self) -> None:
        if not self.device.connected:
            self.monitor_health_label.setText("Поток: устройство отключено")
            return
        if not self._monitoring_requested:
            self.monitor_health_label.setText("Поток: ещё не настроен")
            return
        if self._configuration_apply_in_progress or self._transport_recovery_in_progress:
            return
        now = time.monotonic()
        reference = self._last_telemetry_received_at or self._monitor_configured_at
        if reference is None:
            self.monitor_health_label.setText("Поток: ожидание первого отсчёта…")
            return
        age = max(0.0, now - reference)
        timeout = monitor_stale_timeout(self._active_monitor_downsample())
        if age <= timeout:
            if self._last_telemetry_received_at is None:
                self.monitor_health_label.setText("Поток: ожидание первого отсчёта…")
            else:
                recovery = (
                    f" · восстановлений: {self._monitor_restart_count}"
                    if self._monitor_restart_count
                    else ""
                )
                self.monitor_health_label.setText(f"Поток: работает · возраст {age:.1f} с{recovery}")
            return
        self.monitor_health_label.setText(
            f"Поток: нет отсчётов {age:.1f} с — автоматический перезапуск"
        )
        if now - self._last_monitor_restart_at < timeout:
            return
        if self._command_queue or self._command_timer.isActive():
            self.monitor_health_label.setText(
                "Поток: восстановление ожидает завершения отправки команд"
            )
            return
        self._monitor_restart_count += 1
        self._monitor_recovery_attempt += 1
        if self._monitor_recovery_attempt == 1:
            self._send_monitor_configuration("повторно настроен автоматически")
        else:
            self._start_transport_recovery("перезапущен автоматически")

    def _apply_commander_response(self, response: CommanderResponse) -> None:
        if response.key.startswith("limit."):
            key = response.key.removeprefix("limit.")
            if key in self.device_limit_spins:
                value = float(response.value)
                self.device_limit_confirmed[key].setText(f"{value:g}")
                if key in self._device_limit_copy_pending:
                    self._device_limit_copy_pending.remove(key)
                    self.device_limit_spins[key].setValue(value)
                    self._sync_device_limits_to_profile()
                    self._schedule_user_settings_save()
            return
        if response.key == "enabled":
            self._pwm_requested = bool(response.value)
            self.pwm_state_label.setText("PWM: включён" if response.value else "PWM: отключён")
            return
        if response.key == "motion_mode":
            modes = {
                "torque": MotionMode.TORQUE,
                "vel": MotionMode.VELOCITY,
                "angle": MotionMode.ANGLE,
                "vel open": MotionMode.VELOCITY_OPEN_LOOP,
                "angle open": MotionMode.ANGLE_OPEN_LOOP,
            }
            mode = modes.get(str(response.value).lower())
            if mode is not None:
                index = self.motion_combo.findData(mode)
                self.motion_combo.setCurrentIndex(index)
            return
        if response.key == "torque_mode":
            modes = {
                "volt": TorqueMode.VOLTAGE,
                "dc curr": TorqueMode.DC_CURRENT,
                "foc curr": TorqueMode.FOC_CURRENT,
            }
            mode = modes.get(str(response.value).lower())
            if mode is not None:
                index = self.torque_combo.findData(mode)
                self.torque_combo.setCurrentIndex(index)
            return
        if response.key.startswith("pid."):
            _, loop, field = response.key.split(".")
            if loop in self.pid_tables and field in self.PID_ROW_BY_FIELD:
                row = self.PID_ROW_BY_FIELD[field]
                self.pid_tables[loop].item(row, 1).setText(f"{float(response.value):g}")

    def _log(self, level: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        rendered = f"{timestamp} [{level}] {message}"
        if hasattr(self, "log_view"):
            self.log_view.appendPlainText(rendered)
        if self.project:
            try:
                self.project.event(level, "app", message)
            except Exception:  # noqa: BLE001, S110
                pass

    def _refresh_status(self) -> None:
        self.statusBar().showMessage("Готово. Реальный мотор по умолчанию отключён.")

    def closeEvent(self, event: QCloseEvent) -> None:
        self._save_user_settings()
        experiment = self._friction_experiment
        if experiment is not None and self._friction_running():
            experiment.abort("Приложение закрыто во время опыта")
            self._save_friction_checkpoint()
            if self.project is not None and self._friction_experiment_id is not None:
                self.project.update_experiment(
                    self._friction_experiment_id,
                    "interrupted",
                    finished_at=datetime.now(timezone.utc).isoformat(),
                    error=experiment.abort_reason,
                )
        self._connection_requested = False
        self._settings_save_timer.stop()
        self._command_timer.stop()
        self._command_queue.clear()
        self._reconnect_timer.stop()
        self._telemetry_ui_timer.stop()
        self._telemetry_watchdog_timer.stop()
        self._friction_timer.stop()
        self.telemetry_recorder.stop()
        self.friction_recorder.stop()
        self.device.emergency_stop()
        if self.device.connected:
            try:
                self.device.send(
                    self.protocol.phase_resistance(self.profile.phase_resistance_ohm)
                )
            except Exception:  # noqa: BLE001, S110
                pass
        self.device.disconnect()
        if self.matlab.connected:
            self.matlab.stop()
        event.accept()
