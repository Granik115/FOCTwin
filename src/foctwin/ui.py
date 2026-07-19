from __future__ import annotations

import json
import threading
import time
from pathlib import Path

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QAction, QCloseEvent, QFont
from PySide6.QtWidgets import (
    QAbstractItemView,
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
    QSpinBox,
    QSplitter,
    QStackedWidget,
    QStatusBar,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from foctwin.domain import MotorProfile, MotionMode, SafetyGuard, TorqueMode
from foctwin.matlab_backend import MatlabBackend
from foctwin.project_store import ProjectStore
from foctwin.protocol import CommanderProtocol, parse_monitor_line
from foctwin.scenario import ScenarioCompiler, ScenarioError
from foctwin.serial_device import SerialDevice

try:
    import pyqtgraph as pg
except ImportError:  # pragma: no cover
    pg = None


class DeviceSignals(QObject):
    line = Signal(str)
    state = Signal(bool, str)
    matlab_state = Signal(bool, str)


def spin(value: float, minimum: float = -1e9, maximum: float = 1e9, decimals: int = 6) -> QDoubleSpinBox:
    widget = QDoubleSpinBox()
    widget.setDecimals(decimals)
    widget.setRange(minimum, maximum)
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

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("FOCTwin — Identify. Simulate. Tune.")
        self.resize(1480, 900)
        self.profile = MotorProfile()
        self.protocol = CommanderProtocol(self.profile.command_id)
        self.device = SerialDevice(self.protocol)
        self.guard = SafetyGuard(self.profile.safety)
        self.project: ProjectStore | None = None
        self.matlab = MatlabBackend(Path(__file__).resolve().parents[3] / "matlab")
        self.signals = DeviceSignals()
        self.signals.line.connect(self._on_device_line)
        self.signals.state.connect(self._on_device_state)
        self.signals.matlab_state.connect(self._on_matlab_state)
        self.device.on_line = self.signals.line.emit
        self.device.on_state = self.signals.state.emit
        self._telemetry_time: list[float] = []
        self._telemetry_angle: list[float] = []
        self._telemetry_velocity: list[float] = []
        self._started_at = time.monotonic()
        self._build_actions()
        self._build_shell()
        self._build_pages()
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
            "Полный доступ к соединению, режимам SimpleFOC, ограничениям, PID/LPF и цели.",
        )
        splitter = QSplitter(Qt.Orientation.Horizontal)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        connection = QGroupBox("Serial / Commander")
        connection_form = QFormLayout(connection)
        self.port_edit = QLineEdit("COM3")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(("115200", "230400", "460800", "921600"))
        self.device_id_edit = QLineEdit("A")
        self.device_id_edit.setMaxLength(1)
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        self.connect_button = QPushButton("Подключить")
        self.connect_button.clicked.connect(self._toggle_connection)
        row_layout.addWidget(self.connect_button)
        connection_form.addRow("COM-порт", self.port_edit)
        connection_form.addRow("Скорость", self.baud_combo)
        connection_form.addRow("ID мотора", self.device_id_edit)
        connection_form.addRow(row)
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
        apply_modes = QPushButton("Применить режимы")
        apply_modes.clicked.connect(self._apply_modes)
        send_target = QPushButton("Отправить цель")
        send_target.clicked.connect(lambda: self._send(self.protocol.target(self.target_spin.value())))
        enable = QPushButton("Включить PWM")
        enable.clicked.connect(lambda: self._send(self.protocol.enable()))
        disable = QPushButton("Отключить PWM")
        disable.setObjectName("dangerButton")
        disable.clicked.connect(self._emergency_stop)
        control_form.addRow("Контур движения", self.motion_combo)
        control_form.addRow("Контур момента", self.torque_combo)
        control_form.addRow("Цель", self.target_spin)
        control_form.addRow(apply_modes)
        control_form.addRow(send_target)
        control_form.addRow(enable, disable)
        left_layout.addWidget(control)
        limits = QGroupBox("Жёсткие ограничения приложения")
        limits_form = QFormLayout(limits)
        self.current_limit = spin(self.profile.safety.current_a, 0.001, 100.0)
        self.voltage_limit = spin(self.profile.safety.voltage_v, 0.001, 100.0)
        self.velocity_limit = spin(self.profile.safety.velocity_rad_s, 0.000001, 1000.0)
        self.angle_min = spin(self.profile.safety.angle_min_rad)
        self.angle_max = spin(self.profile.safety.angle_max_rad)
        apply_limits = QPushButton("Проверить и отправить ограничения")
        apply_limits.clicked.connect(self._apply_limits)
        limits_form.addRow("Ток, А", self.current_limit)
        limits_form.addRow("Напряжение, В", self.voltage_limit)
        limits_form.addRow("Скорость, рад/с", self.velocity_limit)
        limits_form.addRow("Координата min, рад", self.angle_min)
        limits_form.addRow("Координата max, рад", self.angle_max)
        limits_form.addRow(apply_limits)
        left_layout.addWidget(limits)
        left_layout.addStretch(1)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        pid_group = QGroupBox("PID / LPF / anti-windup")
        pid_layout = QVBoxLayout(pid_group)
        self.pid_tabs = QTabWidget()
        for loop, params in (
            ("Положение", self.profile.angle),
            ("Скорость", self.profile.velocity),
            ("Ток Q", self.profile.current_q),
            ("Ток D", self.profile.current_d),
        ):
            pid_table = table(("Параметр", "Значение"), 7)
            values = (
                ("P", params.p),
                ("I", params.i),
                ("D", params.d),
                ("Output ramp", params.output_ramp),
                ("Output limit", params.output_limit),
                ("LPF Tf", params.lpf_tf),
                ("Kc (только модель)", params.anti_windup_kc),
            )
            for row_index, (name, value) in enumerate(values):
                pid_table.setItem(row_index, 0, QTableWidgetItem(name))
                pid_table.setItem(row_index, 1, QTableWidgetItem(f"{value:g}"))
            self.pid_tabs.addTab(pid_table, loop)
        pid_layout.addWidget(self.pid_tabs)
        pid_buttons = QHBoxLayout()
        pid_buttons.addWidget(QPushButton("Считать из мотора"))
        pid_buttons.addWidget(QPushButton("Применить выбранный контур"))
        pid_layout.addLayout(pid_buttons)
        right_layout.addWidget(pid_group)
        telemetry = QGroupBox("Живая телеметрия")
        telemetry_layout = QVBoxLayout(telemetry)
        if pg is not None:
            self.live_plot = pg.PlotWidget()
            self.live_plot.showGrid(x=True, y=True, alpha=0.25)
            self.angle_curve = self.live_plot.plot(pen=pg.mkPen("#00bfff", width=2), name="angle")
            self.velocity_curve = self.live_plot.plot(pen=pg.mkPen("#40e0d0", width=2), name="velocity")
            telemetry_layout.addWidget(self.live_plot)
        else:
            self.live_plot = None
            telemetry_layout.addWidget(QLabel("Установите pyqtgraph для живых графиков"))
        right_layout.addWidget(telemetry, 1)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes((480, 900))
        layout.addWidget(splitter, 1)
        return page

    def _identification_page(self) -> QWidget:
        page, layout = titled_page(
            "Идентификация цифрового двойника",
            "Планирование безопасных опытов и подбор механических/электрических параметров.",
        )
        config = QSplitter(Qt.Orientation.Horizontal)
        parameters = QGroupBox("Подбираемые параметры")
        parameters_layout = QVBoxLayout(parameters)
        self.ident_table = table(("Вкл.", "Параметр", "Начало", "Min", "Max", "Ед."), 9)
        rows = (
            (True, "J", 0.07, 1e-4, 2.0, "кг·м²"),
            (True, "Viscous friction", 1e-5, 0.0, 10.0, "Н·м·с/рад"),
            (True, "Coulomb friction", 0.0, 0.0, 10.0, "Н·м"),
            (True, "Breakaway friction", 0.0, 0.0, 15.0, "Н·м"),
            (True, "Breakaway velocity", 0.01, 1e-5, 0.5, "рад/с"),
            (False, "Rs", 0.675, 0.1, 2.0, "Ом"),
            (False, "Ld", 0.0013, 1e-5, 0.02, "Гн"),
            (False, "Lq", 0.0013, 1e-5, 0.02, "Гн"),
            (False, "Flux linkage", 0.05897, 0.001, 0.2, "Вб"),
        )
        for row_index, values in enumerate(rows):
            check = QCheckBox()
            check.setChecked(values[0])
            self.ident_table.setCellWidget(row_index, 0, check)
            for column, value in enumerate(values[1:], start=1):
                self.ident_table.setItem(row_index, column, QTableWidgetItem(str(value)))
        parameters_layout.addWidget(self.ident_table)
        excitation = QGroupBox("План возбуждений")
        excitation_form = QFormLayout(excitation)
        self.excitation_combo = QComboBox()
        self.excitation_combo.addItems(("Ступень", "Ramp", "Синус", "Chirp", "PRBS", "Свой сценарий"))
        self.excitation_amplitude = spin(0.1, 0.0, 1000.0)
        self.excitation_duration = spin(3.0, 0.01, 3600.0)
        repeats = QSpinBox()
        repeats.setRange(1, 1000)
        repeats.setValue(5)
        positions = QLineEdit("-1.0; -0.5; 0; 0.5; 1.0")
        queue_button = QPushButton("Добавить серию опытов")
        excitation_form.addRow("Сигнал", self.excitation_combo)
        excitation_form.addRow("Амплитуда", self.excitation_amplitude)
        excitation_form.addRow("Длительность, с", self.excitation_duration)
        excitation_form.addRow("Повторы", repeats)
        excitation_form.addRow("Начальные положения", positions)
        excitation_form.addRow(queue_button)
        config.addWidget(parameters)
        config.addWidget(excitation)
        config.setSizes((900, 430))
        layout.addWidget(config)
        self.ident_queue = table(("№", "Сигнал", "Положение", "Статус", "Ошибка модели"), 0)
        layout.addWidget(self.ident_queue, 1)
        controls = QHBoxLayout()
        controls.addWidget(QPushButton("Только записать реальные данные"))
        controls.addWidget(QPushButton("Запустить идентификацию"))
        controls.addWidget(QPushButton("Продолжить из checkpoint"))
        controls.addWidget(QPushButton("Остановить"))
        layout.addLayout(controls)
        return page

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

    def _toggle_connection(self) -> None:
        if self.device.connected:
            self.device.disconnect()
            return
        self.protocol = CommanderProtocol(self.device_id_edit.text().strip() or "A")
        self.device.protocol = self.protocol
        try:
            self.device.connect(self.port_edit.text().strip(), int(self.baud_combo.currentText()))
        except Exception as exc:
            QMessageBox.critical(self, "Serial", str(exc))
            self._log("ERROR", f"Не удалось подключиться: {exc}")

    def _apply_modes(self) -> None:
        motion = self.motion_combo.currentData()
        torque = self.torque_combo.currentData()
        self._send(self.protocol.motion_mode(motion))
        self._send(self.protocol.torque_mode(torque))

    def _apply_limits(self) -> None:
        limits = self.profile.safety
        limits.current_a = self.current_limit.value()
        limits.voltage_v = self.voltage_limit.value()
        limits.velocity_rad_s = self.velocity_limit.value()
        limits.angle_min_rad = self.angle_min.value()
        limits.angle_max_rad = self.angle_max.value()
        try:
            limits.validate()
        except ValueError as exc:
            QMessageBox.warning(self, "Ограничения", str(exc))
            return
        self.guard = SafetyGuard(limits)
        for command in (
            self.protocol.current_limit(limits.current_a),
            self.protocol.voltage_limit(limits.voltage_v),
            self.protocol.velocity_limit(limits.velocity_rad_s),
        ):
            self._send(command)
        if self.project:
            self.project.save_profile(self.profile)

    def _send(self, command: str) -> None:
        self._log("TX", command)
        try:
            self.device.send(command)
        except Exception as exc:
            self._log("ERROR", str(exc))
            self.statusBar().showMessage(str(exc), 5000)

    def _emergency_stop(self) -> None:
        sent = self.device.emergency_stop()
        self._log("EMERGENCY", f"Best-effort stop; sent: {sent or 'nothing (not connected)'}")
        self.statusBar().showMessage("Аварийный стоп отправлен; при сомнениях отключите питание", 10000)

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
            except Exception as exc:
                self.signals.matlab_state.emit(False, str(exc))
            else:
                self.signals.matlab_state.emit(True, "MATLAB Engine запущен")

        threading.Thread(target=worker, name="foctwin-matlab", daemon=True).start()

    def _on_matlab_state(self, connected: bool, message: str) -> None:
        self.side_matlab.setText("● MATLAB запущен" if connected else "● MATLAB недоступен")
        self.dashboard_matlab.setText("Запущен" if connected else "Недоступен")
        self._log("MATLAB", message)

    def _on_device_state(self, connected: bool, message: str) -> None:
        self.connect_button.setText("Отключить" if connected else "Подключить")
        self.side_connection.setText("● Мотор подключён" if connected else "● Мотор отключён")
        self.dashboard_motor.setText("Подключён" if connected else "Отключён")
        self._log("SERIAL", message)

    def _on_device_line(self, line: str) -> None:
        self.raw_output.appendPlainText(f"RX  {line}")
        self._log("RX", line)
        parsed = parse_monitor_line(line)
        if not parsed:
            return
        from foctwin.domain import TelemetrySample

        sample = TelemetrySample(timestamp_s=time.monotonic() - self._started_at, raw=line, **parsed)
        violations = self.guard.check(sample)
        if violations:
            self._log("SAFETY", "; ".join(violation.message for violation in violations))
            self._emergency_stop()
        if sample.angle_rad is not None and sample.velocity_rad_s is not None:
            self._telemetry_time.append(sample.timestamp_s)
            self._telemetry_angle.append(sample.angle_rad)
            self._telemetry_velocity.append(sample.velocity_rad_s)
            if len(self._telemetry_time) > 5000:
                self._telemetry_time = self._telemetry_time[-5000:]
                self._telemetry_angle = self._telemetry_angle[-5000:]
                self._telemetry_velocity = self._telemetry_velocity[-5000:]
            if self.live_plot is not None:
                self.angle_curve.setData(self._telemetry_time, self._telemetry_angle)
                self.velocity_curve.setData(self._telemetry_time, self._telemetry_velocity)

    def _log(self, level: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        rendered = f"{timestamp} [{level}] {message}"
        if hasattr(self, "log_view"):
            self.log_view.appendPlainText(rendered)
        if self.project:
            try:
                self.project.event(level, "app", message)
            except Exception:
                pass

    def _refresh_status(self) -> None:
        self.statusBar().showMessage("Готово. Реальный мотор по умолчанию отключён.")

    def closeEvent(self, event: QCloseEvent) -> None:
        self.device.emergency_stop()
        self.device.disconnect()
        if self.matlab.connected:
            self.matlab.stop()
        event.accept()

