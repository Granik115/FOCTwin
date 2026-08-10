"""Non-modal instruction window for the 0.4.2b5 attended hardware workflow."""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import replace
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QStandardPaths, Qt, QTimer, QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from foctwin import __version__
from foctwin.current_trial_controller import (
    CurrentTrialController,
    CurrentTrialControllerError,
    CurrentTrialDecision,
    CurrentTrialEnvironment,
)
from foctwin.instruction_runner import (
    InstructionRunner,
    InstructionRunnerError,
    InstructionSnapshot,
    InstructionStore,
    ScanResult,
)


def _default_exchange_root() -> Path:
    documents = QStandardPaths.writableLocation(
        QStandardPaths.StandardLocation.DocumentsLocation
    )
    base = Path(documents) if documents else Path.home()
    return base / "AutotunerExchange"


class InstructionRunnerDialog(QDialog):
    POLL_INTERVAL_MS = 1_500

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        state_root: Path | None = None,
        exchange_root: Path | None = None,
        auto_scan: bool = True,
        current_trial_controller: CurrentTrialController | None = None,
        current_trial_environment_factory: Callable[[], CurrentTrialEnvironment]
        | None = None,
        current_trial_start_callback: Callable[[CurrentTrialDecision], None] | None = None,
        current_trial_abort_callback: Callable[[str], None] | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(f"FOCTwin {__version__} — локальные инструкции")
        self.setModal(False)
        self.resize(1080, 780)
        self.setMinimumSize(820, 620)

        local_data = QStandardPaths.writableLocation(
            QStandardPaths.StandardLocation.AppLocalDataLocation
        )
        data_root = state_root or Path(local_data or str(Path.home())) / "instruction_runner"
        self.current_trial_controller = current_trial_controller or CurrentTrialController(
            data_root / "current_trial_controller"
        )
        self.current_trial_environment_factory = (
            current_trial_environment_factory or CurrentTrialEnvironment
        )
        self.current_trial_start_callback = current_trial_start_callback
        self.current_trial_abort_callback = current_trial_abort_callback
        self.store = InstructionStore(data_root)
        saved_root = self.store.get_metadata("exchange_root")
        selected_root = exchange_root or (Path(saved_root) if saved_root else _default_exchange_root())
        self.runner = InstructionRunner(
            self.store,
            selected_root,
            app_version=__version__,
            current_trial_gateway=self._current_trial_gateway,
            context_factory=lambda: self.current_trial_environment_factory().to_dict(),
        )
        self._last_history_signature = ""

        self._build_ui(auto_scan=auto_scan)
        self._render_snapshot(self.runner.snapshot())

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(self.POLL_INTERVAL_MS)
        self.poll_timer.timeout.connect(self._poll)
        if auto_scan:
            self.poll_timer.start()
            QTimer.singleShot(100, self._scan_now)

    def _build_ui(self, *, auto_scan: bool) -> None:
        root = QVBoxLayout(self)

        self.safety_label = QLabel(
            "АППАРАТНАЯ БЕТА: удалённый план сам по себе не включает PWM. Вы можете локально "
            "разрешить только выбранный неизменяемый план без таймера; отдельная команда START "
            "использует разрешение один раз. Перезапуск или изменение условий безопасности "
            "его отменяет."
        )
        self.safety_label.setObjectName("danger")
        self.safety_label.setWordWrap(True)
        root.addWidget(self.safety_label)

        status_group = QGroupBox("Состояние обработчика")
        status_form = QFormLayout(status_group)
        self.instance_value = QLabel("—")
        self.instance_value.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.root_value = QLabel("—")
        self.root_value.setWordWrap(True)
        self.root_value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.state_value = QLabel("—")
        self.scan_value = QLabel("—")
        self.counts_value = QLabel("—")
        self.error_value = QLabel("нет")
        self.error_value.setWordWrap(True)
        status_form.addRow("Instance ID", self.instance_value)
        status_form.addRow("Корень обмена", self.root_value)
        status_form.addRow("Состояние", self.state_value)
        status_form.addRow("Последняя проверка", self.scan_value)
        status_form.addRow("Команды", self.counts_value)
        status_form.addRow("Последняя ошибка", self.error_value)
        root.addWidget(status_group)

        controls = QHBoxLayout()
        self.choose_root_button = QPushButton("Выбрать папку обмена…")
        self.choose_root_button.clicked.connect(self._choose_root)
        self.open_root_button = QPushButton("Открыть папку")
        self.open_root_button.clicked.connect(self._open_root)
        self.copy_id_button = QPushButton("Скопировать Instance ID")
        self.copy_id_button.clicked.connect(self._copy_instance_id)
        self.scan_button = QPushButton("Проверить сейчас")
        self.scan_button.clicked.connect(self._scan_now)
        self.auto_scan_checkbox = QCheckBox("Автопроверка каждые 1,5 с")
        self.auto_scan_checkbox.setChecked(auto_scan)
        self.auto_scan_checkbox.toggled.connect(self._toggle_auto_scan)
        for widget in (
            self.choose_root_button,
            self.open_root_button,
            self.copy_id_button,
            self.scan_button,
            self.auto_scan_checkbox,
        ):
            controls.addWidget(widget)
        root.addLayout(controls)

        layout_group = QGroupBox("Папки для FolderBridge")
        layout_form = QFormLayout(layout_group)
        self.inbox_value = QLabel("—")
        self.outbox_value = QLabel("—")
        self.inbox_value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.outbox_value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout_form.addRow("Drive → ПК", self.inbox_value)
        layout_form.addRow("ПК → Drive", self.outbox_value)
        layout_hint = QLabel(
            "Создайте в FolderBridge два задания в режиме «Копирование»: "
            "Autotuner/to_pc → inbox и outbox → Autotuner/from_pc. "
            "Входящие JSON не удаляются; повторную обработку блокирует SQLite."
        )
        layout_hint.setObjectName("hint")
        layout_hint.setWordWrap(True)
        layout_form.addRow("Схема", layout_hint)
        root.addWidget(layout_group)

        samples_group = QGroupBox("Локальная проверка без Google Drive и мотора")
        samples_layout = QGridLayout(samples_group)
        sample_buttons = (
            ("Создать PING", "ping"),
            ("Запросить статус", "get_status"),
            ("Список возможностей", "list_capabilities"),
            ("Самопроверка", "self_test"),
            ("DRY_RUN мотора", "dry_run"),
            ("Имитировать токовый опыт", "run_current_trial"),
            ("Поставить опыт в очередь", "queue_current_trial"),
        )
        self.sample_buttons: list[QPushButton] = []
        for index, (title, command_type) in enumerate(sample_buttons):
            button = QPushButton(title)
            button.clicked.connect(
                lambda checked=False, value=command_type: self._create_sample(value)
            )
            samples_layout.addWidget(button, index // 4, index % 4)
            self.sample_buttons.append(button)
        self.cancel_sample_button = QPushButton("Отменить выбранный ожидающий опыт")
        self.cancel_sample_button.clicked.connect(self._cancel_selected_request)
        self.arm_sample_button = QPushButton("Разрешить выбранный план")
        self.arm_sample_button.clicked.connect(self._arm_selected_request)
        self.start_sample_button = QPushButton("Создать START для разрешённого плана")
        self.start_sample_button.clicked.connect(self._start_selected_request)
        samples_layout.addWidget(self.arm_sample_button, 1, 3)
        samples_layout.addWidget(self.start_sample_button, 2, 0, 1, 2)
        samples_layout.addWidget(self.cancel_sample_button, 2, 2, 1, 2)
        root.addWidget(samples_group)

        history_group = QGroupBox("Последние инструкции")
        history_layout = QVBoxLayout(history_group)
        self.history_table = QTableWidget(0, 6)
        self.history_table.setHorizontalHeaderLabels(
            ("Получена", "Тип", "Состояние", "Command ID", "Код", "Результат / ошибка")
        )
        self.history_table.setAlternatingRowColors(True)
        self.history_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.history_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        header = self.history_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.ResizeMode.Stretch)
        history_layout.addWidget(self.history_table)
        root.addWidget(history_group, 1)

        technical = QGroupBox("Технический журнал окна")
        technical.setCheckable(True)
        technical.setChecked(False)
        technical_layout = QVBoxLayout(technical)
        self.technical_log = QPlainTextEdit()
        self.technical_log.setReadOnly(True)
        self.technical_log.setMaximumHeight(150)
        self.technical_log.document().setMaximumBlockCount(2_000)
        technical_layout.addWidget(self.technical_log)
        technical.toggled.connect(lambda checked: self.technical_log.setVisible(checked))
        self.technical_log.setVisible(False)
        root.addWidget(technical)

    def _append_log(self, level: str, message: str) -> None:
        timestamp = datetime.now().astimezone().strftime("%H:%M:%S")
        self.technical_log.appendPlainText(f"[{timestamp}] {level}: {message}")

    def _toggle_auto_scan(self, enabled: bool) -> None:
        if enabled:
            self.poll_timer.start()
            self._scan_now()
        else:
            self.poll_timer.stop()

    def _poll(self) -> None:
        if self.auto_scan_checkbox.isChecked():
            self._scan_now(silent=True)

    def _scan_now(self, checked: bool = False, *, silent: bool = False) -> None:
        del checked
        self.scan_button.setEnabled(False)
        try:
            result = self.runner.scan_once()
        except (InstructionRunnerError, OSError) as exc:
            self._append_log("ERROR", str(exc))
            if not silent:
                QMessageBox.critical(self, "Инструкции не обработаны", str(exc))
        else:
            if not silent or self._result_has_changes(result):
                self._append_log("INFO", self._result_text(result))
        finally:
            self.scan_button.setEnabled(True)
            self._render_snapshot(self.runner.snapshot())

    @staticmethod
    def _result_has_changes(result: ScanResult) -> bool:
        return bool(result.accepted or result.completed or result.rejected or result.ignored)

    @staticmethod
    def _result_text(result: ScanResult) -> str:
        return (
            f"проверка завершена: принято {result.accepted}, выполнено {result.completed}, "
            f"отклонено {result.rejected}, не этому экземпляру {result.ignored}, "
            f"уже известно {result.duplicates}, отложено {result.deferred}, "
            f"ожидает {result.waiting}"
        )

    def _current_trial_gateway(
        self,
        action: str,
        command_id: str,
        arguments: dict[str, object],
    ) -> dict[str, object]:
        environment = self.current_trial_environment_factory()
        try:
            if action == "submit":
                decision = self.current_trial_controller.submit_instruction(
                    command_id, arguments, environment
                )
            elif action == "refresh":
                decision = self.current_trial_controller.refresh_instruction(
                    command_id, environment
                )
            elif action == "start":
                if self.current_trial_start_callback is None:
                    raise CurrentTrialControllerError(
                        "executor_not_connected",
                        "Исполнитель токового опыта не подключён к окну инструкций",
                    )
                decision = self.current_trial_controller.start_instruction(
                    command_id, environment
                )
                try:
                    self.current_trial_start_callback(decision)
                except Exception as exc:
                    self.current_trial_controller.fail_instruction_start(
                        command_id, str(exc)
                    )
                    raise CurrentTrialControllerError(
                        "executor_start_failed", str(exc)
                    ) from exc
            elif action == "cancel":
                current = self.current_trial_controller.get_by_command(command_id)
                if current is not None and current.state.value == "running":
                    if self.current_trial_abort_callback is None:
                        raise CurrentTrialControllerError(
                            "abort_unavailable",
                            "Локальный аварийный обработчик не подключён",
                        )
                    self.current_trial_abort_callback(command_id)
                    decision = self.current_trial_controller.get_by_command(command_id)
                    if decision is None:
                        raise CurrentTrialControllerError(
                            "request_not_found", "Запрос исчез после остановки"
                        )
                else:
                    decision = self.current_trial_controller.cancel_instruction(command_id)
            else:
                raise CurrentTrialControllerError(
                    "unsupported_action", f"Неизвестное действие контроллера: {action}"
                )
        except CurrentTrialControllerError as exc:
            raise InstructionRunnerError(f"{exc.code}: {exc}") from exc
        return decision.to_dict()

    def _create_sample(self, command_type: str) -> None:
        try:
            path = self.runner.create_sample_command(command_type)
        except InstructionRunnerError as exc:
            QMessageBox.critical(self, "Шаблон не создан", str(exc))
            self._append_log("ERROR", str(exc))
            return
        self._append_log("INFO", f"создан {path.name}")
        self._scan_now()

    def _cancel_selected_request(self, checked: bool = False) -> None:
        del checked
        row = self.history_table.currentRow()
        if row < 0:
            QMessageBox.information(
                self,
                "Отмена инструкции",
                "Сначала выберите строку ожидающего run_current_trial",
            )
            return
        command_type = self.history_table.item(row, 1).text()
        state = self.history_table.item(row, 2).text()
        command_id = self.history_table.item(row, 3).text()
        if command_type != "run_current_trial" or state not in {
            "waiting_for_motor",
            "waiting_for_permission",
            "ready",
            "hardware_running",
        }:
            QMessageBox.information(
                self,
                "Отмена инструкции",
                "Выбранная строка не является ожидающим токовым опытом",
            )
            return
        try:
            path = self.runner.create_sample_command(
                "cancel_current_trial", target_command_id=command_id
            )
        except InstructionRunnerError as exc:
            QMessageBox.critical(self, "Отмена не создана", str(exc))
            return
        self._append_log("INFO", f"создан {path.name} для отмены {command_id}")
        self._scan_now()

    def _selected_current_trial(self, allowed_states: set[str]) -> tuple[str, str] | None:
        row = self.history_table.currentRow()
        if row < 0:
            QMessageBox.information(
                self,
                "Токовый опыт",
                "Сначала выберите строку run_current_trial",
            )
            return None
        command_type = self.history_table.item(row, 1).text()
        state = self.history_table.item(row, 2).text()
        command_id = self.history_table.item(row, 3).text()
        if command_type != "run_current_trial" or state not in allowed_states:
            QMessageBox.information(
                self,
                "Токовый опыт",
                f"Нужен run_current_trial в состоянии: {', '.join(sorted(allowed_states))}",
            )
            return None
        return command_id, state

    def _arm_selected_request(self, checked: bool = False) -> None:
        del checked
        selected = self._selected_current_trial(
            {"waiting_for_motor", "waiting_for_permission"}
        )
        if selected is None:
            return
        command_id, _state = selected
        decision = self.current_trial_controller.get_by_command(command_id)
        if decision is None:
            QMessageBox.warning(self, "Разрешение", "Запрос не найден в контроллере")
            return
        if not self._confirm_current_trial_arm(command_id, decision.config):
            return
        environment = replace(
            self.current_trial_environment_factory(),
            local_permission=True,
        )
        try:
            self.current_trial_controller.arm_instruction(command_id, environment)
        except CurrentTrialControllerError as exc:
            QMessageBox.warning(self, "Разрешение не выдано", str(exc))
            return
        self._append_log(
            "ARM",
            f"одноразово разрешён {command_id} без таймера",
        )
        self._scan_now()

    def _confirm_current_trial_arm(
        self,
        command_id: str,
        config: dict[str, object],
    ) -> bool:
        dialog = QDialog(self)
        dialog.setObjectName("currentTrialArmDialog")
        dialog.setWindowTitle("Одноразовое разрешение аппаратного опыта")
        dialog.setModal(True)
        dialog.setMinimumSize(520, 380)

        screen = QApplication.screenAt(self.frameGeometry().center())
        if screen is None:
            screen = QApplication.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            dialog.resize(
                min(720, max(520, available.width() - 80)),
                min(540, max(380, available.height() - 80)),
            )
        else:
            dialog.resize(720, 540)

        layout = QVBoxLayout(dialog)
        summary = QLabel(
            "Разрешается только указанный Command ID и только показанный неизменяемый план. "
            "Разрешение не ограничено таймером, но используется ровно один раз отдельной "
            "командой START. Закрытие FOCTwin или изменение условий безопасности его отменяет.\n\n"
            f"Command ID: {command_id}"
        )
        summary.setWordWrap(True)
        summary.setObjectName("danger")
        layout.addWidget(summary)

        details_label = QLabel("Полные параметры опыта:")
        layout.addWidget(details_label)
        details = QPlainTextEdit()
        details.setObjectName("currentTrialArmDetails")
        details.setReadOnly(True)
        details.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        details.setPlainText(json.dumps(config, ensure_ascii=False, indent=2))
        layout.addWidget(details, 1)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Yes
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.setObjectName("currentTrialArmButtons")
        allow_button = buttons.button(QDialogButtonBox.StandardButton.Yes)
        cancel_button = buttons.button(QDialogButtonBox.StandardButton.Cancel)
        if allow_button is not None:
            allow_button.setText("Разрешить план")
            allow_button.setDefault(True)
            allow_button.setAutoDefault(True)
        if cancel_button is not None:
            cancel_button.setText("Отмена")
            cancel_button.setDefault(False)
            cancel_button.setAutoDefault(False)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        return dialog.exec() == QDialog.DialogCode.Accepted

    def _start_selected_request(self, checked: bool = False) -> None:
        del checked
        selected = self._selected_current_trial({"ready"})
        if selected is None:
            return
        command_id, _state = selected
        try:
            path = self.runner.create_sample_command(
                "start_current_trial", target_command_id=command_id
            )
        except InstructionRunnerError as exc:
            QMessageBox.warning(self, "START не создан", str(exc))
            return
        self._append_log("INFO", f"создан {path.name} для запуска {command_id}")
        self._scan_now()

    def _choose_root(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "Выберите корень AutotunerExchange",
            str(self.runner.exchange_root),
        )
        if not selected:
            return
        try:
            self.runner.configure_exchange_root(Path(selected))
        except InstructionRunnerError as exc:
            QMessageBox.critical(self, "Папка не принята", str(exc))
            self._append_log("ERROR", str(exc))
            return
        self._append_log("INFO", f"корень обмена изменён: {selected}")
        self._render_snapshot(self.runner.snapshot())
        self._scan_now()

    def _open_root(self) -> None:
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.runner.exchange_root)))

    def _copy_instance_id(self) -> None:
        QApplication.clipboard().setText(self.runner.instance_id)
        self._append_log("INFO", "Instance ID скопирован")

    @staticmethod
    def _format_time(value: str) -> str:
        if not value:
            return "—"
        try:
            parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        except ValueError:
            return value
        return parsed.astimezone().strftime("%d.%m.%Y %H:%M:%S")

    @staticmethod
    def _record_summary(record) -> str:
        if record.error_message:
            return record.error_message
        if not record.result:
            return "—"
        compact = json.dumps(record.result, ensure_ascii=False, separators=(", ", ": "))
        return compact if len(compact) <= 240 else compact[:237] + "…"

    def _render_snapshot(self, snapshot: InstructionSnapshot) -> None:
        self.instance_value.setText(snapshot.instance_id)
        self.root_value.setText(str(snapshot.exchange_root))
        self.state_value.setText(snapshot.state)
        self.scan_value.setText(self._format_time(snapshot.last_scan_at))
        counts = snapshot.command_counts
        if counts:
            self.counts_value.setText(
                ", ".join(f"{state}: {count}" for state, count in sorted(counts.items()))
            )
        else:
            self.counts_value.setText("пока нет")
        self.error_value.setText(snapshot.last_error or "нет")
        self.inbox_value.setText(str(snapshot.exchange_root / "inbox"))
        self.outbox_value.setText(str(snapshot.exchange_root / "outbox"))

        signature = "|".join(
            f"{item.command_id}:{item.state}:{item.finished_at}" for item in snapshot.recent_commands
        )
        if signature == self._last_history_signature:
            return
        self._last_history_signature = signature
        self.history_table.setRowCount(len(snapshot.recent_commands))
        for row, record in enumerate(snapshot.recent_commands):
            values = (
                self._format_time(record.received_at),
                record.command_type,
                record.state,
                record.command_id,
                record.error_code or "—",
                self._record_summary(record),
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setToolTip(value)
                self.history_table.setItem(row, column, item)
