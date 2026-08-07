"""Separate non-modal diagnostics/chat window for the Google Drive bridge."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QObject, QStandardPaths, Qt, QTimer, QUrl, Signal
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from foctwin import __version__
from foctwin.drive_bridge import (
    BridgeMessage,
    BridgeSnapshot,
    BridgeStateStore,
    BridgeSyncResult,
    DriveBridgeAuthorizationRequired,
    DriveBridgeConfigurationError,
    DriveBridgeEngine,
    DriveBridgeError,
    DriveBridgeHttpError,
    DriveBridgeTokenStorageError,
)


class DriveBridgeUiSignals(QObject):
    success = Signal(str, object)
    failure = Signal(str, object)


class DriveBridgeDialog(QDialog):
    POLL_INTERVAL_MS = 3_000

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        state_root: Path | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(f"FOCTwin {__version__} — связь с GPT через Google Drive")
        self.setModal(False)
        self.resize(900, 760)
        self.setMinimumSize(720, 600)

        data_root = state_root or (
            Path(
                QStandardPaths.writableLocation(
                    QStandardPaths.StandardLocation.AppLocalDataLocation
                )
            )
            / "drive_bridge"
        )
        self.engine = DriveBridgeEngine(
            BridgeStateStore(data_root),
            app_version=__version__,
        )
        self.signals = DriveBridgeUiSignals(self)
        self.signals.success.connect(self._on_job_success)
        self.signals.failure.connect(self._on_job_failure)
        self._busy_kind = ""
        self._ready = False
        self._last_transcript_signature = ""
        self._auto_connect_attempted = False

        self._build_ui()
        self._render_snapshot(self.engine.snapshot())

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(self.POLL_INTERVAL_MS)
        self.poll_timer.timeout.connect(self._poll)
        QTimer.singleShot(250, self._attempt_saved_connection)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        self.safety_label = QLabel(
            "ДОМАШНИЙ РЕЖИМ: канал принимает только текст CHAT. Команды мотору и COM-порту "
            "в этой версии программно отключены."
        )
        self.safety_label.setObjectName("danger")
        self.safety_label.setWordWrap(True)
        root.addWidget(self.safety_label)

        status_group = QGroupBox("Состояние канала")
        status_form = QFormLayout(status_group)
        self.connection_value = QLabel("Google Drive не подключён")
        self.connection_value.setWordWrap(True)
        self.bridge_id_value = QLabel("—")
        self.bridge_id_value.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.folder_value = QLabel("Папка ещё не создана")
        self.folder_value.setWordWrap(True)
        self.exchange_value = QLabel("Обмена ещё не было")
        self.queue_value = QLabel("0")
        status_form.addRow("Google", self.connection_value)
        status_form.addRow("Bridge ID", self.bridge_id_value)
        status_form.addRow("Папка", self.folder_value)
        status_form.addRow("Последний обмен", self.exchange_value)
        status_form.addRow("В локальной очереди", self.queue_value)
        root.addWidget(status_group)

        controls = QHBoxLayout()
        self.authorize_button = QPushButton("Подключить Google Drive…")
        self.authorize_button.clicked.connect(self._choose_credentials)
        self.sync_button = QPushButton("Синхронизировать сейчас")
        self.sync_button.clicked.connect(self._start_sync)
        self.open_folder_button = QPushButton("Открыть папку")
        self.open_folder_button.clicked.connect(self._open_folder)
        self.copy_id_button = QPushButton("Скопировать Bridge ID")
        self.copy_id_button.clicked.connect(self._copy_bridge_id)
        self.forget_button = QPushButton("Забыть вход на этом ПК")
        self.forget_button.clicked.connect(self._forget_authorization)
        for button in (
            self.authorize_button,
            self.sync_button,
            self.open_folder_button,
            self.copy_id_button,
            self.forget_button,
        ):
            controls.addWidget(button)
        root.addLayout(controls)

        setup = QGroupBox("Первое подключение: что за JSON")
        setup.setCheckable(True)
        setup.setChecked(False)
        setup_layout = QVBoxLayout(setup)
        setup_text = QLabel(
            "Один раз нужен OAuth client типа <b>Desktop app</b>: включите Google Drive API "
            "в своём проекте Google Cloud, добавьте свой аккаунт как test user, создайте "
            "OAuth Client ID → Desktop app и скачайте JSON. Затем нажмите кнопку подключения "
            "выше. FOCTwin запросит только узкий scope <code>drive.file</code> и сможет видеть "
            "лишь созданные им служебные файлы. "
            "<a href='https://console.cloud.google.com/apis/credentials'>Открыть Google Cloud "
            "Credentials</a>."
        )
        setup_text.setWordWrap(True)
        setup_text.setOpenExternalLinks(True)
        setup_layout.addWidget(setup_text)
        setup.toggled.connect(lambda checked: setup_text.setVisible(checked))
        setup_text.setVisible(False)
        root.addWidget(setup)

        conversation_group = QGroupBox("Переписка FOCTwin ↔ GPT")
        conversation_layout = QVBoxLayout(conversation_group)
        self.transcript = QPlainTextEdit()
        self.transcript.setReadOnly(True)
        self.transcript.setPlaceholderText("Сообщений пока нет")
        self.transcript.document().setMaximumBlockCount(5_000)
        conversation_layout.addWidget(self.transcript, 1)
        self.message_input = QPlainTextEdit()
        self.message_input.setPlaceholderText("Напиши сообщение мне через Google Drive…")
        self.message_input.setMaximumHeight(100)
        conversation_layout.addWidget(self.message_input)
        send_row = QHBoxLayout()
        self.send_button = QPushButton("Отправить")
        self.send_button.clicked.connect(self._queue_message)
        self.send_state = QLabel(
            "Сообщение сначала сохраняется на компьютере, затем попадает на Drive."
        )
        self.send_state.setWordWrap(True)
        send_row.addWidget(self.send_state, 1)
        send_row.addWidget(self.send_button)
        conversation_layout.addLayout(send_row)
        root.addWidget(conversation_group, 1)

        technical = QGroupBox("Технический журнал")
        technical.setCheckable(True)
        technical.setChecked(False)
        technical_layout = QVBoxLayout(technical)
        self.technical_log = QPlainTextEdit()
        self.technical_log.setReadOnly(True)
        self.technical_log.setMaximumHeight(170)
        self.technical_log.document().setMaximumBlockCount(2_000)
        technical_layout.addWidget(self.technical_log)
        technical.toggled.connect(lambda checked: self.technical_log.setVisible(checked))
        self.technical_log.setVisible(False)
        root.addWidget(technical)

    def _attempt_saved_connection(self) -> None:
        if self._auto_connect_attempted:
            return
        self._auto_connect_attempted = True
        snapshot = self.engine.snapshot()
        if snapshot.credentials_path and self.engine.has_saved_authorization():
            self._append_log("INFO", "Найдена сохранённая авторизация; проверяю канал")
            self._start_sync()

    def _choose_credentials(self) -> None:
        selected, _ = QFileDialog.getOpenFileName(
            self,
            "Выберите OAuth client JSON типа Desktop app",
            "",
            "Google OAuth JSON (*.json);;Все файлы (*)",
        )
        if not selected:
            return
        try:
            self.engine.configure_credentials(Path(selected))
        except DriveBridgeError as exc:
            QMessageBox.critical(self, "OAuth JSON не принят", str(exc))
            self._append_log("ERROR", str(exc))
            return
        self._render_snapshot(self.engine.snapshot())
        self._append_log("INFO", f"Выбран OAuth JSON: {selected}")
        self._start_job("authorize", self.engine.authorize)

    def _start_sync(self) -> None:
        self._start_job("sync", self.engine.sync)

    def _poll(self) -> None:
        if self._ready and not self._busy_kind:
            self._start_sync()

    def _start_job(self, kind: str, operation: Callable[[], object]) -> None:
        if self._busy_kind:
            return
        self._busy_kind = kind
        self._set_busy(True)
        if kind == "authorize":
            self.connection_value.setText("Ожидаю вход и разрешение в браузере…")
            self._append_log("INFO", "Запущена авторизация Google в браузере")
        else:
            self.connection_value.setText("Синхронизация с Google Drive…")

        def run() -> None:
            try:
                result = operation()
            except Exception as exc:  # noqa: BLE001
                self.signals.failure.emit(kind, exc)
            else:
                self.signals.success.emit(kind, result)

        threading.Thread(target=run, name=f"drive-bridge-{kind}", daemon=True).start()

    def _on_job_success(self, kind: str, result: object) -> None:
        self._busy_kind = ""
        self._set_busy(False)
        if isinstance(result, BridgeSyncResult):
            self._ready = True
            self.poll_timer.start()
            self._render_snapshot(result.snapshot)
            if kind == "authorize":
                self._append_log("INFO", "Google Drive подключён; служебная папка готова")
            else:
                self._append_log("INFO", "Синхронизация завершена")
            for warning in result.warnings:
                self._append_log("WARN", warning)
            if result.new_incoming_count:
                QApplication.beep()
                self.send_state.setText(
                    f"Получено новых ответов GPT: {result.new_incoming_count}"
                )
        else:
            self._append_log("INFO", f"Операция {kind} завершена")

    def _on_job_failure(self, kind: str, error: object) -> None:
        self._busy_kind = ""
        self._set_busy(False)
        message = str(error)
        fatal = kind == "authorize" or isinstance(
            error,
            (
                DriveBridgeAuthorizationRequired,
                DriveBridgeConfigurationError,
                DriveBridgeTokenStorageError,
            ),
        )
        if isinstance(error, DriveBridgeHttpError) and error.status_code in {400, 401, 403}:
            fatal = True
        if fatal:
            self._ready = False
            self.poll_timer.stop()
        elif self._ready:
            self.poll_timer.start()
        snapshot = self.engine.snapshot()
        self._render_snapshot(snapshot)
        self.connection_value.setText(f"Ошибка: {message}")
        self._append_log("ERROR", message)
        if kind == "authorize":
            QMessageBox.critical(self, "Google Drive не подключён", message)

    def _set_busy(self, busy: bool) -> None:
        self.authorize_button.setEnabled(not busy)
        self.sync_button.setEnabled(not busy)
        self.forget_button.setEnabled(not busy)
        self.send_button.setEnabled(not busy)

    def _queue_message(self) -> None:
        text = self.message_input.toPlainText()
        try:
            message = self.engine.queue_user_message(text)
        except DriveBridgeError as exc:
            QMessageBox.warning(self, "Сообщение не принято", str(exc))
            return
        self.message_input.clear()
        self._render_snapshot(self.engine.snapshot())
        self.send_state.setText(
            f"Сообщение {message.sequence} надёжно сохранено локально; отправляю на Drive…"
        )
        self._append_log("INFO", f"CHAT {message.message_id} добавлен в локальную очередь")
        if self.engine.snapshot().credentials_path:
            self._start_sync()
        else:
            self.send_state.setText(
                "Сообщение сохранено в очереди. Теперь подключите Google Drive."
            )

    def _forget_authorization(self) -> None:
        answer = QMessageBox.question(
            self,
            "Забыть вход?",
            "Удалить OAuth-токен из Windows Credential Manager? Служебная папка и история "
            "на Google Drive останутся.",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        try:
            self.engine.forget_authorization()
        except DriveBridgeError as exc:
            QMessageBox.critical(self, "Не удалось удалить вход", str(exc))
            return
        self._ready = False
        self.poll_timer.stop()
        self._render_snapshot(self.engine.snapshot())
        self.connection_value.setText("Авторизация на этом ПК удалена")
        self._append_log("INFO", "Сохранённая авторизация удалена")

    def _open_folder(self) -> None:
        url = self.engine.snapshot().folder_url
        if url:
            QDesktopServices.openUrl(QUrl(url))

    def _copy_bridge_id(self) -> None:
        bridge_id = self.engine.snapshot().bridge_id
        QApplication.clipboard().setText(bridge_id)
        self.send_state.setText("Bridge ID скопирован")

    @staticmethod
    def _format_timestamp(raw: str) -> str:
        if not raw:
            return "Обмена ещё не было"
        try:
            parsed = datetime.fromisoformat(raw.replace("Z", "+00:00")).astimezone()
            return parsed.strftime("%d.%m.%Y %H:%M:%S")
        except ValueError:
            return raw

    @classmethod
    def _render_message(cls, message: BridgeMessage) -> str:
        sender = "Ты" if message.sender == "foctwin" else "GPT"
        timestamp = cls._format_timestamp(message.created_at)
        return f"[{timestamp}] {sender}:\n{message.text}"

    def _render_snapshot(self, snapshot: BridgeSnapshot) -> None:
        self.bridge_id_value.setText(snapshot.bridge_id)
        self.folder_value.setText(snapshot.folder_url or "Папка ещё не создана")
        self.exchange_value.setText(self._format_timestamp(snapshot.last_sync_at))
        self.queue_value.setText(str(snapshot.pending_count))
        self.open_folder_button.setEnabled(bool(snapshot.folder_url))
        if self._ready and snapshot.last_sync_at and not self._busy_kind:
            account = f" ({snapshot.account})" if snapshot.account else ""
            self.connection_value.setText(f"Подключён, канал слушает{account}")
        elif snapshot.credentials_path and not self._busy_kind:
            self.connection_value.setText("OAuth JSON выбран; вход ещё не подтверждён")
        elif not self._busy_kind:
            self.connection_value.setText("Google Drive не подключён")

        signature = "\n".join(message.message_id for message in snapshot.messages)
        if signature != self._last_transcript_signature:
            self._last_transcript_signature = signature
            rendered = "\n\n".join(
                self._render_message(message) for message in snapshot.messages
            )
            self.transcript.setPlainText(rendered)
            scroll = self.transcript.verticalScrollBar()
            scroll.setValue(scroll.maximum())

    def _append_log(self, level: str, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.technical_log.appendPlainText(f"{timestamp} [{level}] {message}")
