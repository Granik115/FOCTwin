"""Durable, chat-only Google Drive bridge used by the desktop application.

The bridge deliberately keeps its two writers separated:

* FOCTwin owns ``foctwin_to_chatgpt.jsonl`` and ``foctwin_status.json``;
* ChatGPT owns ``chatgpt_to_foctwin.jsonl`` after FOCTwin creates it.

That removes read/modify/write races between the two sides.  Motor commands are not part of
this protocol version; incoming records with any kind other than ``chat`` are ignored.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
import threading
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Protocol

BRIDGE_SCHEMA = 1
BRIDGE_PROTOCOL = "foctwin-drive-bridge"
DRIVE_FILE_SCOPE = "https://www.googleapis.com/auth/drive.file"
MAX_LOCAL_MESSAGE_CHARS = 4_000
MAX_REMOTE_MESSAGE_CHARS = 12_000
MAX_REMOTE_FILE_BYTES = 2 * 1024 * 1024
MAX_MESSAGES = 1_000
STATUS_UPLOAD_INTERVAL = timedelta(seconds=15)

REMOTE_FILE_NAMES = {
    "manifest": "bridge_manifest.json",
    "outbox": "foctwin_to_chatgpt.jsonl",
    "inbox": "chatgpt_to_foctwin.jsonl",
    "status": "foctwin_status.json",
}


class DriveBridgeError(RuntimeError):
    """Base class for errors that can be shown directly in the bridge window."""


class DriveBridgeConfigurationError(DriveBridgeError):
    """The OAuth client file or local bridge configuration is invalid."""


class DriveBridgeAuthorizationRequired(DriveBridgeError):
    """No reusable Google authorization is available on this computer."""


class DriveBridgeTokenStorageError(DriveBridgeError):
    """Windows Credential Manager/keyring could not store the OAuth token."""


class DriveBridgeHttpError(DriveBridgeError):
    """A bounded Google Drive HTTP error."""

    def __init__(self, status_code: int, message: str) -> None:
        self.status_code = status_code
        super().__init__(message)


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def iso_now() -> str:
    return utc_now().isoformat()


def _parse_timestamp(value: str) -> datetime:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


@dataclass(frozen=True)
class BridgeMessage:
    message_id: str
    bridge_id: str
    session_id: str
    sequence: int
    sender: str
    kind: str
    created_at: str
    text: str
    reply_to: str | None = None

    def to_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "schema": BRIDGE_SCHEMA,
            "message_id": self.message_id,
            "bridge_id": self.bridge_id,
            "session_id": self.session_id,
            "sequence": self.sequence,
            "sender": self.sender,
            "kind": self.kind,
            "created_at": self.created_at,
            "text": self.text,
        }
        if self.reply_to:
            payload["reply_to"] = self.reply_to
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> BridgeMessage:
        if payload.get("schema") != BRIDGE_SCHEMA:
            raise ValueError("unsupported message schema")
        message_id = str(payload.get("message_id", "")).strip()
        bridge_id = str(payload.get("bridge_id", "")).strip()
        session_id = str(payload.get("session_id", "")).strip()
        sender = str(payload.get("sender", "")).strip().lower()
        kind = str(payload.get("kind", "")).strip().lower()
        created_at = str(payload.get("created_at", "")).strip()
        text = payload.get("text")
        if not message_id or len(message_id) > 128:
            raise ValueError("invalid message_id")
        if not bridge_id or len(bridge_id) > 128:
            raise ValueError("invalid bridge_id")
        if not session_id or len(session_id) > 128:
            raise ValueError("invalid session_id")
        if sender not in {"foctwin", "chatgpt"}:
            raise ValueError("invalid sender")
        if not kind or len(kind) > 32:
            raise ValueError("invalid kind")
        if not isinstance(text, str) or not text.strip():
            raise ValueError("empty message text")
        if len(text) > MAX_REMOTE_MESSAGE_CHARS:
            raise ValueError("message text is too long")
        _parse_timestamp(created_at)
        sequence = int(payload.get("sequence", 0))
        if sequence < 1:
            raise ValueError("invalid sequence")
        reply_to_value = payload.get("reply_to")
        reply_to = str(reply_to_value).strip() if reply_to_value else None
        return cls(
            message_id=message_id,
            bridge_id=bridge_id,
            session_id=session_id,
            sequence=sequence,
            sender=sender,
            kind=kind,
            created_at=created_at,
            text=text,
            reply_to=reply_to,
        )


def _message_sort_key(message: BridgeMessage) -> tuple[datetime, str, int, str]:
    return (
        _parse_timestamp(message.created_at),
        message.sender,
        message.sequence,
        message.message_id,
    )


def merge_messages(*groups: list[BridgeMessage]) -> list[BridgeMessage]:
    by_id: dict[str, BridgeMessage] = {}
    for group in groups:
        for message in group:
            by_id.setdefault(message.message_id, message)
    ordered = sorted(by_id.values(), key=_message_sort_key)
    return ordered[-MAX_MESSAGES:]


def encode_message_stream(messages: list[BridgeMessage]) -> str:
    return "".join(
        json.dumps(message.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n"
        for message in merge_messages(messages)
    )


def decode_message_stream(
    text: str,
    *,
    expected_bridge_id: str | None = None,
) -> tuple[list[BridgeMessage], list[str]]:
    if len(text.encode("utf-8")) > MAX_REMOTE_FILE_BYTES:
        raise DriveBridgeError("Служебный файл моста больше допустимых 2 МБ")
    messages: list[BridgeMessage] = []
    warnings: list[str] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            payload = json.loads(line)
            if not isinstance(payload, dict):
                raise TypeError("JSON record is not an object")
            message = BridgeMessage.from_dict(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            warnings.append(f"Строка {line_number} проигнорирована: {exc}")
            continue
        if expected_bridge_id and message.bridge_id != expected_bridge_id:
            warnings.append(f"Строка {line_number} относится к другому bridge_id")
            continue
        messages.append(message)
    return merge_messages(messages), warnings


@dataclass(frozen=True)
class BridgeWorkspace:
    folder_id: str
    folder_url: str
    file_ids: dict[str, str]

    def to_dict(self) -> dict[str, object]:
        return {
            "folder_id": self.folder_id,
            "folder_url": self.folder_url,
            "file_ids": dict(self.file_ids),
        }


@dataclass(frozen=True)
class BridgeSnapshot:
    bridge_id: str
    session_id: str
    credentials_path: str
    folder_url: str
    pending_count: int
    messages: list[BridgeMessage]
    last_sync_at: str
    last_error: str
    account: str


@dataclass(frozen=True)
class BridgeSyncResult:
    snapshot: BridgeSnapshot
    warnings: list[str]
    new_incoming_count: int


class TokenStore(Protocol):
    def load(self, key: str) -> str | None: ...

    def save(self, key: str, value: str) -> None: ...

    def delete(self, key: str) -> None: ...


class KeyringTokenStore:
    """Store refresh tokens in Windows Credential Manager through ``keyring``."""

    SERVICE = "FOCTwin.DriveBridge"

    @staticmethod
    def _module():
        try:
            import keyring
            if os.name == "nt":
                from keyring.backends.Windows import WinVaultKeyring

                keyring.set_keyring(WinVaultKeyring())
        except ImportError as exc:  # pragma: no cover - packaging dependency
            raise DriveBridgeTokenStorageError(
                "Модуль безопасного хранения keyring не установлен"
            ) from exc
        return keyring

    def load(self, key: str) -> str | None:
        try:
            return self._module().get_password(self.SERVICE, key)
        except Exception as exc:
            raise DriveBridgeTokenStorageError(
                f"Не удалось прочитать авторизацию из Windows Credential Manager: {exc}"
            ) from exc

    def save(self, key: str, value: str) -> None:
        try:
            self._module().set_password(self.SERVICE, key, value)
        except Exception as exc:
            raise DriveBridgeTokenStorageError(
                f"Не удалось сохранить авторизацию в Windows Credential Manager: {exc}"
            ) from exc

    def delete(self, key: str) -> None:
        keyring = self._module()
        try:
            keyring.delete_password(self.SERVICE, key)
        except keyring.errors.PasswordDeleteError:
            return
        except Exception as exc:
            raise DriveBridgeTokenStorageError(
                f"Не удалось удалить авторизацию из Windows Credential Manager: {exc}"
            ) from exc


def _load_oauth_client(path: Path) -> tuple[dict[str, object], str]:
    try:
        raw = path.read_text(encoding="utf-8")
        if len(raw.encode("utf-8")) > 512 * 1024:
            raise ValueError("file is unexpectedly large")
        payload = json.loads(raw)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise DriveBridgeConfigurationError(f"Не удалось прочитать OAuth JSON: {exc}") from exc
    installed = payload.get("installed") if isinstance(payload, dict) else None
    if not isinstance(installed, dict):
        raise DriveBridgeConfigurationError(
            "Нужен OAuth client типа Desktop app: в JSON отсутствует секция installed"
        )
    client_id = str(installed.get("client_id", "")).strip()
    client_secret = str(installed.get("client_secret", "")).strip()
    if not client_id or not client_secret:
        raise DriveBridgeConfigurationError("В OAuth JSON нет client_id или client_secret")
    fingerprint = hashlib.sha256(client_id.encode("utf-8")).hexdigest()[:24]
    return payload, f"oauth-{fingerprint}"


class DriveTransport(Protocol):
    def authorize(self) -> None: ...

    def connect(self) -> None: ...

    def has_saved_token(self) -> bool: ...

    def forget_token(self) -> None: ...

    def account_label(self) -> str: ...

    def ensure_workspace(
        self,
        bridge_id: str,
        existing: dict[str, object],
    ) -> BridgeWorkspace: ...

    def read_text(self, file_id: str) -> str: ...

    def read_text_if_changed(
        self,
        file_id: str,
        etag: str,
    ) -> tuple[str | None, str]: ...

    def write_text(self, file_id: str, content: str) -> None: ...


class GoogleDriveTransport:
    """Small direct REST client so the portable build does not need Drive Desktop."""

    API_ROOT = "https://www.googleapis.com/drive/v3"
    UPLOAD_ROOT = "https://www.googleapis.com/upload/drive/v3"
    FOLDER_MIME = "application/vnd.google-apps.folder"
    TEXT_MIME = "application/json"

    def __init__(self, credentials_path: Path, token_store: TokenStore | None = None) -> None:
        self.credentials_path = credentials_path
        self.client_config, self.token_key = _load_oauth_client(credentials_path)
        self.token_store = token_store or KeyringTokenStore()
        self.credentials: Any = None
        self.session: Any = None
        self._last_saved_token = ""

    def has_saved_token(self) -> bool:
        return bool(self.token_store.load(self.token_key))

    def authorize(self) -> None:
        try:
            from google_auth_oauthlib.flow import InstalledAppFlow
        except ImportError as exc:  # pragma: no cover - packaging dependency
            raise DriveBridgeConfigurationError(
                "Компоненты Google OAuth не установлены в этой сборке"
            ) from exc
        flow = InstalledAppFlow.from_client_config(self.client_config, [DRIVE_FILE_SCOPE])
        credentials = flow.run_local_server(
            host="127.0.0.1",
            port=0,
            open_browser=True,
            authorization_prompt_message="Открываю браузер для входа в Google…",
            success_message=(
                "FOCTwin получил разрешение. Эту вкладку можно закрыть и вернуться в программу."
            ),
            access_type="offline",
            prompt="consent",
        )
        token_json = credentials.to_json()
        self.token_store.save(self.token_key, token_json)
        self._last_saved_token = token_json
        self.credentials = credentials
        self._make_session()

    def connect(self) -> None:
        raw = self.token_store.load(self.token_key)
        if not raw:
            raise DriveBridgeAuthorizationRequired(
                "Авторизация Google не найдена. Нажмите «Подключить Google Drive»."
            )
        try:
            from google.auth.transport.requests import Request
            from google.oauth2.credentials import Credentials

            credentials = Credentials.from_authorized_user_info(
                json.loads(raw),
                scopes=[DRIVE_FILE_SCOPE],
            )
            if credentials.expired and credentials.refresh_token:
                credentials.refresh(Request())
            if not credentials.valid:
                raise DriveBridgeAuthorizationRequired(
                    "Авторизация Google истекла. Подключите Google Drive заново."
                )
        except DriveBridgeError:
            raise
        except Exception as exc:
            raise DriveBridgeAuthorizationRequired(
                f"Сохранённую авторизацию Google не удалось восстановить: {exc}"
            ) from exc
        self.credentials = credentials
        self._last_saved_token = raw
        self._save_refreshed_token()
        self._make_session()

    def _make_session(self) -> None:
        try:
            from google.auth.transport.requests import AuthorizedSession
        except ImportError as exc:  # pragma: no cover - packaging dependency
            raise DriveBridgeConfigurationError(
                "Компоненты Google Auth не установлены в этой сборке"
            ) from exc
        self.session = AuthorizedSession(self.credentials)

    def _save_refreshed_token(self) -> None:
        if self.credentials is None:
            return
        current = self.credentials.to_json()
        if current != self._last_saved_token:
            self.token_store.save(self.token_key, current)
            self._last_saved_token = current

    def forget_token(self) -> None:
        self.token_store.delete(self.token_key)
        self.credentials = None
        self.session = None

    def _request(
        self,
        method: str,
        url: str,
        *,
        allowed_statuses: set[int] | None = None,
        **kwargs: object,
    ):
        if self.session is None:
            raise DriveBridgeAuthorizationRequired("Google Drive ещё не подключён")
        try:
            response = self.session.request(method, url, timeout=(10, 30), **kwargs)
        except Exception as exc:
            raise DriveBridgeError(f"Нет связи с Google Drive: {exc}") from exc
        self._save_refreshed_token()
        if response.status_code >= 400 and response.status_code not in (allowed_statuses or set()):
            detail = response.text.replace("\n", " ").strip()[:500]
            raise DriveBridgeHttpError(
                response.status_code,
                f"Google Drive вернул HTTP {response.status_code}: {detail}",
            )
        return response

    def account_label(self) -> str:
        response = self._request(
            "GET",
            f"{self.API_ROOT}/about",
            params={"fields": "user(displayName,emailAddress)"},
        )
        user = response.json().get("user", {})
        email = str(user.get("emailAddress", "")).strip()
        name = str(user.get("displayName", "")).strip()
        return email or name or "Google Drive"

    def _metadata(self, file_id: str) -> dict[str, object]:
        response = self._request(
            "GET",
            f"{self.API_ROOT}/files/{file_id}",
            params={
                "fields": "id,name,mimeType,webViewLink,appProperties,parents,size,trashed"
            },
        )
        return response.json()

    def _search(self, query: str) -> list[dict[str, object]]:
        response = self._request(
            "GET",
            f"{self.API_ROOT}/files",
            params={
                "q": query,
                "spaces": "drive",
                "pageSize": "100",
                "fields": (
                    "files(id,name,mimeType,webViewLink,appProperties,parents,size,trashed)"
                ),
                "orderBy": "createdTime asc",
            },
        )
        files = response.json().get("files", [])
        return [item for item in files if isinstance(item, dict)]

    def _create_metadata(self, metadata: dict[str, object]) -> dict[str, object]:
        response = self._request(
            "POST",
            f"{self.API_ROOT}/files",
            params={"fields": "id,name,mimeType,webViewLink,appProperties,parents"},
            json=metadata,
        )
        return response.json()

    def _create_text_file(
        self,
        *,
        name: str,
        folder_id: str,
        bridge_id: str,
        role: str,
        initial_content: str,
    ) -> dict[str, object]:
        metadata = self._create_metadata(
            {
                "name": name,
                "mimeType": self.TEXT_MIME,
                "parents": [folder_id],
                "appProperties": {
                    "foctwin_bridge_id": bridge_id,
                    "foctwin_bridge_role": role,
                },
                "description": (
                    "FOCTwin Drive Bridge service file. Motor commands are disabled in schema 1."
                ),
            }
        )
        self.write_text(str(metadata["id"]), initial_content)
        return metadata

    @staticmethod
    def _app_property_query(key: str, value: str) -> str:
        safe_key = key.replace("'", "\\'")
        safe_value = value.replace("'", "\\'")
        return f"appProperties has {{ key='{safe_key}' and value='{safe_value}' }}"

    def _valid_existing(
        self,
        file_id: str,
        *,
        bridge_id: str,
        role: str,
        mime_type: str | None = None,
    ) -> dict[str, object] | None:
        if not file_id:
            return None
        try:
            metadata = self._metadata(file_id)
        except DriveBridgeHttpError as exc:
            if exc.status_code in {403, 404}:
                return None
            raise
        properties = metadata.get("appProperties", {})
        if not isinstance(properties, dict):
            return None
        if properties.get("foctwin_bridge_id") != bridge_id:
            return None
        if properties.get("foctwin_bridge_role") != role:
            return None
        if mime_type and metadata.get("mimeType") != mime_type:
            return None
        if metadata.get("trashed"):
            return None
        return metadata

    def ensure_workspace(
        self,
        bridge_id: str,
        existing: dict[str, object],
    ) -> BridgeWorkspace:
        existing_file_ids = existing.get("file_ids", {})
        if not isinstance(existing_file_ids, dict):
            existing_file_ids = {}
        existing_folder_id = str(existing.get("folder_id", "")).strip()
        if existing_folder_id and all(
            str(existing_file_ids.get(role, "")).strip() for role in REMOTE_FILE_NAMES
        ):
            existing_url = str(existing.get("folder_url", "")).strip()
            return BridgeWorkspace(
                existing_folder_id,
                existing_url
                or f"https://drive.google.com/drive/folders/{existing_folder_id}",
                {
                    role: str(existing_file_ids[role]).strip()
                    for role in REMOTE_FILE_NAMES
                },
            )

        folder_id = str(existing.get("folder_id", "")).strip()
        folder = self._valid_existing(
            folder_id,
            bridge_id=bridge_id,
            role="root",
            mime_type=self.FOLDER_MIME,
        )
        if folder is None:
            query = " and ".join(
                (
                    self._app_property_query("foctwin_bridge_id", bridge_id),
                    self._app_property_query("foctwin_bridge_role", "root"),
                    f"mimeType='{self.FOLDER_MIME}'",
                    "trashed=false",
                )
            )
            candidates = self._search(query)
            folder = candidates[0] if candidates else self._create_metadata(
                {
                    "name": f"FOCTwin_Bridge_{bridge_id[:8]}",
                    "mimeType": self.FOLDER_MIME,
                    "appProperties": {
                        "foctwin_bridge_id": bridge_id,
                        "foctwin_bridge_role": "root",
                    },
                    "description": (
                        "Chat-only FOCTwin ↔ ChatGPT test bridge. Motor commands are disabled."
                    ),
                }
            )
            folder_id = str(folder["id"])
        else:
            folder_id = str(folder["id"])

        previous_ids = existing.get("file_ids", {})
        if not isinstance(previous_ids, dict):
            previous_ids = {}
        file_ids: dict[str, str] = {}
        for role, name in REMOTE_FILE_NAMES.items():
            previous_id = str(previous_ids.get(role, "")).strip()
            metadata = self._valid_existing(
                previous_id,
                bridge_id=bridge_id,
                role=role,
                mime_type=self.TEXT_MIME,
            )
            if metadata is None:
                query = " and ".join(
                    (
                        f"'{folder_id}' in parents",
                        self._app_property_query("foctwin_bridge_id", bridge_id),
                        self._app_property_query("foctwin_bridge_role", role),
                        "trashed=false",
                    )
                )
                candidates = self._search(query)
                if candidates:
                    metadata = candidates[0]
                else:
                    initial = "" if role in {"outbox", "inbox"} else "{}\n"
                    metadata = self._create_text_file(
                        name=name,
                        folder_id=folder_id,
                        bridge_id=bridge_id,
                        role=role,
                        initial_content=initial,
                    )
            file_ids[role] = str(metadata["id"])

        folder_url = str(folder.get("webViewLink", "")).strip()
        if not folder_url:
            folder_url = f"https://drive.google.com/drive/folders/{folder_id}"
        return BridgeWorkspace(folder_id, folder_url, file_ids)

    def read_text(self, file_id: str) -> str:
        content, _etag = self.read_text_if_changed(file_id, "")
        return content or ""

    def read_text_if_changed(
        self,
        file_id: str,
        etag: str,
    ) -> tuple[str | None, str]:
        headers = {"If-None-Match": etag} if etag else {}
        response = self._request(
            "GET",
            f"{self.API_ROOT}/files/{file_id}",
            params={"alt": "media"},
            headers=headers,
            allowed_statuses={304},
        )
        if response.status_code == 304:
            return None, etag
        if len(response.content) > MAX_REMOTE_FILE_BYTES:
            raise DriveBridgeError("Служебный файл моста больше допустимых 2 МБ")
        try:
            content = response.content.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise DriveBridgeError("Служебный файл моста не является UTF-8 текстом") from exc
        return content, str(response.headers.get("ETag", "")).strip()

    def write_text(self, file_id: str, content: str) -> None:
        encoded = content.encode("utf-8")
        if len(encoded) > MAX_REMOTE_FILE_BYTES:
            raise DriveBridgeError("Служебный файл моста больше допустимых 2 МБ")
        self._request(
            "PATCH",
            f"{self.UPLOAD_ROOT}/files/{file_id}",
            params={"uploadType": "media"},
            headers={"Content-Type": f"{self.TEXT_MIME}; charset=utf-8"},
            data=encoded,
        )


class BridgeStateStore:
    """Atomic local state: queued messages survive process or power interruption."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.path = root / "state.json"

    @staticmethod
    def _default_state() -> dict[str, object]:
        return {
            "schema": BRIDGE_SCHEMA,
            "bridge_id": str(uuid.uuid4()),
            "session_id": str(uuid.uuid4()),
            "created_at": iso_now(),
            "next_sequence": 1,
            "credentials_path": "",
            "workspace": {},
            "pending_outbound": [],
            "cached_messages": [],
            "last_sync_at": "",
            "last_status_upload_at": "",
            "manifest_hash": "",
            "inbox_etag": "",
            "last_error": "",
            "account": "",
        }

    def load(self) -> dict[str, object]:
        self.root.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            state = self._default_state()
            self.save(state)
            return state
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict) or payload.get("schema") != BRIDGE_SCHEMA:
                raise ValueError("unsupported local state schema")
            if not str(payload.get("bridge_id", "")).strip():
                raise ValueError("bridge_id is missing")
            if not str(payload.get("session_id", "")).strip():
                raise ValueError("session_id is missing")
            payload.setdefault("workspace", {})
            payload.setdefault("pending_outbound", [])
            payload.setdefault("cached_messages", [])
            payload.setdefault("last_sync_at", "")
            payload.setdefault("last_status_upload_at", "")
            payload.setdefault("manifest_hash", "")
            payload.setdefault("inbox_etag", "")
            payload.setdefault("last_error", "")
            payload.setdefault("account", "")
            return payload
        except (OSError, ValueError, json.JSONDecodeError):
            backup = self.root / f"state.corrupt-{utc_now().strftime('%Y%m%d-%H%M%S')}.json"
            try:
                shutil.copy2(self.path, backup)
            except OSError:
                pass
            state = self._default_state()
            self.save(state)
            return state

    def save(self, state: dict[str, object]) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        handle, temporary_name = tempfile.mkstemp(
            prefix="state-",
            suffix=".tmp",
            dir=self.root,
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
                json.dump(state, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary.exists():
                temporary.unlink(missing_ok=True)


class DriveBridgeEngine:
    """Thread-safe durable bridge state machine independent from Qt and serial hardware."""

    def __init__(
        self,
        state_store: BridgeStateStore,
        *,
        app_version: str,
        transport_factory: Callable[[Path], DriveTransport] | None = None,
    ) -> None:
        self.store = state_store
        self.app_version = app_version
        self.transport_factory = transport_factory or (
            lambda path: GoogleDriveTransport(path)
        )
        self._lock = threading.RLock()
        self._state = self.store.load()

    def _messages_from_state(self, key: str) -> list[BridgeMessage]:
        messages: list[BridgeMessage] = []
        raw_messages = self._state.get(key, [])
        if not isinstance(raw_messages, list):
            return messages
        for payload in raw_messages:
            if not isinstance(payload, dict):
                continue
            try:
                messages.append(BridgeMessage.from_dict(payload))
            except (TypeError, ValueError):
                continue
        return merge_messages(messages)

    def _snapshot_locked(self) -> BridgeSnapshot:
        workspace = self._state.get("workspace", {})
        if not isinstance(workspace, dict):
            workspace = {}
        return BridgeSnapshot(
            bridge_id=str(self._state["bridge_id"]),
            session_id=str(self._state["session_id"]),
            credentials_path=str(self._state.get("credentials_path", "")),
            folder_url=str(workspace.get("folder_url", "")),
            pending_count=len(self._messages_from_state("pending_outbound")),
            messages=self._messages_from_state("cached_messages"),
            last_sync_at=str(self._state.get("last_sync_at", "")),
            last_error=str(self._state.get("last_error", "")),
            account=str(self._state.get("account", "")),
        )

    def snapshot(self) -> BridgeSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def configure_credentials(self, path: Path) -> None:
        resolved = path.expanduser().resolve()
        _load_oauth_client(resolved)
        with self._lock:
            self._state["credentials_path"] = str(resolved)
            self._state["last_error"] = ""
            self.store.save(self._state)

    def _transport_locked(self) -> DriveTransport:
        raw_path = str(self._state.get("credentials_path", "")).strip()
        if not raw_path:
            raise DriveBridgeConfigurationError(
                "Сначала выберите OAuth JSON для Desktop app"
            )
        path = Path(raw_path)
        if not path.is_file():
            raise DriveBridgeConfigurationError(
                f"Файл OAuth JSON не найден: {path}"
            )
        return self.transport_factory(path)

    def has_saved_authorization(self) -> bool:
        with self._lock:
            try:
                return self._transport_locked().has_saved_token()
            except DriveBridgeError:
                return False

    def forget_authorization(self) -> None:
        with self._lock:
            transport = self._transport_locked()
            transport.forget_token()
            self._state["account"] = ""
            self._state["last_error"] = ""
            self.store.save(self._state)

    def queue_user_message(self, text: str) -> BridgeMessage:
        cleaned = text.strip()
        if not cleaned:
            raise DriveBridgeConfigurationError("Нельзя отправить пустое сообщение")
        if len(cleaned) > MAX_LOCAL_MESSAGE_CHARS:
            raise DriveBridgeConfigurationError(
                f"Сообщение длиннее {MAX_LOCAL_MESSAGE_CHARS} символов"
            )
        with self._lock:
            sequence = int(self._state.get("next_sequence", 1))
            message = BridgeMessage(
                message_id=str(uuid.uuid4()),
                bridge_id=str(self._state["bridge_id"]),
                session_id=str(self._state["session_id"]),
                sequence=sequence,
                sender="foctwin",
                kind="chat",
                created_at=iso_now(),
                text=cleaned,
            )
            pending = merge_messages(self._messages_from_state("pending_outbound"), [message])
            cached = merge_messages(self._messages_from_state("cached_messages"), [message])
            self._state["pending_outbound"] = [item.to_dict() for item in pending]
            self._state["cached_messages"] = [item.to_dict() for item in cached]
            self._state["next_sequence"] = sequence + 1
            self.store.save(self._state)
            return message

    def authorize(self) -> BridgeSyncResult:
        with self._lock:
            transport = self._transport_locked()
            try:
                transport.authorize()
                return self._sync_with_recovery_locked(transport)
            except Exception as exc:
                self._record_error_locked(exc)
                raise

    def sync(self) -> BridgeSyncResult:
        with self._lock:
            transport = self._transport_locked()
            try:
                transport.connect()
                return self._sync_with_recovery_locked(transport)
            except Exception as exc:
                self._record_error_locked(exc)
                raise

    def _record_error_locked(self, exc: Exception) -> None:
        self._state["last_error"] = str(exc)
        self.store.save(self._state)

    def _workspace_existing_locked(self) -> dict[str, object]:
        workspace = self._state.get("workspace", {})
        return dict(workspace) if isinstance(workspace, dict) else {}

    def _manifest_text_locked(self, workspace: BridgeWorkspace) -> str:
        payload = {
            "schema": BRIDGE_SCHEMA,
            "protocol": BRIDGE_PROTOCOL,
            "bridge_id": self._state["bridge_id"],
            "session_id": self._state["session_id"],
            "created_at": self._state.get("created_at", ""),
            "files": {
                role: {
                    "name": REMOTE_FILE_NAMES[role],
                    "file_id": workspace.file_ids[role],
                    "writer": (
                        "chatgpt" if role == "inbox" else "foctwin"
                    ),
                }
                for role in REMOTE_FILE_NAMES
                if role != "manifest"
            },
            "capabilities": {
                "chat": True,
                "motor_commands": False,
                "accepted_inbound_kinds": ["chat"],
            },
            "instructions": (
                "Append ChatGPT replies only to chatgpt_to_foctwin.jsonl. "
                "Do not write to the FOCTwin-owned outbox or status files."
            ),
        }
        return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"

    def _status_text_locked(self, account: str, pending_count: int) -> str:
        payload = {
            "schema": BRIDGE_SCHEMA,
            "protocol": BRIDGE_PROTOCOL,
            "bridge_id": self._state["bridge_id"],
            "session_id": self._state["session_id"],
            "app_version": self.app_version,
            "updated_at": iso_now(),
            "state": "listening",
            "account": account,
            "pending_outbound": pending_count,
            "motor_connected": False,
            "motor_commands_enabled": False,
        }
        return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"

    def _status_due_locked(self, now: datetime) -> bool:
        raw = str(self._state.get("last_status_upload_at", ""))
        if not raw:
            return True
        try:
            return now - _parse_timestamp(raw) >= STATUS_UPLOAD_INTERVAL
        except ValueError:
            return True

    def _sync_with_recovery_locked(self, transport: DriveTransport) -> BridgeSyncResult:
        try:
            return self._sync_with_transport_locked(transport)
        except DriveBridgeHttpError as exc:
            if exc.status_code not in {403, 404} or not self._workspace_existing_locked():
                raise
            self._state["workspace"] = {}
            self._state["manifest_hash"] = ""
            self._state["inbox_etag"] = ""
            self.store.save(self._state)
            return self._sync_with_transport_locked(transport)

    def _sync_with_transport_locked(self, transport: DriveTransport) -> BridgeSyncResult:
        bridge_id = str(self._state["bridge_id"])
        existing_workspace = self._workspace_existing_locked()
        workspace = transport.ensure_workspace(
            bridge_id,
            existing_workspace,
        )
        workspace_changed = workspace.to_dict() != existing_workspace
        self._state["workspace"] = workspace.to_dict()
        if workspace_changed:
            self._state["manifest_hash"] = ""
            self._state["inbox_etag"] = ""
        self.store.save(self._state)

        warnings: list[str] = []
        manifest_text = self._manifest_text_locked(workspace)
        manifest_hash = hashlib.sha256(manifest_text.encode("utf-8")).hexdigest()
        if self._state.get("manifest_hash") != manifest_hash:
            transport.write_text(workspace.file_ids["manifest"], manifest_text)
            self._state["manifest_hash"] = manifest_hash

        cached_outbox = [
            item
            for item in self._messages_from_state("cached_messages")
            if item.sender == "foctwin" and item.kind == "chat"
        ]
        pending = self._messages_from_state("pending_outbound")
        merged_outbox = merge_messages(cached_outbox, pending)
        if pending or (workspace_changed and merged_outbox):
            transport.write_text(
                workspace.file_ids["outbox"],
                encode_message_stream(merged_outbox),
            )
        uploaded_ids = {item.message_id for item in pending}
        remaining_pending = [
            item
            for item in self._messages_from_state("pending_outbound")
            if item.message_id not in uploaded_ids
        ]
        self._state["pending_outbound"] = [item.to_dict() for item in remaining_pending]

        known_incoming_ids = {
            item.message_id
            for item in self._messages_from_state("cached_messages")
            if item.sender == "chatgpt"
        }
        inbox_text, inbox_etag = transport.read_text_if_changed(
            workspace.file_ids["inbox"],
            str(self._state.get("inbox_etag", "")),
        )
        if inbox_text is None:
            remote_inbox = [
                item
                for item in self._messages_from_state("cached_messages")
                if item.sender == "chatgpt"
            ]
        else:
            remote_inbox, inbox_warnings = decode_message_stream(
                inbox_text,
                expected_bridge_id=bridge_id,
            )
            warnings.extend(inbox_warnings)
            self._state["inbox_etag"] = inbox_etag
        accepted_inbox: list[BridgeMessage] = []
        for message in remote_inbox:
            if message.sender != "chatgpt":
                warnings.append(
                    f"Сообщение {message.message_id} имеет неверного отправителя и проигнорировано"
                )
                continue
            if message.kind != "chat":
                warnings.append(
                    f"Входящий kind={message.kind} проигнорирован: команды мотору отключены"
                )
                continue
            accepted_inbox.append(message)

        cached = merge_messages(
            self._messages_from_state("cached_messages"),
            merged_outbox,
            accepted_inbox,
        )
        self._state["cached_messages"] = [item.to_dict() for item in cached]
        account = str(self._state.get("account", ""))
        if not account or workspace_changed:
            account = transport.account_label()
        now = utc_now()
        if self._status_due_locked(now):
            transport.write_text(
                workspace.file_ids["status"],
                self._status_text_locked(account, len(remaining_pending)),
            )
            self._state["last_status_upload_at"] = now.isoformat()
        self._state["last_sync_at"] = now.isoformat()
        self._state["last_error"] = ""
        self._state["account"] = account
        self.store.save(self._state)

        new_incoming_count = sum(
            item.message_id not in known_incoming_ids for item in accepted_inbox
        )
        return BridgeSyncResult(
            snapshot=self._snapshot_locked(),
            warnings=warnings,
            new_incoming_count=new_incoming_count,
        )
