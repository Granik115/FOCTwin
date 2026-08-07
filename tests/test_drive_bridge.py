import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from foctwin.drive_bridge import (
    BridgeMessage,
    BridgeStateStore,
    BridgeWorkspace,
    DriveBridgeConfigurationError,
    DriveBridgeEngine,
    DriveBridgeError,
    decode_message_stream,
    encode_message_stream,
    iso_now,
)


class FakeDriveTransport:
    def __init__(self, *, fail_connect: bool = False) -> None:
        self.fail_connect = fail_connect
        self.authorized = True
        self.files = {
            "id-manifest": "{}\n",
            "id-outbox": "",
            "id-inbox": "",
            "id-status": "{}\n",
        }
        self.write_counts = {file_id: 0 for file_id in self.files}

    def authorize(self) -> None:
        self.authorized = True

    def connect(self) -> None:
        if self.fail_connect:
            raise DriveBridgeError("network unavailable")
        if not self.authorized:
            raise DriveBridgeError("not authorized")

    def has_saved_token(self) -> bool:
        return self.authorized

    def forget_token(self) -> None:
        self.authorized = False

    def account_label(self) -> str:
        return "test@example.com"

    def ensure_workspace(
        self,
        bridge_id: str,
        existing: dict[str, object],
    ) -> BridgeWorkspace:
        return BridgeWorkspace(
            folder_id="folder-1",
            folder_url="https://drive.google.com/drive/folders/folder-1",
            file_ids={
                "manifest": "id-manifest",
                "outbox": "id-outbox",
                "inbox": "id-inbox",
                "status": "id-status",
            },
        )

    def read_text(self, file_id: str) -> str:
        return self.files[file_id]

    def read_text_if_changed(
        self,
        file_id: str,
        etag: str,
    ) -> tuple[str | None, str]:
        content = self.files[file_id]
        current = hashlib.sha256(content.encode("utf-8")).hexdigest()
        if current == etag:
            return None, current
        return content, current

    def write_text(self, file_id: str, content: str) -> None:
        self.files[file_id] = content
        self.write_counts[file_id] += 1


def write_oauth_json(root: Path, *, desktop: bool = True) -> Path:
    path = root / "client_secret.json"
    key = "installed" if desktop else "web"
    path.write_text(
        json.dumps(
            {
                key: {
                    "client_id": "test.apps.googleusercontent.com",
                    "client_secret": "not-a-real-secret",
                    "auth_uri": "https://accounts.google.com/o/oauth2/auth",
                    "token_uri": "https://oauth2.googleapis.com/token",
                }
            }
        ),
        encoding="utf-8",
    )
    return path


class DriveBridgeTests(unittest.TestCase):
    def test_outbound_message_is_uploaded_once_and_deduplicated(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            transport = FakeDriveTransport()
            engine = DriveBridgeEngine(
                BridgeStateStore(root / "state"),
                app_version="test",
                transport_factory=lambda _path: transport,
            )
            engine.configure_credentials(write_oauth_json(root))

            queued = engine.queue_user_message("Ты меня видишь?")
            self.assertEqual(engine.snapshot().pending_count, 1)
            first = engine.sync()

            self.assertEqual(first.snapshot.pending_count, 0)
            messages, warnings = decode_message_stream(transport.files["id-outbox"])
            self.assertEqual(warnings, [])
            self.assertEqual([message.message_id for message in messages], [queued.message_id])
            self.assertEqual(messages[0].text, "Ты меня видишь?")

            engine.sync()
            messages, _ = decode_message_stream(transport.files["id-outbox"])
            self.assertEqual(len(messages), 1)
            self.assertEqual(transport.write_counts["id-outbox"], 1)

    def test_local_queue_survives_failed_sync_and_engine_restart(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            state_store = BridgeStateStore(root / "state")
            failing = FakeDriveTransport(fail_connect=True)
            first = DriveBridgeEngine(
                state_store,
                app_version="test",
                transport_factory=lambda _path: failing,
            )
            first.configure_credentials(write_oauth_json(root))
            queued = first.queue_user_message("Сообщение перед отключением питания")
            with self.assertRaisesRegex(DriveBridgeError, "network unavailable"):
                first.sync()
            self.assertEqual(first.snapshot().pending_count, 1)

            recovered_transport = FakeDriveTransport()
            recovered = DriveBridgeEngine(
                state_store,
                app_version="test",
                transport_factory=lambda _path: recovered_transport,
            )
            result = recovered.sync()

            self.assertEqual(result.snapshot.pending_count, 0)
            messages, _ = decode_message_stream(recovered_transport.files["id-outbox"])
            self.assertEqual([message.message_id for message in messages], [queued.message_id])

    def test_incoming_chat_is_shown_but_command_kind_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            transport = FakeDriveTransport()
            engine = DriveBridgeEngine(
                BridgeStateStore(root / "state"),
                app_version="test",
                transport_factory=lambda _path: transport,
            )
            engine.configure_credentials(write_oauth_json(root))
            engine.sync()
            snapshot = engine.snapshot()
            incoming = [
                BridgeMessage(
                    message_id="reply-1",
                    bridge_id=snapshot.bridge_id,
                    session_id=snapshot.session_id,
                    sequence=1,
                    sender="chatgpt",
                    kind="chat",
                    created_at=iso_now(),
                    text="Да, вижу.",
                ),
                BridgeMessage(
                    message_id="attempted-command",
                    bridge_id=snapshot.bridge_id,
                    session_id=snapshot.session_id,
                    sequence=2,
                    sender="chatgpt",
                    kind="command",
                    created_at=iso_now(),
                    text="RUN MOTOR",
                ),
            ]
            transport.files["id-inbox"] = encode_message_stream(incoming)

            result = engine.sync()

            self.assertEqual(result.new_incoming_count, 1)
            self.assertTrue(any("команды мотору отключены" in item for item in result.warnings))
            received = [
                message
                for message in result.snapshot.messages
                if message.sender == "chatgpt"
            ]
            self.assertEqual([message.text for message in received], ["Да, вижу."])

    def test_manifest_explicitly_disables_motor_commands(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            transport = FakeDriveTransport()
            engine = DriveBridgeEngine(
                BridgeStateStore(root / "state"),
                app_version="0.4.1b1",
                transport_factory=lambda _path: transport,
            )
            engine.configure_credentials(write_oauth_json(root))
            engine.sync()

            manifest = json.loads(transport.files["id-manifest"])
            self.assertEqual(manifest["protocol"], "foctwin-drive-bridge")
            self.assertFalse(manifest["capabilities"]["motor_commands"])
            self.assertEqual(manifest["capabilities"]["accepted_inbound_kinds"], ["chat"])

    def test_non_desktop_oauth_json_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            engine = DriveBridgeEngine(
                BridgeStateStore(root / "state"),
                app_version="test",
                transport_factory=lambda _path: FakeDriveTransport(),
            )
            with self.assertRaisesRegex(DriveBridgeConfigurationError, "Desktop app"):
                engine.configure_credentials(write_oauth_json(root, desktop=False))


if __name__ == "__main__":
    unittest.main()
