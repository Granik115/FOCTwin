# FOCTwin Drive Bridge 0.4.1b1

This beta is a home-only communications test. It preserves the complete 0.4.0 guarded current
trial but does not connect Google Drive to that trial. The bridge can be opened with no project,
COM port or motor.

## One-time Google setup

FOCTwin cannot reuse a ChatGPT or browser login token. A user-owned Desktop OAuth client is
therefore required once:

1. Open Google Cloud Console and create or select a project.
2. Enable **Google Drive API** for that project.
3. Configure the OAuth consent screen. While the app is in testing, add the Google account that
   will own the bridge folder as a test user.
4. Open **APIs & Services → Credentials → Create credentials → OAuth client ID**.
5. Select **Desktop app**, create it and download the JSON file.
6. In FOCTwin press **Связь с GPT → Подключить Google Drive…** and select that JSON.
7. Complete the Google authorization in the browser. FOCTwin requests only
   `https://www.googleapis.com/auth/drive.file`.

The refresh token is kept in Windows Credential Manager under `FOCTwin.DriveBridge`. The OAuth
JSON remains at the user-selected path and is never copied into a FOCTwin project or export.

## Files created on Drive

FOCTwin creates `FOCTwin_Bridge_<first 8 characters of Bridge ID>` in My Drive. The full Bridge ID
is shown in the window and can be copied.

| File | Writer | Purpose |
|---|---|---|
| `bridge_manifest.json` | FOCTwin | IDs, protocol version and accepted capabilities |
| `foctwin_to_chatgpt.jsonl` | FOCTwin | User messages waiting for ChatGPT |
| `chatgpt_to_foctwin.jsonl` | ChatGPT | ChatGPT replies read by FOCTwin |
| `foctwin_status.json` | FOCTwin | Version, account, heartbeat and queue state |

Each side has exactly one outbound writer. This avoids a race in which both sides download,
append and replace the same Drive file at nearly the same time.

## Message schema

Every non-empty JSONL row is one object:

```json
{
  "schema": 1,
  "message_id": "a UUID or another unique ID",
  "bridge_id": "the full Bridge ID from the manifest",
  "session_id": "the current session ID from the manifest",
  "sequence": 1,
  "sender": "foctwin",
  "kind": "chat",
  "created_at": "2026-08-07T15:00:00+00:00",
  "text": "Ты меня видишь?"
}
```

ChatGPT replies use `sender: "chatgpt"` and may include `reply_to` with the original message ID.
The streams are merged by `message_id`; repeating a synchronization cannot display or upload the
same message twice.

## Interruption behaviour

Pressing **Отправить** first atomically stores the message in the local bridge state. Only then
does the background thread touch Google Drive. If Internet access, OAuth or the process disappears
at that moment, the next successful synchronization uploads the same UUID. A completed upload is
removed from the local pending queue only after Drive accepts it.

The window polls every three seconds while connected. Transient network errors remain visible in
the technical log and polling continues. A broken/expired authorization stops polling and requires
the user to reconnect. Incoming downloads use an ETag, so an unchanged inbox normally costs one
conditional request rather than downloading its whole history.

## Safety boundary

Schema 1 has one accepted inbound kind: `chat`. A syntactically valid record with `kind: command`,
`run`, `abort` or anything else is logged and ignored. The bridge backend imports neither the
Serial service nor current/friction experiment classes. It cannot enable PWM or send Commander
commands.

This boundary is intentional. A later hardware release will introduce a separate versioned
command schema, local safety validation and explicit start policy only after the chat transport is
proven stable.

## Home test checklist

1. Authorize and confirm that the folder URL opens.
2. Send one message and verify that the local queue returns to zero.
3. Ask ChatGPT in the active conversation to read the Bridge ID and answer through the inbox.
4. Send several messages quickly and verify their order and UUID uniqueness.
5. Disconnect Internet, send a message, restart FOCTwin and restore Internet.
6. Confirm that the queued message appears once on Drive.
7. Replace the inbox with a duplicate chat row and confirm it is displayed once.
8. Add a `kind: "command"` row and confirm that it appears only as a technical warning.
