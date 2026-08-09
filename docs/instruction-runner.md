# FOCTwin Instruction Runner 0.4.2b1

This beta adds the first executable instruction channel to FOCTwin. It is intentionally limited
to local diagnostics: it can prove delivery, validation, deduplication, restart recovery and the
return path without opening a project, COM port or motor.

The existing `Связь с GPT` Drive chat remains available for comparison. The new `Инструкции`
window does not use Google Drive directly; FolderBridge is the transport.

## Folder layout

FOCTwin creates the complete layout under the selected exchange root:

```text
C:\AutotunerExchange\
├── inbox\
│   └── commands\
│       └── cmd_<UUID>.json
└── outbox\
    ├── events\
    │   └── event_<UUID>.json
    ├── artifacts\
    └── status.json
```

Recommended FolderBridge jobs, both initially in `Копирование` mode:

| Direction | Drive path | Local path |
|---|---|---|
| Drive → PC | `Autotuner/to_pc` | `C:\AutotunerExchange\inbox` |
| PC → Drive | `Autotuner/from_pc` | `C:\AutotunerExchange\outbox` |

FolderBridge supports the nested `commands`, `events` and `artifacts` directories. FOCTwin does
not delete incoming files. A private SQLite journal outside the synchronized tree remembers the
command UUID and SHA-256, so `Копирование` cannot execute the same command again.

The first launch defaults to the user's Documents folder. Press `Выбрать папку обмена…` if the
FolderBridge job uses `C:\AutotunerExchange` or another location.

Automatic scanning starts when `Инструкции` is opened for the first time. Closing the window only
hides it; scanning continues while the main FOCTwin process remains open. The checkbox can pause
this behaviour explicitly.

## Command envelope

Each command is one immutable UTF-8 JSON file no larger than 256 KiB. Its filename must match its
UUID exactly:

```json
{
  "schema": 1,
  "protocol": "foctwin-instruction-runner",
  "command_id": "8840180e-2571-4b75-8f8b-7c587c2a72ee",
  "target": "INSTANCE-ID-FROM-FOCTWIN",
  "type": "ping",
  "created_at": "2026-08-09T18:00:00+00:00",
  "expires_at": "2026-08-09T19:00:00+00:00",
  "arguments": {}
}
```

The corresponding name is
`cmd_8840180e-2571-4b75-8f8b-7c587c2a72ee.json`. `target` may be the Instance ID shown in the
window or `*`. A command addressed to another instance is journaled as `ignored`. Timestamps must
include a timezone, the command must not already be expired and its maximum lifetime is seven
days.

Do not append multiple commands to one JSON or JSONL file. Create a new immutable file for every
command. Writing it through a temporary name and atomically renaming it to the final `.json` name
is recommended; FolderBridge already downloads files that way.

## Available commands

| Type | Arguments | Result |
|---|---|---|
| `ping` | `{}` | `pong`, version, Instance ID and handling time |
| `get_status` | `{}` | The safe contents represented by `status.json` |
| `list_capabilities` | `{}` | The exact allowlist and hardware flags |
| `self_test` | `{}` | Checks for SQLite and all exchange paths |
| `dry_run` | `{"command":{"type":"...","arguments":{}}}` | Whether the nested type would be allowed; it is never executed |

The `Инструкции` window has buttons that create and immediately process examples of all five
types. `DRY_RUN мотора` asks about `run_current_trial`; the expected result is
`hardware_commands_disabled` with `executed: false`.

## Events and status

Every state transition is an immutable event file. A normal `ping` produces three events with the
same `command_id` and increasing `sequence`:

1. `accepted`;
2. `running`;
3. `completed` with the result in `details.result`.

Invalid JSON, expired commands, filename/UUID mismatch, reused UUID with different content and
unsupported types produce `rejected`. A repeated file with the same path and SHA-256 produces no
second event. `status.json` is the only replaceable output and reports the current version,
Instance ID, command counts and allowlist. It deliberately does not publish the absolute local
Windows path.

If FOCTwin closes after journaling a valid command but before completion, the next scan resumes
the `received` or `running` diagnostic command. The five beta capabilities are side-effect-free,
so replay after a process interruption is safe.

## Safety boundary

The instruction backend imports neither Qt nor any FOCTwin Serial, Commander, friction or current
trial module. The allowlist is fixed in code. Names such as `run_current_trial`, `enable_pwm`,
`set_target`, `serial` and `commander` are rejected as `hardware_commands_disabled`; arbitrary
Python, PowerShell, executable launch and raw Commander text have no handler.

This release does not have an arming policy and therefore cannot run hardware commands even when
a motor is connected elsewhere in FOCTwin. Hardware execution belongs to a later beta after the
button-driven current trial is extracted behind one shared safety controller.

## Home test

1. Open `Инструкции` in FOCTwin; no project or COM port is needed.
2. Select the exchange root used by FolderBridge and copy the displayed Instance ID.
3. Press `Создать PING`, `Самопроверка` and `DRY_RUN мотора`. All three rows should finish; the dry
   run must say that motor execution is blocked.
4. Configure the two FolderBridge jobs and run them once.
5. Put a new valid command file in `Drive/FolderBridge/Autotuner/to_pc/commands`.
6. After both synchronization directions run, verify one accepted/running/completed chain in
   `Autotuner/from_pc/events` and the matching counts in `status.json`.
7. Synchronize the unchanged input again and verify that no duplicate event appears.
8. Close FOCTwin immediately after delivering several commands, reopen it and verify that every
   valid UUID reaches one terminal state.
