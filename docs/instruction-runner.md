# FOCTwin Instruction Runner 0.4.2b2

This beta connects the instruction channel to the same durable current-trial admission controller
used by the local button. It can validate and simulate a current trial, or preserve a hardware
request in a safe waiting state. Remote PWM execution remains disabled in code.

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
| `dry_run` | `{"command":{"type":"...","arguments":{}}}` | Whether the nested type is allowed and its execution policy; it is never executed |
| `run_current_trial` | `{"mode":"simulation","config":{...}}` | Complete a hardware-free plan simulation and export its JSON artifact |
| `run_current_trial` | `{"mode":"hardware","config":{...}}` | Validate and preserve the immutable request as `waiting_for_motor` or `waiting_for_permission` |
| `cancel_current_trial` | `{"command_id":"<waiting command UUID>"}` | Cancel a waiting hardware request; it never enables or configures a motor |

`config` may contain any correctly spelled `CurrentTrialConfig` field. Unknown fields, invalid
limits and unsafe relationships are rejected by the same validator used by the attended button.
Omitting `config` or individual fields uses the conservative current-trial defaults.

The `Инструкции` window has buttons for all diagnostics, a safe current-trial simulation, a
hardware queue example and cancellation of the selected waiting request. `DRY_RUN мотора` now
reports that `run_current_trial` is allowed only under the
`simulate_or_wait_for_local_permission` policy and still returns `executed: false`.

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

A simulated current trial produces `accepted → evaluating → completed` and writes
`outbox/artifacts/<command_id>/current_trial_simulation.json`. A hardware request produces
`accepted → evaluating → waiting_for_motor` at home. The `evaluating` event explicitly carries
`hardware_running: false`. If a motor later appears, the request may move only to
`waiting_for_permission`; this beta has no transition from a remote request to `ready` or PWM.
Cancellation adds one `cancelled` event to the target and completes the separate cancel command.

If FOCTwin closes after journaling a valid command but before completion, the next scan resumes
the `received` or `running` command. Waiting requests remain waiting after process or power loss.
A locally attended trial that was `ready` or `running` is demoted to
`waiting_for_permission` on controller restart and requires another visible confirmation.

## Safety boundary

The instruction backend imports neither Qt nor any FOCTwin Serial, Commander, friction or current
trial module. It communicates with the shared controller through a narrow typed callback. The
controller itself owns no Serial, Commander or Qt object. Names such as `enable_pwm`, `set_target`,
`serial` and `commander` remain absent from the allowlist; arbitrary Python, PowerShell,
executable launch and raw Commander text have no handler.

This release deliberately has no remote arming policy. The shared controller accepts an attended
local button only after a fresh environment check and the existing confirmation dialog. An
instruction source is always denied the `ready` transition, even if its JSON requests hardware,
all limits are valid and the motor is connected. Arming and real remote execution belong to the
next beta.

## Home test

1. Open `Инструкции` in FOCTwin; no project or COM port is needed.
2. Select the exchange root used by FolderBridge and copy the displayed Instance ID.
3. Press `Создать PING`, `Самопроверка`, `DRY_RUN мотора` and `Имитировать токовый опыт`. All four
   rows should finish. The simulation must say `simulated: true`, `executed: false` and create one
   artifact JSON.
4. Press `Поставить опыт в очередь`. Without a motor it must remain `waiting_for_motor` and must
   not create a Serial connection or a `completed` event.
5. Select that row and press `Отменить выбранный ожидающий опыт`; the target must become
   `cancelled`.
6. Configure the two FolderBridge jobs and run them once.
7. Put a new valid command file in `Drive/FolderBridge/Autotuner/to_pc/commands`.
8. After both synchronization directions run, verify one accepted/running/completed chain in
   `Autotuner/from_pc/events` and the matching counts in `status.json`.
9. Synchronize the unchanged input again and verify that no duplicate event appears.
10. Close FOCTwin immediately after delivering several commands and reopen it. Diagnostics must
    reach one terminal state; an unchanged hardware request must remain in exactly one waiting
    state without a second execution chain.
