# FOCTwin releases

Windows test builds are produced by `.github/workflows/windows-release.yml`.

Every pull request to `main` produces or refreshes the pre-release for the version declared
in `pyproject.toml`. A release branch can run the same workflow without a pull request.

To publish through a release branch:

1. update the project version in `pyproject.toml`;
2. commit and validate the update;
3. create and push `release/v<version>` at the validated commit;
4. wait for the Windows release workflow to pass.

The workflow runs tests and Ruff before PyInstaller. It then publishes a portable ZIP and
its SHA-256 checksum as both a short-lived Actions artifact and a persistent GitHub
pre-release asset. The release branch name must exactly match the project version, which
prevents accidentally publishing a commit under the wrong version number.

Until code signing is introduced, Windows SmartScreen may warn when the executable starts.

## 0.5.0b3

- Make manual **Torque + Voltage** control genuinely direct: FOCTwin sends the SimpleFOC
  `NOT_SET` phase-resistance sentinel before applying the mode, so target `A1` means `Uq=1 V`
  instead of a one-ampere target converted through the configured phase resistance.
- Restore the configured `0.675 ohm` phase resistance whenever manual control returns to angle,
  velocity or a current torque mode.
- Label the manual target with its active physical unit and explain the direct-Uq conversion in a
  tooltip.
- Reject a visible direct-voltage program whose known target exceeds its known `ALU` limit. For
  example, `AR-12345 / AT0 / AC0 / ALU1 / A2` now fails validation instead of silently producing
  only a 1 V pulse. Raw commands are never rewritten.
- Persist the last project path and its parent directory. A valid previous project opens
  automatically on the next launch, and both project pickers start in the remembered directory.
- Preserve the corrected SimpleFOC monitor current units from 0.5.0b2 and the local FolderBridge
  file workflow from 0.5.0b1.

## 0.5.0b2

- Restore the documented SimpleFOC monitor conversion from milliamperes to amperes. SimpleFOC's
  monitor prints Q/D currents as `current * 1000`, while Commander current-limit commands such as
  `ALC` remain in amperes.
- Prevent false emergency stops from ordinary monitor packets. The observed manual-control packet
  `Iq=42.6634`, `Id=70.4849` is now interpreted as `0.0426634 A` and `0.0704849 A` rather than as
  physically impossible tens of amperes.
- Keep real current protection active after conversion: a streamed value of `12000.0000 mA` still
  reaches the host guard as `12 A` and trips a `5 A` limit immediately.
- Correct the mistaken 0.4.2b5 current-unit assumption inherited by 0.4.2b6 and 0.5.0b1. Current
  columns in CSV files produced by those affected versions are labelled as amperes but must be
  divided by 1000 before analysis.
- Preserve the local `.focscript` workflow, external FolderBridge synchronization and all other
  0.5.0b1 behaviour.

## 0.5.0b1

- Replace the inactive high-level scenario editor with an executable, deliberately small
  `.focscript` language: exact firmware Commander lines plus `WAIT <seconds>`.
- Add manual Open, Save, Save As, Validate, Run and Stop controls under the renamed
  **Команды и программы** workspace. Editing invalidates the previous validation.
- Create one unique plain-file directory per run with `program.focscript`, ordered
  `execution.jsonl`, `telemetry.csv` and atomic `summary.json`.
- Abort a program on Serial loss, write failure or telemetry-log failure; never auto-resume after
  reconnect. User Stop and the global emergency action send the existing best-effort stop.
- Keep normal completion literal: FOCTwin adds no hidden firmware command, so the program's safe
  `A0`/`AE0` ending remains visible and reviewable.
- Remove the built-in GPT/Google Drive bridge, instruction scanner and their cloud dependencies.
  FolderBridge remains an external file synchronizer with no direct motor path.
- Replace stale editable COM fallbacks with the actual enumerated ports and explain missing cable,
  power or driver when no port exists.
- Preserve the 0.4.2b6 passive current-sense baseline and guarded motor experiments while removing
  their remote-instruction UI entry points. Its inherited current-unit assumption is corrected in
  0.5.0b2.

## 0.4.2b6

- Add a `0.3 s` passive current-sense baseline before the first motor configuration command.
- Record Iq/Id during PWM OFF as diagnostic sensor data without mistaking it for proven physical
  current. A peak full-current signal above `0.05 A` or non-zero Uq/Ud fails before PWM enable.
- Export passive axis statistics, full-current RMS/peak, full-voltage peak and explicit problems in
  the result JSON and UI report.
- Add a regression using all ten raw trial-63 samples and verify the `1.01052 A` signal stops the
  new state machine without ever requesting transport or PWM.

## 0.4.2b5

- Preserve SimpleFOC monitor Q/D currents in their native ampere units instead of dividing them by
  1000. The old conversion weakened all host-side measured-current protection by three orders of
  magnitude.
- Add a regression test for the exact trial-61 packet (`Iq=-5.8088 A`, `Id=-29.9526 A`) and prove
  that both values reach the immediate emergency guard.
- Keep remote hardware execution disabled until the corrected build is installed and a fresh
  PWM-off baseline establishes whether the board current-sense itself is usable.

This release note records what 0.4.2b5 changed, but its unit assumption was incorrect: the
SimpleFOC monitor serializes Q/D currents in milliamperes. Version 0.5.0b2 restores the required
division by 1000.

## 0.4.2b4

- Keep an exact locally approved remote plan armed without a timer. The permission remains
  one-shot, is consumed only by a separate `start_current_trial`, and is still revoked by process
  restart, cancellation or any changed motor/telemetry/PWM prerequisite.
- Replace the oversized message box with a bounded confirmation dialog: the immutable JSON plan
  is scrollable, the action buttons always remain visible, and Enter activates the explicit
  `Разрешить план` button instead of cancelling.
- Treat a physically impossible single coordinate jump as a candidate damaged packet. Its angle
  and firmware-derived velocity are ignored for that packet only; a repeated coordinate on the
  next packet is accepted as real and immediately reaches the existing emergency checks.
- Add regression coverage for the exact `0.0588 → 0.0000 rad`, `-5.6167 rad/s` packet observed in
  remote trial 59, for persistent jumps, no-timeout arms and restart revocation.

## 0.4.2b3

- Reduce the guarded current step from `0.1 A` to `0.01 A`; start current Q/D at `P=0.4`, `I=40`,
  ramp `50 V/s`, target ceiling `0.1 A` and working voltage `2 V`.
- Add a `0, +0.01, 0, -0.01 V` direct-voltage current-sense preflight. FOC Current is never enabled
  unless Iq follows both voltage signs and the Q response dominates Id.
- Stop the trial immediately on a first telemetry-speed sample above twice the `0.5 rad/s` working
  limit, while retaining confirmed working-limit and independent angle-slope checks.
- Replace front-deleted 60,000-element plot lists with bounded deques, cap each rendered window at
  4,000 points and expose current/maximum Qt processing backlog in the UI and `status.json`.
- Stage status, event and artifact files outside the synchronized `outbox`, then atomically rename
  them into place. Automatically publish every `_SEND_ME.zip` under `outbox/artifacts`.
- Add a two-minute, exact-command local arm for hardware instructions. A separate
  `start_current_trial` consumes it once; restart, expiry and changed prerequisites revoke it.
- Route cancellation of a running instruction through the attended executor's emergency stop,
  preserve full-trial retry after genuine power/telemetry loss, and cover the new states with tests.

## 0.4.2b2

- Add one durable current-trial admission controller shared by the existing attended button and
  FolderBridge instructions. It validates the same `CurrentTrialConfig` for both callers and owns
  no Qt, Serial or Commander object.
- Persist request source, immutable config, environment, state and result in a separate SQLite
  journal. Demote a locally `ready` or `running` request to `waiting_for_permission` after process
  restart instead of resuming hardware automatically.
- Accept `run_current_trial` in `simulation` mode, complete a deterministic hardware-free plan and
  export `outbox/artifacts/<command_id>/current_trial_simulation.json` with `executed: false`.
- Accept hardware-mode requests only into `waiting_for_motor` or `waiting_for_permission`. Even
  after a motor appears, a remote request has no transition to `ready` and cannot issue PWM,
  Commander or Serial operations in this beta.
- Add `cancel_current_trial` for waiting requests, immutable waiting/cancelled events, accurate
  waiting counts in `status.json`, and UI buttons for simulation, queue and selected cancellation.
- Route the real local current-trial button through the same controller before the existing
  confirmation-driven executor, record the controller request ID in checkpoints and close the
  request with the physical experiment result.
- Extend tests for invalid configurations, queue transitions, restart recovery, cancellation,
  simulation artifacts and the permanent remote-PWM lock.

## 0.4.2b1

- Add a non-modal `Инструкции` window that can be tested without a project, COM port, motor,
  FolderBridge or Internet.
- Create FolderBridge-compatible `inbox/commands` and `outbox/events|artifacts|status.json`
  directories below a user-selected exchange root, while retaining the existing direct Drive chat
  unchanged for comparison.
- Accept one immutable schema-1 UTF-8 JSON file per command with strict UUID/filename, target,
  timezone, expiry, lifetime, depth and 256 KiB size validation.
- Persist command UUID/SHA-256, state and events in a power-safe local SQLite journal outside the
  synchronized tree. Repeated copy-mode downloads are deduplicated and changed reuse of a UUID is
  rejected as a conflict.
- Publish immutable `accepted`, `running`, `completed`, `rejected`, `ignored` and `failed` event
  files plus one atomically replaced status file. Resume journaled side-effect-free commands after
  an interrupted application run.
- Expose only `ping`, `get_status`, `list_capabilities`, `self_test` and `dry_run`; include UI
  buttons that generate valid local examples for the home test.
- Keep the backend free of Qt, Serial, Commander and experiment imports. Motor/PWM/raw command,
  shell and arbitrary-code paths do not exist; known hardware-like types are explicitly rejected.
- Document the then-current two one-way FolderBridge jobs, envelope/event formats and interruption
  test procedure (the obsolete runner and document are removed in 0.5.0b1).

## 0.4.1b1

- Keep the guarded current experiment from 0.4.0 unchanged on a separate Drive Bridge test
  branch.
- Add a non-modal `Связь с GPT` diagnostics window that does not open a COM port or require a
  project/motor.
- Authorize a user-owned Google Desktop OAuth client through the browser and store the refresh
  token in Windows Credential Manager; Google Drive Desktop is not required.
- Request only the per-file `drive.file` scope and create a uniquely identified
  `FOCTwin_Bridge_<ID>` folder with manifest, status and two single-writer JSONL channels.
- Save each outgoing chat locally before network I/O, merge by UUID and retain the queue across
  application termination, network loss and power interruption.
- Poll the one inbound file every three seconds with conditional ETags, resume automatically after
  transient network failures and expose account, queue, last-sync and technical-log status.
- Accept only schema-1 `chat` records from ChatGPT. Unknown/command records are logged and ignored;
  no Drive code imports or invokes Serial, Commander or current-trial actions.

## 0.4.0

- Add the first direct real-motor tuning building block: one guarded current step with automatic
  transport to and return from the freshly captured shaft coordinate.
- Switch only with PWM disabled: `Angle + Voltage` transport, neutral target, then
  `Torque + FOC Current`, neutral verification, one `0.1 A` step and a recorded zero tail.
- Start from conservative current Q/D values `P=8.4222`, `I=814`, `D=0`, LPF `0.005 s`, an
  output ramp of `3000 V/s`, a `0.5 A` command ceiling and `12 V` working voltage.
- Enforce independent stop envelopes: `1 A` during the experimental current section,
  `5 A / 24 V / 0.5 rad/s / ±4 rad` absolute limits, with the normal experiment kept within
  `±3 rad` and stopped before `±3.5 rad`.
- Record roughly `100 Hz` full-mask telemetry, compute baseline/step/post current, voltage,
  motion and rate statistics, and explicitly flag a missing measured-Iq response instead of
  treating it as a valid tuning result.
- On stale telemetry, Serial loss, a board reset or physical power interruption, send the
  best-effort stop, checkpoint immediately, discard the partial attempt and repeat the whole
  trial after fresh telemetry returns. Reconnect always begins with `AE0` even if manual safe
  connect is disabled.
- Sound the selected recovery alert as soon as the stream has remained unavailable for more
  than five seconds, rather than waiting until recovery has already succeeded.
- Export one `current_trial_<id>_SEND_ME.zip` with the JSON report and every CSV segment.

## 0.3.11

- Raise the explicit two-electrical-period preset to a 3 V direct-Uq preflight ceiling (or the
  lower global experiment limit) instead of retaining the generic 0.5 V default. A deliberately
  higher existing ceiling is preserved.
- Repair a failed schema-8 checkpoint whose 0.5 V ceiling did not find breakaway: keep completed
  PWM-off/on evidence and valid pulses, raise only the preflight and dependent ceilings to 3 V,
  show the correction, and continue above the already tested levels after confirmation.
- Never accept an actuator pulse without at least two complete Uq/Iq/angle samples. Stop safely,
  recover telemetry and retry the same signed voltage instead of recording a false no-motion
  result.
- Discard old zero-sample pulse attempts when restoring a checkpoint and preserve cumulative
  interruption and rejected-angle counters across application restarts.

## 0.3.10

- Do not apply Uq/Ud working-limit checks while evidence mode intentionally keeps PWM disabled.
  SimpleFOC can continue reporting a stale or internal voltage value in that state even though no
  drive voltage is applied. Measured-current, coordinate and angle-slope safety checks remain
  active.
- Resume a schema-8 checkpoint in actuator mode when the PWM-off observation for the current
  position is already complete. The restored configuration now enables PWM before the
  PWM-on baseline instead of leaving the motor in observer mode.
- Cover the measured `Uq=-0.627 V` with a `0.5 V` pulse ceiling and the post-observer checkpoint
  transition with regression tests.

## 0.3.9

- Reconcile the positioning and evidence-mode fixed-velocity Uq ceilings automatically before a
  friction test starts. Values below the pulse ceiling are raised, values above the global
  experiment limit are reduced, and all changes are shown in one warning without cancelling the
  run.
- Keep the global experiment voltage limit unchanged. Invalid primary pulse limits still fail
  validation instead of being silently widened.
- Cover the reported 10 V pulse / 3 V positioning preset conflict and upper-bound clamping with
  Qt smoke tests.

## 0.3.8

- Add an evidence protocol that records raw and accepted shaft angles and compares a PWM-disabled
  observation with an otherwise equivalent PWM-enabled zero baseline. The report distinguishes
  encoder/SSI dropouts that exist without PWM from faults whose rate rises with PWM.
- Confirm breakaway only after the pulse has ended and a configurable residual displacement
  remains. Peak motion that substantially returns during the zero interval is reported as
  elasticity, backlash or encoder quantization rather than static-friction breakaway.
- Repeat every signed breakaway measurement and report within-condition variability instead of
  treating one threshold as deterministic.
- Add a two-electrical-period experiment preset based on the configured pole-pair count. It
  samples eight mechanical positions per electrical period and tests whether the spatial pattern
  repeats at the electrical period.
- Run comparison velocity legs with one fixed Uq ceiling and stop them by travelled distance,
  preventing a local measurement from spanning several radians or changing its available
  actuation with a provisional breakaway estimate.
- Revisit positions after opposite approaches and report approach-direction hysteresis separately
  from ordinary repeat noise and directional friction asymmetry.
- After the map, repeatedly run fixed-limit position steps of ±0.1, ±0.3 and ±0.6 rad without
  adaptive ALC. These trials characterize the loaded position/velocity controller and the
  same-start objective dispersion rather than hiding behaviour behind automatic voltage increases.
- Estimate objective dispersion with robust statistics and recommend a minimum repeat count plus
  median/MAD aggregation before direct real-motor `surrogateopt` tuning is enabled in a future
  release.
- Recommend a stateful semi-mechanical Simulink friction structure from the observed evidence,
  including stick/slip state, coordinate and direction maps, approach history, elastic return,
  electrical-angle periodicity and bounded correlated variability when supported.
- Checkpoint and result schema 8 preserve observer state, raw-angle diagnostics, repeated pulses
  and the fixed-limit position-validation sequence.

## 0.3.7

- Diagnose the 0.3.6 automatic-position failure correctly: a residual error of 0.44–0.98 rad
  while Uq is pinned at 2.16 or 1.56 V is saturation, not an overly large 0.01-rad tolerance.
- Detect a progress stall, distinguish saturated from unsaturated positioning, and raise only the
  positioning ALC in bounded voltage-equivalent steps up to a separately configured maximum.
- Replace two speed magnitudes with low, geometric-middle and high speeds in both directions,
  retaining transient acceleration, rise-time and overshoot evidence.
- Add forward/reverse coordinate passes so the same position can be compared after opposite
  approaches for repeatability, hysteresis and changing preload.
- Save a zero baseline at every measurement position and record every automatic move with start,
  target, final error, duration, initial/final Uq ceiling, boost count, hold values and saturation.
- Produce a structured Russian diagnostic report with evidence and next actions for
  position-dependent or asymmetric friction, poor repeatability, controller-path faults,
  unreliable Iq, failed speed tracking, encoder/current zero quality and telemetry interruptions.
- Checkpoint and result schema 7 preserve the complete diagnostic state.

## 0.3.6

- Repeat the actuator preflight and all four velocity points at up to 20 automatic shaft
  positions. The first position is the current angle; later positions use a signed configurable
  step.
- Move between measurement positions with the loaded angle and velocity controllers in
  `angle + Voltage torque`, while retaining the experiment's voltage, velocity and travel limits.
- Validate the complete predicted sweep around every automatic position before PWM can be
  enabled, stop if a requested position does not settle within 60 seconds, and checkpoint the
  outer position index plus the inner velocity point.
- Keep a continuous experiment coordinate when a board reset changes the reported angle by a
  whole turn, while translating future position targets into the board's new coordinate frame to
  prevent an unintended extra revolution.
- Offer a per-start checkbox for a short sound when telemetry restoration took more than five
  seconds; the alert fires once per long interruption.
- Replace the ambiguous final “model cannot be used” wording with an explicit distinction:
  the motor and Uq position map remain usable, while unconfirmed/wrong-sign Iq cannot be treated
  as a physical torque measurement.
- Checkpoint and result schema 6 include automatic position targets and progress.

## 0.3.5

- Record the shaft coordinate of every direct-Uq breakaway event.
- Divide each useful velocity sweep into configurable position bins and retain local speed, Uq,
  measured Iq, measured torque and a separately labelled voltage-equivalent diagnostic torque.
- Accumulate completed runs into a project-level position-map JSON with measured and diagnostic
  min/max envelopes kept separate.
- Show the predicted travel of every velocity leg before start; for example 0.5 rad/s over the
  configured 2 s settling plus 4 s measurement spans about 3 rad.
- Require measured Iq to remain above its noise floor for most of a point and predominantly match
  the commanded torque direction; three isolated samples no longer validate friction.
- Checkpoint and result schema 5 preserve position observations without changing motor commands.

## 0.3.4

- Filtered isolated encoder zeroes and jumps without masking sustained shaft movement.
- Lowered the default confirmed movement threshold to 0.001 rad and require two matching samples.
- Separated the automatically calculated velocity-controller ALC from the measured-Iq trip.
- Automatically synchronize experiment, FOCTwin and SimpleFOC limits after one confirmation.
- Continue diagnostic speed points after movement in both directions while invalidating any point
  whose measured Iq remains inside the noise floor.
- Increased the default telemetry recovery allowance from 3 to 50 for board-reset workflows.

## 0.3.3

- Replaced the invalid immediate velocity test with a two-stage actuator/friction workflow.
- Actuator preflight alternates bounded direct-Uq pulses and zeros output on first movement.
- Commanded Uq, actual Uq and measured Iq are stored separately; voltage-derived current cannot
  unlock the velocity stage.
- Breakaway and current must be confirmed in both directions before four speed points run.
- Experiment speed protection uses angle slope; the untrusted firmware velocity field is logged
  but ignored for experiment stop decisions.
- Phase resistance is restored on transition, finalization, safe reconnect and application close.
- Aborts cancel pending command queues so a stale enable command cannot run after a stop.

## 0.3.2

- Initial friction identification now uses `velocity + Voltage torque`, isolating it from the
  not-yet-identified FOC Current loop.
- Equivalent torque current is reconstructed from Uq, phase resistance and back EMF; measured Iq
  is retained as a diagnostic value.
- Mean speed and stability are calculated from rolling angle slope, so noisy instantaneous
  firmware velocity no longer rejects uniform motion.
- Both experiment and host working-limit excess require three consecutive samples; twofold
  excursions and travel violations remain immediate stops.
- Old FOC Current checkpoints are rejected instead of mixing incompatible point estimates.

## 0.3.1

- Friction-test current, voltage, velocity, target-speed and travel inputs can be increased above
  the conservative 0.3.0 defaults after an insufficient-torque stop.
- Spin buttons use practical increments, including 0.01 A for the test current limit.
- Raised values persist between launches and are used by the generated Commander configuration.
- Test preflight continues to require the whole experiment envelope to fit inside FOCTwin's active
  software safety limits.

## 0.3.0

- First executable identification workflow: bounded four-point velocity friction test.
- Directional Coulomb, shared viscous and rough breakaway estimates with validity checks.
- Raw CSV, SQLite experiment result, JSON export and a checkpoint after every completed point.
- Automatic safe stop, telemetry recovery and retry of an interrupted point.
- Explicit acceptance of valid estimates into profile history; manual settings are restored with
  PWM left disabled after the test.
