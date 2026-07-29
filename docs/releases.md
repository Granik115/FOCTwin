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
