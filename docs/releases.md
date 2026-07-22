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
