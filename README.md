# FOCTwin

**Identify. Simulate. Tune.**

FOCTwin is a Windows engineering workstation for a SimpleFOC-controlled PMSM:

1. record controlled experiments on the real azimuth drive;
2. identify inertia and nonlinear friction for a Simulink digital twin;
3. tune current, velocity and position controllers in simulation;
4. refine the accepted parameters on the real drive with bounded steps and rollback.

The project intentionally exposes the full control surface instead of hiding it behind a
single wizard. Manual control, raw Commander commands, repeatable FOCTwin scenarios,
identification, tuning, analysis, project history and detailed logs live in separate sections.

> [!WARNING]
> The current firmware cannot be changed and has no confirmed host heartbeat shutdown.
> Software emergency stop is therefore best-effort (`target=0`, then `AE0` repeatedly).
> Loss of USB or a frozen PC cannot guarantee PWM shutdown. Keep physical access to power
> during every real-motor test.

## Current milestone

Version 0.3.10 contains:

- a runnable PySide6 desktop shell with full-control workspaces;
- a typed SimpleFOC Commander encoder for device ID `A`;
- a serial transport with paced Commander writes, transparent DTR recovery and a best-effort
  emergency stop;
- verified read/apply controls for the linked device limits and every firmware PID/LPF loop;
- fragmentation-safe monitoring with staged stream recovery, rejection/counters for damaged USB
  rows, stable live plots, correct mA-to-A conversion, live rate/jitter and non-blocking durable
  CSV recording;
- safe reconnect that requests `AE0` before restoring monitoring and reading configuration;
- persistent manual-control values and one paced action for limits, PID/LPF, modes, target and
  monitoring before PWM is enabled;
- an actuator preflight that alternates short direct-Uq pulses, zeros Uq on the first detected
  movement and finds breakaway independently in both directions;
- an evidence mode that compares raw encoder/SSI behaviour with PWM disabled and enabled, preserves
  both raw and accepted angles, and distinguishes PWM-correlated dropouts from independent faults;
- residual breakaway confirmation after Uq returns to zero, with repeated signed trials that keep
  reversible elastic motion separate from static-friction thresholds;
- a two-electrical-period position preset derived from the motor pole-pair count, plus explicit
  tests for electrical-period repetition and approach-direction hysteresis;
- fixed-Uq, distance-bounded velocity comparisons and repeated fixed-limit position steps of ±0.1,
  ±0.3 and ±0.6 rad, so controller evidence and same-start dispersion are not altered by adaptive
  voltage ceilings;
- separate recording of commanded Uq, actual Uq and measured Iq; movement in both directions
  advances the diagnostic, while missing measured-current evidence invalidates affected friction
  points instead of hiding the rest of the experiment;
- a gated six-point velocity experiment at low, geometric-middle and high speed in both
  directions, retaining transient rise, acceleration and overshoot diagnostics as well as
  steady-state tracking;
- a cumulative position map that keeps each breakaway coordinate and divides velocity sweeps into
  configurable angle bins, storing measured-Iq torque separately from a diagnostic Uq estimate;
- optional multi-position friction runs: the current coordinate is measured first, then the
  loaded angle/velocity controllers move by a signed configurable step and repeat a zero baseline,
  local preflight and all six velocity points at every bounded position;
- configurable forward/reverse map passes that revisit coordinates to expose approach-direction
  hysteresis and repeatability instead of collapsing every coordinate into one sample;
- adaptive automatic positioning that raises only its own ALC after both a progress stall and
  confirmed Uq saturation, up to a separately approved voltage ceiling; a stall without
  saturation is reported as a controller/mode problem;
- continuous host-side angle tracking across board resets, with position commands translated back
  into the board's current turn reference so a ±2π reset cannot request an extra revolution;
- sustained-current validation that rejects sparse or wrong-sign Iq bursts instead of accepting
  a speed point after only three nonzero current samples;
- automatic stop/recovery/retry of an interrupted friction point when telemetry stalls or the
  Serial link reconnects, with 50 recoveries by default for board-reset workflows;
- an optional short alert after telemetry finally returns from an interruption longer than five
  seconds, selectable in every test-start confirmation;
- a structured diagnostic report covering zero-signal quality, position-step performance,
  breakaway envelopes, directional asymmetry, repeated-position variability, speed tracking,
  measured-Iq usability, telemetry interruptions and readiness for Simulink identification,
  saved as a short standalone JSON alongside the full experiment export;
- robust objective-dispersion evidence and a recommended repeat count for a future guarded
  real-motor optimizer, plus a data-driven stateful semi-mechanical friction-model recommendation;
- explicit review before identified friction values are accepted into the active profile;
- automatically synchronized experiment, FOCTwin and SimpleFOC limits, with command ALC kept
  separate from the independently measured-current emergency threshold and every change stated
  in the confirmation dialog;
- automatic reconciliation of the positioning and fixed-velocity Uq ceilings with the pulse
  ceiling and the global experiment voltage limit, reported once without blocking test startup;
- PWM-off observations ignore inactive Uq/Ud telemetry while retaining current and travel safety,
  and a checkpoint taken after that observation resumes with PWM-enabled actuator preflight;
- temporary direct-voltage operation for actuator preflight followed by automatic restoration of
  phase resistance and the user's manual configuration with PWM left disabled;
- rolling angle-slope speed and confirmed small-motion detection for experiment decisions, with
  isolated encoder dropouts/jumps rejected while impossible firmware velocity remains diagnostic;
- debounced working limits for isolated telemetry/control spikes, while twofold excursions,
  travel violations and telemetry loss still stop immediately;
- editable safety limits and a Russian FOCTwin scenario language;
- durable project folders with SQLite events, atomic checkpoints and raw telemetry files;
- the supplied voltage/current Simulink models and MATLAB tuning sources;
- JSON-file MATLAB simulation and checkpointed `surrogateopt` tuning APIs for R2022b;
- tests for protocol encoding, scenarios, safety checks and project recovery.

Hardware execution and MATLAB simulations are deliberately not started automatically.
They become active only after a user opens a project and explicitly connects/enables them.

## Download for Windows

Open [GitHub Releases](https://github.com/Granik115/FOCTwin/releases), expand **Assets** for
the newest version and download `FOCTwin-<version>-windows-x64.zip`. Extract the whole
archive and run `FOCTwin.exe`; installation and a separate Python environment are not
required for manual motor control.

Preview builds are intentionally marked as pre-releases while real-hardware testing is in
progress. They are not code-signed yet, so Windows SmartScreen can display a warning. MATLAB
R2022b and its Python Engine are still required for the simulation and tuning workspaces.

## Update a source checkout

`git pull` is supported for the development/project form of FOCTwin. Clone and update the
stable project branch with:

```powershell
git clone https://github.com/Granik115/FOCTwin.git
cd FOCTwin
git switch main
git pull --ff-only origin main
```

An existing editable Python installation uses the updated sources immediately. Run
`python -m pip install -e ".[dev]"` again only when dependencies change. A portable ZIP from
GitHub Releases has no Git metadata and is updated by downloading the next ZIP instead.

## Development setup

MATLAB R2022b supports Python 3.10, so FOCTwin pins that interpreter family.

```powershell
py -3.10 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
python -m foctwin
```

MATLAB Engine is installed from the R2022b installation rather than from PyPI:

```powershell
cd "C:\Program Files\MATLAB\R2022b\extern\engines\python"
python -m pip install .
```

## Tests

```powershell
python -m unittest discover -s tests -v
ruff check src tests
```

## Repository layout

```text
src/foctwin/        Python application
matlab/api/         stable JSON input/output interface
matlab/models/      supplied Simulink models
matlab/tuning/      tuning prototypes and future optimization services
profiles/           versioned motor/profile defaults
docs/               requirements, architecture and safety notes
tests/              core tests that do not require MATLAB or hardware
```

The manual-control behaviour and the distinction between firmware and host-side limits are
documented in [`docs/manual-control.md`](docs/manual-control.md).

No software license has been selected yet.
