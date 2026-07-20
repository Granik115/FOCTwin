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

This first foundation contains:

- a runnable PySide6 desktop shell with full-control workspaces;
- a typed SimpleFOC Commander encoder for device ID `A`;
- a serial transport with a best-effort emergency stop;
- verified read/apply controls for device limits and every firmware PID/LPF loop;
- configurable monitoring with correct mA-to-A conversion, live rate/jitter and durable CSV recording;
- safe reconnect that requests `AE0` before restoring monitoring and reading configuration;
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
