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
- editable safety limits and a Russian FOCTwin scenario language;
- durable project folders with SQLite events, atomic checkpoints and raw telemetry files;
- the supplied voltage/current Simulink models and MATLAB tuning sources;
- JSON-file MATLAB simulation and checkpointed `surrogateopt` tuning APIs for R2022b;
- tests for protocol encoding, scenarios, safety checks and project recovery.

Hardware execution and MATLAB simulations are deliberately not started automatically.
They become active only after a user opens a project and explicitly connects/enables them.

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

No software license has been selected yet.
