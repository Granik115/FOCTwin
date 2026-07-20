# FOCTwin requirements baseline

This baseline records the user's answers from 2026-07-19. It is the source of truth for
the first implementation; every safety value remains editable in a project profile.

## Platform and dependencies

- Windows 10 and Windows 11, x64.
- Russian-only interface.
- MATLAB R2022b with MATLAB, Simulink, Simscape, Simscape Electrical, Motor Control
  Blockset, Global Optimization Toolbox, MATLAB Compiler and Simulink Compiler available.
- Full MATLAB is installed on the work computer.
- The application works offline except for release updates.
- Both source/development mode and an installed application are required.
- Python 3.10 is selected because it is compatible with MATLAB Engine R2022b.

## Hardware scope

- First target: azimuth axis, one JCM115x25S motor and Commander ID `A`.
- Future target: two axes and multiple saved motor/setup profiles.
- Firmware must not be changed or flashed by FOCTwin.
- Existing USB CDC / Serial protocol, nominally 115200 baud.
- Expected telemetry: angle, velocity, Id and Iq; voltage/other fields are read when the
  existing Commander monitor exposes them.
- COM number is stable; automatic VID/PID search is not required.
- No torque or temperature sensor.
- The cable can wind around the rotating assembly; default software travel is ±2π and is
  user configurable.
- There is no confirmed independent emergency stop. Physical power removal is the last
  resort.

## Baseline safety envelope

- Current: 1 A.
- Motor/driver voltage limit: 12 V, even though the supply bus is 48 V.
- Position: ±2π rad.
- Velocity: configurable; initial profile keeps the current 0.7 rad/s limit.
- Every excitation type has current, voltage, velocity, position and duration limits.
- A telemetry violation triggers target zero and repeated `AE0` commands.
- Because firmware cannot be changed, USB/PC failure cannot provide a guaranteed shutdown.

## Identification

- Primary parameters: inertia, viscous/Coulomb/breakaway friction, friction asymmetry and
  position-dependent irregularity/cogging.
- Rs, Ld, Lq and flux linkage can be enabled later with catalogue values as initial guesses.
- Excitations: step, ramp, sine, chirp, PRBS and custom FOCTwin scenarios.
- Repetitions, initial positions and bounds are configurable, potentially hundreds of trials.
- Target validation tolerances start at 0.001 rad, 0.001 rad/s and 0.001 A, all configurable.
- Models must be validated on trajectories/positions outside the immediate fit batch.
- Profiles preserve identified setups as separate versions.

## Controller tuning

- Supported torque modes: Voltage and FOC Current.
- Current q/d loops can be tuned jointly or separately.
- Position, velocity and current P/I/D, LPF, output ramps, output limits and simulated Kc
  are selectable tuning parameters.
- Objectives and weights are selectable; default priority is steady-state position/velocity
  accuracy under hard safety constraints.
- Test several initial/target positions.
- Typical virtual tuning budget: about one hour, each simulation a few seconds.
- Worker count/resource use is configurable.
- Store at least the best five configurations.
- Optional manual approval between stages.

## Real-motor refinement

- Manual approval, fully automatic and hybrid modes.
- Configurable per-parameter step; default 10%.
- Hundreds of trials are permitted, with resume after power/Serial interruption.
- Current interrupted trial is invalidated and repeated from a known state.
- Default targets: 30 arcsec position and 30 arcsec/s velocity error.
- Preserve every attempted and accepted parameter set; support rollback to known-good.

## Interface and data

- Separate full-control sections rather than a mandatory wizard.
- Manual SimpleFOC Studio-like control is required.
- Optional real/virtual overlay and configurable signal selection.
- Full data-analysis workspace; one run need not be permanently overlaid with another.
- JSON and Excel export.
- Project is a user-selected folder and retains all raw data for now.
- Raw Commander console plus a FOCTwin scenario language.
- Detailed diagnostic logging can include all available working data.
- Stable/beta update channels, GitHub Releases, hash/signature verification and rollback.

