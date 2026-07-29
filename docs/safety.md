# Safety constraints and limitations

## What FOCTwin can enforce

- Synchronize the test, host and SimpleFOC envelopes after explicit confirmation.
- Disable PWM before changing control modes or phase-resistance compensation.
- Stop a direct-Uq pulse as soon as angle movement crosses the configured threshold.
- Keep command ALC separate from the measured-Iq emergency threshold and reject a friction point
  whose measured current remains inside the noise floor.
- Derive experiment speed from angle instead of trusting impossible firmware velocity spikes.
- Stop immediately on travel violation or a twofold current, voltage or angle-speed excursion.
- Require three consecutive samples for a smaller working-limit excess.
- On stop: command target zero, send `AE0` repeatedly and restore phase resistance.

Streamed SimpleFOC current values are expressed in mA by the bundled firmware. FOCTwin
normalizes them to A before applying thresholds or calculating torque.

## What FOCTwin cannot guarantee with the current firmware

- PWM shutdown after the USB cable disconnects.
- PWM shutdown after Windows or the application freezes.
- A deterministic stop latency over a 115200-baud text protocol.
- Detection of temperature, supply faults or mechanical contact without sensors.
- Detection of a violation between received telemetry samples.

Automated real-motor tests therefore remain attended operations. A human must be able to remove
motor power immediately.

## Evidence diagnostic experiment (0.3.10)

Evidence mode begins each position with an attended PWM-disabled observation. It does not issue an
enable command until that observation is complete. The following PWM-enabled zero baseline uses
the same raw-angle counters, so the final report can compare dropout and velocity-spike rates
without silently filtering the source evidence out of the CSV.

During the PWM-disabled observation, Uq/Ud telemetry may contain a stale or internal firmware
value even though the bridge is disabled. FOCTwin therefore suspends only the Uq/Ud working-limit
checks for that phase. Measured-current, coordinate and host-derived angle-speed checks remain
active, and voltage checks resume when actuator configuration enables PWM.

A pulse is not accepted as breakaway merely because its peak angle crossed a threshold. Uq is
zeroed, the configured verification interval elapses, and the signed residual displacement must
remain above the evidence threshold. Motion that returns is retained as evidence of elastic
deflection, backlash or encoder quantization. Each direction is repeated before the test advances.

Velocity comparisons use one user-approved Uq ceiling at every coordinate and terminate after a
bounded travelled distance. Fixed-limit position-validation steps likewise disable adaptive ALC.
Consequently the experiment may report a failed point that legacy positioning could eventually
reach by increasing its ceiling; that failure is intentional controller evidence.

The two-electrical-period preset only constructs a measurement plan. It does not widen travel,
speed, voltage or measured-current safety limits. The full planned sweep must fit inside the
existing envelope before PWM can be enabled.

Evidence mode does not perform automatic controller tuning. Its repeatability estimate is
diagnostic input for a future direct real-motor optimizer; it must not be interpreted as permission
to test unstable gains or remove the independent return controller.

Before startup, FOCTwin 0.3.9 reconciles the positioning and fixed-velocity Uq ceilings with the
configured pulse ceiling. A lower dependent ceiling is raised only up to the already configured
pulse value, and an excessive dependent ceiling is reduced to the unchanged global experiment
limit. The application reports all such changes once and continues; it does not silently increase
the global voltage envelope or the independent measured-current trip.

## Legacy multi-position diagnostic behaviour (0.3.7 compatibility)

The actuator preflight temporarily writes the SimpleFOC `NOT_SET` sentinel to phase resistance.
With `torque + Voltage`, this makes the target a direct Uq command. Default pulses alternate from
±0.1 V to ±0.5 V and last 0.5 s at most. The user-approved maximum Uq and measured-current trip
limit are shown before start. FOCTwin automatically synchronizes the active host and SimpleFOC
limits. The resistance-based current estimate is displayed as a command estimate and does not
silently widen the independent measured-current trip.

Confirmed movement in each direction immediately commands Uq=0. Wrong-direction motion aborts
the experiment. Movement unlocks the diagnostic velocity stage independently of current-sensor
quality; any velocity point whose measured Iq stays inside the noise floor is invalid and cannot
be accepted. No voltage-derived current is accepted as a substitute for the final torque.

Before velocity control starts, PWM is disabled and the configured phase resistance is restored.
The velocity-controller command limit is set slightly above the larger breakaway-equivalent
current. The measured-current trip remains independent and can stop the drive at a lower value.
Final torque and friction coefficients use measured Iq only.

Automatic positioning starts from the breakaway-derived limit but does not assume that one
position represents the whole mechanism. If progress stops while measured Uq is at least 90% of
the active positioning ceiling, FOCTwin raises only the positioning ALC by the configured step.
It never exceeds the separately confirmed maximum positioning Uq or the global experiment
voltage limit. A stall without Uq saturation aborts instead of blindly increasing torque and is
reported as a likely angle/velocity controller or mode-path problem.

Measured current must be present in most samples and predominantly have the sign required by the
commanded direction. Sparse bursts or alternating-sign current no longer validate a point. Each
velocity sweep is also divided into angle bins. Local Uq, speed and a resistance/back-EMF estimate
are retained for diagnosis, but that voltage-equivalent torque cannot be accepted as measured
friction.

During this experiment, the firmware velocity field is diagnostic. Safety speed comes from the
rolling angle slope. This keeps impossible isolated values such as +43 rad/s from stopping a
stationary shaft while preserving actual speed and travel protection. The angle path also rejects
an isolated zero/dropout or jump that returns on the next sample; sustained motion is retained.

The reverse map pass and the third speed level intentionally make the full diagnostic longer.
They provide repeated coordinates, approach-direction evidence and enough speed levels to detect
when a single Coulomb-plus-viscous line is inadequate. The start dialog shows the resulting
position sequence and estimated worst-case duration.

## Failure and restoration policy

Queued configuration writes are cancelled before an abort so a stale `AE1` cannot re-enable PWM.
Normal completion, a user stop, a safety stop and application close all send the emergency
sequence. Normal finalization then restores phase resistance, device limits, PID/LPF settings,
modes and monitoring without enabling PWM. Safe reconnect also restores phase resistance before
manual operation.
