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

## Multi-position two-stage friction experiment (0.3.6)

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

Measured current must be present in most samples and predominantly have the sign required by the
commanded direction. Sparse bursts or alternating-sign current no longer validate a point. Each
velocity sweep is also divided into angle bins. Local Uq, speed and a resistance/back-EMF estimate
are retained for diagnosis, but that voltage-equivalent torque cannot be accepted as measured
friction.

During this experiment, the firmware velocity field is diagnostic. Safety speed comes from the
rolling angle slope. This keeps impossible isolated values such as +43 rad/s from stopping a
stationary shaft while preserving actual speed and travel protection. The angle path also rejects
an isolated zero/dropout or jump that returns on the next sample; sustained motion is retained.

## Failure and restoration policy

Queued configuration writes are cancelled before an abort so a stale `AE1` cannot re-enable PWM.
Normal completion, a user stop, a safety stop and application close all send the emergency
sequence. Normal finalization then restores phase resistance, device limits, PID/LPF settings,
modes and monitoring without enabling PWM. Safe reconnect also restores phase resistance before
manual operation.
