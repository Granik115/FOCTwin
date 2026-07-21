# Safety constraints and limitations

## What FOCTwin can enforce

- Refuse a test whose declared envelope is outside the active host safety limits.
- Disable PWM before changing control modes or phase-resistance compensation.
- Stop a direct-Uq pulse as soon as angle movement crosses the configured threshold.
- Require measured Iq in multiple samples before starting friction identification.
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

## Two-stage friction experiment (0.3.3)

The actuator preflight temporarily writes the SimpleFOC `NOT_SET` sentinel to phase resistance.
With `torque + Voltage`, this makes the target a direct Uq command. Default pulses alternate from
±0.1 V to ±0.5 V and last 0.5 s at most. The user-approved maximum Uq and measured-current trip
limit must fit inside the active host envelope, and the resistance-based current estimate for the
largest pulse must not exceed the current trip.

The first movement in each direction immediately commands Uq=0. Wrong-direction motion aborts
the experiment. The velocity stage is blocked unless both directions moved and measured Iq was
above its noise-derived threshold in at least three samples. No voltage-derived current is
accepted as a substitute.

Before velocity control starts, PWM is disabled and the configured phase resistance is restored.
The velocity-controller current limit is set slightly above the larger breakaway-equivalent
current but remains below the experiment trip limit. Final torque and friction coefficients use
measured Iq only.

During this experiment, the firmware velocity field is diagnostic. Safety speed comes from the
rolling angle slope. This keeps impossible isolated values such as +43 rad/s from stopping a
stationary shaft while preserving actual speed and travel protection.

## Failure and restoration policy

Queued configuration writes are cancelled before an abort so a stale `AE1` cannot re-enable PWM.
Normal completion, a user stop, a safety stop and application close all send the emergency
sequence. Normal finalization then restores phase resistance, device limits, PID/LPF settings,
modes and monitoring without enabling PWM. Safe reconnect also restores phase resistance before
manual operation.
