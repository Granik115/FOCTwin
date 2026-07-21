# Safety constraints and limitations

## What FOCTwin can enforce

- Refuse a scenario whose declared target/limit is outside the active profile.
- Write stricter current, voltage and velocity limits before a test.
- Observe available monitor data and stop when a sample exceeds the envelope.
- On stop: command target zero and transmit `AE0` three times.
- Start every trial from a recorded state and reject incomplete trials.
- Keep the last accepted parameter set for rollback.

Streamed SimpleFOC current values are expressed in mA by the bundled firmware. FOCTwin
normalizes them to A before applying the configured host-side thresholds.

## What FOCTwin cannot guarantee with the current firmware

- PWM shutdown after the USB cable disconnects.
- PWM shutdown after Windows or the application freezes.
- A deterministic stop latency over a 115200-baud text protocol.
- Detection of temperature, supply faults or mechanical contact without sensors.
- Detection of a violation that happens between received telemetry samples.

Consequently, automated real-motor tuning must not be marketed or treated as unattended
operation. A human must remain able to remove motor power.

## Preflight policy for real tests

1. Open a project and select a known profile.
2. Verify position is within the allowed travel and the cable is free.
3. Connect while PWM remains disabled.
4. Check the saved desired configuration and the separate actual-limit readback.
5. Apply the full saved configuration with paced writes.
6. Verify that monitoring produces fresh samples.
7. Only then enable PWM and begin the bounded trial.
8. Disable PWM between trials unless continuous hold is explicitly required.

## Low-speed friction experiment (0.3.0)

The first automatic experiment is deliberately narrow: velocity targets are limited to
0.005–0.05 rad/s, the SimpleFOC current limit to 0.03–0.05 A, the velocity limit to 0.3 rad/s
and travel to a user-selected subset of [-3, 3] rad. The default sequence alternates direction:
`+0.02`, `-0.02`, `+0.05`, `-0.05 rad/s`.

FOCTwin forces all seven monitor fields for the test. If telemetry becomes stale, it sends
target zero and repeated `AE0`, lets the normal monitor/DTR recovery restore the stream, reapplies
the bounded experiment configuration and repeats the interrupted point. It never increases a
limit automatically when the motor fails to move. Exceeding a current, voltage, velocity or
position bound ends the experiment and leaves PWM disabled.

The result is a rough initial estimate. A point is rejected when the measured direction,
tracking error or speed stability is unsuitable. Identified coefficients are written to the
active motor profile only after the user explicitly accepts a valid four-point result.

## Device-limit interaction

The bundled firmware initializes `current_limit` to 10 A. Sending `ALC1` changes the live
motor object to 1 A until another command or a controller reset. Because phase resistance is
configured, SimpleFOC also uses that current limit as the velocity PID output limit in Voltage
torque mode. A motor that no longer starts after `ALC1` is therefore current-limited rather
than necessarily frozen.

The same linking applies to the other nested controllers: `ALV` synchronizes the angle PID
output limit (velocity), while `ALU` synchronizes both current PID output limits (voltage).
FOCTwin therefore exposes these three authoritative limits only in the SimpleFOC limit panel
instead of offering conflicting editable copies in the PID tables.
