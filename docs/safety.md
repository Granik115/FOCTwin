# Safety constraints and limitations

## What FOCTwin can enforce

- Refuse a scenario whose declared target/limit is outside the active profile.
- Write stricter current, voltage and velocity limits before a test.
- Observe available monitor data and stop when a sample exceeds the envelope.
- On stop: command target zero and transmit `AE0` three times.
- Start every trial from a recorded state and reject incomplete trials.
- Keep the last accepted parameter set for rollback.

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
4. Read back supported limits/controller values.
5. Apply the project's stricter current, voltage and velocity limits.
6. Configure monitoring and verify fresh samples.
7. Only then enable PWM and begin the bounded trial.
8. Disable PWM between trials unless continuous hold is explicitly required.

