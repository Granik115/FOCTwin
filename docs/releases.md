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
