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
