# FOCTwin requirements baseline

Базовые требования сформированы 2026-07-19; файловый цикл программ уточнён для 0.5.0b2.

## Platform and dependencies

- Windows 10/11 x64, русский интерфейс.
- Python 3.10 в режиме исходников; portable Windows-сборка не требует отдельного Python.
- MATLAB R2022b и перечисленные проектом toolbox нужны для моделирования, но не для локального
  управления мотором.
- Основные разделы должны работать без Интернета.
- FOCTwin не авторизуется в Google и не синхронизирует Drive. Внешний FolderBridge может
  независимо переносить обычные файлы между Drive и локальными папками.

## Hardware scope

- Первая цель: азимутальная ось, один JCM115x25S и Commander ID `A`.
- Прошивка не изменяется и не прошивается из FOCTwin.
- Existing USB CDC / Serial, номинально 115200 baud.
- Ожидаемая телеметрия: angle, velocity, Id, Iq и доступные поля напряжения.
- COM выбирается только из реально найденных портов; VID/PID auto-discovery пока не требуется.
- Нет отдельного датчика момента или температуры.
- Физическое снятие питания остаётся последней гарантированной мерой остановки.

## Safety baseline

- Current: 1 A.
- Motor/driver voltage: 12 V при шине питания до 48 V.
- Position: ±2π rad.
- Velocity: редактируется, стартовый профиль ограничивает 0.7 rad/s.
- Телеметрическое нарушение вызывает target zero и несколько `AE0` best-effort.
- USB/PC failure не может гарантировать shutdown при неизменяемой прошивке.

## Local command programs

- Программа — UTF-8 text file, предпочтительно `.focscript`.
- Выполняются только точные однострочные команды существующей прошивки и `WAIT <seconds>`.
- В 0.5.0b2 отсутствуют переменные, выражения, условия, циклы и автоматический удалённый запуск.
- Перед каждым запуском пользователь открывает/редактирует файл, явно проверяет его и нажимает
  **«Запустить»** при подключённом моторе.
- Потерянное Serial-соединение прерывает запуск без auto-resume.
- Каждый запуск сохраняет source copy, ordered JSONL events, CSV telemetry и JSON summary в
  отдельной локальной папке.
- Папку открытия и папку результатов можно связать с Google Drive только внешней программой.

## Identification and tuning

- Основные параметры: inertia, viscous/Coulomb/breakaway friction, асимметрия и зависимость от
  координаты.
- Supported torque modes: Voltage и FOC Current.
- Position, velocity, current q/d P/I/D, LPF, ramps и limits доступны как параметры тюнинга.
- Модели проверяются на траекториях и координатах вне непосредственного fit batch.
- Каждый испытанный и принятый набор параметров сохраняется; требуется rollback к known-good.

## Interface and data

- Раздельные full-control sections без обязательного wizard.
- Ручное SimpleFOC Studio-like управление.
- Raw Commander console и локальные `.focscript`-программы.
- Configurable live signals, CSV telemetry, JSON/Excel export и подробные журналы.
- Проект — выбранная пользователем переносимая папка.
- Stable/beta channels через GitHub Releases, SHA-256 verification и rollback архивом.
