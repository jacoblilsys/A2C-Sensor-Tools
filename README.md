# A2C Sensor Tools

Open-source Windows tools for configuring, monitoring, and updating A2C-IMU V2
sensors over classic CAN with a Kvaser interface.

The repository contains two Qt 6 applications:

- **A2C IMU Dashboard** configures CAN, IMU, vibration, filtering, averaging,
  sensor-fusion, gyro-bias, and periodic-message settings and displays live
  acceleration, vibration, gyro, and inclination data.
- **A2C Sensor Firmware Updater** validates an encrypted A2C firmware package,
  checks the connected sensor identity and version, transfers the package, and
  verifies the programmed firmware.

## Requirements

- Windows 10 or Windows 11
- Python 3.10 or newer when running from source
- A supported Kvaser CAN interface
- The current [Kvaser CANlib SDK](https://kvaser.com/canlib-sdk/)
- The matching [Kvaser Windows device driver](https://kvaser.com/canlib-webhelp/section_install_windows.htm)

**The Kvaser CANlib SDK must install `canlib32.dll` before either application
can use a CAN interface.** The A2C applications do not include or replace
`canlib32.dll` or the Kvaser hardware driver.

## Install from source

```powershell
git clone https://github.com/jacoblilsys/A2C-Sensor-Tools.git
cd A2C-Sensor-Tools
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install .
```

Start either application:

```powershell
a2c-imu-dashboard
a2c-firmware-updater
```

Command-line tools are also installed:

```powershell
a2c-can-monitor --help
a2c-firmware-update --help
```

## Firmware updates

Only use an encrypted A2C `.binenc` package supplied by A2C. Connect one sensor
that accepts the selected request ID, use **Check Sensor** before updating, and
keep CAN and sensor power connected until final firmware and CRC verification
complete.

The public updater cannot create or encrypt firmware packages and does not
contain empty-flash factory recovery functions or firmware encryption keys.

## Application updates

Select **Help > Check for Updates** to query the latest GitHub Release. The
check is asynchronous and occurs only when requested by the user. It sends no
CAN traffic, settings, sensor serial numbers, or telemetry. If a newer semantic
version is available, the application offers to open its release page.

## Development

```powershell
python -m pip install -e .
$env:QT_QPA_PLATFORM = "offscreen"
python -m unittest discover -s tests -v
```

The offline test suite does not open a Kvaser channel. Hardware validation is
maintained separately by A2C.

## License

Copyright 2026 A2C.

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
PySide6, Qt, and Kvaser CANlib remain subject to their own licenses; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
