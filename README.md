# A2C Sensor Tools

Open-source Windows tools for configuring, monitoring, and updating A2C-IMU V2
sensors over classic CAN.

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
- For either application: a supported Kvaser CAN interface with the current
  [Kvaser CANlib SDK](https://kvaser.com/canlib-sdk/) and matching
  [Kvaser Windows driver](https://kvaser.com/canlib-webhelp/section_install_windows.htm),
  or a PEAK-System PCAN-USB interface with the
  [PEAK Windows driver and PCAN-Basic API][pcan-basic]

**The Kvaser CANlib SDK must install `canlib32.dll` before either application
can use a Kvaser interface.** The A2C applications do not include or replace
`canlib32.dll` or the Kvaser hardware driver.

**PEAK support requires `PCANBasic.dll`, installed by PEAK's Windows driver/API
package.** The DLL and hardware driver are not included in this repository.
Both the dashboard and firmware updater support PEAK PCAN-USB channels.

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

Select **Kvaser CANlib** or **PEAK PCAN-Basic** in the updater's CAN interface
section. PEAK channels are shown as `PCAN_USBBUS1` through `PCAN_USBBUS16`.
The sample-point selector is disabled for PEAK because PCAN-Basic's classic-CAN
bitrate presets define their own bit timing.

The dashboard has the same adapter selector. It runs PEAK in normal/active
mode with no host acceptance filters, and disables the sample-point selector
because PCAN-Basic's classic-CAN bitrate presets define that timing.

The equivalent PEAK command-line preflight is:

```powershell
a2c-firmware-update update firmware.binenc --adapter peak --channel 0x51 --bitrate 250000 --dry-run
```

Run **Check Sensor** before the first update on a customer PC. The full raw CAN
log identifies the adapter backend, API version, channel, bitrate, and all
unfiltered transmit/receive traffic. See
[PEAK hardware validation](docs/PEAK_HARDWARE_VALIDATION.md) for the first-PC
test procedure.

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

The offline test suite does not open a physical CAN channel. It includes a fake
PCAN-Basic DLL that verifies PEAK channel discovery, classic-CAN bitrate setup,
framing, timestamps, error handling, and channel release. PEAK firmware updates
have also been validated on customer hardware; dashboard coverage is tested
offline and uses the same PCAN transport.

## Periodic fusion debugging (0.2.0-beta.8)

In the dashboard's **Periodic messages** tab, choose a command and subcommand
from the dropdowns, enable the slot, set its interval, and select **Apply all
slots (volatile)**. For sensor-fusion debugging, use normal/fusion mode with
the following configuration, including the three fusion diagnostics supported
as periodic messages by firmware 0x0136 or newer:

| Task | Command | Subcommand | Interval | Output |
| --- | --- | --- | --- | --- |
| 1 | `0x0A` | `0x00` | 40 ms | Acceleration XYZ |
| 2 | `0x0D` | `0x00` | 40 ms | Gyroscope XYZ |
| 3 | `0x0B` | `0x02` | 40 ms | Fusion Euler XYZ, 0.01 degree/count |
| 4 | `0x0A` | `0x01` | 40 ms | Fusion-derived linear acceleration XYZ |
| 5 | `0xE4` | `0x00` | 1000 ms | Measured internal sample rate |
| 6 | `0x18` | `0x12` | 100 ms | Fusion engine, active output, ready/startup and rejection flags |
| 7 | `0x18` | `0x13` | 100 ms | Acceleration error, recovery trigger and startup resets |
| 8 | `0x18` | `0x14` | 500 ms | Fallback, ignored-acceleration and startup sample counters |

Linear acceleration and measured sample rate were already selectable in earlier
app versions; their dropdown labels are clearer in beta.7. Linear acceleration
now also has decoded XYZ values and a separate stream-statistics entry in the
CAN monitor.

The **Fusion** label above the live plots shows a simple state string, for
example `Fusion: Running — x-io Fusion · acceleration rejected`. It distinguishes
startup, not ready, running, InvenSense fallback, acceleration rejection and
recovery. Hover over it for the decoded summary, rejection metrics and counters
received so far. Missing status is marked stale after three seconds, or three
configured summary intervals if longer. Disconnecting, changing target or
changing sensor mode clears the previous state. Vibration and standby modes
show fusion as inactive.

Enabling **Poll sensor at 10 Hz** also requests the fusion summary once per
second in fusion mode when the reported firmware is 0x0135 or newer. With
polling off, the label listens to incoming periodic replies without sending
additional requests. Firmware 0x0135 supports direct diagnostic queries;
periodic fusion diagnostics require 0x0136 or newer.

The periodic dropdown covers the firmware's supported commands: `0x09`, `0x0A`,
`0x0B`, `0x0C`, `0x0D`, `0x0F`, `0x10`, `0x18` (only `0x12`–`0x14`) and
`0xE4`. Other `0x18` calibration subcommands and `0xC6` yaw reference remain
query-only. `0x0F` resets min/max values rather than producing measurement data.
Diagnostic error is shown in degrees, recovery trigger as a percentage, and
wire counters saturate at 65535.

Changes remain in RAM until explicitly saved to sensor flash. Building or
starting the app does not enable slots on a connected sensor automatically.

## Build the Windows customer package

Install the optional build dependency and run the packaging script:

```powershell
python -m pip install ".[build]"
.\packaging\build_windows.ps1
```

The script creates a versioned ZIP and SHA-256 file below `dist`. The ZIP
contains the graphical dashboard, graphical updater, updater command-line
helper, shared Qt runtime files, customer instructions, and license notices.
It does not include vendor CAN drivers or firmware. To add a supplied encrypted
firmware image to a private customer package, pass
`-FirmwarePackage path\to\firmware.binenc`.

## License

Copyright 2026 A2C.

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
PySide6, Qt, Kvaser CANlib, and PEAK PCAN-Basic remain subject to their own licenses; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

[pcan-basic]: https://www.peak-system.com/products/software/development-packages/pcan-basic/
