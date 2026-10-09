# A2C-IMU V2 firmware updater

The graphical updater accepts a complete encrypted `.binenc` firmware package
supplied by A2C and transfers it over classic CAN using a Kvaser or PEAK
interface.

## Safety checks

Before any erase command is sent, the updater:

1. Validates the firmware package size and embedded metadata.
2. Rejects plaintext firmware images.
3. Queries the connected sensor type, hardware revision, and firmware version.
4. Rejects a sensor/package type or hardware mismatch.
5. Rejects the same or an older firmware version unless the advanced override
   is selected explicitly.

During an update it checks every transferred page, verifies the complete
transport CRC, restarts the sensor, reads the installed version and CRC, and
reports success only when they match the selected package.

Immediately before entering the bootloader, the updater measures current CAN
traffic and disables all sensor periodic-message slots in volatile RAM. Saved
periodic settings are not changed and are loaded again when the sensor restarts.
This prevents an old sensor transmit backlog or high-rate periodic stream from
competing with the bootloader transfer.

## Normal workflow

1. Connect only the sensor being updated.
2. Start **A2C Sensor Firmware Updater**.
3. Select the encrypted A2C `.binenc` package.
4. Select the CAN adapter, channel, bitrate, and request ID. The sample-point
   option applies to Kvaser only.
5. Leave the response ID empty unless the bus contains more than one possible
   response.
6. Select **Check Sensor**.
7. Confirm the detected sensor and firmware information.
8. Select **Start Update**.
9. Keep CAN and sensor power connected until installed firmware and CRC
   verification succeeds.

The CAN channel always operates in normal/active mode. The updater installs
no host acceptance filter and records all observed CAN traffic during the
operation. By default, logs are written below
`%LOCALAPPDATA%\A2C\SensorTools\logs`.

For PEAK adapters, recoverable PCAN bus-warning, bus-heavy, overrun, and
bus-off status frames are retained in the raw CAN log instead of immediately
aborting the update. PEAK automatic bus-off reset is enabled when the channel
is opened.

If programming and the transport CRC succeed but the restarted application
cannot be verified, the command exits with code `2` and the GUI reports
**Programmed — verify sensor**. This is not reported as an erase/programming
failure. Power-cycle the sensor and use **Check Sensor** before attempting
another update. A normal confirmed update exits with `0`; a failure before or
during programming exits with `1`.

## Command-line use

Inspect a package without opening a CAN channel:

```powershell
a2c-firmware-update inspect A2C_IMU_L433VCI_0x0132.binenc
```

Run the mandatory live preflight without entering the bootloader or erasing:

```powershell
a2c-firmware-update update A2C_IMU_L433VCI_0x0132.binenc --dry-run
```

Perform a confirmed update:

```powershell
a2c-firmware-update update A2C_IMU_L433VCI_0x0132.binenc `
  --channel 0 --bitrate 250000 --sample-point 87.5 `
  --request-id 0x3E8 --update-id 0x3E8 --yes
```

Use `--response-id` only when automatic discovery is ambiguous. Use
`--allow-same-or-older` only for an intentional reinstall or downgrade.

The public updater cannot build or encrypt firmware packages and cannot perform
factory empty-flash recovery.
