A2C Sensor Firmware Updater - PEAK PCAN test build
==================================================

This package is self-contained. Python, PySide6, and Qt do not need to be
installed.

Required PEAK software
----------------------
Install the current PEAK-System Windows driver and PCAN-Basic API before
starting the updater:

https://www.peak-system.com/products/software/development-packages/pcan-basic/

The PEAK installation supplies PCANBasic.dll and the hardware driver. They are
not included in this A2C package.

First test
----------
1. Extract the complete ZIP file to a normal local folder. Do not run the
   application from inside the ZIP file.
2. Connect the PEAK PCAN-USB adapter and only the A2C sensor being tested.
3. Start A2C-Sensor-Firmware-Updater.exe.
4. Select "PEAK PCAN-Basic" and press Refresh.
5. Select the PCAN_USBBUS channel and the sensor's current CAN bitrate.
6. Select the encrypted A2C .binenc firmware package supplied separately.
7. Press Check Sensor first. Do not start an update unless the detected sensor,
   hardware, and firmware information are correct.
8. Keep CAN and sensor power connected throughout the update.

Diagnostics
-----------
Raw CAN logs are stored below:

%LOCALAPPDATA%\A2C\SensorTools\logs

For remote support, retain the visible application log and the complete raw
CAN log. The raw log records the PCAN-Basic API version, selected channel,
bitrate, and all unfiltered CAN traffic.

If the application reports "Programmed - verify sensor", the firmware transfer
and transport CRC succeeded but the restarted application was not confirmed.
Power-cycle the sensor and press Check Sensor before attempting another update.
The updater temporarily disables periodic output only in RAM during an update;
saved sensor settings are unchanged.

This PEAK build is pending validation with physical PEAK hardware. The
executables are not code-signed, so Windows may show a SmartScreen warning.

Only A2C-Sensor-Firmware-Updater.exe is intended to be started by the user.
Files in the _internal folder are application components and should not be
opened or moved individually.
