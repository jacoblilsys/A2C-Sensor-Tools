# PEAK PCAN-USB hardware validation

The PEAK backend is covered by offline DLL-boundary tests, but it must be
validated with a physical adapter before being declared production-tested.

## Customer-PC preparation

1. Install PEAK's current Windows device driver and PCAN-Basic API package.
2. Confirm the adapter is visible in PEAK-Settings or PCAN-View.
3. Close other CAN applications for the first test.
4. Connect only the A2C sensor being tested and use the correct bus
   termination.
5. Keep a known-good Kvaser updater available as the fallback.

## Safe first test

1. Start **A2C Sensor Firmware Updater**.
2. Select **PEAK PCAN-Basic** and press **Refresh**.
3. Record the displayed `PCAN_USBBUS` channel and adapter/API messages.
4. Select the sensor's current bitrate and IDs.
5. Run **Check Sensor**. Do not start an update until firmware, hardware, and
   sensor type are all read successfully.
6. Open the raw CAN log and confirm the expected request and response IDs.
7. Start an update with stable sensor power, then verify the final firmware
   version and CRC reported by the application.

## Information to retain for remote debugging

- A2C Sensor Tools version and commit
- Windows version and architecture
- PEAK adapter model and driver version
- PCAN-Basic API version printed by the updater
- selected channel, bitrate, and CAN IDs
- complete raw CAN traffic log
- updater's visible log or screenshot of the failure

The updater opens PCAN-Basic in normal/active classic-CAN mode, configures no
host acceptance filters, and calls `CAN_Uninitialize` when the preflight or
update exits, including error paths handled by the updater process.
