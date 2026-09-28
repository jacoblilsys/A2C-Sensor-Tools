# Third-party notices

## Qt for Python / PySide6

The graphical applications use Qt for Python (PySide6), Copyright The Qt
Company Ltd. and contributors. Qt for Python is available under LGPLv3/GPLv3
and commercial licensing options. Distribution of A2C application binaries
must comply with the selected Qt licensing terms.

- <https://doc.qt.io/qtforpython-6/>
- <https://doc.qt.io/qtforpython-6/licenses.html>

## Kvaser CANlib

The applications call Kvaser's installed `canlib32.dll` at runtime. Kvaser
CANlib is proprietary software distributed under Kvaser's own license and is
not part of this Apache-2.0 repository.

Customers must install the Kvaser CANlib SDK and the appropriate Kvaser device
driver:

- <https://kvaser.com/canlib-sdk/>
- <https://kvaser.com/canlib-webhelp/section_install_windows.htm>

Refer to the license included with the installed Kvaser CANlib SDK before
redistributing Kvaser runtime files.

## PEAK-System PCAN-Basic

The firmware updater can call PEAK-System's installed `PCANBasic.dll` at
runtime. PCAN-Basic is proprietary software distributed under PEAK-System's
own license and is not part of this Apache-2.0 repository.

Customers using a PEAK PCAN interface must install PEAK's Windows device driver
and PCAN-Basic API:

- <https://www.peak-system.com/products/software/development-packages/pcan-basic/>
- <https://www.peak-system.com/support/eula/>

The PCAN-Basic package grants use in connection with original PEAK-System or
PEAK-System OEM hardware. Consult the current PEAK EULA and package ReadMe
before redistributing PEAK runtime files.
