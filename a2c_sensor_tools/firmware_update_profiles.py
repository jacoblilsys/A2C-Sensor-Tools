# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Package detection and updater profiles shared by the A2C Qt front end."""

from __future__ import annotations

from dataclasses import dataclass

try:
    from . import can_firmware_update as imu
except ImportError:  # Direct execution from the package directory.
    import can_firmware_update as imu  # type: ignore


FirmwareUpdateError = imu.FirmwareUpdateError


@dataclass(frozen=True)
class SensorProfile:
    key: str
    display_name: str
    cli_filename: str
    package_size: int
    page_size: int
    page_count: int
    erase_description: str


IMU_PROFILE = SensorProfile(
    key="imu",
    display_name="A2C-IMU V2",
    cli_filename="can_firmware_update.py",
    package_size=imu.FLASH_SIZE,
    page_size=imu.PAGE_SIZE,
    page_count=imu.PAGE_COUNT,
    erase_description="the sensor's external firmware area",
)


@dataclass(frozen=True)
class UnifiedPackageInspection:
    profile: SensorProfile
    details: object

    @property
    def program(self):
        return self.details.program

    @property
    def transport_crc(self) -> int:
        return self.details.program_transport_crc


def detect_firmware_package(image: bytes) -> UnifiedPackageInspection:
    """Validate a customer-distributable A2C-IMU V2 updater package."""
    if len(image) == IMU_PROFILE.package_size:
        return UnifiedPackageInspection(IMU_PROFILE, imu.inspect_package(image, require_encrypted=True))
    raise FirmwareUpdateError(
        f"image is {len(image)} bytes; A2C-IMU V2 encrypted packages are "
        f"{IMU_PROFILE.package_size} bytes"
    )
