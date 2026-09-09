import unittest

from a2c_sensor_tools.can_firmware_update import (
    A2C_IMU_SENSOR_TYPE,
    A2C_IMU_V2_HARDWARE,
    BOOTLOADER_END as IMU_BOOTLOADER_END,
    FLASH_SIZE as IMU_FLASH_SIZE,
    INFO_SIZE,
)
from a2c_sensor_tools.firmware_update_profiles import (
    FirmwareUpdateError,
    detect_firmware_package,
)


def _metadata(sensor: int, hardware: int, version: int, crc: int) -> bytes:
    return b"".join(value.to_bytes(4, "little") for value in (sensor, hardware, version, crc))


def imu_package() -> bytes:
    image = bytearray(b"\xA5" * IMU_FLASH_SIZE)
    image[IMU_BOOTLOADER_END - INFO_SIZE : IMU_BOOTLOADER_END] = _metadata(
        A2C_IMU_SENSOR_TYPE, A2C_IMU_V2_HARDWARE, 0x10, 0x11111111
    )
    image[-INFO_SIZE:] = _metadata(
        A2C_IMU_SENSOR_TYPE, A2C_IMU_V2_HARDWARE, 0x130, 0x22222222
    )
    return bytes(image)


class FirmwareProfileTests(unittest.TestCase):
    def test_detects_imu_without_changing_existing_inspection(self) -> None:
        result = detect_firmware_package(imu_package())
        self.assertEqual(result.profile.key, "imu")
        self.assertEqual(result.profile.page_count, 800)
        self.assertEqual(result.program.version, 0x130)

    def test_rejects_unknown_package_size(self) -> None:
        with self.assertRaisesRegex(FirmwareUpdateError, "A2C-IMU V2 encrypted packages"):
            detect_firmware_package(b"bad")


if __name__ == "__main__":
    unittest.main()
