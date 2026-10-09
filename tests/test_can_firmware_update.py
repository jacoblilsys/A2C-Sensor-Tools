import io
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path

from a2c_sensor_tools.can_firmware_update import (
    A2C_IMU_SENSOR_TYPE,
    A2C_IMU_V2_HARDWARE,
    BOOTLOADER_END,
    BOOTLOADER_START,
    FLASH_SIZE,
    FirmwareUpdater,
    FirmwareUpdateError,
    FirmwareVerificationInconclusive,
    INFO_SIZE,
    PROGRAM_START,
    TrafficLoggingChannel,
    build_parser,
    default_can_log_path,
    describe_version_relation,
    inspect_package,
    main,
    stm32_crc,
    validate_update_version_policy,
)
from a2c_sensor_tools.can_sensor_monitor import CanFrame
from a2c_sensor_tools.pcan_basic import PcanError


def metadata(version: int, crc: int) -> bytes:
    return b"".join(
        value.to_bytes(4, "little")
        for value in (A2C_IMU_SENSOR_TYPE, A2C_IMU_V2_HARDWARE, version, crc)
    )


def plaintext_image() -> bytes:
    image = bytearray(FLASH_SIZE)
    image[BOOTLOADER_START : BOOTLOADER_START + 8] = (
        (0x20010000).to_bytes(4, "little") + (0x08001009).to_bytes(4, "little")
    )
    image[PROGRAM_START : PROGRAM_START + 8] = (
        (0x20010000).to_bytes(4, "little") + (0x08007009).to_bytes(4, "little")
    )
    image[BOOTLOADER_END - INFO_SIZE : BOOTLOADER_END] = metadata(0x0F, 0x12345678)
    image[FLASH_SIZE - INFO_SIZE :] = metadata(0x125, 0x89ABCDEF)
    return bytes(image)


class CommandLineTests(unittest.TestCase):
    def test_peak_channel_accepts_hex_handle(self) -> None:
        args = build_parser().parse_args(
            [
                "update",
                "firmware.binenc",
                "--adapter",
                "peak",
                "--channel",
                "0x51",
                "--dry-run",
            ]
        )
        self.assertEqual(args.adapter, "peak")
        self.assertEqual(args.channel, 0x51)

    def test_version_relation_describes_equal_newer_and_older_sensors(self) -> None:
        self.assertEqual(
            describe_version_relation(0x132, 0x132),
            "sensor already runs the selected firmware 0x132",
        )
        self.assertIn(
            "sensor firmware 0x133 is newer",
            describe_version_relation(0x132, 0x133),
        )
        self.assertIn(
            "selected package 0x132 is newer",
            describe_version_relation(0x132, 0x131),
        )

    def test_same_version_is_allowed_for_preflight_but_blocked_for_update(self) -> None:
        validate_update_version_policy(
            0x132,
            0x132,
            allow_same_or_older=False,
            dry_run=True,
        )
        with self.assertRaisesRegex(FirmwareUpdateError, "not newer"):
            validate_update_version_policy(
                0x132,
                0x132,
                allow_same_or_older=False,
                dry_run=False,
            )
        validate_update_version_policy(
            0x132,
            0x132,
            allow_same_or_older=True,
            dry_run=False,
        )


class Stm32CrcTests(unittest.TestCase):
    def test_known_words_from_legacy_csharp_tool(self) -> None:
        self.assertEqual(stm32_crc(bytes.fromhex("00000000")), 0xC704DD7B)
        self.assertEqual(stm32_crc(bytes.fromhex("FFFFFFFF")), 0x00000000)
        self.assertEqual(stm32_crc(bytes.fromhex("AAAAAAAA")), 0x42FC4B29)

    def test_crc_requires_complete_words(self) -> None:
        with self.assertRaises(ValueError):
            stm32_crc(b"\x00\x01")

    def test_default_log_uses_local_application_data(self) -> None:
        with patch.dict("os.environ", {"LOCALAPPDATA": r"C:\Local"}, clear=False):
            path = default_can_log_path(Path("firmware.binenc"), "stamp", 123)
        self.assertEqual(
            path,
            Path(r"C:\Local\A2C\SensorTools\logs\firmware_CAN_stamp_123.log"),
        )


class PackageInspectionTests(unittest.TestCase):
    def test_plaintext_image_is_identified_but_not_updateable(self) -> None:
        image = plaintext_image()
        result = inspect_package(image, require_encrypted=False)
        self.assertFalse(result.encrypted)
        self.assertEqual(result.program.version, 0x125)
        with self.assertRaisesRegex(FirmwareUpdateError, "plaintext application"):
            inspect_package(image, require_encrypted=True)

    def test_raw_cubeide_image_with_empty_bootloader_can_be_inspected(self) -> None:
        image = bytearray(plaintext_image())
        image[BOOTLOADER_START:BOOTLOADER_END] = bytes(
            BOOTLOADER_END - BOOTLOADER_START
        )
        result = inspect_package(bytes(image), require_encrypted=False)
        self.assertFalse(result.encrypted)
        self.assertEqual(result.bootloader.sensor_type, 0)

    def test_wrong_program_hardware_is_rejected(self) -> None:
        image = bytearray(plaintext_image())
        image[FLASH_SIZE - 12 : FLASH_SIZE - 8] = (0x101).to_bytes(4, "little")
        with self.assertRaisesRegex(FirmwareUpdateError, "expected A2C_IMU_V2"):
            inspect_package(bytes(image), require_encrypted=False)


class _EraseChannel:
    def __init__(self) -> None:
        self.writes = []
        self.responses = [CanFrame(0x125, bytes((0xEA, 0x01, *b"EOE")))]

    def write(self, can_id: int, data: bytes, _extended: bool = False) -> None:
        self.writes.append((can_id, data))

    def read(self, _timeout_ms: int):
        return self.responses.pop(0) if self.responses else None


class FirmwareUpdaterProtocolTests(unittest.TestCase):
    def test_erase_matches_legacy_csharp_frame(self) -> None:
        channel = _EraseChannel()
        updater = FirmwareUpdater(channel, 0x3E8, 0x3E8, 0x125, 2.0)
        updater.erase(0.1)
        self.assertEqual(channel.writes, [(0x3E8, bytes((0xEE, 0x01, *b"ERASE")))])

    def test_traffic_logger_records_all_tx_and_rx_ids(self) -> None:
        channel = _EraseChannel()
        stream = io.StringIO()
        traced = TrafficLoggingChannel(channel, stream)
        traced.write(0x3E8, b"\x01\x02")
        frame = traced.read(1)
        self.assertIsNotNone(frame)
        log = stream.getvalue()
        self.assertIn("TX STD ID=0x3E8 DLC=2", log)
        self.assertIn("RX STD ID=0x125 DLC=5", log)

    def test_periodic_slots_are_disabled_only_in_volatile_ram(self) -> None:
        channel = _EraseChannel()
        channel.responses.clear()
        updater = FirmwareUpdater(channel, 0x3E8, 0x3E8, 0x125, 0.0)
        with patch.object(updater, "drain", return_value=17) as drain:
            drained = updater.disable_periodic_tasks()

        self.assertEqual(drained, 17)
        self.assertEqual(len(channel.writes), 8)
        for task, (can_id, payload) in enumerate(channel.writes, start=1):
            self.assertEqual(can_id, 0x3E8)
            self.assertEqual(payload, bytes((0x52, task, 0, 0, 0, 0, 0, 0)))
        drain.assert_called_once_with(quiet_ms=25, maximum_ms=1000)

    def test_reboot_poll_retries_transient_adapter_error(self) -> None:
        class RecoveryChannel:
            def __init__(self) -> None:
                self.writes = 0
                self.response_ready = False

            def write(self, _can_id: int, _data: bytes, _extended: bool = False) -> None:
                self.writes += 1
                if self.writes == 1:
                    raise PcanError("temporary bus warning")
                self.response_ready = True

            def read(self, _timeout_ms: int):
                if not self.response_ready:
                    return None
                self.response_ready = False
                return CanFrame(0x123, bytes((0xEF, 0x04, 0, 0, 0x01, 0x32)))

        channel = RecoveryChannel()
        updater = FirmwareUpdater(channel, 0x3E8, 0x3E8, None, 0.0)
        with patch("a2c_sensor_tools.can_firmware_update.time.sleep"):
            response_id = updater.wait_for_firmware(0x132, 0.5)

        self.assertEqual(response_id, 0x123)
        self.assertEqual(channel.writes, 2)

    def test_information_query_retries_transient_adapter_error(self) -> None:
        channel = _EraseChannel()
        channel.responses.clear()
        updater = FirmwareUpdater(channel, 0x3E8, 0x3E8, 0x123, 0.0)
        with patch.object(
            updater,
            "query_information",
            side_effect=[PcanError("temporary bus warning"), (0x89ABCDEF, 0x123)],
        ) as query, patch("a2c_sensor_tools.can_firmware_update.time.sleep"):
            result = updater.query_information_with_retry(0x07, 0.5)

        self.assertEqual(result, (0x89ABCDEF, 0x123))
        self.assertEqual(query.call_count, 2)

    def test_inconclusive_verification_has_distinct_exit_code(self) -> None:
        def handler(_args):
            raise FirmwareVerificationInconclusive("final check unavailable")

        parser = unittest.mock.Mock()
        parser.parse_args.return_value = SimpleNamespace(handler=handler)
        with patch(
            "a2c_sensor_tools.can_firmware_update.build_parser", return_value=parser
        ):
            self.assertEqual(main([]), 2)

if __name__ == "__main__":
    unittest.main()
