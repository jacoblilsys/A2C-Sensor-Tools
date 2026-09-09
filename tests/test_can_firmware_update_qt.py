import unittest
from unittest.mock import patch
from pathlib import Path

try:
    from a2c_sensor_tools.can_firmware_update_qt import (
        UpdateConfiguration,
        UpdateOutputParser,
        build_update_arguments,
        default_can_log_path,
        parse_can_id,
        parse_standard_can_id,
    )
except ImportError as exc:  # Qt is optional for command-line-only installations.
    QT_IMPORT_ERROR = exc
else:
    QT_IMPORT_ERROR = None


@unittest.skipIf(QT_IMPORT_ERROR is not None, f"Qt 6 unavailable: {QT_IMPORT_ERROR}")
class FirmwareUpdaterQtTests(unittest.TestCase):
    def test_can_id_accepts_expected_notation(self) -> None:
        self.assertEqual(parse_standard_can_id("1000"), 0x3E8)
        self.assertEqual(parse_standard_can_id("0x3E8"), 0x3E8)
        self.assertEqual(parse_standard_can_id("3E8"), 0x3E8)
        self.assertIsNone(parse_standard_can_id("", optional=True))

    def test_can_id_rejects_out_of_range_value(self) -> None:
        with self.assertRaisesRegex(ValueError, "11-bit"):
            parse_standard_can_id("0x800")

    def test_response_id_accepts_j1939_extended_identifier(self) -> None:
        self.assertEqual(parse_can_id("0x18FF5012"), 0x18FF5012)

    def test_cli_arguments_include_selected_bus_and_ids(self) -> None:
        config = UpdateConfiguration(
            image=Path("firmware.binenc"),
            channel=2,
            bitrate=500_000,
            sample_point="75",
            request_id=0x3E8,
            update_id=0x3E8,
            response_id=0x125,
            allow_same_or_older=True,
            can_log=Path("traffic.log"),
        )
        arguments = build_update_arguments(config, dry_run=False)
        self.assertIn("--yes", arguments)
        self.assertIn("500000", arguments)
        self.assertIn("0x125", arguments)
        self.assertIn("--allow-same-or-older", arguments)
        self.assertNotIn("--standalone", arguments)

    def test_progress_parser_handles_carriage_return_chunks(self) -> None:
        parser = UpdateOutputParser()
        self.assertEqual(parser.feed("Starting\nProgramming   1/800"), ["Starting"])
        self.assertEqual(
            parser.feed(" pages\rProgramming   2/800 pages\r"),
            ["Programming   1/800 pages", "Programming   2/800 pages"],
        )
        self.assertEqual(parser.flush(), [])

    def test_default_log_uses_local_application_data(self) -> None:
        with patch.dict("os.environ", {"LOCALAPPDATA": r"C:\Local"}, clear=False):
            path = default_can_log_path(Path("firmware.binenc"), "20260909_120000")
        self.assertEqual(
            path,
            Path(r"C:\Local\A2C\SensorTools\logs\firmware_CAN_QT_20260909_120000.log"),
        )


if __name__ == "__main__":
    unittest.main()
