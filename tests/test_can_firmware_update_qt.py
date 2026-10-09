import os
import subprocess
import sys
import unittest
from unittest.mock import patch
from pathlib import Path

try:
    from PySide6.QtCore import QProcess
    from PySide6.QtWidgets import QApplication, QMessageBox
    from a2c_sensor_tools.can_firmware_update_qt import (
        FirmwareUpdaterWindow,
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
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_can_id_accepts_expected_notation(self) -> None:
        self.assertEqual(parse_standard_can_id("1000"), 0x3E8)
        self.assertEqual(parse_standard_can_id("0x3E8"), 0x3E8)
        self.assertEqual(parse_standard_can_id("3E8"), 0x3E8)
        self.assertIsNone(parse_standard_can_id("", optional=True))

    def test_module_can_be_loaded_from_direct_script_directory(self) -> None:
        script = (
            Path(__file__).resolve().parents[1]
            / "a2c_sensor_tools"
            / "can_firmware_update_qt.py"
        )
        environment = os.environ.copy()
        environment.setdefault("QT_QPA_PLATFORM", "offscreen")
        probe = (
            "import runpy; "
            f"runpy.run_path({str(script)!r}, run_name='direct_launch_probe')"
        )
        result = subprocess.run(
            [sys.executable, "-c", probe],
            cwd=script.parent,
            env=environment,
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_can_id_rejects_out_of_range_value(self) -> None:
        with self.assertRaisesRegex(ValueError, "11-bit"):
            parse_standard_can_id("0x800")

    def test_response_id_accepts_j1939_extended_identifier(self) -> None:
        self.assertEqual(parse_can_id("0x18FF5012"), 0x18FF5012)

    def test_cli_arguments_include_selected_bus_and_ids(self) -> None:
        config = UpdateConfiguration(
            image=Path("firmware.binenc"),
            adapter="kvaser",
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
        self.assertEqual(arguments[arguments.index("--adapter") + 1], "kvaser")
        self.assertIn("500000", arguments)
        self.assertIn("0x125", arguments)
        self.assertIn("--allow-same-or-older", arguments)
        self.assertNotIn("--standalone", arguments)

    def test_cli_arguments_include_peak_adapter_and_handle(self) -> None:
        config = UpdateConfiguration(
            image=Path("firmware.binenc"),
            adapter="peak",
            channel=0x51,
            bitrate=250_000,
            sample_point="87.5",
            request_id=0x3E8,
            update_id=0x3E8,
            response_id=None,
            allow_same_or_older=False,
            can_log=Path("traffic.log"),
        )
        arguments = build_update_arguments(config, dry_run=True)
        self.assertEqual(arguments[arguments.index("--adapter") + 1], "peak")
        self.assertEqual(arguments[arguments.index("--channel") + 1], "81")
        self.assertIn("--dry-run", arguments)

    def test_peak_selection_disables_kvaser_sample_point(self) -> None:
        with patch.object(FirmwareUpdaterWindow, "refresh_channels"):
            window = FirmwareUpdaterWindow()
            peak_index = window.adapter_combo.findData("peak")
            kvaser_index = window.adapter_combo.findData("kvaser")
            window.adapter_combo.setCurrentIndex(peak_index)
            self.assertFalse(window.sample_point_combo.isEnabled())
            window.adapter_combo.setCurrentIndex(kvaser_index)
            self.assertTrue(window.sample_point_combo.isEnabled())
            window.close()

    def test_inconclusive_verification_is_a_warning_not_failed_programming(self) -> None:
        with patch.object(FirmwareUpdaterWindow, "refresh_channels"):
            window = FirmwareUpdaterWindow()
        window._dry_run = False
        with (
            patch.object(QMessageBox, "warning") as warning,
            patch.object(QMessageBox, "critical") as critical,
        ):
            window._process_finished(2, QProcess.ExitStatus.NormalExit)

        self.assertEqual(window.progress_bar.value(), 100)
        self.assertIn("verify sensor", window.progress_bar.format().lower())
        warning.assert_called_once()
        critical.assert_not_called()
        window.close()

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
