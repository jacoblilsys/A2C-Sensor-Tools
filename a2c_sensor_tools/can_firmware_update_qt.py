#!/usr/bin/env python3
# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Qt 6 front end for the A2C-IMU V2 CAN firmware updater."""

from __future__ import annotations

import re
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QProcess, QSettings, Qt, QUrl
from PySide6.QtGui import QCloseEvent, QDesktopServices, QFont
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

try:
    from .a2c_app_support import APP_VERSION, install_help_menu
    from .firmware_update_profiles import (
        FirmwareUpdateError,
        detect_firmware_package,
    )
    from .can_sensor_monitor import (
        KvaserCanlib,
        KvaserError,
        SAMPLE_POINT_SEGMENTS,
        SUPPORTED_BITRATES,
    )
    from .pcan_basic import PcanBasic, PcanError
except ImportError:  # Direct execution from the Tools directory.
    from a2c_app_support import APP_VERSION, install_help_menu  # type: ignore
    from firmware_update_profiles import (  # type: ignore
        FirmwareUpdateError,
        detect_firmware_package,
    )
    from can_sensor_monitor import (  # type: ignore
        KvaserCanlib,
        KvaserError,
        SAMPLE_POINT_SEGMENTS,
        SUPPORTED_BITRATES,
    )
    from pcan_basic import PcanBasic, PcanError  # type: ignore


APP_TITLE = "A2C Sensor Firmware Updater"
DEFAULT_REQUEST_ID = 0x3E8
DEFAULT_PROGRESS_MAXIMUM = 100
VERIFICATION_INCONCLUSIVE_EXIT_CODE = 2
PROGRESS_PATTERN = re.compile(r"Programming\s+(\d+)/(\d+)\s+pages")
ADAPTER_NAMES = {
    "kvaser": "Kvaser CANlib",
    "peak": "PEAK PCAN-Basic",
}


def default_can_log_path(image: Path, stamp: str) -> Path:
    root = os.environ.get("LOCALAPPDATA")
    log_directory = (
        Path(root) / "A2C" / "SensorTools" / "logs"
        if root
        else Path.home() / ".a2c-sensor-tools" / "logs"
    )
    return log_directory / f"{image.stem}_CAN_QT_{stamp}.log"


def parse_standard_can_id(text: str, *, optional: bool = False) -> Optional[int]:
    """Parse decimal, 0x-prefixed, or bare hexadecimal 11-bit CAN IDs."""
    value_text = text.strip()
    if not value_text:
        if optional:
            return None
        raise ValueError("CAN ID is required")

    if value_text.lower().startswith("0x"):
        digits = value_text[2:]
        base = 16
    elif any(character in "abcdefABCDEF" for character in value_text):
        digits = value_text
        base = 16
    else:
        digits = value_text
        base = 10

    if not digits:
        raise ValueError(f"invalid CAN ID: {text!r}")
    try:
        value = int(digits, base)
    except ValueError as exc:
        raise ValueError(f"invalid CAN ID: {text!r}") from exc
    if not 0 <= value <= 0x7FF:
        raise ValueError(f"CAN ID 0x{value:X} is outside the standard 11-bit range")
    return value


def parse_can_id(text: str, *, optional: bool = False) -> Optional[int]:
    """Parse a standard or extended CAN identifier."""
    value_text = text.strip()
    if not value_text:
        if optional:
            return None
        raise ValueError("CAN ID is required")
    if value_text.lower().startswith("0x"):
        digits, base = value_text[2:], 16
    elif any(character in "abcdefABCDEF" for character in value_text):
        digits, base = value_text, 16
    else:
        digits, base = value_text, 10
    if not digits:
        raise ValueError(f"invalid CAN ID: {text!r}")
    try:
        value = int(digits, base)
    except ValueError as exc:
        raise ValueError(f"invalid CAN ID: {text!r}") from exc
    if not 0 <= value <= 0x1FFFFFFF:
        raise ValueError(f"CAN ID 0x{value:X} is outside the 29-bit range")
    return value


def format_bitrate(bitrate: int) -> str:
    if bitrate >= 1_000_000:
        return f"{bitrate / 1_000_000:g} Mbit/s"
    return f"{bitrate // 1000} kbit/s"


@dataclass(frozen=True)
class UpdateConfiguration:
    image: Path
    adapter: str
    channel: int
    bitrate: int
    sample_point: str
    request_id: int
    update_id: int
    response_id: Optional[int]
    allow_same_or_older: bool
    can_log: Path


def build_update_arguments(config: UpdateConfiguration, *, dry_run: bool) -> list[str]:
    arguments = [
        "update",
        str(config.image),
        "--adapter",
        config.adapter,
        "--channel",
        str(config.channel),
        "--bitrate",
        str(config.bitrate),
        "--sample-point",
        config.sample_point,
        "--request-id",
        f"0x{config.request_id:X}",
        "--update-id",
        f"0x{config.update_id:X}",
        "--can-log",
        str(config.can_log),
    ]
    if config.response_id is not None:
        arguments.extend(("--response-id", f"0x{config.response_id:X}"))
    if config.allow_same_or_older:
        arguments.append("--allow-same-or-older")
    if dry_run:
        arguments.append("--dry-run")
    else:
        arguments.append("--yes")
    return arguments


class UpdateOutputParser:
    """Split QProcess output on both newlines and CLI carriage-return progress."""

    def __init__(self) -> None:
        self._partial = ""

    def feed(self, text: str) -> list[str]:
        combined = self._partial + text
        parts = re.split(r"\r\n|\r|\n", combined)
        if combined.endswith(("\r", "\n")):
            self._partial = ""
        else:
            self._partial = parts.pop()
        return [part.strip() for part in parts if part.strip()]

    def flush(self) -> list[str]:
        if not self._partial.strip():
            self._partial = ""
            return []
        line = self._partial.strip()
        self._partial = ""
        return [line]


class FirmwareUpdaterWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.setMinimumSize(820, 650)

        # Retain the original key so existing IMU updater settings migrate.
        self._settings = QSettings("A2C", "IMUFirmwareUpdater")
        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._read_process_output)
        self._process.finished.connect(self._process_finished)
        self._process.errorOccurred.connect(self._process_error)
        self._output_parser = UpdateOutputParser()
        self._inspection = None
        self._running = False
        self._dry_run = False
        self._next_progress_log_percent = 0
        self._can_log_path: Optional[Path] = None
        self._last_process_error: Optional[str] = None
        self._channel_by_adapter: dict[str, int] = {}

        self._release_checker = install_help_menu(self, APP_TITLE)
        self._build_ui()
        self._restore_settings()
        self.refresh_channels(show_dialog=False)
        self.inspect_selected_image(show_dialog=False)

    def _build_ui(self) -> None:
        central = QWidget(self)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(10)

        firmware_group = QGroupBox("Firmware package")
        firmware_layout = QGridLayout(firmware_group)
        self.image_edit = QLineEdit()
        self.image_edit.setPlaceholderText("Select an encrypted .binenc firmware package")
        self.image_edit.editingFinished.connect(self.inspect_selected_image)
        self.browse_button = QPushButton("Browse…")
        self.browse_button.clicked.connect(self.browse_image)
        self.image_summary = QLabel("No firmware selected")
        self.image_summary.setWordWrap(True)
        self.image_summary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        firmware_layout.addWidget(self.image_edit, 0, 0)
        firmware_layout.addWidget(self.browse_button, 0, 1)
        firmware_layout.addWidget(self.image_summary, 1, 0, 1, 2)
        outer.addWidget(firmware_group)

        can_group = QGroupBox("CAN interface")
        can_grid = QGridLayout(can_group)
        self.adapter_combo = QComboBox()
        for adapter, display_name in ADAPTER_NAMES.items():
            self.adapter_combo.addItem(display_name, adapter)
        self.adapter_combo.currentIndexChanged.connect(self._adapter_changed)
        self.channel_combo = QComboBox()
        self.channel_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.channel_combo.currentIndexChanged.connect(self._channel_changed)
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_channels)
        self.bitrate_combo = QComboBox()
        for bitrate in SUPPORTED_BITRATES:
            self.bitrate_combo.addItem(format_bitrate(bitrate), bitrate)
        self.sample_point_combo = QComboBox()
        for sample_point in SAMPLE_POINT_SEGMENTS:
            self.sample_point_combo.addItem(f"{sample_point}%", sample_point)

        can_grid.addWidget(QLabel("Adapter"), 0, 0)
        can_grid.addWidget(self.adapter_combo, 0, 1, 1, 3)
        can_grid.addWidget(QLabel("Channel"), 1, 0)
        can_grid.addWidget(self.channel_combo, 1, 1, 1, 3)
        can_grid.addWidget(self.refresh_button, 1, 4)
        can_grid.addWidget(QLabel("Baud rate"), 2, 0)
        can_grid.addWidget(self.bitrate_combo, 2, 1)
        can_grid.addWidget(QLabel("Sample point"), 2, 2)
        can_grid.addWidget(self.sample_point_combo, 2, 3)
        can_grid.setColumnStretch(1, 1)
        can_grid.setColumnStretch(3, 1)
        outer.addWidget(can_group)

        protocol_group = QGroupBox("Sensor and bootloader IDs")
        protocol_form = QFormLayout(protocol_group)
        self.request_id_edit = QLineEdit("0x3E8")
        self.update_id_edit = QLineEdit("0x3E8")
        self.response_id_edit = QLineEdit()
        self.response_id_edit.setPlaceholderText("Auto-discover")
        for edit in (self.request_id_edit, self.update_id_edit, self.response_id_edit):
            edit.setMaximumWidth(180)
        self.request_id_edit.setToolTip("Standard 11-bit ID accepted by the running sensor")
        self.update_id_edit.setToolTip(
            "Bootloader control ID. Keep 0x3E8 unless that ID is already allowed by the sensor hardware filter."
        )
        self.response_id_edit.setToolTip(
            "Optional expected sensor transmit ID; normally discovered automatically."
        )
        protocol_form.addRow("Sensor request ID", self.request_id_edit)
        protocol_form.addRow("Bootloader/update ID", self.update_id_edit)
        protocol_form.addRow("Expected response ID", self.response_id_edit)
        outer.addWidget(protocol_group)

        options_row = QHBoxLayout()
        self.allow_older_checkbox = QCheckBox("Allow same or older firmware")
        options_row.addWidget(self.allow_older_checkbox)
        options_row.addStretch(1)
        outer.addLayout(options_row)

        action_row = QHBoxLayout()
        self.preflight_button = QPushButton("Check Sensor")
        self.preflight_button.clicked.connect(self.start_preflight)
        self.start_button = QPushButton("Start Update")
        self.start_button.setDefault(True)
        self.start_button.clicked.connect(self.start_update)
        self.abort_button = QPushButton("Abort")
        self.abort_button.setEnabled(False)
        self.abort_button.clicked.connect(self.abort_update)
        action_row.addWidget(self.preflight_button)
        action_row.addWidget(self.start_button)
        action_row.addWidget(self.abort_button)
        action_row.addStretch(1)
        outer.addLayout(action_row)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, DEFAULT_PROGRESS_MAXIMUM)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Ready")
        outer.addWidget(self.progress_bar)

        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("Update log"))
        log_header.addStretch(1)
        self.open_can_log_button = QPushButton("Open raw CAN log")
        self.open_can_log_button.setEnabled(False)
        self.open_can_log_button.clicked.connect(self.open_can_log)
        self.save_log_button = QPushButton("Save visible log…")
        self.save_log_button.clicked.connect(self.save_visible_log)
        self.clear_log_button = QPushButton("Clear")
        log_header.addWidget(self.open_can_log_button)
        log_header.addWidget(self.save_log_button)
        log_header.addWidget(self.clear_log_button)
        outer.addLayout(log_header)

        self.log_edit = QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.clear_log_button.clicked.connect(self.log_edit.clear)
        self.log_edit.document().setMaximumBlockCount(5000)
        fixed_font = QFont("Consolas")
        fixed_font.setStyleHint(QFont.StyleHint.Monospace)
        self.log_edit.setFont(fixed_font)
        outer.addWidget(self.log_edit, 1)

        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar(self))
        self.statusBar().showMessage("Select a firmware package and CAN channel")

        self._input_widgets = (
            self.image_edit,
            self.browse_button,
            self.adapter_combo,
            self.channel_combo,
            self.refresh_button,
            self.bitrate_combo,
            self.sample_point_combo,
            self.request_id_edit,
            self.update_id_edit,
            self.response_id_edit,
            self.allow_older_checkbox,
        )

    def _restore_settings(self) -> None:
        self.image_edit.setText(str(self._settings.value("image", "")))
        self.request_id_edit.setText(str(self._settings.value("request_id", "0x3E8")))
        self.update_id_edit.setText(str(self._settings.value("update_id", "0x3E8")))
        self.response_id_edit.setText(str(self._settings.value("response_id", "")))

        adapter = str(self._settings.value("adapter", "kvaser"))
        index = self.adapter_combo.findData(adapter)
        if index >= 0:
            self.adapter_combo.setCurrentIndex(index)

        bitrate = int(self._settings.value("bitrate", 250_000))
        index = self.bitrate_combo.findData(bitrate)
        if index >= 0:
            self.bitrate_combo.setCurrentIndex(index)
        sample_point = str(self._settings.value("sample_point", "87.5"))
        index = self.sample_point_combo.findData(sample_point)
        if index >= 0:
            self.sample_point_combo.setCurrentIndex(index)
        self.allow_older_checkbox.setChecked(
            str(self._settings.value("allow_older", "false")).lower() == "true"
        )
        geometry = self._settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)

    def _save_settings(self) -> None:
        self._settings.setValue("image", self.image_edit.text().strip())
        adapter = str(self.adapter_combo.currentData())
        self._settings.setValue("adapter", adapter)
        channel = self.channel_combo.currentData()
        if channel is not None:
            self._channel_by_adapter[adapter] = int(channel)
        for saved_adapter, saved_channel in self._channel_by_adapter.items():
            self._settings.setValue(f"channel_{saved_adapter}", saved_channel)
        self._settings.setValue("bitrate", self.bitrate_combo.currentData())
        self._settings.setValue("sample_point", self.sample_point_combo.currentData())
        self._settings.setValue("request_id", self.request_id_edit.text().strip())
        self._settings.setValue("update_id", self.update_id_edit.text().strip())
        self._settings.setValue("response_id", self.response_id_edit.text().strip())
        self._settings.setValue("allow_older", self.allow_older_checkbox.isChecked())
        self._settings.setValue("geometry", self.saveGeometry())

    def _append_log(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_edit.appendPlainText(f"[{stamp}] {message}")
        scroll_bar = self.log_edit.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def browse_image(self) -> None:
        current = Path(self.image_edit.text().strip()) if self.image_edit.text().strip() else Path.cwd()
        start_directory = current.parent if current.suffix else current
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select encrypted firmware package",
            str(start_directory),
            "Encrypted firmware (*.binenc);;Binary images (*.bin);;All files (*)",
        )
        if filename:
            self.image_edit.setText(filename)
            self.inspect_selected_image()

    def inspect_selected_image(self, show_dialog: bool = True) -> None:
        self._inspection = None
        text = self.image_edit.text().strip()
        if not text:
            self.image_summary.setText("No firmware selected")
            self.image_summary.setStyleSheet("")
            self._apply_profile_ui()
            self._update_start_enabled()
            return

        path = Path(text).expanduser()
        try:
            inspection = detect_firmware_package(path.read_bytes())
        except (OSError, ValueError, FirmwareUpdateError) as exc:
            self.image_summary.setText(f"Invalid package: {exc}")
            self.image_summary.setStyleSheet("color: #b00020;")
            if show_dialog:
                self._append_log(f"Firmware validation failed: {exc}")
            self._apply_profile_ui()
            self._update_start_enabled()
            return

        self._inspection = inspection
        self.image_edit.setText(str(path.resolve()))
        crc_text = f"transport CRC 0x{inspection.transport_crc:08X}"
        self.image_summary.setText(
            f"{inspection.profile.display_name}  ·  firmware 0x{inspection.program.version:X}  ·  "
            f"sensor 0x{inspection.program.sensor_type:08X}  ·  "
            f"hardware 0x{inspection.program.hardware:08X}  ·  "
            f"{inspection.profile.page_count} × {inspection.profile.page_size}-byte pages  ·  {crc_text}"
        )
        self.image_summary.setStyleSheet("color: #176b2c;")
        self._apply_profile_ui()
        if show_dialog:
            self._append_log(
                f"Validated {path.name} as {inspection.profile.display_name}: "
                f"firmware 0x{inspection.program.version:X}, "
                f"hardware 0x{inspection.program.hardware:08X}"
            )
        self._update_start_enabled()

    def _apply_profile_ui(self) -> None:
        is_kvaser = self.adapter_combo.currentData() == "kvaser"
        self.sample_point_combo.setEnabled(not self._running and is_kvaser)
        self.sample_point_combo.setToolTip(
            "Kvaser bit timing sample point"
            if is_kvaser
            else "PEAK classic-CAN bitrate presets define the bit timing and sample point."
        )

    def _adapter_changed(self, _index: int) -> None:
        self._apply_profile_ui()
        self.refresh_channels(show_dialog=False)

    def _channel_changed(self, _index: int) -> None:
        channel = self.channel_combo.currentData()
        adapter = self.adapter_combo.currentData()
        if adapter is not None and channel is not None:
            self._channel_by_adapter[str(adapter)] = int(channel)
        self._update_start_enabled()

    def refresh_channels(self, show_dialog: bool = True) -> None:
        adapter = str(self.adapter_combo.currentData())
        adapter_name = ADAPTER_NAMES[adapter]
        previous = self._channel_by_adapter.get(adapter)
        if previous is None:
            default_channel = 0 if adapter == "kvaser" else 0x51
            saved = self._settings.value(f"channel_{adapter}")
            if saved is None and adapter == "kvaser":
                saved = self._settings.value("channel", default_channel)
            previous = int(saved if saved is not None else default_channel)
        self.channel_combo.clear()
        try:
            api = PcanBasic() if adapter == "peak" else KvaserCanlib()
            channels = api.list_channels()
        except (KvaserError, PcanError, OSError) as exc:
            self.channel_combo.addItem(f"{adapter_name} unavailable", None)
            self._append_log(str(exc))
            if show_dialog:
                QMessageBox.warning(self, APP_TITLE, str(exc))
            self._update_start_enabled()
            return

        for channel in channels:
            description = f" — {channel.description}" if channel.description else ""
            self.channel_combo.addItem(
                f"{channel.name}{description}", channel.number
            )
        index = self.channel_combo.findData(previous)
        self.channel_combo.setCurrentIndex(index if index >= 0 else 0)
        if not channels:
            self.channel_combo.addItem(f"No {adapter_name} channels found", None)
        elif show_dialog:
            self._append_log(f"Found {len(channels)} {adapter_name} channel(s)")
        self._update_start_enabled()

    def _configuration(self) -> UpdateConfiguration:
        self.inspect_selected_image(show_dialog=False)
        if self._inspection is None:
            raise ValueError("select a valid encrypted firmware package")
        channel = self.channel_combo.currentData()
        if channel is None:
            raise ValueError("select an available CAN channel")

        request_id = parse_standard_can_id(self.request_id_edit.text())
        update_id = parse_standard_can_id(self.update_id_edit.text())
        response_id = parse_standard_can_id(self.response_id_edit.text(), optional=True)
        image = Path(self.image_edit.text().strip()).resolve()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        can_log = default_can_log_path(image, stamp)
        return UpdateConfiguration(
            image=image,
            adapter=str(self.adapter_combo.currentData()),
            channel=int(channel),
            bitrate=int(self.bitrate_combo.currentData()),
            sample_point=str(self.sample_point_combo.currentData()),
            request_id=int(request_id),
            update_id=int(update_id),
            response_id=response_id,
            allow_same_or_older=self.allow_older_checkbox.isChecked(),
            can_log=can_log,
        )

    def _update_start_enabled(self) -> None:
        ready = (
            not self._running
            and self._inspection is not None
            and self.channel_combo.currentData() is not None
        )
        self.start_button.setEnabled(ready)
        self.preflight_button.setEnabled(ready)

    def start_preflight(self) -> None:
        try:
            config = self._configuration()
        except ValueError as exc:
            QMessageBox.warning(self, APP_TITLE, str(exc))
            return
        self._start_process(config, dry_run=True)

    def start_update(self) -> None:
        try:
            config = self._configuration()
        except ValueError as exc:
            QMessageBox.warning(self, APP_TITLE, str(exc))
            return

        assert self._inspection is not None
        profile = self._inspection.profile
        answer = QMessageBox.question(
            self,
            "Confirm firmware update",
            f"Update {profile.display_name} to firmware 0x{self._inspection.program.version:X}?\n\n"
            f"This will erase and rewrite {profile.erase_description}. "
            "Keep power and CAN connected until completion.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._start_process(config, dry_run=False)

    def _start_process(self, config: UpdateConfiguration, *, dry_run: bool) -> None:
        assert self._inspection is not None
        profile = self._inspection.profile
        self._save_settings()
        self._dry_run = dry_run
        self._last_process_error = None
        self._can_log_path = config.can_log
        self.open_can_log_button.setEnabled(False)
        self._output_parser = UpdateOutputParser()
        self._next_progress_log_percent = 0
        self.progress_bar.setRange(0, 0 if dry_run else profile.page_count)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Checking sensor…" if dry_run else "Starting update…")

        update_arguments = build_update_arguments(config, dry_run=dry_run)
        cli_path = Path(__file__).with_name(profile.cli_filename).resolve()
        if getattr(sys, "frozen", False):
            executable_directory = Path(sys.executable).resolve().parent
            helper_candidates = (
                executable_directory / "_internal" / "a2c-firmware-update.exe",
                executable_directory / "a2c-firmware-update.exe",
            )
            program = next(
                (candidate for candidate in helper_candidates if candidate.exists()),
                helper_candidates[0],
            )
            arguments = update_arguments
            working_directory = program.parent
            if not program.exists():
                QMessageBox.critical(
                    self,
                    APP_TITLE,
                    f"The packaged updater helper is missing:\n{program}",
                )
                return
        else:
            program = Path(sys.executable)
            arguments = ["-u", str(cli_path), *update_arguments]
            working_directory = cli_path.parent.parent
        mode = "Preflight" if dry_run else "Update"
        self._append_log(
            f"{mode} started for {profile.display_name}: {ADAPTER_NAMES[config.adapter]}, "
            f"channel {config.channel}, "
            f"{format_bitrate(config.bitrate)}, "
            f"request 0x{config.request_id:X}, update 0x{config.update_id:X}"
        )
        self._append_log(f"Raw CAN traffic: {config.can_log}")
        self.statusBar().showMessage(f"{mode} running")
        self._set_running(True)
        self._process.setWorkingDirectory(str(working_directory))
        self._process.start(str(program), arguments)

    def _set_running(self, running: bool) -> None:
        self._running = running
        for widget in self._input_widgets:
            widget.setEnabled(not running)
        self._apply_profile_ui()
        self.abort_button.setEnabled(running)
        self._update_start_enabled()

    def _read_process_output(self) -> None:
        raw = bytes(self._process.readAllStandardOutput())
        for line in self._output_parser.feed(raw.decode("utf-8", errors="replace")):
            self._handle_process_line(line)

    def _handle_process_line(self, line: str) -> None:
        if line.startswith("ERROR:"):
            self._last_process_error = line.removeprefix("ERROR:").strip()
        match = PROGRESS_PATTERN.search(line)
        if match:
            current = int(match.group(1))
            total = int(match.group(2))
            self.progress_bar.setRange(0, total)
            self.progress_bar.setValue(current)
            self.progress_bar.setFormat(f"Programming {current}/{total} pages — %p%")
            percent = current * 100 // total
            if current == 1 or current == total or percent >= self._next_progress_log_percent:
                self._append_log(line)
                self._next_progress_log_percent = ((percent // 5) + 1) * 5
            return
        self._append_log(line)

    def _process_finished(self, exit_code: int, exit_status: QProcess.ExitStatus) -> None:
        if self._process.bytesAvailable():
            self._read_process_output()
        for line in self._output_parser.flush():
            self._handle_process_line(line)
        success = exit_status == QProcess.ExitStatus.NormalExit and exit_code == 0
        verification_inconclusive = (
            exit_status == QProcess.ExitStatus.NormalExit
            and exit_code == VERIFICATION_INCONCLUSIVE_EXIT_CODE
            and not self._dry_run
        )
        self._set_running(False)
        if self._can_log_path is not None and self._can_log_path.exists():
            self.open_can_log_button.setEnabled(True)

        if success:
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(100)
            self.progress_bar.setFormat("Preflight passed" if self._dry_run else "Update complete")
            message = "Sensor check completed successfully" if self._dry_run else "Firmware update completed successfully"
            self._append_log(message)
            self.statusBar().showMessage(message)
            if not self._dry_run:
                QMessageBox.information(self, APP_TITLE, message)
        elif verification_inconclusive:
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(100)
            self.progress_bar.setFormat("Programmed — verify sensor")
            message = (
                "Firmware programming and transport CRC verification completed, but "
                "the restarted application could not be verified. Power-cycle the "
                "sensor, then use Check Sensor before attempting another update."
            )
            self._append_log(message)
            self.statusBar().showMessage("Programmed; final verification inconclusive")
            QMessageBox.warning(self, APP_TITLE, message)
        else:
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(0)
            self.progress_bar.setFormat("Failed — see log")
            exit_message = f"Updater stopped with exit code {exit_code}"
            self._append_log(exit_message)
            if self._last_process_error:
                message = f"The operation could not continue:\n\n{self._last_process_error}"
                self.statusBar().showMessage(self._last_process_error)
            else:
                message = exit_message
                self.statusBar().showMessage(exit_message)
            QMessageBox.critical(self, APP_TITLE, message)

    def _process_error(self, error: QProcess.ProcessError) -> None:
        self._append_log(f"Could not run updater: {self._process.errorString()} ({error.name})")
        if error == QProcess.ProcessError.FailedToStart:
            self._set_running(False)
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(0)
            self.progress_bar.setFormat("Could not start updater")
            self.statusBar().showMessage("Could not start updater")
            QMessageBox.critical(self, APP_TITLE, self._process.errorString())

    def abort_update(self) -> None:
        if not self._running:
            return
        answer = QMessageBox.warning(
            self,
            "Abort update?",
            "Aborting can leave the sensor in bootloader mode. It can normally be recovered "
            "by restarting the update or power-cycling it. Abort now?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer == QMessageBox.StandardButton.Yes:
            self._append_log("Abort requested by user")
            self._process.kill()

    def open_can_log(self) -> None:
        if self._can_log_path is not None and self._can_log_path.exists():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self._can_log_path)))

    def save_visible_log(self) -> None:
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save update log",
            str(Path.cwd() / "A2C_firmware_update.log"),
            "Log files (*.log);;Text files (*.txt);;All files (*)",
        )
        if not filename:
            return
        try:
            Path(filename).write_text(self.log_edit.toPlainText() + "\n", encoding="utf-8")
        except OSError as exc:
            QMessageBox.critical(self, APP_TITLE, f"Could not save log: {exc}")

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._running:
            answer = QMessageBox.warning(
                self,
                "Update still running",
                "Closing now will abort the update and may leave the sensor in bootloader mode. Close?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self._process.kill()
            self._process.waitForFinished(2000)
        self._save_settings()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    app.setApplicationVersion(APP_VERSION)
    app.setOrganizationName("A2C")
    window = FirmwareUpdaterWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
