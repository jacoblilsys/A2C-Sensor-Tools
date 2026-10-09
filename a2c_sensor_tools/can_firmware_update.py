#!/usr/bin/env python3
# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Inspect and update A2C-IMU V2 firmware through a supported CAN adapter.

The CAN transport uses the adapter vendor's installed Windows DLL. Firmware
package creation and empty-flash factory recovery intentionally remain in
A2C's private tooling.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import sys
import time
from typing import Callable, Optional, TextIO

try:
    from .can_sensor_monitor import (
        CanFrame,
        KvaserCanlib,
        KvaserError,
        SAMPLE_POINT_SEGMENTS,
        SUPPORTED_BITRATES,
        parse_integer,
    )
    from .pcan_basic import PcanBasic, PcanError
except ImportError:  # Direct execution from the Tools directory.
    from can_sensor_monitor import (  # type: ignore
        CanFrame,
        KvaserCanlib,
        KvaserError,
        SAMPLE_POINT_SEGMENTS,
        SUPPORTED_BITRATES,
        parse_integer,
    )
    from pcan_basic import PcanBasic, PcanError  # type: ignore


FLASH_SIZE = 0x32000
PAGE_SIZE = 0x100
PAGE_COUNT = FLASH_SIZE // PAGE_SIZE
BOOTLOADER_START = 0x1000
BOOTLOADER_END = 0x7000
PROGRAM_START = BOOTLOADER_END
INFO_SIZE = 16

A2C_IMU_SENSOR_TYPE = 0x00050001
A2C_IMU_V2_HARDWARE = 0x00000202

CMD_ENTER = 0xE0
CMD_PAGE = 0xE1
CMD_PAGE_DONE = 0xE2
CMD_PAGE_END = 0xE4
CMD_ERASE = 0xEE
CMD_ERASE_DONE = 0xEA
CMD_IMAGE_CRC = 0xF5
CMD_EXIT = 0xFE
CMD_INFO = 0xEF
CMD_SET_PERIODIC_TASK = 0x52

PAGE_PROGRAMMED = 0xA7
PERIODIC_TASK_COUNT = 8
TRAFFIC_SAMPLE_MS = 250
ESTIMATED_CLASSIC_CAN_BITS_PER_FRAME = 125
HIGH_TRAFFIC_UTILIZATION = 0.50
VERIFICATION_INCONCLUSIVE_EXIT_CODE = 2


def default_can_log_path(image: Path, stamp: str, process_id: int) -> Path:
    root = os.environ.get("LOCALAPPDATA")
    log_directory = (
        Path(root) / "A2C" / "SensorTools" / "logs"
        if root
        else Path.home() / ".a2c-sensor-tools" / "logs"
    )
    return log_directory / f"{image.stem}_CAN_{stamp}_{process_id}.log"


class FirmwareUpdateError(RuntimeError):
    """The image or bootloader protocol failed a safety check."""


class FirmwareVerificationInconclusive(FirmwareUpdateError):
    """Programming passed, but the restarted application could not be verified."""


@dataclass(frozen=True)
class FirmwareMetadata:
    sensor_type: int
    hardware: int
    version: int
    crc: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "FirmwareMetadata":
        if len(data) != INFO_SIZE:
            raise ValueError("firmware metadata must be exactly 16 bytes")
        values = tuple(
            int.from_bytes(data[offset : offset + 4], "little")
            for offset in range(0, INFO_SIZE, 4)
        )
        return cls(*values)

    def describe(self) -> str:
        return (
            f"sensor=0x{self.sensor_type:08X}, hardware=0x{self.hardware:08X}, "
            f"version=0x{self.version:X}, embedded CRC=0x{self.crc:08X}"
        )


@dataclass(frozen=True)
class PackageInspection:
    bootloader: FirmwareMetadata
    program: FirmwareMetadata
    program_transport_crc: int
    full_transport_crc: int
    encrypted: bool


def stm32_crc(data: bytes) -> int:
    """Return the legacy STM32 hardware CRC used by this bootloader.

    Bytes are consumed as big-endian 32-bit words.  The CRC is non-reflected,
    initialized to 0xFFFFFFFF, uses polynomial 0x04C11DB7, and has no final XOR.
    """
    if not data or len(data) % 4:
        raise ValueError("STM32 CRC input must be non-empty and a multiple of 4 bytes")

    crc = 0xFFFFFFFF
    for offset in range(0, len(data), 4):
        crc ^= int.from_bytes(data[offset : offset + 4], "big")
        for _ in range(32):
            crc = ((crc << 1) ^ (0x04C11DB7 if crc & 0x80000000 else 0)) & 0xFFFFFFFF
    return crc


def _looks_like_vector_table(data: bytes, offset: int) -> bool:
    if len(data) < offset + 8:
        return False
    stack = int.from_bytes(data[offset : offset + 4], "little")
    reset = int.from_bytes(data[offset + 4 : offset + 8], "little")
    stack_ok = 0x10000000 <= stack < 0x30000000
    reset_address = reset & ~1
    reset_ok = 0x08000000 <= reset_address < 0x08100000 and bool(reset & 1)
    return stack_ok and reset_ok


def _check_metadata(metadata: FirmwareMetadata, section: str) -> None:
    if metadata.sensor_type != A2C_IMU_SENSOR_TYPE:
        raise FirmwareUpdateError(
            f"{section} sensor type is 0x{metadata.sensor_type:08X}; "
            f"expected A2C-IMU 0x{A2C_IMU_SENSOR_TYPE:08X}"
        )
    if metadata.hardware != A2C_IMU_V2_HARDWARE:
        raise FirmwareUpdateError(
            f"{section} hardware is 0x{metadata.hardware:08X}; "
            f"expected A2C_IMU_V2 0x{A2C_IMU_V2_HARDWARE:08X}"
        )
    if metadata.version in (0, 0xFFFFFFFF, 0x01020305):
        raise FirmwareUpdateError(
            f"{section} has placeholder/invalid version 0x{metadata.version:08X}"
        )


def inspect_package(image: bytes, require_encrypted: bool = True) -> PackageInspection:
    if len(image) != FLASH_SIZE:
        raise FirmwareUpdateError(
            f"image is {len(image)} bytes; A2C_IMU_V2 packages must be {FLASH_SIZE} bytes"
        )

    bootloader = FirmwareMetadata.from_bytes(
        image[BOOTLOADER_END - INFO_SIZE : BOOTLOADER_END]
    )
    program = FirmwareMetadata.from_bytes(image[FLASH_SIZE - INFO_SIZE :])
    _check_metadata(program, "program")

    encrypted = not _looks_like_vector_table(image, PROGRAM_START)
    bootloader_present = any(image[BOOTLOADER_START:BOOTLOADER_END])
    if require_encrypted or bootloader_present:
        _check_metadata(bootloader, "bootloader")
        if (
            bootloader.sensor_type != program.sensor_type
            or bootloader.hardware != program.hardware
        ):
            raise FirmwareUpdateError("bootloader and program metadata do not target the same sensor")
    if require_encrypted and not encrypted:
        raise FirmwareUpdateError(
            "image contains a plaintext application vector table. Use the package command "
            "to create a .binenc file; do not send CubeIDE's raw Release .bin."
        )

    return PackageInspection(
        bootloader=bootloader,
        program=program,
        program_transport_crc=stm32_crc(image[PROGRAM_START:FLASH_SIZE]),
        full_transport_crc=stm32_crc(image[BOOTLOADER_START:FLASH_SIZE]),
        encrypted=encrypted,
    )


def _format_frame(frame: CanFrame) -> str:
    payload = " ".join(f"{value:02X}" for value in frame.data)
    return f"ID 0x{frame.can_id:X}, DLC {len(frame.data)}, [{payload}]"


class TrafficLoggingChannel:
    """Log every transmitted and received frame without applying CAN filters."""

    def __init__(self, channel, stream: TextIO) -> None:
        self._channel = channel
        self._stream = stream
        self._started = time.monotonic()
        self._records_since_flush = 0

    def _record(
        self,
        direction: str,
        can_id: int,
        data: bytes,
        *,
        extended: bool = False,
        flags: int = 0,
        adapter_timestamp_ms: Optional[int] = None,
    ) -> None:
        elapsed = time.monotonic() - self._started
        frame_type = "EXT" if extended else "STD"
        payload = " ".join(f"{value:02X}" for value in data)
        adapter_time = (
            f" adapter_ms={adapter_timestamp_ms}" if adapter_timestamp_ms is not None else ""
        )
        self._stream.write(
            f"+{elapsed:012.6f} {direction} {frame_type} ID=0x{can_id:X} "
            f"DLC={len(data)} FLAGS=0x{flags:X}{adapter_time} DATA=[{payload}]\n"
        )
        self._records_since_flush += 1
        if self._records_since_flush >= 64:
            self._stream.flush()
            self._records_since_flush = 0

    def write(self, can_id: int, data: bytes, extended: bool = False) -> None:
        self._channel.write(can_id, data, extended)
        self._record("TX", can_id, data, extended=extended)

    def read(self, timeout_ms: int = 100) -> Optional[CanFrame]:
        frame = self._channel.read(timeout_ms)
        if frame is not None:
            self._record(
                "RX",
                frame.can_id,
                frame.data,
                extended=frame.is_extended,
                flags=frame.flags,
                adapter_timestamp_ms=frame.timestamp_ms,
            )
        return frame

    def flush_log(self) -> None:
        self._stream.flush()
        self._records_since_flush = 0


class FirmwareUpdater:
    def __init__(
        self,
        channel,
        request_id: int,
        update_id: int,
        response_id: Optional[int],
        frame_delay_ms: float,
    ) -> None:
        self.channel = channel
        self.request_id = request_id
        self.update_id = update_id
        self.response_id = response_id
        self.frame_delay = frame_delay_ms / 1000.0

    @property
    def control_id(self) -> int:
        return self.update_id

    def drain(self, quiet_ms: int = 50, maximum_ms: int = 500) -> int:
        count = 0
        deadline = time.monotonic() + maximum_ms / 1000.0
        while time.monotonic() < deadline:
            frame = self.channel.read(quiet_ms)
            if frame is None:
                break
            count += 1
        return count

    def observe_traffic(self, duration_ms: int = TRAFFIC_SAMPLE_MS) -> tuple[int, int, float]:
        """Count all frames and frames from this sensor over a fixed interval."""
        if duration_ms <= 0:
            raise ValueError("traffic observation duration must be positive")
        started = time.monotonic()
        deadline = started + duration_ms / 1000.0
        total = 0
        sensor = 0
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            frame = self.channel.read(min(10, max(1, int(remaining * 1000))))
            if frame is None:
                continue
            total += 1
            if self.response_id is not None and frame.can_id == self.response_id:
                sensor += 1
        return total, sensor, max(time.monotonic() - started, 0.001)

    def disable_periodic_tasks(self, task_count: int = PERIODIC_TASK_COUNT) -> int:
        """Disable application periodic slots in RAM before entering bootloader."""
        for task in range(1, task_count + 1):
            self.channel.write(
                self.request_id,
                bytes((CMD_SET_PERIODIC_TASK, task, 0, 0, 0, 0, 0, 0)),
            )
            if self.frame_delay:
                time.sleep(min(self.frame_delay, 0.020))
        return self.drain(quiet_ms=25, maximum_ms=1000)

    def _wait_for(
        self,
        predicate: Callable[[CanFrame], bool],
        timeout: float,
        operation: str,
        nack_command: Optional[int] = None,
        any_response_id: bool = False,
        trace_label: Optional[str] = None,
        trace_start: Optional[float] = None,
    ) -> CanFrame:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise FirmwareUpdateError(f"timeout waiting for {operation}")
            frame = self.channel.read(min(100, max(1, int(remaining * 1000))))
            if frame is not None and trace_label is not None:
                elapsed = time.monotonic() - (trace_start or time.monotonic())
                print(f"{trace_label} RX +{elapsed:0.3f}s: {_format_frame(frame)}")
            if frame is None or frame.is_error or frame.is_extended:
                continue
            if (
                not any_response_id
                and self.response_id is not None
                and frame.can_id != self.response_id
            ):
                continue
            if (
                nack_command is not None
                and len(frame.data) >= 2
                and frame.data[0] == CMD_EXIT
                and frame.data[1] == nack_command
            ):
                error = int.from_bytes(frame.data[2:6], "big") if len(frame.data) >= 6 else 0
                raise FirmwareUpdateError(
                    f"bootloader rejected command 0x{nack_command:02X}: "
                    f"error 0x{error:08X} ({_format_frame(frame)})"
                )
            if predicate(frame):
                return frame

    def query_information(self, item: int, timeout: float = 1.0) -> tuple[int, int]:
        self.drain(quiet_ms=10, maximum_ms=50)
        self.channel.write(self.request_id, bytes((CMD_INFO, item, 0, 0, 0, 0, 0, 0)))
        frame = self._wait_for(
            lambda rx: len(rx.data) >= 6
            and rx.data[0] == CMD_INFO
            and rx.data[1] == item,
            timeout,
            f"sensor-information item 0x{item:02X}",
            any_response_id=self.response_id is None,
        )
        if self.response_id is None:
            self.response_id = frame.can_id
        return int.from_bytes(frame.data[2:6], "big"), frame.can_id

    def query_information_with_retry(
        self, item: int, timeout: float, attempt_timeout: float = 0.5
    ) -> tuple[int, int]:
        deadline = time.monotonic() + timeout
        last_communication_error: Optional[Exception] = None
        while time.monotonic() < deadline:
            try:
                return self.query_information(
                    item,
                    min(attempt_timeout, max(0.05, deadline - time.monotonic())),
                )
            except (FirmwareUpdateError, KvaserError, PcanError) as exc:
                last_communication_error = exc
                time.sleep(min(0.10, max(0.0, deadline - time.monotonic())))
        detail = (
            f"; last communication error: {last_communication_error}"
            if last_communication_error is not None
            else ""
        )
        raise FirmwareUpdateError(
            f"sensor-information item 0x{item:02X} was not received within "
            f"{timeout:g} seconds{detail}"
        )

    def enter(self, timeout: float) -> int:
        self.drain()
        payload = bytes(
            (
                CMD_ENTER,
                ord("E"),
                ord("N"),
                ord("T"),
                ord("B"),
                (self.update_id >> 8) & 0xFF,
                self.update_id & 0xFF,
            )
        )
        self.channel.write(self.request_id, payload)
        welcome = self._wait_for(
            lambda rx: rx.data == bytes((0xDA, *b"WELCOME")),
            timeout,
            "bootloader WELCOME",
            any_response_id=self.response_id is None,
        )
        if self.response_id is not None and welcome.can_id != self.response_id:
            raise FirmwareUpdateError(
                f"WELCOME came from ID 0x{welcome.can_id:X}, expected 0x{self.response_id:X}"
            )
        self.response_id = welcome.can_id
        return welcome.can_id

    def erase(self, timeout: float) -> None:
        # Match the deployed C# updater exactly.  The current bootloader only
        # validates "ERA", but older tools have always sent the complete word.
        payload = bytes((CMD_ERASE, 0x01, *b"ERASE"))
        started = time.monotonic()
        print(
            f"Erase TX: ID 0x{self.control_id:X}, DLC {len(payload)}, "
            f"[{' '.join(f'{value:02X}' for value in payload)}]"
        )
        self.channel.write(self.control_id, payload)
        self._wait_for(
            lambda rx: rx.data == bytes((CMD_ERASE_DONE, 0x01, ord("E"), ord("O"), ord("E"))),
            timeout,
            "external-flash erase completion",
            nack_command=CMD_ERASE,
            trace_label="Erase",
            trace_start=started,
        )

    def send_page(self, page_number: int, page: bytes, timeout: float) -> int:
        if len(page) != PAGE_SIZE:
            raise ValueError("page must be exactly 256 bytes")
        expected_crc = stm32_crc(page)
        self.channel.write(
            self.control_id,
            bytes((CMD_PAGE, 0x01, page_number >> 8, page_number & 0xFF, 0, 0, 0)),
        )
        if self.frame_delay:
            time.sleep(self.frame_delay)
        for offset in range(0, PAGE_SIZE, 8):
            self.channel.write(self.control_id, page[offset : offset + 8])
            if self.frame_delay:
                time.sleep(self.frame_delay)
        self.channel.write(
            self.control_id,
            bytes(
                (
                    CMD_PAGE_END,
                    page_number >> 8,
                    page_number & 0xFF,
                    *expected_crc.to_bytes(4, "big"),
                )
            ),
        )

        response = self._wait_for(
            lambda rx: len(rx.data) == 8
            and rx.data[0] == CMD_PAGE_DONE
            and int.from_bytes(rx.data[2:4], "big") == page_number,
            timeout,
            f"page {page_number} acknowledgement",
            nack_command=CMD_PAGE,
        )
        status = response.data[1]
        reported_crc = int.from_bytes(response.data[4:8], "big")
        if status != PAGE_PROGRAMMED:
            raise FirmwareUpdateError(
                f"page {page_number} returned status 0x{status:02X}, expected 0x{PAGE_PROGRAMMED:02X}"
            )
        if reported_crc != expected_crc:
            raise FirmwareUpdateError(
                f"page {page_number} CRC mismatch: sensor 0x{reported_crc:08X}, "
                f"host 0x{expected_crc:08X}"
            )
        return expected_crc

    def verify_transport_crc(
        self, inspection: PackageInspection, timeout: float
    ) -> str:
        candidates = [
            ("program", inspection.program_transport_crc),
            ("bootloader+program", inspection.full_transport_crc),
        ]

        for attempt, (scope, expected) in enumerate(candidates):
            self.channel.write(
                self.control_id,
                bytes((CMD_IMAGE_CRC, 0x01, *expected.to_bytes(4, "big"), 0)),
            )
            response = self._wait_for(
                lambda rx: len(rx.data) == 6 and rx.data[0] == CMD_IMAGE_CRC,
                timeout,
                "whole-image CRC",
                nack_command=CMD_IMAGE_CRC,
            )
            reported = int.from_bytes(response.data[2:6], "big")
            if reported == expected:
                return scope
            matching = next((name for name, value in candidates if value == reported), None)
            if matching is None:
                raise FirmwareUpdateError(
                    f"external-flash CRC 0x{reported:08X} matches neither supported scope "
                    f"(program 0x{inspection.program_transport_crc:08X}, "
                    f"bootloader+program 0x{inspection.full_transport_crc:08X})"
                )
            if attempt == len(candidates) - 1:
                break
        raise FirmwareUpdateError("unable to confirm the bootloader's transport CRC scope")

    def exit(self) -> None:
        self.channel.write(self.control_id, bytes((CMD_EXIT, 0x01, *b"EXITB")))

    def wait_for_firmware(self, version: int, timeout: float) -> int:
        deadline = time.monotonic() + timeout
        last_communication_error: Optional[Exception] = None
        while time.monotonic() < deadline:
            try:
                self.channel.write(
                    self.request_id, bytes((CMD_INFO, 0x04, 0, 0, 0, 0, 0, 0))
                )
                frame = self._wait_for(
                    lambda rx: len(rx.data) >= 6
                    and rx.data[0] == CMD_INFO
                    and rx.data[1] == 0x04,
                    min(0.5, max(0.05, deadline - time.monotonic())),
                    "application firmware version",
                    any_response_id=True,
                )
            except (FirmwareUpdateError, KvaserError, PcanError) as exc:
                last_communication_error = exc
                time.sleep(min(0.10, max(0.0, deadline - time.monotonic())))
                continue
            reported = int.from_bytes(frame.data[2:6], "big")
            if reported == version:
                self.response_id = frame.can_id
                return frame.can_id
            time.sleep(0.25)
        detail = (
            f"; last communication error: {last_communication_error}"
            if last_communication_error is not None
            else ""
        )
        raise FirmwareUpdateError(
            f"application did not report firmware 0x{version:X} within {timeout:g} seconds"
            f"{detail}"
        )


def _validate_standard_id(value: int, name: str, allow_zero: bool = True) -> None:
    minimum = 0 if allow_zero else 1
    if not minimum <= value <= 0x7FF:
        raise FirmwareUpdateError(f"{name} must be a standard CAN ID from {minimum} to 0x7FF")


def _print_inspection(path: Path, inspection: PackageInspection) -> None:
    print(f"Image: {path}")
    print(f"Size: {FLASH_SIZE} bytes ({PAGE_COUNT} pages)")
    print(f"Format: {'encrypted updater package' if inspection.encrypted else 'plaintext/raw image'}")
    print(f"Bootloader: {inspection.bootloader.describe()}")
    print(f"Program:    {inspection.program.describe()}")
    print(f"Transport CRC, program:            0x{inspection.program_transport_crc:08X}")
    print(f"Transport CRC, bootloader+program: 0x{inspection.full_transport_crc:08X}")


def command_inspect(args: argparse.Namespace) -> int:
    path = args.image.resolve()
    inspection = inspect_package(path.read_bytes(), require_encrypted=False)
    _print_inspection(path, inspection)
    if not inspection.encrypted:
        print("WARNING: this raw image must not be sent to the sensor bootloader.")
    return 0


def command_update(args: argparse.Namespace) -> int:
    image_path = args.image.resolve()
    image = image_path.read_bytes()
    inspection = inspect_package(image, require_encrypted=True)
    _validate_standard_id(args.request_id, "request ID", allow_zero=False)
    _validate_standard_id(args.update_id, "update ID", allow_zero=False)
    if args.response_id is not None:
        _validate_standard_id(args.response_id, "response ID")
    if args.frame_delay_ms < 0:
        raise FirmwareUpdateError("frame delay cannot be negative")
    if args.frame_delay_ms < 1.0 and not args.unsafe_fast:
        raise FirmwareUpdateError(
            "frame delays below 1 ms can overrun the firmware's eight-frame ring; "
            "use --unsafe-fast only for controlled testing"
        )
    if not args.yes and not args.dry_run:
        raise FirmwareUpdateError(
            "update erases the sensor's external firmware image; inspect the summary, then pass --yes"
        )

    if args.adapter == "peak":
        api = PcanBasic(args.dll)
        adapter_name = "PEAK PCAN-Basic"
        adapter_detail = f"PCAN-Basic API {api.api_version()}"
    else:
        api = KvaserCanlib(args.dll)
        adapter_name = "Kvaser CANlib"
        adapter_detail = "Kvaser CANlib"
    channels = api.list_channels()
    selected = next((item for item in channels if item.number == args.channel), None)
    if selected is None:
        raise FirmwareUpdateError(
            f"{adapter_name} channel {args.channel} does not exist or is not attached"
        )

    print(
        f"Target package 0x{inspection.program.version:X} for hardware "
        f"0x{inspection.program.hardware:08X}; {PAGE_COUNT} pages"
    )
    print(
        f"CAN: {adapter_name}, {selected.name}, {args.bitrate} bit/s, "
        f"request ID 0x{args.request_id:X}"
    )
    print(f"Adapter API: {adapter_detail}")
    print(
        f"Bootloader control ID: 0x{args.update_id:X} (application-entered mode)"
    )
    if args.can_log is not None:
        log_path = args.can_log.resolve()
    else:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = default_can_log_path(image_path, stamp, os.getpid())
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"CAN mode: normal/active; host acceptance filters: none")
    print(f"Complete CAN traffic log: {log_path}")

    with log_path.open("w", encoding="utf-8") as traffic_log, api.open_channel(
        args.channel, args.bitrate, args.sample_point
    ) as raw_can_channel:
        traffic_log.write(
            "# A2C firmware update CAN traffic\n"
            f"# Adapter: {adapter_name}; API: {adapter_detail}\n"
            "# CAN mode: normal/active (ACK and transmit enabled)\n"
            "# Host acceptance filters: none configured\n"
            f"# Channel: {selected.name} ({args.channel}); bitrate: {args.bitrate}; "
            f"sample point: {'vendor preset' if args.adapter == 'peak' else args.sample_point}\n"
        )
        can_channel = TrafficLoggingChannel(raw_can_channel, traffic_log)
        updater = FirmwareUpdater(
            can_channel,
            args.request_id,
            args.update_id,
            args.response_id,
            args.frame_delay_ms,
        )

        sensor_type, response_id = updater.query_information(0x06, args.query_timeout)
        hardware, _ = updater.query_information(0x05, args.query_timeout)
        current_version, _ = updater.query_information(0x04, args.query_timeout)
        print(
            f"Sensor on response ID 0x{response_id:X}: type 0x{sensor_type:08X}, "
            f"hardware 0x{hardware:08X}, firmware 0x{current_version:X}"
        )
        if sensor_type != inspection.program.sensor_type:
            raise FirmwareUpdateError("connected sensor type does not match the package")
        if hardware != inspection.program.hardware:
            raise FirmwareUpdateError("connected sensor hardware does not match the package")
        if inspection.program.version <= current_version and not args.allow_same_or_older:
            raise FirmwareUpdateError(
                f"package version 0x{inspection.program.version:X} is not newer than "
                f"sensor version 0x{current_version:X}; use --allow-same-or-older only if intentional"
            )

        if args.dry_run:
            print("Dry run complete; no bootloader command or erase was sent")
            return 0

        total_frames, sensor_frames, observed_seconds = updater.observe_traffic()
        sensor_rate = sensor_frames / observed_seconds
        estimated_capacity = args.bitrate / ESTIMATED_CLASSIC_CAN_BITS_PER_FRAME
        estimated_utilization = sensor_rate / estimated_capacity
        print(
            f"Observed {sensor_frames} sensor frames ({sensor_rate:.0f} frame/s) "
            f"during a {observed_seconds:.2f} s pre-update sample"
        )
        if estimated_utilization >= HIGH_TRAFFIC_UTILIZATION:
            print(
                "WARNING: sensor traffic is using an estimated "
                f"{estimated_utilization * 100:.0f}% of nominal CAN capacity; "
                "temporarily silencing periodic output"
            )
        elif total_frames > sensor_frames:
            print(
                f"Observed {total_frames - sensor_frames} additional frame(s) from other CAN IDs"
            )
        drained = updater.disable_periodic_tasks()
        print(
            "Periodic application slots disabled in RAM for the update; saved settings "
            f"are unchanged ({drained} queued frame(s) drained)"
        )

        welcome_id = updater.enter(args.enter_timeout)
        print(f"Bootloader WELCOME received on CAN ID 0x{welcome_id:X}")
        updater.erase(args.erase_timeout)
        print("External firmware area erased")

        start = time.monotonic()
        for page_number in range(PAGE_COUNT):
            offset = page_number * PAGE_SIZE
            updater.send_page(
                page_number,
                image[offset : offset + PAGE_SIZE],
                args.page_timeout,
            )
            elapsed = max(time.monotonic() - start, 0.001)
            completed = page_number + 1
            rate = completed / elapsed
            remaining = (PAGE_COUNT - completed) / rate if rate else 0
            sys.stdout.write(
                f"\rProgramming {completed:3d}/{PAGE_COUNT} pages "
                f"({completed * 100 / PAGE_COUNT:5.1f}%), {rate:4.1f} page/s, "
                f"ETA {remaining:5.1f} s"
            )
            sys.stdout.flush()
        print()

        scope = updater.verify_transport_crc(inspection, args.crc_timeout)
        print(f"External-flash transport CRC verified ({scope})")
        updater.exit()
        print("Reset requested; waiting for the application")
        try:
            response_id = updater.wait_for_firmware(
                inspection.program.version, args.reboot_timeout
            )
            installed_crc, crc_response_id = updater.query_information_with_retry(
                0x07, args.query_timeout
            )
        except (FirmwareUpdateError, KvaserError, PcanError) as exc:
            raise FirmwareVerificationInconclusive(
                "programming and external-flash CRC verification succeeded, but final "
                f"application verification was inconclusive: {exc}. Power-cycle the "
                "sensor and run Check Sensor before attempting another update"
            ) from exc
        if crc_response_id != response_id:
            raise FirmwareUpdateError(
                f"application CRC response moved from CAN ID 0x{response_id:X} "
                f"to 0x{crc_response_id:X}"
            )
        if installed_crc != inspection.program.crc:
            raise FirmwareUpdateError(
                f"firmware 0x{inspection.program.version:X} responded, but installed "
                f"CRC 0x{installed_crc:08X} does not match package CRC "
                f"0x{inspection.program.crc:08X}; the pre-application loader may "
                "have declined a same-version or older image"
            )
        print(
            f"Update complete: firmware 0x{inspection.program.version:X}, CRC "
            f"0x{installed_crc:08X}, responded on CAN ID 0x{response_id:X}"
        )
        can_channel.flush_log()
    return 0


def _add_common_can_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--adapter",
        choices=("kvaser", "peak"),
        default="kvaser",
        help="CAN adapter backend (default: kvaser)",
    )
    parser.add_argument(
        "--channel",
        type=parse_integer,
        default=0,
        help="Kvaser channel number or PEAK PCAN handle, e.g. 0x51",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        choices=SUPPORTED_BITRATES,
        default=250_000,
        help="classic CAN bitrate (default: 250000)",
    )
    parser.add_argument(
        "--sample-point",
        choices=tuple(SAMPLE_POINT_SEGMENTS),
        default="87.5",
        help="CAN sample point in percent (default: 87.5)",
    )
    parser.add_argument(
        "--dll", help="optional path to canlib32.dll or PCANBasic.dll"
    )
    parser.add_argument(
        "--request-id",
        type=parse_integer,
        default=0x3E8,
        help="ID accepted by the running sensor (default: 0x3E8)",
    )
    parser.add_argument(
        "--update-id",
        type=parse_integer,
        default=0x3E8,
        help=(
            "temporary ID selected for application-entered bootloader traffic "
            "(default: 0x3E8; a different ID must already pass the sensor's hardware filter)"
        ),
    )
    parser.add_argument(
        "--response-id",
        type=parse_integer,
        help="optional expected sensor transmit ID; normally discovered automatically",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect and update A2C-IMU V2 firmware over Kvaser or PEAK CAN."
    )
    commands = parser.add_subparsers(dest="command", required=True)

    inspect_parser = commands.add_parser("inspect", help="inspect an image without CAN access")
    inspect_parser.add_argument("image", type=Path)
    inspect_parser.set_defaults(handler=command_inspect)

    update_parser = commands.add_parser("update", help="erase, program, verify, and reboot a sensor")
    update_parser.add_argument("image", type=Path, help="encrypted 204800-byte .binenc package")
    _add_common_can_arguments(update_parser)
    update_parser.add_argument(
        "--allow-same-or-older",
        action="store_true",
        help=(
            "bypass the host version guard; the sensor's pre-application loader "
            "may still decline the image, which installed-CRC verification reports"
        ),
    )
    update_parser.add_argument(
        "--frame-delay-ms",
        type=float,
        default=5.0,
        help="delay between each 8-byte page frame (default: 5 ms)",
    )
    update_parser.add_argument(
        "--unsafe-fast",
        action="store_true",
        help="allow a frame delay below the conservative 1 ms minimum",
    )
    update_parser.add_argument("--query-timeout", type=float, default=1.5)
    update_parser.add_argument("--enter-timeout", type=float, default=3.0)
    update_parser.add_argument("--erase-timeout", type=float, default=120.0)
    update_parser.add_argument("--page-timeout", type=float, default=12.0)
    update_parser.add_argument("--crc-timeout", type=float, default=10.0)
    update_parser.add_argument("--reboot-timeout", type=float, default=60.0)
    update_parser.add_argument(
        "--can-log",
        type=Path,
        help="complete unfiltered TX/RX traffic log (default: local A2C application-data directory)",
    )
    update_parser.add_argument(
        "--yes", action="store_true", help="confirm external firmware image erase and update"
    )
    update_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate the package and live preflight only; send no bootloader or erase command",
    )
    update_parser.set_defaults(handler=command_update)

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.handler(args)
    except FirmwareVerificationInconclusive as exc:
        print(f"VERIFICATION INCONCLUSIVE: {exc}", file=sys.stderr)
        return VERIFICATION_INCONCLUSIVE_EXIT_CODE
    except (FirmwareUpdateError, KvaserError, PcanError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nUpdate interrupted; the sensor may remain in bootloader mode.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
