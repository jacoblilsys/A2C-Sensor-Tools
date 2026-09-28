#!/usr/bin/env python3
# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Minimal PEAK PCAN-Basic binding used by the firmware updater.

PCANBasic.dll is supplied by PEAK's Windows driver/API installation and is
not distributed with A2C Sensor Tools.
"""

from __future__ import annotations

import ctypes as ct
import os
import time
from typing import Optional

try:
    from .can_sensor_monitor import (
        CAN_MSG_ERROR_FRAME,
        CAN_MSG_EXT,
        CAN_MSG_RTR,
        CAN_MSG_STD,
        CanFrame,
        ChannelInfo,
    )
except ImportError:  # Direct execution from the package directory.
    from can_sensor_monitor import (  # type: ignore
        CAN_MSG_ERROR_FRAME,
        CAN_MSG_EXT,
        CAN_MSG_RTR,
        CAN_MSG_STD,
        CanFrame,
        ChannelInfo,
    )


PCAN_NONEBUS = 0x00
PCAN_USBBUS_HANDLES = (
    0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x509, 0x50A, 0x50B, 0x50C, 0x50D, 0x50E, 0x50F, 0x510,
)

PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020

PCAN_API_VERSION = 0x05
PCAN_CHANNEL_CONDITION = 0x0D
PCAN_CHANNEL_UNAVAILABLE = 0x00
PCAN_CHANNEL_AVAILABLE = 0x01
PCAN_CHANNEL_OCCUPIED = 0x02
PCAN_CHANNEL_PCANVIEW = PCAN_CHANNEL_AVAILABLE | PCAN_CHANNEL_OCCUPIED

PCAN_MESSAGE_STANDARD = 0x00
PCAN_MESSAGE_RTR = 0x01
PCAN_MESSAGE_EXTENDED = 0x02
PCAN_MESSAGE_ERRFRAME = 0x40
PCAN_MESSAGE_STATUS = 0x80

PCAN_BAUDRATES = {
    50_000: 0x472F,
    100_000: 0x432F,
    125_000: 0x031C,
    250_000: 0x011C,
    500_000: 0x001C,
    1_000_000: 0x0014,
}


class PcanError(RuntimeError):
    """A native PCAN-Basic operation failed."""


class TPCANMsg(ct.Structure):
    _fields_ = (
        ("ID", ct.c_uint32),
        ("MSGTYPE", ct.c_ubyte),
        ("LEN", ct.c_ubyte),
        ("DATA", ct.c_ubyte * 8),
    )


class TPCANTimestamp(ct.Structure):
    _fields_ = (
        ("millis", ct.c_uint32),
        ("millis_overflow", ct.c_uint16),
        ("micros", ct.c_uint16),
    )


class PcanBasic:
    """Small ctypes wrapper for PEAK PCAN-USB classic-CAN channels."""

    def __init__(self, dll_path: Optional[str] = None, *, _dll=None) -> None:
        if _dll is not None:
            self._dll = _dll
        else:
            if os.name != "nt":
                raise PcanError("PEAK PCANBasic.dll is available only on Windows")
            try:
                self._dll = ct.WinDLL(dll_path or "PCANBasic.dll")
            except OSError as exc:
                raise PcanError(
                    "Unable to load PCANBasic.dll. Install the PEAK PCAN Windows "
                    "device driver/PCAN-Basic API and use a matching Python architecture."
                ) from exc
        self._bind_functions()

    def _bind_functions(self) -> None:
        self._dll.CAN_Initialize.argtypes = (
            ct.c_uint16, ct.c_uint16, ct.c_ubyte, ct.c_uint32, ct.c_uint16,
        )
        self._dll.CAN_Initialize.restype = ct.c_uint32
        self._dll.CAN_Uninitialize.argtypes = (ct.c_uint16,)
        self._dll.CAN_Uninitialize.restype = ct.c_uint32
        self._dll.CAN_Read.argtypes = (
            ct.c_uint16, ct.POINTER(TPCANMsg), ct.POINTER(TPCANTimestamp),
        )
        self._dll.CAN_Read.restype = ct.c_uint32
        self._dll.CAN_Write.argtypes = (ct.c_uint16, ct.POINTER(TPCANMsg))
        self._dll.CAN_Write.restype = ct.c_uint32
        self._dll.CAN_GetValue.argtypes = (
            ct.c_uint16, ct.c_ubyte, ct.c_void_p, ct.c_uint32,
        )
        self._dll.CAN_GetValue.restype = ct.c_uint32
        self._dll.CAN_GetErrorText.argtypes = (
            ct.c_uint32, ct.c_uint16, ct.c_char_p,
        )
        self._dll.CAN_GetErrorText.restype = ct.c_uint32

    def error_text(self, status: int) -> str:
        buffer = ct.create_string_buffer(256)
        result = self._dll.CAN_GetErrorText(status, 0x09, buffer)
        if result == PCAN_ERROR_OK:
            return buffer.value.decode("utf-8", errors="replace")
        return "unknown PCAN-Basic error"

    def check(self, status: int, operation: str) -> None:
        if status != PCAN_ERROR_OK:
            raise PcanError(
                f"{operation} failed with status 0x{status:X}: {self.error_text(status)}"
            )

    def api_version(self) -> str:
        buffer = ct.create_string_buffer(64)
        status = self._dll.CAN_GetValue(
            PCAN_NONEBUS, PCAN_API_VERSION, buffer, len(buffer)
        )
        self.check(status, "CAN_GetValue(PCAN_API_VERSION)")
        return buffer.value.decode("ascii", errors="replace")

    def _channel_condition(self, handle: int) -> Optional[int]:
        condition = ct.c_uint32()
        status = self._dll.CAN_GetValue(
            handle,
            PCAN_CHANNEL_CONDITION,
            ct.byref(condition),
            ct.sizeof(condition),
        )
        if status != PCAN_ERROR_OK:
            return None
        return condition.value

    def list_channels(self) -> list[ChannelInfo]:
        channels: list[ChannelInfo] = []
        for index, handle in enumerate(PCAN_USBBUS_HANDLES, start=1):
            condition = self._channel_condition(handle)
            if condition in (None, PCAN_CHANNEL_UNAVAILABLE):
                continue
            if condition == PCAN_CHANNEL_AVAILABLE:
                state = "available"
            elif condition == PCAN_CHANNEL_PCANVIEW:
                state = "shared with PCAN-View"
            elif condition & PCAN_CHANNEL_OCCUPIED:
                state = "occupied"
            else:
                state = f"condition 0x{condition:X}"
            channels.append(
                ChannelInfo(
                    number=handle,
                    name=f"PCAN_USBBUS{index}",
                    description=f"PEAK PCAN-USB, {state}, handle 0x{handle:X}",
                )
            )
        return channels

    def open_channel(
        self, channel: int, bitrate: int, sample_point: str = "87.5"
    ) -> "PcanChannel":
        del sample_point  # PCAN-Basic presets define their own bit timing.
        try:
            pcan_bitrate = PCAN_BAUDRATES[bitrate]
        except KeyError as exc:
            raise ValueError(f"PEAK PCAN-Basic does not support {bitrate} bit/s here") from exc
        if channel not in PCAN_USBBUS_HANDLES:
            raise ValueError(f"0x{channel:X} is not a PCAN-USB channel handle")
        self.check(
            self._dll.CAN_Initialize(channel, pcan_bitrate, 0, 0, 0),
            f"CAN_Initialize(PCAN handle 0x{channel:X})",
        )
        return PcanChannel(self, channel)


class PcanChannel:
    def __init__(self, api: PcanBasic, handle: int) -> None:
        self._api = api
        self._handle = handle
        self._closed = False

    def __enter__(self) -> "PcanChannel":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._api._dll.CAN_Uninitialize(self._handle)
        self._closed = True

    def write(self, can_id: int, data: bytes, extended: bool = False) -> None:
        if self._closed:
            raise PcanError("PCAN channel is closed")
        maximum_id = 0x1FFFFFFF if extended else 0x7FF
        if not 0 <= can_id <= maximum_id:
            raise ValueError(f"CAN ID 0x{can_id:X} is outside the selected frame format")
        if len(data) > 8:
            raise ValueError("Classic CAN payload cannot exceed 8 bytes")

        message = TPCANMsg()
        message.ID = can_id
        message.MSGTYPE = PCAN_MESSAGE_EXTENDED if extended else PCAN_MESSAGE_STANDARD
        message.LEN = len(data)
        for index, value in enumerate(data):
            message.DATA[index] = value
        self._api.check(
            self._api._dll.CAN_Write(self._handle, ct.byref(message)), "CAN_Write"
        )

    def read(self, timeout_ms: int = 100) -> Optional[CanFrame]:
        if self._closed:
            raise PcanError("PCAN channel is closed")
        deadline = time.monotonic() + max(0, timeout_ms) / 1000.0
        while True:
            message = TPCANMsg()
            timestamp = TPCANTimestamp()
            status = self._api._dll.CAN_Read(
                self._handle, ct.byref(message), ct.byref(timestamp)
            )
            if status == PCAN_ERROR_OK:
                flags = CAN_MSG_EXT if message.MSGTYPE & PCAN_MESSAGE_EXTENDED else CAN_MSG_STD
                if message.MSGTYPE & PCAN_MESSAGE_RTR:
                    flags |= CAN_MSG_RTR
                if message.MSGTYPE & (PCAN_MESSAGE_ERRFRAME | PCAN_MESSAGE_STATUS):
                    flags |= CAN_MSG_ERROR_FRAME
                timestamp_ms = timestamp.millis + (timestamp.millis_overflow << 32)
                return CanFrame(
                    can_id=message.ID,
                    data=bytes(message.DATA[: min(message.LEN, 8)]),
                    flags=flags,
                    timestamp_ms=timestamp_ms,
                )
            if status != PCAN_ERROR_QRCVEMPTY:
                self._api.check(status, "CAN_Read")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            time.sleep(min(0.001, remaining))
