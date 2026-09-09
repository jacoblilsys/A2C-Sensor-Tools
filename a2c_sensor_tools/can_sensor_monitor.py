#!/usr/bin/env python3
# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Small Kvaser CAN monitor for the A2C IMU sensor.

The program uses Kvaser's installed canlib32.dll directly through ctypes, so
no third-party Python package is required.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import ctypes as ct
from dataclasses import dataclass, field
import os
import shutil
import struct
import sys
import time
from typing import Optional, Sequence


CAN_OK = 0
CAN_ERR_NOMSG = -2
CAN_ERR_TIMEOUT = -7

CAN_DRIVER_NORMAL = 4
CAN_MSG_RTR = 0x0001
CAN_MSG_STD = 0x0002
CAN_MSG_EXT = 0x0004
CAN_MSG_ERROR_FRAME = 0x0020

CAN_CHANNELDATA_CHANNEL_NAME = 13
CAN_CHANNELDATA_DEVDESCR_ASCII = 26

SUPPORTED_BITRATES = (50_000, 100_000, 125_000, 250_000, 500_000, 1_000_000)
SAMPLE_POINT_SEGMENTS = {
    "87.5": (13, 2),
    "75": (14, 5),
}

CMD_SET_SYSTEM_MODE = 0x40
CMD_SET_PERIODIC_TASK = 0x52
CMD_SET_YAW_REFERENCE = 0x5B
CMD_SET_GYRO_CALIBRATION = 0x5D
CMD_CALIBRATE_USING_GRAVITY = 0x20
CMD_GET_CALIBRATION_INFORMATION = 0x18
CMD_GET_SYSTEM_MODE = 0xC0
CMD_GET_PERIODIC_TASK = 0xC1
CMD_GET_VIBRATION_CONFIGURATION = 0xC5
CMD_GET_YAW_REFERENCE = 0xC6
CMD_GET_GYRO_CALIBRATION = 0xC8
CMD_GET_SAMPLING_TIME = 0xE4
CMD_GET_IMU_SETTINGS = 0x62
CMD_GET_SENSOR_INFORMATION = 0xEF
CMD_SEND_ACCELERATION = 0x0A
CMD_SEND_INCLINATION = 0x0B
CMD_SEND_RMS = 0x10

MODE_NAMES = {
    0: "standby",
    1: "normal (sensor fusion)",
    2: "vibration XYZ",
    3: "vibration XY vector",
    4: "vibration YZ vector",
    5: "vibration XZ vector",
    6: "vibration XYZ vector",
}

GYRO_BIAS_MODE_NAMES = {
    0: "legacy continuous MOBILE",
    1: "stationary automatic",
    2: "fixed hardware offset",
}

GYRO_GATE_STATE_NAMES = {
    0: "disabled",
    1: "moving",
    2: "qualifying stationary",
    3: "stationary qualified",
    4: "learning candidate",
    5: "bias latched",
}

GYRO_REJECTION_REASON_NAMES = (
    (0x0001, "accel norm"),
    (0x0002, "accel noise"),
    (0x0004, "gravity direction"),
    (0x0008, "gyro mean"),
    (0x0010, "gyro noise"),
    (0x0020, "temperature span"),
    (0x0040, "bias step"),
    (0x0080, "candidate timeout"),
    (0x0100, "FIFO/SPI data quality"),
)

GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS = tuple(range(0x0B, 0x12))
GYRO_RUNTIME_DIAGNOSTIC_NAMES = {
    0x0B: "summary",
    0x0C: "qualified time",
    0x0D: "accepted bias",
    0x0E: "last bias step",
    0x0F: "counters",
    0x10: "accel metrics",
    0x11: "gyro metrics",
}

ACCEL_FSR_BY_INDEX = {
    0: 2,
    1: 4,
    2: 8,
    3: 16,
}

RMS_VECTOR_NAME_BY_ID_OFFSET = {
    31: "XY",
    32: "YZ",
    33: "XZ",
    34: "XYZ",
}

SENSOR_INFORMATION_NAMES = {
    0x04: "firmware version",
    0x05: "hardware revision",
    0x06: "sensor type",
    0x14: "serial number",
}


def gyro_rejection_reasons(mask: int) -> tuple[str, ...]:
    """Return readable names for runtime stationary-gate rejection bits."""
    reasons = tuple(
        name for bit, name in GYRO_REJECTION_REASON_NAMES if mask & bit
    )
    unknown = mask & ~sum(bit for bit, _name in GYRO_REJECTION_REASON_NAMES)
    if unknown:
        reasons += (f"unknown 0x{unknown:04X}",)
    return reasons


def decode_gyro_calibration_runtime(
    frame: "CanFrame",
) -> tuple[int, dict[str, object]]:
    """Decode one query-only 0x18/0x0B..0x11 gyro runtime reply."""
    data = frame.data
    if (
        len(data) != 8
        or data[0] != CMD_GET_CALIBRATION_INFORMATION
        or data[1] not in GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS
    ):
        raise ValueError("not a gyro runtime calibration diagnostic response")

    subcommand = data[1]
    if subcommand == 0x0B:
        rejection_mask = int.from_bytes(data[6:8], "big")
        values: dict[str, object] = {
            "version": data[2],
            "mode": data[3],
            "gate_state": data[4],
            "vendor_accuracy": data[5],
            "rejection_mask": rejection_mask,
            "rejection_reasons": gyro_rejection_reasons(rejection_mask),
        }
    elif subcommand == 0x0C:
        values = {"qualified_ms": int.from_bytes(data[2:6], "big")}
    elif subcommand == 0x0D:
        values = {"accepted_bias_mdps": struct.unpack(">hhh", data[2:8])}
    elif subcommand == 0x0E:
        values = {"last_bias_step_mdps": struct.unpack(">hhh", data[2:8])}
    elif subcommand == 0x0F:
        accepted, rejected, rearmed = struct.unpack(">HHH", data[2:8])
        values = {
            "accepted_count": accepted,
            "rejected_count": rejected,
            "rearm_count": rearmed,
        }
    elif subcommand == 0x10:
        accel_norm, accel_noise, gravity_drift = struct.unpack(">HHH", data[2:8])
        values = {
            "accel_norm_mg": accel_norm,
            "accel_noise_mg": accel_noise,
            "gravity_drift_mdeg": gravity_drift,
        }
    else:
        gyro_norm, gyro_noise, temperature_span = struct.unpack(">HHH", data[2:8])
        values = {
            "gyro_norm_mdps": gyro_norm,
            "gyro_noise_mdps": gyro_noise,
            "temperature_span_centi_c": temperature_span,
        }
    return subcommand, values


class KvaserError(RuntimeError):
    """A native Kvaser CANlib operation failed."""


@dataclass(frozen=True)
class ChannelInfo:
    number: int
    name: str
    description: str


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    flags: int = CAN_MSG_STD
    timestamp_ms: int = 0

    @property
    def is_extended(self) -> bool:
        return bool(self.flags & CAN_MSG_EXT)

    @property
    def is_error(self) -> bool:
        return bool(self.flags & CAN_MSG_ERROR_FRAME)


class KvaserCanlib:
    """Minimal binding for the native calls used by this monitor."""

    def __init__(self, dll_path: Optional[str] = None) -> None:
        if os.name != "nt":
            raise KvaserError("Kvaser canlib32.dll is available only on Windows")

        try:
            self._dll = ct.WinDLL(dll_path or "canlib32.dll")
        except OSError as exc:
            raise KvaserError(
                "Unable to load canlib32.dll. Install the Kvaser Windows driver "
                "and use a Python architecture matching the installed DLL."
            ) from exc

        self._bind_functions()
        self._dll.canInitializeLibrary()

    def _bind_functions(self) -> None:
        self._dll.canInitializeLibrary.argtypes = []
        self._dll.canInitializeLibrary.restype = None

        self._dll.canGetErrorText.argtypes = [ct.c_int, ct.c_char_p, ct.c_uint]
        self._dll.canGetErrorText.restype = ct.c_int

        self._dll.canGetNumberOfChannels.argtypes = [ct.POINTER(ct.c_int)]
        self._dll.canGetNumberOfChannels.restype = ct.c_int

        self._dll.canGetChannelData.argtypes = [
            ct.c_int,
            ct.c_int,
            ct.c_void_p,
            ct.c_size_t,
        ]
        self._dll.canGetChannelData.restype = ct.c_int

        self._dll.canOpenChannel.argtypes = [ct.c_int, ct.c_int]
        self._dll.canOpenChannel.restype = ct.c_int

        self._dll.canSetBusParams.argtypes = [
            ct.c_int,
            ct.c_long,
            ct.c_uint,
            ct.c_uint,
            ct.c_uint,
            ct.c_uint,
            ct.c_uint,
        ]
        self._dll.canSetBusParams.restype = ct.c_int

        self._dll.canSetBusOutputControl.argtypes = [ct.c_int, ct.c_uint]
        self._dll.canSetBusOutputControl.restype = ct.c_int
        self._dll.canBusOn.argtypes = [ct.c_int]
        self._dll.canBusOn.restype = ct.c_int
        self._dll.canBusOff.argtypes = [ct.c_int]
        self._dll.canBusOff.restype = ct.c_int
        self._dll.canClose.argtypes = [ct.c_int]
        self._dll.canClose.restype = ct.c_int

        self._dll.canWrite.argtypes = [
            ct.c_int,
            ct.c_long,
            ct.c_void_p,
            ct.c_uint,
            ct.c_uint,
        ]
        self._dll.canWrite.restype = ct.c_int

        self._dll.canReadWait.argtypes = [
            ct.c_int,
            ct.POINTER(ct.c_long),
            ct.c_void_p,
            ct.POINTER(ct.c_uint),
            ct.POINTER(ct.c_uint),
            ct.POINTER(ct.c_ulong),
            ct.c_ulong,
        ]
        self._dll.canReadWait.restype = ct.c_int

    def error_text(self, status: int) -> str:
        buffer = ct.create_string_buffer(256)
        result = self._dll.canGetErrorText(status, buffer, len(buffer))
        if result == CAN_OK:
            return buffer.value.decode("ascii", errors="replace")
        return "unknown CANlib error"

    def check(self, status: int, operation: str) -> None:
        if status < CAN_OK:
            raise KvaserError(
                f"{operation} failed with status {status}: {self.error_text(status)}"
            )

    def _channel_text(self, channel: int, item: int) -> str:
        buffer = ct.create_string_buffer(256)
        self.check(
            self._dll.canGetChannelData(channel, item, buffer, len(buffer)),
            "canGetChannelData",
        )
        return buffer.value.decode("utf-8", errors="replace")

    def list_channels(self) -> list[ChannelInfo]:
        count = ct.c_int()
        self.check(self._dll.canGetNumberOfChannels(ct.byref(count)), "canGetNumberOfChannels")
        return [
            ChannelInfo(
                number=channel,
                name=self._channel_text(channel, CAN_CHANNELDATA_CHANNEL_NAME),
                description=self._channel_text(channel, CAN_CHANNELDATA_DEVDESCR_ASCII),
            )
            for channel in range(count.value)
        ]

    def open_channel(
        self, channel: int, bitrate: int, sample_point: str
    ) -> "KvaserChannel":
        handle = self._dll.canOpenChannel(channel, 0)
        if handle < CAN_OK:
            raise KvaserError(
                f"canOpenChannel({channel}) failed with status {handle}: "
                f"{self.error_text(handle)}"
            )

        result = KvaserChannel(self, handle)
        try:
            tseg1, tseg2 = SAMPLE_POINT_SEGMENTS[sample_point]
            self.check(
                self._dll.canSetBusParams(handle, bitrate, tseg1, tseg2, 1, 1, 0),
                "canSetBusParams",
            )
            self.check(
                self._dll.canSetBusOutputControl(handle, CAN_DRIVER_NORMAL),
                "canSetBusOutputControl",
            )
            self.check(self._dll.canBusOn(handle), "canBusOn")
            result._on_bus = True
            return result
        except Exception:
            result.close()
            raise


class KvaserChannel:
    def __init__(self, api: KvaserCanlib, handle: int) -> None:
        self._api = api
        self._handle = handle
        self._on_bus = False
        self._closed = False

    def __enter__(self) -> "KvaserChannel":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        if self._on_bus:
            self._api._dll.canBusOff(self._handle)
            self._on_bus = False
        self._api._dll.canClose(self._handle)
        self._closed = True

    def bus_off(self) -> None:
        """Disconnect this interface from the CAN bus without closing it."""
        if self._closed:
            raise KvaserError("CAN channel is closed")
        if self._on_bus:
            self._api.check(self._api._dll.canBusOff(self._handle), "canBusOff")
            self._on_bus = False

    def bus_on(self) -> None:
        """Reconnect an interface previously taken off the CAN bus."""
        if self._closed:
            raise KvaserError("CAN channel is closed")
        if not self._on_bus:
            self._api.check(self._api._dll.canBusOn(self._handle), "canBusOn")
            self._on_bus = True

    def write(self, can_id: int, data: bytes, extended: bool = False) -> None:
        maximum_id = 0x1FFFFFFF if extended else 0x7FF
        if not 0 <= can_id <= maximum_id:
            raise ValueError(f"CAN ID 0x{can_id:X} is outside the selected frame format")
        if len(data) > 8:
            raise ValueError("Classic CAN payload cannot exceed 8 bytes")

        payload = (ct.c_ubyte * len(data)).from_buffer_copy(data)
        flags = CAN_MSG_EXT if extended else CAN_MSG_STD
        self._api.check(
            self._api._dll.canWrite(
                self._handle, can_id, payload, len(data), flags
            ),
            "canWrite",
        )

    def read(self, timeout_ms: int = 100) -> Optional[CanFrame]:
        can_id = ct.c_long()
        payload = (ct.c_ubyte * 64)()
        dlc = ct.c_uint()
        flags = ct.c_uint()
        timestamp = ct.c_ulong()

        status = self._api._dll.canReadWait(
            self._handle,
            ct.byref(can_id),
            payload,
            ct.byref(dlc),
            ct.byref(flags),
            ct.byref(timestamp),
            max(0, timeout_ms),
        )
        if status in (CAN_ERR_NOMSG, CAN_ERR_TIMEOUT):
            return None
        self._api.check(status, "canReadWait")
        length = min(dlc.value, len(payload))
        return CanFrame(
            can_id=can_id.value,
            data=bytes(payload[:length]),
            flags=flags.value,
            timestamp_ms=timestamp.value,
        )


class SensorDecoder:
    """Stateful decoder for the sensor's CAN messages."""

    def __init__(
        self, accel_fsr_g: int = 16, sensor_can_id: Optional[int] = None
    ) -> None:
        if accel_fsr_g not in (2, 4, 8, 16):
            raise ValueError("accelerometer full-scale range must be 2, 4, 8, or 16 g")
        self.accel_fsr_g = accel_fsr_g
        self.sensor_can_id = sensor_can_id
        self.system_mode: Optional[int] = None

    @property
    def counts_per_g(self) -> int:
        return 10_000 if self.accel_fsr_g == 2 else 1_000

    def _remember_base_id(self, frame: CanFrame) -> None:
        self.sensor_can_id = frame.can_id

    def _format_xyz(self, label: str, counts: tuple[int, int, int]) -> str:
        values = tuple(value / self.counts_per_g for value in counts)
        return (
            f"{label:<7} X={values[0]:+9.4f} g  Y={values[1]:+9.4f} g  "
            f"Z={values[2]:+9.4f} g"
        )

    def decode(self, frame: CanFrame) -> Optional[str]:
        if frame.is_error:
            return "CAN error frame"
        if frame.flags & CAN_MSG_RTR:
            return f"remote request, DLC={len(frame.data)}"
        if not frame.data:
            return None

        data = frame.data
        command = data[0]

        if command == CMD_SEND_ACCELERATION and len(data) >= 8 and data[1] == 0:
            self._remember_base_id(frame)
            counts = struct.unpack(">hhh", data[2:8])
            return self._format_xyz("AVERAGE", counts)

        if command == CMD_SEND_INCLINATION and len(data) >= 8 and data[1] <= 2:
            counts = struct.unpack(">hhh", data[2:8])
            scale = (1.0, 10.0, 100.0)[data[1]]
            angles = tuple(value / scale for value in counts)
            return (
                f"ANGLES  roll={angles[0]:+8.2f} deg  "
                f"pitch={angles[1]:+8.2f} deg  yaw={angles[2]:+8.2f} deg"
            )

        if command == CMD_SEND_RMS and len(data) == 7:
            counts = struct.unpack(">HHH", data[1:7])
            return self._format_xyz("RMS", counts)

        if command == CMD_SEND_RMS and len(data) == 3:
            counts = struct.unpack(">H", data[1:3])[0]
            vector_name = "vector"
            if self.sensor_can_id is not None:
                offset = frame.can_id - self.sensor_can_id
                vector_name = RMS_VECTOR_NAME_BY_ID_OFFSET.get(offset, vector_name)
            value_g = counts / self.counts_per_g
            return f"RMS {vector_name:<6}={value_g:9.4f} g"

        if command == CMD_GET_SYSTEM_MODE and len(data) >= 2:
            self._remember_base_id(frame)
            # A direct C0 query replies [C0, mode]. The aggregate settings
            # reply uses [C0, 00, mode, 00].
            mode = data[2] if len(data) >= 4 and data[1] == 0 else data[1]
            self.system_mode = mode
            return f"MODE    {mode}: {MODE_NAMES.get(mode, 'unknown')}"

        if command == CMD_GET_PERIODIC_TASK and len(data) >= 7:
            self._remember_base_id(frame)
            task = data[1]
            state = "on" if data[2] else "off"
            interval_ms = int.from_bytes(data[5:7], "big")
            return (
                f"PERIODIC task={task}, {state}, command=0x{data[3]:02X}, "
                f"subcommand=0x{data[4]:02X}, interval={interval_ms} ms"
            )

        if command == CMD_GET_VIBRATION_CONFIGURATION and len(data) >= 8:
            self._remember_base_id(frame)
            flags = data[1]
            highpass_hz = int.from_bytes(data[2:4], "big")
            lowpass_hz = int.from_bytes(data[4:6], "big")
            window_ms = int.from_bytes(data[6:8], "big")
            return (
                "CONFIG  "
                f"LP={'on' if flags & 0x01 else 'off'} ({lowpass_hz} Hz), "
                f"HP={'on' if flags & 0x02 else 'off'} ({highpass_hz} Hz), "
                f"window={window_ms} ms, flags=0x{flags:02X}"
            )

        if command == CMD_GET_SAMPLING_TIME and len(data) >= 8:
            self._remember_base_id(frame)
            mode = data[1]
            rate_hz = int.from_bytes(data[2:6], "big") / 1000.0
            window_ms = int.from_bytes(data[6:8], "big")
            return (
                f"SAMPLE RATE mode={mode} ({MODE_NAMES.get(mode, 'unknown')}), "
                f"measured={rate_hz:.3f} Hz over {window_ms} ms"
            )

        if command == CMD_GET_YAW_REFERENCE and len(data) >= 8:
            self._remember_base_id(frame)
            raw, adjusted, offset = struct.unpack(">hhh", data[2:8])
            return (
                f"YAW REF {'active' if data[1] else 'clear'}, "
                f"raw={raw / 100.0:+.2f} deg, "
                f"output={adjusted / 100.0:+.2f} deg, "
                f"offset={offset / 100.0:+.2f} deg"
            )

        if (
            command == CMD_GET_CALIBRATION_INFORMATION
            and len(data) == 8
            and data[1] in GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS
        ):
            self._remember_base_id(frame)
            subcommand, values = decode_gyro_calibration_runtime(frame)
            if subcommand == 0x0B:
                reasons = values["rejection_reasons"]
                reason_text = ", ".join(reasons) if reasons else "none"
                mode = int(values["mode"])
                gate_state = int(values["gate_state"])
                return (
                    "GYRO GATE "
                    f"v{values['version']}, "
                    f"mode={mode} ({GYRO_BIAS_MODE_NAMES.get(mode, 'unknown')}), "
                    f"state={gate_state} ({GYRO_GATE_STATE_NAMES.get(gate_state, 'unknown')}), "
                    f"vendor accuracy={values['vendor_accuracy']}, rejected by={reason_text}"
                )
            if subcommand == 0x0C:
                return f"GYRO GATE qualified={int(values['qualified_ms'])} ms"
            if subcommand == 0x0D:
                bias = values["accepted_bias_mdps"]
                return (
                    "GYRO BIAS accepted "
                    f"X={bias[0] / 1000.0:+.3f}, Y={bias[1] / 1000.0:+.3f}, "
                    f"Z={bias[2] / 1000.0:+.3f} dps"
                )
            if subcommand == 0x0E:
                step = values["last_bias_step_mdps"]
                return (
                    "GYRO BIAS last step "
                    f"X={step[0] / 1000.0:+.3f}, Y={step[1] / 1000.0:+.3f}, "
                    f"Z={step[2] / 1000.0:+.3f} dps"
                )
            if subcommand == 0x0F:
                return (
                    f"GYRO GATE accepted={values['accepted_count']}, "
                    f"rejected={values['rejected_count']}, rearmed={values['rearm_count']}"
                )
            if subcommand == 0x10:
                gravity_drift = int(values["gravity_drift_mdeg"])
                gravity_drift_text = (
                    "invalid/opposite vector"
                    if gravity_drift == 0xFFFF
                    else f"{gravity_drift / 1000.0:.3f} deg"
                )
                return (
                    f"GYRO GATE accel norm={int(values['accel_norm_mg']) / 1000.0:.3f} g, "
                    f"noise={values['accel_noise_mg']} mg, "
                    f"gravity drift={gravity_drift_text}"
                )
            return (
                f"GYRO GATE gyro norm={int(values['gyro_norm_mdps']) / 1000.0:.3f} dps, "
                f"noise={int(values['gyro_noise_mdps']) / 1000.0:.3f} dps, "
                f"temperature span={int(values['temperature_span_centi_c']) / 100.0:.2f} C"
            )

        if command == CMD_GET_GYRO_CALIBRATION and len(data) >= 8:
            self._remember_base_id(frame)
            if data[1] != 0:
                return f"GYRO CAL unknown configuration subcommand 0x{data[1]:02X}"
            mode = data[2]
            dwell_ms = data[3] * 100
            gyro_threshold_mdps = int.from_bytes(data[4:6], "big")
            accel_tolerance_mg = int.from_bytes(data[6:8], "big")
            return (
                "GYRO CAL "
                f"mode={mode} ({GYRO_BIAS_MODE_NAMES.get(mode, 'unknown')}), "
                f"gyro threshold={gyro_threshold_mdps / 1000.0:.3f} dps, "
                f"accel tolerance=+/-{accel_tolerance_mg} mg, "
                f"dwell={dwell_ms} ms"
            )

        if command == 0x59 and len(data) >= 3 and data[1] == 0x01:
            self._remember_base_id(frame)
            full_scale = ACCEL_FSR_BY_INDEX.get(data[2])
            if full_scale is None:
                return f"ACCEL FSR unknown index {data[2]}"
            self.accel_fsr_g = full_scale
            return (
                f"ACCEL FSR +/-{full_scale} g; output scale "
                f"{self.counts_per_g} counts/g"
            )

        if command == 0x58 and len(data) >= 3:
            self._remember_base_id(frame)
            sensor = "accel" if data[1] == 0x01 else "gyro" if data[1] == 0x02 else "unknown"
            return f"BANDWIDTH {sensor} setting index={data[2]}"

        if command == 0x60 and len(data) >= 4:
            self._remember_base_id(frame)
            return f"AVERAGING standard={data[2]}, RMS={data[3]}"

        if command == CMD_GET_SENSOR_INFORMATION and len(data) >= 6:
            self._remember_base_id(frame)
            information_type = data[1]
            value = int.from_bytes(data[2:6], "big")
            name = SENSOR_INFORMATION_NAMES.get(
                information_type, f"item 0x{information_type:02X}"
            )
            return f"INFO    {name}=0x{value:08X} ({value})"

        if command == 0xFE and len(data) >= 5:
            self._remember_base_id(frame)
            error = int.from_bytes(data[3:5], "big")
            return (
                f"NACK    command=0x{data[1]:02X}, subcommand=0x{data[2]:02X}, "
                f"error=0x{error:04X}"
            )

        if command == 0xAA and len(data) >= 5:
            self._remember_base_id(frame)
            code = int.from_bytes(data[3:5], "big")
            return (
                f"ACK     command=0x{data[1]:02X}, subcommand=0x{data[2]:02X}, "
                f"code=0x{code:04X}"
            )

        return None


def parse_integer(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"{value!r} is not a decimal or 0x-prefixed integer"
        ) from exc


def request_payload(command: int, subcommand: int = 0) -> bytes:
    return bytes((command, subcommand, 0, 0, 0, 0, 0, 0))


def default_startup_queries() -> list[tuple[int, int]]:
    """Return the settings/diagnostic queries sent after opening the bus."""
    queries = [
        (CMD_GET_SYSTEM_MODE, 0),
        (CMD_GET_VIBRATION_CONFIGURATION, 0),
        (CMD_GET_SAMPLING_TIME, 0),
        (CMD_GET_YAW_REFERENCE, 0),
        (CMD_GET_GYRO_CALIBRATION, 0),
        (CMD_GET_IMU_SETTINGS, 0),
        (CMD_GET_SENSOR_INFORMATION, 0x04),
        (CMD_GET_SENSOR_INFORMATION, 0x05),
        (CMD_GET_SENSOR_INFORMATION, 0x14),
    ]
    queries.extend(
        (CMD_GET_CALIBRATION_INFORMATION, subcommand)
        for subcommand in GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS
    )
    queries.extend((CMD_GET_PERIODIC_TASK, task) for task in range(1, 9))
    return queries


def format_raw_frame(frame: CanFrame) -> str:
    frame_type = "EXT" if frame.is_extended else "STD"
    payload = " ".join(f"{byte:02X}" for byte in frame.data)
    return (
        f"RAW     id=0x{frame.can_id:X} {frame_type} DLC={len(frame.data)} "
        f"data=[{payload}] flags=0x{frame.flags:X}"
    )


StreamKey = tuple[int, int, int, int]


@dataclass
class MessageStatistics:
    label: str
    count: int = 0
    latest: str = ""
    arrival_times: deque[float] = field(default_factory=deque)

    def record(self, now: float, latest: str) -> None:
        self.count += 1
        self.latest = latest.replace("\r", " ").replace("\n", " ")
        self.arrival_times.append(now)
        self.rate(now)

    def rate(self, now: float, window_seconds: float = 1.0) -> float:
        cutoff = now - window_seconds
        while self.arrival_times and self.arrival_times[0] < cutoff:
            self.arrival_times.popleft()
        if len(self.arrival_times) < 2:
            return 0.0
        observed_seconds = self.arrival_times[-1] - self.arrival_times[0]
        if observed_seconds <= 0.0:
            return 0.0
        return (len(self.arrival_times) - 1) / observed_seconds


def message_stream(frame: CanFrame) -> tuple[StreamKey, str]:
    """Return a stable dashboard key and human-readable protocol name."""
    dlc = len(frame.data)
    if frame.is_error:
        return (frame.can_id, -2, -1, dlc), "CAN CONTROLLER ERROR"
    if not frame.data:
        return (frame.can_id, -1, -1, dlc), "EMPTY FRAME"

    command = frame.data[0]
    subtype = -1
    label = f"UNKNOWN (0x{command:02X})"

    if command == CMD_SEND_ACCELERATION:
        label = "AVERAGE (0x0A)"
    elif command == CMD_SEND_INCLINATION:
        label = "ANGLES (0x0B)"
    elif command == CMD_SEND_RMS:
        label = "RMS XYZ (0x10)" if dlc == 7 else "RMS VECTOR (0x10)"
    elif command == CMD_GET_SYSTEM_MODE:
        label = "MODE (0xC0)"
    elif command == CMD_GET_PERIODIC_TASK:
        subtype = frame.data[1] if dlc >= 2 else -1
        label = f"PERIODIC T{subtype} (0xC1)"
    elif command == CMD_GET_VIBRATION_CONFIGURATION:
        label = "VIBRATION CONFIG (0xC5)"
    elif command == CMD_GET_YAW_REFERENCE:
        label = "YAW REFERENCE (0xC6)"
    elif command == CMD_GET_GYRO_CALIBRATION:
        label = "GYRO CAL CONFIG (0xC8)"
    elif command == CMD_GET_CALIBRATION_INFORMATION:
        subtype = frame.data[1] if dlc >= 2 else -1
        name = GYRO_RUNTIME_DIAGNOSTIC_NAMES.get(
            subtype, f"ITEM 0x{subtype:02X}"
        )
        prefix = "GYRO CAL" if subtype in GYRO_RUNTIME_DIAGNOSTIC_NAMES else "CALIBRATION"
        label = f"{prefix} {name.upper()} (0x18)"
    elif command == CMD_GET_SAMPLING_TIME:
        label = "SAMPLE RATE (0xE4)"
    elif command == CMD_GET_SENSOR_INFORMATION:
        subtype = frame.data[1] if dlc >= 2 else -1
        name = SENSOR_INFORMATION_NAMES.get(subtype, f"ITEM 0x{subtype:02X}")
        label = f"INFO {name.upper()} (0xEF)"
    elif command == 0x59:
        subtype = frame.data[1] if dlc >= 2 else -1
        sensor = "ACCEL" if subtype == 1 else "GYRO" if subtype == 2 else "UNKNOWN"
        label = f"{sensor} RANGE (0x59)"
    elif command == 0x58:
        subtype = frame.data[1] if dlc >= 2 else -1
        sensor = "ACCEL" if subtype == 1 else "GYRO" if subtype == 2 else "UNKNOWN"
        label = f"{sensor} BANDWIDTH (0x58)"
    elif command == 0x60:
        label = "AVERAGING (0x60)"
    elif command == 0xFE:
        subtype = frame.data[1] if dlc >= 2 else -1
        label = f"NACK/ERROR FOR 0x{subtype:02X} (0xFE)"
    elif command == 0xAA:
        subtype = frame.data[1] if dlc >= 2 else -1
        label = f"ACK FOR 0x{subtype:02X} (0xAA)"

    return (frame.can_id, command, subtype, dlc), label


def update_statistics(
    statistics: dict[StreamKey, MessageStatistics],
    frame: CanFrame,
    decoded: Optional[str],
    now: float,
) -> None:
    key, label = message_stream(frame)
    latest = decoded if decoded is not None else format_raw_frame(frame)
    stream = statistics.setdefault(key, MessageStatistics(label=label))
    stream.record(now, latest)


def _enable_windows_ansi() -> None:
    if os.name != "nt" or not sys.stdout.isatty():
        return
    kernel32 = ct.windll.kernel32
    stdout_handle = kernel32.GetStdHandle(-11)
    mode = ct.c_ulong()
    if kernel32.GetConsoleMode(stdout_handle, ct.byref(mode)):
        kernel32.SetConsoleMode(stdout_handle, mode.value | 0x0004)


def render_dashboard(
    statistics: dict[StreamKey, MessageStatistics],
    channel_info: ChannelInfo,
    bitrate: int,
    start_time: float,
    now: float,
    received_frames: int,
) -> None:
    rates = {key: stream.rate(now) for key, stream in statistics.items()}
    total_rate = sum(rates.values())
    width = max(80, shutil.get_terminal_size(fallback=(160, 30)).columns)

    lines = [
        f"A2C IMU CAN dashboard | {channel_info.name} | {bitrate} bit/s",
        (
            f"Elapsed {now - start_time:8.1f} s | total {received_frames} | "
            f"rate {total_rate:7.1f} msg/s"
        ),
        "",
        f"{'CAN ID':<7} {'MESSAGE TYPE':<28} {'COUNT':>9} {'MSG/S':>8}  LATEST",
        f"{'-' * 7} {'-' * 28} {'-' * 9} {'-' * 8}  {'-' * 20}",
    ]

    for key in sorted(statistics):
        can_id, _command, _subtype, _dlc = key
        stream = statistics[key]
        prefix = (
            f"{f'0x{can_id:X}':<7.7} {stream.label:<28.28} "
            f"{stream.count:>9} {rates[key]:>8.1f}  "
        )
        available = max(0, width - len(prefix))
        lines.append(prefix + stream.latest[:available])

    if not statistics:
        lines.append("Waiting for CAN frames...")
    lines.extend(
        (
            "",
            "0xEF = sensor-information reply.  0xFE = NACK/error reply.",
            "Press Ctrl+C to stop.",
        )
    )
    sys.stdout.write("\x1b[H\x1b[J" + "\n".join(line[:width] for line in lines))
    sys.stdout.flush()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Monitor and poll an A2C IMU using a Kvaser CAN interface."
    )
    parser.add_argument("--list", action="store_true", help="list Kvaser channels and exit")
    parser.add_argument("--channel", type=int, default=0, help="Kvaser channel number (default: 0)")
    parser.add_argument(
        "--bitrate",
        type=int,
        choices=SUPPORTED_BITRATES,
        default=250_000,
        help="classic CAN bitrate in bit/s (default: 250000)",
    )
    parser.add_argument(
        "--sample-point",
        choices=tuple(SAMPLE_POINT_SEGMENTS),
        default="87.5",
        help="CAN sample point in percent (default: 87.5)",
    )
    parser.add_argument(
        "--request-id",
        type=parse_integer,
        default=0x3E8,
        help="CAN ID accepted by the sensor for requests (default: 0x3E8/1000)",
    )
    parser.add_argument(
        "--request-extended",
        action="store_true",
        help="send requests using a 29-bit extended CAN ID",
    )
    parser.add_argument(
        "--sensor-id",
        type=parse_integer,
        help="known sensor transmit base ID, used to label vector RMS frames",
    )
    parser.add_argument(
        "--accel-fsr",
        choices=("auto", "2", "4", "8", "16"),
        default="auto",
        help="accelerometer range in g; auto queries it and initially assumes 16 (default: auto)",
    )
    parser.add_argument(
        "--poll-ms",
        type=int,
        default=250,
        help="request average and RMS at this interval; 0 disables polling (default: 250)",
    )
    parser.add_argument(
        "--no-query",
        action="store_true",
        help="do not issue startup configuration queries",
    )
    parser.add_argument(
        "--set-mode",
        type=int,
        choices=tuple(MODE_NAMES),
        help="set the volatile sensor mode before monitoring (2 is vibration XYZ)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 runs until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="show raw frames in addition to decoded messages",
    )
    parser.add_argument(
        "--display-hz",
        type=float,
        default=10.0,
        help=(
            "maximum displayed rate per measurement stream; 0 is unlimited "
            "(default: 10)"
        ),
    )
    parser.add_argument(
        "--decoded-only",
        action="store_true",
        help="suppress unknown raw CAN frames",
    )
    parser.add_argument(
        "--dashboard",
        action="store_true",
        help="redraw a static per-message statistics table instead of scrolling",
    )
    parser.add_argument(
        "--refresh-ms",
        type=int,
        default=250,
        help="dashboard refresh interval in milliseconds (default: 250)",
    )
    return parser


def print_channels(channels: Sequence[ChannelInfo]) -> None:
    if not channels:
        print("No Kvaser CAN channels found.")
        return
    for channel in channels:
        details = channel.name
        if channel.description and channel.description not in details:
            details += f" - {channel.description}"
        print(f"{channel.number}: {details}")


def run_monitor(args: argparse.Namespace, api: KvaserCanlib) -> int:
    channels = api.list_channels()
    if not channels:
        raise KvaserError("No Kvaser CAN channels found")
    if args.channel < 0 or args.channel >= len(channels):
        raise KvaserError(
            f"Channel {args.channel} does not exist; use --list to see available channels"
        )
    if args.poll_ms < 0:
        raise ValueError("--poll-ms cannot be negative")
    if args.duration < 0:
        raise ValueError("--duration cannot be negative")
    if args.display_hz < 0:
        raise ValueError("--display-hz cannot be negative")
    if args.refresh_ms < 50:
        raise ValueError("--refresh-ms must be at least 50")
    if args.dashboard and args.raw:
        raise ValueError("--dashboard and --raw cannot be used together")
    if args.dashboard and not sys.stdout.isatty():
        raise ValueError("--dashboard requires an interactive terminal")

    initial_fsr = 16 if args.accel_fsr == "auto" else int(args.accel_fsr)
    decoder = SensorDecoder(initial_fsr, args.sensor_id)
    channel_info = channels[args.channel]

    if not args.dashboard:
        print(
            f"Opening channel {args.channel}: {channel_info.name}; "
            f"{args.bitrate} bit/s, {args.sample_point}% sample point"
        )
        print(
            f"Request ID 0x{args.request_id:X} "
            f"({'extended' if args.request_extended else 'standard'}); "
            "press Ctrl+C to stop"
        )
        if args.accel_fsr == "auto":
            print("Acceleration scaling initially assumes +/-16 g until the sensor replies.")

    with api.open_channel(args.channel, args.bitrate, args.sample_point) as channel:
        if args.set_mode is not None:
            channel.write(
                args.request_id,
                request_payload(CMD_SET_SYSTEM_MODE, args.set_mode),
                args.request_extended,
            )
            if not args.dashboard:
                print(f"Requested mode {args.set_mode}: {MODE_NAMES[args.set_mode]}")
            # A vibration-to-fusion transition performs a complete IMU reset
            # and takes roughly 150 ms. Do not flood the small CAN RX queue
            # with startup queries while that command is still executing.
            time.sleep(0.25)

        if not args.no_query:
            for command, subcommand in default_startup_queries():
                channel.write(
                    args.request_id,
                    request_payload(command, subcommand),
                    args.request_extended,
                )
                time.sleep(0.01)

        start_time = time.monotonic()
        deadline = start_time + args.duration if args.duration else None
        poll_interval = args.poll_ms / 1000.0 if args.poll_ms else None
        next_poll = time.monotonic()
        minimum_display_interval = 1.0 / args.display_hz if args.display_hz else 0.0
        last_measurement_display: dict[tuple[int, int, int], float] = {}
        received_frames = 0
        suppressed_measurements = 0
        frame_counts: Counter[tuple[int, int, int]] = Counter()
        statistics: dict[StreamKey, MessageStatistics] = {}
        dashboard_interval = args.refresh_ms / 1000.0
        next_dashboard_refresh = start_time

        if args.dashboard:
            _enable_windows_ansi()
            sys.stdout.write("\x1b[?25l\x1b[2J")
            render_dashboard(
                statistics,
                channel_info,
                args.bitrate,
                start_time,
                start_time,
                received_frames,
            )

        try:
            while deadline is None or time.monotonic() < deadline:
                now = time.monotonic()
                if poll_interval is not None and now >= next_poll:
                    channel.write(
                        args.request_id,
                        request_payload(CMD_SEND_ACCELERATION, 0),
                        args.request_extended,
                    )
                    if decoder.system_mode is not None and decoder.system_mode >= 2:
                        channel.write(
                            args.request_id,
                            request_payload(CMD_SEND_RMS, 0),
                            args.request_extended,
                        )
                    elif decoder.system_mode == 1:
                        channel.write(
                            args.request_id,
                            request_payload(CMD_SEND_INCLINATION, 2),
                            args.request_extended,
                        )
                        channel.write(
                            args.request_id,
                            request_payload(CMD_GET_YAW_REFERENCE, 0),
                            args.request_extended,
                        )
                    channel.write(
                        args.request_id,
                        request_payload(CMD_GET_SAMPLING_TIME, 0),
                        args.request_extended,
                    )
                    while next_poll <= now:
                        next_poll += poll_interval

                timeout_ms = 100
                if deadline is not None:
                    timeout_ms = min(timeout_ms, max(0, int((deadline - now) * 1000)))
                if poll_interval is not None:
                    timeout_ms = min(timeout_ms, max(0, int((next_poll - now) * 1000)))
                if args.dashboard:
                    timeout_ms = min(
                        timeout_ms,
                        max(0, int((next_dashboard_refresh - now) * 1000)),
                    )

                frame = channel.read(timeout_ms)
                if frame is None:
                    render_time = time.monotonic()
                    if args.dashboard and render_time >= next_dashboard_refresh:
                        render_dashboard(
                            statistics,
                            channel_info,
                            args.bitrate,
                            start_time,
                            render_time,
                            received_frames,
                        )
                        while next_dashboard_refresh <= render_time:
                            next_dashboard_refresh += dashboard_interval
                    continue

                received_frames += 1
                command = frame.data[0] if frame.data else -1
                frame_counts[(frame.can_id, command, len(frame.data))] += 1
                timestamp = f"+{frame.timestamp_ms / 1000.0:10.3f}s"
                decoded = decoder.decode(frame)
                receive_time = time.monotonic()
                update_statistics(statistics, frame, decoded, receive_time)

                if args.raw and not args.dashboard:
                    print(f"{timestamp} {format_raw_frame(frame)}")

                display_decoded = decoded is not None
                if (
                    display_decoded
                    and not args.dashboard
                    and not args.raw
                    and minimum_display_interval
                    and frame.data
                    and frame.data[0] in (CMD_SEND_ACCELERATION, CMD_SEND_RMS)
                ):
                    stream = (frame.can_id, frame.data[0], len(frame.data))
                    last_display = last_measurement_display.get(stream)
                    display_time = time.monotonic()
                    if (
                        last_display is not None
                        and display_time - last_display < minimum_display_interval
                    ):
                        display_decoded = False
                        suppressed_measurements += 1
                    else:
                        last_measurement_display[stream] = display_time

                if display_decoded and not args.dashboard:
                    print(f"{timestamp} id=0x{frame.can_id:X} {decoded}")
                elif (
                    decoded is None
                    and not args.dashboard
                    and not args.raw
                    and not args.decoded_only
                ):
                    print(f"{timestamp} {format_raw_frame(frame)}")

                if args.dashboard and receive_time >= next_dashboard_refresh:
                    render_dashboard(
                        statistics,
                        channel_info,
                        args.bitrate,
                        start_time,
                        receive_time,
                        received_frames,
                    )
                    while next_dashboard_refresh <= receive_time:
                        next_dashboard_refresh += dashboard_interval
        finally:
            if args.dashboard:
                final_time = time.monotonic()
                render_dashboard(
                    statistics,
                    channel_info,
                    args.bitrate,
                    start_time,
                    final_time,
                    received_frames,
                )
                sys.stdout.write("\n\x1b[?25h")
                sys.stdout.flush()
            else:
                print(
                    f"Capture summary: received {received_frames} frames; "
                    f"suppressed {suppressed_measurements} repeated measurement displays."
                )
                for (can_id, command, dlc), count in frame_counts.most_common(8):
                    command_text = f"0x{command:02X}" if command >= 0 else "none"
                    print(
                        f"  id=0x{can_id:X}, command={command_text}, "
                        f"DLC={dlc}: {count} frames"
                    )

    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        api = KvaserCanlib()
        if args.list:
            print_channels(api.list_channels())
            return 0
        return run_monitor(args, api)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except (KvaserError, OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
